#include "chain.hpp"
#include "../cblas_abi.hpp"
#include "../model_access.hpp"
#include "../numeric_sincos.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <string>
#include <utility>

namespace {
    // Workspace-width allocation bound: nv is the caller-declared model
    // width; a negative extent is rejected before the first allocation.
    std::size_t checked_geometry_width(mjtSize nv, std::size_t factor) {
        if (nv < 0)
            throw std::invalid_argument(
                "geometry workspace needs nonnegative nv");
        return static_cast<std::size_t>(nv) * factor;
    }

    // mj_jac destination contract: jacp/jacr are (3,nv) row-major blocks
    // and the query point is exactly 3 doubles. The engine ABI call
    // receives .data() only after every extent checks out. Deliberately
    // NOT model_access::point_jacobian_into — this site has its own
    // recorded contract: both Jacobian halves are filled, the body gate
    // is a bare require_id (world body ID 0 is admissible here), and the
    // point extent is checked without the finite scan.
    void body_jacobian(const mjModel *m, const mjData *d,
                       std::span<mjtNum> jacp, std::span<mjtNum> jacr,
                       std::span<const mjtNum> point, int body) {
        model_access::require_id(body, m->nbody, "geometry body");
        if (m->nv < 0)
            throw std::invalid_argument(
                "geometry Jacobian needs a nonnegative model width");
        const std::size_t width = model_access::checked_product(
            static_cast<std::size_t>(model_access::kXYZ),
            static_cast<std::size_t>(m->nv),
            std::numeric_limits<std::size_t>::max());
        if (jacp.size() != width || jacr.size() != width ||
            point.size() != static_cast<std::size_t>(model_access::kXYZ))
            throw std::invalid_argument(
                "body Jacobian destinations must be 3*nv blocks");
        mj_jac(m, d, jacp.data(), jacr.data(), point.data(), body);
    }
} // namespace

namespace drivetrain {
    double dot(std::span<const double> a, std::span<const double> b) {
        if (a.size() != b.size() ||
            a.size() > static_cast<std::size_t>(std::numeric_limits<int>::max()))
            throw std::invalid_argument("dot width");
        return blas::ddot(static_cast<int>(a.size()), a.data(), 1,
                          b.data(), 1);
    }

    double unwrap(double value, double reference) {
        return reference +
               std::atan2(numeric::math_sin(value - reference), numeric::math_cos(value - reference));
    }

    int resolve(const mjModel *m, mjtObj kind, const char *name) {
        const int id = mj_name2id(m, kind, name);
        if (id < 0)
            throw std::invalid_argument("model has no '" + std::string(name) + "'");
        return id;
    }

    ChainGeometry chain_geometry(Vec2 front, Vec2 rear, double rf, double rr, Vec2 up,
                                 std::optional<double> reference) {
        for (double const v: front)
            finite(v, "front centre");
        for (double const v: rear)
            finite(v, "rear centre");
        for (double const v: up)
            finite(v, "chain up");
        positive(rf, "front radius");
        positive(rr, "rear radius");
        finite_optional(reference, "angular unwrap reference");
        if (std::sqrt(dot(up, up)) == 0.)
            throw std::invalid_argument("chain up cannot be zero");
        const double difference = rf - rr;
        const Vec2 vector = {rear[0] - front[0], rear[1] - front[1]};
        const double distance = validation::derived(std::sqrt(dot(vector, vector)), "chain_geometry.distance");
        if (distance <= std::abs(difference))
            throw std::invalid_argument("sprockets have no valid external tangent");
        const Vec2 direction = {vector[0] / distance, vector[1] / distance};
        const Vec2 perpendicular = {direction[1], -direction[0]};
        const double ratio = difference / distance,
                factor = std::sqrt(std::max(0., 1. - ratio * ratio));
        const Vec2 term = {factor * perpendicular[0], factor * perpendicular[1]};
        const Vec2 first = {ratio * direction[0] + term[0], ratio * direction[1] + term[1]};
        const Vec2 second = {
            ratio * direction[0] - term[0],
            ratio * direction[1] - term[1]
        };
        const Vec2 normal = dot(first, up) >= dot(second, up) ? first : second;
        double psi = std::atan2(normal[1], normal[0]);
        if (reference)
            psi = unwrap(psi, *reference);
        return {
            .length = validation::derived(std::sqrt(distance * distance - difference * difference) +
                             difference * psi,
                             "chain geometric coordinate"),
            .psi = psi
        };
    }

    Vec2 chain_center_gradient(Vec2 front, Vec2 rear, double rf, double rr, Vec2 up,
                               std::optional<double> reference) {
        const double psi = chain_geometry(front, rear, rf, rr, up, reference).psi;
        const Vec2 a = {rear[0] - front[0], rear[1] - front[1]};
        const double D = std::sqrt(dot(a, a)), difference = rf - rr,
                root = std::sqrt(D * D - difference * difference);
        const Vec2 perpendicular = {a[1] / D, -a[0] / D},
                normal = {numeric::np_cos(psi), numeric::np_sin(psi)};
        const double sign = dot(normal, perpendicular) >= 0. ? 1. : -1.;
        const double radial = D / root - sign * difference * difference / (D * root);
        const Vec2 result = {
            radial * a[0] / D + difference * (-a[1]) / (D * D),
            radial * a[1] / D + difference * a[0] / (D * D)
        };
        validation::derived_array(result, "chain_center_gradient");
        return result;
    }

    std::pair<double, double> chain_tension(double e, double rate, double k, double c) {
        finite(e, "chain extension");
        finite(rate, "chain extension rate");
        positive(k, "chain stiffness");
        nonnegative(c, "chain damping");
        if (e <= 0.)
            return {0., 0.};
        return {
            validation::derived(std::max(0., validation::derived(k * e + c * rate, "chain raw tension")), "chain tension"),
            validation::derived(.5 * k * e * e, "chain energy")
        };
    }

    GeometryWorkspace::GeometryWorkspace(mjtSize nv)
        : jp_f(checked_geometry_width(nv, 3)), jr_f(jp_f.size()), jp_r(jp_f.size()),
          jr_r(jp_f.size()), jacobian(checked_geometry_width(nv, 1)),
          difference(checked_geometry_width(nv, 2)), coordinates(jacobian.size()),
          jr_frame(jp_f.size()) {
    }

    GeometryWorkspace::GeometryWorkspace(const mjModel* model):GeometryWorkspace(model->nv) {
        spatial=model->nq!=model->nv || mj_name2id(model,mjOBJ_JOINT,"root_y")>=0 ||
            mj_name2id(model,mjOBJ_JOINT,"root_free")>=0;
        if(spatial) {
            centre_sites.assign(static_cast<std::size_t>(model->nbody),-1);
            const auto site_bodies=model_access::readonly_buffer(model->site_bodyid,model->nsite);
            for(const char* name:{"crank","rear_wheel","cassette","drive_shaft","motor_rotor"}) {
                const int body=mj_name2id(model,mjOBJ_BODY,name);
                const auto site_name="spatial_chain_center_"+std::string(name);
                const int site=mj_name2id(model,mjOBJ_SITE,site_name.c_str());
                if(site<0) continue;
                if(body<=0 || site_bodies[static_cast<std::size_t>(site)]!=body)
                    throw std::invalid_argument("geometry.centre_site: wrong sprocket body");
                centre_sites[static_cast<std::size_t>(body)]=site;
            }
        }
    }

    double GeometryWorkspace::angle(const mjModel *m, const mjData *d, int body,
                                    bool accumulated) {
        model_access::require_id(body, m->nbody, "geometry.body");
        if (body == 0)
            throw std::invalid_argument("geometry.body: requires physical body ID");
        // mj_jac writes 3*nv into the jp/jr scratch — a workspace narrower
        // than the model is rejected before the destination is formed.
        if (m->nv < 0 || std::cmp_greater(m->nv, jacobian.size()))
            throw std::invalid_argument(
                "geometry workspace is narrower than the model");
        if(spatial || m->nq!=m->nv) {
            const auto roots=model_access::readonly_buffer(m->body_rootid,m->nbody);
            const auto parents=model_access::readonly_buffer(m->body_parentid,m->nbody);
            const auto joint_start=model_access::readonly_buffer(m->body_jntadr,m->nbody);
            const auto joint_count=model_access::readonly_buffer(m->body_jntnum,m->nbody);
            const auto types=model_access::readonly_buffer(m->jnt_type,m->njnt);
            const auto axes=model_access::readonly_buffer(m->jnt_axis,3*m->njnt);
            const auto addresses=model_access::readonly_buffer(m->jnt_qposadr,m->njnt);
            const auto q=model_access::readonly_buffer(d->qpos,m->nq);
            const auto q0=model_access::readonly_buffer(m->qpos0,m->nq);
            const int root=roots[static_cast<std::size_t>(body)];
            model_access::require_id(root,m->nbody,"geometry.root");
            const auto rotation=model_access::readonly_buffer(d->xmat,9*m->nbody);
            const auto base=rotation.subspan(9*static_cast<std::size_t>(root),9);
            const auto current=rotation.subspan(9*static_cast<std::size_t>(body),9);
            for(const double value:base) finite(value,"geometry.root_rotation");
            for(const double value:current) finite(value,"geometry.sprocket_rotation");
            double axle_alignment=0.;
            for(std::size_t row=0;row<3;++row) axle_alignment+=base[3*row+1]*current[3*row+1];
            if(std::abs(axle_alignment-1.)>1e-9)
                throw std::invalid_argument("transmission.geometry: sprocket axles must remain parallel in the frame");
            double xx=0.,zx=0.;
            for(std::size_t k=0;k<3;++k) {xx+=base[3*k]*current[3*k];zx+=base[3*k+2]*current[3*k];}
            const double raw=std::atan2(-zx,xx);
            if(!accumulated) return raw;
            double reference=0.;
            // Only internal scalar hinge coordinates unwrap sprocket turns.
            // Free/ball components never masquerade as angular scalars.
            for(int owner=body;owner!=root && owner>0;owner=parents[static_cast<std::size_t>(owner)]) {
                const auto at=static_cast<std::size_t>(owner);
                for(int j=joint_start[at];j<joint_start[at]+joint_count[at];++j) {
                    const auto ji=static_cast<std::size_t>(j);
                    if(types[ji]==mjJNT_HINGE && std::abs(axes[3*ji+1]-1.)<=1e-12) {
                        const auto qa=static_cast<std::size_t>(addresses[ji]);
                        reference+=q[qa]-q0[qa];
                    }
                }
            }
            return unwrap(raw,reference);
        }
        const auto rotation =
                model_access::readonly_buffer(d->xmat, 9 * m->nbody)
                .subspan(9 * static_cast<std::size_t>(body), 9);
        // Nonfinite rotation entries are rejected before any tolerance
        // comparison — a NaN would silently pass a `|x| > eps` predicate.
        for (const double value: rotation) finite(value, "geometry.rotation");
        const double raw = std::atan2(-rotation[6], rotation[0]);
        if (!accumulated)
            return raw;
        if (std::abs(rotation[1]) > 1e-9 || std::abs(rotation[4] - 1.) > 1e-9 ||
            std::abs(rotation[7]) > 1e-9)
            throw std::invalid_argument(
                "geometric transmission requires planar sprocket frames");
        body_jacobian(m, d, jp_f, jr_f,
                      model_access::readonly_buffer(d->xpos, 3 * m->nbody)
                      .subspan(3 * static_cast<std::size_t>(body), 3),
                      body);
        const auto q = model_access::readonly_buffer(d->qpos, m->nq),
                q0 = model_access::readonly_buffer(m->qpos0, m->nq);
        if (q.size() < coordinates.size())
            throw std::invalid_argument(
                "geometry.qpos: model coordinates below workspace width");
        for (std::size_t i = 0; i < coordinates.size(); ++i)
            coordinates[i] = q[i] - q0[i];
        return unwrap(raw, dot(std::span<const double>(jr_f).subspan(coordinates.size(),
                                                                     coordinates.size()),
                               coordinates));
    }

    double GeometryWorkspace::evaluate(const mjModel *m, mjData *d,
                                       const GearingConfig &gear, int front, int rear,
                                       int frame, std::optional<Vec2> angles,
                                       std::optional<double> reference, bool accumulated) {
        if(spatial || m->nq!=m->nv)
            return evaluate_spatial(m,d,gear,front,rear,frame,angles,reference,accumulated);
        model_access::require_id(front, m->nbody, "geometry.front");
        model_access::require_id(rear, m->nbody, "geometry.rear");
        model_access::require_id(frame, m->nbody, "geometry.frame");
        if (accumulated)
            for (int const t: model_access::readonly_buffer(m->jnt_type, m->njnt))
                if (t != mjJNT_HINGE && t != mjJNT_SLIDE)
                    throw std::invalid_argument(
                        "transmission geometry needs scalar planar coordinates");
        theta_f = angle(m, d, front, accumulated);
        theta_r = angle(m, d, rear, accumulated);
        if (angles) {
            theta_f = unwrap(theta_f, (*angles)[0]);
            theta_r = unwrap(theta_r, (*angles)[1]);
        }
        if (accumulated)
            reference = std::numbers::pi / 2. - angle(m, d, frame, true);
        const auto pos = model_access::readonly_buffer(d->xpos, 3 * m->nbody),
                rot = model_access::readonly_buffer(d->xmat, 9 * m->nbody);
        const auto cf = pos.subspan(3 * static_cast<std::size_t>(front), 3),
                cr = pos.subspan(3 * static_cast<std::size_t>(rear), 3);
        const Vec2 f = {cf[0], cf[2]}, r = {cr[0], cr[2]},
                up = {
                    rot[9 * static_cast<std::size_t>(frame) + 2],
                    rot[9 * static_cast<std::size_t>(frame) + 8]
                };
        const double rf = gear.chain_pitch_m * gear.front_teeth / (2. * std::numbers::pi),
                rr = gear.chain_pitch_m * gear.rear_teeth / (2. * std::numbers::pi);
        validation::derived(rf, "geometry.front_radius");
        validation::derived(rr, "geometry.rear_radius");
        const auto geometry = chain_geometry(f, r, rf, rr, up, reference);
        psi = geometry.psi;
        const auto gradient = chain_center_gradient(f, r, rf, rr, up, psi);
        body_jacobian(m, d, jp_f, jr_f, cf, front);
        body_jacobian(m, d, jp_r, jr_r, cr, rear);
        const auto n = jacobian.size();
        for (std::size_t i = 0; i < n; ++i) {
            difference[i] = jp_r[i] - jp_f[i];
            difference[n + i] = jp_r[2 * n + i] - jp_f[2 * n + i];
        }
        blas::dgemv(blas::Order::row_major, blas::Transpose::yes, 2,
                    static_cast<int>(n), 1., difference.data(),
                    static_cast<int>(n), gradient.data(), 1, 0.,
                    jacobian.data(), 1);
        for (std::size_t i = 0; i < n; ++i)
            jacobian[i] = jacobian[i] + rf * jr_f[n + i] - rr * jr_r[n + i];
        validation::derived_array(jacobian, "geometry.jacobian");
        return validation::derived(geometry.length + rf * theta_f - rr * theta_r, "geometry.coordinate");
    }

    double GeometryWorkspace::evaluate_spatial(const mjModel* m,mjData* d,
        const GearingConfig& gear,int front,int rear,int frame,std::optional<Vec2> angles,
        std::optional<double> reference,bool accumulated) {
        model_access::require_id(front,m->nbody,"geometry.front");
        model_access::require_id(rear,m->nbody,"geometry.rear");
        model_access::require_id(frame,m->nbody,"geometry.frame");
        const auto n=jacobian.size();
        if(std::cmp_not_equal(n,m->nv)) throw std::invalid_argument("geometry workspace width");
        const auto positions=model_access::readonly_buffer(d->xpos,3*m->nbody);
        const auto rotations=model_access::readonly_buffer(d->xmat,9*m->nbody);
        const auto sites=model_access::readonly_buffer(d->site_xpos,3*m->nsite);
        const auto centre=[&](int body)->std::span<const double> {
            const auto at=static_cast<std::size_t>(body);
            const int site=at<centre_sites.size()?centre_sites[at]:-1;
            return site>=0?sites.subspan(3*static_cast<std::size_t>(site),3):positions.subspan(3*at,3);
        };
        const auto cf=centre(front),cr=centre(rear);
        const auto basis=rotations.subspan(9*static_cast<std::size_t>(frame),9);
        std::array<double,3> delta{},local{},axis{};
        for(std::size_t k=0;k<3;++k) {delta[k]=cr[k]-cf[k];axis[k]=basis[3*k+1];}
        for(std::size_t k=0;k<3;++k) for(std::size_t row=0;row<3;++row) local[k]+=basis[3*row+k]*delta[row];
        if(!std::isfinite(local[1]) || std::abs(local[1])>1e-9)
            throw std::invalid_argument("transmission.geometry: sprocket sites must share the authored chain plane");
        // The sprocket plane is attached to the frame. Global yaw/roll and
        // translation cannot alter the internal chain coordinate.
        const Vec2 f{0.,0.},r{local[0],local[2]},up{0.,1.};
        const double rf=gear.chain_pitch_m*gear.front_teeth/(2.*std::numbers::pi);
        const double rr=gear.chain_pitch_m*gear.rear_teeth/(2.*std::numbers::pi);
        theta_f=angle(m,d,front,accumulated);theta_r=angle(m,d,rear,accumulated);
        if(angles) {theta_f=unwrap(theta_f,(*angles)[0]);theta_r=unwrap(theta_r,(*angles)[1]);}
        const auto geometry=chain_geometry(f,r,rf,rr,up,reference);
        psi=geometry.psi;const auto gradient=chain_center_gradient(f,r,rf,rr,up,psi);
        body_jacobian(m,d,jp_f,jr_f,cf,front);body_jacobian(m,d,jp_r,jr_r,cr,rear);
        mj_jac(m,d,nullptr,jr_frame.data(),positions.subspan(3*static_cast<std::size_t>(frame),3).data(),frame);
        for(std::size_t col=0;col<n;++col) {
            const std::array<double,3> omega{jr_frame[col],jr_frame[n+col],jr_frame[2*n+col]};
            const std::array<double,3> transport{omega[1]*delta[2]-omega[2]*delta[1],
                omega[2]*delta[0]-omega[0]*delta[2],omega[0]*delta[1]-omega[1]*delta[0]};
            double dx=0.,dz=0.,front_spin=0.,rear_spin=0.;
            for(std::size_t row=0;row<3;++row) {
                const auto velocity=jp_r[row*n+col]-jp_f[row*n+col]-transport[row];
                dx+=basis[3*row]*velocity;dz+=basis[3*row+2]*velocity;
                front_spin+=axis[row]*(jr_f[row*n+col]-jr_frame[row*n+col]);
                rear_spin+=axis[row]*(jr_r[row*n+col]-jr_frame[row*n+col]);
            }
            jacobian[col]=gradient[0]*dx+gradient[1]*dz+rf*front_spin-rr*rear_spin;
        }
        validation::derived_array(jacobian,"geometry.jacobian");
        return validation::derived(geometry.length+rf*theta_f-rr*theta_r,"geometry.coordinate");
    }
} // namespace drivetrain
