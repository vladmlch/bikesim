#include "runtime/spatial_tire_factory.hpp"
#include "runtime/setup_values.hpp"
#include "runtime/value_readers.hpp"
#include "io/hash.hpp"
#include "io/json.hpp"
#include "stepper.hpp"
#include "spatial/topology.hpp"
#include "writers/pneumatic_3d_tire.hpp"
#include "model/rider_geometry.hpp"

#include <cmath>
#include <fstream>
#include <numbers>
#include <span>
#include <stdexcept>

namespace runtime {
namespace {
using namespace setupvalue;
namespace cfg=bike::configuration;
namespace pn=bike::pneumatic3d;
std::string digest(const Value& v) {
    const auto text=bike::io::canonical_json(v);
    return bike::io::sha256(std::as_bytes(std::span{text}));
}
Value carcass_wire(const cfg::TyreSpecs& t) {
    const double r=t.casing_width_mm/2000.,R=t.outer_radius_mm/1000.-r;
    return Value{Object{{"pressure_pa",real(t.pressure_pa())},{"k_area_n_per_m3",real(1e6)},
        {"maxwell_k_n_per_m3",real(t.rate_stiffening*1e6)},{"maxwell_tau_s",real(t.rate_relaxation_s)},
        {"rim_clearance_m",real(t.rim_strike_deflection_mm()/1000.)},{"rim_stiffness_n_per_m3",real(1e8)},
        {"initial_volume_m3",real(2.*std::numbers::pi*std::numbers::pi*R*r*r)},
        {"constant_pressure",Value{true}},{"provenance_status",text("provisional")}}};
}
Value brush_wire(const cfg::SpatialTyreCoefficientProfile& p,double radius) {
    return Value{Object{{"kx_n_per_m3",real(p.kx_n_per_m3)},{"ky_n_per_m3",real(p.ky_n_per_m3)},
        {"relaxation_x_m",real(p.relaxation_x_m)},{"relaxation_y_m",real(p.relaxation_y_m)},
        {"mu_x_scale",real(1.)},{"mu_y_scale",real(1.)},
        {"camber_coefficient_per_rad",real(p.camber_coefficient_per_rad)},
        {"pneumatic_trail_m",real(p.pneumatic_trail_m)},{"r_eff_m",real(radius)},
        {"provenance_status",text(p.status)}}};
}
pn::CarcassSpec3d carcass(const Value& v,std::string path) {
    vread::Dict d{.value=&bike::io::as_object(v,path),.path=std::move(path)};
    vread::exact(d,vread::keys("pressure_pa","k_area_n_per_m3","maxwell_k_n_per_m3","maxwell_tau_s",
        "rim_clearance_m","rim_stiffness_n_per_m3","initial_volume_m3","constant_pressure","provenance_status"));
    const auto real_at=[&](const char* k){return vread::finite_real(vread::field(d,k),d.child(k));};
    return {.pressure_pa=real_at("pressure_pa"),.k_area_n_per_m3=real_at("k_area_n_per_m3"),
        .maxwell_k_n_per_m3=real_at("maxwell_k_n_per_m3"),.maxwell_tau_s=real_at("maxwell_tau_s"),
        .rim_clearance_m=real_at("rim_clearance_m"),.rim_stiffness_n_per_m3=real_at("rim_stiffness_n_per_m3"),
        .initial_volume_m3=real_at("initial_volume_m3"),.constant_pressure=vread::boolean(vread::field(d,"constant_pressure"),d.child("constant_pressure")),
        .provenance_status=vread::string(vread::field(d,"provenance_status"),d.child("provenance_status"))};
}
pn::BrushSpec3d brush(const Value& v,std::string path) {
    vread::Dict d{.value=&bike::io::as_object(v,path),.path=std::move(path)};
    vread::exact(d,vread::keys("kx_n_per_m3","ky_n_per_m3","relaxation_x_m","relaxation_y_m",
        "mu_x_scale","mu_y_scale","camber_coefficient_per_rad","pneumatic_trail_m","r_eff_m","provenance_status"));
    const auto real_at=[&](const char* k){return vread::finite_real(vread::field(d,k),d.child(k));};
    return {.kx_n_per_m3=real_at("kx_n_per_m3"),.ky_n_per_m3=real_at("ky_n_per_m3"),
        .relaxation_x_m=real_at("relaxation_x_m"),.relaxation_y_m=real_at("relaxation_y_m"),
        .mu_x_scale=real_at("mu_x_scale"),.mu_y_scale=real_at("mu_y_scale"),
        .camber_coefficient_per_rad=real_at("camber_coefficient_per_rad"),.pneumatic_trail_m=real_at("pneumatic_trail_m"),
        .r_eff_m=real_at("r_eff_m"),.provenance_status=vread::string(vread::field(d,"provenance_status"),d.child("provenance_status"))};
}
}
bike::io::Value project_spatial_tire_config(const cfg::SimulationPhysicsConfig& config) {
    if(!config.spatial)throw std::invalid_argument("tires.pneumatic_3d: spatial configuration required");
    const auto profile=cfg::spatial_tyre_coefficient_profile(config.spatial->tyre.lateral_coefficients_id);
    cfg::validate_spatial_tyre_profile(profile);
    const auto& front=config.tires.pneumatic.front;const auto& rear=config.tires.pneumatic.rear;
    return Value{Object{{"tier",text(bike::spatial::to_string(config.spatial->tyre.tier))},
        {"outer_radius_m",reals(std::array{front.outer_radius_mm/1000.,rear.outer_radius_mm/1000.})},
        {"casing_width_m",reals(std::array{front.casing_width_mm/1000.,rear.casing_width_mm/1000.})},
        {"carcass",Value{Array{carcass_wire(front),carcass_wire(rear)}}},
        {"brush",Value{Array{brush_wire(profile,front.outer_radius_mm/1000.),brush_wire(profile,rear.outer_radius_mm/1000.)}}},
        {"patch_capacity",integer(0)},{"coefficient_profile",text(profile.id)},
        {"provenance_status",text(profile.status)}}};
}
pn::Pneumatic3dSpec decode_spatial_tire_config(const Value& value) {
    const vread::Dict d{.value=&bike::io::as_object(value,"config.tire.pneumatic_3d"),.path="config.tire.pneumatic_3d"};
    vread::exact(d,vread::keys("tier","outer_radius_m","casing_width_m","carcass","brush","patch_capacity","coefficient_profile","provenance_status"));
    pn::Pneumatic3dSpec out;
    const auto tier=bike::spatial::parse_tyre_tier(vread::string(vread::field(d,"tier"),d.child("tier")));
    if(!tier)vread::invalid(d.child("tier"),"expected fast or detailed");out.tier=*tier;
    out.outer_radius_m=vread::fixed<2>(vread::field(d,"outer_radius_m"),d.child("outer_radius_m"));
    out.casing_width_m=vread::fixed<2>(vread::field(d,"casing_width_m"),d.child("casing_width_m"));
    const auto capacity=vread::integer(vread::field(d,"patch_capacity"),d.child("patch_capacity"));
    const auto dims=pn::tier_dims(out.tier);
    if(capacity<0 || capacity>static_cast<std::int64_t>(dims.n_theta_fan)*dims.n_phi)
        vread::invalid(d.child("patch_capacity"),"outside grid capacity");
    out.patch_capacity=static_cast<std::uint32_t>(capacity);
    const auto& c=vread::sequence(vread::field(d,"carcass"),d.child("carcass"));
    const auto& b=vread::sequence(vread::field(d,"brush"),d.child("brush"));
    if(c.size()!=2 || b.size()!=2)vread::invalid(d.path,"expected front/rear coefficient records");
    for(std::size_t w=0;w<2;++w){out.carcass[w]=carcass(c[w],d.child("carcass")+"["+std::to_string(w)+"]");
        out.brush[w]=brush(b[w],d.child("brush")+"["+std::to_string(w)+"]");}
    const auto p=cfg::spatial_tyre_coefficient_profile(vread::string(vread::field(d,"coefficient_profile"),d.child("coefficient_profile")));
    if(vread::string(vread::field(d,"provenance_status"),d.child("provenance_status"))!=p.status)
        vread::invalid(d.child("provenance_status"),"differs from registered coefficient evidence");
    for(std::size_t w=0;w<2;++w)
        if(out.carcass[w].provenance_status!=p.status || out.brush[w].provenance_status!=p.status)
            vread::invalid(d.path,"child coefficient provenance differs from registered evidence");
    return out;
}
bike::rider::SteerProfile resolved_steer_profile(const bike::spatial::SteerConfig& config) {
    bike::rider::SteerProfile p;
    if(config.profile_id.empty() || config.profile_id=="roll-heading-v1" || config.profile_id=="steer-provisional-v1") {
        p.speed_nodes_mps={1.,3.,5.,8.};
        // Pin decimal source coefficients exactly as in the profile record;
        // arithmetic scaling can differ by one ulp and change its digest.
        p.gains={{{-64.,-12.8,-9.6,-3.2,-1.6,-.48}},
                 {{-40.,-8.,-6.,-2.,-1.,-.3}},
                 {{-32.,-6.4,-4.8,-1.6,-.8,-.24}},
                 {{-24.,-4.8,-3.6,-1.2,-.6,-.18}}};
        p.cap_nm=config.torque_cap_nm.value_or(40.);p.delay_s=config.reaction_delay_s;
        // Normalize/hash through the public cold decoder; no measured hash is fabricated.
        auto wire=Value{Object{{"schema",integer(1)},{"speed_nodes_mps",reals(p.speed_nodes_mps)},
            {"gains",Value{Array{}}},{"reference_torque_nm",real(0.)},{"cap_nm",real(p.cap_nm)},
            {"delay_s",real(p.delay_s)},{"sha256_hex",Value{}},{"provenance_status",text("provisional")}}};
        Array gains;for(const auto& row:p.gains)gains.push_back(reals(row));set(wire,"gains",Value{std::move(gains)});
        p=bike::rider::read_steer_profile(wire);
    } else {
        std::ifstream input(config.profile_id,std::ios::binary|std::ios::ate);
        if(!input || input.tellg()<0 || input.tellg()>1024*1024)
            throw std::invalid_argument("spatial_3d.steer.profile_id: unreadable or oversized profile file");
        std::string content(static_cast<std::size_t>(input.tellg()),'\0');input.seekg(0);
        input.read(content.data(),static_cast<std::streamsize>(content.size()));
        if(!input)throw std::invalid_argument("spatial_3d.steer.profile_id: incomplete read");
        p=bike::rider::read_steer_profile(bike::io::parse_json(content));
        if(config.torque_cap_nm && *config.torque_cap_nm!=p.cap_nm)
            throw std::invalid_argument("spatial_3d.steer.torque_cap_nm: differs from selected profile");
        if(config.reaction_delay_s!=p.delay_s)
            throw std::invalid_argument("spatial_3d.steer.reaction_delay_s: differs from selected profile");
    }
    if(!config.profile_sha256.empty() && config.profile_sha256!=p.sha256_hex)
        throw std::invalid_argument("spatial_3d.steer.profile_sha256: content mismatch");
    return p;
}
void resolve_spatial_runtime_profiles(cfg::SimulationPhysicsConfig& config) {
    if(!config.spatial || config.spatial->mode!=bike::spatial::Mode::spatial3d)return;
    auto& s=*config.spatial;
    if(s.tyre.lateral_coefficients_id.empty())s.tyre.lateral_coefficients_id="mtb-provisional-v1";
    const auto hash=digest(project_spatial_tire_config(config));
    if(!s.tyre.lateral_coefficients_sha256.empty() && s.tyre.lateral_coefficients_sha256!=hash)
        throw std::invalid_argument("tires.pneumatic_3d.lateral_coefficients_sha256: resolved coefficient content mismatch");
    s.tyre.lateral_coefficients_sha256=hash;
    if(s.steer.controller!=bike::spatial::SteerController::off) {
        if(!s.steer.torque_cap_nm)
            s.steer.torque_cap_nm=bike::model::resolve_shoulder_profile(config.articulated).swivel_strength_nm;
        const auto profile=resolved_steer_profile(s.steer);
        if(s.steer.profile_id.empty())s.steer.profile_id="roll-heading-v1";
        s.steer.profile_sha256=profile.sha256_hex;
    }
}
void install_spatial_tire(Stepper& stepper,const RuntimeConfig& config) {
    if(!config.spatial)return;
    if(!config.terrain3d)throw std::invalid_argument("config.spatial.terrain: physical source required");
    const auto& wire=field(field(config.writer_config,"tire"),"pneumatic_3d");
    auto spec=decode_spatial_tire_config(wire);
    if(spec.tier!=config.spatial->tyre.tier)
        throw std::invalid_argument("config.tire.pneumatic_3d.tier: differs from spatial declaration");
    if(bike::io::as_string(field(wire,"coefficient_profile"),"config.tire.pneumatic_3d.coefficient_profile")!=
       config.spatial->tyre.lateral_coefficients_id)
        throw std::invalid_argument("config.tire.pneumatic_3d.coefficient_profile: differs from spatial declaration");
    if(digest(wire)!=config.spatial->tyre.lateral_coefficients_sha256)
        throw std::invalid_argument("config.tire.pneumatic_3d: coefficient digest mismatch");
    const auto topology=bike::spatial::bind_topology(*stepper.model(),*config.spatial);
    const auto dims=pn::tier_dims(spec.tier);
    if(!spec.patch_capacity)spec.patch_capacity=dims.n_theta_fan*dims.n_phi;
    pn::TyreStateIdentity3d identity{.schema=1,.tier=spec.tier,.dims=dims,.patch_capacity=spec.patch_capacity,
        .topology_fingerprint_hex=topology.fingerprint_hex,
        .config_fingerprint_hex=digest(config.writer_config),
        .terrain_sha256_hex=config.terrain3d->fingerprint_hex(),
        .calibration_sha256_hex=config.spatial->tyre.lateral_coefficients_sha256};
    if(stepper.has_spatial_tire()) {
        if(digest(pn::tyre_identity_to_wire(stepper.spatial_tire_writer().state().identity))!=
           digest(pn::tyre_identity_to_wire(identity)))
            throw std::invalid_argument("config.tire.pneumatic_3d: installed producer identity mismatch");
        return;
    }
    stepper.install_spatial_tire(*config.spatial,config.terrain3d,std::move(spec),std::move(identity));
}
}
