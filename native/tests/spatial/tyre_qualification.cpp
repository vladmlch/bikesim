// W28 source-only qualification harness. Every sampled coefficient domain is
// exploratory until the declared tyre has measured data. Execution: V06.
#include "support/migration_test.hpp"
#include "spatial/support/spatial_fixture.hpp"
#include "spatial/support/terrain_fixture.hpp"
#include "spatial/support/tyre_fixture.hpp"
#include "pneumatic3d/model.hpp"
#include "pneumatic3d/state.hpp"
#include "configuration/physical_config.hpp"
#include "io/json.hpp"
#include "fixture_paths.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <numbers>
#include <string>
#include <vector>

namespace {
namespace pn=bike::pneumatic3d;namespace sp=bike::spatial;namespace io=bike::io;
using Kin=std::array<pn::WheelKinematics3d,2>;
constexpr double rad=std::numbers::pi/180.;
const io::Value& member(const io::Value& v,std::string_view key) {
    const auto* p=io::find_member(io::as_object(v,"qualification"),key);
    if(!p)throw std::invalid_argument("qualification."+std::string(key)+": missing field");return *p;
}
pn::TyreStateIdentity3d identity(sp::TyreTier tier,const bike::terrain::Terrain3dSource& terrain) {
    pn::TyreStateIdentity3d id;id.tier=tier;id.dims=pn::tier_dims(tier);
    id.patch_capacity=id.dims.n_theta_fan*id.dims.n_phi;id.topology_fingerprint_hex=std::string(64,'a');
    id.config_fingerprint_hex=std::string(64,'b');id.terrain_sha256_hex=terrain.fingerprint_hex();
    id.calibration_sha256_hex=std::string(64,'c');return id;
}
Kin kinematics(const pn::Pneumatic3dSpec& spec,const std::array<double,2>& height,
    double v,double kappa,double alpha,double gamma,double yaw=0.,double phase=0.) {
    Kin k;
    for(std::size_t w=0;w<2;++w) {
        k[w]=spatial_tests::wheel_at({4.+.5*static_cast<double>(w),0.,height[w]},yaw,gamma,v,
            (1.+kappa)*v/spec.brush[w].r_eff_m);
        k[w].hub.linear_velocity_world_mps[0]-=std::sin(yaw)*v*std::tan(alpha);
        k[w].hub.linear_velocity_world_mps[1]+=std::cos(yaw)*v*std::tan(alpha);
        k[w].wheel_phase_rad=phase;
    }
    return k;
}
std::array<double,2> load_height(pn::Pneumatic3dModel& model,double load,double gamma) {
    const auto& spec=model.spec();std::array<double,2> low,high,height;
    for(std::size_t w=0;w<2;++w) {
        high[w]=spec.outer_radius_m[w]+.02;low[w]=spec.outer_radius_m[w]-.04;height[w]=high[w];
    }
    for(int n=0;n<40;++n) {
        for(std::size_t w=0;w<2;++w)height[w]=.5*(low[w]+high[w]);
        bt::require(model.evaluate(kinematics(spec,height,0.,0.,0.,gamma),{},0.,false)==CoreStatus::ok,
            "load setup uses nonmutating valid contact probes");
        for(std::size_t w=0;w<2;++w) {
            double actual=0.;for(const auto& p:model.wrenches(w))actual+=p.normal_load_n;
            if(actual>load)low[w]=height[w];else high[w]=height[w];
        }
    }
    return height;
}
double norm(const sp::Vec3& v){return std::hypot(std::hypot(v[0],v[1]),v[2]);}
sp::Vec3 force(const pn::Pneumatic3dModel& model) {
    sp::Vec3 f{};for(std::size_t w=0;w<2;++w)for(const auto& p:model.wrenches(w))
        for(std::size_t j=0;j<3;++j)f[j]+=p.force_world_n[j];return f;
}
struct Report {std::size_t cases=0,invalid=0,exploratory=0;double max_ellipse=0.;};
void observe(const pn::Pneumatic3dModel& model,Report& report) {
    for(std::size_t w=0;w<2;++w)for(const auto& p:model.wrenches(w)) {
        if(!p.valid)continue;
        bt::require(std::isfinite(norm(p.force_world_n))&&std::isfinite(norm(p.couple_world_nm))&&
            std::isfinite(p.loss_j)&&p.loss_j>=-1e-12,"contact wrench and loss remain physical");
        const double cap=.8*p.normal_load_n;
        if(cap>0.) {
            const double ellipse=(p.fx_n*p.fx_n+p.fy_n*p.fy_n)/(cap*cap);
            report.max_ellipse=std::max(report.max_ellipse,ellipse);
            bt::require(ellipse<=1.+1e-12,"all slip/camber terms share the same friction ellipse");
        } else bt::require(p.fx_n==0.&&p.fy_n==0.,"unloaded contact has no tangential force");
    }
}
void matrix(sp::TyreTier tier,Report& report) {
    bike::terrain::Terrain3dSource terrain(spatial_tests::plane_grid(0.,0.,0.,33,33,.5,.25));
    for(double load:std::array{100.,300.,600.,900.})for(double pressure:std::array{1e5,1.5e5,2e5})
    for(double camber:std::array{0.,-5.,5.,-15.,15.,-30.,30.}) {
        auto spec=spatial_tests::synthetic_spec(tier);for(auto& c:spec.carcass)c.pressure_pa=pressure;
        pn::Pneumatic3dModel model(spec,terrain.query_view(),identity(tier,terrain));
        const auto empty=model.committed_state();const auto height=load_height(model,load,camber*rad);
        for(double v:std::array{0.,.1,1.,3.,8.})for(double kappa:std::array{0.,-.02,.02,-.1,.1,-.3,.3})
        for(double slip:std::array{0.,-1.,1.,-3.,3.,-6.,6.}) {
            ++report.cases;++report.exploratory;
            bt::require(model.restore(empty)==CoreStatus::ok,"matrix row resets all material/patch history");
            bool valid=true;
            for(int tick=0;tick<8;++tick) {
                const auto kin=kinematics(spec,height,v,kappa,slip*rad,camber*rad,0.,tick*.0005*v/spec.brush[0].r_eff_m);
                if(model.evaluate(kin,{},.0005,true)!=CoreStatus::ok){valid=false;break;}
                observe(model,report);
            }
            if(!valid)++report.invalid;
        }
    }
}
void provenance_and_withheld() {
    const auto registry=spatial_tests::json(std::filesystem::path(BIKE_SPATIAL_SOURCE_ROOT)/
        "native/tests/spatial/data/tyre_profiles.json");
    const auto& profiles=io::as_array(member(registry,"profiles"),"profiles");
    bt::require(profiles.size()==2,"front and rear profiles are independently identified");
    const std::array expected{bike::configuration::kFrontTyre,bike::configuration::kRearTyre};
    for(std::size_t i=0;i<profiles.size();++i) {
        const auto& p=profiles[i];const auto status=io::as_string(member(p,"status"),"status");
        const auto count=io::as_int64(member(p,"source_measurements"),"source_measurements");
        bt::require(status!="measured"||count>0,"provenance cannot promote invented coefficients");
        if(count==0)bt::require(status=="provisional"&&io::is_null(member(p,"digest")),
            "unmeasured profiles retain explicit provisional/null evidence");
        bt::require(io::as_string(member(p,"name"),"name")==expected[i].name&&
            std::abs(io::as_double(member(p,"outer_radius_m"),"outer_radius")-expected[i].outer_radius_mm*.001)<1e-12,
            "registry retains actual configuration tyre identity");
        for(const auto& [family,value]:io::as_object(member(p,"coefficient_families"),"families")) {
            (void)family;bt::require(io::as_string(member(value,"status"),"family.status")=="provisional"&&
                io::as_int64(member(value,"source_measurements"),"family.source_measurements")==0,
                "every coefficient family carries its own unresolved provenance");
        }
    }
    const auto source=spatial_tests::json(spatial_tests::fixture("references/tyre-lateral.json"));
    const auto& rows=io::as_array(member(source,"rows"),"rows");
    // Fixed withheld split by source-row index, independent of measured values.
    // This cross-check fits C_alpha = slope*Fz; it never calibrates MTB values.
    double xx=0.,xy=0.;std::size_t fit=0,withheld=0;
    for(std::size_t i=0;i<rows.size();++i) {
        const auto& row=rows[i];
        bt::require(io::as_string(member(row,"applicability"),"applicability")=="cross_check_only",
            "city/cargo reference rows cannot become an MTB measured domain");
        if(i%3==0)continue;
        const double x=io::as_double(member(row,"normal_load_n"),"load");
        const double y=io::as_double(member(row,"cornering_stiffness_n_per_rad"),"stiffness");
        xx+=x*x;xy+=x*y;++fit;
    }
    bt::require(fit>0&&xx>0.,"independent measured fit partition is nonempty");
    const double slope=xy/xx;double errors=0.;
    for(std::size_t i=0;i<rows.size();i+=3) {
        const auto& row=rows[i];const double x=io::as_double(member(row,"normal_load_n"),"load");
        const double y=io::as_double(member(row,"cornering_stiffness_n_per_rad"),"stiffness");
        errors+=(slope*x-y)*(slope*x-y);++withheld;
    }
    bt::require(withheld>0&&std::isfinite(errors),"withheld RMS uses held-out measured rows");
    std::cout<<"{\"reference_fit_rows\":"<<fit<<",\"withheld_rows\":"<<withheld
        <<",\"withheld_rms_n_per_rad\":"<<std::sqrt(errors/static_cast<double>(withheld))
        <<",\"applicability\":\"cross_check_only\",\"mtb_source_measurements\":0}\n";
}
void refinement_and_patch_lifecycle() {
    const auto plane=spatial_tests::plane_grid(0.,0.,0.,81,33,.1,.2);
    std::vector<std::uint32_t> materials(80U*32U);
    for(std::size_t y=0;y<32;++y)for(std::size_t x=0;x<80;++x)
        materials[y*80+x]=((x/2U)%2U==0)?1U:2U;
    auto surfaces=std::vector<bike::terrain::SurfaceSpec3d>(plane.surfaces().begin(),plane.surfaces().end());
    auto second=surfaces.front();second.id=2;second.name="second material, same physical coefficients";
    surfaces.push_back(second);
    auto grid=bike::terrain::Grid3d::create(plane.descriptor(),
        {plane.elevation().begin(),plane.elevation().end()},std::move(materials),std::move(surfaces),plane.metadata());
    bt::require(grid.has_value(),"striped contact-lifecycle grid is valid");
    bike::terrain::Terrain3dSource striped(std::move(*grid)),flat(plane);
    auto fast=spatial_tests::synthetic_spec(sp::TyreTier::fast);
    auto detailed=spatial_tests::synthetic_spec(sp::TyreTier::detailed);
    pn::Pneumatic3dModel coarse(fast,flat.query_view(),identity(fast.tier,flat));
    pn::Pneumatic3dModel fine(detailed,flat.query_view(),identity(detailed.tier,flat));
    for(double load:std::array{100.,300.,600.,900.}) {
        const auto h=load_height(coarse,load,0.);const auto k=kinematics(fast,h,0.,0.,0.,0.);
        bt::require(coarse.evaluate(k,{},0.,false)==CoreStatus::ok&&fine.evaluate(k,{},0.,false)==CoreStatus::ok,
            "normal grid refinement stays on an identical pose");
        const auto a=force(coarse),b=force(fine);
        bt::require(std::abs(a[2]-b[2])<=.02*std::max(1.,std::abs(b[2])),
            "fast versus detailed normal quadrature differs by at most 2 percent");
    }
    for(auto tier:{sp::TyreTier::fast,sp::TyreTier::detailed}) {
        const auto spec=spatial_tests::synthetic_spec(tier);
        pn::Pneumatic3dModel model(spec,striped.query_view(),identity(tier,striped));
        const auto h=load_height(model,600.,0.);std::size_t max_patches=0;double previous_loss=0.;
        for(int tick=0;tick<800;++tick) {
            const double t=tick*.001;const double speed=t<.4?.5:-.5;
            auto k=kinematics(spec,h,speed,.05,0.,0.,0.,t*1.4);
            for(auto& wheel:k)wheel.hub.position_world_m[0]+=t<.4?.5*t:.4-.5*t;
            bt::require(model.evaluate(k,{},.001,true)==CoreStatus::ok,"moving material boundaries split and merge without invalid state");
            max_patches=std::max(max_patches,model.patches(0).size());
            bt::require(model.stored_energy_j()>=0.&&model.dissipated_j()+1e-12>=previous_loss,
                "patch lifecycle cannot erase losses or create negative stored energy");
            previous_loss=model.dissipated_j();
        }
        bt::require(max_patches>=2,"lifecycle sample actually crosses a boundary and splits contact");
    }
}
void physical_properties(sp::TyreTier tier) {
    bike::terrain::Terrain3dSource terrain(spatial_tests::plane_grid(0.,0.,0.,33,33,.5,.25));
    auto spec=spatial_tests::synthetic_spec(tier);auto id=identity(tier,terrain);
    pn::Pneumatic3dModel a(spec,terrain.query_view(),id),b(spec,terrain.query_view(),id),c(spec,terrain.query_view(),id);
    const auto h=load_height(a,300.,.05);Report report;
    for(int tick=0;tick<200;++tick) {
        const double t=tick*.0005;
        auto k=kinematics(spec,h,3.,-.1,.03,.05,0.,t*8.);
        auto mirrored=kinematics(spec,h,3.,-.1,-.03,-.05,0.,t*8.);
        auto yaw=kinematics(spec,h,3.,-.1,.03,.05,std::numbers::pi/2.,t*8.);
        bt::require(a.evaluate(k,{},.0005,true)==CoreStatus::ok&&b.evaluate(mirrored,{},.0005,true)==CoreStatus::ok&&
            c.evaluate(yaw,{},.0005,true)==CoreStatus::ok,"mirrored/rotated models remain valid");
        const auto fa=force(a),fb=force(b),fc=force(c);
        bt::require(std::abs(fa[0]-fb[0])<=1e-8&&std::abs(fa[1]+fb[1])<=1e-8&&std::abs(fa[2]-fb[2])<=1e-8,
            "mirror changes only lateral force sign");
        bt::require(std::abs(fc[0]+fa[1])<=1e-8&&std::abs(fc[1]-fa[0])<=1e-8&&std::abs(fc[2]-fa[2])<=1e-8,
            "yaw rotation preserves local physics");observe(a,report);
    }
    for(auto& brush:spec.brush){brush.mu_x_scale=0.;brush.mu_y_scale=0.;}
    pn::Pneumatic3dModel no_grip(spec,terrain.query_view(),id);
    for(int tick=0;tick<20;++tick) {
        bt::require(no_grip.evaluate(kinematics(spec,h,3.,-.3,.1,.2,0.,tick*.01),{},.0005,true)==CoreStatus::ok,
            "zero-friction pressure/load boundary is valid");
        for(std::size_t w=0;w<2;++w)for(const auto& p:no_grip.wrenches(w))
            bt::require(p.fx_n==0.&&p.fy_n==0.,"mu to zero removes the entire tangential budget");
    }
    spec=spatial_tests::synthetic_spec(tier);
    const auto run=[&](double dt) {
        pn::Pneumatic3dModel m(spec,terrain.query_view(),id);double work=0.;
        const auto count=static_cast<int>(std::llround(.2/dt));
        for(int tick=0;tick<count;++tick) {
            const double t=tick*dt;const double v=t<.05?1.:t<.1?0.:-1.;
            const auto kin=kinematics(spec,h,v,.05,.02,0.,0.,t*4.);
            bt::require(m.evaluate(kin,{},dt,true)==CoreStatus::ok,"start/stop/reverse cannot become invalid");
            for(std::size_t w=0;w<2;++w)for(const auto& p:m.wrenches(w)) {
                const auto& hub=kin[w].hub;const auto& axle=kin[w].axle_world;
                const sp::Vec3 omega{axle[0]*kin[w].relative_spin_rad_s,axle[1]*kin[w].relative_spin_rad_s,axle[2]*kin[w].relative_spin_rad_s};
                const sp::Vec3 r{p.point_world_m[0]-hub.position_world_m[0],p.point_world_m[1]-hub.position_world_m[1],p.point_world_m[2]-hub.position_world_m[2]};
                const sp::Vec3 vel{hub.linear_velocity_world_mps[0]+omega[1]*r[2]-omega[2]*r[1],
                    hub.linear_velocity_world_mps[1]+omega[2]*r[0]-omega[0]*r[2],hub.linear_velocity_world_mps[2]+omega[0]*r[1]-omega[1]*r[0]};
                work-=dt*(p.force_world_n[0]*vel[0]+p.force_world_n[1]*vel[1]+
                    p.couple_world_nm[0]*omega[0]+p.couple_world_nm[1]*omega[1]+p.couple_world_nm[2]*omega[2]);
            }
        }
        double energy=0.,loss=0.;for(const auto& brush:m.committed_state().brush) {
            energy+=brush.elastic_energy_j;loss+=brush.sliding_loss_j+brush.convective_release_j;
        }
        bt::require(energy+loss<=work+1e-6*std::max(1.,std::abs(work))+1e-9,
            "brush energy plus losses cannot exceed supplied tangential work");
        return force(m);
    };
    const auto coarse=run(.001),fine=run(.0005),finer=run(.00025);
    const double e1=std::hypot(coarse[0]-fine[0],coarse[1]-fine[1]);
    const double e2=std::hypot(fine[0]-finer[0],fine[1]-finer[1]);
    bt::require(e2<=e1+1e-7,"step halving converges on stop/start/reverse force");
}
}
int main(){return bt::run([]{
    provenance_and_withheld();refinement_and_patch_lifecycle();Report report;
    for(auto tier:{sp::TyreTier::fast,sp::TyreTier::detailed}){physical_properties(tier);matrix(tier,report);}
    std::cout<<"{\"cases\":"<<report.cases<<",\"invalid_cases\":"<<report.invalid
        <<",\"exploratory_cases\":"<<report.exploratory<<",\"max_friction_ellipse\":"<<report.max_ellipse<<"}\n";
    bt::require(report.cases==41160&&report.exploratory==report.cases,"both complete Cartesian matrices are explicit exploratory samples");
    bt::require(report.invalid==0,"all qualified samples remain valid");
});}
