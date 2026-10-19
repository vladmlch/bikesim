#include "tyre.hpp"

#include <algorithm>
#include <cmath>
#include <ranges>
#include <stdexcept>
#include <utility>

#include "../config_validation.hpp"
#include "../engine_call.hpp"
#include "../model/mapping.hpp"
#include "../terrain/profile.hpp"
#include "../writers/pyfloat.hpp"
#include "../numeric_sincos.hpp"

namespace bike::legacy {
TyreSpecs make_tyre_specs(std::string name,double radius,double rim,double height,double pressure,double crr,
        double width,double casing,double area,double stiffness,double length,double rate,double relaxation,
        double loss,double epsilon,double longitudinal,double rim_stiffness,double tread){
    TyreSpecs result{.name=std::move(name),.outer_radius_mm=radius,.rim_radius_mm=rim,.section_height_mm=height,.pressure_bar=pressure,.crr_reference=crr,.casing_width_mm=width,.compressed_casing_mm=casing,.area_factor=area,.carcass_stiffness_n_mm2=stiffness,.contact_length_factor=length,.rate_stiffening=rate,.rate_relaxation_s=relaxation,.loss_factor=loss,.hysteresis_rate_eps_mps=epsilon,.relaxation_length_mm=longitudinal,.rim_stiffness_n_mm2=rim_stiffness,.tread_loss_crr=tread};
    nativecfg::validate(result,"legacy.tyre");return result;
}
TyreSpecs front_tyre() {
    return {.name="Schwalbe Magic Mary 29x2.4", .outer_radius_mm=372., .rim_radius_mm=320., .section_height_mm=57., .pressure_bar=1.5, .crr_reference=.011,
        .casing_width_mm=60., .compressed_casing_mm=6., .area_factor=.4435, .carcass_stiffness_n_mm2=.1194, .contact_length_factor=.8589, .rate_stiffening=.2484, .rate_relaxation_s=.2, .loss_factor=.0662, .hysteresis_rate_eps_mps=.01, .relaxation_length_mm=60., .rim_stiffness_n_mm2=28., .tread_loss_crr=.00586};
}
TyreSpecs rear_tyre() {
    return {.name="Schwalbe Hans Dampf 27.5x2.4", .outer_radius_mm=352., .rim_radius_mm=300., .section_height_mm=55., .pressure_bar=1.7, .crr_reference=.0103,
        .casing_width_mm=60., .compressed_casing_mm=6., .area_factor=.4576, .carcass_stiffness_n_mm2=.1225, .contact_length_factor=.8801, .rate_stiffening=.2498, .rate_relaxation_s=.2, .loss_factor=.0690, .hysteresis_rate_eps_mps=.01, .relaxation_length_mm=60., .rim_stiffness_n_mm2=28., .tread_loss_crr=.00481};
}
TyreSpecs with_pressure(TyreSpecs tyre, double pressure_bar) {
    tyre.pressure_bar = pressure_bar;
    nativecfg::validate(tyre, "legacy.tyre");
    return tyre;
}
double static_stiffness_target_n_mm(double pressure_bar) { return STATIC_STIFFNESS_INTERCEPT_N_MM + STATIC_STIFFNESS_SLOPE_N_MM_PER_BAR * pressure_bar; }
double crr_target(const TyreSpecs& tyre, std::optional<double> pressure_bar) {
    const double pressure = pressure_bar.value_or(tyre.pressure_bar);
    if (!std::isfinite(pressure) || pressure <= 0.)
        throw std::invalid_argument("legacy.tyre.pressure_bar must be finite and positive");
    return tyre.crr_reference * pyfloat::float_pow(pressure / CRR_REFERENCE_PRESSURE_BAR, CRR_PRESSURE_EXPONENT);
}
TierSpec tier_spec(std::string_view name) {
    if (name == "fast") return {.name="fast", .n_rays=64, .half_angle_deg=75., .timestep_s=.0005, .discretised_brush=false};
    if (name == "detailed") return {.name="detailed", .n_rays=256, .half_angle_deg=75., .timestep_s=.00025, .discretised_brush=true};
    throw std::invalid_argument("unknown legacy tyre tier '" + std::string(name) + "'");
}
void TyreConfig::validate() const {
    if (model != "sphere" && model != "pneumatic")
        throw std::invalid_argument("unknown legacy tyre model '" + model + "'");
    (void)tier_spec(tier);
    nativecfg::validate(front, "legacy.tyre.front");
    nativecfg::validate(rear, "legacy.tyre.rear");
    if (surface) (void)terrain::get_surface(*surface);
}
TyreConfig TyreConfig::with_pressures(double front_bar, double rear_bar) const {
    auto result = *this;
    result.front = with_pressure(front, front_bar);
    result.rear = with_pressure(rear, rear_bar);
    return result;
}
biketyre::RoadProfile build_road_profile(const terrain::TrackSpec& track,
        const terrain::HeightFieldSpec& field, const geometry::BikeSpecs& specs) {
    const auto x = field.track_x();
    auto z = terrain::build_profile(track, x);
    const double ground = geometry::compute_ground_z(specs) / 1000.;
    for (auto& height : z) height = ground + height;
    return {x.front(), x[1] - x[0], std::move(z)};
}
std::optional<double> first_crossing_reference(double theta,double cx,double cz,double radius,const biketyre::RoadProfile& road,std::size_t samples){
    if(samples==0)return std::nullopt;
    for(double const value:{theta,cx,cz,radius})if(!std::isfinite(value))throw std::invalid_argument("ray reference inputs must be finite");
    const double ux=numeric::np_sin(theta),uz=-numeric::np_cos(theta);
    const auto below=[&](double s){return cz+s*uz<=road.height_at(cx+s*ux);};
    const auto grid=geometry::detail::np_linspace(0.,radius,samples);
    for(std::size_t k=0;k<grid.size();++k)if(below(grid[k])){if(k==0)return 0.;double a=grid[k-1],b=grid[k];
        for(int i=0;i<60;++i){const double mid=.5*(a+b);if(below(mid))b=mid;else a=mid;}return .5*(a+b);}
    return std::nullopt;
}
namespace {
nativecfg::SurfaceSpec surface(const terrain::SurfaceSpec& value) {
    return {.name=value.name, .mu_peak=value.mu_peak, .mu_slide=value.mu_slide,
        .slip_stiffness_per_load=value.slip_stiffness_per_load, .stribeck_speed_mps=value.stribeck_speed_mps};
}
nativecfg::SurfaceMap surfaces(const terrain::SurfaceMap& map) {
    // surfaces()[0] is the default, including when sections cover x=0.
    nativecfg::SurfaceMap result{.surface=surface(map.surfaces().front()), .sections={}};
    for (const auto& section : map.sections())
        result.sections.push_back({.start_m=section.start_m, .end_m=section.end_m,
            .surface=surface(terrain::get_surface(section.surface))});
    return result;
}
struct WheelBinding { int body, spin, geom; };
WheelBinding resolve(const mjModel& model, std::string_view side, double radius) {
    const auto prefix = std::string(side);
    const int body = bike::model::required_id(model, mjOBJ_BODY, prefix + "_wheel");
    const int joint = bike::model::required_id(model, mjOBJ_JOINT, prefix + "_wheel_spin");
    const int geom = bike::model::required_id(model, mjOBJ_GEOM, "geom_" + prefix + "_contact");
    const auto types = std::views::counted(model.geom_type, model.ngeom);
    const auto sizes = std::views::counted(model.geom_size, 3 * model.ngeom);
    const auto geometry_index=static_cast<std::size_t>(geom);
    if (types[geometry_index] != mjGEOM_SPHERE || std::abs(sizes[3 * geometry_index] - radius) > 1e-6)
        throw std::invalid_argument("legacy tyre contact geometry/radius mismatch: " + prefix);
    return {.body=body, .spin=std::views::counted(model.jnt_dofadr, model.njnt)[static_cast<std::size_t>(joint)], .geom=geom};
}
}
struct TyreForceApplier::Impl {
    // NOLINTNEXTLINE(cppcoreguidelines-avoid-const-or-ref-data-members) aggregate mirrors the Python reference structure
    mjModel& model;
    TyreConfig config;
    biketyre::RoadProfile profile;
    nativecfg::SurfaceMap surface_map;
    std::array<WheelBinding, 2> wheels;
    std::array<std::unique_ptr<biketyre::PneumaticTyreModel>, 2> tyres, backups;
    Impl(mjModel& m, TyreConfig c, biketyre::RoadProfile p,
            const terrain::SurfaceMap& surface_map_in)
        : model(m), config(std::move(c)), profile(std::move(p)),
          surface_map(surfaces(config.surface ? terrain::SurfaceMap::uniform(*config.surface)
                                             : surface_map_in)),
          wheels{resolve(m, "front", config.front.outer_radius_mm / 1000.),
                 resolve(m, "rear", config.rear.outer_radius_mm / 1000.)} {
        const auto tier = tier_spec(config.tier);
        for (std::size_t i = 0; i < 2; ++i) {
            const auto& spec = i == 0 ? config.front : config.rear;
            tyres[i] = std::make_unique<biketyre::PneumaticTyreModel>(spec,
                tier.n_rays, tier.half_angle_deg, tier.discretised_brush);
            backups[i] = std::make_unique<biketyre::PneumaticTyreModel>(spec,
                tier.n_rays, tier.half_angle_deg, tier.discretised_brush);
            if (tier.discretised_brush) {
                tyres[i]->prewarm_detailed_brushes();
                backups[i]->prewarm_detailed_brushes();
            }
        }
        // Source only overrides the detailed tier. Fast preserves the
        // ordinary legacy timestep selected by the model configuration.
        if (tier.name == "detailed") model.opt.timestep = tier.timestep_s;
    }
};
TyreForceApplier::TyreForceApplier(mjModel& model, TyreConfig config,
        biketyre::RoadProfile profile, const terrain::SurfaceMap& map) {
    config.validate();
    if (!config.pneumatic()) throw std::invalid_argument("TyreForceApplier requires pneumatic tyres");
    impl_ = std::make_unique<Impl>(model, std::move(config), std::move(profile), map);
}
TyreForceApplier::~TyreForceApplier() = default;
void TyreForceApplier::apply(mjData& data) {
    auto& p = *impl_;
    const auto positions = std::views::counted(data.xpos, 3 * p.model.nbody);
    const auto com = std::views::counted(data.xipos, 3 * p.model.nbody);
    std::array<std::array<double, 6>, 2> wrenches{};
    for (std::size_t i = 0; i < 2; ++i) p.backups[i]->assign(*p.tyres[i]);
    try {
        for (std::size_t i = 0; i < 2; ++i) {
            const int body = p.wheels[i].body;
            const auto body_index=static_cast<std::size_t>(body);
            std::array<double, 6> velocity{};
            struct VelocityCall{const mjModel* model;const mjData* data;int body;double* velocity;} call{.model=&p.model,.data=&data,.body=body,.velocity=velocity.data()};
            engine::ErrorBuffer error;
            if(!engine::invoke([](void* raw)noexcept{auto const& c=*static_cast<VelocityCall*>(raw);mj_objectVelocity(c.model,c.data,mjOBJ_BODY,c.body,c.velocity,0);},&call,error))engine::throw_failure(error);
            const auto& output = p.tyres[i]->evaluate(positions.subspan(3 * body_index, 3),
                std::span<const double>(velocity).subspan(3, 3), velocity[1], p.profile,
                p.surface_map, p.model.opt.timestep);
            auto& wrench = wrenches[i];
            std::ranges::copy(output.force_world_n, wrench.begin());
            for (const auto& patch : output.patches()) {
                const double rx = patch.centroid_world_m[0] - com[3 * body_index];
                const double ry = patch.centroid_world_m[1] - com[3 * body_index + 1];
                const double rz = patch.centroid_world_m[2] - com[3 * body_index + 2];
                const auto& f = patch.force_world_n;
                wrench[3] += ry * f[2] - rz * f[1];
                wrench[4] += rz * f[0] - rx * f[2];
                wrench[5] += rx * f[1] - ry * f[0];
            }
            if(!std::ranges::all_of(wrench,[](double value){return std::isfinite(value);}))
                throw std::invalid_argument("legacy tyre wrench is nonfinite");
        }
    } catch (...) {
        for (std::size_t i = 0; i < 2; ++i) p.tyres[i]->assign(*p.backups[i]);
        throw;
    }
    auto const applied = std::views::counted(data.xfrc_applied, 6 * p.model.nbody);
    for (std::size_t i = 0; i < 2; ++i)
        std::ranges::copy(wrenches[i], applied.subspan(6 * static_cast<std::size_t>(p.wheels[i].body), 6).begin());
}
void TyreForceApplier::reset(mjData* data) {
    auto& p = *impl_;
    for (std::size_t i = 0; i < 2; ++i) {
        p.tyres[i]->reset();
        if (data) std::ranges::fill(std::views::counted(data->xfrc_applied,
            6 * p.model.nbody).subspan(6 * static_cast<std::size_t>(p.wheels[i].body), 6), 0.);
    }
}
void TyreForceApplier::set_pressures(double front_bar, double rear_bar) {
    if (!std::isfinite(front_bar) || !std::isfinite(rear_bar))
        throw std::invalid_argument("legacy tyre live pressures must be finite");
    auto next = impl_->config.with_pressures(biketyre::clamp_pressure_bar(front_bar),
                                             biketyre::clamp_pressure_bar(rear_bar));
    impl_->tyres[0]->set_pressure(next.front.pressure_bar);
    impl_->tyres[1]->set_pressure(next.rear.pressure_bar);
    impl_->config = std::move(next);
}
const biketyre::WheelOutputs& TyreForceApplier::front_outputs() const { return impl_->tyres[0]->last_outputs(); }
const biketyre::WheelOutputs& TyreForceApplier::rear_outputs() const { return impl_->tyres[1]->last_outputs(); }
const TyreConfig& TyreForceApplier::config() const noexcept { return impl_->config; }
} // namespace bike::legacy
