// model/mass.hpp — T15 mass/inertia boundary: physics/inertia.py's wheel
// profiles and tensor calculators, physics/component_masses.py's MJCF
// mass distribution, physics/mass.py's BikeMassSpecs plus the component
// center-of-mass and static-system-CG calculators, and
// sim/ride/mass_properties.py's compiled-model reads.
//
// Bitwise contract: expressions keep the oracle's operation order —
// elementwise NumPy array ops become per-component binary64 ops,
// np.linalg.norm / np.dot route through the Accelerate ILP64 ddot entry
// points (geometry::detail), builtin sum() routes through
// numeric::PythonSum (CPython's Neumaier float accumulator),
// ndarray.sum() through numeric::numpy_pairwise_sum, and `a ** b` goes
// through pyfloat::pow so each call reaches libm pow() like the oracle.
// `np.linalg.eigvalsh` reaches Accelerate's dsyevd ILP64 entry point —
// the same LAPACK routine numpy's gufunc calls, through the same
// $NEWLAPACK$ILP64 ABI.
#pragma once

#include <array>
#include <cstddef>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "../configuration/rider_spec.hpp" // configuration::RiderSpecs
#include "../geometry/specs.hpp"           // geometry::BikeSpecs, Vec3
#include "../kinematics/solver.hpp"        // kinematics::HorstLinkageSolver

// mujoco's mjModel/mjData are opaque to this header's consumers; the .cpp
// includes mujoco/mujoco.h.
struct mjModel_;
struct mjData_;

namespace bike::model {

using geometry::Vec3;
// Row-major 3x3 double matrix — the domain of numpy's (3, 3) float64
// arrays in the ported sources.
using Mat3 = std::array<double, 9>;

// ---------------------------------------------------------------------------
// physics/inertia.py — annular-cylinder wheel profile.
// ---------------------------------------------------------------------------

// WheelComponent NamedTuple (field order verbatim).
struct WheelComponent {
    double mass_fraction;
    double inner_radius_m;
    double outer_radius_m;
    double width_m;
    Vec3 offset_m;
};

// FRONT_WHEEL_PROFILE / REAR_WHEEL_PROFILE — the synthetic ring/core
// tuples, verbatim.
inline constexpr std::array<WheelComponent, 2> kFrontWheelProfile{
    WheelComponent{.mass_fraction = 0.75, .inner_radius_m = 0.320,
                   .outer_radius_m = 0.372, .width_m = 0.060,
                   .offset_m = {0.0, 0.0, 0.0}},
    WheelComponent{.mass_fraction = 0.25, .inner_radius_m = 0.000,
                   .outer_radius_m = 0.045, .width_m = 0.110,
                   .offset_m = {0.0, 0.0, 0.0}},
};
inline constexpr std::array<WheelComponent, 2> kRearWheelProfile{
    WheelComponent{.mass_fraction = 0.75, .inner_radius_m = 0.300,
                   .outer_radius_m = 0.352, .width_m = 0.064,
                   .offset_m = {0.0, 0.0, 0.0}},
    WheelComponent{.mass_fraction = 0.25, .inner_radius_m = 0.000,
                   .outer_radius_m = 0.045, .width_m = 0.148,
                   .offset_m = {0.0, 0.0, 0.0}},
};

// ring_inertia(mass_kg, inner_m, outer_m, width_m) -> np.diag 3x3.
[[nodiscard]] Mat3 ring_inertia(double mass_kg, double inner_m,
                                double outer_m, double width_m);

// parallel_axis(mass_kg, offset_m) -> mass * (r·r I - r⊗r).
[[nodiscard]] Mat3 parallel_axis(double mass_kg, const Vec3 &offset_m);

// wheel_body_inertia(mass_kg, components) -> (com, tensor).
struct WheelInertia {
    Vec3 com;
    Mat3 tensor;
};
[[nodiscard]] WheelInertia
wheel_body_inertia(double mass_kg, std::span<const WheelComponent> components);

// ---------------------------------------------------------------------------
// MJCF element carrier for the component-mass/inertial ports.
//
// Python operates on xml.etree.ElementTree.Element; the model boundary
// needs only the three operations the sources perform: attribute get with
// a default, attribute set, and find/append of the `inertial` child. The
// T16 MJCF emission layer substitutes its real element type for this
// carrier — the validation and serialization semantics being ported live
// here, not in the element shape.
// ---------------------------------------------------------------------------
struct XmlGeomElement {
    // geom.get("mass", "0") — nullopt when the attribute is absent.
    std::optional<std::string> mass;
};

// The inertial sub-element add_body_inertial emits (attribute insertion
// order preserved).
struct XmlInertialAttributes {
    std::string mass;
    std::string pos;
    std::string fullinertia;
};

// The `body` element surface add_body_inertial touches: find("inertial")
// reports whether an inertial child already exists; ET.SubElement writes
// the attribute dict in declared order.
struct XmlBodyElement {
    std::optional<XmlInertialAttributes> inertial;
};

// component_masses.py assign_component_mass(geoms, total_kg) — writes
// format(total_kg * weight / denominator, ".17g") into each element.
void assign_component_mass(std::span<XmlGeomElement> geoms,
                           double total_kg);

// component_masses.py register_component(registry, component_id, parts).
// The registry maps component ids to the positive-mass element views;
// the ordered pair list keeps the Python dict's insertion order (map
// lookups use linear scans over it).
using ComponentRegistry =
    std::vector<std::pair<std::string, std::vector<const XmlGeomElement *>>>;
void register_component(ComponentRegistry &registry,
                        const std::string &component_id,
                        std::span<const XmlGeomElement> parts);

// inertia.py add_body_inertial(body, mass_kg, com_m, tensor) — full
// validation then the MuJoCo full-tensor serialization order.
void add_body_inertial(XmlBodyElement &body, double mass_kg,
                       const Vec3 &com_m, const Mat3 &tensor);

// ---------------------------------------------------------------------------
// physics/mass.py — BikeMassSpecs.
// ---------------------------------------------------------------------------

// BikeMassSpecs dataclass: the fifteen physical budgets in declared field
// order then the two legacy wheel fractions. A plain (mutable) record like
// the Python dataclass — no __post_init__ exists upstream.
struct BikeMassSpecs {
    double motor_mass = 2.90;
    double battery_mass = 4.30;
    double frame_structure_mass = 3.20;
    double saddle_post_mass = 0.90;
    double crank_pedals_mass = 0.85;
    double steer_assembly_mass = 1.10;
    double stanchions_mass = 0.85;
    double fork_lowers_mass = 1.50;
    double chainstay_mass = 1.25;
    double seatstay_mass = 1.05;
    double rocker_mass = 0.40;
    double shock_yoke_mass = 0.30;
    double shock_damper_mass = 0.60;
    double front_wheel_mass = 2.40;
    double rear_wheel_mass = 2.80;
    double wheel_rim_tire_fraction = 0.75;
    double wheel_hub_core_fraction = 0.25;

    // component_masses property: {component_id: kg} in declared field
    // order ("_mass" suffix stripped).
    [[nodiscard]] std::vector<std::pair<std::string, double>>
    component_masses() const;
    // component_provenance property: {component_id: {"mass":
    // "synthetic", "distribution": "synthetic"}} — same field order.
    struct ComponentProvenance {
        std::string_view mass;
        std::string_view distribution;
    };
    [[nodiscard]] std::vector<
            std::pair<std::string, ComponentProvenance>>
    component_provenance() const;
    [[nodiscard]] double total_bike_mass() const;
    // compute_wheel_rotational_inertia — the legacy analytic I_yy.
    [[nodiscard]] double
    compute_wheel_rotational_inertia(double wheel_mass,
                                     double outer_radius,
                                     double hub_radius = 0.045) const;
};

// compute_component_centers_of_mass's dict values: (CoM m, mass kg). The
// ordered vector mirrors the Python dict's insertion order exactly —
// callers that need key lookup use find() rather than relying on order.
struct ComponentCenter {
    std::string name;
    Vec3 pos_m;
    double mass_kg;
};
using ComponentCenters = std::vector<ComponentCenter>;

// mass.py saddle_top_z_for_rider — the seated rider's required saddle top
// height, or nullopt (None) for other variants / no rider.
[[nodiscard]] std::optional<double>
saddle_top_z_for_rider(const geometry::BikeSpecs &specs,
                       const configuration::RiderSpecs *rider_specs);

// compute_component_centers_of_mass(specs=None, mass_specs=None,
// solver=None, rider_specs=None). Null/None defaults build the declared
// Python defaults: BikeSpecs()/BikeMassSpecs()/a fresh
// HorstLinkageSolver(specs)/no rider.
[[nodiscard]] ComponentCenters compute_component_centers_of_mass(
    const geometry::BikeSpecs &specs, const BikeMassSpecs &mass_specs,
    const kinematics::HorstLinkageSolver &solver,
    const configuration::RiderSpecs *rider_specs = nullptr);
[[nodiscard]] ComponentCenters compute_component_centers_of_mass(
    const geometry::BikeSpecs &specs, const BikeMassSpecs &mass_specs,
    const configuration::RiderSpecs *rider_specs = nullptr);
[[nodiscard]] ComponentCenters
compute_component_centers_of_mass(const geometry::BikeSpecs &specs);

// compute_static_system_cg(...) result dict — key order verbatim.
struct StaticSystemCg {
    double total_mass_kg;
    double bike_mass_kg;
    double rider_mass_kg;
    Vec3 cg_pos_m;
    double wheelbase_m;
    double front_axle_x;
    double rear_axle_x;
    double front_load_n;
    double rear_load_n;
    double front_load_pct;
    double rear_load_pct;
};
// Python signature: (specs=None, mass_specs=None, rider_specs=None,
// solver=None).
[[nodiscard]] StaticSystemCg compute_static_system_cg(
    const geometry::BikeSpecs &specs, const BikeMassSpecs &mass_specs,
    const configuration::RiderSpecs *rider_specs = nullptr,
    const kinematics::HorstLinkageSolver *solver = nullptr);

// mass.py's module alias is the same function:
// compute_unloaded_analytic_system_cg = compute_static_system_cg.

// ---------------------------------------------------------------------------
// sim/ride/mass_properties.py — compiled-model reads.
// ---------------------------------------------------------------------------

// compiled_center_of_mass(model, data) -> np.ndarray (3,).
[[nodiscard]] Vec3 compiled_center_of_mass(const mjModel_ &model,
                                           const mjData_ &data);

// static_contact_loads(model, data, front_geom_id, rear_geom_id)
// -> (front_load, rear_load).
[[nodiscard]] std::pair<double, double>
static_contact_loads(const mjModel_ &model, const mjData_ &data,
                     int front_geom_id, int rear_geom_id);

} // namespace bike::model
