#pragma once
#include "builder.hpp"
#include "xml_writer.hpp"
#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <span>
#include <vector>

namespace bike::model::detail {
using xml::Element;
using xml::Attributes;
inline Vec3 add(const Vec3& a,const Vec3& b) { return {a[0]+b[0],a[1]+b[1],a[2]+b[2]}; }
inline Vec3 sub(const Vec3& a,const Vec3& b) { return {a[0]-b[0],a[1]-b[1],a[2]-b[2]}; }
inline Vec3 mul(const Vec3& a,double b) { return {a[0]*b,a[1]*b,a[2]*b}; }
inline Vec3 div(const Vec3& a,double b) { return {a[0]/b,a[1]/b,a[2]/b}; }
inline Vec3 lateral(Vec3 a,double y) { a[1]=y; return a; }
inline std::string f(double value,int precision=6) { return geometry::detail::py_fixed(value,precision); }
inline std::string g(double value,int precision=17) { return geometry::detail::py_g(value,precision); }
inline std::string repr(double value) { return geometry::detail::py_repr(value); }
std::string vec(const Vec3& value,int precision=6);
std::string vec17(const Vec3& value);
std::string fromto(const Vec3& a,const Vec3& b,int precision=6);
Element& geom(Element& parent,std::string name,std::string type,Attributes attributes={});
Element& joint(Element& parent,std::string name,std::string type="hinge",Attributes attributes={});
Element& site(Element& parent,std::string name,std::string pos,std::string size="0.008",std::string rgba={});
Element& marker(Element& parent,std::string name,const Vec3& pos,std::string size="0.013",std::string material="mat_pivot_yellow");
Element& body(Element& parent,std::string name,const Vec3& pos);
Element& required(Element& root,std::string_view tag,std::string_view name);
Element& block(Element& root,std::string_view tag);
Element& inertial(Element& body,double mass,const Vec3& com,const Mat3& tensor);
void set_inertia(Element& body,double mass,const Vec3& com,const Mat3& tensor);
using MassRegistry = std::vector<std::pair<std::string,std::vector<Element*>>>;
void register_component(MassRegistry* registry,std::string name,const std::vector<Element*>& parts);
void assign_budgets(Element& frame,MassRegistry& registry,const BikeMassSpecs& masses);

struct BuildContext {
    // NOLINTNEXTLINE(cppcoreguidelines-avoid-const-or-ref-data-members) aggregate mirrors the Python reference structure
    const BuildRequest& request;
    // NOLINTNEXTLINE(cppcoreguidelines-avoid-const-or-ref-data-members) aggregate mirrors the Python reference structure
    const kinematics::HorstLinkageSolver& solver;
    configuration::RiderSpecs rider;
    std::optional<SeatedPose> seated;
    std::optional<ArticulatedPose> articulated;
    std::unique_ptr<configuration::SimulationPhysicsConfig> physics;
    geometry::FixedFramePoints fixed;
    kinematics::KinematicState state;
    Vec3 cg;
    bool crank_joint;
    MassRegistry registry;
    [[nodiscard]] bool physical() const { return physics && physics->physics_mode=="physical"; }
    [[nodiscard]] bool spatial() const { return request.spatial && request.spatial->mode==bike::spatial::Mode::spatial3d; }
    MassRegistry* mass_registry() { return physical()? &registry : nullptr; }
};
void build_visual_and_assets(Element& root);
void build_environment(Element& world,const BuildContext& context);
void build_terrain(Element& root,Element& world,const BuildContext& context);
void build_terrain(Element& root,Element& world,const BuildContext& context,
                   const terrain::RenderGrid3d& grid);
Element& build_frame_body(Element& world,BuildContext& context);
void build_bb_and_motor(Element& frame,BuildContext& context);
Element& build_rear_wheel(Element& seatstay,const Vec3& axle,BuildContext& context);
void build_steering_and_fork(Element& frame,BuildContext& context);
void build_rear_linkage(Element& frame,BuildContext& context);
void build_equality_constraints(Element& root,std::string_view mode);
void build_contact_exclusions(Element& root);
void build_chain_constraint(Element& root,double ratio);
void build_actuators(Element& root,const BuildContext& context);
void build_sensors(Element& root,const BuildContext& context);
void finish_physical_topology(Element& root,const BuildContext& context);
inline constexpr int kSpatialCrashMask=1;
inline constexpr int kSpatialTyreMask=2;
void finish_spatial_topology(Element& root,const BuildContext& context);
void finalize_geometric_transmission(Element& root,const configuration::SimulationPhysicsConfig& physics);
void build_articulated_rider(Element& world,const ArticulatedPose& pose,const SegmentMasses& masses,
                             const configuration::JointEnvelopeMap* envelopes,bool lock_ankles);
void add_rider_actuators(Element& root,const configuration::ArticulatedConfig& config);
void add_saddle_pin(Element& root,double solref);
void finish_rider_attachments(Element& root,const BuildContext& context);
} // namespace bike::model::detail
