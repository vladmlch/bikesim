#include "accounting.hpp"
#include "checkpoint_io.hpp"
#include "../rider/state_codec.hpp"

#include <limits>
#include <map>
#include <memory>
#include <set>

namespace runtime {
namespace {
using namespace checkpoint_io;

Value constraints(const ConstraintSnapshot& c) {
    return Value{Object{{"start", real(c.interval_start_s)}, {"end", real(c.interval_end_s)},
        {"qvel", reals(c.qvel_start)}, {"components", components(c.components)}}};
}
ConstraintSnapshot constraints(const Value& value, std::size_t nv) {
    exact(value, {"start", "end", "qvel", "components"});
    ConstraintSnapshot out{.interval_start_s=number(field(value,"start")),
        .interval_end_s=number(field(value,"end")), .qvel_start=numbers(field(value,"qvel"),nv),
        .components=components(field(value,"components"),nv)};
    if (out.interval_start_s < 0. || out.interval_end_s <= out.interval_start_s)
        vread::invalid("checkpoint.constraints", "invalid interval");
    return out;
}
Value diagnostics(const AccountingDiagnostics& d) {
    Object contacts;
    for (const auto& [name, item] : d.contacts)
        contacts.emplace_back(name, rider::encode_attachment_entry(item));
    auto state=std::make_unique<rider::SpindleController::State>();
    state->last_terms=d.terms; state->effort_diagnostics=d.effort;
    return Value{Object{{"contacts",Value{std::move(contacts)}},
        {"attachment_errors",strings(d.attachment_errors)}, {"rider",rider::encode_spindle_state(*state)}}};
}
AccountingDiagnostics diagnostics(const Value& value) {
    exact(value,{"contacts","attachment_errors","rider"});
    AccountingDiagnostics out;
    for (const auto& [name,item] : bike::io::as_object(field(value,"contacts"),"checkpoint.contacts"))
        out.contacts.emplace_back(name,rider::decode_attachment_entry(item,"checkpoint.contacts."+name));
    out.attachment_errors=strings(field(value,"attachment_errors"));
    const std::unique_ptr<rider::SpindleController::State> state{
        new rider::SpindleController::State(rider::decode_spindle_state(field(value,"rider"),"checkpoint.rider"))};
    out.terms=state->last_terms; out.effort=state->effort_diagnostics;
    return out;
}
Value raw(const RawStep& r) {
    Object attachments;
    for (const auto& [name,item] : r.attachment_raw)
        attachments.emplace_back(name,rider::encode_attachment_entry(writers::RiderAttachmentEntry{item}));
    auto rider_state=std::make_unique<rider::SpindleController::State>();
    if(r.rider_control_terms)rider_state->last_terms=*r.rider_control_terms;
    if(r.effort_state)rider_state->effort_diagnostics=*r.effort_state;
    Object powers;
    for(const auto& [name,value]:r.numerical_constraint_power_w)powers.emplace_back(name,real(value));
    return Value{Object{{"interval",integer(r.interval_id)}, {"time",real(r.time_s)}, {"end",real(r.end_time_s)},
        {"qpos",reals(r.qpos)}, {"qvel",reals(r.qvel)}, {"components",components(r.components)},
        {"attachments",Value{std::move(attachments)}}, {"actuator_force",reals(r.actuator_force)},
        {"qfrc_passive",reals(r.qfrc_passive)}, {"contact_truth",wire(Wire{r.contact_truth})},
        {"channels",wire(Wire{r.channels})}, {"diagnostics",wire(Wire{r.diagnostics})},
        {"attachment_errors",strings(r.attachment_errors)},
        {"rider",rider::encode_spindle_state(*rider_state)},
        {"has_terms",Value{r.rider_control_terms.has_value()}}, {"has_effort",Value{r.effort_state.has_value()}},
        {"constraint_power",Value{std::move(powers)}}, {"loss",real(r.loss_step_j)},
        {"mechanical",optional(r.mechanical_energy_j)},
        {"elastic",r.elastic_energy_j?wire(Wire{*r.elastic_energy_j}):Value{}},
        {"battery",real(r.battery_energy_j)}, {"electrical",wire(r.electrical_power_w)},
        {"invalid_controller",Value{r.invalid_controller}}, {"effort_base",wire(Wire{r.effort_base})},
        {"constraints",constraints(r.constraint_snapshot)}, {"full",Value{r.full}}}};
}
RawStep raw(const Value& value,const AccountingConfig& config) {
    exact(value,{"interval","time","end","qpos","qvel","components","attachments","actuator_force",
        "qfrc_passive","contact_truth","channels","diagnostics","attachment_errors","rider",
        "has_terms","has_effort","constraint_power","loss","mechanical","elastic","battery",
        "electrical","invalid_controller","effort_base","constraints","full"});
    RawStep out;
    out.interval_id=count(field(value,"interval"));out.time_s=number(field(value,"time"));out.end_time_s=number(field(value,"end"));
    out.qpos=numbers(field(value,"qpos"),config.max_qpos);out.qvel=numbers(field(value,"qvel"),config.max_dofs);
    out.components=components(field(value,"components"),config.max_dofs);
    for(const auto& [name,item]:bike::io::as_object(field(value,"attachments"),"checkpoint.attachments")) {
        auto decoded=rider::decode_attachment_entry(item,"checkpoint.attachment."+name);
        if(!std::holds_alternative<rider::AttachmentRaw>(decoded))
            vread::invalid("checkpoint.attachment","raw input required");
        out.attachment_raw.emplace_back(name,std::move(std::get<rider::AttachmentRaw>(decoded)));
    }
    out.actuator_force=numbers(field(value,"actuator_force"),config.max_actuators);
    out.qfrc_passive=numbers(field(value,"qfrc_passive"),config.max_dofs);
    out.contact_truth=object(field(value,"contact_truth"));out.channels=object(field(value,"channels"));
    out.diagnostics=object(field(value,"diagnostics"));out.attachment_errors=strings(field(value,"attachment_errors"));
    const std::unique_ptr<rider::SpindleController::State> rider_state{
        new rider::SpindleController::State(rider::decode_spindle_state(field(value,"rider"),"checkpoint.raw.rider"))};
    if(boolean(field(value,"has_terms")))out.rider_control_terms=rider_state->last_terms;
    if(boolean(field(value,"has_effort")))out.effort_state=rider_state->effort_diagnostics;
    for(const auto& [name,item]:bike::io::as_object(field(value,"constraint_power"),"checkpoint.constraint_power"))
        out.numerical_constraint_power_w.emplace_back(name,number(item));
    out.loss_step_j=number(field(value,"loss"));out.mechanical_energy_j=optional(field(value,"mechanical"));
    if(!bike::io::is_null(field(value,"elastic")))out.elastic_energy_j=object(field(value,"elastic"));
    out.battery_energy_j=number(field(value,"battery"));out.electrical_power_w=vread::wire_from_value(field(value,"electrical"),"checkpoint.electrical");
    out.invalid_controller=boolean(field(value,"invalid_controller"));out.effort_base=object(field(value,"effort_base"));
    out.constraint_snapshot=constraints(field(value,"constraints"),config.max_dofs);out.full=boolean(field(value,"full"));
    if(out.interval_id<0 || out.time_s<0. || out.end_time_s<=out.time_s ||
       std::abs((out.end_time_s-out.time_s)-config.timestep_s)>1e-10 ||
       out.constraint_snapshot.interval_start_s!=out.time_s || out.constraint_snapshot.interval_end_s!=out.end_time_s)
        vread::invalid("checkpoint.raw","invalid interval identity");
    return out;
}
Value sample(const SamplePtr& p) {
    if(!p)return {};
    const auto& r=*p;
    return Value{Object{{"interval",integer(r.interval_id)},{"time",real(r.time_s)},{"end",real(r.end_time_s)},
        {"qpos",reals(r.qpos)},{"qvel",reals(r.qvel)},{"forces",components(r.forces)},
        {"powers",wire(Wire{r.powers_w})},{"channels",wire(Wire{r.channels})},
        {"constraints",r.constraints?constraints(*r.constraints):Value{}}}};
}
SamplePtr sample(const Value& value,const AccountingConfig& config) {
    if(bike::io::is_null(value))return {};
    exact(value,{"interval","time","end","qpos","qvel","forces","powers","channels","constraints"});
    auto out=std::make_shared<PhysicalSampleData>();
    out->interval_id=count(field(value,"interval"));out->time_s=number(field(value,"time"));out->end_time_s=number(field(value,"end"));
    out->qpos=numbers(field(value,"qpos"),config.max_qpos);out->qvel=numbers(field(value,"qvel"),config.max_dofs);
    out->forces=components(field(value,"forces"),config.max_dofs);out->powers_w=object(field(value,"powers"));out->channels=object(field(value,"channels"));
    if(!bike::io::is_null(field(value,"constraints")))out->constraints=std::make_shared<const ConstraintSnapshot>(constraints(field(value,"constraints"),config.max_dofs));
    if(out->interval_id<0 || out->time_s<0. || out->end_time_s<=out->time_s ||
       std::abs(out->dt_s()-config.timestep_s)>1e-10)
        vread::invalid("checkpoint.sample","invalid interval identity");
    return out;
}
using SampleOwners=std::map<std::int64_t,SamplePtr>;
Value sample_ids(std::span<const SamplePtr> values,SampleOwners& owners) {
    Array out;out.reserve(values.size());
    for(const auto& row:values) {
        if(!row)throw std::logic_error("checkpoint.accounting: null committed sample");
        const auto [where,inserted]=owners.emplace(row->interval_id,row);
        if(!inserted&&where->second!=row&&
           bike::io::canonical_json(sample(where->second))!=bike::io::canonical_json(sample(row)))
            throw std::logic_error("checkpoint.accounting: conflicting sample owners");
        out.push_back(integer(row->interval_id));
    }
    return Value{std::move(out)};
}
SampleOwners sample_owners(const Value& value,const AccountingConfig& config,std::int64_t last) {
    SampleOwners out;std::int64_t previous=-1;
    for(const auto& item:bike::io::as_array(value,"checkpoint.samples")) {
        auto row=sample(item,config);
        if(!row || row->interval_id<=previous || row->interval_id>last)
            vread::invalid("checkpoint.samples","unordered or uncommitted interval");
        previous=row->interval_id;out.emplace(previous,std::move(row));
    }
    return out;
}
std::vector<SamplePtr> sample_refs(const Value& value,const SampleOwners& owners,std::set<std::int64_t>& used) {
    std::vector<SamplePtr> out;std::int64_t previous=-1;
    for(const auto& item:bike::io::as_array(value,"checkpoint.sample_ids")) {
        const auto id=count(item);const auto found=owners.find(id);
        if(id<=previous||found==owners.end())vread::invalid("checkpoint.sample_ids","unordered or missing sample owner");
        previous=id;used.insert(id);out.push_back(found->second);
    }
    return out;
}
} // namespace

bike::io::Value PeriodAccounting::checkpoint_state() const {
    if(async_ || !detached_.empty())
        throw std::invalid_argument("spatial checkpoint requires synchronous accounting; pending workers cannot be serialized");
    Array buffered,violations;
    for(const auto& row:buffer_.raws())buffered.push_back(raw(row));
    for(const auto& violation:state_.period_violations)violations.push_back(Value{Object{
        {"time_s",real(violation.time_s)},{"violations",strings(violation.violations)}}});
    SampleOwners owners;
    auto pending=sample_ids(pending_,owners),retained=sample_ids(retained_,owners);
    if(latest_)(void)sample_ids(std::span<const SamplePtr>(&latest_,1),owners);
    Array rows;rows.reserve(owners.size());for(const auto& [id,row]:owners){(void)id;rows.push_back(sample(row));}
    return Value{Object{{"schema",integer(1)}, {"energy_channels",wire(Wire{state_.energy_channels})},
        {"period_violations",Value{std::move(violations)}},
        {"endpoint_mass",state_.last_endpoint_mass?wire(Wire{*state_.last_endpoint_mass}):Value{}},
        {"measurement_source",state_.last_measurement_source?Value{Object{
            {"interval",integer(state_.last_measurement_source->interval_id)},
            {"time",real(state_.last_measurement_source->time_s)}}}:Value{}},
        {"buffered",Value{std::move(buffered)}}, {"samples",Value{std::move(rows)}},
        {"pending_ids",std::move(pending)}, {"retained_ids",std::move(retained)},
        {"latest_id",latest_?integer(latest_->interval_id):Value{}}, {"diagnostics",diagnostics(last_diagnostics_)}}};
}

void PeriodAccounting::restore_checkpoint_state(const bike::io::Value& value) {
    if(async_ || !detached_.empty())vread::invalid("checkpoint.accounting","synchronous owner required");
    exact(value,{"schema","energy_channels","period_violations","endpoint_mass","measurement_source",
        "buffered","samples","pending_ids","retained_ids","latest_id","diagnostics"});
    if(count(field(value,"schema"))!=1)vread::invalid("checkpoint.accounting.schema","unsupported version");
    // Bootstrap runtime owns energy, history, status, monitor and generation.
    // The candidate constructor has already validated and restored them.
    AccountingState staged=state_;
    staged.energy_channels=object(field(value,"energy_channels"));
    staged.period_violations.clear();
    for(const auto& item:bike::io::as_array(field(value,"period_violations"),"checkpoint.violations")) {
        exact(item,{"time_s","violations"});
        staged.period_violations.push_back(ReferenceFailure{.time_s=number(field(item,"time_s")),.violations=strings(field(item,"violations"))});
    }
    staged.last_endpoint_mass.reset();staged.last_measurement_source.reset();
    if(!bike::io::is_null(field(value,"endpoint_mass")))staged.last_endpoint_mass=object(field(value,"endpoint_mass"));
    if(!bike::io::is_null(field(value,"measurement_source"))) {
        const auto& m=field(value,"measurement_source");exact(m,{"interval","time"});
        staged.last_measurement_source=MeasurementSource{.interval_id=count(field(m,"interval")),.time_s=number(field(m,"time"))};
    }
    const auto last=staged.history.last_id.value_or(-1);
    if(last==std::numeric_limits<std::int64_t>::max() || staged.model_status.last_interval!=last)
        vread::invalid("checkpoint.accounting.history","inconsistent committed interval markers");
    if(staged.last_measurement_source && (staged.last_measurement_source->interval_id<0 ||
        staged.last_measurement_source->interval_id>last || staged.last_measurement_source->time_s<0.))
        vread::invalid("checkpoint.accounting.measurement_source","uncommitted measurement");
    PeriodBuffer buffer(config_.period_steps,last+1,staged.history.last_end);
    for(const auto& item:bike::io::as_array(field(value,"buffered"),"checkpoint.buffered"))buffer.push(raw(item,config_));
    const auto owners=sample_owners(field(value,"samples"),config_,last);
    std::set<std::int64_t> used;
    auto pending=sample_refs(field(value,"pending_ids"),owners,used);
    auto retained=sample_refs(field(value,"retained_ids"),owners,used);
    SamplePtr latest;
    if(!bike::io::is_null(field(value,"latest_id"))) {
        const auto id=count(field(value,"latest_id"));const auto found=owners.find(id);
        if(found==owners.end())vread::invalid("checkpoint.latest_id","missing sample owner");
        used.insert(id);latest=found->second;
    }
    if(used.size()!=owners.size())vread::invalid("checkpoint.samples","unreferenced sample owner");
    auto saved_diagnostics=diagnostics(field(value,"diagnostics"));
    if((last>=0)!=static_cast<bool>(latest) || (latest && latest->interval_id!=last))
        vread::invalid("checkpoint.accounting.latest","does not name the committed endpoint");
    // All validation and owning allocations finish above. No live owner is
    // changed by a malformed final row or a failed allocation.
    state_=std::move(staged);buffer_=std::move(buffer);
    pending_.swap(pending);retained_.swap(retained);latest_.swap(latest);
    last_diagnostics_=std::move(saved_diagnostics);
    detached_.clear();detached_at_=0;
    // Accounted rider/contact diagnostics are already owned by their
    // bootstrap snapshots. last_diagnostics_ may be a moved-from publication
    // envelope; republishing it here would erase the restored diagnostics.
}
} // namespace runtime
