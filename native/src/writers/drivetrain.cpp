#include "drivetrain.hpp"
#include "../engaged.hpp"
#include "../engine_call.hpp"
#include "../model_topology.hpp"
#include "pyfloat.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>
#include <ranges>

namespace drivetrain {
    ArithmeticError::~ArithmeticError() = default;

    namespace {
        // The member-init list consumes transmission_model/drive_mode before
        // the ctor body could run, so the typed config is validated in the
        // init expression itself — still before any field is used.
        DriveConfig checked_config(DriveConfig config) {
            validate(config);
            return config;
        }
    }

    void validate(const DriveSnapshot &s, const DriveConfig &config,
                  const mjModel &model) {
        const bool simplified = config.transmission_model != "elastic_chain",
                effort = config.drive_mode == "crank_effort" ||
                         config.drive_mode == "articulated_effort";
        // Topology is selected by the config, not discovered from the snapshot.
        if (s.hub.has_value() == simplified)
            throw std::invalid_argument("hub topology");
        if (s.ideal_hub.has_value() != (simplified && effort) ||
            s.clutch.has_value() != (simplified && effort && config.motor_clutch) ||
            s.freewheel.has_value() != (simplified && effort && config.rotor_inertia_kgm2 > 0.))
            throw std::invalid_argument("transmission topology");
        // A pending actuation exists only after a compute that also set the clock.
        if (s.pending_actuation && !s.last_time_s)
            throw std::invalid_argument("pending actuation without a live interval");
        // Shift counters, direction, cooldowns and the shift clock are set
        // together on a shift and cleared together on reset.
        const bool shifted = s.shifting.shift_count > 0;
        if (s.shift_time_s.has_value() != shifted ||
            (s.shifting.direction != "none") != shifted ||
            (!shifted && (s.shifting.cooldown_s != 0. || s.shifting.cut_remaining_s != 0.)))
            throw std::invalid_argument("shift clock/count mismatch");
        const auto &cassette = config.policies.shifting.cassette;
        if (config.policies.shifting.enabled &&
            (!std::ranges::contains(cassette, s.shifting.rear_teeth) ||
             !std::ranges::contains(cassette, s.shifting.from_teeth)))
            throw std::invalid_argument("shifter teeth outside the cassette");
        // The store is sized by the config and only ever drains.
        if (s.battery.initial_energy_j != config.policies.battery.energy_j ||
            s.battery.energy_j > s.battery.initial_energy_j ||
            s.battery.drawn_energy_j > s.battery.initial_energy_j)
            throw std::invalid_argument("battery energy outside the configured store");
        // Auxiliary transmissions are fixed 1:1 one-way clutches.
        for (const auto *aux: {s.clutch ? &*s.clutch : nullptr,
                               s.freewheel ? &*s.freewheel : nullptr})
            if (aux && (aux->ratio != 1. || aux->rear_teeth != 3 ||
                        aux->coefficients.size() != 1 || aux->coefficients[0] != 1.))
                throw std::invalid_argument("auxiliary transmission topology");
        if (!s.ideal_hub)
            return;
        const TransmissionSnapshot &hub = *s.ideal_hub;
        // The ideal-hub ratio is always the shifter's current gear.
        if (hub.ratio != static_cast<double>(config.policies.gearing.front_teeth) /
                         s.shifting.rear_teeth)
            throw std::invalid_argument("transmission ratio/gear mismatch");
        if (config.transmission_model == "geometric_ideal_mid_drive") {
            if (hub.rear_teeth != s.shifting.rear_teeth)
                throw std::invalid_argument("transmission rear-teeth mismatch");
            if (hub.coefficients.size() != static_cast<std::size_t>(model.nv))
                throw std::invalid_argument("transmission coefficient width");
            // A prepared snapshot carries the linearized Jacobian that the
            // live wrap coefficients were synced from.
            if (hub.prepared && hub.coefficients != hub.prepared->jacobian)
                throw std::invalid_argument("transmission coefficients/prepared Jacobian mismatch");
        } else if (hub.rear_teeth != config.policies.gearing.rear_teeth) {
            throw std::invalid_argument("transmission rear-teeth mismatch");
        }
    }

    DrivetrainWriter::DrivetrainWriter(mjModel *m, mjData *d, DriveConfig c)
        : model_(m), data_(d), config_(checked_config(std::move(c))),
          simplified_(config_.transmission_model != "elastic_chain"),
          effort_(config_.drive_mode == "crank_effort" ||
                  config_.drive_mode == "articulated_effort"),
          geometric_hub_(config_.transmission_model == "geometric_ideal_mid_drive"),
          geometry_(m->nv), frame_(resolve(m, mjOBJ_BODY, "frame")),
          crank_(resolve(m, mjOBJ_BODY, "crank")),
          rear_(resolve(m, mjOBJ_BODY, "rear_wheel")),
          human_actuator_(mj_name2id(m, mjOBJ_ACTUATOR, "human_crank")),
          motor_actuator_(mj_name2id(m, mjOBJ_ACTUATOR, "mid_drive")),
          pedaling_(config_.policies.pedaling),
          shifting_(config_.policies.gearing, config_.policies.shifting),
          assist_(config_.policies.assist), battery_(config_.policies.battery.energy_j),
          transmission_(static_cast<std::size_t>(m->nv)),
          pedaling_scratch_(config_.policies.pedaling),
          shifting_scratch_(config_.policies.gearing, config_.policies.shifting),
          assist_scratch_(config_.policies.assist),
          battery_scratch_(config_.policies.battery.energy_j) {
        if (m->nq != m->nv)
            throw std::invalid_argument(
                "physical chain supports scalar planar coordinates only (nq == nv)");
        for (const char *name: {
                 "crank_spin", "rear_wheel_spin", "front_wheel_spin",
                 "pedal_front_spin", "pedal_rear_spin"
             }) {
            const int id = resolve(m, mjOBJ_JOINT, name);
            topology::joint(m, id, name, mjJNT_HINGE, {0., 1., 0.});
            Joint const j{
                .qpos = buffer(m->jnt_qposadr, m->njnt)[static_cast<std::size_t>(id)],
                .dof = buffer(m->jnt_dofadr, m->njnt)[static_cast<std::size_t>(id)]
            };
            joints_.emplace(name, j);
            bearing_joints_.push_back(j);
        }
        for (const char *name: {"cassette_spin", "drive_shaft_spin", "rotor_spin"}) {
            const std::string key(name);
            if ((key == "cassette_spin" && !simplified_) ||
                (key == "drive_shaft_spin" && config_.motor_clutch) ||
                (key == "rotor_spin" && config_.rotor_inertia_kgm2 > 0.)) {
                const int id = resolve(m, mjOBJ_JOINT, name);
                topology::joint(m, id, name, mjJNT_HINGE, {0., 1., 0.});
                Joint const j{
                    .qpos = buffer(m->jnt_qposadr, m->njnt)[static_cast<std::size_t>(id)],
                    .dof = buffer(m->jnt_dofadr, m->njnt)[static_cast<std::size_t>(id)]
                };
                joints_.emplace(name, j);
                bearing_joints_.push_back(j);
            }
        }
        if (config_.drive_mode == "articulated_effort" && human_actuator_ >= 0)
            throw std::invalid_argument(
                "articulated effort must not have a human_crank actuator");
        if (effort_ && motor_actuator_ < 0)
            throw std::invalid_argument("physical effort needs a mid_drive actuator");
        if (human_actuator_ >= 0)
            topology::actuator(m, human_actuator_, "human_crank", resolve(m, mjOBJ_JOINT, "crank_spin"));
        if (motor_actuator_ >= 0) {
            const char *target = config_.rotor_inertia_kgm2 > 0. ? "rotor_spin" :
                                 config_.motor_clutch ? "drive_shaft_spin" : "crank_spin";
            topology::actuator(m, motor_actuator_, "mid_drive", resolve(m, mjOBJ_JOINT, target));
        }
        if (!simplified_) {
            cassette_ = resolve(m, mjOBJ_BODY, "cassette");
            const auto parents = buffer(m->body_parentid, m->nbody);
            if (parents[static_cast<std::size_t>(cassette_)] !=
                parents[static_cast<std::size_t>(rear_)])
                throw std::invalid_argument(
                    "cassette and wheel must have the same carrier body");
            hub_.emplace(config_.policies.hub_stiffness_nm_rad,
                         config_.policies.hub_damping_nm_s);
        }
        if (simplified_ && effort_) {
            const bool geometric =
                    config_.transmission_model == "geometric_ideal_mid_drive";
            ideal_hub_ = std::make_unique<Transmission>(
                m, config_.policies.gearing, geometric,
                geometric ? "geometric_mid_drive_freehub" : "ideal_mid_drive_freehub",
                config_.motor_clutch ? "drive_shaft_spin" : "crank_spin", "rear_wheel_spin",
                config_.motor_clutch ? "drive_shaft" : "crank");
            if (config_.motor_clutch)
                clutch_ = std::make_unique<Transmission>(
                    m, GearingConfig{.front_teeth = 3, .rear_teeth = 3, .chain_pitch_m = .0127},
                    false, "crank_clutch",
                    "crank_spin", "drive_shaft_spin");
            if (config_.rotor_inertia_kgm2 > 0.)
                freewheel_ = std::make_unique<Transmission>(
                    m, GearingConfig{.front_teeth = 3, .rear_teeth = 3, .chain_pitch_m = .0127},
                    false, "motor_freewheel",
                    "rotor_spin", "crank_spin");
        }
        // Rows are emitted in the declared force-component order — the
        // table position IS the serialized slot (pinned by the
        // force_component_table_ordered static_assert in the header).
        for (const ForceComponentSpec &spec: force_component_specs)
            if (spec.slot != ForceComponent::ideal_transmission || simplified_)
                components_.emplace_back(std::string(spec.name),
                                         std::vector<double>(transmission_.size()));
        // TickBank::components are construction-sized by contract (see the
        // struct comment): a fresh writer may go straight to restore()
        // without a reset(), and restore() reseeds no bank extents, so the
        // rows/views/names must exist before the first stage call. reset()
        // repeats this same seeding idempotently.
        for (auto &bank: banks_) {
            bank.component_count = components_.size();
            for (std::size_t i = 0; i < bank.components.size(); ++i) {
                bank.components[i].assign(transmission_.size(), 0.);
                bank.component_views[i] = ForceComponentView{
                    .kind = force_component_specs[i].kind,
                    .values = std::span<const double>(bank.components[i])};
                bank.component_names[i] = force_component_specs[i].name;
            }
        }
    }

    double DrivetrainWriter::angle(int body, std::optional<double> reference) {
        const double raw = geometry_.angle(model_, data_, body, false);
        return reference ? unwrap(raw, *reference) : raw;
    }

    void DrivetrainWriter::shifts(DriveTelemetry &d, const ShiftingSnapshot &s,
                                  std::optional<double> shift_time_s) const {
        // Lane writes mirror the former map assignments one-to-one; the
        // wire order comes from the schema's declared emission order, not
        // from this call sequence.
        d.set(TelemetryField::gear_front_teeth,
              static_cast<std::int32_t>(config_.policies.gearing.front_teeth));
        d.set(TelemetryField::gear_rear_teeth,
              static_cast<std::int32_t>(s.rear_teeth));
        d.set(TelemetryField::gear_ratio,
              static_cast<double>(config_.policies.gearing.front_teeth) /
                  s.rear_teeth);
        d.set(TelemetryField::shift_active, s.cut_remaining_s > 0.);
        d.set_label(TelemetryField::shift_direction,
                    engaged(telemetry_label(TelemetryField::shift_direction,
                                            s.direction)));
        d.set(TelemetryField::shift_from_teeth,
              static_cast<std::int32_t>(s.from_teeth));
        d.set(TelemetryField::shift_count,
              static_cast<std::int32_t>(s.shift_count));
        // Presence 2 serializes the explicit None the map's monostate
        // carried — the key stays present either way.
        d.set_optional(TelemetryField::shift_time_s, shift_time_s);
        d.set(TelemetryField::shift_torque_factor,
              s.cut_remaining_s > 0. ? config_.policies.shifting.torque_factor
                                     : 1.);
    }

    // Post-commit snapshot view of a staged update: Transmission::state()
    // reads wrap_prm/tendon_range from the model, which commit() rewrites
    // from update — so the mirror is the staged state plus those rows.
    // The into-form copy-assigns every member so a bank-resident mirror
    // reuses its vectors and telemetry lanes.
    void DrivetrainWriter::committed_into(const TransmissionUpdate &update,
                                          TransmissionSnapshot &snapshot) {
        snapshot = update.state;
        snapshot.range = update.range;
        snapshot.coefficients = update.coefficients;
        if (update.prepared_valid) {
            // Engage-once: emplace() on an engaged optional would destroy
            // the slot's vectors, so a bank-resident mirror reuses them.
            PreparedTransmission &slot =
                    snapshot.prepared ? *snapshot.prepared
                                      : snapshot.prepared.emplace();
            slot = update.prepared;
        } else
            snapshot.prepared.reset();
    }

    void DrivetrainWriter::committed_into(
            const TransmissionUpdate &update,
            std::optional<TransmissionSnapshot> &out) {
        committed_into(update, out ? *out : out.emplace());
    }

    TransmissionSnapshot
    DrivetrainWriter::committed_snapshot(const TransmissionUpdate &update) {
        TransmissionSnapshot snapshot;
        committed_into(update, snapshot);
        return snapshot;
    }

    void DrivetrainWriter::reset() {
        mj_kinematics(model_, data_);
        // Stage: every allocation and every candidate lands before the
        // first live write, so a failure anywhere above the commit line
        // leaves the drivetrain exactly as found.
        // The candidate snapshot stages in member-owned storage and the
        // transmission updates in tick bank 0 — the 8192-byte frame
        // contract keeps the multi-kilobyte candidates off the stack.
        snapshot_scratch_ = live_;
        DriveSnapshot &next = snapshot_scratch_;
        next.shift_time_s.reset();
        next.pending_actuation.reset();
        next.last_time_s.reset();
        next.pedaling = PedalingSnapshot{};
        next.assist = AssistSnapshot{};
        next.battery = BatterySnapshot{
            .initial_energy_j = battery_.state().initial_energy_j,
            .energy_j = battery_.state().initial_energy_j,
            .drawn_energy_j = 0.};
        next.shifting = {};
        next.shifting.rear_teeth = next.shifting.from_teeth =
                config_.policies.gearing.rear_teeth;
        next.hub = hub_ ? std::optional<FreehubSnapshot>(FreehubSnapshot{})
                        : std::nullopt;
        detail::TickBank &staging = banks_.at(0);
        staging.ideal_hub_staged = staging.clutch_staged =
                staging.freewheel_staged = false;
        if (simplified_) {
            next.angles = DriveAngles{.values = {angle(crank_), 0.},
                                      .count = 1};
            next.reference = 0.;
            next.psi.reset();
            if (ideal_hub_) {
                // Live order was set_ratio(configured gearing) then reset();
                // staging composes both candidates into one update.
                const double ratio =
                        static_cast<double>(config_.policies.gearing.front_teeth) /
                        config_.policies.gearing.rear_teeth;
                TransmissionUpdate &update =
                        staging.ideal_hub ? *staging.ideal_hub
                                          : staging.ideal_hub.emplace();
                if (geometric_hub_) {
                    // reset() seeded at the current gear, then the ratio
                    // staging composes on top — pending stays set inside the
                    // composed update, but the published reset state clears
                    // it, exactly like the sequential live order did.
                    ideal_hub_->stage_reset_into(data_, update);
                    ideal_hub_->stage_ratio_into(data_, ratio, update);
                    update.state.shift_pending = false;
                } else {
                    ideal_hub_->make_update_into(update);
                    ideal_hub_->stage_ratio_into(data_, ratio, update);
                    // reset() publishes the boundary at the new ratio — the
                    // direct relative() value, not the composed delta.
                    const auto q = buffer(data_->qpos, model_->nq);
                    const auto driver = joints_.at(config_.motor_clutch
                                                           ? "drive_shaft_spin"
                                                           : "crank_spin"),
                            driven = joints_.at("rear_wheel_spin");
                    update.state.boundary = validation::finite(
                        ratio * q[static_cast<std::size_t>(driver.qpos)] -
                            q[static_cast<std::size_t>(driven.qpos)],
                        "transmission boundary");
                    update.range[1] = engaged(update.state.boundary);
                    update.state.range = update.range;
                }
                staging.ideal_hub_staged = true;
                committed_into(update, next.ideal_hub);
            }
            if (clutch_) {
                clutch_->stage_reset_into(
                    data_, staging.clutch ? *staging.clutch
                                          : staging.clutch.emplace());
                staging.clutch_staged = true;
                committed_into(*staging.clutch, next.clutch);
            }
            if (freewheel_) {
                freewheel_->stage_reset_into(
                    data_, staging.freewheel ? *staging.freewheel
                                             : staging.freewheel.emplace());
                staging.freewheel_staged = true;
                committed_into(*staging.freewheel, next.freewheel);
            }
        } else {
            next.angles = DriveAngles{
                .values = {angle(crank_), angle(cassette_body())},
                .count = 2};
            next.reference = geometry_.evaluate(
                model_, data_, config_.policies.gearing, crank_, cassette_body(),
                frame_,
                Vec2{next.angles->values[0], next.angles->values[1]},
                std::nullopt, false);
            next.psi = geometry_.psi;
        }
        // Lane-equivalent of the former seed map — only these fields and the
        // shift block publish on reset, matching the sparse wire dict. The
        // lanes write into next.last in place (the copy-assign above left
        // the previous lanes engaged — clear() restores the sparse shape).
        DriveTelemetry &last = next.last;
        last.clear();
        last.set(TelemetryField::chain_energy_j, 0.);
        last.set(TelemetryField::freehub_energy_j, 0.);
        last.set(TelemetryField::motor_torque_nm, 0.);
        last.set(TelemetryField::human_torque_nm, 0.);
        last.set(TelemetryField::electrical_power_w, 0.);
        last.set(TelemetryField::freehub_torque_nm, 0.);
        last.set(TelemetryField::motor_freewheel_engaged, false);
        last.set(TelemetryField::motor_freewheel_torque_nm, 0.);
        last.set(TelemetryField::motor_freewheel_dissipation_power_w, 0.);
        shifts(last, next.shifting, next.shift_time_s);
        // Open-label lanes are the only telemetry storage a marked tick
        // can grow: set_open writes them every tick and the snapshot
        // copy-assigns carry them. Pre-reserve on the staged snapshot —
        // the commit swap hands this exact storage to live_ — and on
        // every bank below, so no lane ever reallocs inside the warm core.
        // coasting_reason's bound is its longest closed-domain label;
        // assist_mode's is the configured mode name.
        last.ensure_open_capacity(
                TelemetryField::coasting_reason,
                coast_reason_name(CoastReason::no_effort).size());
        last.ensure_open_capacity(
                TelemetryField::assist_mode,
                config_.policies.assist.mode.size());
        // Seed both banks and the shared policy scratch from the CANDIDATE
        // — still inside the stage section, so every grow lands before the
        // commit line and an injected failure publishes nothing. Seeding
        // from the staged updates (not the live transmissions) also gives
        // every slot the exact post-commit shape, including the geometric
        // prepared vectors. Bank 0's update slots ARE the staging slots —
        // skip them so the staged candidates survive to commit.
        const bool hub_staged = staging.ideal_hub_staged,
                clutch_staged = staging.clutch_staged,
                freewheel_staged = staging.freewheel_staged;
        for (auto &bank: banks_) {
            bank.snapshot = next;
            // Copy-assign preserves lane presence but not string
            // capacity — reserve the open lanes on the bank's own
            // telemetry so set_open/bank.snapshot=live_ never reallocs
            // inside a marked tick.
            bank.snapshot.last.ensure_open_capacity(
                    TelemetryField::coasting_reason,
                    coast_reason_name(CoastReason::no_effort).size());
            bank.snapshot.last.ensure_open_capacity(
                    TelemetryField::assist_mode,
                    config_.policies.assist.mode.size());
            bank.result = PedalingState{};
            bank.pedaling = next.pedaling;
            bank.shifting = next.shifting;
            bank.assist = next.assist;
            bank.battery = next.battery;
            bank.hub = next.hub;
            bank.ctrl_count = 0;
            bank.ideal_hub_staged = bank.clutch_staged =
                    bank.freewheel_staged = false;
            bank.component_count = components_.size();
            for (std::size_t i = 0; i < bank.components.size(); ++i) {
                bank.components[i].assign(transmission_.size(), 0.);
                bank.component_views[i] = ForceComponentView{
                    .kind = force_component_specs[i].kind,
                    .values = std::span<const double>(bank.components[i])};
                bank.component_names[i] = force_component_specs[i].name;
            }
            const auto seed_update =
                    [](Transmission *t,
                       const std::optional<TransmissionUpdate> &staged_slot,
                       const bool was_staged,
                       std::optional<TransmissionUpdate> &slot) {
                if (&slot == &staged_slot)
                    return;
                if (!t) {
                    slot.reset();
                    return;
                }
                TransmissionUpdate &u = slot ? *slot : slot.emplace();
                if (was_staged)
                    u = *staged_slot;
                else
                    t->make_update_into(u);
            };
            seed_update(ideal_hub_.get(), staging.ideal_hub, hub_staged,
                        bank.ideal_hub);
            seed_update(clutch_.get(), staging.clutch, clutch_staged,
                        bank.clutch);
            seed_update(freewheel_.get(), staging.freewheel, freewheel_staged,
                        bank.freewheel);
        }
        for (auto &bank: settlement_banks_) {
            bank.write_count = 0;
            bank.battery.reset();
            bank.assist.reset();
            bank.clear_pending = false;
            const auto seed_solve =
                    [](
                            Transmission *t,
                            const std::optional<TransmissionUpdate> &staged_slot,
                            const bool was_staged,
                            std::optional<SolvedTransmission> &slot) {
                if (!t) {
                    slot.reset();
                    return;
                }
                SolvedTransmission &s = slot ? *slot : slot.emplace();
                if (was_staged)
                    committed_into(*staged_slot, s.state);
                else
                    t->state_into(s.state);
                // The solve slot's post-commit shape never carries a
                // prepared mirror — commit(SolvedTransmission&) swaps the
                // live state in, and state_.prepared is disengaged by
                // invariant. Seeding it engaged (the committed_into/
                // state_into paths mirror the candidate) would only have
                // stage_solved_into's whole-snapshot assign destroy the
                // vectors on the first warm settle.
                s.state.prepared.reset();
                // s.force stays unbound: stage_solved_into() binds it to
                // the transmission's persistent force_ scratch each settle.
            };
            seed_solve(ideal_hub_.get(), staging.ideal_hub, hub_staged,
                       bank.ideal_solve);
            seed_solve(clutch_.get(), staging.clutch, clutch_staged,
                       bank.clutch_solve);
            seed_solve(freewheel_.get(), staging.freewheel, freewheel_staged,
                       bank.freewheel_solve);
        }
        // Commit: guarded model writes publish first (an engine failure
        // poisons the owning Stepper per the E1 contract), then the policy
        // resets and the snapshot swap — all memory-only, so nothing past
        // this line can allocate.
        if (hub_staged)
            ideal_hub_->commit(*staging.ideal_hub);
        if (clutch_staged)
            clutch_->commit(*staging.clutch);
        if (freewheel_staged)
            freewheel_->commit(*staging.freewheel);
        shifting_.reset();
        assist_.reset();
        battery_.reset();
        pedaling_.reset();
        if (hub_)
            engaged(hub_).reset();
        // Swap (not move): live_ gains the candidate's storage outright and
        // the scratch keeps the old live_ storage for the next stage — a
        // move would empty the scratch and the copy would allocate.
        std::swap(live_, snapshot_scratch_);
        // A publication: retire every outstanding staged handle.
        ++commit_epoch_;
        // Scalar policy scratch — copies of the post-reset policies, no
        // allocations involved.
        pedaling_scratch_ = pedaling_;
        shifting_scratch_ = shifting_;
        assist_scratch_ = assist_;
        battery_scratch_ = battery_;
        hub_scratch_ = hub_;
    }

    void DrivetrainWriter::restart_clock() {
        // Same publication-invalidation rule as a commit: an in-flight
        // stage's recorded epoch no longer matches the live clock state.
        ++commit_epoch_;
        live_.last_time_s.reset();
        assist_.reset();
        pedaling_.reset();
    }

    // NOLINTBEGIN(bugprone-easily-swappable-parameters) positional params mirror the Python API
    PedalingState DrivetrainWriter::prepare(const RideControl &control, double dt,
                                            bool braking, bool active, bool advance,
                                            bool contact, std::optional<double> slip,
                                            std::optional<double> ceiling) {
        // NOLINTEND(bugprone-easily-swappable-parameters)
        const auto tick = stage_prepare(
            {.control = control, .dt = dt, .braking = braking, .active = active,
             .advance = advance, .contact = contact, .slip = slip, .ceiling = ceiling});
        const auto result = tick.result;
        commit(tick);
        return result;
    }

    const ForceComponents &
    DrivetrainWriter::components(const RideControl &c, double dt, double speed,
                                 bool braking, bool active, bool advance, double sensed,
                                 const std::optional<PedalingState> &pedaling, bool contact,
                                 std::optional<double> slip) {
        const auto tick = stage_components(
            {.control = c, .dt = dt, .speed = speed, .sensed = sensed, .braking = braking,
             .active = active, .advance = advance, .contact = contact, .pedaling = pedaling,
             .slip = slip});
        commit(tick);
        return components_;
    }

    void DrivetrainWriter::components_into(const TickInputs &inputs,
                                           std::span<ForceComponentView> out) {
        if (out.size() != components_.size())
            throw std::invalid_argument("drivetrain component view count");
        const auto tick = stage_components(inputs);
        commit(tick);
        // Views bind the published rows — the same storage components()
        // returns — so they stay valid until the writer's next compute.
        for (std::size_t i = 0; i < components_.size(); ++i)
            out[i] = ForceComponentView{
                .kind = force_component_specs[i].kind,
                .values = components_[i].second};
    }

    // The warm-core twin: the tick's public gates are prechecked here so a
    // valid warm tick never constructs a throw — the double-advance clock
    // check mirrors tire.cpp's own precheck, the unsettled actuation maps
    // to its dedicated status, and the uninitialized-reference gate is the
    // geometry status. Residual kernel rejections (derived-value overflow,
    // engine failures inside a transmission commit) are caught and mapped
    // so noexcept is proved by the catch-all.
    CoreStatus DrivetrainWriter::try_components_into(
            const TickInputs &inputs,
            std::span<ForceComponentView> out) noexcept {
        if (out.size() != components_.size() || !std::isfinite(inputs.dt) ||
            inputs.dt <= 0. || !std::isfinite(inputs.speed) ||
            !std::isfinite(inputs.sensed) ||
            (inputs.advance && live_.last_time_s &&
             data_->time <= *live_.last_time_s))
            return CoreStatus::invalid_input;
        if (inputs.active && inputs.advance && live_.pending_actuation)
            return CoreStatus::pending_actuation;
        if (!live_.reference)
            return CoreStatus::invalid_geometry;
        try {
            components_into(inputs, out);
        } catch (const std::bad_alloc &) {
            return CoreStatus::engine_failure;
        } catch (const engine::EngineFailure &) {
            // EngineFailure derives std::exception — it must be caught
            // before the generic mapping, or an engine error inside a
            // transmission commit is misreported as bad input.
            return CoreStatus::engine_failure;
        } catch (const std::exception &) {
            return CoreStatus::invalid_input;
        } catch (...) {
            return CoreStatus::engine_failure;
        }
        return CoreStatus::ok;
    }

    // .at() provably cannot throw: the index flips between 0 and 1 over a
    // fixed array of two — NOLINT sits on the signature the check reports.
    detail::TickBank &DrivetrainWriter::next_tick_bank() // NOLINT(bugprone-exception-escape)
            noexcept {
        staged_bank_ = 1 - staged_bank_;
        auto &bank = banks_.at(staged_bank_);
        ++bank.generation;
        bank.staged_epoch = commit_epoch_;
        return bank;
    }

    // Same bounded-index guarantee as next_tick_bank.
    detail::SettlementBank &DrivetrainWriter::next_settlement_bank() // NOLINT(bugprone-exception-escape)
            noexcept {
        staged_settlement_ = 1 - staged_settlement_;
        auto &bank = settlement_banks_.at(staged_settlement_);
        ++bank.generation;
        bank.staged_epoch = commit_epoch_;
        return bank;
    }

    detail::TickBank &
    DrivetrainWriter::checked_tick(const PreparedTick &tick) {
        if (tick.owner_ != this || tick.bank_ != &banks_.at(staged_bank_) ||
            tick.bank_->generation != tick.generation_ ||
            tick.bank_->staged_epoch != commit_epoch_)
            throw std::logic_error("stale prepared tick");
        return *tick.bank_;
    }

    detail::SettlementBank &
    DrivetrainWriter::checked_settlement(PreparedSettlement &settlement) {
        if (settlement.owner_ != this ||
            settlement.bank_ != &settlement_banks_.at(staged_settlement_) ||
            settlement.bank_->generation != settlement.generation_ ||
            settlement.bank_->staged_epoch != commit_epoch_)
            throw std::logic_error("stale prepared settlement");
        return *settlement.bank_;
    }

    PreparedTick DrivetrainWriter::stage_prepare(const PrepareInputs &in) {
        if (in.advance && live_.last_time_s && data_->time <= *live_.last_time_s)
            throw std::invalid_argument(
                "drivetrain state can advance only once per timestamp");
        detail::TickBank &bank = next_tick_bank();
        bank.advance = in.advance;
        bank.probe = false;
        bank.ctrl_count = 0;
        bank.component_count = 0;
        bank.snapshot = live_;
        bank.ideal_hub_staged = bank.clutch_staged =
                bank.freewheel_staged = false;
        bank.shifting = shifting_.state();
        bank.assist = assist_.state();
        bank.battery = battery_.state();
        bank.hub = hub_ ? std::optional(hub_->state()) : std::nullopt;
        // The pedaling policy always advances on a detached candidate; only a
        // committed tick publishes it back.
        pedaling_scratch_ = pedaling_;
        stage_pedal_advance(bank, in, pedaling_scratch_);
        bank.pedaling = pedaling_scratch_.state();
        stage_snapshot_mirrors(bank);
        validate_tick(bank);
        return PreparedTick{*this, bank};
    }

    // The snapshot embeds the same mirrors state() would report after the
    // commit — candidate values where a stage produced one, current state
    // otherwise — so restore() round-trips the published snapshot verbatim
    // and commit() never allocates a fresh mirror. Bank-resident mirrors are
    // written in place: copy-assigns reuse the vectors and telemetry lanes.
    void DrivetrainWriter::stage_snapshot_mirrors(detail::TickBank &bank) {
        bank.snapshot.pedaling = bank.pedaling;
        bank.snapshot.shifting = bank.shifting;
        bank.snapshot.assist = bank.assist;
        bank.snapshot.battery = bank.battery;
        bank.snapshot.hub = bank.hub;
        const auto mirror = [](Transmission *transmission,
                               const std::optional<TransmissionUpdate> &update,
                               bool staged,
                               std::optional<TransmissionSnapshot> &out) {
            if (!transmission) {
                out.reset();
                return;
            }
            if (staged)
                committed_into(*update, out);
            else
                transmission->state_into(out ? *out : out.emplace());
        };
        mirror(ideal_hub_.get(), bank.ideal_hub, bank.ideal_hub_staged,
               bank.snapshot.ideal_hub);
        mirror(clutch_.get(), bank.clutch, bank.clutch_staged,
               bank.snapshot.clutch);
        mirror(freewheel_.get(), bank.freewheel, bank.freewheel_staged,
               bank.snapshot.freewheel);
    }

    // Every candidate is validated while staged so commit()'s publication is
    // genuinely memory-only: set_state() validation can never throw there.
    void DrivetrainWriter::validate_tick(detail::TickBank &bank) {
        pedaling_scratch_ = pedaling_;
        pedaling_scratch_.set_state(bank.pedaling);
        shifting_scratch_ = shifting_;
        shifting_scratch_.set_state(bank.shifting);
        assist_scratch_ = assist_;
        assist_scratch_.set_state(bank.assist);
        battery_scratch_ = battery_;
        battery_scratch_.set_state(bank.battery);
        if (hub_) {
            hub_scratch_ = hub_;
            engaged(hub_scratch_).set_state(engaged(bank.hub));
        }
    }

    void DrivetrainWriter::stage_pedal_advance(detail::TickBank &bank,
                                               const PrepareInputs &in,
                                               PedalingPolicy &pedaling) {
        const auto crank = joints_.at("crank_spin"), wheel = joints_.at("rear_wheel_spin");
        const auto q = buffer(data_->qpos, model_->nq), v = buffer(data_->qvel, model_->nv);
        double required = v[static_cast<std::size_t>(wheel.dof)] / shifting_.gear_ratio() *
                          60. / (2. * std::numbers::pi);
        const double effort = in.control.human_torque_nm.value_or(config_.human_torque_nm);
        const bool enabled = in.active && in.control.rider_enabled && effort_;
        double torque_factor = shifting_.torque_factor();
        if (in.advance && in.active && config_.policies.shifting.enabled && effort_) {
            if (!ideal_hub_)
                throw std::invalid_argument(
                    "automatic shifting needs the live ideal_mid_drive model");
            // The shifter and the transmission update stage together: a
            // rejected ratio discards the whole candidate — EMAs, counters,
            // gear — and the commit lands both in one move. When the tick
            // already carries a staged prepare, the ratio staging composes
            // onto it (sequential prepare→set_ratio order preserved).
            shifting_scratch_ = shifting_;
            const bool shifted = shifting_scratch_.update(
                v[static_cast<std::size_t>(crank.dof)] * 60. / (2. * std::numbers::pi),
                required, in.dt, enabled && effort > 0., in.braking, in.contact, in.slip);
            if (shifted) {
                // Compose onto the candidate already staged this tick or
                // seed the bank slot from the live transmission.
                if (bank.ideal_hub_staged) {
                    ideal_hub_->stage_ratio_into(
                        data_, shifting_scratch_.gear_ratio(),
                        engaged(bank.ideal_hub));
                } else {
                    // Engage-once: the slot keeps its vectors/lanes across
                    // ticks; emplace() would release them for no reason.
                    TransmissionUpdate &update =
                            bank.ideal_hub ? *bank.ideal_hub
                                           : bank.ideal_hub.emplace();
                    ideal_hub_->make_update_into(update);
                    ideal_hub_->stage_ratio_into(
                        data_, shifting_scratch_.gear_ratio(), update);
                    bank.ideal_hub_staged = true;
                }
                bank.snapshot.shift_time_s = data_->time;
            }
            bank.shifting = shifting_scratch_.state();
            required = v[static_cast<std::size_t>(wheel.dof)] / shifting_scratch_.gear_ratio() *
                       60. / (2. * std::numbers::pi);
            torque_factor = shifting_scratch_.torque_factor();
        }
        auto result = pedaling.update(q[static_cast<std::size_t>(crank.qpos)],
                                      v[static_cast<std::size_t>(crank.dof)], required,
                                      effort, in.dt, enabled, in.braking);
        if (in.ceiling)
            result.effort_nm = std::min(
                result.effort_nm,
                nonnegative(*in.ceiling, "automatic rider effort ceiling"));
        result.effort_nm *= torque_factor;
        bank.result = result;
    }

    PreparedTick DrivetrainWriter::stage_components(const TickInputs &in) {
        positive(in.dt, "drivetrain interval");
        finite(in.speed, "bike speed");
        finite(in.sensed, "measured pedal torque");
        if (!live_.reference)
            throw std::runtime_error("initialize the drivetrain before applying forces");
        if (in.advance && live_.last_time_s && data_->time <= *live_.last_time_s)
            throw std::invalid_argument(
                "drivetrain state can advance only once per timestamp");
        detail::TickBank &bank = next_tick_bank();
        bank.advance = in.advance;
        bank.probe = !in.advance;
        bank.ctrl_count = 0;
        bank.snapshot = live_;
        bank.ideal_hub_staged = bank.clutch_staged =
                bank.freewheel_staged = false;
        bank.pedaling = pedaling_.state();
        bank.shifting = shifting_.state();
        bank.assist = assist_.state();
        bank.battery = battery_.state();
        bank.hub = hub_ ? std::optional(hub_->state()) : std::nullopt;
        if (!in.advance) {
            // Probe evaluation runs on the detached candidate: pending
            // settlement and the interval clock are cleared only there.
            bank.snapshot.pending_actuation.reset();
            bank.snapshot.last_time_s.reset();
        }
        // An unsettled actuation rejects before any policy advance or model
        // staging — repeated rejected calls cannot creep EMAs or effort.
        if (in.active && bank.snapshot.pending_actuation)
            throw std::runtime_error("previous motor interval was not settled");
        if (in.advance) {
            if (ideal_hub_) {
                ideal_hub_->stage_prepare_into(
                    data_, bank.ideal_hub ? *bank.ideal_hub
                                          : bank.ideal_hub.emplace());
                bank.ideal_hub_staged = true;
            }
            if (clutch_) {
                clutch_->stage_prepare_into(
                    data_, bank.clutch ? *bank.clutch
                                       : bank.clutch.emplace());
                bank.clutch_staged = true;
            }
            if (freewheel_) {
                freewheel_->stage_prepare_into(
                    data_, bank.freewheel ? *bank.freewheel
                                          : bank.freewheel.emplace());
                bank.freewheel_staged = true;
            }
        }
        stage_components_telemetry(bank, in,
                                   stage_components_metrics(bank, in));
        return PreparedTick{*this, bank};
    }

    DrivetrainWriter::TickMetrics
    DrivetrainWriter::stage_components_metrics(detail::TickBank &bank,
                                               const TickInputs &in) {
        TickMetrics m;
        PedalingState &ps = m.ps;
        if (in.pedaling) {
            ps = *in.pedaling;
            bank.result = ps;
        } else {
            pedaling_scratch_ = pedaling_;
            stage_pedal_advance(bank,
                                {.control = in.control, .dt = in.dt, .braking = in.braking,
                                 .active = in.active, .advance = in.advance,
                                 .contact = in.contact, .slip = in.slip,
                                 .ceiling = std::nullopt},
                                pedaling_scratch_);
            bank.pedaling = pedaling_scratch_.state();
            ps = bank.result;
        }
        bank.component_count = components_.size();
        for (auto &component: bank.components)
            std::ranges::fill(component, 0.);
        const auto crank = joints_.at("crank_spin"), wheel = joints_.at("rear_wheel_spin");
        const auto q = buffer(data_->qpos, model_->nq), v = buffer(data_->qvel, model_->nv);
        if (!simplified_) {
            m.angles.count = 2;
            m.angles.values = {
                angle(crank_, engaged(bank.snapshot.angles).values[0]),
                angle(cassette_body(), engaged(bank.snapshot.angles).values[1])
            };
            m.extension =
                    geometry_.evaluate(model_, data_, config_.policies.gearing, crank_,
                                       cassette_body(), frame_,
                                       Vec2{m.angles.values[0], m.angles.values[1]},
                                       bank.snapshot.psi, false) -
                    engaged(live_.reference);
            m.psi = geometry_.psi;
            m.rate = dot(geometry_.jacobian, v);
            auto const t =
                    chain_tension(m.extension, m.rate, config_.chain_k_n_m,
                                  config_.chain_c_ns_m);
            m.tension = t.first;
            m.energy = t.second;
            for (std::size_t i = 0; i < transmission_.size(); ++i)
                bank.components[force_component_index(ForceComponent::chain)][i] = -m.tension * geometry_.jacobian[i];
            const auto cassette = joints_.at("cassette_spin");
            hub_scratch_ = hub_;
            m.torque =
                    engaged(hub_scratch_).update(q[static_cast<std::size_t>(cassette.qpos)],
                                        q[static_cast<std::size_t>(wheel.qpos)],
                                        v[static_cast<std::size_t>(cassette.dof)],
                                        v[static_cast<std::size_t>(wheel.dof)]);
            bank.hub = engaged(hub_scratch_).state();
            bank.components[force_component_index(ForceComponent::freehub)][static_cast<std::size_t>(wheel.dof)] = m.torque;
            bank.components[force_component_index(ForceComponent::freehub)][static_cast<std::size_t>(cassette.dof)] = -m.torque;
            m.relative_rate = v[static_cast<std::size_t>(cassette.dof)] -
                              v[static_cast<std::size_t>(wheel.dof)];
            m.deflection = std::max(q[static_cast<std::size_t>(cassette.qpos)] -
                                    q[static_cast<std::size_t>(wheel.qpos)] -
                                    engaged(engaged(bank.hub).boundary),
                                    0.);
        }
        for (auto const j: bearing_joints_)
            bank.components[force_component_index(ForceComponent::drive_bearings)][static_cast<std::size_t>(j.dof)] =
                    validation::derived(-config_.bearing_c_nms_rad * v[static_cast<std::size_t>(j.dof)], "DrivetrainWriter.bearing_force");
        m.omega_crank = v[static_cast<std::size_t>(crank.dof)];
        m.cadence = validation::derived(
                m.omega_crank * 60. / (2. * std::numbers::pi),
                "DrivetrainWriter.cadence");
        const auto shaft = joints_.contains("drive_shaft_spin")
                               ? joints_.at("drive_shaft_spin")
                               : joints_.contains("rotor_spin")
                                     ? joints_.at("rotor_spin")
                                     : crank;
        m.omega = v[static_cast<std::size_t>(shaft.dof)];
        const double rpm = validation::derived(
                m.omega * 60. / (2. * std::numbers::pi),
                "DrivetrainWriter.shaft_rpm");
        m.human =
                in.active && config_.drive_mode == "crank_effort"
                    ? human_crank_torque(ps.effort_nm, q[static_cast<std::size_t>(crank.qpos)],
                                         config_.torque_ripple)
                    : 0.;
        m.sensor = config_.drive_mode == "crank_effort" ? m.human : in.sensed;
        m.assist_sensor = ps.mode == PedalMode::pedaling ? m.sensor : 0.;
        if (in.active && effort_) {
            assist_scratch_ = assist_;
            m.request = assist_scratch_.step(m.assist_sensor, m.cadence, in.speed, in.braking,
                                    in.dt, in.control.motor_torque_nm, rpm);
            bank.assist = assist_scratch_.state();
        }
        const auto &b = config_.policies.battery;
        const double budget = bank.battery.energy_j / in.dt;
        m.safety = in.control.motor_limit_nm
                       ? std::min(m.request, *in.control.motor_limit_nm)
                       : m.request;
        m.delivered =
                b.enabled
                    ? limit_torque_by_energy(m.safety, m.omega, b.copper_w_per_nm2,
                                             b.speed_w_per_rad_s2, b.idle_w, budget)
                    : m.safety;
        m.enabled = in.active && m.delivered > 0. && !in.braking;
        if (!m.enabled)
            m.delivered = 0.;
        m.electrical = motor_electrical_power(
            m.delivered, m.omega, b.copper_w_per_nm2, b.speed_w_per_rad_s2,
            b.idle_w, m.enabled);
        if (b.enabled && m.electrical > budget + std::max(1e-10, std::abs(budget) * 1e-12))
            throw ArithmeticError("delivered motor torque exceeds the battery budget");
        return m;
    }

    void DrivetrainWriter::stage_components_telemetry(
            detail::TickBank &bank, const TickInputs &in, const TickMetrics &m) {
        for (std::size_t i = 0; i < bank.component_count; ++i)
            validation::derived_array(bank.components[i],
                                      "DrivetrainWriter.force");
        const double chain_loss = std::max(0., validation::derived(
            (m.tension - config_.chain_k_n_m * std::max(m.extension, 0.)) * m.rate, "DrivetrainWriter.chain_dissipation"));
        const double hub_loss = simplified_ ? 0. : std::max(0., validation::derived(
            (m.torque - config_.policies.hub_stiffness_nm_rad * m.deflection) * m.relative_rate, "DrivetrainWriter.freehub_dissipation"));
        bank.snapshot.pending_actuation =
                in.active
                    ? std::optional<PendingActuation>{
                        {.requested = m.delivered, .omega = m.omega, .dt = in.dt,
                         .enabled = m.enabled}
                    }
                    : std::nullopt;
        if (human_actuator_ >= 0)
            bank.ctrl[bank.ctrl_count++] = {human_actuator_, m.human};
        if (motor_actuator_ >= 0)
            bank.ctrl[bank.ctrl_count++] = {motor_actuator_, m.delivered};
        if (effort_)
            bank.assist.torque = m.delivered;
        // Lane-equivalent of the former map literal: clear() drops the
        // seeded lanes wholesale (the old assignment's replace semantics),
        // then every tick field writes in place — presence 2 on the
        // optional lanes serializes the explicit None the map's monostate
        // carried. set_open reuses the lane's string capacity, so steady
        // state allocates nothing.
        DriveTelemetry &last = bank.snapshot.last;
        last.clear();
        last.set_label(TelemetryField::transmission_model,
                       engaged(telemetry_label(
                           TelemetryField::transmission_model,
                           config_.transmission_model)));
        last.set(TelemetryField::omits_suspension_coupling,
                 config_.transmission_model == "ideal_mid_drive");
        last.set(TelemetryField::chain_extension_m, m.extension);
        last.set(TelemetryField::chain_extension_rate_mps, m.rate);
        last.set(TelemetryField::chain_tension_n, m.tension);
        last.set(TelemetryField::chain_energy_j, m.energy);
        last.set(TelemetryField::chain_dissipation_power_w, chain_loss);
        last.set(TelemetryField::freehub_torque_nm,
                 simplified_ ? 0. : m.torque);
        last.set(TelemetryField::freehub_energy_j,
                 simplified_ ? 0. : engaged(bank.hub).energy_j);
        last.set(TelemetryField::freehub_engaged,
                 simplified_ ? false : m.torque > 0.);
        last.set(TelemetryField::freehub_deflection_rad, m.deflection);
        last.set(TelemetryField::freehub_dissipation_power_w, hub_loss);
        last.set(TelemetryField::cadence_rpm, m.cadence);
        last.set(TelemetryField::crank_rad_s, m.omega_crank);
        last.set(TelemetryField::drive_shaft_rad_s, m.omega);
        last.set(TelemetryField::human_torque_nm, m.human);
        last.set(TelemetryField::human_sensor_nm, m.sensor);
        last.set_optional(TelemetryField::human_setpoint_nm,
                          in.control.human_torque_nm);
        last.set(TelemetryField::assist_demand_gated,
                 in.braking ||
                 m.assist_sensor <= config_.policies.assist.engage_torque_nm ||
                 m.omega_crank <=
                 config_.policies.assist.gate_min_crank_rad_s);
        last.set(TelemetryField::assist_sensor_nm, m.assist_sensor);
        last.set(TelemetryField::human_command_nm, m.ps.effort_nm);
        // The PedalMode ordinal doubles as the rider_mode label index
        // (pedaling.hpp pins the same ordering as rider_mode_labels).
        last.set_label(TelemetryField::rider_mode,
                       static_cast<std::uint8_t>(m.ps.mode));
        last.set_open(TelemetryField::coasting_reason,
                      coast_reason_name(m.ps.reason));
        last.set(TelemetryField::required_cadence_rpm,
                 m.ps.required_cadence_rpm);
        last.set_optional(TelemetryField::crank_target_phase_rad,
                          m.ps.target_phase_rad);
        last.set(TelemetryField::crank_target_rate_rad_s,
                 m.ps.target_rate_rad_s);
        last.set(TelemetryField::motor_request_nm, m.request);
        last.set(TelemetryField::motor_torque_nm, m.delivered);
        last.set(TelemetryField::motor_freewheel_engaged, m.delivered > 0.);
        last.set(TelemetryField::motor_freewheel_torque_nm, m.delivered);
        last.set(TelemetryField::motor_freewheel_dissipation_power_w, 0.);
        last.set(TelemetryField::motor_limited_request_nm, m.safety);
        last.set_optional(TelemetryField::motor_setpoint_nm,
                          in.control.motor_torque_nm);
        last.set_optional(TelemetryField::motor_limit_nm,
                          in.control.motor_limit_nm);
        last.set_label(TelemetryField::motor_control_source,
                       engaged(telemetry_label(
                           TelemetryField::motor_control_source,
                           in.control.motor_torque_nm ? "external_request"
                                                      : "assist")));
        last.set(TelemetryField::safety_limited, m.safety < m.request);
        last.set(TelemetryField::motor_shaft_power_w, m.delivered * m.omega);
        last.set(TelemetryField::electrical_power_w, 0.);
        last.set(TelemetryField::battery_energy_j, bank.battery.energy_j);
        last.set(TelemetryField::motor_enabled, m.enabled);
        last.set(TelemetryField::energy_limited, m.delivered < m.safety);
        last.set(TelemetryField::battery_empty, bank.battery.energy_j == 0.);
        last.set_open(TelemetryField::assist_mode,
                      config_.policies.assist.mode);
        last.set(TelemetryField::assist_gain, bank.assist.last_gain);
        shifts(last, bank.shifting, bank.snapshot.shift_time_s);
        if (simplified_)
            bank.snapshot.angles = DriveAngles{
                .values = {angle(crank_,
                                 engaged(bank.snapshot.angles).values[0]),
                           0.},
                .count = 1};
        else {
            bank.snapshot.angles = m.angles;
            bank.snapshot.psi = m.psi;
        }
        bank.snapshot.last_time_s = data_->time;
        if (bank.advance)
            stage_snapshot_mirrors(bank);
        validate_tick(bank);
    }

    void DrivetrainWriter::commit(const PreparedTick &tick) {
        detail::TickBank &bank = checked_tick(tick);
        if (bank.probe) {
            // Probe ticks publish only their declared telemetry: the force
            // rows the caller applies and the probe diagnostics channel.
            // Policy, clock, pending and model state stay untouched.
            for (std::size_t i = 0; i < bank.component_count; ++i)
                std::ranges::copy(bank.components[i],
                                  components_[i].second.begin());
            {
                const auto ctrl = buffer(data_->ctrl, model_->nu);
                for (std::size_t i = 0; i < bank.ctrl_count; ++i)
                    ctrl[static_cast<std::size_t>(bank.ctrl[i].first)] =
                            bank.ctrl[i].second;
            }
            // Copy-assign (not move): the bank's open-label lanes keep
            // their string capacity for the next stage.
            live_.probe_last = bank.snapshot.last;
            ++commit_epoch_;
            return;
        }
        if (!bank.advance)
            return;
        // Guarded live-model writes first (E1 boundary inside each
        // Transmission::commit): an engine failure propagates and poisons the
        // owning Stepper before any logical publication. The *_staged flags
        // mark the updates this tick refreshed; they exist only when the
        // transmission does — staging adds them under the topology guard.
        if (bank.ideal_hub_staged)
            ideal_hub_->commit(engaged(bank.ideal_hub));
        if (bank.clutch_staged)
            clutch_->commit(engaged(bank.clutch));
        if (bank.freewheel_staged)
            freewheel_->commit(engaged(bank.freewheel));
        // Memory-only publication: every candidate was fully evaluated and
        // validated during staging, so the writes below are copies into
        // preallocated extents.
        {
            const auto ctrl = buffer(data_->ctrl, model_->nu);
            for (std::size_t i = 0; i < bank.ctrl_count; ++i)
                ctrl[static_cast<std::size_t>(bank.ctrl[i].first)] =
                        bank.ctrl[i].second;
        }
        for (std::size_t i = 0; i < bank.component_count; ++i)
            std::ranges::copy(bank.components[i],
                              components_[i].second.begin());
        pedaling_.set_state(bank.pedaling);
        shifting_.set_state(bank.shifting);
        assist_.set_state(bank.assist);
        battery_.set_state(bank.battery);
        if (hub_)
            engaged(hub_).set_state(engaged(bank.hub));
        // The snapshot's embedded policy/transmission mirrors were staged
        // with the committed values — copy-assign (not move) keeps every
        // vector and telemetry lane in the bank at capacity for the next
        // stage.
        live_ = bank.snapshot;
        ++commit_epoch_;
    }

    void DrivetrainWriter::validate_settlement(
        const detail::SettlementBank &bank) {
        if (bank.battery) {
            battery_scratch_ = battery_;
            battery_scratch_.set_state(*bank.battery);
        }
        if (bank.assist) {
            assist_scratch_ = assist_;
            assist_scratch_.set_state(*bank.assist);
        }
    }

    PreparedSettlement DrivetrainWriter::stage_settle() {
        detail::SettlementBank &bank = next_settlement_bank();
        bank.write_count = 0;
        bank.battery.reset();
        bank.assist.reset();
        bank.clear_pending = false;
        PreparedSettlement settlement;
        settlement.owner_ = this;
        settlement.bank_ = &bank;
        settlement.generation_ = bank.generation;
        // The force publication is a view over the persistent transmission_
        // scratch — the binding boxes it before commit(), and the settle()
        // convenience wrapper returns the same span it always did.
        settlement.force = transmission_;

        // Phase 1 — pending validation and detached policy candidates. Every
        // rejection lands before the solve below, so a refused settlement
        // publishes nothing — not even the transmissions' solve telemetry.
        const PendingActuation *pending = live_.pending_actuation
                                              ? &*live_.pending_actuation
                                              : nullptr;
        double pending_torque = 0., pending_delivered = 0.;
        if (pending) {
            pending_torque =
                    motor_actuator_ >= 0
                        ? buffer(data_->actuator_force,
                                 model_->nu)[static_cast<std::size_t>(motor_actuator_)]
                        : 0.;
            if (pending_torque < -1e-10 ||
                pending_torque > pending->requested + 1e-8)
                throw ArithmeticError(
                    "solved motor effort violates the reserved effort ceiling");
            pending_torque = std::max(pending_torque, 0.);
            const auto &b = config_.policies.battery;
            const double power = motor_electrical_power(
                pending_torque, pending->omega, b.copper_w_per_nm2,
                b.speed_w_per_rad_s2, b.idle_w,
                pending->enabled && pending_torque > 0.);
            pending_delivered = power;
            if (b.enabled) {
                const double store = battery_.state().energy_j;
                if (power * pending->dt >
                    store + std::max(1e-10, store * 1e-12))
                    throw ArithmeticError(
                        "solved motor energy exceeds available battery storage");
                const auto [delivered, candidate] =
                        battery_.debit(power, pending->dt);
                pending_delivered = delivered;
                bank.battery = candidate;
            }
            AssistSnapshot assist = assist_.state();
            assist.torque = pending_torque;
            bank.assist = assist;
            bank.clear_pending = true;
        }
        validate_settlement(bank);

        // Phase 2 — the telemetry write plan. Every settlement write is a
        // (TelemetryField, scalar) pair: lanes always exist on
        // live_.last, so unlike the former map-slot plan there is no
        // detached-map fallback — a restored sparse `last` still resolves
        // every declared field, and the geometric merge emits the same
        // fixed lanes stage_solved_into populated. Scalars are stored by
        // value; the only non-POD alternative (string_view for open
        // lanes) never flows through this plan because the transmissions'
        // solve diagnostics contain real/label fields only.
        const auto record = [&bank](TelemetryField field,
                                    const TelemetryScalar &value) {
            if (bank.write_count == bank.writes.size())
                throw std::logic_error("settlement write plan overflow");
            bank.writes[bank.write_count++] =
                    detail::SettlementBank::Write{.field = field,
                                                  .value = value};
        };

        // Phase 3 — the solve. Each transmission stages its own candidate:
        // the reaction force lands in the persistent transmission_ scratch
        // now, but the solve's telemetry/state publication rides the
        // settlement's commit — a throw here (an unprepared geometric hub)
        // leaves every transmission and the pending reservation
        // byte-identical.
        std::ranges::fill(transmission_, 0.);
        if (ideal_hub_) {
            // Engage-once like the tick slots: restore() seeds no bank
            // extents, so a fresh writer's first settle must materialize the
            // slot; stage_solved_into() overwrites the state wholesale.
            ideal_hub_->stage_solved_into(
                data_, bank.ideal_solve ? *bank.ideal_solve
                                        : bank.ideal_solve.emplace());
            std::ranges::copy(engaged(bank.ideal_solve).force,
                              transmission_.begin());
        }
        struct Aux {
            Transmission *transmission;
            std::optional<SolvedTransmission> *slot;
            TelemetryField torque, engaged, dissipation;
        };
        for (const Aux &aux: {
                 Aux{.transmission = clutch_.get(),
                     .slot = &bank.clutch_solve,
                     .torque = TelemetryField::crank_clutch_torque_nm,
                     .engaged = TelemetryField::crank_clutch_engaged,
                     .dissipation =
                         TelemetryField::crank_clutch_dissipation_power_w},
                 Aux{.transmission = freewheel_.get(),
                     .slot = &bank.freewheel_solve,
                     .torque = TelemetryField::motor_freewheel_torque_nm,
                     .engaged = TelemetryField::motor_freewheel_engaged,
                     .dissipation =
                         TelemetryField::motor_freewheel_dissipation_power_w}})
            if (aux.transmission) {
                SolvedTransmission &solved =
                        *aux.slot ? **aux.slot : aux.slot->emplace();
                aux.transmission->stage_solved_into(data_, solved);
                const std::span<const double> f = solved.force;
                for (std::size_t i = 0; i < transmission_.size(); ++i)
                    transmission_[i] += f[i];
                const double torque =
                        f[static_cast<std::size_t>(aux.transmission->driven_dof())];
                record(aux.torque, torque);
                record(aux.engaged, torque > 1e-8);
                record(aux.dissipation,
                       std::max(0., torque * aux.transmission->relative_rate(data_)));
            }
        if (ideal_hub_) {
            const double torque =
                    transmission_[static_cast<std::size_t>(ideal_hub_->driven_dof())];
            record(TelemetryField::freehub_torque_nm, torque);
            record(TelemetryField::freehub_engaged, torque > 1e-8);
            if (config_.motor_clutch)
                record(TelemetryField::freehub_dissipation_power_w,
                       std::max(0., torque * ideal_hub_->relative_rate(data_)));
            // Merge from the staged candidate — not the live diagnostics,
            // which publish only at commit. Every merged field is a
            // declared lane, so a duplicate key simply records twice and
            // apply() lands the later value — the former map assignment's
            // last-wins semantics.
            engaged(bank.ideal_solve).state.diagnostics.for_each(
                [&record](TelemetryField field,
                          const TelemetryScalar &value) {
                    record(field, value);
                });
        }
        if (pending) {
            const double energy = bank.battery
                                      ? bank.battery->energy_j
                                      : battery_.state().energy_j;
            record(TelemetryField::motor_torque_nm, pending_torque);
            record(TelemetryField::motor_shaft_power_w,
                   pending_torque * pending->omega);
            record(TelemetryField::electrical_power_w, pending_delivered);
            record(TelemetryField::battery_energy_j, energy);
            record(TelemetryField::motor_enabled,
                   pending->enabled && pending_torque > 0.);
            record(TelemetryField::battery_empty, energy == 0.);
        }
        return settlement;
    }

    // A stale or foreign handle terminates here — checked_settlement throws
    // std::logic_error inside this noexcept commit, per the declared
    // precondition; NOLINT sits on the signature the check reports.
    void DrivetrainWriter::commit(PreparedSettlement &settlement) // NOLINT(bugprone-exception-escape)
            noexcept {
        // A stale/foreign handle is a programming error — termination
        // under the noexcept contract is the declared behavior.
        // cppcheck-suppress throwInNoexceptFunction
        detail::SettlementBank &bank = checked_settlement(settlement);
        // The transmission solve candidates publish through their own
        // memory-only swap first; every store below is then provably
        // allocation-free: apply() writes POD lanes in place and the
        // staged scalars were plain values — no string ever travels the
        // settlement plan (the geometric status is a closed label).
        if (bank.ideal_solve)
            ideal_hub_->commit(*bank.ideal_solve);
        if (bank.clutch_solve)
            clutch_->commit(*bank.clutch_solve);
        if (bank.freewheel_solve)
            freewheel_->commit(*bank.freewheel_solve);
        for (std::size_t i = 0; i < bank.write_count; ++i)
            live_.last.apply(bank.writes[i].field, bank.writes[i].value);
        if (bank.battery)
            battery_.publish(*bank.battery);
        if (bank.assist)
            assist_.publish(*bank.assist);
        if (bank.clear_pending)
            live_.pending_actuation.reset();
        ++commit_epoch_;
    }

    std::span<const double> DrivetrainWriter::settle() {
        auto settlement = stage_settle();
        const std::span<const double> force = settlement.force;
        commit(settlement);
        return force;
    }

    const DriveTelemetry &
    DrivetrainWriter::diagnostics(bool probe) const noexcept {
        static const DriveTelemetry empty;
        return probe ? (live_.probe_last ? *live_.probe_last : empty)
                     : live_.last;
    }

    std::optional<int>
    DrivetrainWriter::body_id(std::string_view name) const noexcept {
        if (name == "frame") return frame_;
        if (name == "crank") return crank_;
        if (name == "rear_wheel") return rear_;
        if (name == "cassette" && cassette_ != absent_id) return cassette_;
        return std::nullopt;
    }

    Diagnostics DrivetrainWriter::stored_energy() {
        if (simplified_)
            return {};
        if (!live_.reference || !live_.angles)
            throw std::runtime_error("initialize the drivetrain before reading energy");
        const double e =
                geometry_.evaluate(model_, data_, config_.policies.gearing, crank_,
                                   cassette_body(),
                                   frame_, Vec2{live_.angles->values[0],
                                                live_.angles->values[1]},
                                   live_.psi, false) -
                *live_.reference;
        const auto qc = joints_.at("cassette_spin"), qw = joints_.at("rear_wheel_spin");
        const auto q = buffer(data_->qpos, model_->nq);
        const double phi = q[static_cast<std::size_t>(qc.qpos)] -
                           q[static_cast<std::size_t>(qw.qpos)],
                boundary = engaged(hub_).state().boundary.value_or(phi);
        return {
            {"chain", .5 * config_.chain_k_n_m * pyfloat::pow(std::max(e, 0.), 2.)},
            {
                "freehub", .5 * config_.policies.hub_stiffness_nm_rad *
                           pyfloat::pow(std::max(phi - boundary, 0.), 2.)
            }
        };
    }

    DriveSnapshot DrivetrainWriter::state() const {
        auto s = live_;
        s.pedaling = pedaling_.state();
        s.shifting = shifting_.state();
        s.assist = assist_.state();
        s.battery = battery_.state();
        s.hub = hub_ ? std::optional(hub_->state()) : std::nullopt;
        s.ideal_hub = ideal_hub_ ? std::optional(ideal_hub_->state()) : std::nullopt;
        s.clutch = clutch_ ? std::optional(clutch_->state()) : std::nullopt;
        s.freewheel = freewheel_ ? std::optional(freewheel_->state()) : std::nullopt;
        return s;
    }

    void DrivetrainWriter::restore(const DriveSnapshot &s) {
        validate(s, config_, *model_);
        auto p = pedaling_;
        auto sh = shifting_;
        auto a = assist_;
        auto b = battery_;
        auto h = hub_;
        p.set_state(s.pedaling);
        sh.set_state(s.shifting);
        a.set_state(s.assist);
        b.set_state(s.battery);
        if (s.hub.has_value() != h.has_value())
            throw std::invalid_argument("hub topology");
        if (h)
            h->set_state(*s.hub);
        finite_optional(s.reference, "chain reference");
        finite_optional(s.psi, "chain psi");
        finite_optional(s.last_time_s, "last time");
        finite_optional(s.shift_time_s, "shift time");
        if (s.angles) {
            if (s.angles->count != (simplified_ ? 1U : 2U))
                throw std::invalid_argument("drive angle width");
            for (std::size_t i = 0; i < s.angles->count; ++i)
                finite(s.angles->values[i], "drive angle");
        }
        if (s.reference.has_value() != s.angles.has_value() ||
            (!simplified_ && s.reference.has_value() != s.psi.has_value()))
            throw std::invalid_argument("drive initialization state");
        if (s.pending_actuation) {
            nonnegative(s.pending_actuation->requested, "pending request");
            finite(s.pending_actuation->omega, "pending omega");
            positive(s.pending_actuation->dt, "pending dt");
        }
        for (const auto &pair: {
                 std::pair{ideal_hub_.get(), &s.ideal_hub},
                 std::pair{clutch_.get(), &s.clutch},
                 std::pair{freewheel_.get(), &s.freewheel}
             }) {
            if (static_cast<bool>(pair.first) != pair.second->has_value())
                throw std::invalid_argument("transmission topology");
            if (pair.first)
                pair.first->validate(engaged(*pair.second));
        }
        // Complete all allocations before model/policy mutation — the
        // snapshot copy stages in member-owned storage (the transmission
        // candidates materialize as call arguments, one transient slot
        // each), keeping restore() under the frame contract.
        snapshot_scratch_ = s;
        if (ideal_hub_)
            ideal_hub_->restore(engaged(s.ideal_hub));
        if (clutch_)
            clutch_->restore(engaged(s.clutch));
        if (freewheel_)
            freewheel_->restore(engaged(s.freewheel));
        pedaling_ = p;
        shifting_ = std::move(sh);
        assist_ = std::move(a);
        battery_ = b;
        hub_ = h;
        live_ = std::move(snapshot_scratch_);
        // A publication: retire every outstanding staged handle. The banks
        // reseed themselves from the restored live_ on their next stage —
        // extents cannot change across a restore (validate() pins the
        // topology), so no capacity reseeding is needed.
        ++commit_epoch_;
    }
} // namespace drivetrain
