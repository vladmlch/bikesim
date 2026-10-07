#include "drivetrain.hpp"
#include "../engaged.hpp"
#include "../model_topology.hpp"
#include "pyfloat.hpp"
#include <algorithm>
#include <array>
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
          transmission_(static_cast<std::size_t>(m->nv)) {
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
        for (const char *name: {"chain", "freehub", "drive_bearings"})
            components_.emplace_back(name, std::vector<double>(transmission_.size()));
        if (simplified_)
            components_.emplace_back("ideal_transmission",
                                     std::vector<double>(transmission_.size()));
    }

    double DrivetrainWriter::angle(int body, std::optional<double> reference) {
        const double raw = geometry_.angle(model_, data_, body, false);
        return reference ? unwrap(raw, *reference) : raw;
    }

    void DrivetrainWriter::shifts(Diagnostics &d, const ShiftingSnapshot &s,
                                  std::optional<double> shift_time_s) const {
        d["gear_front_teeth"] = config_.policies.gearing.front_teeth;
        d["gear_rear_teeth"] = s.rear_teeth;
        d["gear_ratio"] = static_cast<double>(config_.policies.gearing.front_teeth) /
                          s.rear_teeth;
        d["shift_active"] = s.cut_remaining_s > 0.;
        d["shift_direction"] = s.direction;
        d["shift_from_teeth"] = s.from_teeth;
        d["shift_count"] = s.shift_count;
        d["shift_time_s"] = shift_time_s ? DiagnosticValue(*shift_time_s)
                                         : DiagnosticValue(std::monostate{});
        d["shift_torque_factor"] =
                s.cut_remaining_s > 0. ? config_.policies.shifting.torque_factor : 1.;
    }

    // Post-commit snapshot view of a staged update: Transmission::state()
    // reads wrap_prm/tendon_range from the model, which commit() rewrites
    // from update — so the mirror is the staged state plus those rows.
    TransmissionSnapshot
    DrivetrainWriter::committed_snapshot(const TransmissionUpdate &update) {
        TransmissionSnapshot snapshot = update.state;
        snapshot.range = update.range;
        snapshot.coefficients = update.coefficients;
        snapshot.prepared = update.prepared_valid
                                ? std::optional<PreparedTransmission>(update.prepared)
                                : std::nullopt;
        return snapshot;
    }

    void DrivetrainWriter::reset() {
        mj_kinematics(model_, data_);
        // Stage: every allocation and every candidate lands before the
        // first live write, so a failure anywhere above the commit line
        // leaves the drivetrain exactly as found.
        DriveSnapshot next = live_;
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
        std::optional<TransmissionUpdate> hub_update, clutch_update,
                freewheel_update;
        if (simplified_) {
            next.angles = std::vector<double>{angle(crank_)};
            next.reference = 0.;
            next.psi.reset();
            if (ideal_hub_) {
                // Live order was set_ratio(configured gearing) then reset();
                // staging composes both candidates into one update.
                const double ratio =
                        static_cast<double>(config_.policies.gearing.front_teeth) /
                        config_.policies.gearing.rear_teeth;
                if (geometric_hub_) {
                    // reset() seeded at the current gear, then the ratio
                    // staging composes on top — pending stays set inside the
                    // composed update, but the published reset state clears
                    // it, exactly like the sequential live order did.
                    TransmissionUpdate update = ideal_hub_->stage_ratio(
                        data_, ratio, ideal_hub_->stage_reset(data_));
                    update.state.shift_pending = false;
                    hub_update = std::move(update);
                } else {
                    TransmissionUpdate update =
                            ideal_hub_->stage_ratio(data_, ratio);
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
                    update.range[1] = *update.state.boundary;
                    update.state.range = update.range;
                    hub_update = std::move(update);
                }
                next.ideal_hub = committed_snapshot(*hub_update);
            }
            if (clutch_) {
                clutch_update = clutch_->stage_reset(data_);
                next.clutch = committed_snapshot(*clutch_update);
            }
            if (freewheel_) {
                freewheel_update = freewheel_->stage_reset(data_);
                next.freewheel = committed_snapshot(*freewheel_update);
            }
        } else {
            next.angles = std::vector<double>{
                angle(crank_), angle(cassette_body())};
            next.reference = geometry_.evaluate(
                model_, data_, config_.policies.gearing, crank_, cassette_body(),
                frame_,
                Vec2{(*next.angles)[0], (*next.angles)[1]}, std::nullopt, false);
            next.psi = geometry_.psi;
        }
        Diagnostics last = {
            {"chain_energy_j", 0.},
            {"freehub_energy_j", 0.},
            {"motor_torque_nm", 0.},
            {"human_torque_nm", 0.},
            {"electrical_power_w", 0.},
            {"freehub_torque_nm", 0.},
            {"motor_freewheel_engaged", false},
            {"motor_freewheel_torque_nm", 0.},
            {"motor_freewheel_dissipation_power_w", 0.}
        };
        shifts(last, next.shifting, next.shift_time_s);
        next.last = std::move(last);
        // Commit: guarded model writes publish first (an engine failure
        // poisons the owning Stepper per the E1 contract), then the policy
        // resets and the snapshot swap — all memory-only.
        if (hub_update)
            ideal_hub_->commit(*hub_update);
        if (clutch_update)
            clutch_->commit(*clutch_update);
        if (freewheel_update)
            freewheel_->commit(*freewheel_update);
        shifting_.reset();
        assist_.reset();
        battery_.reset();
        pedaling_.reset();
        if (hub_)
            engaged(hub_).reset();
        live_ = std::move(next);
    }

    void DrivetrainWriter::restart_clock() {
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
        auto tick = stage_prepare(
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
        auto tick = stage_components(
            {.control = c, .dt = dt, .speed = speed, .sensed = sensed, .braking = braking,
             .active = active, .advance = advance, .contact = contact, .pedaling = pedaling,
             .slip = slip});
        commit(tick);
        return components_;
    }

    PreparedTick DrivetrainWriter::stage_prepare(const PrepareInputs &in) {
        if (in.advance && live_.last_time_s && data_->time <= *live_.last_time_s)
            throw std::invalid_argument(
                "drivetrain state can advance only once per timestamp");
        PreparedTick tick;
        tick.advance = in.advance;
        tick.snapshot = live_;
        tick.shifting = shifting_.state();
        tick.assist = assist_.state();
        tick.battery = battery_.state();
        tick.hub = hub_ ? std::optional(hub_->state()) : std::nullopt;
        // The pedaling policy always advances on a detached candidate; only a
        // committed tick publishes it back.
        auto pedaling = pedaling_;
        stage_pedal_advance(tick, in, pedaling);
        tick.pedaling = pedaling.state();
        stage_snapshot_mirrors(tick);
        validate_tick(tick);
        return tick;
    }

    // The snapshot embeds the same mirrors state() would report after the
    // commit — candidate values where a stage produced one, current state
    // otherwise — so restore() round-trips the published snapshot verbatim
    // and commit() never allocates a fresh mirror.
    void DrivetrainWriter::stage_snapshot_mirrors(PreparedTick &tick) const {
        tick.snapshot.pedaling = tick.pedaling;
        tick.snapshot.shifting = tick.shifting;
        tick.snapshot.assist = tick.assist;
        tick.snapshot.battery = tick.battery;
        tick.snapshot.hub = tick.hub;
        const auto mirror = [](const Transmission *transmission,
                               const std::optional<TransmissionUpdate> &update)
                -> std::optional<TransmissionSnapshot> {
            if (!transmission)
                return std::nullopt;
            return update ? committed_snapshot(*update) : transmission->state();
        };
        tick.snapshot.ideal_hub = mirror(ideal_hub_.get(), tick.ideal_hub);
        tick.snapshot.clutch = mirror(clutch_.get(), tick.clutch);
        tick.snapshot.freewheel = mirror(freewheel_.get(), tick.freewheel);
    }

    // Every candidate is validated while staged so commit()'s publication is
    // genuinely memory-only: set_state() validation can never throw there.
    void DrivetrainWriter::validate_tick(const PreparedTick &tick) const {
        auto pedaling = pedaling_;
        pedaling.set_state(tick.pedaling);
        auto shifter = shifting_;
        shifter.set_state(tick.shifting);
        auto assist = assist_;
        assist.set_state(tick.assist);
        auto battery = battery_;
        battery.set_state(tick.battery);
        if (hub_) {
            auto hub = hub_;
            engaged(hub).set_state(engaged(tick.hub));
        }
    }

    void DrivetrainWriter::stage_pedal_advance(PreparedTick &tick,
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
            auto shifter = shifting_;
            const bool shifted = shifter.update(
                v[static_cast<std::size_t>(crank.dof)] * 60. / (2. * std::numbers::pi),
                required, in.dt, enabled && effort > 0., in.braking, in.contact, in.slip);
            if (shifted) {
                tick.ideal_hub = tick.ideal_hub
                                     ? ideal_hub_->stage_ratio(data_, shifter.gear_ratio(),
                                                               std::move(*tick.ideal_hub))
                                     : ideal_hub_->stage_ratio(data_, shifter.gear_ratio());
                tick.snapshot.shift_time_s = data_->time;
            }
            tick.shifting = shifter.state();
            required = v[static_cast<std::size_t>(wheel.dof)] / shifter.gear_ratio() *
                       60. / (2. * std::numbers::pi);
            torque_factor = shifter.torque_factor();
        }
        auto result = pedaling.update(q[static_cast<std::size_t>(crank.qpos)],
                                      v[static_cast<std::size_t>(crank.dof)], required,
                                      effort, in.dt, enabled, in.braking);
        if (in.ceiling)
            result.effort_nm = std::min(
                result.effort_nm,
                nonnegative(*in.ceiling, "automatic rider effort ceiling"));
        result.effort_nm *= torque_factor;
        tick.result = std::move(result);
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
        PreparedTick tick;
        tick.advance = in.advance;
        tick.probe = !in.advance;
        tick.snapshot = live_;
        tick.pedaling = pedaling_.state();
        tick.shifting = shifting_.state();
        tick.assist = assist_.state();
        tick.battery = battery_.state();
        tick.hub = hub_ ? std::optional(hub_->state()) : std::nullopt;
        if (!in.advance) {
            // Probe evaluation runs on the detached candidate: pending
            // settlement and the interval clock are cleared only there.
            tick.snapshot.pending_actuation.reset();
            tick.snapshot.last_time_s.reset();
        }
        // An unsettled actuation rejects before any policy advance or model
        // staging — repeated rejected calls cannot creep EMAs or effort.
        if (in.active && tick.snapshot.pending_actuation)
            throw std::runtime_error("previous motor interval was not settled");
        if (in.advance) {
            if (ideal_hub_)
                tick.ideal_hub = ideal_hub_->stage_prepare(data_);
            if (clutch_)
                tick.clutch = clutch_->stage_prepare(data_);
            if (freewheel_)
                tick.freewheel = freewheel_->stage_prepare(data_);
        }
        stage_components_telemetry(tick, in,
                                   stage_components_metrics(tick, in));
        return tick;
    }

    DrivetrainWriter::TickMetrics
    DrivetrainWriter::stage_components_metrics(PreparedTick &tick,
                                               const TickInputs &in) {
        TickMetrics m;
        PedalingState &ps = m.ps;
        if (in.pedaling) {
            ps = *in.pedaling;
            tick.result = ps;
        } else {
            auto pedaling = pedaling_;
            stage_pedal_advance(tick,
                                {.control = in.control, .dt = in.dt, .braking = in.braking,
                                 .active = in.active, .advance = in.advance,
                                 .contact = in.contact, .slip = in.slip,
                                 .ceiling = std::nullopt},
                                pedaling);
            tick.pedaling = pedaling.state();
            ps = tick.result;
        }
        tick.components = components_;
        for (auto &component: tick.components)
            std::ranges::fill(component.second, 0.);
        const auto crank = joints_.at("crank_spin"), wheel = joints_.at("rear_wheel_spin");
        const auto q = buffer(data_->qpos, model_->nq), v = buffer(data_->qvel, model_->nv);
        if (!simplified_) {
            m.angles = {
                angle(crank_, engaged(tick.snapshot.angles)[0]),
                angle(cassette_body(), engaged(tick.snapshot.angles)[1])
            };
            m.extension =
                    geometry_.evaluate(model_, data_, config_.policies.gearing, crank_,
                                       cassette_body(), frame_,
                                       Vec2{m.angles[0], m.angles[1]},
                                       tick.snapshot.psi, false) -
                    engaged(live_.reference);
            m.psi = geometry_.psi;
            m.rate = dot(geometry_.jacobian, v);
            auto const t =
                    chain_tension(m.extension, m.rate, config_.chain_k_n_m,
                                  config_.chain_c_ns_m);
            m.tension = t.first;
            m.energy = t.second;
            for (std::size_t i = 0; i < transmission_.size(); ++i)
                tick.components[0].second[i] = -m.tension * geometry_.jacobian[i];
            const auto cassette = joints_.at("cassette_spin");
            auto hub = hub_;
            m.torque =
                    engaged(hub).update(q[static_cast<std::size_t>(cassette.qpos)],
                                        q[static_cast<std::size_t>(wheel.qpos)],
                                        v[static_cast<std::size_t>(cassette.dof)],
                                        v[static_cast<std::size_t>(wheel.dof)]);
            tick.hub = engaged(hub).state();
            tick.components[1].second[static_cast<std::size_t>(wheel.dof)] = m.torque;
            tick.components[1].second[static_cast<std::size_t>(cassette.dof)] = -m.torque;
            m.relative_rate = v[static_cast<std::size_t>(cassette.dof)] -
                              v[static_cast<std::size_t>(wheel.dof)];
            m.deflection = std::max(q[static_cast<std::size_t>(cassette.qpos)] -
                                    q[static_cast<std::size_t>(wheel.qpos)] -
                                    engaged(tick.hub->boundary),
                                    0.);
        }
        for (auto const j: bearing_joints_)
            tick.components[2].second[static_cast<std::size_t>(j.dof)] =
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
        m.assist_sensor = ps.mode == "pedaling" ? m.sensor : 0.;
        if (in.active && effort_) {
            auto assist = assist_;
            m.request = assist.step(m.assist_sensor, m.cadence, in.speed, in.braking,
                                    in.dt, in.control.motor_torque_nm, rpm);
            tick.assist = assist.state();
        }
        const auto &b = config_.policies.battery;
        const double budget = tick.battery.energy_j / in.dt;
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
            PreparedTick &tick, const TickInputs &in, const TickMetrics &m) {
        for (const auto &component: tick.components)
            validation::derived_array(component.second, "DrivetrainWriter.force");
        const double chain_loss = std::max(0., validation::derived(
            (m.tension - config_.chain_k_n_m * std::max(m.extension, 0.)) * m.rate, "DrivetrainWriter.chain_dissipation"));
        const double hub_loss = simplified_ ? 0. : std::max(0., validation::derived(
            (m.torque - config_.policies.hub_stiffness_nm_rad * m.deflection) * m.relative_rate, "DrivetrainWriter.freehub_dissipation"));
        tick.snapshot.pending_actuation =
                in.active
                    ? std::optional<PendingActuation>{
                        {.requested = m.delivered, .omega = m.omega, .dt = in.dt,
                         .enabled = m.enabled}
                    }
                    : std::nullopt;
        if (human_actuator_ >= 0)
            tick.ctrl.emplace_back(human_actuator_, m.human);
        if (motor_actuator_ >= 0)
            tick.ctrl.emplace_back(motor_actuator_, m.delivered);
        if (effort_)
            tick.assist.torque = m.delivered;
        auto const opt = [](std::optional<double> x) -> DiagnosticValue {
            return x ? DiagnosticValue(*x) : DiagnosticValue(std::monostate{});
        };
        tick.snapshot.last = {
            {"transmission_model", config_.transmission_model},
            {"omits_suspension_coupling", config_.transmission_model == "ideal_mid_drive"},
            {"chain_extension_m", m.extension},
            {"chain_extension_rate_mps", m.rate},
            {"chain_tension_n", m.tension},
            {"chain_energy_j", m.energy},
            {
                "chain_dissipation_power_w",
                chain_loss
            },
            {"freehub_torque_nm", simplified_ ? 0. : m.torque},
            {"freehub_energy_j", simplified_ ? 0. : engaged(tick.hub).energy_j},
            {"freehub_engaged", simplified_ ? false : m.torque > 0.},
            {"freehub_deflection_rad", m.deflection},
            {
                "freehub_dissipation_power_w",
                hub_loss
            },
            {"cadence_rpm", m.cadence},
            {"crank_rad_s", m.omega_crank},
            {"drive_shaft_rad_s", m.omega},
            {"human_torque_nm", m.human},
            {"human_sensor_nm", m.sensor},
            {"human_setpoint_nm", opt(in.control.human_torque_nm)},
            {
                "assist_demand_gated",
                in.braking || m.assist_sensor <= config_.policies.assist.engage_torque_nm ||
                m.omega_crank <= config_.policies.assist.gate_min_crank_rad_s
            },
            {"assist_sensor_nm", m.assist_sensor},
            {"human_command_nm", m.ps.effort_nm},
            {"rider_mode", m.ps.mode},
            {"coasting_reason", m.ps.reason},
            {"required_cadence_rpm", m.ps.required_cadence_rpm},
            {"crank_target_phase_rad", opt(m.ps.target_phase_rad)},
            {"crank_target_rate_rad_s", m.ps.target_rate_rad_s},
            {"motor_request_nm", m.request},
            {"motor_torque_nm", m.delivered},
            {"motor_freewheel_engaged", m.delivered > 0.},
            {"motor_freewheel_torque_nm", m.delivered},
            {"motor_freewheel_dissipation_power_w", 0.},
            {"motor_limited_request_nm", m.safety},
            {"motor_setpoint_nm", opt(in.control.motor_torque_nm)},
            {"motor_limit_nm", opt(in.control.motor_limit_nm)},
            {
                "motor_control_source",
                std::string(in.control.motor_torque_nm ? "external_request" : "assist")
            },
            {"safety_limited", m.safety < m.request},
            {"motor_shaft_power_w", m.delivered * m.omega},
            {"electrical_power_w", 0.},
            {"battery_energy_j", tick.battery.energy_j},
            {"motor_enabled", m.enabled},
            {"energy_limited", m.delivered < m.safety},
            {"battery_empty", tick.battery.energy_j == 0.},
            {"assist_mode", config_.policies.assist.mode},
            {"assist_gain", tick.assist.last_gain}
        };
        shifts(tick.snapshot.last, tick.shifting, tick.snapshot.shift_time_s);
        if (simplified_)
            tick.snapshot.angles =
                    std::vector<double>{angle(crank_, engaged(tick.snapshot.angles)[0])};
        else {
            tick.snapshot.angles = m.angles;
            tick.snapshot.psi = m.psi;
        }
        tick.snapshot.last_time_s = data_->time;
        if (tick.advance)
            stage_snapshot_mirrors(tick);
        validate_tick(tick);
    }

    void DrivetrainWriter::commit(PreparedTick &tick) {
        if (tick.probe) {
            // Probe ticks publish only their declared telemetry: the force
            // rows the caller applies and the probe diagnostics channel.
            // Policy, clock, pending and model state stay untouched.
            if (!tick.components.empty())
                for (std::size_t i = 0; i < components_.size(); ++i)
                    std::ranges::copy(tick.components[i].second,
                                      components_[i].second.begin());
            {
                const auto ctrl = buffer(data_->ctrl, model_->nu);
                for (const auto &[index, value]: tick.ctrl)
                    ctrl[static_cast<std::size_t>(index)] = value;
            }
            live_.probe_last = std::move(tick.snapshot.last);
            return;
        }
        if (!tick.advance)
            return;
        // Guarded live-model writes first (E1 boundary inside each
        // Transmission::commit): an engine failure propagates and poisons the
        // owning Stepper before any logical publication. A staged update
        // exists only when the transmission does — staging adds them under
        // the same topology guard.
        if (tick.ideal_hub)
            ideal_hub_->commit(*tick.ideal_hub);
        if (tick.clutch)
            clutch_->commit(*tick.clutch);
        if (tick.freewheel)
            freewheel_->commit(*tick.freewheel);
        // Memory-only publication: every candidate was fully evaluated and
        // validated during staging, so the writes below are moves into live
        // storage or copies into preallocated extents.
        {
            const auto ctrl = buffer(data_->ctrl, model_->nu);
            for (const auto &[index, value]: tick.ctrl)
                ctrl[static_cast<std::size_t>(index)] = value;
        }
        if (!tick.components.empty())
            for (std::size_t i = 0; i < components_.size(); ++i)
                std::ranges::copy(tick.components[i].second,
                                  components_[i].second.begin());
        pedaling_.set_state(tick.pedaling);
        shifting_.set_state(std::move(tick.shifting));
        assist_.set_state(tick.assist);
        battery_.set_state(tick.battery);
        if (hub_)
            engaged(hub_).set_state(engaged(tick.hub));
        // The snapshot's embedded policy/transmission mirrors were staged
        // with the committed values — the publication below is one move.
        live_ = std::move(tick.snapshot);
    }

    void DrivetrainWriter::validate_settlement(
        const PreparedSettlement &settlement) const {
        if (settlement.battery) {
            auto battery = battery_;
            battery.set_state(*settlement.battery);
        }
        if (settlement.assist) {
            auto assist = assist_;
            assist.set_state(*settlement.assist);
        }
    }

    PreparedSettlement DrivetrainWriter::stage_settle() {
        PreparedSettlement settlement;
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
                settlement.battery = candidate;
            }
            AssistSnapshot assist = assist_.state();
            assist.torque = pending_torque;
            settlement.assist = assist;
            settlement.clear_pending = true;
        }
        validate_settlement(settlement);

        // Phase 2 — the telemetry write plan. Every settlement key resolves
        // to a map slot BEFORE the solve runs; when a key is absent (a
        // restored sparse `last`) or the geometric merge must carry its
        // string status, the whole publication moves onto a detached
        // candidate map. commit() then performs either plain slot writes or
        // one map move — never an allocation.
        constexpr std::array<std::string_view, 3> clutch_keys{
            "crank_clutch_torque_nm", "crank_clutch_engaged",
            "crank_clutch_dissipation_power_w"};
        constexpr std::array<std::string_view, 3> freewheel_keys{
            "motor_freewheel_torque_nm", "motor_freewheel_engaged",
            "motor_freewheel_dissipation_power_w"};
        // Transmission::solved publishes this fixed geometric telemetry
        // contract — pre-created here so the post-solve merge never has to
        // allocate a fresh map node.
        constexpr std::array<std::string_view, 11> geometric_keys{
            "transmission_phi_m", "transmission_boundary_m",
            "transmission_gap_m", "transmission_tension_n",
            "transmission_constraint_defect_m", "transmission_interval_work_j",
            "transmission_reaction_error_n", "shift_parameter_work_j",
            "shift_interval_constraint_work_j",
            "shift_constraint_work_cumulative_j",
            "transmission_reference_status"};
        constexpr std::array<std::string_view, 6> pending_keys{
            "motor_torque_nm", "motor_shaft_power_w", "electrical_power_w",
            "battery_energy_j", "motor_enabled", "battery_empty"};
        std::vector<std::string_view> keys;
        if (ideal_hub_) {
            keys.emplace_back("freehub_torque_nm");
            keys.emplace_back("freehub_engaged");
            if (config_.motor_clutch)
                keys.emplace_back("freehub_dissipation_power_w");
            if (geometric_hub_)
                for (const std::string_view key: geometric_keys)
                    keys.emplace_back(key);
        }
        if (clutch_)
            for (const std::string_view key: clutch_keys)
                keys.emplace_back(key);
        if (freewheel_)
            for (const std::string_view key: freewheel_keys)
                keys.emplace_back(key);
        if (pending)
            for (const std::string_view key: pending_keys)
                keys.emplace_back(key);
        // A geometric hub merges its solve telemetry — including the
        // std::string status — so it always publishes through the candidate
        // map. An ideal hub normally publishes an empty map, but a restored
        // snapshot can carry arbitrary diagnostics and staging preserves
        // them; any nonempty map takes the detached path as well, because
        // keys outside the declared set could never resolve to a slot and
        // an in-place fallback would write live_.last mid-staging.
        const bool merge_telemetry =
                ideal_hub_ &&
                (geometric_hub_ || !ideal_hub_->diagnostics().empty());
        bool candidate_map = merge_telemetry;
        if (!candidate_map)
            for (const std::string_view key: keys)
                if (live_.last.find(std::string(key)) == live_.last.end()) {
                    candidate_map = true;
                    break;
                }
        Diagnostics &target =
                candidate_map ? settlement.last.emplace(live_.last) : live_.last;
        for (const std::string_view key: keys)
            static_cast<void>(target[std::string(key)]);
        // The status value is a std::string — give its slot a heap buffer
        // now so the post-solve assignment stays allocation-free. Only an
        // owned geometric hub ever writes this key; seeding it on a hubless
        // (passive) or ideal-drift path would publish a spurious empty
        // entry.
        if (ideal_hub_ && geometric_hub_) {
            DiagnosticValue &status =
                    target["transmission_reference_status"];
            if (!std::holds_alternative<std::string>(status))
                status.emplace<std::string>();
            std::get<std::string>(status).reserve(64);
        }
        std::vector<std::pair<std::string_view, DiagnosticValue *> > plan;
        plan.reserve(keys.size());
        for (const std::string_view key: keys) {
            const auto it = target.find(std::string(key));
            plan.emplace_back(key, &it->second);
        }
        // post-solve lookups are pointer-only: every planned key was
        // resolved above, so slot_of never misses a declared write.
        const auto slot_of = [&plan](std::string_view key) noexcept
                -> DiagnosticValue * {
            for (const auto &[k, slot]: plan)
                if (k == key)
                    return slot;
            return nullptr;
        };
        const auto record = [&settlement](DiagnosticValue *slot,
                                          const DiagnosticValue &value) {
            if (settlement.last)
                *slot = value;
            else if (settlement.write_count == PreparedSettlement::write_capacity)
                throw std::logic_error("settlement write plan overflow");
            else
                settlement.writes[settlement.write_count++] =
                        PreparedSettlement::Write{.slot = slot, .value = value};
        };

        // Phase 3 — the solve. Each transmission stages its own candidate:
        // the reaction force lands in the persistent transmission_ scratch
        // now, but the solve's telemetry/state publication rides the
        // settlement's commit — a throw here (a bad_alloc inside a
        // candidate map, an unprepared geometric hub) leaves every
        // transmission and the pending reservation byte-identical.
        std::ranges::fill(transmission_, 0.);
        if (ideal_hub_) {
            settlement.ideal_solve = ideal_hub_->stage_solved(data_);
            std::ranges::copy(settlement.ideal_solve->force,
                              transmission_.begin());
        }
        struct Aux {
            Transmission *transmission;
            std::optional<SolvedTransmission> *slot;
            std::string_view torque_key, engaged_key, dissipation_key;
        };
        for (const Aux &aux: {
                 Aux{.transmission = clutch_.get(),
                     .slot = &settlement.clutch_solve,
                     .torque_key = clutch_keys[0],
                     .engaged_key = clutch_keys[1],
                     .dissipation_key = clutch_keys[2]},
                 Aux{.transmission = freewheel_.get(),
                     .slot = &settlement.freewheel_solve,
                     .torque_key = freewheel_keys[0],
                     .engaged_key = freewheel_keys[1],
                     .dissipation_key = freewheel_keys[2]}})
            if (aux.transmission) {
                *aux.slot = aux.transmission->stage_solved(data_);
                const std::span<const double> f = (*aux.slot)->force;
                for (std::size_t i = 0; i < transmission_.size(); ++i)
                    transmission_[i] += f[i];
                const double torque =
                        f[static_cast<std::size_t>(aux.transmission->driven_dof())];
                record(slot_of(aux.torque_key), torque);
                record(slot_of(aux.engaged_key), torque > 1e-8);
                record(slot_of(aux.dissipation_key),
                       std::max(0., torque * aux.transmission->relative_rate(data_)));
            }
        if (ideal_hub_) {
            const double torque =
                    transmission_[static_cast<std::size_t>(ideal_hub_->driven_dof())];
            record(slot_of("freehub_torque_nm"), torque);
            record(slot_of("freehub_engaged"), torque > 1e-8);
            if (config_.motor_clutch)
                record(slot_of("freehub_dissipation_power_w"),
                       std::max(0., torque * ideal_hub_->relative_rate(data_)));
            // Merge from the staged candidate — not the live diagnostics,
            // which publish only at commit. Keys outside the declared set
            // only exist when merge_telemetry already forced the detached
            // map, and that is asserted rather than assumed: writing
            // live_.last here would be a staging-time publication.
            for (const auto &[k, v]: settlement.ideal_solve->state.diagnostics)
                if (DiagnosticValue *slot = slot_of(k))
                    record(slot, v);
                else if (settlement.last)
                    (*settlement.last)[k] = v;
                else
                    throw std::logic_error(
                        "diagnostics merge requires the detached map");
        }
        if (pending) {
            const double energy = settlement.battery
                                      ? settlement.battery->energy_j
                                      : battery_.state().energy_j;
            record(slot_of("motor_torque_nm"), pending_torque);
            record(slot_of("motor_shaft_power_w"),
                   pending_torque * pending->omega);
            record(slot_of("electrical_power_w"), pending_delivered);
            record(slot_of("battery_energy_j"), energy);
            record(slot_of("motor_enabled"),
                   pending->enabled && pending_torque > 0.);
            record(slot_of("battery_empty"), energy == 0.);
        }
        return settlement;
    }

    void DrivetrainWriter::commit(PreparedSettlement &settlement) noexcept {
        // The transmission solve candidates publish through their own
        // memory-only swap first; every store below is then provably
        // allocation-free: the candidate map move never allocates, the
        // in-place slots were resolved during staging (std::map element
        // addresses are stable), and only scalar diagnostic values ever
        // travel the in-place path — a string write would have forced the
        // candidate map during staging.
        if (settlement.ideal_solve)
            ideal_hub_->commit(*settlement.ideal_solve);
        if (settlement.clutch_solve)
            clutch_->commit(*settlement.clutch_solve);
        if (settlement.freewheel_solve)
            freewheel_->commit(*settlement.freewheel_solve);
        if (settlement.last)
            live_.last = std::move(*settlement.last);
        else
            for (std::size_t i = 0; i < settlement.write_count; ++i)
                *settlement.writes[i].slot = settlement.writes[i].value;
        if (settlement.battery)
            battery_.publish(*settlement.battery);
        if (settlement.assist)
            assist_.publish(*settlement.assist);
        if (settlement.clear_pending)
            live_.pending_actuation.reset();
    }

    std::span<const double> DrivetrainWriter::settle() {
        auto settlement = stage_settle();
        const std::span<const double> force = settlement.force;
        commit(settlement);
        return force;
    }

    const Diagnostics &DrivetrainWriter::diagnostics(bool probe) const {
        static const Diagnostics empty;
        return probe ? (live_.probe_last ? *live_.probe_last : empty) : live_.last;
    }

    Diagnostics DrivetrainWriter::stored_energy() {
        if (simplified_)
            return {};
        if (!live_.reference || !live_.angles)
            throw std::runtime_error("initialize the drivetrain before reading energy");
        const double e =
                geometry_.evaluate(model_, data_, config_.policies.gearing, crank_,
                                   cassette_body(),
                                   frame_, Vec2{(*live_.angles)[0], (*live_.angles)[1]},
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
            if (s.angles->size() != (simplified_ ? 1U : 2U))
                throw std::invalid_argument("drive angle width");
            for (double const x: *s.angles)
                finite(x, "drive angle");
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
        auto next = s; // Complete all allocations before model/policy mutation.
        auto ideal_candidate = s.ideal_hub;
        auto clutch_candidate = s.clutch;
        auto freewheel_candidate = s.freewheel;
        if (ideal_hub_)
            ideal_hub_->restore(std::move(engaged(ideal_candidate)));
        if (clutch_)
            clutch_->restore(std::move(engaged(clutch_candidate)));
        if (freewheel_)
            freewheel_->restore(std::move(engaged(freewheel_candidate)));
        pedaling_ = p;
        shifting_ = std::move(sh);
        assist_ = std::move(a);
        battery_ = b;
        hub_ = h;
        live_ = std::move(next);
    }
} // namespace drivetrain
