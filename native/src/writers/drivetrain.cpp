#include "drivetrain.hpp"
#include "../engaged.hpp"
#include "../model_topology.hpp"
#include "pyfloat.hpp"
#include <algorithm>
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

    void DrivetrainWriter::shifts(Diagnostics &d) const {
        const auto &s = shifting_.state();
        d["gear_front_teeth"] = config_.policies.gearing.front_teeth;
        d["gear_rear_teeth"] = s.rear_teeth;
        d["gear_ratio"] = shifting_.gear_ratio();
        d["shift_active"] = s.cut_remaining_s > 0.;
        d["shift_direction"] = s.direction;
        d["shift_from_teeth"] = s.from_teeth;
        d["shift_count"] = s.shift_count;
        d["shift_time_s"] = live_.shift_time_s
                                ? DiagnosticValue(*live_.shift_time_s)
                                : DiagnosticValue(std::monostate{});
        d["shift_torque_factor"] = shifting_.torque_factor();
    }

    void DrivetrainWriter::reset() {
        mj_kinematics(model_, data_);
        shifting_.reset();
        live_.shift_time_s.reset();
        if (simplified_) {
            live_.angles = std::vector<double>{angle(crank_)};
            live_.reference = 0.;
            live_.psi.reset();
            if (ideal_hub_) {
                ideal_hub_->set_ratio(data_, shifting_.gear_ratio());
                ideal_hub_->reset(data_);
            }
            if (clutch_)
                clutch_->reset(data_);
            if (freewheel_)
                freewheel_->reset(data_);
        } else {
            live_.angles = std::vector<double>{
                angle(crank_), angle(cassette_body())};
            live_.reference = geometry_.evaluate(
                model_, data_, config_.policies.gearing, crank_, cassette_body(),
                frame_,
                Vec2{(*live_.angles)[0], (*live_.angles)[1]}, std::nullopt, false);
            live_.psi = geometry_.psi;
            engaged(hub_).reset();
        }
        assist_.reset();
        battery_.reset();
        pedaling_.reset();
        live_.pending_actuation.reset();
        live_.last_time_s.reset();
        live_.last = {
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
        shifts(live_.last);
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
        if (advance && live_.last_time_s && data_->time <= *live_.last_time_s)
            throw std::invalid_argument(
                "drivetrain state can advance only once per timestamp");
        const auto crank = joints_.at("crank_spin"), wheel = joints_.at("rear_wheel_spin");
        const auto q = buffer(data_->qpos, model_->nq), v = buffer(data_->qvel, model_->nv);
        double required = v[static_cast<std::size_t>(wheel.dof)] / shifting_.gear_ratio() *
                          60. / (2. * std::numbers::pi);
        const double effort = control.human_torque_nm.value_or(config_.human_torque_nm);
        const bool enabled = active && control.rider_enabled && effort_;
        if (advance && active && config_.policies.shifting.enabled && effort_) {
            if (!ideal_hub_)
                throw std::invalid_argument(
                    "automatic shifting needs the live ideal_mid_drive model");
            if (shifting_.update(
                v[static_cast<std::size_t>(crank.dof)] * 60. / (2. * std::numbers::pi),
                required, dt, enabled && effort > 0., braking, contact, slip)) {
                ideal_hub_->set_ratio(data_, shifting_.gear_ratio());
                live_.shift_time_s = data_->time;
            }
            required = v[static_cast<std::size_t>(wheel.dof)] / shifting_.gear_ratio() *
                       60. / (2. * std::numbers::pi);
        }
        auto policy = pedaling_;
        auto result = (advance ? pedaling_ : policy)
                .update(q[static_cast<std::size_t>(crank.qpos)],
                        v[static_cast<std::size_t>(crank.dof)], required, effort,
                        dt, enabled, braking);
        if (ceiling)
            result.effort_nm = std::min(
                result.effort_nm, nonnegative(*ceiling, "automatic rider effort ceiling"));
        result.effort_nm *= shifting_.torque_factor();
        return result;
    }

    const ForceComponents &
    DrivetrainWriter::components(const RideControl &c, double dt, double speed,
                                 bool braking, bool active, bool advance, double sensed,
                                 const std::optional<PedalingState> &pedaling, bool contact,
                                 std::optional<double> slip) {
        positive(dt, "drivetrain interval");
        finite(speed, "bike speed");
        finite(sensed, "measured pedal torque");
        if (!live_.reference)
            throw std::runtime_error("initialize the drivetrain before applying forces");
        if (advance && live_.last_time_s && data_->time <= *live_.last_time_s)
            throw std::invalid_argument(
                "drivetrain state can advance only once per timestamp");
        if (advance)
            return compute(c, dt, speed, braking, active, true, sensed, pedaling, contact,
                           slip);
        // Probe policies are detached typed copies; transmission helpers are never touched.
        const auto saved = live_;
        auto const saved_p = pedaling_;
        auto saved_s = shifting_;
        auto saved_a = assist_;
        auto const saved_b = battery_;
        auto const saved_h = hub_;
        live_.pending_actuation.reset();
        live_.last_time_s.reset();
        try {
            compute(c, dt, speed, braking, active, false, sensed, pedaling, contact, slip);
            const auto probe = live_.last;
            live_ = saved;
            live_.probe_last = probe;
        } catch (...) {
            live_ = saved;
            pedaling_ = saved_p;
            shifting_ = std::move(saved_s);
            assist_ = std::move(saved_a);
            battery_ = saved_b;
            hub_ = saved_h;
            throw;
        }
        pedaling_ = saved_p;
        shifting_ = std::move(saved_s);
        assist_ = std::move(saved_a);
        battery_ = saved_b;
        hub_ = saved_h;
        return components_;
    }

    const ForceComponents &
    DrivetrainWriter::compute(const RideControl &c, double dt, double speed, bool braking,
                              bool active, bool advance, double sensed,
                              std::optional<PedalingState> state, bool contact,
                              std::optional<double> slip) {
        if (advance) {
            if (ideal_hub_)
                ideal_hub_->prepare(data_);
            if (clutch_)
                clutch_->prepare(data_);
            if (freewheel_)
                freewheel_->prepare(data_);
        }
        const auto ps =
                state
                    ? *state
                    : prepare(c, dt, braking, active, advance, contact, slip, std::nullopt);
        for (auto &component: components_)
            std::ranges::fill(component.second, 0.);
        const auto crank = joints_.at("crank_spin"), wheel = joints_.at("rear_wheel_spin");
        const auto q = buffer(data_->qpos, model_->nq), v = buffer(data_->qvel, model_->nv);
        double extension = 0., rate = 0., tension = 0., energy = 0., torque = 0.,
                deflection = 0., relative_rate = 0.;
        std::vector<double> angles;
        std::optional<double> psi;
        if (!simplified_) {
            angles = {
                angle(crank_, engaged(live_.angles)[0]),
                angle(cassette_body(), engaged(live_.angles)[1])
            };
            extension = geometry_.evaluate(model_, data_, config_.policies.gearing, crank_,
                                           cassette_body(), frame_, Vec2{angles[0], angles[1]},
                                           live_.psi, false) -
                        engaged(live_.reference);
            psi = geometry_.psi;
            rate = dot(geometry_.jacobian, v);
            auto const t =
                    chain_tension(extension, rate, config_.chain_k_n_m, config_.chain_c_ns_m);
            tension = t.first;
            energy = t.second;
            for (std::size_t i = 0; i < transmission_.size(); ++i)
                components_[0].second[i] = -tension * geometry_.jacobian[i];
            const auto cassette = joints_.at("cassette_spin");
            torque = engaged(hub_).update(q[static_cast<std::size_t>(cassette.qpos)],
                                          q[static_cast<std::size_t>(wheel.qpos)],
                                          v[static_cast<std::size_t>(cassette.dof)],
                                          v[static_cast<std::size_t>(wheel.dof)]);
            components_[1].second[static_cast<std::size_t>(wheel.dof)] = torque;
            components_[1].second[static_cast<std::size_t>(cassette.dof)] = -torque;
            relative_rate = v[static_cast<std::size_t>(cassette.dof)] -
                            v[static_cast<std::size_t>(wheel.dof)];
            deflection = std::max(q[static_cast<std::size_t>(cassette.qpos)] -
                                  q[static_cast<std::size_t>(wheel.qpos)] -
                                  engaged(engaged(hub_).state().boundary),
                                  0.);
        }
        for (auto const j: bearing_joints_)
            components_[2].second[static_cast<std::size_t>(j.dof)] =
                    validation::derived(-config_.bearing_c_nms_rad * v[static_cast<std::size_t>(j.dof)], "DrivetrainWriter.bearing_force");
        const double omega_crank = v[static_cast<std::size_t>(crank.dof)],
                cadence = validation::derived(omega_crank * 60. / (2. * std::numbers::pi), "DrivetrainWriter.cadence");
        const auto shaft = joints_.contains("drive_shaft_spin")
                               ? joints_.at("drive_shaft_spin")
                               : joints_.contains("rotor_spin")
                                     ? joints_.at("rotor_spin")
                                     : crank;
        const double omega = v[static_cast<std::size_t>(shaft.dof)],
                rpm = validation::derived(omega * 60. / (2. * std::numbers::pi), "DrivetrainWriter.shaft_rpm");
        const double human =
                active && config_.drive_mode == "crank_effort"
                    ? human_crank_torque(ps.effort_nm, q[static_cast<std::size_t>(crank.qpos)],
                                         config_.torque_ripple)
                    : 0.;
        const double sensor = config_.drive_mode == "crank_effort" ? human : sensed,
                assist_sensor = ps.mode == "pedaling" ? sensor : 0.;
        const double request = active && effort_
                                   ? assist_.step(assist_sensor, cadence, speed, braking,
                                                  dt, c.motor_torque_nm, rpm)
                                   : 0.;
        const auto &b = config_.policies.battery;
        const double budget = battery_.state().energy_j / dt,
                safety =
                        c.motor_limit_nm ? std::min(request, *c.motor_limit_nm) : request;
        double delivered =
                b.enabled
                    ? limit_torque_by_energy(safety, omega, b.copper_w_per_nm2,
                                             b.speed_w_per_rad_s2, b.idle_w, budget)
                    : safety;
        const bool enabled = active && delivered > 0. && !braking;
        if (!enabled)
            delivered = 0.;
        const double electrical = motor_electrical_power(
            delivered, omega, b.copper_w_per_nm2, b.speed_w_per_rad_s2, b.idle_w, enabled);
        if (b.enabled && electrical > budget + std::max(1e-10, std::abs(budget) * 1e-12))
            throw ArithmeticError("delivered motor torque exceeds the battery budget");
        if (active && live_.pending_actuation)
            throw std::runtime_error("previous motor interval was not settled");
        for (const auto &component: components_)
            validation::derived_array(component.second, "DrivetrainWriter.force");
        const double chain_loss = std::max(0., validation::derived(
            (tension - config_.chain_k_n_m * std::max(extension, 0.)) * rate, "DrivetrainWriter.chain_dissipation"));
        const double hub_loss = simplified_ ? 0. : std::max(0., validation::derived(
            (torque - config_.policies.hub_stiffness_nm_rad * deflection) * relative_rate, "DrivetrainWriter.freehub_dissipation"));
        live_.pending_actuation =
                active
                    ? std::optional<PendingActuation>{
                        {.requested = delivered, .omega = omega, .dt = dt, .enabled = enabled}
                    }
                    : std::nullopt;
        auto const ctrl = buffer(data_->ctrl, model_->nu);
        if (human_actuator_ >= 0)
            ctrl[static_cast<std::size_t>(human_actuator_)] = human;
        if (motor_actuator_ >= 0)
            ctrl[static_cast<std::size_t>(motor_actuator_)] = delivered;
        if (effort_) {
            auto s = assist_.state();
            s.torque = delivered;
            assist_.set_state(s);
        }
        auto const opt = [](std::optional<double> x) -> DiagnosticValue {
            return x ? DiagnosticValue(*x) : DiagnosticValue(std::monostate{});
        };
        live_.last = {
            {"transmission_model", config_.transmission_model},
            {"omits_suspension_coupling", config_.transmission_model == "ideal_mid_drive"},
            {"chain_extension_m", extension},
            {"chain_extension_rate_mps", rate},
            {"chain_tension_n", tension},
            {"chain_energy_j", energy},
            {
                "chain_dissipation_power_w",
                chain_loss
            },
            {"freehub_torque_nm", simplified_ ? 0. : torque},
            {"freehub_energy_j", simplified_ ? 0. : engaged(hub_).state().energy_j},
            {"freehub_engaged", simplified_ ? false : torque > 0.},
            {"freehub_deflection_rad", deflection},
            {
                "freehub_dissipation_power_w",
                hub_loss
            },
            {"cadence_rpm", cadence},
            {"crank_rad_s", omega_crank},
            {"drive_shaft_rad_s", omega},
            {"human_torque_nm", human},
            {"human_sensor_nm", sensor},
            {"human_setpoint_nm", opt(c.human_torque_nm)},
            {
                "assist_demand_gated",
                braking || assist_sensor <= config_.policies.assist.engage_torque_nm ||
                omega_crank <= config_.policies.assist.gate_min_crank_rad_s
            },
            {"assist_sensor_nm", assist_sensor},
            {"human_command_nm", ps.effort_nm},
            {"rider_mode", ps.mode},
            {"coasting_reason", ps.reason},
            {"required_cadence_rpm", ps.required_cadence_rpm},
            {"crank_target_phase_rad", opt(ps.target_phase_rad)},
            {"crank_target_rate_rad_s", ps.target_rate_rad_s},
            {"motor_request_nm", request},
            {"motor_torque_nm", delivered},
            {"motor_freewheel_engaged", delivered > 0.},
            {"motor_freewheel_torque_nm", delivered},
            {"motor_freewheel_dissipation_power_w", 0.},
            {"motor_limited_request_nm", safety},
            {"motor_setpoint_nm", opt(c.motor_torque_nm)},
            {"motor_limit_nm", opt(c.motor_limit_nm)},
            {
                "motor_control_source",
                std::string(c.motor_torque_nm ? "external_request" : "assist")
            },
            {"safety_limited", safety < request},
            {"motor_shaft_power_w", delivered * omega},
            {"electrical_power_w", 0.},
            {"battery_energy_j", battery_.state().energy_j},
            {"motor_enabled", enabled},
            {"energy_limited", delivered < safety},
            {"battery_empty", battery_.state().energy_j == 0.},
            {"assist_mode", config_.policies.assist.mode},
            {"assist_gain", assist_.state().last_gain}
        };
        shifts(live_.last);
        if (simplified_)
            live_.angles = std::vector<double>{angle(crank_, engaged(live_.angles)[0])};
        else {
            live_.angles = angles;
            live_.psi = psi;
        }
        live_.last_time_s = data_->time;
        return components_;
    }

    std::span<const double> DrivetrainWriter::settle() {
        std::ranges::fill(transmission_, 0.);
        if (ideal_hub_)
            std::ranges::copy(ideal_hub_->solved(data_), transmission_.begin());
        for (auto *t: {clutch_.get(), freewheel_.get()})
            if (t) {
                const auto f = t->solved(data_);
                for (std::size_t i = 0; i < transmission_.size(); ++i)
                    transmission_[i] += f[i];
                const double torque = f[static_cast<std::size_t>(t->driven_dof())];
                const std::string prefix =
                        t == clutch_.get() ? "crank_clutch" : "motor_freewheel";
                live_.last[prefix + "_torque_nm"] = torque;
                live_.last[prefix + "_engaged"] = torque > 1e-8;
                live_.last[prefix + "_dissipation_power_w"] =
                        std::max(0., torque * t->relative_rate(data_));
            }
        if (ideal_hub_) {
            const double torque =
                    transmission_[static_cast<std::size_t>(ideal_hub_->driven_dof())];
            live_.last["freehub_torque_nm"] = torque;
            live_.last["freehub_engaged"] = torque > 1e-8;
            if (config_.motor_clutch)
                live_.last["freehub_dissipation_power_w"] =
                        std::max(0., torque * ideal_hub_->relative_rate(data_));
            for (const auto &[k, v]: ideal_hub_->diagnostics())
                live_.last[k] = v;
        }
        if (!live_.pending_actuation)
            return transmission_;
        const auto p = *live_.pending_actuation;
        double torque = motor_actuator_ >= 0
                            ? buffer(data_->actuator_force,
                                     model_->nu)[static_cast<std::size_t>(motor_actuator_)]
                            : 0.;
        if (torque < -1e-10 || torque > p.requested + 1e-8)
            throw ArithmeticError(
                "solved motor effort violates the reserved effort ceiling");
        torque = std::max(torque, 0.);
        const auto &b = config_.policies.battery;
        const double power = motor_electrical_power(torque, p.omega, b.copper_w_per_nm2,
                                                    b.speed_w_per_rad_s2, b.idle_w,
                                                    p.enabled && torque > 0.);
        double delivered = power;
        if (b.enabled) {
            if (power * p.dt > battery_.state().energy_j +
                std::max(1e-10, battery_.state().energy_j * 1e-12))
                throw ArithmeticError(
                    "solved motor energy exceeds available battery storage");
            delivered = battery_.draw(power, p.dt);
        }
        live_.last["motor_torque_nm"] = torque;
        live_.last["motor_shaft_power_w"] = torque * p.omega;
        live_.last["electrical_power_w"] = delivered;
        live_.last["battery_energy_j"] = battery_.state().energy_j;
        live_.last["motor_enabled"] = p.enabled && torque > 0.;
        live_.last["battery_empty"] = battery_.state().energy_j == 0.;
        auto s = assist_.state();
        s.torque = torque;
        assist_.set_state(s);
        live_.pending_actuation.reset();
        return transmission_;
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
