// binding.cpp — nanobind FFI edge for the native Stepper.
//
// qpos/qvel/qacc/qfrc_constraint/ctrl (and qfrc_applied/actuator_force
// in drive_binding.cpp) return zero-copy read-only numpy views onto
// mjData's FIXED buffers — their addresses are stable for the mjData
// lifetime. The declared owner policy is nb::rv_policy::reference_internal:
// the array's base retains the Stepper, so a view can never outlive its
// model/data. Views are LIVE windows — contents change with each
// forward/step — re-read the property rather than caching the array.
// efc_force is the exception: d->efc_force is a per-frame constraint
// arena allocation whose address and length move whenever the contact
// set changes, so the property returns an OWNED snapshot copy instead
// (writeable; frozen at access time — see the prop for the policy).
#include <algorithm>
#include <array>
#include <cstddef>
#include <exception>
#include <ranges>
#include <span>
#include <stdexcept>
#include <vector>
#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/pair.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>
#include "stepper.hpp"
#include "config.hpp"
#include "binding_readers.hpp"
#include "binding_arrays.hpp"
#include "engine_call.hpp"
#include "diag.hpp"
#include "rider/contact_binding.hpp"
#include "drivetrain/policy_binding.hpp"
#include "drivetrain/drive_binding.hpp"
#include "writers/cruise.hpp"
#include "writers/resistance.hpp"
#include "writers/tire.hpp"

namespace nb = nanobind;

namespace {
    void translate_engine_failure(const std::exception_ptr &exception,
                                  void *python_class) {
        try {
            std::rethrow_exception(exception);
        } catch (const engine::EngineFailure &failure) {
            // A stock MuJoCo Python callback can set its original Python
            // error before raising mju_error. Keep that exception intact.
            if (!PyErr_Occurred())
                PyErr_SetString(static_cast<PyObject *>(python_class), failure.what());
        }
    }

    // Read-only numpy view over a Stepper span — shared by every view
    // prop_ro. The empty nb::handle() owner pairs with the declared
    // nb::rv_policy::reference_internal at each def site, which makes the
    // Stepper the array's owner (the view keeps its Stepper alive).
    nb::ndarray<nb::numpy, const double, nb::shape<-1> >
    as_view(std::span<const double> v) {
        return nb::ndarray<nb::numpy, const double, nb::shape<-1> >(
            v.data(), {v.size()}, nb::handle());
    }

    nb::ndarray<nb::numpy, double, nb::shape<-1>> as_owned(std::vector<double> values) {
        return wire::owned_array<double>(values);
    }

    nb::ndarray<nb::numpy, bool, nb::shape<-1>> as_owned_bool(std::vector<char> values) {
        return wire::owned_flags(values);
    }

    // One TireSnapshot as a dict carrying BOTH consumers' schemas: the
    // manifest's readable digest keys (what the artifact serializes in
    // tools/golden_episode._tire_snapshot_row) and the flat-array keys
    // resistance_components's side_input reads.
    nb::dict snapshot_dict(const TireSnapshot &s) {
        nb::dict d;
        d["time_s"] = s.time_s;
        d["interval_id"] = s.interval_id;
        d["backend"] = s.backend;
        d["geometric_contact"] = s.geometric_contact;
        d["effective_radius_m"] = s.effective_radius_m;
        std::vector<double> loads;
        std::vector<char> working;
        loads.reserve(s.patches.size());
        working.reserve(s.patches.size());
        nb::list patches;
        for (const TirePatch &p: s.patches) {
            loads.push_back(p.normal_load_n);
            working.push_back(static_cast<char>(p.working_surface));
            nb::dict pd;
            pd["normal_load_n"] = p.normal_load_n;
            pd["tangent_force_n"] = p.tangent_force_n;
            pd["slip_mps"] = p.slip_mps;
            pd["working_surface"] = p.working_surface;
            patches.append(pd);
        }
        d["patches"] = std::move(patches);
        d["patch_loads"] = as_owned(std::move(loads));
        d["patch_working"] = as_owned_bool(std::move(working));
        d["eff_radius"] = s.effective_radius_m;
        return d;
    }

    // One tire-snapshot side, flat-array schema:
    // {'patch_loads': f64[n], 'patch_working': bool[n], 'eff_radius': float}.
    // Dtype and layout conversions are accepted. Keep their ndarray owners alive
    // through the writer call: a span alone cannot retain a conversion temporary.
    struct OwnedTireSideInput {
        nb::ndarray<const double, nb::shape<-1>, nb::c_contig> loads;
        nb::ndarray<const bool, nb::shape<-1>, nb::c_contig> working;
        double effective_radius_m;

        [[nodiscard]] TireSideInput view() const {
            return {
                .patch_loads = std::views::counted(loads.data(),
                                                   static_cast<std::ptrdiff_t>(loads.size())),
                .patch_working = std::views::counted(working.data(),
                                                     static_cast<std::ptrdiff_t>(working.size())),
                .effective_radius_m = effective_radius_m
            };
        }
    };

    OwnedTireSideInput side_input(const nb::dict &snaps, const char *side) {
        if (!snaps.contains(side))
            throw std::invalid_argument(
                "resistance_components: missing side '" + std::string(side) +
                "'");
        const wire::Dict root{.value = snaps, .path = "resistance_components"};
        const auto parsed = wire::section(root, side);
        constexpr auto required = wire::keys("patch_loads", "patch_working", "eff_radius");
        constexpr auto optional = wire::keys("time_s", "interval_id", "backend", "geometric_contact", "effective_radius_m", "patches");
        wire::exact(parsed, required, optional);
        if (parsed.contains("patches")) {
            const auto patches = wire::sequence(parsed["patches"], parsed.child("patches"));
            std::size_t index = 0;
            for (nb::handle const value: patches) {
                const std::string patch_path = parsed.child("patches") + "." + std::to_string(index++);
                const wire::Dict patch{.value = wire::mapping(value, patch_path), .path = patch_path};
                wire::exact(patch, wire::keys("normal_load_n", "tangent_force_n", "slip_mps", "working_surface"));
                for (const char *key: {"normal_load_n", "tangent_force_n", "slip_mps"})
                    static_cast<void>(wire::finite_real(patch[key], patch.child(key)));
                static_cast<void>(wire::boolean(patch["working_surface"], patch.child("working_surface")));
            }
        }
        for (const char *key: {"time_s", "effective_radius_m"})
            if (parsed.contains(key)) static_cast<void>(wire::finite_real(parsed[key], parsed.child(key)));
        if (parsed.contains("interval_id")) static_cast<void>(wire::integer(parsed["interval_id"], parsed.child("interval_id")));
        if (parsed.contains("backend")) static_cast<void>(wire::string(parsed["backend"], parsed.child("backend")));
        if (parsed.contains("geometric_contact")) static_cast<void>(wire::boolean(parsed["geometric_contact"], parsed.child("geometric_contact")));
        const nb::dict d = parsed.value;
        const auto arr = [&d, side](const char *key) -> nb::object {
            if (!d.contains(key))
                throw std::invalid_argument(
                    "resistance_components: '" + std::string(side) + "." + key +
                    "' is required");
            return d[key];
        };
        using F64 = nb::ndarray<const double, nb::shape<-1>, nb::c_contig>;
        using B1 = nb::ndarray<const bool, nb::shape<-1>, nb::c_contig>;
        const auto raw_loads = arr("patch_loads");
        static_cast<void>(wire::vector(raw_loads, parsed.child("patch_loads")));
        F64 loads;
        try { loads = nb::cast<F64>(raw_loads); }
        catch (const nb::cast_error &) { wire::invalid(parsed.child("patch_loads"), "expected 1-D numeric array"); }
        B1 working;
        try { working = nb::cast<B1>(arr("patch_working")); }
        catch (const nb::cast_error &) { wire::invalid(parsed.child("patch_working"), "expected 1-D flag array"); }
        return {.loads = loads, .working = working, .effective_radius_m = wire::finite_real(arr("eff_radius"), parsed.child("eff_radius"))};
    }
} // namespace

// Binding glue builds 15-20KB frames of nanobind temporaries at import
// time; the 8KB frame guard stays armed for the per-step code.
NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
NB_MODULE(bike_native, m) {
    nb::object fatal_error = nb::module_::import_("mujoco").attr("FatalError");
    m.attr("FatalError") = fatal_error;
    // Nanobind retains the translator for the interpreter lifetime; keep its
    // payload alive even if module attributes are changed by Python code.
    Py_INCREF(fatal_error.ptr());
    nb::register_exception_translator(&translate_engine_failure,
                                      fatal_error.ptr());
    bind_drive_policies(m);
    auto stepper_class = nb::class_<Stepper>(m, "Stepper");
    // Isolated fatal-path test hook. It restores the model option before the
    // exception reaches Python, while Stepper retains its poisoned state.
    m.def("_engine_test_forward_failure", [](Stepper &stepper) {
        stepper.mutate([&] {
            mjModel *model = stepper.model();
            const int previous_solver = model->opt.solver;
            model->opt.solver = 99;
            try {
                stepper.forward();
            } catch (...) {
                model->opt.solver = previous_solver;
                throw;
            }
            model->opt.solver = previous_solver;
        });
    });
    m.def("_engine_test_set_const_failure", [](Stepper &stepper) {
        stepper.mutate([&] {
            mjModel *model = stepper.model();
            if (model->nbody < 2)
                throw std::invalid_argument("setConst test needs a nonworld body");
            const auto ipos = std::views::counted(model->body_ipos, 3 * model->nbody);
            const auto simple = std::views::counted(model->body_simple, model->nbody);
            const auto sameframe = std::views::counted(model->body_sameframe, model->nbody);
            const auto previous_ipos = ipos[3];
            const auto previous_simple = simple[1];
            const auto previous_sameframe = sameframe[1];
            simple[1] = 1;
            ipos[3] = previous_ipos + .123;
            try {
                engine::set_const(model, stepper.data());
            } catch (...) {
                ipos[3] = previous_ipos;
                simple[1] = previous_simple;
                sameframe[1] = previous_sameframe;
                throw;
            }
            ipos[3] = previous_ipos;
            simple[1] = previous_simple;
            sameframe[1] = previous_sameframe;
        });
    });
    m.def("_engine_test_try_status", [](Stepper &stepper, bool step) {
        return stepper.mutate([&] {
            stepper.refresh_time_callback_policy();
            mjModel *model = stepper.model();
            const int previous_solver = model->opt.solver;
            model->opt.solver = 99;
            engine::ErrorBuffer error;
            const bool succeeded = step ? stepper.try_step(error)
                                        : stepper.try_forward(error);
            model->opt.solver = previous_solver;
            return std::pair{succeeded, std::string(error.message.data())};
        });
    });
    stepper_class
            .def(nb::init<const std::string &>())
            .def(nb::init<const std::string &, nb::handle>(),
                 nb::arg("mjb_path"), nb::arg("config").none())
            .def("step", &Stepper::step)
            .def("forward", &Stepper::forward)
            .def("reset", &Stepper::reset)
            // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) fixed Python state-array signature
            .def("set_state", [](Stepper &s, nb::handle qpos, nb::handle qvel,
                                 nb::handle act, nb::handle warmstart, nb::handle time) {
                const auto qp = wire::vector(qpos, "set_state.qpos");
                const auto qv = wire::vector(qvel, "set_state.qvel");
                const auto a = wire::vector(act, "set_state.act");
                const auto w = wire::vector(warmstart, "set_state.warmstart");
                s.set_state(qp, qv, a, w, wire::finite_real(time, "set_state.time"));
            }, nb::arg("qpos").none(), nb::arg("qvel").none(), nb::arg("act").none(),
               nb::arg("warmstart").none(), nb::arg("time").none())
            .def_prop_ro("qpos", [](Stepper &s) { return as_view(s.qpos()); },
                         nb::rv_policy::reference_internal)
            .def_prop_ro("qvel", [](Stepper &s) { return as_view(s.qvel()); },
                         nb::rv_policy::reference_internal)
            .def_prop_ro("qacc", [](Stepper &s) { return as_view(s.qacc()); },
                         nb::rv_policy::reference_internal)
            .def_prop_ro("qfrc_constraint", [](Stepper &s) {
                return as_view(s.qfrc_constraint());
            }, nb::rv_policy::reference_internal)
            // Owned snapshot, NOT a view: efc_force points into mjData's
            // constraint arena, which rewinds and reallocates its offset
            // whenever the contact set changes — a view would silently go
            // stale (or dangle) after the next forward. The capsule-owned
            // copy is frozen at access time and writeable; the capsule is
            // the owner, so automatic_reference keeps it (reference_internal
            // would reject an ndarray that already has an owner).
            .def_prop_ro("efc_force", [](Stepper &s) {
                return wire::owned_array<double>(s.efc_force());
            }, nb::rv_policy::automatic_reference)
            .def_prop_ro("ctrl", [](Stepper &s) { return as_view(s.ctrl()); },
                         nb::rv_policy::reference_internal)
            // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) fixed front/rear Python signature
            .def("brake_torques", [](Stepper &s, nb::handle front, nb::handle rear) {
                const double checked_front = wire::finite_real(front, "brake_torques.front_demand");
                const double checked_rear = wire::finite_real(rear, "brake_torques.rear_demand");
                return s.brake_torques(checked_front, checked_rear);
            }, nb::arg("front_demand").none(), nb::arg("rear_demand").none())
            // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) fixed front/rear Python signature
            .def("apply_brake", [](Stepper &s, nb::handle front, nb::handle rear) {
                const double checked_front = wire::finite_real(front, "apply_brake.front_demand");
                const double checked_rear = wire::finite_real(rear, "apply_brake.rear_demand");
                s.apply_brake(checked_front, checked_rear);
            }, nb::arg("front_demand").none(), nb::arg("rear_demand").none())
            .def("cruise_compute", &Stepper::cruise_compute,
                 nb::arg("rear_in_contact"), nb::arg("traction_limited") = false,
                 nb::arg("controller_grounded") = nb::none())
            .def("cruise_reset", &Stepper::cruise_reset)
            .def("cruise_set_target_speed", [](Stepper &s, nb::handle value) {
                s.cruise_set_target_speed(wire::finite_real(value, "cruise_set_target_speed.value_kmh"));
            }, nb::arg("value_kmh").none())
            .def("cruise_set_assist_compensation",
                 [](Stepper &s, nb::handle value) {
                     return s.cruise_set_assist_compensation(wire::finite_real(value, "cruise_set_assist_compensation.support_factor"));
                 }, nb::arg("support_factor").none())
            .def("cruise_state", [](const Stepper &s) {
                const CruiseState state = s.cruise_state();
                nb::dict out;
                out["target_speed_mps"] = state.target_speed_mps;
                out["integral_mps_s"] = state.integral_mps_s;
                out["torque_nm"] = state.torque_nm;
                out["engaged"] = state.engaged;
                out["gain_scale"] = state.gain_scale;
                return out;
            })
            .def("set_cruise_state", [](Stepper &s, nb::handle raw) {
                const nb::dict state = wire::mapping(raw, "cruise_state");
                // Disabled controller errors precede candidate parsing, as for
                // the other controller APIs. All reads finish before mutation.
                static_cast<void>(s.cruise_state());
                wire::exact_keys(state, wire::keys("target_speed_mps", "integral_mps_s", "torque_nm", "engaged", "gain_scale"), {}, "cruise_state");
                if (state.size() != 5)
                    throw std::invalid_argument(
                        "cruise state requires exactly target_speed_mps, "
                        "integral_mps_s, torque_nm, engaged, gain_scale");
                const auto number = [&state](const char *key) {
                    try {
                        return nativecfg::detail::req_f64(state, "cruise_state", key);
                    } catch (const nb::cast_error &) {
                        throw std::invalid_argument(std::string(key) + " must be numeric");
                    }
                };
                CruiseState candidate{.target_speed_mps = number("target_speed_mps")};
                candidate.integral_mps_s = number("integral_mps_s");
                candidate.torque_nm = number("torque_nm");
                const nb::object engaged =
                        nativecfg::detail::req(state, "cruise_state", "engaged");
                if (!nb::isinstance<nb::bool_>(engaged))
                    throw std::invalid_argument("engaged must be bool");
                candidate.engaged = nb::cast<bool>(engaged);
                candidate.gain_scale = number("gain_scale");
                s.set_cruise_state(candidate);
            }, nb::arg("state").none())
            .def("resistance_components", [](Stepper &s, nb::handle raw) {
                     const nb::dict snaps = wire::mapping(raw, "resistance_components");
                     // Named locals in Python's dict order — arg eval order is
                     // unspecified, so on malformed input the 'front' error must win.
                     wire::exact_keys(snaps, wire::keys("front", "rear"), {}, "resistance_components");
                     const OwnedTireSideInput front = side_input(snaps, "front");
                     const OwnedTireSideInput rear = side_input(snaps, "rear");
                     nb::dict out;
                     for (auto &[name, vec]:
                          s.resistance_components(front.view(), rear.view()))
                         out[name.c_str()] = as_owned(std::move(vec));
                     return out;
                 }, nb::arg("snapshots"),
                 "Compute resistance from snapshot arrays; 1-D loads/working arrays "
                 "may be converted to contiguous float64/bool for this call.")
            .def("suspension_components", [](Stepper &s) {
                nb::dict out;
                for (auto &[name, vec]: s.suspension_components())
                    out[name.c_str()] = as_owned(std::move(vec));
                return out;
            })
            .def("rider_forces_qfrc", [](Stepper &s) {
                return as_owned(s.rider_forces_qfrc());
            })
            .def("total", [](Stepper &s, nb::handle raw) {
                     const nb::dict components = wire::mapping(raw, "total");
                     // Insertion order is the contract: PyDict_Keys preserves it,
                     // and the fold inside Stepper::total mirrors
                     // ForceAccumulator.total() exactly. Nanobind may convert a 1-D
                     // ndarray's dtype/layout to contiguous float64. Copy each array
                     // into vecs while its owner is alive; reject wrong-rank or
                     // non-convertible values before the fold.
                     std::vector<std::vector<double> > vecs;
                     const nb::list keys = components.keys();
                     vecs.reserve(keys.size());
                     for (nb::handle const key: keys) {
                         const std::string name = wire::string(key, "total");
                         const nb::object value = components[key];
                         vecs.push_back(wire::vector(value, "total." + name));
                     }
                     return as_owned(s.total(vecs));
                 }, nb::arg("components"),
                 "Sum finite nv-wide 1-D arrays in dict insertion order; ndarray "
                 "dtype/layout conversion to contiguous float64 is accepted.")
            // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) fixed Python state-array signature
            .def("set_tire_state", [](Stepper &s, nb::handle raw_names, nb::handle raw_row) {
                const auto sequence = wire::sequence(raw_names, "set_tire_state.names");
                std::vector<std::string> names;
                names.reserve(sequence.size());
                for (nb::handle const value: sequence) names.push_back(wire::string(value, "set_tire_state.names"));
                const auto row_values = wire::sequence(raw_row, "set_tire_state.row");
                std::vector<double> row;
                row.reserve(row_values.size());
                // Tire state owns the named sentinel domain; the wire reader checks original types only.
                for (nb::handle const value: row_values) row.push_back(wire::real(value, "set_tire_state.row"));
                s.set_tire_state(names, row);
            }, nb::arg("names").none(), nb::arg("row").none())
            .def("tire_state", [](Stepper &s) {
                return as_owned(s.tire_state());
            })
            .def_prop_ro("tire_state_names", [](Stepper &s) {
                return s.tire_state_names();
            })
            .def("tire_qfrc", [](Stepper &s, nb::handle dt) {
                return as_owned(s.tire_qfrc(wire::finite_real(dt, "tire_qfrc.dt")));
            })
            .def("tire_snapshots", [](Stepper &s) {
                const TireWriter *t = s.tire();
                if (!t)
                    throw std::logic_error(
                        "tire writer: Stepper was built without a tire config "
                        "(pass the dict from tools.native_config.project)");
                nb::dict out;
                const auto &snaps = t->snapshots();
                const std::array<const char *, 2> sides = {"front", "rear"};
                for (std::size_t i = 0; i < 2; ++i)
                    if (snaps[i])
                        out[sides[i]] = snapshot_dict(*snaps[i]);
                return out;
            })
            .def("tire_diagnostics", [](Stepper &s) {
                const TireWriter *t = s.tire();
                if (!t)
                    throw std::logic_error(
                        "tire writer: Stepper was built without a tire config "
                        "(pass the dict from tools.native_config.project)");
                nb::dict out;
                const auto &diags = t->diagnostics();
                const std::array<const char *, 2> sides = {"front", "rear"};
                for (std::size_t i = 0; i < 2; ++i) {
                    if (!diags[i])
                        continue;
                    const TireDiagnostics &g = *diags[i];
                    nb::dict d;
                    d["multi_support"] = g.multi_support;
                    d["penetration_m"] = g.penetration_m;
                    d["normal_speed_mps"] = g.normal_speed_mps;
                    d["slip_mps"] = g.slip_mps;
                    d["normal_load_n"] = g.normal_load_n;
                    d["tangent_force_n"] = g.tangent_force_n;
                    d["friction_coefficient"] = g.friction_coefficient;
                    d["surface"] = g.surface;
                    d["branch_release_loss_j"] = g.branch_release_loss_j;
                    d["brush_loss_j"] = g.brush_loss_j;
                    d["radial_energy_j"] = g.radial_energy_j;
                    d["shear_energy_j"] = g.shear_energy_j;
                    d["outside_material_load_range"] =
                            g.outside_material_load_range;
                    out[sides[i]] = d;
                }
                return out;
            })
            .def_prop_ro("time", &Stepper::time);
    bind_drivetrain(m, stepper_class);
    bind_rider_contact_math(m);
}

NATIVE_DIAG_POP
