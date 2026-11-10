#include "kernel_test_support.hpp"
#include "support/fixture_test.hpp"
#include "drivetrain/motor.hpp"
#include "writers/tire_common.hpp"
#include "contact/laws.hpp"
#include "contact/tabulated_radial.hpp"
#include "config_validation.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numbers>

void kernel_properties() {
    for (double const slip : {-2., -.1, 0., .1, 2.}) {
        const auto brush = tirecommon::brush_step(0., slip, 2., 500., 10000., .8, .04, .001);
        bt::require(std::abs(brush.force) <= 400. + 1e-12 && brush.loss >= 0.,
            "shared tyre/contact brush respects friction and nonnegative dissipation");
    }
    const auto airborne = tirecommon::brush_step(.01, 1., 2., 0., 10000., .8, .04, .001);
    bt::require(airborne.force == 0. && airborne.xi == 0. && airborne.loss >= 0.,
        "airborne contact releases stored brush state without force");
    // Independent dyadic table: slopes 4 and 8, prefix area 1/8.
    const nativecfg::TireTable table{.deflection_m={0., .25, .5}, .force_n={0., 1., 3.}};
    const contactlaw::TabulatedRadial radial(table);
    bt::require(radial.elastic_response(.25) == std::pair{1., .125},
        "table knot has exact force and integrated area");
    bt::require(radial.elastic_response(.375) == std::pair{2., .3125},
        "table interpolation integrates its local affine force");
    bt::require(radial.elastic_response(.5) == std::pair{3., .625},
        "last knot belongs to the last closed material segment");
    // Decisions 22/39 (2026-11-10): the tabulated radial loss is the
    // rate-independent hysteresis η·F·tanh(δ̇/ε) — |η·F| < F keeps the
    // unilateral force positive even when the release rate saturates the
    // tanh, and the stored elastic energy is untouched.
    {
        const auto [unloaded, stored] = radial.normal_contact(.375, -2., .99, .01);
        bt::require(std::abs(unloaded - .02) < 1e-9 && stored == .3125,
            "hysteresis subtracts at most η·F: unloading never goes "
            "tensile or erases elastic energy");
    }
    bt::require(radial.elastic_response(-1.) == std::pair{0., 0.},
        "negative penetration is outside contact, with zero elastic energy");
    bt::invalid([&] { (void)radial.elastic_response(std::nextafter(.5, 1.)); });
    bt::invalid([&] { (void)contactlaw::TabulatedRadial({.deflection_m={0., .25, .25}, .force_n={0., 1., 2.}}); });
    bt::invalid([&] { (void)contactlaw::TabulatedRadial({.deflection_m={0., .25, .5}, .force_n={0., 2., 1.}}); });
    nativecfg::TireMaterial material{.loss_factor = .1, .hysteresis_rate_eps_mps = .01, .pressure_pa_gauge = 100000.,
        .provenance = "authored independent dyadic regression", .valid_load_range_n = {0., 3.},
        .table = table};
    nativecfg::validate(material);
    bt::require(nativecfg::is_load_in_valid_range(material, 0.) &&
                nativecfg::is_load_in_valid_range(material, 3.) &&
                !nativecfg::is_load_in_valid_range(material, -1.) &&
                !nativecfg::is_load_in_valid_range(material, 4.),
        "material applicability includes both endpoints without limiting force");
    bt::invalid([&] {
        (void)nativecfg::is_load_in_valid_range(material, std::numeric_limits<double>::quiet_NaN());
    });
    material.valid_load_range_n[1] = 4.;
    bt::invalid([&] { nativecfg::validate(material); });
    // Decision 22 + ban 39 (2026-11-10): the compliant_2d linear material's
    // radial law is rate-independent hysteresis
    // F = kδ + η·kδ·tanh(δ̇/ε), total clamped at 0, elastic energy ½kδ².
    // The three asserts below establish that zero deflection rate returns
    // the pure elastic force, the contact never pulls, and the energy the
    // law dissipates over a load cycle is set by η alone — not by the
    // cycle frequency (a viscous term would scale with ω).
    {
        const double k = 130000., eta = .0662, eps = .01;
        const auto [still_force, still_energy] =
                contactlaw::normal_contact_hysteresis(.005, 0., k, eta, eps);
        bt::require(still_force == 650. &&
                    still_energy == .5 * k * .005 * .005,
                    "zero deflection rate gives the pure elastic "
                    "force/energy");
        for (const double rate : {-100., -1., -.01, 0., .01, 1., 100.})
            for (const double delta : {1e-6, .005, .02})
                bt::require(contactlaw::normal_contact_hysteresis(
                                delta, rate, k, eta, eps).first >= 0.,
                            "normal force is never tensile");
        bt::require(contactlaw::normal_contact_hysteresis(
                            0., 1., k, eta, eps).first == 0. &&
                    contactlaw::normal_contact_hysteresis(
                            -.001, 1., k, eta, eps).first == 0.,
                    "outside contact there is no normal force");
        // Two cycles a decade apart in frequency (10 Hz vs 100 Hz) over
        // δ(t) = A·(1 + sin ωt) with analytic δ̇; the dissipated work per
        // cycle must agree within 1e-3 relative — the stated tolerance.
        // Both peak rates (0.63 and 6.3 m/s) sit far above ε, where the
        // tanh is saturated and the law is genuinely rate-independent;
        // quasi-static rates soften inside the ε-wide sign boundary, so
        // the assertion probes the rolling-wheel regime the law models.
        const double amplitude = .01;
        const auto cycle_loss = [&](double omega) {
            constexpr int n = 4001;
            double loss = 0.;
            const double dt =
                    2. * std::numbers::pi / (omega * (n - 1));
            for (int i = 0; i < n; ++i) {
                const double phase = 2. * std::numbers::pi * i / (n - 1);
                const double delta = amplitude * (1. + std::sin(phase));
                const double rate = amplitude * omega * std::cos(phase);
                const auto [force, energy] =
                        contactlaw::normal_contact_hysteresis(
                                delta, rate, k, eta, eps);
                loss += (force - k * delta) * rate * dt;
            }
            return loss;
        };
        const double slow = cycle_loss(20. * std::numbers::pi);
        const double fast = cycle_loss(200. * std::numbers::pi);
        bt::require(slow > 0. && fast > 0. &&
                    std::abs(fast - slow) <= 1e-3 * slow,
                    "hysteresis cycle loss is frequency-independent "
                    "(10 Hz vs 100 Hz within 1e-3 relative)");
    }
}
