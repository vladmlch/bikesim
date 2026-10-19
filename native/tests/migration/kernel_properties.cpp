#include "kernel_test_support.hpp"
#include "support/migration_test.hpp"
#include "drivetrain/chain.hpp"
#include "drivetrain/motor.hpp"
#include "writers/tire_common.hpp"
#include "contact/tabulated_radial.hpp"
#include "config_validation.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

void kernel_properties() {
    double previous_energy = 0.;
    for (double const extension : {0., .001, .01, .1}) {
        const auto response = drivetrain::chain_tension(extension, 0., 2000., 40.);
        bt::require(response.first >= 0. && response.second >= previous_energy,
            "unilateral chain cannot push and stored energy grows with extension");
        previous_energy = response.second;
    }
    for (double const request : {0., 1., 20., 100.}) {
        for (double const omega : {-3., 0., 7.}) {
            for (double const budget : {0., 1., 50., 1000.}) {
                const double torque = drivetrain::limit_torque_by_energy(request, omega, .3, .2, 1., budget);
                bt::require(torque >= 0. && torque <= request,
                    "energy-limited motor torque stays within the requested interval");
                if (torque > 0.) {
                    const double used = drivetrain::motor_electrical_power(torque, omega, .3, .2, 1., true);
                    bt::require(used <= budget + 1e-10,
                        "delivered motor power fits the independently checked energy budget");
                }
            }
        }
    }
    for (double const slip : {-2., -.1, 0., .1, 2.}) {
        const auto brush = tirecommon::brush_step(0., slip, 2., 500., 10000., .8, .04, .001);
        bt::require(std::abs(brush.force) <= 400. + 1e-12 && brush.loss >= 0.,
            "shared tyre/contact brush respects friction and nonnegative dissipation");
    }
    const auto airborne = tirecommon::brush_step(.01, 1., 2., 0., 10000., .8, .04, .001);
    bt::require(airborne.force == 0. && airborne.xi == 0. && airborne.loss >= 0.,
        "airborne contact releases stored brush state without force");
    drivetrain::Battery battery(20.);
    bt::invalid([&] { (void)battery.debit(1., -1.); });
    bt::require(battery.state().energy_j == 20. && battery.state().drawn_energy_j == 0.,
        "invalid debit cannot change energy");
    bt::invalid([&] {
        (void)drivetrain::chain_tension(std::numeric_limits<double>::quiet_NaN(), 0., 1., 0.);
    });

    // Independent dyadic table: slopes 4 and 8, prefix area 1/8.
    const nativecfg::TireTable table{.deflection_m={0., .25, .5}, .force_n={0., 1., 3.}};
    const contactlaw::TabulatedRadial radial(table);
    bt::require(radial.elastic_response(.25) == std::pair{1., .125},
        "table knot has exact force and integrated area");
    bt::require(radial.elastic_response(.375) == std::pair{2., .3125},
        "table interpolation integrates its local affine force");
    bt::require(radial.elastic_response(.5) == std::pair{3., .625},
        "last knot belongs to the last closed material segment");
    bt::require(radial.normal_contact(.375, -2., 2.) == std::pair{0., .3125},
        "unloading cannot create tensile normal force or erase elastic energy");
    bt::require(radial.elastic_response(-1.) == std::pair{0., 0.},
        "negative penetration is outside contact, with zero elastic energy");
    bt::invalid([&] { (void)radial.elastic_response(std::nextafter(.5, 1.)); });
    bt::invalid([&] { (void)contactlaw::TabulatedRadial({.deflection_m={0., .25, .25}, .force_n={0., 1., 2.}}); });
    bt::invalid([&] { (void)contactlaw::TabulatedRadial({.deflection_m={0., .25, .5}, .force_n={0., 2., 1.}}); });
    nativecfg::TireMaterial material{.radial_c_ns_m = 2., .pressure_pa_gauge = 100000.,
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
}
