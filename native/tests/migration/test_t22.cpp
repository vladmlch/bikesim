// T22 authored regressions. Execution and oracle qualification are separate
// gates; none was run in the user's no-build/no-run implementation session.
#include "support/migration_test.hpp"
#include "diag.hpp"
#include "initialization/equilibrium.hpp"
#include "initialization/setup_owner.hpp"

#include <cmath>
#include <limits>
#include <string>

NATIVE_DIAG_PUSH
// NATIVE_DIAG_IGNORE is the project diagnostic-pragma macro from diag.hpp; cppcheck cannot resolve it.
// cppcheck-suppress unknownMacro
NATIVE_DIAG_IGNORE("-Wframe-larger-than")

namespace {
namespace init=bike::initialization;

[[gnu::noinline]] void budgets() {
    const init::EquilibriumOptions defaults;
    bt::require(init::equilibrium_step_budget(.0005,defaults)==160000,
                "physical equilibrium budgets eighty seconds");
    bt::require(init::equilibrium_step_budget(.001,defaults)==80000,
                "budget scales with the selected physical timestep");
    bt::require(init::equilibrium_step_budget(.004,defaults)==40000,
                "historical forty-thousand-step floor is retained");
    bt::require(init::equilibrium_cycle_steps(.0005)==40 &&
                init::equilibrium_cycle_steps(.001)==20 &&
                init::equilibrium_cycle_steps(.1)==1,
                "relaxation windows remain twenty milliseconds with a one-step floor");
    init::EquilibriumOptions explicit_budget;explicit_budget.max_steps=7;
    bt::require(init::equilibrium_step_budget(.0005,explicit_budget)==7,
                "an explicit step budget has priority");
    explicit_budget.max_steps=0;
    bt::invalid([&]{(void)init::equilibrium_step_budget(.0005,explicit_budget);});
    explicit_budget.max_steps=-1;
    bt::invalid([&]{(void)init::equilibrium_step_budget(.0005,explicit_budget);});
    for(double invalid:{0.,-1.,std::numeric_limits<double>::infinity(),
                        std::numeric_limits<double>::quiet_NaN()}) {
        bt::invalid([&]{(void)init::equilibrium_step_budget(invalid,defaults);});
        bt::invalid([&]{(void)init::equilibrium_cycle_steps(invalid);});
        init::EquilibriumOptions bad;bad.tolerance=invalid;
        bt::invalid([&]{(void)init::equilibrium_step_budget(.0005,bad);});
    }
}

[[gnu::noinline]] void convergence_and_failure() {
    auto request=init::read_setup_request(bt::fixture("requests/welded_flat.toml"));
    request.model.physics.value().equilibrium_cache_enabled=false; // NOLINT(bugprone-unchecked-optional-access) optional engagement is the fixture/request contract under test; .value() reports violations loudly
    request.model.physics.value().equilibrium_state_file.reset(); // NOLINT(bugprone-unchecked-optional-access) optional engagement is the fixture/request contract under test; .value() reports violations loudly
    request.initial_state.reset();
    request.model.physics.value().initial_speed_mps=0.; // NOLINT(bugprone-unchecked-optional-access) optional engagement is the fixture/request contract under test; .value() reports violations loudly
    auto insufficient=init::make_setup(request);
    init::EquilibriumOptions short_budget;short_budget.max_steps=1;
    short_budget.tolerance=1e-12;
    bool failed=false;
    try {(void)init::settle(*insufficient,short_budget);}
    catch(const init::EquilibriumFailure& error) {
        failed=true;
        bt::require(error.steps==1 && error.residual_qacc>short_budget.tolerance,
                    "budget exhaustion reports the actually measured residual");
        bt::require(!error.worst_accelerations.empty() &&
                    std::string(error.what()).find("did not converge")!=std::string::npos,
                    "failed equilibrium preserves named acceleration diagnostics");
    }
    bt::require(failed,"an unconverged solve cannot return success");
    bt::invalid([&]{(void)insufficient->seal_bootstrap();});

    auto owner=init::make_setup(request);
    const auto result=init::settle(*owner);
    bt::require(result.steps>0 && std::isfinite(result.residual_qacc) &&
                result.residual_qacc<=.05 && !result.cache_hit,
                "fresh physical equilibrium passes the actual zero-velocity acceptance gate");
    bt::require(result.static_front_load_n>0. && result.static_rear_load_n>0.,
                "finalized equilibrium exposes both static wheel loads");
    const auto transferred=owner->capture_initial_state();
    bt::require(transferred.sha256().size()==64,
                "completed t=0 state can be captured before sealing");
    (void)owner->seal_bootstrap();
    bt::invalid([&]{(void)init::settle(*owner);});
}

[[gnu::noinline]] void slope_brake_and_crank() {
    auto request=init::read_setup_request(bt::fixture("requests/welded_flat.toml"));
    request.model.physics.value().equilibrium_cache_enabled=false; // NOLINT(bugprone-unchecked-optional-access) optional engagement is the fixture/request contract under test; .value() reports violations loudly
    request.model.physics.value().equilibrium_state_file.reset(); // NOLINT(bugprone-unchecked-optional-access) optional engagement is the fixture/request contract under test; .value() reports violations loudly
    request.initial_state.reset();
    request.model.physics.value().initial_speed_mps=0.; // NOLINT(bugprone-unchecked-optional-access) optional engagement is the fixture/request contract under test; .value() reports violations loudly
    request.model.physics.value().initial_front_brake=1.; // NOLINT(bugprone-unchecked-optional-access) optional engagement is the fixture/request contract under test; .value() reports violations loudly
    request.model.physics.value().initial_rear_brake=1.; // NOLINT(bugprone-unchecked-optional-access) optional engagement is the fixture/request contract under test; .value() reports violations loudly
    request.model.physics.value().drive.crank_phase_rad=.75; // NOLINT(bugprone-unchecked-optional-access) optional engagement is the fixture/request contract under test; .value() reports violations loudly
    request.track.grade_profile=bike::terrain::GradeProfile{{{0.,.04},{request.track.length_m,.04}}};
    request.model.field=bike::terrain::HeightFieldSpec::for_track(request.track);
    auto owner=init::make_setup(request);const auto result=init::settle(*owner);
    bt::require(result.residual_qacc<=.05 && result.initial_crank_phase_rad==.75 &&
                std::isfinite(result.settled_crank_phase_rad),
                "graded standstill with commanded brakes retains crank initialization and physical acceptance");
    bt::require(result.static_front_load_n>0. && result.static_rear_load_n>0. &&
                std::isfinite(result.pitch_rad),
                "slope solve measures both supported wheels and an actual settled chassis angle");
}
} // namespace

int main(){return bt::run([]{budgets();convergence_and_failure();slope_brake_and_crank();});}
NATIVE_DIAG_POP
