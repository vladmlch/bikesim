#include "session.hpp"
#include "pacer.hpp"
#include "../research/policy_session.hpp"

namespace bike::viewer {
RunResult run_research(research::PolicySession& session,const ViewerOptions& supplied,const LifecycleHooks& lifecycle) {
    auto options=supplied;options.mode=Mode::research;
    SessionHooks hooks;
    hooks.frame=[&]{return session.environment().render_frame();};
    hooks.terminal=[&]{return session.done();};
    hooks.advance=[&](std::int64_t target,double budget){
        if(session.done())return;
        if(!session.pending()) {if(session.paused())return;session.begin_advance();}
        (void)session.advance_pending(budget,target);
        if(lifecycle.after_advance)lifecycle.after_advance();
    };
    hooks.stop=[&]{session.request_stop();while(session.pending())(void)session.advance_pending(kComputeSliceS);};
    hooks.pause=[&](bool paused){if(paused){session.pause();while(session.pending())(void)session.advance_pending(kComputeSliceS);}else session.resume();};
    hooks.brakes=[&](double front,double rear){session.set_brakes(front,rear);};
    hooks.reset=[&]{while(session.pending())(void)session.advance_pending(kComputeSliceS);if(lifecycle.before_reset)lifecycle.before_reset();session.reset();if(lifecycle.after_reset)lifecycle.after_reset();};
    hooks.operator_event=lifecycle.operator_event;
    hooks.close_requested=lifecycle.close_requested;
    return run(std::move(hooks),session.environment().model_image(),options);
}
} // namespace bike::viewer
