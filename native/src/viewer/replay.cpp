#include "session.hpp"
namespace bike::viewer {
int replay_exit_code(bool completed,bool passed) noexcept {return completed?(passed?0:1):2;}
RunResult run_replay(SessionHooks hooks,const runtime::ModelImage& image,ViewerOptions options,bool checked) {
    options.mode=checked?Mode::checked_replay:Mode::inspection;
    options.title=checked?"Checked replay":"Recording inspection (no numerical certification)";
    return run(std::move(hooks),image,options);
}
} // namespace bike::viewer
