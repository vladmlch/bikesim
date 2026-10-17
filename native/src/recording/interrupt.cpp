#include "publish.hpp"
#include "../platform/signals.hpp"
#include "../research/lifecycle.hpp"

namespace bike::recording {
Publication finalize_guarded(RunCapture& capture, const std::filesystem::path& root,
                              const FinalizeOptions& options) {
    platform::InterruptShield shield;
    auto publication = finalize(capture, root, options);
    // The sealed path stays cached even when the deferred signal is delivered
    // to the owning frontend after its critical section. ResearchRun/ride
    // already shield seal+finalize together and call finalize directly.
    if (shield.deferred()) throw research::OperatorInterrupted{};
    return publication;
}
} // namespace bike::recording
