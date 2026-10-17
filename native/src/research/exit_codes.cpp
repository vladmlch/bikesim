#include "evidence.hpp"
namespace bike::research {
int episode_exit_code(const runtime::WireObject& status) {
    using namespace runtime::sample_wire;
    if (!is_none(required(status, "error"))) return 3;
    if (!boolean(required(status, "numerically_valid")) || !boolean(required(status, "model_valid"))) return 2;
    return 0;
}
} // namespace bike::research
