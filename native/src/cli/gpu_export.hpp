#pragma once

#include <iosfwd>

namespace bike::cli {

[[nodiscard]] int run_gpu_export(int argc, const char *const *argv,
                                 std::ostream &out, std::ostream &err);

} // namespace bike::cli
