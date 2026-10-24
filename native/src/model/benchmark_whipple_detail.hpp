#pragma once
#include <string>
namespace bike::model {struct WhippleParameters;}
namespace bike::model::detail {
// Internal builder dispatch; the public W00 benchmark declarations stay frozen.
std::string generate_whipple_mjcf(const WhippleParameters&);
}
