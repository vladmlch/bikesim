#pragma once
#include "response_allocation.hpp"
#include "../io/value.hpp"

namespace bike::rider {
[[nodiscard]] ResponseAllocation decode_response_allocation(const io::Value&,std::string_view path);
[[nodiscard]] io::Value encode_response_allocation(const ResponseAllocation&);
} // namespace bike::rider
