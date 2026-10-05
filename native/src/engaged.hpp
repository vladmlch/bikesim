#pragma once
#include <optional>
#include <stdexcept>

// Unwraps an optional whose engagement is a state-machine invariant that
// the compiler cannot see locally (e.g. coasting mode implies phase_rad is
// engaged). Unlike unchecked `*opt`, a violated invariant throws
// std::logic_error; unlike `opt.value()`, the guard is visible to
// bugprone-unchecked-optional-access, so call sites stay warning-free.
template<typename T>
T &engaged(std::optional<T> &opt) {
    if (!opt.has_value()) throw std::logic_error("engagement invariant violated");
    return *opt;
}

template<typename T>
const T &engaged(const std::optional<T> &opt) {
    if (!opt.has_value()) throw std::logic_error("engagement invariant violated");
    return *opt;
}
