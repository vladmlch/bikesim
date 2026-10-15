// config.hpp — the nanobind adapter side of the config bridge (task T07).
//
// The typed result (nativecfg::WriterConfig), the section/key tables and
// the whole decode live in writer_config.hpp — Python-free and shared
// with the owned bike::io::Value boundary. This header supplies only the
// nanobind node: NbNode delegates every scalar/sequence/mapping read to
// the existing wire:: readers so the Python-facing strictness
// (numbers.Real/Integral membership, PyNumber_Index, ndarray bool
// rejection, MemoryError passthrough) is unchanged, while the checks the
// reads feed are the same code the Value decode runs.
//
// An empty dict leaves every writer disabled (the Stepper(path) contract
// stays intact); native_config_from_dict mirrors decode(Value)'s empty
// object → empty WriterConfig rule so no caller needs its own check.
// Every missing
// required key or unknown key raises std::invalid_argument naming the
// path — nanobind's default exception translator surfaces it as
// ValueError. Physical invariants remain enforced by each typed writer
// constructor through the shared nativecfg::validate.
#pragma once

#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <string_view>

#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include "binding_readers.hpp"
#include "writer_config.hpp"

namespace nb = nanobind;

namespace nativecfg {
    namespace detail {
        // The nanobind flavor of the shared decode node (writer_config.hpp
        // documents the protocol). Every read delegates to the existing
        // wire:: reader, preserving the nb-side diagnostics verbatim.
        struct NbNode {
            nb::object value;

            [[nodiscard]] bool is_none() const { return value.is_none(); }
            [[nodiscard]] bool is_mapping() const {
                return nb::isinstance<nb::dict>(value);
            }
            [[nodiscard]] bool contains(std::string_view key) const {
                return nb::borrow<nb::dict>(value).contains(
                    nb::str(key.data(), key.size()));
            }
            [[nodiscard]] NbNode member(std::string_view key) const {
                return {nb::object(
                    nb::borrow<nb::dict>(value)[nb::str(key.data(),
                                                       key.size())])};
            }
            // const& — the sink is invoked once per member, so it must
            // stay invocable for the whole iteration (no forwarding).
            template <class F>
            void each_key(std::string_view path, const F &f) const {
                for (auto const item: nb::borrow<nb::dict>(value))
                    f(wire::string(item.first, path));
            }
            [[nodiscard]] bool boolean(std::string_view path) const {
                return wire::boolean(value, path);
            }
            [[nodiscard]] double real(std::string_view path) const {
                return wire::real(value, path);
            }
            [[nodiscard]] double finite_real(std::string_view path) const {
                return wire::finite_real(value, path);
            }
            [[nodiscard]] std::optional<double> optional_real(
                std::string_view path) const {
                return wire::optional_real(value, path);
            }
            [[nodiscard]] std::int64_t integer(std::string_view path) const {
                return wire::integer(value, path);
            }
            [[nodiscard]] int integer32(std::string_view path) const {
                return wire::integer32(value, path);
            }
            [[nodiscard]] std::string string(std::string_view path) const {
                return wire::string(value, path);
            }
            [[nodiscard]] Seq<NbNode> sequence(std::string_view path) const {
                return {{nb::borrow<nb::object>(
                    wire::sequence(value, path))}};
            }
            [[nodiscard]] std::size_t seq_size() const {
                return nb::borrow<nb::list>(value).size();
            }
            [[nodiscard]] NbNode seq_at(std::size_t index) const {
                return {nb::object(nb::borrow<nb::list>(value)[index])};
            }
            void numeric_gate(std::string_view path) const {
                wire::numeric_array_type(value, path);
            }
        };

        // Legacy 3-arg spellings for binding.cpp's non-config dict reads
        // (the cruise_state setter); new code uses the Dict<N> readers.
        inline nb::object req(const nb::dict &d, const char *section,
                              const char *key) {
            if (!d.contains(key))
                throw std::invalid_argument(
                    "native config: missing key '" + std::string(section) +
                    "." + key + "'");
            return d[key];
        }

        inline std::string path(const char *s, const char *k) {
            return std::string(s).starts_with("config")
                       ? std::string(s) + "." + k
                       : "config." + std::string(s) + "." + k;
        }

        inline double req_f64(const nb::dict &d, const char *s,
                              const char *k) {
            return wire::finite_real(req(d, s, k), path(s, k));
        }
    } // namespace detail

    // Whole-config reader: an empty dict disables every writer (the
    // decode(Value) empty-object rule, mirrored); a non-empty one must
    // name the schema and may carry each writer's section — the shared
    // decode_root.
    inline WriterConfig native_config_from_dict(const nb::dict &d) {
        if (d.empty()) return {};
        return detail::decode_root(
            detail::Dict<detail::NbNode>{
                .value = detail::NbNode{nb::borrow<nb::object>(d)},
                .path = "config"});
    }

    // The configured-ctor adapter used by binding.cpp: the nb::handle
    // surface keeps its mapping() check (None or a non-dict raises
    // "config: expected dict") before the shared decode.
    inline WriterConfig writer_config_from_handle(nb::handle config) {
        return native_config_from_dict(wire::mapping(config, "config"));
    }
} // namespace nativecfg
