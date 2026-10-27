// configuration/load.hpp — the T13 port of physics/resolution.py (TOML
// loading, `_merge` precedence, `_construct` resolution and
// resolve_config_paths) plus the text half of plant_profile.py's
// write_physics_toml (api-index T13):
//
//   SimulationPhysicsConfig load_physics(const std::filesystem::path& file);
//   SimulationPhysicsConfig resolve_physics(const bike::io::Value& overrides,
//       const SimulationPhysicsConfig& base);
//   std::string write_physics_toml(const SimulationPhysicsConfig& config);
//
// Semantics preserved from the source (resolution.py):
//   * precedence defaults < TOML/file values < explicitly supplied values —
//     `resolve_physics` merges the caller's override document over
//     `asdict(base)`, exactly the resolve_physics_config(asdict(base),
//     overrides) call shape the source uses; a null override member is the
//     'flag absent' sentinel and never replaces a base value;
//   * `_merge` is a recursive object+object merge — any non-object override
//     member replaces the base member wholesale;
//   * `_construct` rejects non-table sections ('{cls} requires a TOML
//     table'), unknown keys ('unknown {cls} parameter(s): ...') and missing
//     required members (the CPython missing-arguments text, wrapped as
//     'invalid {cls}: ...'), promotes a material table containing
//     deflection_m/force_n to TabulatedTireSpec, and merges partial
//     [tires.pneumatic.front|rear] sections onto the declared tyre defaults;
//   * resolve_config_paths anchors joint_envelope_path, joint_strength_path,
//     equilibrium_state_file and nonempty spatial_3d.terrain.path at the
//     declaring TOML's directory —
//     Path(directory)/file).resolve() — never the process working directory;
//     absolute spellings pass through byte-identical.
//
// Error classes follow the file boundary's contract: structural/domain
// violations throw std::invalid_argument carrying the source message text;
// OS failures (missing/unreadable file) throw std::system_error.
#pragma once

#include <filesystem>
#include <string>
#include <string_view>

#include "model_config.hpp"
#include "../io/value.hpp"

namespace bike::configuration {

// physics/resolution.py load_physics_config(path): bounded no-follow read,
// strict TOML parse, referenced-asset resolution against the file's parent,
// then construction through post_init.
[[nodiscard]] SimulationPhysicsConfig
load_physics(const std::filesystem::path& file);

// resolve_physics_config(asdict(base), overrides): recursively merges the
// override document over the base's encoded form and constructs the result.
// `overrides` must be an object — 'configuration and overrides must be
// mappings' otherwise. The merged document runs the full validation the TOML
// path applies, so a resolved base carries its post_init-normalized members
// (drive_mode/pitch_assist) while an unresolved SimulationPhysicsConfig{}
// base re-resolves them — the same asymmetry the source exposes between a
// constructed and a declared-default `values` document.
// Spatial tyre aliases override the corresponding inherited fields without
// hiding contradictions within the override document. Explicit spatial=planar
// removes inherited spatial sections (including articulated.shoulder_profile
// and its digest); the caller must select a valid planar
// tyre backend, and explicit planar+spatial fields remain invalid.
[[nodiscard]] SimulationPhysicsConfig
resolve_physics(const bike::io::Value& overrides,
                const SimulationPhysicsConfig& base);

// plant_profile.py write_physics_toml, the document half: emits the
// asdict-equivalent TOML text (banner comment, bare-key tables in field
// order, scalars then subtables per section). The source's exclusive
// non-overwriting file step belongs to the caller's write boundary
// (platform::write_exclusive) — file output is intentionally absent from
// this signature. Nonfinite scalars fail with the json.dumps message —
// 'Out of range float values are not JSON compliant'.
[[nodiscard]] std::string
write_physics_toml(const SimulationPhysicsConfig& config);

namespace detail {

// tomllib.load equivalent over an in-memory document: strict TOML into the
// bike::io::Value tree (tables -> insertion-order objects, integers stay
// decimal lexemes, date/time kinds become Tagged sentinels that the typed
// domain rejects). `origin` names the document in parse diagnostics.
[[nodiscard]] bike::io::Value parse_toml(std::string_view source,
                                         std::string_view origin);

// Bounded (512 MiB), no-follow, regular-file TOML read+parse.
[[nodiscard]] bike::io::Value
parse_toml_file(const std::filesystem::path& file);

// resolution.py resolve_config_paths, in place: relative
// articulated.joint_envelope_path / joint_strength_path and
// equilibrium_state_file members and nonempty spatial_3d.terrain.path are
// rewritten to the weakly-canonical absolute spelling anchored at `directory`;
// absolute and null members are untouched. An empty spatial terrain path
// stays empty for kind='none'; strict source validation still rejects it for
// a terrain kind which requires an asset.
void resolve_config_paths(bike::io::Value& values,
                          const std::filesystem::path& directory);

// resolution.py `_merge(base, override)` over Value documents: object+object
// members merge recursively, null override members are skipped, everything
// else replaces. Both operands must be objects — the guard text is the
// source's 'configuration and overrides must be mappings'.
[[nodiscard]] bike::io::Value merge_values(const bike::io::Value& base,
                                           const bike::io::Value& overrides);

// Decodes the encode_tree/to_value wire tags back to plain members:
// {"$tuple": [...]} objects unwrap to their array and {"$nonfinite": "..."}
// leaves become the corresponding number lexeme — the document asdict()
// produced before the tags were applied. Plain members pass through.
[[nodiscard]] bike::io::Value plain_document(const bike::io::Value& encoded);

// resolution.py `_construct(SimulationPhysicsConfig, values)`:
// construction + post_init validation, no merging (the caller composes
// merge_values for layered documents).
[[nodiscard]] SimulationPhysicsConfig
construct_config(const bike::io::Value& values);

} // namespace detail
} // namespace bike::configuration
