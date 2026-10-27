// json.hpp — strict lossless JSON boundary (task T05). Ports
// src/bike_sim/recording/json_codec.py's bike_recording.json.v1 contract:
// a SAX parser retaining exact number lexemes, explicit duplicate-key
// rejection, bare NaN/Infinity rejection, finite-float-token validation,
// the $bike_recording tagged forms (escape objects, float64 sentinels,
// bytes/base64, decimal, date/time/datetime, timedelta) and deterministic
// serialization. Source authority: json_codec.py:1-126.
//
// Input resource limits (documented boundary, fail-closed — never silently
// truncated): source text at most kMaxJsonBytes; container nesting at most
// kMaxDepth levels.
#pragma once

#include <cstddef>
#include <string>
#include <string_view>

#include "value.hpp"

namespace bike::io {

// Resource bounds of the parse boundary: a cold-path input is bounded the
// same way fixture payloads are bounded (512 MiB) and nesting is capped so
// deep hostile documents fail with an explicit diagnostic instead of
// exhausting the stack.
inline constexpr std::size_t kMaxJsonBytes = 512ULL * 1024ULL * 1024ULL;
inline constexpr std::size_t kMaxDepth = 512;

// One shared contract bound, not three literals: an external steering
// profile document is capped at 1 MiB by the runtime resolver
// (runtime/spatial_tire_factory.cpp), by capture
// (research/metadata.cpp) and by the recording reader
// (recording/database_validate.cpp) — all three must reject the same byte
// count, so the constant lives at this leaf where every layer can reach it.
inline constexpr std::size_t kExternalSteerProfileBoundBytes = 1024ULL * 1024ULL;

// Reserved key of the bike_recording.json.v1 tag scheme (json_codec.py:17).
inline constexpr std::string_view kRecordingTag = "$bike_recording";

// Parse a JSON document into the owned value tree.
//   * Number lexemes are kept verbatim (no binary64 round-trip; arbitrary
//     integer magnitude preserved). A floating-token lexeme that overflows
//     binary64 (positive exponent beyond double range — the pinned
//     RapidJSON scanner's NumberTooBig limit) is a syntax error here; the
//     recording path rejects such tokens either way through its finite
//     check, so the codec outcome is unchanged.
//   * Duplicate object keys are rejected during parsing.
//   * Bare NaN/Infinity tokens, trailing content and malformed UTF-8 are
//     syntax errors carrying source/offset.
// Errors: std::invalid_argument ("bike::io::parse_json: ...").
Value parse_json(std::string_view source);

// Deterministic serialization of the value tree for fixture/provenance
// hashing: object keys sorted by code point (UTF-8 byte order), compact
// separators, number lexemes emitted verbatim, Tagged/Bytes emitted in
// their tagged wire spelling. No recording-codec escaping is applied — an
// Object that literally contains the reserved key serializes as the
// literal object it is. Not a formatter for user-facing CLI output.
// Errors: std::invalid_argument on invalid number lexemes, invalid UTF-8
// strings, duplicate keys in a constructed Object or malformed Tagged.
std::string canonical_json(const Value &value);

// Serialize through the recording codec (to_json_value + dumps): object
// keys sorted, compact separators, UTF-8 passthrough; an Object containing
// the reserved key is emitted in the escaped {"$bike_recording":"object",
// "items":[[key,value],...]} form; Tagged and Bytes serialize to their tag
// objects with the source's member sets and normalized field spellings.
// A number lexeme that is not a finite binary64 (or is not valid JSON
// number syntax at all) is rejected, mirroring dumps(allow_nan=False).
// Errors: std::invalid_argument with the same domain as canonical_json.
std::string recording_json(const Value &value);

// parse_json followed by the recording tag decode (from_json_value):
// objects whose member set matches a known tag decode to Tagged/Bytes or —
// for the escaped-object tag — an Object; any other object containing the
// reserved key is rejected. Semantic errors carry a complete field path
// ($["a"][0]["b"]).
// Errors: std::invalid_argument ("bike::io::parse_recording_json: ...").
Value parse_recording_json(std::string_view source);

} // namespace bike::io
