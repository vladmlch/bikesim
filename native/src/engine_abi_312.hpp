// MuJoCo 3.12.0 private error-interception ABI. Keep private declarations here.
// Source: google-deepmind/mujoco tag 3.12.0, python/mujoco/private.h.
#pragma once

#include <mujoco/mujoco.h>

#include <type_traits>

// The official 3.12.0 ABI deliberately exports reserved names. Limit the
// diagnostic exception to these two declarations; first-party names stay gated.
#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wreserved-identifier"
#endif
extern "C" {
MJAPI mjfLogHandler _mjPRIVATE_setTlsLogHandler(mjfLogHandler handler);
MJAPI mjfLogHandler _mjPRIVATE_getGlobalLogHandler(void);
MJAPI void mj_makeRawData(mjData **data, const mjModel *model);
}
#if defined(__clang__)
#pragma clang diagnostic pop
#endif

static_assert(std::is_same_v<mjfLogHandler, void (*)(const mjLogMessage *)>);
static_assert(std::is_same_v<decltype(&_mjPRIVATE_setTlsLogHandler),
                             mjfLogHandler (*)(mjfLogHandler)>);
static_assert(std::is_same_v<decltype(&_mjPRIVATE_getGlobalLogHandler),
                             mjfLogHandler (*)()>);
static_assert(std::is_same_v<decltype(&mj_makeRawData),
                             void (*)(mjData **, const mjModel *)>);

// T04 callback-provenance pins: the public slot the provenance split reads
// (mjcb_time is the only engine callback a standalone runtime may install —
// the documented native timer in engine_call.hpp), the callback signature,
// and the public version functions engine::identity() resolves.
static_assert(std::is_same_v<mjfTime, mjtNum (*)()>,
              "mjcb_time callback ABI drifted");
static_assert(std::is_same_v<decltype(mjcb_time), mjfTime>,
              "mjcb_time is no longer the process-global mjfTime slot");
static_assert(std::is_same_v<decltype(&mj_version), int (*)()>,
              "mj_version ABI drifted");
static_assert(std::is_same_v<decltype(&mj_versionString), const char *(*)()>,
              "mj_versionString ABI drifted");
