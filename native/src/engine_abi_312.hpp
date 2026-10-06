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
