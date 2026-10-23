#pragma once

#include "policy_v1.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BIKE_POLICY_V2_ABI_VERSION 2U
#define BIKE_SENSOR_V2_SCHEMA_VERSION 2U
#define BIKE_SENSOR_V2_VALUE_COUNT 11U

typedef struct BikeObservationV2 {
    uint32_t struct_size;
    uint32_t sensor_schema_version;
    uint32_t valid;
    uint32_t valid_mask;
    double time_s;
    double source_time_s;
    /* accel XYZ, gyro XYZ, front/rear/crank rad/s, motor/human N m. */
    double values[BIKE_SENSOR_V2_VALUE_COUNT];
} BikeObservationV2;

typedef struct BikePolicyV2 {
    uint32_t struct_size;
    uint32_t abi_version;
    uint32_t capabilities;
    void* (*create)(const char* config_json, BikeErrorV1* error);
    int (*reset)(void* instance, const BikeObservationV2* observation,
                 BikeErrorV1* error);
    int (*act)(void* instance, const BikeObservationV2* observation,
               BikeDemandV1* demand, BikeErrorV1* error);
    void (*destroy)(void* instance);
} BikePolicyV2;

/* Callbacks catch their own exceptions; destroy never throws. Host-owned input
   and error storage may not be retained. V2 is not a prefix cast of V1. */
const BikePolicyV2* bike_policy_v2(void);

#ifdef __cplusplus
}
#endif
