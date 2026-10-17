#ifndef BIKE_POLICY_V1_H
#define BIKE_POLICY_V1_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BIKE_POLICY_ABI_VERSION 1U
#define BIKE_SENSOR_SCHEMA_VERSION 1U
#define BIKE_SENSOR_VALUE_COUNT 9U
#define BIKE_SENSOR_ALL_FIELDS 0x1ffU
#define BIKE_CONTROL_MOTOR_TORQUE (1U << 0)
#define BIKE_CONTROL_MOTOR_LIMIT (1U << 1)
#define BIKE_CONTROL_HUMAN_TORQUE (1U << 2)
#define BIKE_CONTROL_CRANK_RATE (1U << 3)
#define BIKE_CONTROL_POSTURE (1U << 4)
#define BIKE_CONTROL_PELVIS_OFFSET (1U << 5)

typedef struct BikeErrorV1 {
    char* data;
    size_t capacity;
    size_t written;
} BikeErrorV1;

typedef struct BikeObservationV1 {
    uint32_t struct_size;
    uint8_t valid;
    double time_s;
    double source_time_s;
    /* acceleration xyz, pitch-up gyro, front/rear/crank encoder,
       measured motor/human torque. No model/force/recording pointers. */
    double values[BIKE_SENSOR_VALUE_COUNT];
    uint32_t sensor_schema_version;
    uint32_t valid_mask;
} BikeObservationV1;

typedef struct BikeDemandV1 {
    uint8_t present;
    double torque_nm;
} BikeDemandV1;

typedef struct BikeControlV1 {
    uint32_t struct_size;
    uint32_t present_mask;
    double motor_torque_nm;
    double motor_limit_nm;
    double human_torque_nm;
    double crank_target_rate_rad_s;
    double torso_lean_rad;
    double pelvis_pitch_rad;
    double pelvis_offset_m[2];
    uint8_t use_saddle;
    uint8_t rider_enabled;
} BikeControlV1;

typedef struct BikePolicyV1 {
    uint32_t abi_version;
    uint32_t struct_size;
    uint64_t required_capabilities; /* v1 host supports zero required flags */
    int (*create)(const char* config_json, size_t config_size,
                  void** instance, BikeErrorV1* error);
    int (*reset)(void* instance, const char* seed_decimal, size_t seed_size,
                 BikeErrorV1* error);
    int (*act)(void* instance, const BikeObservationV1* observation,
               const BikeDemandV1* demand, BikeControlV1* control,
               BikeErrorV1* error);
    void (*destroy)(void* instance);
    uint32_t sensor_schema_version;
} BikePolicyV1;

/* Export with default symbol visibility. Every callback must catch its own
   exceptions; destroy is nonthrowing. Inputs/error/control storage belongs to
   the host and may not be retained. The opaque instance belongs to the module. */
const BikePolicyV1* bike_policy_v1(void);

#ifdef __cplusplus
}
#endif
#endif
