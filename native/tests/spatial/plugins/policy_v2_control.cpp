#include "bike/policy_v2.h"
#include <cmath>
#include <new>
#include <stdexcept>
#include <span>
#include <string_view>

namespace {
struct Instance{int mode=0;};
void error(BikeErrorV1* out,const char* message)noexcept {
    if(!out)return;const std::string_view text(message);out->written=0;
    if(!out->data)return;
    const auto buffer=std::span(out->data,out->capacity);
    for(char c:text){if(out->written==buffer.size())break;buffer[out->written++]=c;}
}
bool valid(const BikeObservationV2* observation)noexcept {
    if(!observation||observation->struct_size!=sizeof(BikeObservationV2)||observation->sensor_schema_version!=2||observation->valid>1||
        observation->valid_mask>0x7ffU||(!observation->valid&&observation->valid_mask!=0)||
        !std::isfinite(observation->time_s)||!std::isfinite(observation->source_time_s)||observation->source_time_s>observation->time_s)return false;
    for(double x:observation->values)if(!std::isfinite(x))return false;return true;
}
void* create(const char* json,BikeErrorV1* out) {
    try{auto* instance=new Instance;const std::string_view value(json?json:"");
        instance->mode=value.find("nan")!=std::string_view::npos?1:value.find("throw")!=std::string_view::npos?2:0;return instance;}
    catch(...){error(out,"create allocation failure");return nullptr;}
}
int reset(void* instance,const BikeObservationV2* observation,BikeErrorV1* out){if(!instance||!valid(observation)){error(out,"invalid reset observation");return 1;}return 0;}
int act(void* raw,const BikeObservationV2* observation,BikeDemandV1* demand,BikeErrorV1* out){
    if(!raw||!demand||!valid(observation)){error(out,"invalid act observation");return 1;}
    const auto* instance=static_cast<Instance*>(raw);
    if(instance->mode==2)throw std::runtime_error("deliberate negative-control plugin exception");
    demand->present=1;demand->torque_nm=instance->mode==1?std::nan(""):25.;return 0;
}
void destroy(void* instance){delete static_cast<Instance*>(instance);}
const BikePolicyV2 descriptor{.struct_size=sizeof(BikePolicyV2),.abi_version=BIKE_POLICY_V2_ABI_VERSION,.capabilities=0,
    .create=create,.reset=reset,.act=act,.destroy=destroy};
}
#if defined(_WIN32)
#define BIKE_SPATIAL_TEST_EXPORT __declspec(dllexport)
#else
#define BIKE_SPATIAL_TEST_EXPORT __attribute__((visibility("default")))
#endif
extern "C" BIKE_SPATIAL_TEST_EXPORT const BikePolicyV2* bike_policy_v2(void){return &descriptor;}
