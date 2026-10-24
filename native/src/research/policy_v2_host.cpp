#include "policy.hpp"
#include "environment.hpp"
#include "../diag.hpp"
#include "../io/hash.hpp"
#include "../io/json.hpp"
#include "../platform/files.hpp"
#include "../runtime/setup_values.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <dlfcn.h>
#include <filesystem>
#include <iterator>
#include <utility>

namespace bike::research {
void validate_policy_abi_v2(const BikePolicyV2* api) {
    if(!api||api->struct_size!=sizeof(BikePolicyV2)||api->abi_version!=BIKE_POLICY_V2_ABI_VERSION)
        throw PolicyFailure("policy ABI2: descriptor size or version mismatch");
    if(api->capabilities!=0||!api->create||!api->reset||!api->act||!api->destroy)
        throw PolicyFailure("policy ABI2: unsupported capability or null callback");
}
NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wlarge-by-value-copy")
BikeObservationV2 policy_observation_to_abi_v2(const runtime::spatial::SensorObservationV2& value) {
    value.validate();BikeObservationV2 out{};out.struct_size=static_cast<std::uint32_t>(sizeof(out));out.sensor_schema_version=BIKE_SENSOR_V2_SCHEMA_VERSION;
    out.valid=value.valid?1U:0U;out.valid_mask=value.valid_mask;out.time_s=value.time_s;out.source_time_s=value.source_time_s;
    std::ranges::copy(value.values,std::begin(out.values));return out;
}
NATIVE_DIAG_POP
namespace {
using namespace runtime::setupvalue;
template<class Call> void invoke(std::string_view operation,Call&& call) {
    std::array<char,2048> storage{};BikeErrorV1 error{.data=storage.data(),.capacity=storage.size(),.written=0};int status=-1;
    try{status=std::forward<Call>(call)(&error);}catch(const std::exception& e){throw PolicyFailure("policy ABI2 "+std::string(operation)+" threw: "+e.what());}
    catch(...){throw PolicyFailure("policy ABI2 "+std::string(operation)+" threw a non-standard exception");}
    if(error.data!=storage.data()||error.capacity!=storage.size()||error.written>storage.size())
        throw PolicyFailure("policy ABI2 returned an invalid error buffer");
    if(status!=0)throw PolicyFailure("policy ABI2 "+std::string(operation)+" failed: "+std::string(storage.data(),error.written));
}
struct Library {
    void* handle=nullptr;
    explicit Library(const std::filesystem::path& path):handle(dlopen(path.c_str(),RTLD_NOW|RTLD_LOCAL)){
        if(!handle)throw PolicyFailure("cannot load native ABI2 policy: "+path.string());}
    ~Library(){if(handle)(void)dlclose(handle);}
    Library(const Library&)=delete;Library& operator=(const Library&)=delete;
    Library(Library&&)=delete;Library& operator=(Library&&)=delete;
    const BikePolicyV2* descriptor() const {
        // NOLINTNEXTLINE(concurrency-mt-unsafe) this loader is confined to its owning research session
        (void)dlerror();
        void* symbol=dlsym(handle,"bike_policy_v2");
        // NOLINTNEXTLINE(concurrency-mt-unsafe) same confined POSIX error slot as the matching dlsym call
        const char* error=dlerror();
        if(error||!symbol)throw std::invalid_argument("plugin: unsupported sensor schema 2 (bike_policy_v2 missing)");
        using Entry=const BikePolicyV2*(*)();Entry entry{};static_assert(sizeof(entry)==sizeof(symbol));
        // POSIX supplies the callable symbol representation. Copying it avoids
        // writing a function-pointer object through an unrelated void** alias.
        std::memcpy(&entry,&symbol,sizeof(entry));
        try{return entry();}catch(const std::exception& e){throw PolicyFailure("policy ABI2 entrypoint threw: "+std::string(e.what()));}
        catch(...){throw PolicyFailure("policy ABI2 entrypoint threw a non-standard exception");}
    }
};
struct Instance {
    const BikePolicyV2* api=nullptr;void* value=nullptr;
    ~Instance(){
        // NOLINTNEXTLINE(bugprone-empty-catch) destructors cannot propagate plugin failures; explicit close reports them
        if(value&&api){try{api->destroy(value);}catch(...){}}}
    Instance()=default;Instance(const Instance&)=delete;Instance& operator=(const Instance&)=delete;
    Instance(Instance&&)=delete;Instance& operator=(Instance&&)=delete;
    void close(){void* old=std::exchange(value,nullptr);if(!old)return;
        try{api->destroy(old);}catch(const std::exception& e){throw PolicyFailure("policy ABI2 destroy threw: "+std::string(e.what()));}
        catch(...){throw PolicyFailure("policy ABI2 destroy threw a non-standard exception");}}
};
class PluginPolicyV2 final:public Policy {
public:
    ~PluginPolicyV2() override;
    PluginPolicyV2(const PluginPolicyV2&)=delete;PluginPolicyV2& operator=(const PluginPolicyV2&)=delete;
    PluginPolicyV2(PluginPolicyV2&&)=delete;PluginPolicyV2& operator=(PluginPolicyV2&&)=delete;
    PluginPolicyV2(std::filesystem::path path,const Value& config):path_(std::move(path)),digest_(platform::file_sha256(path_)),
        json_(io::canonical_json(config)),library_(path_){
        instance_.api=library_.descriptor();validate_policy_abi_v2(instance_.api);
        invoke("create",[&](BikeErrorV1* error){instance_.value=instance_.api->create(json_.c_str(),error);return instance_.value?0:-1;});
        if(platform::file_sha256(path_)!=digest_)throw PolicyFailure("native ABI2 policy binary changed while loading");
    }
    void reset(std::string_view seed) override{(void)validate_seed(seed);require_open();reset_pending_=true;}
    runtime::RideControl act(const runtime::SensorObservation&,std::optional<double>) override{
        throw std::invalid_argument("plugin ABI2 requires sensor schema2");}
    runtime::RideControl act_v2(const runtime::spatial::SensorObservationV2& observation,std::optional<double> demand) override{
        require_open();validate_policy_input(observation,demand);const auto input=policy_observation_to_abi_v2(observation);
        if(reset_pending_){invoke("reset",[&](BikeErrorV1* error){return instance_.api->reset(instance_.value,&input,error);});reset_pending_=false;}
        BikeDemandV1 output{.present=static_cast<std::uint8_t>(demand.has_value()),.torque_nm=demand.value_or(0.)};
        invoke("act",[&](BikeErrorV1* error){return instance_.api->act(instance_.value,&input,&output,error);});
        if(output.present>1||!std::isfinite(output.torque_nm)||output.torque_nm<0.)throw PolicyFailure("policy ABI2 returned an invalid demand");
        runtime::RideControl control;if(output.present)control.motor_torque_nm=output.torque_nm;
        validate_motor_policy_control(control);return control;
    }
    Value metadata() const override{return Value{Object{{"reference",text("plugin:"+path_.string())},{"class",text("native::PluginPolicyV2")},
        {"kind",text("native_plugin")},{"abi_version",integer(2)},{"sensor_schema_version",integer(2)},
        {"source_file",text(path_.string())},{"source_sha256",text(digest_)},{"binary_sha256",text(digest_)},
        {"current_binary_sha256",text(platform::file_sha256(path_))},{"configuration_json",text(json_)},
        {"configuration_sha256",text(io::sha256(std::as_bytes(std::span{json_})))},{"resumable",Value{false}},{"trusted_native_code",Value{true}}}};}
    void close() override{instance_.close();}
private:
    void require_open()const{if(!instance_.value)throw PolicyFailure("native ABI2 policy is closed");}
    std::filesystem::path path_;std::string digest_,json_;Library library_;Instance instance_;bool reset_pending_=true;
};
PluginPolicyV2::~PluginPolicyV2() = default;
}
std::unique_ptr<Policy> load_plugin_policy_v2(std::string_view filename,const io::Value& config) {
    if(filename.empty()||filename.find('\0')!=std::string_view::npos)throw std::invalid_argument("native ABI2 policy requires a nonempty absolute path");
    const std::filesystem::path path(filename);if(!path.is_absolute())throw std::invalid_argument("native ABI2 policy requires an absolute path");
    return std::make_unique<PluginPolicyV2>(std::filesystem::weakly_canonical(path),config);
}
} // namespace bike::research
