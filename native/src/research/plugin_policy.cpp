#include "policy.hpp"
#include "environment.hpp"
#include "../io/hash.hpp"
#include "../io/json.hpp"
#include "../platform/files.hpp"
#include "../runtime/setup_values.hpp"
#include <array>
#include <bit>
#include <dlfcn.h>
#include <filesystem>
#include <system_error>
#include <utility>

namespace bike::research {
namespace {
using namespace runtime::setupvalue;
class Library {
public:
    explicit Library(const std::filesystem::path& path) : handle_(dlopen(path.c_str(), RTLD_NOW | RTLD_LOCAL)) {
        if (!handle_) {
            const char* error = dlerror();
            throw PolicyFailure("cannot load native policy: " + std::string(error ? error : "unknown loader error"));
        }
    }
    ~Library() { if (handle_) (void)dlclose(handle_); }
    Library(const Library&) = delete;
    Library& operator=(const Library&) = delete;
    [[nodiscard]] const BikePolicyV1* descriptor() const {
        (void)dlerror();
        void* symbol = dlsym(handle_, "bike_policy_v1");
        const char* error = dlerror();
        if (error || !symbol) throw PolicyFailure("native policy is missing bike_policy_v1");
        using Entry = const BikePolicyV1* (*)(void);
        static_assert(sizeof(Entry) == sizeof(symbol));
        const Entry entry = std::bit_cast<Entry>(symbol);
        try { return entry(); }
        catch (const std::exception& cause) { throw PolicyFailure("policy entrypoint threw: " + std::string(cause.what())); }
        catch (...) { throw PolicyFailure("policy entrypoint threw a non-standard exception"); }
    }
private:
    void* handle_;
};
struct Instance {
    const BikePolicyV1* api = nullptr;
    void* value = nullptr;
    ~Instance() {
        if (value && api) { try { api->destroy(value); } catch (...) {} }
    }
    Instance() = default;
    Instance(const Instance&) = delete;
    Instance& operator=(const Instance&) = delete;
    void close() {
        void* previous = std::exchange(value, nullptr);
        if (!previous) return;
        try { api->destroy(previous); }
        catch (const std::exception& cause) { throw PolicyFailure("policy destroy threw: " + std::string(cause.what())); }
        catch (...) { throw PolicyFailure("policy destroy threw a non-standard exception"); }
    }
};
template<class Callback> void invoke(std::string_view operation, Callback&& callback) {
    std::array<char, 2048> storage{};
    BikeErrorV1 error{storage.data(), storage.size(), 0};
    int status = -1;
    try { status = callback(&error); }
    catch (const std::exception& cause) { throw PolicyFailure("policy " + std::string(operation) + " threw: " + cause.what()); }
    catch (...) { throw PolicyFailure("policy " + std::string(operation) + " threw a non-standard exception"); }
    if (error.data != storage.data() || error.capacity != storage.size() || error.written > storage.size())
        throw PolicyFailure("policy " + std::string(operation) + " returned an invalid error buffer");
    if (status != 0) throw PolicyFailure("policy " + std::string(operation) + " failed (" + std::to_string(status) + "): " +
        std::string(storage.data(), error.written));
}
class PluginPolicy final : public Policy {
public:
    ~PluginPolicy() override;
    PluginPolicy(std::filesystem::path path, const Value& config)
        : path_(std::move(path)), digest_(platform::file_sha256(path_)),
          config_(io::canonical_json(config)), library_(path_) {
        config_digest_ = io::sha256(std::as_bytes(std::span{config_}));
        instance_.api = library_.descriptor();
        validate_policy_abi(instance_.api);
        invoke("create", [&](BikeErrorV1* error) {
            return instance_.api->create(config_.data(), config_.size(), &instance_.value, error);
        });
        if (!instance_.value) throw PolicyFailure("policy create succeeded with a null instance");
        if (platform::file_sha256(path_) != digest_) throw PolicyFailure("native policy binary changed while loading");
    }
    void reset(std::string_view seed) override {
        require_open(); const auto normalized = validate_seed(seed);
        invoke("reset", [&](BikeErrorV1* error) { return instance_.api->reset(instance_.value, normalized.data(), normalized.size(), error); });
    }
    runtime::RideControl act(const runtime::SensorObservation& observation, std::optional<double> demand) override {
        require_open(); validate_policy_input(observation, demand);
        const auto input = policy_observation_to_abi(observation);
        const BikeDemandV1 requested{static_cast<std::uint8_t>(demand.has_value() ? 1 : 0), demand.value_or(0.)};
        BikeControlV1 output{};
        output.struct_size = static_cast<std::uint32_t>(sizeof(output));
        output.rider_enabled = 1; output.use_saddle = 1;
        invoke("act", [&](BikeErrorV1* error) { return instance_.api->act(instance_.value, &input, &requested, &output, error); });
        return policy_control_from_abi(output);
    }
    Value metadata() const override {
        return Value{Object{{"reference", text("plugin:" + path_.string())}, {"class", text("native::PluginPolicy")},
            {"kind", text("native_plugin")}, {"abi_version", integer(BIKE_POLICY_ABI_VERSION)},
            {"sensor_schema_version", integer(BIKE_SENSOR_SCHEMA_VERSION)},
            {"source_file", text(path_.string())}, {"source_sha256", text(digest_)},
            {"binary_sha256", text(digest_)}, {"current_binary_sha256", text(platform::file_sha256(path_))},
            {"configuration_json", text(config_)}, {"configuration_sha256", text(config_digest_)},
            {"trusted_native_code", Value{true}}}};
    }
    void close() override { instance_.close(); }
private:
    void require_open() const { if (!instance_.value) throw PolicyFailure("native policy is closed"); }
    std::filesystem::path path_;
    std::string digest_, config_, config_digest_;
    Library library_; // destroyed after its instance and all callbacks
    Instance instance_;
};
PluginPolicy::~PluginPolicy() = default;
} // namespace
std::unique_ptr<Policy> load_plugin_policy(std::string_view filename, const io::Value& config) {
    if (filename.empty() || filename.find('\0') != std::string_view::npos)
        throw std::invalid_argument("native policy requires a nonempty explicit library path");
    const std::filesystem::path path(filename);
    if (!path.is_absolute()) throw std::invalid_argument("native policy library path must be absolute");
    try { return std::make_unique<PluginPolicy>(std::filesystem::weakly_canonical(path), config); }
    catch (const std::system_error& error) { throw PolicyFailure("cannot load native policy '" + path.string() + "': " + error.what()); }
}
} // namespace bike::research
