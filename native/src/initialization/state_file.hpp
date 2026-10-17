#pragma once

#include <filesystem>
#include <optional>
#include <string>
#include "../io/npy.hpp"
#include "../io/value.hpp"
#include "equilibrium.hpp"

namespace bike::initialization {
struct SetupContext;
struct StateFile {
    // 0 imports the immutable legacy NPZ inventory; 1 is the native typed
    // array envelope. Every array owns dtype/shape/order/endian and bytes.
    int schema_version=1;
    io::Value metadata;
    io::NpzArchive arrays;
};
[[nodiscard]] StateFile read_state_file(const std::filesystem::path&);
void write_state_file(const std::filesystem::path&,const StateFile&);
[[nodiscard]] std::string fingerprint(const StateFile&);
[[nodiscard]] std::string fingerprint(const SetupContext&);
[[nodiscard]] std::string legacy_equilibrium_fingerprint(const SetupContext&);
[[nodiscard]] StateFile capture_state(const SetupContext&,std::int64_t steps,double residual);
[[nodiscard]] std::filesystem::path equilibrium_cache_path(const SetupContext&);
[[nodiscard]] std::optional<StateFile> load_equilibrium_cache(const SetupContext&);
[[nodiscard]] StateFile load_equilibrium_state(const SetupContext&,const std::filesystem::path&);
void save_equilibrium_cache(const SetupContext&,const StateFile&);
// Stages complete native/legacy material state, then commits through each
// writer's validated restore. Rejection restores the original owner;
// fatal engine failures retain the Stepper poison contract.
void restore_equilibrium_state(SetupContext&,const StateFile&);
[[nodiscard]] const io::NpyArray& state_array(const StateFile&,std::string_view name);

// Explicit t=0 transfer retains the target's own configuration/source
// identity. It permits timestep and distant road mesh differences only.
class PhysicalInitialState {
public:
    explicit PhysicalInitialState(std::string payload_json);
    [[nodiscard]] static PhysicalInitialState capture(const SetupContext&);
    [[nodiscard]] static PhysicalInitialState project_stations(const SetupContext&,int station_count);
    [[nodiscard]] io::Value payload() const;
    [[nodiscard]] std::string sha256() const;
    [[nodiscard]] const std::string& payload_json() const noexcept{return payload_json_;}
    [[nodiscard]] EquilibriumResult prepare(SetupContext&) const;
    void restore(SetupContext&) const;
private:
    std::string payload_json_;
};
} // namespace bike::initialization
