#pragma once

#include "../io/npy.hpp"
#include "../io/value.hpp"
#include "../runtime/control.hpp"

#include <mujoco/mujoco.h>
#include <cstdint>
#include <filesystem>
#include <span>
#include <string>
#include <utility>
#include <variant>
#include <vector>

namespace runtime { class NativeResearchRuntime; }
namespace bike::golden {
using Leaf = std::variant<std::monostate, bool, std::int64_t, double, std::string>;
using Row = std::vector<std::pair<std::string, Leaf>>;

// Python tools/golden_episode.py EpisodeArtifact, including every v2 field:
// channel_names, initial, controls, rows, first_failure, manifest,
// state_qpos, state_qvel, state_act, state_warmstart, state_time,
// ctrl_written, force_names, forces, tire_state, tire_state_names,
// qfrc_applied. NPY arrays retain dtype, shape, byte order and raw bits.
// NPZ keys: channels, channel_names, init_<each initial key>, controls,
// control_names and the nine state/force/tyre arrays and two name arrays.
// stored_rows additionally preserves list-valued non_numeric channels:
// Python flatten_row descends dictionaries but leaves lists intact, while
// episode_compare._leaves descends lists too. rows is that comparison view.
struct EpisodeArtifact {
    std::vector<std::string> channel_names;
    io::NpzArchive initial;
    std::vector<io::Value> controls;
    std::vector<Row> rows;
    io::Value first_failure;
    io::Value manifest{io::Value::Object{}};
    io::NpyArray state_qpos, state_qvel, state_act, state_warmstart, state_time;
    io::NpyArray ctrl_written, qfrc_applied;
    std::vector<std::string> force_names;
    io::NpyArray forces, tire_state;
    std::vector<std::string> tire_state_names;
    std::vector<io::Value> stored_rows;
    std::vector<std::byte> model_image;
};

[[nodiscard]] Row flatten_row(const io::Value &row);
[[nodiscard]] io::Value storage_row(const io::Value &row);
[[nodiscard]] EpisodeArtifact capture_episode(runtime::NativeResearchRuntime &env,
    std::size_t steps, const runtime::RideControl &control);
void save(const EpisodeArtifact &, const std::filesystem::path &out_dir,
          const mjModel *model = nullptr);
[[nodiscard]] EpisodeArtifact load_episode(const std::filesystem::path &directory);
[[nodiscard]] std::vector<Row> replay_episode(runtime::NativeResearchRuntime &env,
                                             const EpisodeArtifact &episode);
[[nodiscard]] std::vector<std::string> compare_rows(const std::vector<Row> &golden,
    const std::vector<Row> &candidate, double rtol = 1e-12, double atol = 1e-12);
[[nodiscard]] std::vector<std::string> compare_episode(const EpisodeArtifact &golden,
    runtime::NativeResearchRuntime &env, double rtol = 1e-12, double atol = 1e-12);
[[nodiscard]] bool same_failure(const io::Value &golden, const io::Value &candidate,
                               double rtol, double atol);
} // namespace bike::golden
