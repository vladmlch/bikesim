// This gate requires a real Python-frozen artifact for the host platform.
// Missing acquisition fails explicitly; it is never replaced by a native
// self-golden or synthetic numeric values. Keep ADR-0003 parity until the
// two-platform captures and cross-reads have actually been qualified.
#include "support/golden.hpp"
#include "support/migration_test.hpp"
#include "golden/episode.hpp"
#include "io/json.hpp"
#include "research/environment.hpp"
#include "runtime/research.hpp"

#include <algorithm>
#include <filesystem>
#include <iostream>
#include <ranges>
#include <string>

namespace {
namespace io = bike::io;
namespace golden = bike::golden;
void same(const io::NpyArray &expected, const io::NpyArray &actual, std::string_view name) {
    bt::require(expected.dtype == actual.dtype && expected.shape == actual.shape &&
        expected.fortran_order == actual.fortran_order && expected.bytes == actual.bytes,
        "bitwise episode array differs: " + std::string(name));
}
void fields(const golden::EpisodeArtifact &a, const golden::EpisodeArtifact &b) {
    same(a.state_qpos, b.state_qpos, "state_qpos"); same(a.state_qvel, b.state_qvel, "state_qvel");
    same(a.state_act, b.state_act, "state_act"); same(a.state_warmstart, b.state_warmstart, "state_warmstart");
    same(a.state_time, b.state_time, "state_time"); same(a.ctrl_written, b.ctrl_written, "ctrl_written");
    same(a.qfrc_applied, b.qfrc_applied, "qfrc_applied"); same(a.forces, b.forces, "forces");
    same(a.tire_state, b.tire_state, "tire_state");
    bt::require(a.force_names == b.force_names && a.tire_state_names == b.tire_state_names && a.channel_names == b.channel_names,
        "force, tyre and recorder channel names/order agree");
    bt::require(a.initial.size() == b.initial.size(), "initial fields all present");
    for (const auto &[name, value] : a.initial) {
        const auto found = std::ranges::find_if(b.initial, [&](const auto &item) { return item.first == name; });
        bt::require(found != b.initial.end(), "initial field " + name); same(value, found->second, "initial." + name);
    }
    bt::require(io::canonical_json(io::Value{a.controls}) == io::canonical_json(io::Value{b.controls}), "applied control payloads agree");
}
void check() {
    const auto fixture = bt::oracle::root() / "episode/pinned_savage_40";
    const auto source = bt::oracle::root() / bt::oracle::platform() / "episode/pinned_savage_40";
    bt::require(std::filesystem::is_regular_file(source / "episode.npz"),
        "task 3.03 oracle acquisition required: " + (source / "episode.npz").string());
    const auto episode = golden::load_episode(source);
    bt::require(episode.rows.size() == 40 && episode.controls.size() == 40, "pinned Python artifact has forty rows and controls");
    bt::require(episode.state_qpos.shape.size() == 2 && episode.state_qpos.shape[0] == 40, "forty force-entry qpos records");
    bt::require(episode.forces.shape.size() == 3 && episode.forces.shape[0] == 40 &&
        episode.forces.shape[1] == episode.force_names.size(), "complete force component matrix");
    const auto resaved = std::filesystem::current_path() / "golden-episode-python-crossread";
    std::filesystem::remove_all(resaved);
    golden::save(episode, resaved);
    const auto roundtrip = golden::load_episode(resaved);
    fields(episode, roundtrip);
    bt::require(episode.model_image == roundtrip.model_image, "MJB payload survives resave");
    bt::require(io::canonical_json(episode.manifest) == io::canonical_json(roundtrip.manifest), "manifest survives resave");
    bt::require(golden::compare_rows(episode.rows, roundtrip.rows, 0., 0.).empty(), "every loaded row survives resave");
    std::cout << "Python cross-read artifact: " << resaved.string() << '\n';
    auto environment = bike::research::make_environment(fixture / "request.toml");
    runtime::RideControl control; control.human_torque_nm = 35.;
    const auto captured = golden::capture_episode(*environment, 40, control);
    const auto differences = golden::compare_rows(episode.rows, captured.rows);
    for (const auto &difference : differences) std::cerr << difference << '\n';
    bt::require(differences.empty(), "native recorder rows reproduce Python oracle");
    fields(episode, captured);
    bt::require(golden::same_failure(episode.first_failure, captured.first_failure, 1e-12, 1e-12), "same first monitor failure");
    const auto replay = golden::compare_episode(episode, *environment);
    for (const auto &difference : replay) std::cerr << difference << '\n';
    bt::require(replay.empty(), "explicit initial-state replay reproduces every recorded control and row");
    const auto self = golden::compare_episode(captured, *environment, 0., 0.);
    bt::require(self.empty(), "native capture self-replays at zero tolerance");
}
} // namespace
int main() { return bt::run(check); }
