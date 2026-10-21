#include "support/migration_test.hpp"
#include "cli/recording.hpp"
#include "golden/episode.hpp"
#include "io/json.hpp"
#include "platform/files.hpp"

#include <array>
#include <bit>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <sstream>
#include <string_view>

namespace {
namespace io = bike::io;
namespace golden = bike::golden;
using io::Value;
struct Scratch {
    std::filesystem::path root = std::filesystem::current_path() / "golden-episode-codec-scratch";
    Scratch() { std::filesystem::remove_all(root); std::filesystem::create_directory(root); }
    ~Scratch() { std::error_code ignored; std::filesystem::remove_all(root, ignored); }
    Scratch(const Scratch &) = delete;
    Scratch &operator=(const Scratch &) = delete;
    Scratch(Scratch &&) = delete;
    Scratch &operator=(Scratch &&) = delete;
};
void same(const io::NpyArray &a, const io::NpyArray &b) {
    bt::require(a.dtype == b.dtype && a.shape == b.shape && a.fortran_order == b.fortran_order && a.bytes == b.bytes,
        "NPY dtype, dimensions, order and all payload bits survive");
}
void npz_contract() {
    const std::array reals{0., -0., std::bit_cast<double>(UINT64_C(0xfff8000000001234)),
        std::numeric_limits<double>::infinity()};
    const std::array<std::int64_t, 2> integers{std::numeric_limits<std::int64_t>::min(), std::numeric_limits<std::int64_t>::max()};
    const std::array<std::string, 3> strings{"", "front", "\xd1\x88\xd0\xb8\xd0\xbd\xd0\xb0"};
    io::NpzArchive archive{{"real", io::double_array(reals, {2, 2})},
        {"integer", io::integer_array(integers, {2})}, {"flags", io::bool_array({true, false}, {2})},
        {"names", io::string_array(strings, {3})}, {"scalar", io::double_array(std::span(reals).first(1), {})},
        {"empty", io::double_array(std::span<const double>{}, {0, 4})}};
    const auto bytes = io::encode_npz(archive);
    bt::require(bytes.size() > 30 && bytes[0] == std::byte{0x50} && bytes[1] == std::byte{0x4b} &&
        bytes[2] == std::byte{3} && bytes[3] == std::byte{4}, "local ZIP signature");
    bt::require(bytes[8] == std::byte{0} && bytes[9] == std::byte{0}, "STORED, compatible with np.savez");
    bt::require(io::encode_npz(archive) == bytes, "encoding has no timestamp nondeterminism");
    const auto decoded = io::parse_npz(bytes);
    bt::require(decoded.size() == archive.size(), "every member present");
    for (std::size_t i = 0; i < archive.size(); ++i) { bt::require(decoded[i].first == archive[i].first, "member insertion order"); same(decoded[i].second, archive[i].second); }
    auto duplicate = archive; duplicate.push_back(archive.front());
    bt::invalid([&] { (void)io::encode_npz(duplicate); });
    for (const auto *key : {"../outside", "sub/member", "a\\b", "", ".hidden"}) {
        bt::invalid([&] { (void)io::encode_npz({{key, archive.front().second}}); });
    }
    auto corrupt = bytes;
    // The first STORED member begins after its local header and simple name;
    // changing the final byte of its known NPY payload must fail the CRC gate.
    const auto payload_end = 30 + std::string("real.npy").size() + io::encode_npy(archive.front().second).size();
    corrupt[payload_end - 1] ^= std::byte{1};
    bt::invalid([&] { (void)io::parse_npz(corrupt); });
}
void comparison_contract() {
    const auto nested = io::parse_json(R"({"state":{"q":[1.0,"nan",null]},"flag":true,"text":"front"})");
    const auto stored = golden::storage_row(nested);
    bt::require(io::find_member(io::as_object(stored, "row"), "state.q") != nullptr, "storage keeps list-valued non_numeric channels");
    const auto row = golden::flatten_row(nested);
    bt::require(golden::compare_rows({row}, {golden::flatten_row(stored)}).empty(), "nested and stored row key spaces agree");
    auto candidate = row;
    candidate.front().second = 1.001;
    const auto differences = golden::compare_rows({row}, {candidate});
    bt::require(differences.size() == 1 && differences.front().starts_with("step=0 channel=state.q.0"), "one changed leaf gives exactly one diagnostic");
    bt::require(golden::compare_rows({row}, {}).front() == "row count: golden=1 candidate=0", "row-count diagnostic");
    const auto nan = std::numeric_limits<double>::quiet_NaN();
    bt::require(golden::compare_rows({{{"n", std::string("nan")}}}, {{{"n", nan}}}).empty(), "portable NaN sentinel compares as NaN");
    bt::require(golden::compare_rows({{{"x", 1.}}}, {{{"x", 2.}}}, .5, 0.).empty(), "candidate supplies rtol scale");
    bt::require(golden::compare_rows({{{"x", 2.}}}, {{{"x", 1.}}}, .5, 0.).size() == 1, "asymmetry follows numpy.isclose");
    bt::require(golden::compare_rows({{{"x", std::string(" +1_000.0 ")}}}, {{{"x", 1000.}}}).empty(), "Python decimal string conversion");
    bt::require(golden::compare_rows({{{"x", std::string("nan(payload)")}}}, {{{"x", nan}}}).size() == 1, "non-Python NaN spellings are not converted");
    bt::require(golden::compare_rows({{{"x", std::string("text")}}}, {{{"x", std::string("text")}, {"extra", false}}}).size() == 1, "candidate-only channel is a divergence");
    bt::require(golden::compare_rows({{{"x", std::numeric_limits<double>::infinity()}}}, {{{"x", 1e300}}}).size() == 1, "infinity cannot match finite values");
    bt::invalid([&] { (void)golden::compare_rows({row}, {row}, -1., 0.); });
    bt::invalid([&] { (void)golden::compare_rows({row}, {row}, nan, 0.); });
    golden::Row many, other;
    for (int i = 0; i < 51; ++i) { many.emplace_back(std::to_string(i), 0.); other.emplace_back(std::to_string(i), 1.); }
    const auto flood = golden::compare_rows({many, many}, {other, other});
    bt::require(flood.size() == 52 && flood.back() == "…truncated", "diffs stop after the first whole row exceeding fifty");
    bt::require(golden::same_failure(Value{}, Value{}, 0., 0.), "absent first failures agree");
    bt::require(golden::same_failure(io::parse_json(R"([0.1,["one","two"]])"), io::parse_json(R"([0.1,["one","two"]])"), 0., 0.), "failure time and ordered violations agree");
    bt::require(!golden::same_failure(io::parse_json(R"([0.1,["one","two"]])"), io::parse_json(R"([0.1,["two","one"]])"), 0., 0.), "failure violation order remains significant");
}
void artifact_contract() {
    Scratch scratch;
    // Heap — the artifact and each load result are ~1KB; this frame is held
    // under the 8192-byte budget.
    const auto episode_owner = std::make_unique<golden::EpisodeArtifact>();
    auto& episode = *episode_owner;
    // Synthetic format exercise only. These are not engine bytes or an
    // oracle trajectory; no reference golden directory is written here.
    episode.model_image = {std::byte{1}, std::byte{2}};
    episode.channel_names = {"a", "q", "status"};
    episode.stored_rows = {io::parse_json(R"({"a":-0.0,"q":[1.0,"nan"],"status":"ok"})"), io::parse_json(R"({"a":2.0,"q":[null,"-inf"]})")};
    for (const auto &row : episode.stored_rows) episode.rows.push_back(golden::flatten_row(row));
    episode.controls = {io::parse_json(R"({"human_torque_nm":35.0,"posture":{"pelvis_offset_m":[0.0,0.01]},"motor_torque_nm":null})")};
    const std::array zero{0.};
    const std::array velocity{0., -0.};
    episode.initial = {{"qpos", io::double_array(zero, {1})}, {"qvel", io::double_array(velocity, {2})},
        {"act", io::double_array(std::span<const double>{}, {0})}, {"ctrl", io::double_array(zero, {1})}};
    episode.forces = io::double_array(velocity, {1, 1, 2}); episode.force_names = {"independent"};
    const std::array live{.125, -0.}; episode.qfrc_applied = io::double_array(live, {1, 2});
    episode.manifest = io::parse_json(R"({"producer":"synthetic codec exercise"})");
    golden::save(episode, scratch.root / "saved");
    // NOLINTNEXTLINE(modernize-make-unique) make_unique's inlined forwarding keeps the by-value result on this frame; new T(f()) elides
    const auto loaded_owner = std::unique_ptr<golden::EpisodeArtifact>{new golden::EpisodeArtifact(
        golden::load_episode(scratch.root / "saved"))};
    const auto& loaded = *loaded_owner;
    bt::require(golden::compare_rows(episode.rows, loaded.rows, 0., 0.).empty(), "numeric, absent, list and null channels survive save/load");
    bt::require(episode.channel_names == loaded.channel_names && episode.force_names == loaded.force_names, "names survive");
    bt::require(io::canonical_json(Value{episode.controls}) == io::canonical_json(Value{loaded.controls}), "nested applied controls survive manifest sidecar");
    same(episode.forces, loaded.forces); same(episode.qfrc_applied, loaded.qfrc_applied);
    bt::require(loaded.model_image == episode.model_image, "model payload is retained byte for byte");
    bool exists_rejected = false;
    try { golden::save(episode, scratch.root / "saved"); } catch (const std::system_error &) { exists_rejected = true; }
    bt::require(exists_rejected, "save refuses an existing artifact directory");
    auto old = io::parse_npz(io::read_numpy_bytes(scratch.root / "saved/episode.npz"));
    std::erase_if(old, [](const auto &item) { return item.first == "qfrc_applied"; });
    const auto bytes = io::encode_npz(old);
    std::filesystem::remove(scratch.root / "saved/episode.npz");
    bike::platform::write_exclusive(scratch.root / "saved/episode.npz", bytes);
    // NOLINTNEXTLINE(modernize-make-unique) same inlined-forwarding frame budget as loaded_owner
    const auto legacy_owner = std::unique_ptr<golden::EpisodeArtifact>{new golden::EpisodeArtifact(
        golden::load_episode(scratch.root / "saved"))};
    const auto& legacy = *legacy_owner;
    bt::require(legacy.qfrc_applied.shape == std::vector<std::size_t>{0, 2} && legacy.qfrc_applied.bytes.empty(), "old artifacts never synthesize live exit totals from components");
    same(legacy.forces, episode.forces);
    // Python json.dumps leaves nonfinite constants unquoted within lists;
    // strings containing the same letters remain strings during decoding.
    std::ofstream manifest(scratch.root / "saved/manifest.json", std::ios::binary | std::ios::trunc);
    manifest << R"({"non_numeric":[{"q":[NaN,Infinity,-Infinity],"status":"NaN outside"},{}],"controls":[]})";
    manifest.close();
    // NOLINTNEXTLINE(modernize-make-unique) same inlined-forwarding frame budget as loaded_owner
    const auto nonfinite_owner = std::unique_ptr<golden::EpisodeArtifact>{new golden::EpisodeArtifact(
        golden::load_episode(scratch.root / "saved"))};
    const auto& nonfinite = *nonfinite_owner;
    const auto wanted = golden::flatten_row(io::parse_json(R"({"a":-0.0,"q":["nan","inf","-inf"],"status":"NaN outside"})"));
    bt::require(golden::compare_rows({wanted}, {nonfinite.rows.front()}, 0., 0.).empty(), "legacy nonfinite list tokens retain comparison meaning");
}
void cli_contract() {
    const auto call = [](auto arguments) {
        std::ostringstream out, error;
        return bike::cli::recording_command(std::span(arguments), out, error);
    };
    bt::require(call(std::array<std::string_view, 2>{"golden", "--help"}) == 0, "nested help dispatch succeeds");
    bt::require(call(std::array<std::string_view, 4>{"golden", "compare", "--golden", "unused"}) == 2, "missing request is usage error before I/O");
    bt::require(call(std::array<std::string_view, 8>{"golden", "compare", "--golden", "unused", "--request", "unused", "--rtol", "-1"}) == 2, "negative tolerance is usage error before I/O");
    bt::require(call(std::array<std::string_view, 8>{"golden", "freeze", "--request", "unused", "--steps", "-1", "--out", "unused"}) == 2, "negative steps are usage error before setup");
    bt::require(call(std::array<std::string_view, 8>{"golden", "compare", "--golden", "unused", "--request", "unused", "--request", "duplicate"}) == 2, "duplicate options rejected");
}
} // namespace
int main() { return bt::run([] { npz_contract(); comparison_contract(); artifact_contract(); cli_contract(); }); }
