// Port of tests/reference/test_native_replay_recording_v3.py — the
// native-produced contract-3 container under the shared verifier, the
// failed-recording analyzable/playable split, and playback that structurally
// never touches replay machinery.
#include "replay_reference_support.hpp"

namespace {
namespace rr = replay_reference;
using rr::count;
using rr::field;

// test_native_replay_v3_and_suffix_capture — headless and incremental replay
// share the certificate; restart rebuilds a fresh environment that never
// reuses the replayed command streams, and the report is identical.
void v3_and_suffix_capture() {
    rr::Directory tmp;
    const auto publication = rr::record_container(tmp.path);
    const auto expected = rr::replay::check(publication.directory, 0);
    bt::require(expected.matched && rr::flag(expected.details, "passed") &&
        rr::text_of(expected.details, "backend") == "native",
        "native-produced container replays under the same-backend gate");
    rr::replay::ReplaySession session(publication.directory, 0);
    while (!session.advance(std::nullopt, session.snapshot().step + 1)) {
        bt::require(!session.done(), "mid-replay states are never terminal");
    }
    bt::require(rr::equal(session.report(), expected.details),
        "incremental report is the headless certificate");
    session.restart();
    while (!session.advance()) {}
    bt::require(rr::equal(session.report(), expected.details),
        "a restart rebuilds the same evidence without reusing command streams");
    session.close();
}

// test_native_failed_recording_stays_analyzable_and_playable — a
// policy-failure episode is analyzable and frame-playable evidence, never a
// completed replayable prefix.
void failed_stays_playable() {
    rr::Directory tmp;
    const auto publication = rr::record_failed(tmp.path);
    rr::recording_error([&] { (void)rr::replay::load_replay_input(publication.directory); },
        "not completed replay");
    const auto result = rr::replay::check(publication.directory, 0);
    bt::require(!result.matched, "a failed episode is never checked-replay evidence");
    const rr::replay::PlaybackSession playback(publication.directory, 0);
    bt::require(playback.data().times_s.size() >= 2,
        "a failed episode still carries recorded physics frames");
    {
        const rr::rec::Recording reader(publication.directory);
        const std::array params{rr::integer(playback.data().episode_id)};
        const auto rows = reader.rows(
            "SELECT count(*) AS n FROM physics WHERE episode_id=?", params);
        bt::require(rr::count(field(rows.at(0), "n")) ==
            static_cast<std::int64_t>(playback.data().times_s.size()),
            "playback frames are the recorded physics rows");
    }
}

// test_native_playback_never_loads_artifact_or_identity — the playback path
// constructs only a display model: no execution gate, runtime provenance or
// research machinery exists anywhere on it (there is nothing to spy on in a
// statically linked runtime — the contract is structural).
void playback_never_verifies() {
    rr::Directory tmp;
    const auto publication = rr::record_container(tmp.path);
    const auto data = rr::replay::load_playback(publication.directory, 0);
    // Python pins Playback.verified=False: stored poses are display evidence,
    // never verification. The native type carries no verified claim at all.
    const auto model = rr::replay::make_playback_model(data);
    const std::array params{rr::integer(data.episode_id)};
    std::int64_t physics_rows;
    {
        const rr::rec::Recording reader(publication.directory);
        physics_rows = rr::count(field(
            reader.rows("SELECT count(*) AS n FROM physics WHERE episode_id=?", params).at(0), "n"));
    }
    bt::require(static_cast<std::int64_t>(data.times_s.size()) == physics_rows &&
        data.qpos.size() == data.times_s.size() && data.qvel.size() == data.times_s.size(),
        "playback frame times are the recorded physics rows");
    const auto nq = static_cast<std::size_t>(model->nq), nv = static_cast<std::size_t>(model->nv);
    for (const auto& row : data.qpos) bt::require(row.size() == nq, "recorded qpos width matches the model");
    for (const auto& row : data.qvel) bt::require(row.size() == nv, "recorded qvel width matches the model");
}
} // namespace
int main() {
    return bt::run([] {
        v3_and_suffix_capture();
        failed_stays_playable();
        playback_never_verifies();
    });
}
