// Reader/analysis regressions authored from the source contract. Not run.
#include "recording_reader_test_support.hpp"
#include "recording_fixture.hpp"
#include "recording/reader.hpp"
#include "io/hash.hpp"
#include <limits>

namespace {
namespace rec = bike::recording;
namespace io = bike::io;
using namespace rec::detail;
void reader_and_analysis() {
    bt::require(io::recording_json(real(1000000.))=="1000000.0"&&io::recording_json(real(.0001))=="0.0001"&&
        io::recording_json(real(-0.))=="-0.0","recording float spelling shares the source fixed/exponent and signed-zero rules");
    recording_reader_test::Directory const tmp;
    const auto publication = recording_fixture::publish(tmp.path / "runs", 3);
    const auto before = bike::platform::file_sha256(publication.directory / "run.duckdb");
    rec::Recording reader(publication.directory);
    bt::require(reader.schema_version() == 1 && reader.episode_ids() == std::vector<std::int64_t>{0}, "container schema and selected episode identity");
    bt::invalid([&] { (void)reader.episode(-1); });
    bt::invalid([&] { (void)reader.episode(1); });
    bt::invalid([&] { (void)reader.episode(0, true); });
    const auto limited = reader.query("SELECT interval_id FROM physics ORDER BY interval_id", 2);
    bt::require(io::as_array(field(limited,"rows"),"rows").size() == 2 && io::as_bool(field(limited,"has_more"),"more") &&
        io::as_bool(field(limited,"truncated"),"truncated"), "query uses one lookahead row with explicit truncation");
    bt::invalid([&] { (void)reader.query("SELECT 1 AS v", 0); });
    bt::invalid([&] { (void)reader.query("SELECT 1 AS v", std::numeric_limits<std::uint64_t>::max()); });
    for (auto sql : {"CREATE TABLE injected(i INT)", "SELECT 1; SELECT 2", "DELETE FROM physics", "ATTACH 'other.db' AS other", "INSTALL httpfs"})
        recording_reader_test::rejects([&] { (void)reader.query(sql); });
    recording_reader_test::rejects([&] { (void)reader.query("SELECT * FROM read_csv('/etc/passwd')"); });
    const auto complex = reader.query("WITH q AS (SELECT 9007199254740993::BIGINT AS exact_integer) SELECT exact_integer,NULL::DOUBLE AS absent,"
        "'NaN'::DOUBLE AS invalid, [1.0,NULL,3.0]::DOUBLE[] AS vector, {'nullable':NULL,'empty':[]} AS record,"
        "map(['$bike_recording'],['literal']) AS reserved, CAST('1.20' AS DECIMAL(4,2)) AS decimal, from_hex('00ff') AS bytes FROM q");
    const auto& row = io::as_array(field(complex,"rows"),"rows").at(0);
    bt::require(std::get<io::Number>(field(row,"exact_integer").data).lexeme == "9007199254740993", "SQL bigint never rounds through double");
    bt::require(io::is_null(field(row,"absent")) && std::get<io::Tagged>(field(row,"invalid").data).kind == "float64", "null and nonfinite values remain distinct");
    bt::require(std::get<io::Bytes>(field(row,"bytes").data).value == std::vector<std::byte>{std::byte{0},std::byte{255}}, "query byte values are owned and lossless");
    bt::require(equal(complex, io::parse_recording_json(io::recording_json(complex))), "nested, decimal, bytes and reserved-key values round trip through tagged JSON");
    const auto data = rec::window(reader, 0, .0005, .002, "x_m,powers_w.chain,physics.qpos", 20);
    const auto& rows = io::as_array(field(data,"rows"),"window rows");
    const auto& overlap = io::as_array(field(field(data,"overlap_before_start"),"rows"),"overlap rows");
    bt::require(rows.size() == 1 && count(field(rows[0],"interval_id")) == 1 && overlap.size() == 1 &&
        count(field(overlap[0],"interval_id")) == 0, "window selects interval starts in [a,b), separately reports incoming overlap");
    bt::require(io::as_double(field(rows[0],"x_m"),"x") == .002 && io::as_double(field(rows[0],"powers_w.chain"),"power") == 5., "scalar/vector and parameterized force aliases use coordinate identity");
    bt::invalid([&] { (void)rec::window(reader, 0, 0., 1., "physics.qpos.missing"); });
    const auto same = rec::compare(reader, reader, 0, 0, "exact", "physics.qpos,forces.qfrc");
    bt::require(io::as_bool(field(same,"compatible"),"compatible") && io::as_bool(field(same,"agrees"),"agrees") &&
        count(field(same,"compared_intervals")) == 3, "two independent cursors compare every interval even for identical Recording objects");
    const auto short_run = recording_fixture::publish(tmp.path / "short", 2); rec::Recording const short_reader(short_run.directory);
    const auto prefix = rec::compare(reader, short_reader, 0, 0, "exact", "physics.qpos");
    bt::require(!io::as_bool(field(prefix,"agrees"),"agrees") && count(field(field(prefix,"unmatched_extents"),"left_intervals")) == 1,
        "matching common prefix does not pass unequal extents");
    bt::require(!io::as_array(field(rec::fields(reader,"physics.qp*",1),"rows"),"fields").empty(), "catalog glob search is bounded");
    bt::require(bike::platform::file_sha256(publication.directory / "run.duckdb") == before && !std::filesystem::exists(publication.directory / "run.duckdb.wal"),
        "read-only analysis leaves database bytes and sidecars untouched");
    recording_reader_test::Directory invalid;
    std::filesystem::create_symlink(publication.directory / "manifest.json", invalid.path / "manifest.json");
    recording_reader_test::rejects([&] { (void)rec::read_manifest(invalid.path); });
    std::filesystem::remove(invalid.path / "manifest.json");
    auto manifest = reader.manifest(); set(manifest,"schema_version",integer(3));
    recording_reader_test::write(invalid.path / "manifest.json", io::recording_json(manifest));
    bt::invalid([&] { (void)rec::read_manifest(invalid.path); });
    reader.close();reader.close();
    bt::invalid([&]{(void)reader.query("SELECT 1");});
}
}
int main() { return bt::run(reader_and_analysis); }
