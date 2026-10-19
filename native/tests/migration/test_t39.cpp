// Lossless piece domains and generic-source presence. Not executed.
#include "support/migration_test.hpp"
#include "recording_fixture.hpp"
#include "recording/encoding.hpp"
#include "runtime/encode.hpp"
#include <bit>
#include <limits>

namespace {
namespace rec = bike::recording;
namespace io = bike::io;
rec::FieldSpec field(std::string name,std::string type,bool optional=false) {
    return {.key=name,.dtype=std::move(type),.unit="1",.phase="interval",.optional=optional,.measurement="declared",.source="authored regression",.description="source value",.frame="n/a",.path={name},.aliases={},.shape={},.axes={},.except_keys={},.children={}};
}
rec::TableSpec spec() {
    auto structure=field("object","struct",true);
    structure.children={field("id","bigint"),field("note","varchar",true)};
    auto map=field("map","map<bigint>",true);
    return {.name="authored",.kind="physics",.keys={field("episode_id","bigint"),field("interval_id","bigint")},
        .fields={field("vector","double[]",true),structure,map},.description="authored values",.source="T39",.claims={}, .exempt_keys={}};
}
void generic_contract() {
    auto table=spec(); table.validate();
    std::vector<io::Value::Object> rows{
        {{"interval_id",rec::value::integer(0)}},
        {{"interval_id",rec::value::integer(1)},{"vector",io::Value{}},{"object",io::Value{}},{"map",io::Value{}}},
        {{"interval_id",rec::value::integer(2)},{"vector",io::Value{io::Value::Array{}}},
         {"object",io::Value{io::Value::Object{{"id",rec::value::integer(9007199254740993LL)},{"note",io::Value{}}}}},
         {"map",io::Value{io::Value::Object{{"big",rec::value::integer(9007199254740993LL)}}}}}};
    auto result=rec::encode_rows(table,rows,4);
    bt::require(result.columns.at("vector_present").b==std::vector<char>({0,1,1}), "absent, explicit null and empty list have distinct presence");
    bt::require(result.columns.at("vector").valid==std::vector<char>({0,0,1}), "empty list is valid while absent/null are invalid");
    bt::require(result.columns.at("object").children.at("id").i.back()==9007199254740993LL,
        "nested integer stays above the binary64 exact integer range");
    bt::require(result.columns.at("map").items->i.back()==9007199254740993LL,
        "the map<bigint> generic fallback is typed and lossless");
    auto fragment=rec::slice_table(result,2,3); rec::validate_pieces(fragment,table);
    bt::require(fragment.columns.at("map").offsets==std::vector<std::int32_t>({0,1}), "nested map slicing rebases offsets");
    std::size_t batches=0, extent=0;
    rec::piece_fragments("authored",std::move(result),1,[&](std::string_view,runtime::EncodedTable&& batch){  // NOLINT(cppcoreguidelines-rvalue-reference-param-not-moved) callback signature is fixed by piece_fragments; the batch is validated in place
        rec::validate_pieces(batch,table); ++batches; extent+=static_cast<std::size_t>(batch.rows);
    });
    bt::require(batches==3&&extent==3,"fragmentation covers exactly the requested immutable prefix");
    rows[2].emplace_back("unknown",io::Value{true});
    bt::invalid([&]{(void)rec::encode_rows(table,rows,4);}); rows[2].pop_back();
    rec::value::set(rows[2],"map",io::Value{io::Value::Object{{"duplicate",rec::value::integer(1)},{"duplicate",rec::value::integer(2)}}});
    bt::invalid([&]{(void)rec::encode_rows(table,rows,4);});
}
void physical_contract() {
    auto episode=recording_fixture::episode(3);
    auto const modified=std::make_shared<runtime::PhysicalSampleData>(*episode.samples[0]);
    const auto nan=std::bit_cast<double>(std::uint64_t{0x7ff8000000001234ULL});
    modified->qvel[0]=nan; episode.samples[0]=modified;
    const auto catalog=recording_fixture::catalog();
    auto table=runtime::encode_table_native(episode,rec::compile_plan(catalog,"physics"),0,3);
    rec::validate_pieces(table,catalog,"physics");
    bt::require(std::bit_cast<std::uint64_t>(table.columns.at("qvel").d[0])==0x7ff8000000001234ULL,
        "optimized publication preserves a nondefault NaN payload");
    bt::require(std::signbit(table.columns.at("qpos").d[1]),"optimized publication preserves signed zero");
    auto moved=rec::slice_table(table,0,2); auto const again=std::move(moved);
    rec::validate_pieces(again,catalog,"physics");
    table.columns.at("qpos").offsets.back()=1;
    bt::invalid([&]{rec::validate_pieces(table,catalog,"physics");});
    auto const channels=runtime::encode_table_native(episode,rec::compile_plan(catalog,"channel_energy"),0,3);
    bt::require(channels.rows==3,"absent roots retain physical coverage");
    modified->channels.emplace_back("component_work_j",runtime::WireObject{});
    auto empty_root=runtime::encode_table_native(episode,rec::compile_plan(catalog,"channel_energy"),0,3);
    rec::validate_pieces(empty_root,catalog,"channel_energy");
    bt::require(empty_root.columns.at("__component_work_j_present").b[0] &&
        empty_root.columns.at("__component_work_j_valid").b[0] &&
        !empty_root.columns.at("component_work_j_present").b[0],
        "an empty routed root remains present/valid while its terminal leaf is absent");
    empty_root.columns.at("__component_work_j_valid").b[0]=0;
    bt::invalid([&]{rec::validate_pieces(empty_root,catalog,"channel_energy");});
    modified->channels.emplace_back("unreviewed",runtime::Wire{1.});
    bt::invalid([&]{(void)runtime::encode_table_native(episode,rec::compile_plan(catalog,"channel_energy"),0,3);});
}
}
int main(){return bt::run([]{generic_contract();physical_contract();});}
