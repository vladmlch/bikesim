#include "support/migration_test.hpp"
#include "support/spatial_fixture.hpp"
#include "terrain/generator3d.hpp"
#include "terrain/trackfile.hpp"
#include "io/json.hpp"

#include <array>
#include <cstdint>
#include <iostream>
#include <string>
#include <sys/resource.h>

namespace {
std::uint64_t peak_rss_bytes() {
    struct rusage usage{};
    bt::require(::getrusage(RUSAGE_SELF,&usage)==0,"getrusage succeeds");
    bt::require(usage.ru_maxrss>=0,"peak RSS is nonnegative");
#if defined(__APPLE__)
    return static_cast<std::uint64_t>(usage.ru_maxrss);
#else
    return static_cast<std::uint64_t>(usage.ru_maxrss)*1024ull;
#endif
}
}
int main() { return bt::run([] {
    using namespace bike::terrain;
    Track3dSpec spec;
    spec.base_track_spec=track_to_dict(flat(300.));
    spec.dx_m=.005;spec.dy_m=.010;spec.half_width_m=1.5;
    spec.roughness={.002,.01};
    spec.surfaces={{1,"hardpack",.8,.6,12.,4.5,{}}};
    std::array<std::string,5> first_hashes;
    for(int pass=0;pass<2;++pass) for(std::size_t seed=0;seed<first_hashes.size();++seed) {
        spec.seed_decimal=std::to_string(seed);
        const auto grid=generate_grid3d(spec);
        bt::require(grid.nx()==60001 && grid.ny()==301,"declared 300 m physical terrain resolution retained");
        bt::require(grid.elevation().size()==18'060'301,"declared 300 m sample count");
        bt::require(grid.elevation().size_bytes()==144'482'408,"binary64 physical elevation bytes");
        const auto metadata_bytes=bike::io::canonical_json(grid.metadata().resolved_spec).size()+
            bike::io::canonical_json(grid.metadata().provenance).size();
        const auto visible_owner_bytes=sizeof(Grid3d)+grid.elevation().size_bytes()+grid.materials().size_bytes()+
            grid.surfaces().size_bytes()+metadata_bytes+grid.sha256_hex().size();
        bt::require(visible_owner_bytes<=(256ull<<20),"canonical owned terrain payload exceeds 256 MiB");
        const auto peak=peak_rss_bytes();
        bt::require(peak<=(512ull<<20),"generator process peak RSS exceeds 512 MiB");
        if(pass==0)first_hashes[seed]=grid.sha256_hex();
        else bt::require(first_hashes[seed]==grid.sha256_hex(),"seed hash differs between same-process generations");
        std::cout<<"{\"seed\":"<<seed<<",\"pass\":"<<pass<<",\"sha256\":\""<<grid.sha256_hex()
            <<"\",\"peak_rss_bytes\":"<<peak<<",\"visible_owner_bytes\":"<<visible_owner_bytes
            <<",\"allocator_overhead_bytes\":null}\n";
    }
}); }
