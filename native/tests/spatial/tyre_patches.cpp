#include "support/migration_test.hpp"
#include "support/tyre_test_helpers.hpp"
#include "pneumatic3d/patches.hpp"

#include <algorithm>
#include <array>
#include <limits>
#include <vector>

int main() { return bt::run([] {
    using namespace bike::pneumatic3d;
    using namespace spatial_tests::tyre;
    std::array<ContactElement3d,8> e;
    std::array<NormalElement3d,8> n;
    for (std::size_t i=0;i<e.size();++i) {
        e[i]=element(static_cast<std::uint32_t>(i),.01);
        e[i].point_world_m={static_cast<double>(i),0.,0.};
        n[i].element=static_cast<std::uint32_t>(i);n[i].normal_load_n=10.;n[i].active=true;
    }
    PatchState3d s;
    s.capacity=8;s.ids.assign(8,0);s.slot_used.assign(8,0);
    s.element_to_patch_slot.assign(8,std::numeric_limits<std::uint32_t>::max());
    std::array<std::uint32_t,8> backing{};
    std::array<Patch3d,8> out;
    const auto prepare=[&] { for(auto& p:out) p.elements=backing; };
    std::size_t count=0;
    prepare();
    bt::require(assign_patches(e,n,true,s,out,count)==CoreStatus::ok && count==1,"connected fan is one patch");
    const auto first_id=out[0].id;
    bt::require(out[0].normal_load_n==80. && out[0].force_world_n[2]==80.,"normal wrench integrates exactly");
    bt::require(out[0].centroid_world_m[0]==3.5,"centroid is load weighted");
    n[2].active=false;n[2].normal_load_n=0.;
    prepare();
    bt::require(assign_patches(e,n,true,s,out,count)==CoreStatus::ok && count==2,"gap splits one contact");
    bt::require(out[1].id==first_id && out[0].id!=first_id,"largest descendant retains ID");
    const auto new_id=out[0].id;
    n[2].active=true;n[2].normal_load_n=10.;prepare();
    bt::require(assign_patches(e,n,true,s,out,count)==CoreStatus::ok && count==1,"gap closes again");
    bt::require(out[0].id==first_id && out[0].id!=new_id,"largest ancestor survives merge");
    for(std::size_t i=4;i<8;++i) e[i].surface_id=1;
    prepare();
    bt::require(assign_patches(e,n,true,s,out,count)==CoreStatus::ok && count==2,"material boundary splits patches");
    bt::require(out[0].normal_load_n+out[1].normal_load_n==80.,"identity split preserves load");
    const auto before=bytes(s);prepare();
    bt::require(assign_patches(e,n,false,s,out,count)==CoreStatus::ok && bytes(s)==before,"patch probe preserves committed IDs and membership");
    PatchState3d small;
    small.capacity=1;small.ids={0};small.slot_used={0};
    small.element_to_patch_slot.assign(8,std::numeric_limits<std::uint32_t>::max());
    const auto small_before=bytes(small);prepare();
    bt::require(assign_patches(e,n,true,small,out,count)==CoreStatus::invalid_geometry,"capacity overflow is explicit");
    bt::require(bytes(small)==small_before,"overflow leaves complete patch state unchanged");
    // Checkerboard material IDs make all eight cells disconnected, disproving
    // the ceil(N/2) bound even before a normal discontinuity is introduced.
    for(std::size_t i=0;i<8;++i) e[i].surface_id=static_cast<std::uint32_t>(i%2);
    prepare();bt::require(assign_patches(e,n,true,s,out,count)==CoreStatus::ok && count==8,"N slots cover material-separated cells");
    for(const auto& p:out) bt::require(p.elements.size()==1,"no disconnected contact is silently dropped");
    for(auto& x:n) { x.active=false;x.normal_load_n=0.; }
    prepare();bt::require(assign_patches(e,n,true,s,out,count)==CoreStatus::ok && count==0,"airborne patch inventory empty");
    bt::require(std::all_of(s.ids.begin(),s.ids.end(),[](auto id){return id==0;}),"inactive IDs canonical zero");
    for(std::size_t i=0;i<e.size();++i) {
        e[i].surface_id=0;
        n[i].active=true;n[i].normal_load_n=std::numeric_limits<double>::max()/4.;
    }
    const auto overflow_before=bytes(s);prepare();
    bt::require(assign_patches(e,n,true,s,out,count)==CoreStatus::invalid_geometry &&
                bytes(s)==overflow_before,"finite element loads cannot commit an overflowing aggregate patch");
}); }
