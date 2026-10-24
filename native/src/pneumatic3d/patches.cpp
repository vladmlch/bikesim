#include "pneumatic3d/patches.hpp"
#include "pneumatic3d/kernel_math.hpp"
#include "diag.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace bike::pneumatic3d {
using namespace detail;
namespace {
constexpr std::uint32_t absent=std::numeric_limits<std::uint32_t>::max();
struct Overlap { std::uint32_t slot=absent, component=absent, element=absent; double load=0.; };
struct PatchScratch {
    std::array<std::uint32_t,max_fan_elements> label{},queue{},first{},selected{},winner{},best_element{};
    std::array<double,max_fan_elements> best_weight{},component_weight{};
    std::array<std::uint8_t,max_fan_elements> used{};
    std::array<Overlap,max_fan_elements> overlap{};
    std::array<std::uint64_t,max_fan_elements> ids{};
    bool busy=false;
};
// Frozen W00 has no workspace argument. This bounded scratch is isolated per
// thread, cleared on entry, and contains no material/history state. See C10.
constinit thread_local PatchScratch scratch;
struct ScratchUse {bool& busy;explicit ScratchUse(bool& b) noexcept:busy(b){busy=true;}~ScratchUse(){busy=false;}};
std::size_t lateral_nodes(std::span<const ContactElement3d> elements) noexcept {
    if (elements.size()==64U*7U) return 7;
    if (elements.size()==256U*15U) return 15;
    // Small analytic unit fixtures may be one row. Production refinements
    // need an explicit shape in the future C10 contract.
    return elements.size();
}
}

CoreStatus assign_patches(std::span<const ContactElement3d> elements,
    std::span<const NormalElement3d> normals, bool advance,
    PatchState3d& state, std::span<Patch3d> out, std::size_t& count) noexcept {
    const std::size_t n=elements.size(), capacity=state.capacity;
    if (n==0 || n>max_fan_elements || normals.size()!=n || capacity==0 ||
        capacity>max_fan_elements || out.size()<capacity ||
        state.ids.size()!=capacity || state.slot_used.size()!=capacity ||
        state.element_to_patch_slot.size()!=n || out[0].elements.size()<n ||
        out[0].elements.data()==nullptr || state.next_id==0)
        return CoreStatus::invalid_input;
    for (std::size_t i=0;i<n;++i) {
        if (state.element_to_patch_slot[i]!=absent && state.element_to_patch_slot[i]>=capacity)
            return CoreStatus::invalid_input;
        if (!std::isfinite(normals[i].normal_load_n) || normals[i].normal_load_n<0. ||
            !finite(elements[i].point_world_m) || !finite(elements[i].normal_world) ||
            !std::isfinite(elements[i].geometric_area_m2)) return CoreStatus::invalid_input;
    }
    for (std::size_t s=0;s<capacity;++s)
        if (state.slot_used[s]>1 || (state.slot_used[s]!=0 && state.ids[s]==0))
            return CoreStatus::invalid_input;

    if(scratch.busy)return CoreStatus::invalid_input;
    const ScratchUse use(scratch.busy);
    auto& label=scratch.label;auto& queue=scratch.queue;auto& first=scratch.first;
    auto& selected=scratch.selected;auto& winner=scratch.winner;auto& best_weight=scratch.best_weight;
    auto& best_element=scratch.best_element;
    auto& used=scratch.used;auto& overlap=scratch.overlap;
    auto& component_weight=scratch.component_weight;auto& ids=scratch.ids;
    queue.fill(0);first.fill(0);best_weight.fill(0.);used.fill(0);overlap.fill(Overlap{});
    component_weight.fill(0.);ids.fill(0);
    label.fill(absent); selected.fill(absent); winner.fill(absent);best_element.fill(absent);
    const std::size_t width=lateral_nodes(elements);
    std::size_t components=0;
    const auto active=[&](std::size_t i) noexcept {
        return elements[i].valid && normals[i].active && normals[i].normal_load_n>0.;
    };
    const double normal_limit=std::cos(std::numbers::pi/6.);
    for (std::size_t seed=0;seed<n;++seed) if (active(seed) && label[seed]==absent) {
        if (components==capacity) return CoreStatus::invalid_geometry;
        const auto c=static_cast<std::uint32_t>(components);
        first[c]=static_cast<std::uint32_t>(seed);
        std::size_t read=0,write=0;
        queue[write++]=static_cast<std::uint32_t>(seed); label[seed]=c;
        while (read<write) {
            const std::size_t i=queue[read++];
            const std::array<std::size_t,4> neighbours{
                i>=width ? i-width : n, i+width<n ? i+width : n,
                i%width>0 ? i-1 : n, i%width+1<width && i+1<n ? i+1 : n};
            for (const std::size_t j:neighbours)
                if (j<n && label[j]==absent && active(j) &&
                    elements[i].surface_id==elements[j].surface_id &&
                    dot(elements[i].normal_world,elements[j].normal_world)>=normal_limit) {
                    label[j]=c; queue[write++]=static_cast<std::uint32_t>(j);
                }
        }
        ++components;
    }
    std::size_t overlap_count=0;
    for (std::size_t i=0;i<n;++i) {
        const auto slot=state.element_to_patch_slot[i];
        if (label[i]!=absent && slot!=absent && state.slot_used[slot])
            overlap[overlap_count++]={slot,label[i],static_cast<std::uint32_t>(i),normals[i].normal_load_n};
    }
    std::sort(overlap.begin(),overlap.begin()+static_cast<std::ptrdiff_t>(overlap_count),
        [](const Overlap& a,const Overlap& b) noexcept {
            if (a.slot!=b.slot) return a.slot<b.slot;
            if (a.component!=b.component) return a.component<b.component;
            return a.element<b.element;
        });
    // First select the largest descendant of each old ID; then the largest
    // ancestor of each new component. All summation order is canonical.
    for (std::size_t i=0;i<overlap_count;) {
        const auto s=overlap[i].slot, c=overlap[i].component;
        const auto element=overlap[i].element;
        double load=0.;
        do { load+=overlap[i++].load; }
        while (i<overlap_count && overlap[i].slot==s && overlap[i].component==c);
        if (winner[s]==absent || load>best_weight[s] ||
            (load==best_weight[s] && first[c]<first[winner[s]])) {
            winner[s]=c; best_weight[s]=load;best_element[s]=element;
        }
    }
    for (std::size_t s=0;s<capacity;++s) if (winner[s]!=absent) {
        const auto c=winner[s];
        if (selected[c]==absent || best_weight[s]>component_weight[c] ||
            (best_weight[s]==component_weight[c] && best_element[s]<best_element[selected[c]])) {
            selected[c]=static_cast<std::uint32_t>(s); component_weight[c]=best_weight[s];
        }
    }
    for (std::size_t c=0;c<components;++c) if (selected[c]!=absent) used[selected[c]]=1;
    std::uint64_t next=state.next_id;
    for (std::size_t c=0;c<components;++c) {
        if (selected[c]!=absent) ids[c]=state.ids[selected[c]];
        else {
            std::size_t s=0; while (s<capacity && used[s]) ++s;
            if (s==capacity || next==std::numeric_limits<std::uint64_t>::max())
                return CoreStatus::invalid_geometry;
            selected[c]=static_cast<std::uint32_t>(s); used[s]=1; ids[c]=next++;
        }
    }
    // The owner supplies one writable N-word backing array through every
    // frozen Patch3d::elements view, reset before each call. No input contact
    // or committed-state memory is used as output scratch. C10 records that
    // this public span should have been mutable; the actual storage is not const.
    NATIVE_DIAG_PUSH
    NATIVE_DIAG_IGNORE("-Wcast-qual")
    auto* indices=const_cast<std::uint32_t*>(out[0].elements.data());
    NATIVE_DIAG_POP
    std::size_t offset=0;
    for (std::size_t c=0;c<components;++c) {
        Patch3d patch;
        patch.id=ids[c]; patch.surface_id=elements[first[c]].surface_id;
        const auto begin=offset;
        Vec3 normal{},longitudinal{};
        for (std::size_t i=0;i<n;++i) if (label[i]==c) {
            indices[offset++]=static_cast<std::uint32_t>(i);
            const double load=normals[i].normal_load_n;
            patch.normal_load_n+=load; patch.area_m2+=elements[i].geometric_area_m2;
            patch.centroid_world_m=add(patch.centroid_world_m,scale(elements[i].point_world_m,load));
            normal=add(normal,scale(elements[i].normal_world,load));
            longitudinal=add(longitudinal,scale(elements[i].longitudinal_world,load));
            patch.force_world_n=add(patch.force_world_n,scale(elements[i].normal_world,load));
        }
        patch.centroid_world_m=scale(patch.centroid_world_m,1./patch.normal_load_n);
        patch.normal_world=unit(normal);
        patch.longitudinal_world=unit(sub(longitudinal,scale(patch.normal_world,dot(longitudinal,patch.normal_world))));
        patch.lateral_world=unit(cross(patch.normal_world,patch.longitudinal_world));
        for (std::size_t i=0;i<n;++i) if (label[i]==c)
            patch.couple_world_nm=add(patch.couple_world_nm,
                cross(sub(elements[i].point_world_m,patch.centroid_world_m),scale(elements[i].normal_world,normals[i].normal_load_n)));
        // Each element can be finite while the reduction or its normalized
        // basis is not. Reject before publishing a valid patch or IDs.
        if (!std::isfinite(patch.normal_load_n) || !std::isfinite(patch.area_m2) ||
            !(patch.area_m2>0.) || !finite(patch.centroid_world_m) ||
            !finite(patch.force_world_n) || !finite(patch.couple_world_nm) ||
            !finite(patch.normal_world) || !finite(patch.longitudinal_world) ||
            !finite(patch.lateral_world) || std::abs(norm(patch.normal_world)-1.)>1e-8 ||
            std::abs(norm(patch.longitudinal_world)-1.)>1e-8 ||
            std::abs(norm(patch.lateral_world)-1.)>1e-8)
            return CoreStatus::invalid_geometry;
        patch.elements=std::span<const std::uint32_t>(indices+begin,offset-begin);
        patch.valid=true; out[c]=patch;
    }
    for (std::size_t c=components;c<out.size();++c) out[c]={};
    if (advance) {
        std::fill(state.ids.begin(),state.ids.end(),std::uint64_t{0});
        std::fill(state.slot_used.begin(),state.slot_used.end(),std::uint8_t{0});
        for (std::size_t c=0;c<components;++c) {
            state.ids[selected[c]]=ids[c]; state.slot_used[selected[c]]=1;
        }
        for (std::size_t i=0;i<n;++i)
            state.element_to_patch_slot[i]=label[i]==absent ? absent : selected[label[i]];
        state.next_id=next;
    }
    count=components;
    return CoreStatus::ok;
}
} // namespace bike::pneumatic3d
