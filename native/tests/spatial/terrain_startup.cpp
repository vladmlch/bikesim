#include "support/migration_test.hpp"
#include "support/spatial_fixture.hpp"
#include "support/terrain_fixture.hpp"
#include "terrain/spec_io.hpp"
#include "terrain/asset3d.hpp"
#include "terrain/generator3d.hpp"
#include "terrain/render3d.hpp"
#include "model/builder.hpp"
#include "platform/files.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cstdlib>
#include <filesystem>
#include <memory>
#include <ranges>
#include <string>
#include <system_error>
#include <vector>

namespace {
using namespace bike::terrain;
struct TempDirectory {
    std::filesystem::path path;
    TempDirectory() {
        std::string pattern=(std::filesystem::temp_directory_path()/"bike-terrain3d-XXXXXX").string();
        std::vector<char> buffer(pattern.begin(),pattern.end());buffer.push_back('\0');
        const auto* made=::mkdtemp(buffer.data());
        if(!made)throw std::runtime_error("cannot create terrain test directory");
        path=made;
    }
    ~TempDirectory(){std::error_code error;std::filesystem::remove_all(path,error);}
    TempDirectory(const TempDirectory&)=delete;
    TempDirectory& operator=(const TempDirectory&)=delete;
};
bool exact(double a,double b){return std::bit_cast<std::uint64_t>(a)==std::bit_cast<std::uint64_t>(b);}
}

int main(){return bt::run([]{
    using bike::spatial::TerrainRequest3d;
    using bike::spatial::TerrainRequestKind;
    const auto genspec=spatial_tests::fixture("terrain/genspec/single_rock.toml");
    TerrainRequest3d request;request.kind=TerrainRequestKind::genspec;request.path=genspec;
    const auto generated=detail::load_terrain3d_source(request);
    bt::require(generated->grid()!=nullptr,"genspec produces owned grid");
    const auto payload=write_terrain3d(*generated->grid());
    TempDirectory temp;
    const auto asset=temp.path/"same.terrain3d";
    bike::platform::write_exclusive(asset,payload);
    request.kind=TerrainRequestKind::grid_asset;request.path=asset;
    const auto restored=detail::load_terrain3d_source(request);
    bt::require(generated->fingerprint_hex()==restored->fingerprint_hex(),"asset and corresponding genspec share canonical identity");
    std::array<GridSample3d,50> before{};
    for(std::size_t i=0;i<before.size();++i){
        const double x=20.*static_cast<double>(i)/static_cast<double>(before.size()-1);
        before[i]=generated->query_view().sample(x,.5);
        const auto loaded=restored->query_view().sample(x,.5);
        bt::require(before[i].status==QueryStatus3d::ok && exact(before[i].height_m,loaded.height_m),"loaded samples match generated samples");
    }
    const auto render_a=make_render_grid(*generated,{9,101});
    const auto render_b=make_render_grid(*generated,{5,51});
    bt::require(render_a.physical_asset_sha256_hex==generated->fingerprint_hex() &&
        render_b.physical_asset_sha256_hex==generated->fingerprint_hex(),"render provenance retains physical identity");
    bt::require(render_a.resolution.ncol!=render_b.resolution.ncol && render_a.normalized_height.size()!=render_b.normalized_height.size(),"render resolution has distinct provenance");
    for(std::size_t i=0;i<before.size();++i){
        const double x=20.*static_cast<double>(i)/static_cast<double>(before.size()-1);
        const auto after=generated->query_view().sample(x,.5);
        bt::require(exact(after.height_m,before[i].height_m) && after.normal_world==before[i].normal_world &&
            after.surface_id==before[i].surface_id,"render resampling cannot mutate physical terrain");
    }
    const Terrain3dSource flat(spatial_tests::plane_grid(0.,0.,2.,8,7,.5,.5));
    const auto constant=make_render_grid(flat,{4,6});
    bt::require(constant.z_min_m==2. && constant.z_max_m==2. &&
        std::ranges::all_of(constant.normalized_height,[](float value){return value==0.f;}),"constant grid has zero normalized heights and original datum");
    const auto varying=std::make_shared<const Terrain3dSource>(spatial_tests::plane_grid(.1,.2,.3,16,7,.25,.5));
    const auto raster=make_render_grid(*varying,{7,5});
    for(std::uint32_t row=1;row<raster.nrow;++row)
        bt::require(raster.normalized_height[row*raster.ncol]!=raster.normalized_height[(row-1)*raster.ncol],"asymmetric render rows are distinct");

    auto model_request=std::make_unique<bike::model::BuildRequest>();
    model_request->mode="ride";
    const auto planar_before=bike::model::generate_mjcf(*model_request);
    bike::spatial::Config config;config.mode=bike::spatial::Mode::spatial3d;config.root=bike::spatial::RootKind::constrained4;
    config.steer.controller=bike::spatial::SteerController::off;
    config.terrain.kind=TerrainRequestKind::grid_asset;config.terrain.path=asset;
    model_request->physics=bike::configuration::SimulationPhysicsConfig{};
    model_request->physics->physics_mode="physical";model_request->physics->tires.backend="pneumatic_3d";
    model_request->physics->spatial=config;model_request->spatial=config;model_request->terrain3d=varying;
    model_request->field.nrow=7;model_request->field.ncol=5;
    const auto model=bike::model::compile_mjcf(bike::model::generate_mjcf(*model_request));
    bike::model::populate_render_heightfield(*model,raster);
    // Checked views over the engine arrays: a raw pointer subscript is rejected by
    // the repository's -Wunsafe-buffer-usage policy (see legacy/ride.cpp).
    const int field=mj_name2id(model.get(),mjOBJ_HFIELD,"road");
    bt::require(field>=0,"compiled terrain field exists");
    const auto field_index=static_cast<std::size_t>(field);
    const auto field_rows=std::views::counted(model->hfield_nrow,model->nhfield);
    const auto field_columns=std::views::counted(model->hfield_ncol,model->nhfield);
    const auto field_addresses=std::views::counted(model->hfield_adr,model->nhfield);
    const auto field_data=std::views::counted(model->hfield_data,model->nhfielddata);
    const auto start=static_cast<std::size_t>(field_addresses[field_index]);
    bt::require(field_rows[field_index]==7 && field_columns[field_index]==5,"compiled hfield preserves rectangular resolution");
    for(std::size_t i=0;i<raster.normalized_height.size();++i)
        bt::require(field_data[start+i]==raster.normalized_height[i],"all hfield rows are populated");
    const int ground=mj_name2id(model.get(),mjOBJ_GEOM,"terrain");
    bt::require(ground>=0,"compiled terrain geom exists");
    const auto contype=std::views::counted(model->geom_contype,model->ngeom);
    const auto conaffinity=std::views::counted(model->geom_conaffinity,model->ngeom);
    const auto ground_index=static_cast<std::size_t>(ground);
    for(const char* name:{"geom_front_tire","geom_rear_tire"}){
        const int wheel=mj_name2id(model.get(),mjOBJ_GEOM,name);
        bt::require(wheel>=0,"wheel collision geom exists");
        const auto wheel_index=static_cast<std::size_t>(wheel);
        bt::require((contype[wheel_index]&conaffinity[ground_index])==0 &&
            (contype[ground_index]&conaffinity[wheel_index])==0,"pneumatic3d wheel has one physical contact owner");
    }
    model_request=std::make_unique<bike::model::BuildRequest>();model_request->mode="ride";
    bt::require(planar_before==bike::model::generate_mjcf(*model_request),"spatial render work does not alter planar builder state");
    request.dx_m=.01;bt::invalid([&]{(void)detail::load_terrain3d_source(request);});
    request.dx_m.reset();request.corridor_half_width_m=2.;bt::invalid([&]{(void)detail::load_terrain3d_source(request);});
    request.corridor_half_width_m=1.5;request.max_asset_bytes=1;bt::invalid([&]{(void)detail::load_terrain3d_source(request);});
    request.kind=TerrainRequestKind::none;bt::invalid([&]{(void)detail::load_terrain3d_source(request);});
    bt::invalid([&]{(void)make_render_grid(*generated,{0,10});});
    bt::invalid([&]{(void)make_render_grid(flat,{65536,2048});});
});}
