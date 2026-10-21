#pragma once
#include "migration_test.hpp"
#include "model/builder.hpp"
#include "engine_call.hpp"
#include "model_access.hpp"
#include <memory>
#include <string>

namespace bt {
struct DataDeleter { void operator()(mjData* data) const noexcept { mj_deleteData(data); } };
// Model precedes data so destruction always releases data before its model.
struct ModelFixture {
    bike::model::ModelOwner model;
    std::unique_ptr<mjData, DataDeleter> data;
    explicit ModelFixture(std::string_view xml) : model(bike::model::compile_mjcf(xml)), data(engine::make_data(model.get())) {
        require(data != nullptr, "MuJoCo test fixture owns its data");
    }
    [[nodiscard]] int dof(const char* name) const {
        const int id = mj_name2id(model.get(), mjOBJ_JOINT, name);
        model_access::require_id(id, model->njnt, name);
        const auto address = model_access::readonly_buffer(model->jnt_dofadr, model->njnt)[static_cast<std::size_t>(id)];
        model_access::require_id(address, model->nv, name);
        return static_cast<int>(address);
    }
};
} // namespace bt
