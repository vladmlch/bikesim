#include "render_owner.hpp"
#include "render_fields.hpp"
#include "../engine_call.hpp"
#include <algorithm>
#include <span>
#include <stdexcept>

namespace bike::viewer {
runtime::RenderFrame capture_frame(const mjModel& model, const mjData& data,
    int generation, std::int64_t step, std::string digest, runtime::PresentationState presentation) {
    if (generation < 0 || step < 0) throw std::invalid_argument("render frame: negative generation or step");
    runtime::RenderFrame frame;
    frame.model_sha256 = std::move(digest);
    frame.state.generation = generation;
    frame.state.step = step;
    frame.state.time_s = data.time;
    frame.state.integration_state.resize(static_cast<std::size_t>(engine::state_size(&model, mjSTATE_INTEGRATION)));
    engine::get_state(&model, &data, frame.state.integration_state.data(), mjSTATE_INTEGRATION);
    presentation.time_s = data.time;
    const auto fields = detail::readonly_model_fields(model);
    for (std::size_t i = 0; i < fields.size(); ++i)
        presentation.model_fields[i].assign(fields[i].begin(), fields[i].end());
    frame.presentation = std::move(presentation);
    frame.state.view = frame.presentation.as_wire();
    return frame;
}
} // namespace bike::viewer
