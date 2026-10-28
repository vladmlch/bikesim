// Batch 3.34 shared pinned-plant builder — the migration welded bootstrap
// (tests/fixtures/migration/bootstrap/welded) carries the pinned
// spindle/pin/connect plant's compiled model.mjb plus its projected
// writer_config; test_pinned_measurement.cpp and
// test_realtime_deferred_checks.cpp both construct the same
// (model, data, RiderContactsConfig) triple from it, like the oracle's
// test_pinned_topology._compiled().
#pragma once

#include "support/migration_test.hpp"
#include "runtime_reference_support.hpp"
#include "engine_call.hpp"
#include "io/hash.hpp"
#include "writer_config.hpp"
#include "writers/rider_contacts.hpp"
#include <mujoco/mujoco.h>
#include <array>
#include <cmath>
#include <cstddef>
#include <memory>
#include <ranges>
#include <span>
#include <string>

namespace pinned_test {

namespace rr = runtime_reference;

// mjData/mjModel arrays under -Wunsafe-buffer-usage: counted views, exactly
// like test_native_attachment_wrench.cpp's field()/dof helpers.
template <class T>
[[nodiscard]] inline std::span<const T> cbuf(const T *p, mjtSize n) {
    return std::views::counted(p, n);
}
template <class T>
[[nodiscard]] inline std::span<T> mbuf(T *p, mjtSize n) {
    return std::views::counted(p, n);
}
[[nodiscard]] inline int int_at(const int *p, mjtSize n, int index) {
    return cbuf(p, n)[static_cast<std::size_t>(index)];
}

struct ModelDelete {
    void operator()(mjModel *value) const noexcept {
        if (value) mj_deleteModel(value);
    }
};
struct DataDelete {
    void operator()(mjData *value) const noexcept {
        if (value) mj_deleteData(value);
    }
};

// The pinned plant pair: compiled model image plus the projected
// RiderContactsConfig, both straight from the welded capture.
struct Pinned {
    std::unique_ptr<mjModel, ModelDelete> model;
    std::unique_ptr<mjData, DataDelete> data;
    rider::RiderContactsConfig contacts;
};

inline Pinned pinned() {
    const auto dir = rr::welded_dir();
    const auto image = rr::model_image(dir);
    Pinned out{
        .model = {engine::load_model_image(image.mjb), ModelDelete{}},
        .data = {nullptr, DataDelete{}},
        .contacts = {},
    };
    out.data.reset(engine::make_data(out.model.get()));
    const auto config = rr::parse_doc(dir / "config.json");
    const auto decoded = nativecfg::decode(rr::member(config, "writer_config"));
    bt::require(decoded.rider_contacts.has_value(),
                "the projected config carries rider_contacts");
    out.contacts = *decoded.rider_contacts;
    return out;
}

inline int eq_id(const mjModel *m, const char *name) {
    const int id = mj_name2id(m, mjOBJ_EQUALITY, name);
    bt::require(id >= 0, std::string("equality ") + name + " resolves");
    return id;
}

// xpos[body] + xmat[body] @ eq_data[eq, offset:offset+3] — the python
// row's body-frame anchor rotated to world.
inline std::array<double, 3> world_anchor(const mjModel *m, const mjData *d,
                                          int eq, int body, int offset) {
    std::array<double, 3> point{};
    const auto xmat =
        cbuf(d->xmat, 9 * m->nbody).subspan(9 * static_cast<std::size_t>(body), 9);
    const auto xpos =
        cbuf(d->xpos, 3 * m->nbody).subspan(3 * static_cast<std::size_t>(body), 3);
    const auto anchor = cbuf(m->eq_data, m->neq * mjNEQDATA)
                            .subspan(static_cast<std::size_t>(eq) * mjNEQDATA +
                                         static_cast<std::size_t>(offset),
                                     3);
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            point[static_cast<std::size_t>(r)] +=
                xmat[static_cast<std::size_t>(3 * r + c)] *
                anchor[static_cast<std::size_t>(c)];
    for (int r = 0; r < 3; ++r)
        point[static_cast<std::size_t>(r)] +=
            xpos[static_cast<std::size_t>(r)];
    return point;
}

inline bool close(std::span<const double> a, std::span<const double> b,
                  double atol) {
    if (a.size() != b.size()) return false;
    for (std::size_t i = 0; i < a.size(); ++i)
        if (std::abs(a[i] - b[i]) > atol + 1e-10 * std::abs(b[i]))
            return false;
    return true;
}

} // namespace pinned_test
