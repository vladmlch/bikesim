#pragma once
#include "../runtime/runtime.hpp"
#include <array>
#include <memory>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace bike::viewer {
// NOLINTNEXTLINE(performance-enum-size) enum width matches the reference ABI layout
enum class CameraMode : std::uint8_t { side, chase };
class Camera {
public:
    explicit Camera(double smoothing = .08);
    void set_mode(CameraMode);
    void cycle_mode();
    void reset_preset() noexcept;
    void update(mjvCamera&, double bike_x, double bike_z);
    void update_spatial(mjvCamera&, const std::array<double, 3>& position_world_m,
                        std::optional<double> heading_rad);
    [[nodiscard]] std::string_view mode_name() const noexcept;
private:
    CameraMode mode_ = CameraMode::side;
    double smoothing_;
    std::optional<std::array<double, 3>> lookat_;
    bool reset_ = true;
};
// The object is main-thread confined. Construction and apply need no GL
// context; draw initializes GPU resources lazily. Destroy it BEFORE its GLFW
// context. It exposes only owned inspection values, never mutable mjData.
class RenderOwner {
public:
    explicit RenderOwner(const runtime::ModelImage&);
    ~RenderOwner();
    RenderOwner(const RenderOwner&) = delete;
    RenderOwner& operator=(const RenderOwner&) = delete;
    RenderOwner(RenderOwner&&) = delete;
    RenderOwner& operator=(RenderOwner&&) = delete;
    void apply(const runtime::RenderFrame&);
    void draw(int framebuffer_width, int framebuffer_height);
    void set_camera(CameraMode);
    void cycle_camera();
    void reset_camera();
    void move_camera(int action, double dx, double dy);
    void set_markers(bool);
    void set_overlay(std::string);
    void close() noexcept;
    [[nodiscard]] bool closed() const noexcept;
    [[nodiscard]] bool has_markers() const noexcept;
    [[nodiscard]] std::vector<double> integration_state() const;
    [[nodiscard]] std::array<std::vector<double>, 6> model_fields() const;
    [[nodiscard]] std::string_view model_sha256() const noexcept;
private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
// A common, observational capture helper for the legacy owners. The owner
// supplies a digest belonging to this compiled model and serializes capture.
[[nodiscard]] runtime::RenderFrame capture_frame(const mjModel&, const mjData&,
    int generation, std::int64_t step, std::string model_sha256,
    runtime::PresentationState presentation = {});
} // namespace bike::viewer
