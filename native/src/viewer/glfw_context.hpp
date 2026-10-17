#pragma once
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace bike::viewer {
struct WindowEvent {
    enum class Kind { key, mouse, scroll } kind = Kind::key;
    int key = 0, action = 0;
    double dx = 0., dy = 0.;
    int modifiers = 0;
};
class GlfwContext {
public:
    GlfwContext(const std::string& title, int width, int height, bool hidden = false);
    ~GlfwContext();
    GlfwContext(const GlfwContext&) = delete;
    GlfwContext& operator=(const GlfwContext&) = delete;
    [[nodiscard]] std::vector<WindowEvent> poll();
    [[nodiscard]] bool open() const;
    [[nodiscard]] std::pair<int, int> framebuffer_size() const;
    void swap();
    void wait(double seconds);
    void request_close();
private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
} // namespace bike::viewer
