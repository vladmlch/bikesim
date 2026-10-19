#include "glfw_context.hpp"
#include <cmath>
#include <exception>
#include <stdexcept>
#include <string_view>
#include <thread>
#include <mujoco/mujoco.h>
#if defined(BIKE_BUILD_GUI) && BIKE_BUILD_GUI
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#if defined(__APPLE__)
#include <pthread.h>
#endif
#endif

namespace bike::viewer {
namespace {
// Initialized when the native process loads this TU, before main().
const std::thread::id main_thread = std::this_thread::get_id();
void require_main_thread() {
    if (std::this_thread::get_id() != main_thread)
        throw std::runtime_error("GLFW lifecycle requires the application main thread");
#if defined(__APPLE__) && defined(BIKE_BUILD_GUI) && BIKE_BUILD_GUI
    if (pthread_main_np() == 0) throw std::runtime_error("GLFW must run on the macOS main thread");
#endif
}
#if defined(BIKE_BUILD_GUI) && BIKE_BUILD_GUI
unsigned context_count = 0;
std::string glfw_error(std::string_view operation) {
    const char* detail = nullptr;
    const int code = glfwGetError(&detail);
    return std::string(operation) + " (GLFW " + std::to_string(code) + "): " +
        (detail ? detail : "no display/context available");
}
#else
// Runtime capability probe: always false in a non-GUI build, opaque to the
// caller's never-returns analysis.
[[gnu::noinline]] bool gui_linked() { return false; }
#endif
}
struct GlfwContext::Impl {
    std::vector<WindowEvent> events;
    std::exception_ptr callback_failure;
#if defined(BIKE_BUILD_GUI) && BIKE_BUILD_GUI
    GLFWwindow* window = nullptr;
    double cursor_x = 0., cursor_y = 0.;
    bool registered = false;
    template<class F> static void callback(GLFWwindow* target, F&& fn) noexcept {
        auto* p = static_cast<Impl*>(glfwGetWindowUserPointer(target));
        if (!p || p->callback_failure) return;
        try { fn(*p); } catch (...) { p->callback_failure = std::current_exception(); }
    }
    void push(WindowEvent event) {
        if (events.size() >= 4096) throw std::runtime_error("viewer input queue exceeded 4096 events");
        events.push_back(event);
    }
    ~Impl() {
        if (window) glfwDestroyWindow(window);
        if (registered && --context_count == 0) glfwTerminate();
    }
#endif
};
GlfwContext::GlfwContext(const std::string& title, int width, int height, bool hidden)
    : impl_(std::make_unique<Impl>()) {
    require_main_thread();
    if (width <= 0 || height <= 0 || width > 16384 || height > 16384 || title.find('\0') != std::string::npos)
        throw std::invalid_argument("viewer window dimensions/title are invalid");
#if defined(BIKE_BUILD_GUI) && BIKE_BUILD_GUI
    if (context_count == 0 && glfwInit() != GLFW_TRUE) throw std::runtime_error(glfw_error("viewer initialization failed"));
    ++context_count;
    auto& p = *impl_; p.registered = true;
    glfwDefaultWindowHints();
    glfwWindowHint(GLFW_VISIBLE, hidden ? GLFW_FALSE : GLFW_TRUE);
    glfwWindowHint(GLFW_DOUBLEBUFFER, GLFW_TRUE);
    glfwWindowHint(GLFW_SAMPLES, 4);
    p.window = glfwCreateWindow(width, height, title.c_str(), nullptr, nullptr);
    if (!p.window) throw std::runtime_error(glfw_error("viewer window creation failed"));
    glfwMakeContextCurrent(p.window);
    glfwSwapInterval(0); // pacing is cooperative and separate from fixed physics dt
    glfwGetCursorPos(p.window, &p.cursor_x, &p.cursor_y);
    glfwSetWindowUserPointer(p.window, &p);
    glfwSetKeyCallback(p.window, [](GLFWwindow* window, int key, int, int action, int modifiers) {
        // Match the source's press-only toggle semantics. Repeat must not
        // release a brake or issue a second reset while a key remains held.
        if (action != GLFW_PRESS) return;
        Impl::callback(window, [key,modifiers](Impl& owner) { owner.push({WindowEvent::Kind::key, key, 0, 0., 0.,modifiers}); });
    });
    glfwSetCursorPosCallback(p.window, [](GLFWwindow* window, double x, double y) {
        Impl::callback(window, [window, x, y](Impl& owner) {
            const double dx = x - owner.cursor_x, dy = y - owner.cursor_y;
            owner.cursor_x = x; owner.cursor_y = y;
            const bool left = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
            const bool right = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
            const bool middle = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS;
            if (!left && !right && !middle) return;
            int view_width = 0, view_height = 0; glfwGetWindowSize(window, &view_width, &view_height);
            if (view_width <= 0 || view_height <= 0) return;
            const bool shift = glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS;
            const int action = right ? (shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V) :
                left ? (shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V) : mjMOUSE_ZOOM;
            owner.push({WindowEvent::Kind::mouse, 0, action, dx / view_height, dy / view_height});
        });
    });
    glfwSetScrollCallback(p.window, [](GLFWwindow* window, double, double dy) {
        Impl::callback(window, [dy](Impl& owner) { owner.push({WindowEvent::Kind::scroll, 0, mjMOUSE_ZOOM, 0., -.05 * dy}); });
    });
#else
    (void)hidden;
    if (!gui_linked()) throw std::runtime_error("viewer unavailable: configure BIKE_BUILD_GUI=ON");
#endif
}
GlfwContext::~GlfwContext() {
    if (std::this_thread::get_id() != main_thread) std::terminate();
}
std::vector<WindowEvent> GlfwContext::poll() {
    require_main_thread();
#if defined(BIKE_BUILD_GUI) && BIKE_BUILD_GUI
    glfwPollEvents();
    if (impl_->callback_failure) std::rethrow_exception(impl_->callback_failure);
    auto events = std::move(impl_->events); impl_->events.clear(); return events;
#else
    return {};
#endif
}
bool GlfwContext::open() const {
    require_main_thread();
#if defined(BIKE_BUILD_GUI) && BIKE_BUILD_GUI
    return glfwWindowShouldClose(impl_->window) == GLFW_FALSE;
#else
    return false;
#endif
}
std::pair<int, int> GlfwContext::framebuffer_size() const {
    require_main_thread();
    int width = 0, height = 0;  // NOLINT(misc-const-correctness) written through pointers by glfwGetFramebufferSize under BIKE_BUILD_GUI
#if defined(BIKE_BUILD_GUI) && BIKE_BUILD_GUI
    glfwMakeContextCurrent(impl_->window);
    glfwGetFramebufferSize(impl_->window, &width, &height);
#endif
    return {width, height};
}
void GlfwContext::swap() {
    require_main_thread();
#if defined(BIKE_BUILD_GUI) && BIKE_BUILD_GUI
    glfwSwapBuffers(impl_->window);
#endif
}
void GlfwContext::wait(double seconds) {
    require_main_thread();
    if (!std::isfinite(seconds) || seconds < 0. || seconds > .1) throw std::invalid_argument("viewer idle wait must be in [0, .1]");
#if defined(BIKE_BUILD_GUI) && BIKE_BUILD_GUI
    if (seconds > 0.) glfwWaitEventsTimeout(seconds);
#endif
}
void GlfwContext::request_close() {
    require_main_thread();
#if defined(BIKE_BUILD_GUI) && BIKE_BUILD_GUI
    glfwSetWindowShouldClose(impl_->window, GLFW_TRUE);
#endif
}
} // namespace bike::viewer
