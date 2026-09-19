#include "render/window.hpp"

#include <GLFW/glfw3.h>

#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

#include "render/gl.hpp"

namespace achilles::render {

namespace {

int g_window_count = 0;
}  // namespace

void Window::ScrollCallback(
    GLFWwindow* handle, double /*xoffset*/, double yoffset
) {
  auto* self = static_cast<Window*>(glfwGetWindowUserPointer(handle));
  if (self != nullptr) {
    self->pending_scroll_dy_ += yoffset;
  }
}

Window::Window(int width, int height, std::string_view title) {
  if (g_window_count == 0 && glfwInit() == GLFW_FALSE) {
    throw std::runtime_error("achilles::render: glfwInit failed");
  }
  ++g_window_count;

  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

  handle_ = glfwCreateWindow(
      width, height, std::string(title).c_str(), nullptr, nullptr
  );
  if (handle_ == nullptr) {
    --g_window_count;
    if (g_window_count == 0) {
      glfwTerminate();
    }
    throw std::runtime_error("achilles::render: glfwCreateWindow failed");
  }

  glfwSetWindowUserPointer(handle_, this);
  glfwSetScrollCallback(handle_, &Window::ScrollCallback);
  glfwMakeContextCurrent(handle_);
  glfwSwapInterval(1);
  gl::LoadGLFunctions();
}

Window::~Window() {
  if (handle_ != nullptr) {
    glfwDestroyWindow(handle_);
    --g_window_count;
    if (g_window_count == 0) {
      glfwTerminate();
    }
  }
}

Window::Window(Window&& other) noexcept
    : handle_(std::exchange(other.handle_, nullptr)),
      last_cursor_x_(other.last_cursor_x_),
      last_cursor_y_(other.last_cursor_y_),
      cursor_dx_(other.cursor_dx_),
      cursor_dy_(other.cursor_dy_),
      pending_scroll_dy_(other.pending_scroll_dy_),
      scroll_dy_(other.scroll_dy_),
      has_last_cursor_(other.has_last_cursor_) {
  if (handle_ != nullptr) {
    glfwSetWindowUserPointer(handle_, this);
  }
}

Window& Window::operator=(Window&& other) noexcept {
  if (this != &other) {
    if (handle_ != nullptr) {
      glfwDestroyWindow(handle_);
      --g_window_count;
      if (g_window_count == 0) {
        glfwTerminate();
      }
    }
    handle_ = std::exchange(other.handle_, nullptr);
    last_cursor_x_ = other.last_cursor_x_;
    last_cursor_y_ = other.last_cursor_y_;
    cursor_dx_ = other.cursor_dx_;
    cursor_dy_ = other.cursor_dy_;
    pending_scroll_dy_ = other.pending_scroll_dy_;
    scroll_dy_ = other.scroll_dy_;
    has_last_cursor_ = other.has_last_cursor_;
    if (handle_ != nullptr) {
      glfwSetWindowUserPointer(handle_, this);
    }
  }
  return *this;
}

bool Window::ShouldClose() const { return glfwWindowShouldClose(handle_) != 0; }

void Window::PollEvents() {
  cursor_dx_ = 0.0;
  cursor_dy_ = 0.0;
  pending_scroll_dy_ = 0.0;

  glfwPollEvents();

  double x = 0.0;
  double y = 0.0;
  glfwGetCursorPos(handle_, &x, &y);
  if (has_last_cursor_ && IsLeftMouseDown()) {
    cursor_dx_ = x - last_cursor_x_;
    cursor_dy_ = y - last_cursor_y_;
  }
  last_cursor_x_ = x;
  last_cursor_y_ = y;
  has_last_cursor_ = true;

  scroll_dy_ = pending_scroll_dy_;
}

void Window::SwapBuffers() { glfwSwapBuffers(handle_); }

std::pair<int, int> Window::FramebufferSize() const {
  int width = 0;
  int height = 0;
  glfwGetFramebufferSize(handle_, &width, &height);
  return {width, height};
}

float Window::Aspect() const {
  auto [width, height] = FramebufferSize();
  return height > 0 ? static_cast<float>(width) / static_cast<float>(height)
                    : 1.0F;
}

std::pair<float, float> Window::CursorDelta() const {
  return {static_cast<float>(cursor_dx_), static_cast<float>(cursor_dy_)};
}

float Window::ScrollDelta() const { return static_cast<float>(scroll_dy_); }

bool Window::IsLeftMouseDown() const {
  return glfwGetMouseButton(handle_, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
}

}  // namespace achilles::render
