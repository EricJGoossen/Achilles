#include "display/window/screen.hpp"

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>

namespace achilles::display {

namespace {

// One screen per process: GLFW and ImGui's backends are process-wide.
bool g_screen_open = false;
std::string g_last_glfw_error;

void RecordGlfwError(int /*code*/, const char* description) {
  g_last_glfw_error = description;
}

}  // namespace

struct Screen::Impl {
  GLFWwindow* host = nullptr;
  ImGuiContext* gui = nullptr;
};

Screen::Screen(std::string_view title) : impl_(std::make_unique<Impl>()) {
  if (g_screen_open) {
    throw std::runtime_error("Screen: one is already open in this process.");
  }
  glfwSetErrorCallback(RecordGlfwError);
  if (glfwInit() == GLFW_FALSE) {
    throw std::runtime_error(
        "Screen: GLFW couldn't start (" + g_last_glfw_error +
        ") -- is there a display server ($DISPLAY)?"
    );
  }
  // EGL, as the camera sensors' own context is, so the two share one
  // library and one thread without stepping on each other. Every window the
  // GUI makes later shares this context, and so its textures.
  glfwWindowHint(GLFW_CONTEXT_CREATION_API, GLFW_EGL_CONTEXT_API);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
  // The host is never shown: display windows are each their own.
  glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
  const std::string name(title);
  impl_->host = glfwCreateWindow(64, 64, name.c_str(), nullptr, nullptr);
  if (impl_->host == nullptr) {
    const std::string why = g_last_glfw_error;
    glfwTerminate();
    throw std::runtime_error("Screen: couldn't make a window (" + why + ").");
  }
  // The context hints above stay in force: the GUI makes each display's
  // window with them, sharing this context, which only another EGL context
  // can. Only visibility goes back to the default.
  glfwWindowHint(GLFW_VISIBLE, GLFW_TRUE);
  glfwMakeContextCurrent(impl_->host);
  glfwSwapInterval(1);

  IMGUI_CHECKVERSION();
  impl_->gui = ImGui::CreateContext();
  ImGuiIO& io = ImGui::GetIO();
  // Window layouts aren't saved between runs: no imgui.ini in the working
  // directory.
  io.IniFilename = nullptr;
  // Every GUI window its own desktop window, with the desktop's own title
  // bar and close button, never merged into another.
  io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;
  io.ConfigViewportsNoAutoMerge = true;
  io.ConfigViewportsNoDecoration = false;
  io.ConfigViewportsNoTaskBarIcon = false;
  ImGui::StyleColorsDark();
  if (!ImGui_ImplGlfw_InitForOpenGL(impl_->host, true) ||
      !ImGui_ImplOpenGL3_Init("#version 330 core")) {
    ImGui::DestroyContext(impl_->gui);
    glfwDestroyWindow(impl_->host);
    glfwTerminate();
    throw std::runtime_error("Screen: the GUI couldn't start on this context.");
  }
  g_screen_open = true;
}

Screen::~Screen() {
  MakeCurrent();
  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext(impl_->gui);
  glfwDestroyWindow(impl_->host);
  glfwTerminate();
  g_screen_open = false;
}

void Screen::BeginFrame() const {
  glfwPollEvents();
  // A camera may have rendered on its own context since the last frame.
  MakeCurrent();
  ImGui_ImplOpenGL3_NewFrame();
  ImGui_ImplGlfw_NewFrame();
  ImGui::NewFrame();
}

void Screen::EndFrame() const {
  ImGui::Render();
  // The host draws nothing anyone sees; each display window is drawn, and
  // shown, in its own.
  ImGui::UpdatePlatformWindows();
  ImGui::RenderPlatformWindowsDefault();
  MakeCurrent();
}

void Screen::MakeCurrent() const { glfwMakeContextCurrent(impl_->host); }

}  // namespace achilles::display
