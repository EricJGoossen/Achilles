#include "display/window/window.hpp"

#include <GL/gl.h>
#include <imgui.h>

#include <algorithm>
#include <array>
#include <cfloat>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>
#include <vector>

#include "display/contract/frame.hpp"
#include "display/contract/input.hpp"

namespace achilles::display {

static_assert(std::is_same_v<GLuint, std::uint32_t>);

namespace {

// ImGui's key for each display::Key, in the Key's order.
constexpr std::array<ImGuiKey, kKeyCount> kImGuiKeys = {
    ImGuiKey_W,
    ImGuiKey_A,
    ImGuiKey_S,
    ImGuiKey_D,
    ImGuiKey_Q,
    ImGuiKey_E,
    ImGuiKey_R,
    ImGuiKey_F,
    ImGuiKey_UpArrow,
    ImGuiKey_DownArrow,
    ImGuiKey_LeftArrow,
    ImGuiKey_RightArrow,
    ImGuiKey_Space,
    ImGuiMod_Shift,
    ImGuiMod_Ctrl,
    ImGuiKey_Escape,
};
static_assert(static_cast<std::size_t>(Key::kEscape) + 1 == kKeyCount);
static_assert(kButtonCount <= ImGuiMouseButton_COUNT);

// Tags every window's GUI label, so windows sharing a title stay apart.
std::uint64_t g_next_window_tag = 1;

// Where the next window first opens on the desktop: windows are laid out
// in rows across its work area, left to right, a new row starting below the
// tallest of the last once one would run off it, so a new window never
// covers the others. Back to the top only once a row would start below the
// work area's bottom; one that just overhangs it stays.
constexpr float kLayoutMargin = 24.0F;
// Room for the desktop's own title bar and border around each window.
constexpr float kLayoutGap = 40.0F;
// How much of a window has to be on the desktop for its row to start there.
constexpr float kLayoutRowMinimum = 120.0F;
struct Layout {
  float x = 0.0F;
  float y = 0.0F;
  float row_height = 0.0F;
  bool started = false;
};
Layout g_layout;

// An area of the desktop, by its corners.
struct Area {
  ImVec2 min;
  ImVec2 max;
};

Area Spanning(const ImVec2& at, const ImVec2& size) {
  return {.min = at, .max = ImVec2(at.x + size.x, at.y + size.y)};
}

// The desktop's work area: its first monitor's, or the GUI's own viewport
// before any monitor is known.
Area WorkArea() {
  const ImGuiPlatformIO& platform = ImGui::GetPlatformIO();
  if (!platform.Monitors.empty()) {
    const ImGuiPlatformMonitor& monitor = platform.Monitors.front();
    return Spanning(monitor.WorkPos, monitor.WorkSize);
  }
  const ImGuiViewport* viewport = ImGui::GetMainViewport();
  return Spanning(viewport->WorkPos, viewport->WorkSize);
}

// Where a width x height window goes, advancing the layout past it.
ImVec2 Place(float width, float height) {
  const Area area = WorkArea();
  const float left = area.min.x + kLayoutMargin;
  const float top = area.min.y + kLayoutMargin;
  if (!g_layout.started) {
    g_layout = Layout{.x = left, .y = top, .row_height = 0.0F, .started = true};
  }
  if (g_layout.x > left && g_layout.x + width > area.max.x) {
    g_layout.x = left;
    g_layout.y += g_layout.row_height + kLayoutGap;
    g_layout.row_height = 0.0F;
  }
  if (g_layout.y > top && g_layout.y + kLayoutRowMinimum > area.max.y) {
    g_layout.x = left;
    g_layout.y = top;
    g_layout.row_height = 0.0F;
  }
  const ImVec2 at(g_layout.x, g_layout.y);
  g_layout.x += width + kLayoutGap;
  g_layout.row_height = std::max(g_layout.row_height, height);
  return at;
}

ImU32 ToImGui(const Color& color) {
  return ImGui::ColorConvertFloat4ToU32(ImVec4(color.r, color.g, color.b, 1.0F)
  );
}

// A value image as gray RGB: white at 0, black at `scale` and beyond, and
// black where a value isn't finite.
void ToGray(const Image& image, std::vector<std::uint8_t>* out) {
  const std::size_t pixels = std::size_t{image.width} * image.height;
  out->resize(3 * pixels);
  for (std::size_t i = 0; i < pixels; ++i) {
    const float value = image.values[i];
    const float shade = std::isfinite(value)
                            ? 1.0F - std::clamp(value / image.scale, 0.0F, 1.0F)
                            : 0.0F;
    const auto byte = static_cast<std::uint8_t>(std::lround(shade * 255.0F));
    (*out)[(3 * i) + 0] = byte;
    (*out)[(3 * i) + 1] = byte;
    (*out)[(3 * i) + 2] = byte;
  }
}

}  // namespace

Window::Window(std::string_view title, float width, float height)
    : title_(title),
      label_(title_ + "##display" + std::to_string(g_next_window_tag++)),
      width_(width),
      height_(height) {}

Window::~Window() { FreeTextures(); }

Window::Window(Window&& other) noexcept
    : title_(std::move(other.title_)),
      label_(std::move(other.label_)),
      width_(other.width_),
      height_(other.height_),
      placed_(other.placed_),
      open_(other.open_),
      values_(std::move(other.values_)),
      pointer_(other.pointer_),
      histories_(std::move(other.histories_)),
      textures_(std::move(other.textures_)),
      gray_(std::move(other.gray_)) {
  other.textures_.clear();
}

Window& Window::operator=(Window&& other) noexcept {
  if (this != &other) {
    FreeTextures();
    title_ = std::move(other.title_);
    label_ = std::move(other.label_);
    width_ = other.width_;
    height_ = other.height_;
    placed_ = other.placed_;
    open_ = other.open_;
    values_ = std::move(other.values_);
    pointer_ = other.pointer_;
    histories_ = std::move(other.histories_);
    textures_ = std::move(other.textures_);
    gray_ = std::move(other.gray_);
    other.textures_.clear();
  }
  return *this;
}

void Window::FreeTextures() {
  if (!textures_.empty()) {
    glDeleteTextures(static_cast<GLsizei>(textures_.size()), textures_.data());
    textures_.clear();
  }
}

void Window::Upload(std::size_t index, const Image& image) {
  while (textures_.size() <= index) {
    GLuint texture = 0;
    glGenTextures(1, &texture);
    glBindTexture(GL_TEXTURE_2D, texture);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    textures_.push_back(texture);
  }
  const std::uint8_t* pixels = image.rgb.data();
  if (image.rgb.empty()) {
    ToGray(image, &gray_);
    pixels = gray_.data();
  }
  glBindTexture(GL_TEXTURE_2D, textures_[index]);
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
  glTexImage2D(
      GL_TEXTURE_2D,
      0,
      GL_RGB8,
      static_cast<GLsizei>(image.width),
      static_cast<GLsizei>(image.height),
      0,
      GL_RGB,
      GL_UNSIGNED_BYTE,
      pixels
  );
}

Input Window::Present(const Frame& frame) {
  values_.resize(frame.sliders.size(), 0.0F);
  pointer_ = Pointer{};
  if (!placed_) {
    ImGui::SetNextWindowPos(Place(width_, height_));
    ImGui::SetNextWindowSize(ImVec2(width_, height_));
    placed_ = true;
  }
  // Its own desktop window, whose title bar and close button are the
  // desktop's: none of the GUI's own inside it.
  constexpr ImGuiWindowFlags kFlags = ImGuiWindowFlags_NoTitleBar |
                                      ImGuiWindowFlags_NoCollapse |
                                      ImGuiWindowFlags_NoSavedSettings;
  bool open = open_;
  const bool shown = ImGui::Begin(label_.c_str(), &open, kFlags);
  open_ = open;
  Input input;
  input.focused =
      shown && ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows);
  if (input.focused) {
    for (std::size_t i = 0; i < kKeyCount; ++i) {
      input.keys_held[i] = ImGui::IsKeyDown(kImGuiKeys[i]);
      input.keys_pressed[i] = ImGui::IsKeyPressed(kImGuiKeys[i], false);
    }
  }
  if (shown) {
    for (const std::string& line : frame.text) {
      ImGui::TextUnformatted(line.c_str());
    }
    if (frame.aspect > 0.0F) {
      DrawArea(frame);
    }
    DrawTraces(frame);
    DrawControls(frame);
  } else {
    // Minimized: nothing is held, so springing controls let go.
    for (std::size_t i = 0; i < frame.sliders.size(); ++i) {
      if (frame.sliders[i].spring) {
        values_[i] = 0.0F;
      }
    }
  }
  ImGui::End();
  input.values = values_;
  input.pointer = pointer_;
  return input;
}

void Window::DrawArea(const Frame& frame) {
  // Whatever room the controls leave, letterboxed to the frame's aspect and
  // centered.
  const float controls = static_cast<float>(frame.sliders.size()) *
                         ImGui::GetFrameHeightWithSpacing();
  const ImVec2 room = ImGui::GetContentRegionAvail();
  const float room_width = std::max(room.x, 1.0F);
  const float room_height = std::max(room.y - controls, 1.0F);
  float width = room_width;
  float height = room_width / frame.aspect;
  if (height > room_height) {
    height = room_height;
    width = room_height * frame.aspect;
  }
  const ImVec2 cursor = ImGui::GetCursorScreenPos();
  const ImVec2 origin(cursor.x + (0.5F * (room_width - width)), cursor.y);
  const auto at = [&](float x, float y) {
    return ImVec2(origin.x + (x * width), origin.y + (y * height));
  };

  ImDrawList* draw = ImGui::GetWindowDrawList();
  draw->AddRectFilled(
      at(0.0F, 0.0F), at(1.0F, 1.0F), ToImGui(frame.background)
  );
  for (std::size_t i = 0; i < frame.images.size(); ++i) {
    const Image& image = frame.images[i];
    if (image.width == 0 || image.height == 0) {
      continue;
    }
    Upload(i, image);
    draw->AddImage(
        static_cast<ImTextureID>(textures_[i]),
        at(image.where.x, image.where.y),
        at(image.where.x + image.where.width,
           image.where.y + image.where.height)
    );
  }
  for (const Lines& lines : frame.lines) {
    const ImU32 color = ToImGui(lines.color);
    for (std::size_t i = 0; i + 3 < lines.xy.size(); i += 4) {
      draw->AddLine(
          at(lines.xy[i], lines.xy[i + 1]),
          at(lines.xy[i + 2], lines.xy[i + 3]),
          color,
          1.5F
      );
    }
  }
  for (const Points& points : frame.points) {
    const ImU32 color = ToImGui(points.color);
    const float half = 0.5F * points.size;
    for (std::size_t i = 0; i + 1 < points.xy.size(); i += 2) {
      const ImVec2 p = at(points.xy[i], points.xy[i + 1]);
      draw->AddRectFilled(
          ImVec2(p.x - half, p.y - half), ImVec2(p.x + half, p.y + half), color
      );
    }
  }
  // Claims the area, so the controls go below it -- as a button for every
  // mouse button, so a drag on it is the display's, never a move of the
  // window.
  ImGui::InvisibleButton(
      "area",
      ImVec2(room_width, height),
      ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight |
          ImGuiButtonFlags_MouseButtonMiddle
  );
  TrackPointer(origin.x, origin.y, width, height);
}

void Window::TrackPointer(float left, float top, float width, float height) {
  // Over the area, or dragging from it: the area's until let go.
  if (!ImGui::IsItemHovered() && !ImGui::IsItemActive()) {
    return;
  }
  const ImGuiIO& io = ImGui::GetIO();
  pointer_.over = true;
  pointer_.x = (io.MousePos.x - left) / width;
  pointer_.y = (io.MousePos.y - top) / height;
  pointer_.dx = io.MouseDelta.x / width;
  pointer_.dy = io.MouseDelta.y / height;
  pointer_.scroll = io.MouseWheel;
  for (std::size_t i = 0; i < kButtonCount; ++i) {
    const auto button = static_cast<ImGuiMouseButton>(i);
    pointer_.held[i] = ImGui::IsMouseDown(button);
    pointer_.pressed[i] = ImGui::IsMouseClicked(button);
    pointer_.released[i] = ImGui::IsMouseReleased(button);
  }
}

void Window::DrawTraces(const Frame& frame) {
  histories_.resize(frame.traces.size());
  for (std::size_t i = 0; i < frame.traces.size(); ++i) {
    Record(frame.traces[i], &histories_[i]);
  }
  for (std::size_t i = 0; i < frame.traces.size(); ++i) {
    const Trace& trace = frame.traces[i];
    if (trace.plot.empty()) {
      DrawOwnPlot(trace, histories_[i], i);
      continue;
    }
    // A shared plot is drawn once, at its first trace, with all of its own.
    bool first = true;
    for (std::size_t j = 0; j < i && first; ++j) {
      first = frame.traces[j].plot != trace.plot;
    }
    if (first) {
      std::vector<std::size_t> members;
      for (std::size_t j = i; j < frame.traces.size(); ++j) {
        if (frame.traces[j].plot == trace.plot) {
          members.push_back(j);
        }
      }
      DrawSharedPlot(frame, members);
    }
  }
}

void Window::Record(const Trace& trace, History* history) {
  if (trace.time == history->last_time) {
    return;
  }
  if (history->values.size() < kTraceSamples) {
    history->values.push_back(trace.value);
  } else {
    history->values[history->next] = trace.value;
    history->next = (history->next + 1) % kTraceSamples;
  }
  history->last_time = trace.time;
}

void Window::DrawOwnPlot(
    const Trace& trace, const History& history, std::size_t index
) {
  constexpr float kPlotHeight = 44.0F;
  const std::string overlay = Fixed(trace.value, 3);
  ImGui::PushID(static_cast<int>(index));
  ImGui::PlotLines(
      trace.label.c_str(),
      history.values.data(),
      static_cast<int>(history.values.size()),
      static_cast<int>(history.next),
      overlay.c_str(),
      FLT_MAX,
      FLT_MAX,
      ImVec2(0.0F, kPlotHeight)
  );
  ImGui::PopID();
}

void Window::DrawSharedPlot(
    const Frame& frame, const std::vector<std::size_t>& members
) {
  constexpr float kPlotHeight = 90.0F;
  // Distinct, readable on the dark background; repeats past eight lines.
  constexpr std::array<Color, 8> kPalette = {{
      {0.35F, 0.60F, 0.95F},
      {0.95F, 0.55F, 0.25F},
      {0.35F, 0.80F, 0.50F},
      {0.95F, 0.80F, 0.25F},
      {0.90F, 0.45F, 0.70F},
      {0.65F, 0.50F, 0.95F},
      {0.40F, 0.85F, 0.90F},
      {0.85F, 0.85F, 0.85F},
  }};
  ImGui::TextUnformatted(frame.traces[members.front()].plot.c_str());

  // One scale for every line, so they compare.
  float low = FLT_MAX;
  float high = -FLT_MAX;
  for (std::size_t member : members) {
    for (float value : histories_[member].values) {
      low = std::min(low, value);
      high = std::max(high, value);
    }
  }
  if (low > high) {
    low = 0.0F;
    high = 1.0F;
  }
  if (high - low < 1e-6F) {
    low -= 0.5F;
    high += 0.5F;
  }

  const ImVec2 at = ImGui::GetCursorScreenPos();
  const float width = std::max(ImGui::GetContentRegionAvail().x, 1.0F);
  ImDrawList* draw = ImGui::GetWindowDrawList();
  draw->AddRectFilled(
      at,
      ImVec2(at.x + width, at.y + kPlotHeight),
      ImGui::GetColorU32(ImGuiCol_FrameBg)
  );
  const auto y_of = [&](float value) {
    return at.y + (kPlotHeight * (high - value) / (high - low));
  };
  if (low < 0.0F && high > 0.0F) {
    draw->AddLine(
        ImVec2(at.x, y_of(0.0F)),
        ImVec2(at.x + width, y_of(0.0F)),
        ImGui::GetColorU32(ImGuiCol_Border)
    );
  }
  for (std::size_t k = 0; k < members.size(); ++k) {
    const History& history = histories_[members[k]];
    const std::size_t count = history.values.size();
    const ImU32 color = ToImGui(kPalette[k % kPalette.size()]);
    const float step =
        width / static_cast<float>(std::max<std::size_t>(kTraceSamples - 1, 1));
    for (std::size_t j = 1; j < count; ++j) {
      const float v0 = history.values[(history.next + j - 1) % count];
      const float v1 = history.values[(history.next + j) % count];
      draw->AddLine(
          ImVec2(at.x + (step * static_cast<float>(j - 1)), y_of(v0)),
          ImVec2(at.x + (step * static_cast<float>(j)), y_of(v1)),
          color,
          1.5F
      );
    }
  }
  const std::string top = Fixed(high, 3);
  const std::string bottom = Fixed(low, 3);
  const ImU32 scale = ImGui::GetColorU32(ImGuiCol_TextDisabled);
  draw->AddText(ImVec2(at.x + 2.0F, at.y), scale, top.c_str());
  draw->AddText(
      ImVec2(at.x + 2.0F, at.y + kPlotHeight - ImGui::GetTextLineHeight()),
      scale,
      bottom.c_str()
  );
  ImGui::Dummy(ImVec2(width, kPlotHeight));

  // The legend: each line's name and latest value, in its color.
  for (std::size_t k = 0; k < members.size(); ++k) {
    const Trace& trace = frame.traces[members[k]];
    const Color& color = kPalette[k % kPalette.size()];
    const std::string entry = trace.label + " " + Fixed(trace.value, 3);
    if (k > 0) {
      ImGui::SameLine();
    }
    ImGui::PushStyleColor(
        ImGuiCol_Text, ImVec4(color.r, color.g, color.b, 1.0F)
    );
    ImGui::TextUnformatted(entry.c_str());
    ImGui::PopStyleColor();
  }
}

void Window::DrawControls(const Frame& frame) {
  for (std::size_t i = 0; i < frame.sliders.size(); ++i) {
    const Slider& slider = frame.sliders[i];
    const std::string label(slider.label);
    ImGui::PushID(static_cast<int>(i));
    ImGui::SliderFloat(
        label.c_str(), &values_[i], slider.min, slider.max, "%.2f"
    );
    if (slider.spring && !ImGui::IsItemActive()) {
      values_[i] = 0.0F;
    }
    ImGui::PopID();
  }
}

}  // namespace achilles::display
