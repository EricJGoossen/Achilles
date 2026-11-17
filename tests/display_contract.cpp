// The contract between a window and what it holds (display/contract/): what
// a Frame holds and lets go of, what an Input answers, and what counts as a
// display or a controller at all -- plain data and concepts, no screen.
#include <gtest/gtest.h>

#include <cstddef>
#include <string_view>
#include <type_traits>

#include "display/contract/controller.hpp"
#include "display/contract/display.hpp"
#include "display/contract/frame.hpp"
#include "display/contract/input.hpp"
#include "display/views/camera/camera_view.hpp"
#include "display/views/camera/depth_view.hpp"
#include "display/views/lidar_view.hpp"
#include "display/views/reading_view.hpp"
#include "domain/sensing/camera.hpp"
#include "domain/sensing/encoder.hpp"
#include "interface/attachment_handle.hpp"

using achilles::display::Button;
using achilles::display::ControllerLike;
using achilles::display::DisplayLike;
using achilles::display::Frame;
using achilles::display::IDisplay;
using achilles::display::Input;
using achilles::display::Key;
using achilles::display::kKeyCount;
using achilles::display::Pointer;
using achilles::domain::sensing::Camera;
using achilles::domain::sensing::Encoder;
using achilles::interface::AttachmentHandle;

namespace {

// Would-be displays, each missing one part of the contract.

// Draws, says what it is and how big: a display.
class Shows final : public IDisplay {
 public:
  static constexpr std::string_view kName = "shows";
  static constexpr float kWidth = 1.0F;
  static constexpr float kHeight = 1.0F;
  void Draw(Frame* /*frame*/) const override {}
};

// Has no name.
class Nameless final : public IDisplay {
 public:
  static constexpr float kWidth = 1.0F;
  static constexpr float kHeight = 1.0F;
  void Draw(Frame* /*frame*/) const override {}
};

// Says all a display says, but draws through no IDisplay.
struct NotADisplay {
  static constexpr std::string_view kName = "not a display";
  static constexpr float kWidth = 1.0F;
  static constexpr float kHeight = 1.0F;
  void Draw(Frame* /*frame*/) const {}
};

static_assert(DisplayLike<Shows>);
static_assert(!DisplayLike<Nameless>);
static_assert(!DisplayLike<NotADisplay>);

// Every view is a display, and is made with exactly the handles it shows:
// a camera's view can't be made with an encoder's handle.
static_assert(DisplayLike<achilles::display::CameraView>);
static_assert(DisplayLike<achilles::display::CameraAxesView>);
static_assert(DisplayLike<achilles::display::DepthView>);
static_assert(DisplayLike<achilles::display::LidarView>);
static_assert(DisplayLike<achilles::display::ReadingView<Encoder, 3>>);
static_assert(!std::is_constructible_v<
              achilles::display::CameraView,
              AttachmentHandle<Encoder>>);
static_assert(!std::is_constructible_v<
              achilles::display::ReadingView<Encoder>,
              AttachmentHandle<Camera>>);
// None is made without its handles.
static_assert(!std::is_default_constructible_v<achilles::display::CameraView>);

// A display isn't a controller.
static_assert(!ControllerLike<Shows>);

}  // namespace

TEST(Frame, ClearLetsGoOfEverythingButItsRoom) {
  Frame frame;
  frame.aspect = 1.5F;
  frame.background = {1.0F, 1.0F, 1.0F};
  frame.text.emplace_back("hello");
  frame.images.push_back({});
  frame.points.push_back({});
  frame.lines.push_back({});
  frame.traces.push_back({.label = "x", .value = 1.0F});
  const std::size_t room = frame.traces.capacity();

  frame.Clear();
  EXPECT_EQ(frame.aspect, 0.0F);
  EXPECT_FLOAT_EQ(frame.background.r, Frame{}.background.r);
  EXPECT_TRUE(frame.text.empty());
  EXPECT_TRUE(frame.images.empty());
  EXPECT_TRUE(frame.points.empty());
  EXPECT_TRUE(frame.lines.empty());
  EXPECT_TRUE(frame.traces.empty());
  // Kept for the next frame, so drawing doesn't allocate again.
  EXPECT_EQ(frame.traces.capacity(), room);
}

TEST(Fixed, ANumberWithTheDecimalsAskedFor) {
  EXPECT_EQ(achilles::display::Fixed(1.0F), "1.00");
  EXPECT_EQ(achilles::display::Fixed(-0.125F, 3), "-0.125");
  EXPECT_EQ(achilles::display::Fixed(2.75F, 0), "3");
  EXPECT_EQ(achilles::display::Fixed(1234.5F, 1), "1234.5");
}

TEST(Input, NothingUntilAWindowSaysSo) {
  const Input input;
  EXPECT_EQ(input.seconds, 0.0F);
  EXPECT_FALSE(input.focused);
  EXPECT_FALSE(input.pointer.over);
  EXPECT_EQ(input.pointer.scroll, 0.0F);
}

TEST(Input, KeysByName) {
  Input input;
  for (std::size_t i = 0; i < kKeyCount; ++i) {
    EXPECT_FALSE(input.Held(static_cast<Key>(i))) << "key " << i;
    EXPECT_FALSE(input.Pressed(static_cast<Key>(i))) << "key " << i;
  }
  input.keys_held[static_cast<std::size_t>(Key::kW)] = true;
  input.keys_pressed[static_cast<std::size_t>(Key::kEscape)] = true;
  EXPECT_TRUE(input.Held(Key::kW));
  EXPECT_FALSE(input.Pressed(Key::kW));
  EXPECT_TRUE(input.Pressed(Key::kEscape));
  EXPECT_FALSE(input.Held(Key::kEscape));
}

TEST(Input, ThePointersButtonsByName) {
  Pointer pointer;
  EXPECT_FALSE(pointer.over);
  EXPECT_FALSE(pointer.Held(Button::kLeft));
  pointer.held[static_cast<std::size_t>(Button::kRight)] = true;
  pointer.pressed[static_cast<std::size_t>(Button::kMiddle)] = true;
  pointer.released[static_cast<std::size_t>(Button::kLeft)] = true;
  EXPECT_TRUE(pointer.Held(Button::kRight));
  EXPECT_FALSE(pointer.Held(Button::kLeft));
  EXPECT_TRUE(pointer.Pressed(Button::kMiddle));
  EXPECT_TRUE(pointer.Released(Button::kLeft));
  EXPECT_FALSE(pointer.Released(Button::kRight));
}
