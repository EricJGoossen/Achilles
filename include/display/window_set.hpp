#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string_view>
#include <utility>
#include <vector>

#include "display/contract/controller.hpp"
#include "display/contract/display.hpp"
#include "display/contract/frame.hpp"
#include "display/contract/input.hpp"
#include "display/window/window.hpp"

namespace achilles::display {

// One window in a WindowSet, as Add() returns it: never reused, so an id of
// a window since removed finds nothing rather than another window.
struct WindowId {
  std::uint64_t value = 0;

  friend bool operator==(const WindowId&, const WindowId&) = default;
};

// Every window up: each with the display it draws and the controllers riding
// on it (controller.hpp), in the order they were opened. What draws the
// displays, hands each window's input to its controllers, and has them write
// before every Step(). Any display or controller can be added -- nothing
// lists the kinds -- each kept behind one virtual call a frame, which drawing
// its window dwarfs.
//
// Windows come and go at any point between two frames: Add() opens one for a
// display, Find() reaches its display to change its settings, Swap() puts
// another display in it, Remove() closes it; Control() puts a controller on
// it, and Release() takes them all off. Nothing here touches the sim but the
// controllers' writes, and the handles stay valid as long as the sim keeps
// them (until it's next Init()ed).
//
// A window that has drawn holds textures in the screen's GL context, so the
// context must be current when one is dropped: in Draw(), and around a
// Remove() or the set's destruction (Renderer makes it so).
class WindowSet {
 public:
  // Opens a window titled `title` for `display`.
  template <DisplayLike D>
  WindowId Add(D display, std::string_view title) {
    const WindowId id{next_id_++};
    entries_.push_back(
        {.id = id,
         .window = Window(title, D::kWidth, D::kHeight),
         .display = std::make_unique<D>(std::move(display)),
         .controllers = {}}
    );
    return id;
  }

  // The display in window `id`, to change its settings from the next frame
  // on -- or null if it's gone, or its display isn't a D. Valid until the
  // next Add(), Swap(), Remove() or Draw().
  template <DisplayLike D>
  D* Find(WindowId id) {
    Entry* entry = At(id);
    return entry == nullptr ? nullptr : dynamic_cast<D*>(entry->display.get());
  }

  // Puts `display` in window `id`, in place of whatever it showed: the
  // window, its title and its controllers stay. Throws std::invalid_argument
  // if `id` is gone.
  template <DisplayLike D>
  void Swap(WindowId id, D display) {
    Entry* entry = At(id);
    if (entry == nullptr) {
      throw std::invalid_argument("WindowSet::Swap: no such window.");
    }
    entry->display = std::make_unique<D>(std::move(display));
  }

  // Puts `controller` on window `id`, steered by its input from the next
  // Draw() on, beside any already there; false if `id` is gone.
  template <ControllerLike C>
  bool Control(WindowId id, C controller) {
    Entry* entry = At(id);
    if (entry == nullptr) {
      return false;
    }
    entry->controllers.push_back(std::make_unique<C>(std::move(controller)));
    return true;
  }

  // The first C on window `id`, to change its settings from the next frame
  // on -- or null if there's none, or `id` is gone. Valid until the next
  // Control(), Release(), Remove() or Draw().
  template <ControllerLike C>
  C* FindController(WindowId id) {
    Entry* entry = At(id);
    if (entry == nullptr) {
      return nullptr;
    }
    for (const std::unique_ptr<IController>& controller : entry->controllers) {
      if (auto* found = dynamic_cast<C*>(controller.get()); found != nullptr) {
        return found;
      }
    }
    return nullptr;
  }

  // Takes every controller off window `id`: it writes nothing from then on.
  // False if `id` is gone.
  bool Release(WindowId id) {
    Entry* entry = At(id);
    if (entry == nullptr) {
      return false;
    }
    entry->controllers.clear();
    return true;
  }

  // Closes window `id`, its display and controllers with it; false if it's
  // already gone.
  bool Remove(WindowId id) {
    return std::erase_if(entries_, [&](const Entry& entry) {
             return entry.id == id;
           }) > 0;
  }

  bool Contains(WindowId id) const {
    return std::ranges::any_of(entries_, [&](const Entry& entry) {
      return entry.id == id;
    });
  }

  std::size_t Size() const { return entries_.size(); }

  // Window `id`'s title, or empty if it's gone.
  std::string_view Title(WindowId id) const {
    const auto it = std::ranges::find_if(entries_, [&](const Entry& entry) {
      return entry.id == id;
    });
    return it == entries_.end() ? std::string_view{} : it->window.Title();
  }

  // Draws every window's display and hands what the window took in to its
  // controllers (Feed), dropping first the windows closed since. Runs between
  // the screen's BeginFrame() and EndFrame(). Returns how many are up.
  std::size_t Draw() {
    std::erase_if(entries_, [](const Entry& entry) {
      return !entry.window.IsOpen();
    });
    for (Entry& entry : entries_) {
      frame_.Clear();
      entry.display->Draw(&frame_);
      const Input input = entry.window.Present(frame_);
      // A window closed this frame steers nothing more.
      FeedControllers(entry, entry.window.IsOpen() ? input : Input{});
    }
    return entries_.size();
  }

  // Hands `input` to window `id`'s controllers, as if the window had taken it
  // in: what they write from the next ApplyWrites() on. Draw() does this for
  // every window each frame; false if `id` is gone.
  bool Feed(WindowId id, const Input& input) {
    Entry* entry = At(id);
    if (entry == nullptr) {
      return false;
    }
    FeedControllers(*entry, input);
    return true;
  }

  // Has every controller write what it was last fed, in the order they were
  // put on: call before every Step() -- however many there are per frame --
  // since a truth actuator's effort lasts its one tick.
  void ApplyWrites() const {
    for (const Entry& entry : entries_) {
      for (const std::unique_ptr<IController>& controller : entry.controllers) {
        controller->ApplyWrites();
      }
    }
  }

 private:
  struct Entry {
    WindowId id;
    Window window;
    std::unique_ptr<IDisplay> display;
    std::vector<std::unique_ptr<IController>> controllers;
  };

  static void FeedControllers(const Entry& entry, const Input& input) {
    for (const std::unique_ptr<IController>& controller : entry.controllers) {
      controller->Feed(input);
    }
  }

  Entry* At(WindowId id) {
    const auto it = std::ranges::find_if(entries_, [&](const Entry& entry) {
      return entry.id == id;
    });
    return it == entries_.end() ? nullptr : &*it;
  }

  std::vector<Entry> entries_;
  // Reused window to window and frame to frame, so drawing allocates only as
  // a frame grows.
  Frame frame_;
  std::uint64_t next_id_ = 1;
};

}  // namespace achilles::display
