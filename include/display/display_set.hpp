#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <tuple>
#include <utility>
#include <vector>

#include "display/attached_display.hpp"
#include "display/display.hpp"
#include "display/frame.hpp"
#include "display/input.hpp"
#include "display/window.hpp"
#include "util/tmp.hpp"

namespace achilles::display {

// One display in a DisplaySet, as Add() returns it: never reused, so an id
// of a display since removed finds nothing rather than another display.
struct DisplayId {
  std::uint64_t value = 0;

  friend bool operator==(const DisplayId&, const DisplayId&) = default;
};

// Every attached display, each bound to its attachments' handles and paired
// with its own window: what draws them, in the order they were attached,
// and what applies the efforts their controls ask for. Any DisplayLike can
// be added -- nothing lists the kinds -- each kept behind one virtual call a
// frame, which drawing its window dwarfs.
//
// Displays come and go at any point between two frames: Add() puts one up,
// Find() reaches one to change its settings, Swap() puts another in its
// window, Remove() takes one down. Nothing here touches the sim, and the
// handles stay valid as long as the sim keeps them (until it's next
// Init()ed).
//
// A window that has drawn holds textures in the screen's GL context, so the
// context must be current when one is dropped: in Draw(), and around a
// Remove() or the set's destruction (Renderer makes it so).
class DisplaySet {
 public:
  // Puts up `display`, bound to `handles`, in a new window titled `title`.
  template <DisplayLike D>
  DisplayId Add(D display, HandlesOfT<D> handles, std::string_view title) {
    const DisplayId id{next_id_++};
    entries_.push_back(
        {.id = id,
         .window = Window(title, D::kWidth, D::kHeight),
         .display = std::make_unique<AttachedDisplay<D>>(
             std::move(display), std::move(handles)
         )}
    );
    return id;
  }

  // The display `id` names, to change its settings from the next frame on --
  // or null if it's gone, or isn't a D. Valid until the next Add(), Swap(),
  // Remove() or Draw().
  template <DisplayLike D>
  D* Find(DisplayId id) {
    Entry* entry = At(id);
    if (entry == nullptr) {
      return nullptr;
    }
    auto* attached = dynamic_cast<AttachedDisplay<D>*>(entry->display.get());
    return attached == nullptr ? nullptr : &attached->Display();
  }

  // Replaces the display `id` names, of whatever kind, with `display` bound
  // to `handles`, in the same window, under the same id. Throws
  // std::invalid_argument if `id` is gone.
  template <DisplayLike D>
  void Swap(DisplayId id, D display, HandlesOfT<D> handles) {
    Entry* entry = At(id);
    if (entry == nullptr) {
      throw std::invalid_argument("DisplaySet::Swap: no such display.");
    }
    entry->display = std::make_unique<AttachedDisplay<D>>(
        std::move(display), std::move(handles)
    );
  }

  // Takes the display `id` names down, with its window; false if it's
  // already gone. Its actuators get no effort from it from then on.
  bool Remove(DisplayId id) {
    return std::erase_if(entries_, [&](const Entry& entry) {
             return entry.id == id;
           }) > 0;
  }

  bool Contains(DisplayId id) const {
    return std::ranges::any_of(entries_, [&](const Entry& entry) {
      return entry.id == id;
    });
  }

  std::size_t Size() const { return entries_.size(); }

  // The title of the display `id` names' window, or empty if it's gone.
  std::string_view Title(DisplayId id) const {
    const auto it = std::ranges::find_if(entries_, [&](const Entry& entry) {
      return entry.id == id;
    });
    return it == entries_.end() ? std::string_view{} : it->window.Title();
  }

  // Draws every display into its window and takes what its controls were
  // left at (Feed), dropping first the ones whose window was closed since.
  // Runs between the screen's BeginFrame() and EndFrame(). Returns how many
  // are up.
  std::size_t Draw() {
    std::erase_if(entries_, [](const Entry& entry) {
      return !entry.window.IsOpen();
    });
    for (Entry& entry : entries_) {
      frame_.Clear();
      entry.display->Draw(&frame_);
      const Input input = entry.window.Present(frame_);
      // A window closed this frame asks nothing more of its actuators.
      entry.display->Feed(entry.window.IsOpen() ? input : Input{});
    }
    return entries_.size();
  }

  // Turns `input` -- what the controls of the display `id` names were left
  // at -- into that display's efforts, applied from the next ApplyEfforts()
  // on. Draw() does this for every display each frame; false if `id` is
  // gone.
  bool Feed(DisplayId id, const Input& input) {
    Entry* entry = At(id);
    if (entry == nullptr) {
      return false;
    }
    entry->display->Feed(input);
    return true;
  }

  // Adds every display's efforts to what its actuators apply on the next
  // tick. A truth actuator's effort lasts that one tick, so this runs before
  // every Step() -- however many there are per frame.
  void ApplyEfforts() const {
    for (const Entry& entry : entries_) {
      entry.display->ApplyEfforts();
    }
  }

 private:
  struct Entry {
    DisplayId id;
    Window window;
    std::unique_ptr<IAttachedDisplay> display;
  };

  Entry* At(DisplayId id) {
    const auto it = std::ranges::find_if(entries_, [&](const Entry& entry) {
      return entry.id == id;
    });
    return it == entries_.end() ? nullptr : &*it;
  }

  std::vector<Entry> entries_;
  // Reused display to display and frame to frame, so drawing allocates only
  // as a frame grows.
  Frame frame_;
  std::uint64_t next_id_ = 1;
};

}  // namespace achilles::display
