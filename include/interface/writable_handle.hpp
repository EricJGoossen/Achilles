#pragma once

namespace achilles::interface {

// A handle something outside the sim can write through: a writable state
// handle (state_handle.hpp: a joint's value) or an actuator's handle
// (attachment_handle.hpp: its command). Each names what it's written with,
// Written, and takes one in Write(), which waits in the sim for the next
// Step(). A read-only state handle and a sensor's handle aren't one: they
// have no Write().
template <typename H>
concept WritableHandle = requires {
  typename H::Written;
} && requires(const H& handle, const typename H::Written& value) {
  handle.Write(value);
};

}  // namespace achilles::interface
