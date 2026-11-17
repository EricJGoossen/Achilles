#pragma once

#include <string_view>

#include "display/contract/display.hpp"
#include "domain/attachment/attachment_kind.hpp"
#include "interface/attachment_handle.hpp"
#include "interface/simulation.hpp"
#include "util/tmp.hpp"

namespace achilles::test_support {

// The handle of the K named `name`, from the sim: a sensor's or an
// actuator's, by what K is.
template <typename K>
interface::AttachmentHandle<K> HandleNamed(
    const interface::Simulation& sim, std::string_view name
) {
  if constexpr (domain::attachment::SensorLike<K>) {
    return sim.Sensor<K>(name);
  } else {
    return sim.Actuator<K>(name);
  }
}

// The handles a display D is bound to, of the attachments `names` names,
// one per kind D names, in order -- what a test hands a display.
template <typename D, typename... Names>
display::HandlesOfT<D> HandlesNamed(
    const interface::Simulation& sim, const Names&... names
) {
  return [&]<typename... Ks>(util::TypeList<Ks...> /*kinds*/) {
    return display::HandlesOfT<D>{HandleNamed<Ks>(sim, names)...};
  }(typename D::Attachments{});
}

}  // namespace achilles::test_support
