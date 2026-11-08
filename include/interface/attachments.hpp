#pragma once

#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <string_view>
#include <tuple>
#include <vector>

#include "algorithms/attachment_kinds/attachment_kinds.hpp"
#include "domain/joint_data.hpp"
#include "engine/pass/sim_context.hpp"
#include "interface/attachment_handle.hpp"
#include "util/tmp.hpp"

// Turns what the .arow says about a scene's sensors and actuators into the
// built sim. What makes one well formed is its own business (each handle's
// Configure): this only walks them and finds each one's kind.
namespace achilles::interface {

// One sensor or actuator as Simulation::Attachments() lists it: its handle,
// and the name and instance Sensor() or Actuator() would find it by.
template <domain::attachment::AttachmentLike K>
struct NamedAttachment {
  std::string name;
  std::size_t instance = 0;
  AttachmentHandle<K> handle;
};

template <domain::attachment::AttachmentLike K>
using NamedAttachments = std::vector<NamedAttachment<K>>;

// Every sensor and actuator of a scene, one list per registered kind, in
// AttachmentKinds order: std::get<NamedAttachments<K>>(lists) is kind K's,
// archetype by archetype, instance by instance.
using AttachmentLists = util::ApplyT<
    std::tuple,
    util::
        TransformT<NamedAttachments, algorithms::attachment::AttachmentKinds>>;

namespace detail {

using AttachmentHandles =
    util::TransformT<AttachmentHandle, algorithms::attachment::AttachmentKinds>;

// Calls fn.template operator()<H>() for the handle whose kind is named
// `kind`; false if none is.
template <typename Fn, typename... Handles>
bool WithKindNamed(
    util::TypeList<Handles...> /*handles*/, std::string_view kind, const Fn& fn
) {
  return (
      (Handles::kListName == kind ? (fn.template operator()<Handles>(), true)
                                  : false) ||
      ...
  );
}

}  // namespace detail

// Configures every attachment of `archetypes`, each with its own random
// stream drawn from `seed`. Run once the sim is allocated and before its Setup
// list (whose InitSteps seed each attachment's state from its configuration).
// Throws std::invalid_argument for an attachment of no registered kind, and
// whatever its own Configure throws.
template <engine::pass::SimContextLike SimStateT, typename Archetypes>
void ConfigureAttachments(
    const SimStateT& sim_state, const Archetypes& archetypes, std::uint64_t seed
) {
  archetypes.ForEachAttachment(
      seed,
      [&](const domain::JointAttachment& attachment,
          std::size_t instance,
          std::size_t /*local_instance*/,
          std::size_t entry,
          std::uint64_t stream) {
        const bool known = detail::WithKindNamed(
            detail::AttachmentHandles{},
            attachment.list,
            [&]<typename Handle>() {
              Handle::Configure(sim_state, attachment, instance, entry, stream);
            }
        );
        if (!known) {
          throw std::invalid_argument(
              attachment.key + " '" + attachment.name + "': no " +
              attachment.key + " kind named '" + attachment.list + "'."
          );
        }
      }
  );
}

}  // namespace achilles::interface
