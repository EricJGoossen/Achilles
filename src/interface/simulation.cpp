// Simulation's loading, building and lookups: everything that runs once,
// outside the tick, compiled here once instead of in every file that
// includes the header.
// Doc: docs/code/interface_simulation.md

#include "interface/simulation.hpp"

#include <cstddef>
#include <cstdint>
#include <exception>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "algorithms/clock/clock_data.hpp"
#include "algorithms/integrators.hpp"
#include "domain/archetype/archetype.hpp"
#include "domain/joint_data.hpp"
#include "domain/math/vector3.hpp"
#include "domain/sim_clock.hpp"
#include "engine/topology/orderings.hpp"
#include "interface/archetype_loader.hpp"
#include "interface/attachments.hpp"
#include "interface/sim_config_loader.hpp"
#include "util/io.hpp"

namespace achilles::interface {

bool Simulation::LoadConfigFile(const std::string& path) {
  try {
    config_ = LoadSimConfig(path);
  } catch (const std::exception& e) {
    return util::Warning("Failed to read config from file", e.what());
  }
  return true;
}

bool Simulation::LoadArowFile(const std::string& path) {
  try {
    archetypes_ = domain::archetype::ArchetypeSet(LoadArchetypes(path));
  } catch (const std::exception& e) {
    return util::Warning("Failed to load world from file", e.what());
  }
  return true;
}

bool Simulation::Init(std::optional<std::uint64_t> seed) {
  if (archetypes_.All().empty()) {
    return util::Warning(
        "Simulation::Init() called before Simulation::LoadArowFile() -- no "
        "archetypes loaded."
    );
  }
  seed_ = seed.value_or(GenerateSeed());
  try {
    state_ = Allocator::Build(archetypes_);
    ConfigureAttachments(state_->context, archetypes_, seed_);
  } catch (const std::exception& e) {
    state_.reset();
    return util::Warning("Simulation allocation failed", e.what());
  }
  // Model constants derived from the initial state, and every sensor's
  // and actuator's starting state (algorithms::Setup).
  return RunStepList(&algorithms::StepIntegrator<algorithms::Setup>, 0.0F);
}

std::vector<JointAddress> Simulation::Joints() const {
  return archetypes_.JointAddresses();
}

domain::math::Vector3<float> Simulation::Up() const {
  // Every lane of the config holds the same value: the first's.
  const auto linear = config_.base_acceleration.Linear();
  const domain::math::Vector3<float> against(
      linear.X().get(0), linear.Y().get(0), linear.Z().get(0)
  );
  const float length = against.Norm();
  return length > 0.0F ? against * (1.0F / length)
                       : domain::math::Vector3<float>(0.0F, 0.0F, 1.0F);
}

AttachmentLists Simulation::Attachments() const {
  const auto& context = Context("Attachments");
  AttachmentLists lists;
  archetypes_.ForEachAttachment(
      seed_,
      [&](const domain::JointAttachment& attachment,
          std::size_t /*instance*/,
          std::size_t local_instance,
          std::size_t entry,
          std::uint64_t /*stream*/,
          const domain::archetype::Archetype& archetype) {
        // Init() configured every attachment, so its kind is registered.
        detail::WithKindNamed(
            detail::AttachmentHandles{},
            attachment.list,
            [&]<typename Handle>() {
              std::get<NamedAttachments<typename Handle::Kind>>(lists)
                  .push_back(
                      {.name = attachment.name,
                       .instance = local_instance,
                       .handle = Handle::Find(context, entry),
                       .joint =
                           {.archetype = archetype.name,
                            .joint =
                                archetype.joint_names.empty()
                                    ? std::string()
                                    : archetype.joint_names[attachment.joint],
                            .instance = local_instance}}
                  );
            }
        );
      }
  );
  return lists;
}

domain::SimClock Simulation::Clock() const {
  using algorithms::clock::ClockField;
  using algorithms::clock::ClockView;
  if (!state_.has_value()) {
    return {};
  }
  return state_->context.template ViewFor<ClockView>()
      .template Load<ClockField::kClock, domain::SimClock>(0);
}

std::size_t Simulation::JointRow(const JointAddress& joint, const char* caller)
    const {
  return archetypes_.RowOf(
      joint,
      Context(caller).template LayoutFor<engine::topology::TopologicalOrdering>(
      )
  );
}

const algorithms::RegisteredSimContext& Simulation::Context(const char* caller
) const {
  if (!state_.has_value()) {
    throw std::runtime_error(
        std::string("Simulation::") + caller +
        "() called before Simulation::Init() -- the sim is not yet "
        "initialized."
    );
  }
  return state_->context;
}

}  // namespace achilles::interface
