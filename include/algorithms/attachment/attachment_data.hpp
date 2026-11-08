#pragma once

#include <cstddef>
#include <cstdint>
#include <string_view>
#include <tuple>
#include <type_traits>

#include "algorithms/attachment_kinds/attachment_kinds.hpp"
#include "algorithms/attachment_kinds/sim_input.hpp"
#include "algorithms/shared_slots.hpp"
#include "domain/attachment/attachment.hpp"
#include "domain/attachment/sim_quantities.hpp"
#include "domain/math/scalar.hpp"
#include "domain/math/vector3.hpp"
#include "domain/spatial/transform.hpp"
#include "engine/assembler.hpp"
#include "engine/field_contract.hpp"
#include "engine/memory/buffering_policy.hpp"
#include "engine/topology/layout_policy.hpp"
#include "engine/topology/orderings.hpp"
#include "engine/view/view_contract.hpp"
#include "engine/view/view_factory.hpp"
#include "util/tmp.hpp"

namespace achilles::algorithms::attachment {

enum class AttachmentField : std::uint8_t {
  kConfig,     // each attachment itself
  kCommand,    // its kind's Command, the caller's latest
  kState,      // its kind's State
  kOutput,     // what the Read and Drive passes write
  kPublished,  // the last complete output, what the user reads
  // How each body looks, from the .arow -- per joint, read by a camera at
  // every body (input::Visual*, OwnInput):
  kVisualPose,     // the seen shape's frame, in the body's own
  kVisualExtents,  // its half-extents, or zero to be unseen
  kVisualColor,    // rgb in [0, 1]; zero lets the camera pick one
  kVisualShape,    // which shape, a domain::geometry::VisualShape code
  kFirstInput,  // then one per AttachmentInputs entry, read at the mount joint
                // or, by a scanning kind, at every body

  kCount = kFirstInput + util::kSizeV<AttachmentInputs>
};
static_assert(engine::FieldEnumLike<AttachmentField>);

template <AttachmentField F>
struct AttachmentFieldTraits;

// Config, command, state and output hold a different record type per kind, so
// they use SegmentedLayout: each kind's rows take exactly its own records'
// size.
template <>
struct AttachmentFieldTraits<AttachmentField::kConfig> {
  using Assembler = util::ApplyT<engine::LeafAssembler, AttachmentKinds>;
  using Ordering = AttachmentOrdering;
  using Layout = engine::topology::SegmentedLayout;
};
template <>
struct AttachmentFieldTraits<AttachmentField::kCommand> {
  using Assembler = util::ApplyT<engine::LeafAssembler, AttachmentCommands>;
  using Ordering = AttachmentOrdering;
  using Layout = engine::topology::SegmentedLayout;
};
template <>
struct AttachmentFieldTraits<AttachmentField::kState> {
  using Assembler = util::ApplyT<engine::LeafAssembler, AttachmentStates>;
  using Ordering = AttachmentOrdering;
  using Layout = engine::topology::SegmentedLayout;
};
// Only the output is double-buffered: it's the one thing handed to the
// user. This is the sim's side, the half this tick's output goes into;
// ReadStep flips the group once its passes are done, so it becomes kPublished.
template <>
struct AttachmentFieldTraits<AttachmentField::kOutput> {
  using Assembler = util::ApplyT<engine::LeafAssembler, AttachmentOutputs>;
  using SharedAs = AttachmentOutputSlot;
  using Buffering = engine::memory::
      DoubleBuffered<AttachmentOutputSlot, engine::memory::BufferSide::kWrite>;
  using Ordering = AttachmentOrdering;
  using Layout = engine::topology::SegmentedLayout;
};
// The user's side: the last complete output, read between ticks, and by the
// Read and Drive passes as the output to carry forward when none is due.
template <>
struct AttachmentFieldTraits<AttachmentField::kPublished> {
  using Assembler = util::ApplyT<engine::LeafAssembler, AttachmentOutputs>;
  using SharedAs = AttachmentOutputSlot;
  using Buffering = engine::memory::
      DoubleBuffered<AttachmentOutputSlot, engine::memory::BufferSide::kRead>;
  using Ordering = AttachmentOrdering;
  using Layout = engine::topology::SegmentedLayout;
};
// The visual fields are one value per joint, loaded from the .arow under their
// kName; nothing writes them after.
template <>
struct AttachmentFieldTraits<AttachmentField::kVisualPose> {
  using Assembler = domain::spatial::TransformAssembler<ScalarOperationT>;
  using Ordering = engine::topology::TopologicalOrdering;
  using Layout = engine::topology::PlanarLayout;
  static constexpr std::string_view kName = "visual_pose";
};
template <>
struct AttachmentFieldTraits<AttachmentField::kVisualExtents> {
  using Assembler = domain::math::Vector3Assembler<ScalarOperationT>;
  using Ordering = engine::topology::TopologicalOrdering;
  using Layout = engine::topology::PlanarLayout;
  static constexpr std::string_view kName = "visual_extents";
};
template <>
struct AttachmentFieldTraits<AttachmentField::kVisualColor> {
  using Assembler = domain::math::Vector3Assembler<ScalarOperationT>;
  using Ordering = engine::topology::TopologicalOrdering;
  using Layout = engine::topology::PlanarLayout;
  static constexpr std::string_view kName = "visual_color";
};
template <>
struct AttachmentFieldTraits<AttachmentField::kVisualShape> {
  using Assembler = domain::math::ScalarAssembler<ScalarOperationT>;
  using Ordering = engine::topology::TopologicalOrdering;
  using Layout = engine::topology::PlanarLayout;
  static constexpr std::string_view kName = "visual_shape";
};
// An input's source field, exactly as its own Algorithm declared it -- the
// same SharedAs slot, so this is the source's block, not a copy -- minus
// any kName (the source already populates it).
template <AttachmentField F>
  requires(F >= AttachmentField::kFirstInput && F < AttachmentField::kCount)
struct AttachmentFieldTraits<F> {
  using Source = InputSource<std::tuple_element_t<
      static_cast<std::size_t>(F) -
          static_cast<std::size_t>(AttachmentField::kFirstInput),
      util::ToTupleT<AttachmentInputs>>>;
  using Assembler = typename Source::Type::Assembler;
  using SharedAs = typename Source::Type::SharedAs;
  using Ordering = typename Source::Ordering;
  using Indexing = typename Source::Indexing;
  using Buffering = typename Source::Buffering;
  using Layout = typename Source::Layout;
};
static_assert(engine::FieldTraitsLike<AttachmentField, AttachmentFieldTraits>);

using AttachmentView =
    engine::view::ViewFactory<AttachmentField, AttachmentFieldTraits>;
static_assert(engine::view::ViewLike<AttachmentView, AttachmentField>);

// The field each own input is.
template <typename Quantity>
inline constexpr AttachmentField kOwnFieldOf = AttachmentField::kCount;
template <>
inline constexpr AttachmentField kOwnFieldOf<input::VisualPose> =
    AttachmentField::kVisualPose;
template <>
inline constexpr AttachmentField kOwnFieldOf<input::VisualExtents> =
    AttachmentField::kVisualExtents;
template <>
inline constexpr AttachmentField kOwnFieldOf<input::VisualColor> =
    AttachmentField::kVisualColor;
template <>
inline constexpr AttachmentField kOwnFieldOf<input::VisualShape> =
    AttachmentField::kVisualShape;

// Input's own field: AttachmentInputs' entries follow kFirstInput (scene inputs
// included -- the index, not the field, says where it's read), and an own
// input is the field kOwnFieldOf names.
template <typename Input>
inline constexpr AttachmentField kInputFieldOf = static_cast<
    AttachmentField>(static_cast<std::size_t>(AttachmentField::kFirstInput) + util::kIndexOfV<Input, AttachmentInputs>);
template <typename Quantity>
inline constexpr AttachmentField kInputFieldOf<OwnInput<Quantity>> =
    kOwnFieldOf<Quantity>;

// An own input's field gives its quantity's Value.
template <typename Quantity>
inline constexpr bool kOwnFieldGivesValue = std::is_same_v<
    typename Quantity::Value,
    engine::AssembledT<
        typename AttachmentFieldTraits<kOwnFieldOf<Quantity>>::Assembler>>;
static_assert(kOwnFieldGivesValue<input::VisualPose>);
static_assert(kOwnFieldGivesValue<input::VisualExtents>);
static_assert(kOwnFieldGivesValue<input::VisualColor>);
static_assert(kOwnFieldGivesValue<input::VisualShape>);

}  // namespace achilles::algorithms::attachment
