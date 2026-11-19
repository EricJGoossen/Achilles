# archetype_set.hpp

## Role

Every loaded archetype together, and every question that looks across them:
name lookups, global instance numbering, attachment entries and their random
streams.

## Connections

- Built by `interface::Simulation::LoadArowFile` from
  `interface::LoadArchetypes`.
- `engine/allocation/sim_allocator.hpp` builds the sim from it and populates
  fields by name.
- `interface/attachments.hpp` walks every attachment with
  `ForEachAttachment` at init.
- `Simulation` turns user names into rows with `RowOf`, `AttachmentEntry`
  and `AttachmentRow`.

## Invariants

- Instances are numbered end to end in archetype order, the same numbering
  every ordering policy and `JointLayout` use.
- A list's entries are numbered archetype by archetype, instance by
  instance, in declaration order, matching the rows the list ordering gives
  them.
- Each attachment's random stream comes from the seed and its place among
  all attachments, whatever its list. Adding or reordering attachments in a
  scene changes the streams of those after it.
- Name lookups resolve user input, so a miss throws `std::invalid_argument`.

## Pitfalls

- Archetypes built without joint names can't be addressed by joint name;
  `JointAddresses()` is empty for them.

## When changing it

- `policy/ordering/` if numbering changes; both must stay in
  step.
- `interface/simulation.hpp` and `interface/attachments.hpp`.
