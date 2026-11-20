# Glossary

## The scene

- **Archetype**: a reusable tree of joints described in one `.arow` file:
  its joints, their field values and their attachments. Not a concept
  archetype (see [testing.md](testing.md) §4).
- **ArchetypeSet**: every loaded archetype. Turns names into rows.
- **Scene**: the one root archetype, attached to nothing. Holds the world
  and fixed geometry.
- **Robot**: an archetype attached to a joint of another archetype.
- **Instance**: one copy of an archetype. `copies` repeats the shape; every
  copy shares the same field values.
- **Joint**: one node of an archetype's tree, together with the body it
  moves.

## Attachments

- **Attachment**: something mounted on a joint that isn't a joint: a sensor
  or an actuator. Every instance of its archetype gets one.
- **Kind**: a sensor or actuator type, such as `Encoder` or `DcMotor`.
  Instances of one kind differ only in data. Named in `.arow` by its
  `kListName`.
- **Truth sensor / truth actuator**: a kind that reads or writes an exact
  sim quantity, with no error model.
- **Sim quantity**: a named sim value a kind reads, such as
  `JointPosition` or `WorldTransform`, bound to an algorithm's field.
- **Effect**: a reusable error model, such as noise, bias or quantization,
  that kinds apply to their output.
- **Reading**: an attachment's last output, with its sample time and
  sequence number.
- **Command**: what the user sends an actuator.
- **Handle**: the user's value for one attachment: a view and a row. Reads
  a sensor or commands an actuator.
- **State handle**: the user's raw, unsynchronized view of one sim quantity
  at one joint, for debugging and logging. Read-only or writable, per the
  state quantity list. Truth sensors are the synchronized path.
- **Writable handle**: the one write interface shared by writable state
  handles and actuator handles.

## Data and memory

- **Field**: one named quantity an algorithm stores per row, declared in its
  field enum.
- **Field traits**: the compile-time description of one field: its
  assembler, layout, ordering, indexing, buffering, slot and name.
- **Assembler**: converts a domain type to and from its flat scalar leaves
  in memory.
- **Leaf**: one scalar of a field's flattened value.
- **Slot** (`SharedAs`): a tag that makes fields of different algorithms
  one block of memory. The first hosted algorithm to name it owns it.
- **Arena**: the one block of memory allocated at init.
- **Block**: one field's region of the arena.
- **Buffering / parity**: a double-buffered field has a read half and a
  write half; flipping its parity swaps them.
- **Padding**: extra rows that fill out a lane group, seeded with a safe
  value (`PaddingSeed`).

## Rows and lanes

- **Row**: one entry of a field: usually one joint of one instance, or one
  pair, or one attachment.
- **Lane**: one element of a SIMD batch. `kLanes` lanes per batch.
- **Lane group**: `kLanes` consecutive rows processed together.
- **Base row**: the reserved row a root joint's parent index points at,
  seeded by an op's `Initialize`.
- **Topology**: the structure a traversal walks: parents, segments or pairs.
- **Row set**: an ordering for rows other than one per joint, such as
  collision pairs. Lives in `algorithms/row_sets/`.

## Policies

- **Policy**: an interchangeable choice of structure, in `policy/`.
- **Ordering / ordering plan**: decides which rows exist, their order and
  padding. The plan is its init-time result.
- **Layout**: how a field's rows are laid out in memory.
- **Indexing**: how a traversal index maps to a field element: per row, per
  lane group, or one global value.
- **Traversal**: how rows are walked: forward or backward over a tree,
  linearly, or over pairs.
- **Gate** (`PairGate`): a per-lane-group word field a gated pair traversal
  reads to decide which groups to visit; a zero word skips the group.

## Running

- **Algorithm**: one folder in `algorithms/`: a field enum, its traits, its
  ops and its steps.
- **View**: an algorithm's typed access to all its fields.
- **Cursor**: a view's typed access to one field.
- **Op**: math on one row group, declaring the fields it reads and writes.
- **Target / parent**: which row an op argument reads: the row being
  processed, or its parent.
- **Pass**: an op paired with a traversal.
- **Step**: one stage of an algorithm, running its passes. Named in step
  lists.
- **Step list**: an ordered list of steps.
- **Tick**: one `Simulation::Step` call, advancing the sim by `dt`.
- **Tick list / integrator**: the step list one tick runs, such as Euler or
  RK4. Chosen at init.
- **Setup list**: the step list run once at `Init`.
- **Hosting list**: every algorithm the sim hosts. Decides what memory
  exists, not what runs.
- **SimContext** (`sim_state`): the runtime object holding every view and
  topology, passed to every step.
- **SimAllocator**: the init-time builder of the arena and the SimContext.
- **Sim config**: whole-sim settings, such as the base transform and
  `base_acceleration` (minus gravity).
- **`MathematicalT` / `ScalarOperationT`**: the batch and scalar types set in
  `conventions.hpp`.
