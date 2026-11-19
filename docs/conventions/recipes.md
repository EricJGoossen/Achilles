# Recipes

Checklists for common changes. Each step follows the doc it links.

## Add an algorithm

1. Create `algorithms/<name>/` with its data, ops and step files
   ([algorithms.md](algorithms.md)).
2. For each field shared with another algorithm, add a tag to
   `shared_slots.hpp` if none exists, and name it in `SharedAs` on both
   sides with the same assembler and ordering.
3. If it needs rows other than one per joint, add a row set in
   `algorithms/row_sets/`.
4. Put shape constants in `conventions.hpp`, and repeat counts beside the
   step list that repeats them.
5. Add it to the hosting list, after every algorithm whose slots it reads.
6. Add its steps to the tick lists, and to the setup list if it needs
   init-time work.
7. Give `kName` to fields loaded from `.arow` or read by sensors, and update
   [file-formats.md](file-formats.md).
8. Write tests for each file ([testing.md](testing.md)), at lane sizes 4, 8
   and 32.
9. If the change is significant, add an example ([examples.md](examples.md)).

## Add a sensor or actuator kind

1. Check it models a real part ([sim-model.md](sim-model.md)). A variant of an
   existing kind is a template or alias, not a new object.
2. Add its file to `domain/sensing/` or `domain/actuation/`
   ([domain.md](domain.md)). Its error model uses the shared effects.
3. If it reads a sim value that doesn't exist yet, expose it first (see
   below).
4. Register it in `SensorKinds` or `ActuatorKinds`.
5. Read and validate its settings in `interface`, throwing on bad input
   ([errors.md](errors.md)).
6. Update [file-formats.md](file-formats.md) with its `kind` name and
   settings.
7. Write tests for the kind, and an `EXPECT_THROW` test for each setting
   check.

## Expose a new sim value

For a value that sensors or state handles can't reach yet.

1. Add the quantity to `domain/attachment/sim_quantities.hpp`, named for
   what it is, not for the algorithm that computes it.
2. Bind it to the field that holds it in
   `algorithms/attachment_kinds/sim_input.hpp`. If no algorithm shares that
   field yet, give the field a `SharedAs` slot.
3. List it in `interface/state_quantities.hpp`, marked read-only or
   writable.
4. Test reading it through a state handle, and writing it if it is
   writable.

## Add a policy

1. If it is a new policy type, create `policy/<type>/` with the concept file
   named after the folder, and place it in the layer table in
   [policy.md](policy.md).
2. Add the policy in its own file, as an empty struct with only `static`
   members, ending with its concept `static_assert`.
3. If engine traits can omit it, document the default in `engine/`.
4. Write a concept archetype for a new concept, and a test for each shape
   of the policy ([testing.md](testing.md) §4, §9.2).

## Add a domain object

1. Put it in the lowest domain layer that has everything it needs
   ([domain.md](domain.md)).
2. Template math types on `T`, so they work for scalars and batches.
3. Add a concrete alias in `conventions.hpp` if algorithms use it.
4. Write its comments ([comments.md](comments.md)) and tests.

## Add an example

Follow [examples.md](examples.md).
