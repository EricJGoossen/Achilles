// NOLINTBEGIN(misc-include-cleaner) -- main.cpp is deliberately a kitchen
// sink while actively developing: it includes every header so that a
// single-TU compile (and clang-tidy run, via scripts/check-tidy.sh's
// "every header must be reachable from a src/*.cpp file" invariant)
// exercises the whole codebase, not just what main() itself calls. This
// is not the standard for the rest of the codebase -- it's specific to
// this file.
#include <iostream>

#include "algorithms/aba/aba_data.hpp"
#include "algorithms/aba/aba_ops.hpp"
#include "algorithms/aba/aba_step.hpp"
#include "algorithms/conventions.hpp"
#include "algorithms/registry.hpp"
#include "algorithms/sim_config.hpp"
#include "domain/archetype.hpp"
#include "domain/joint_topology.hpp"
#include "domain/math/activation_mask.hpp"
#include "domain/math/matrix.hpp"
#include "domain/math/quaternion.hpp"
#include "domain/math/vector3.hpp"
#include "domain/math/vector6.hpp"
#include "domain/spatial/dual.hpp"
#include "domain/spatial/inertia.hpp"
#include "domain/spatial/transform.hpp"
#include "engine/algorithm_contract.hpp"
#include "engine/assembler.hpp"
#include "engine/field_contract.hpp"
#include "engine/memory/arena.hpp"
#include "engine/memory/binding.hpp"
#include "engine/memory/sim_allocator.hpp"
#include "engine/op_contract.hpp"
#include "engine/pass/algorithm_step.hpp"
#include "engine/pass/op_invoker.hpp"
#include "engine/pass/sim_context.hpp"
#include "engine/pass/traversals.hpp"
#include "engine/topology/layout.hpp"
#include "engine/topology/layout_policy.hpp"
#include "engine/topology/ordering_policy.hpp"
#include "engine/view/view.hpp"
#include "engine/view/view_contract.hpp"
#include "engine/view/view_factory.hpp"
#include "interface/archetype_loader.hpp"
#include "interface/sim_config_loader.hpp"
#include "interface/simulation.hpp"
#include "util/buffer.hpp"
#include "util/io.hpp"
#include "util/simd_ops.hpp"
#include "util/tmp.hpp"
#include "util/yaml.hpp"
// NOLINTEND(misc-include-cleaner)

int main() {}
