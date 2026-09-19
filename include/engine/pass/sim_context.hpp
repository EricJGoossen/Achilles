#pragma once

#include <concepts>
#include <tuple>
#include <type_traits>
#include <unordered_map>

#include "domain/joint_topology.hpp"
#include "engine/algorithm_contract.hpp"
#include "engine/memory/binding.hpp"
#include "engine/topology/layout_policy.hpp"
#include "util/tmp.hpp"

namespace achilles::engine::pass {

// Keyed by memory::SlotId<Policy>() -- one entry per distinct ordering
// policy any hosted field actually names. Lives here (not in engine::memory,
// where it's built) because SimContext's own constructor has to name this
// type, and SimContext lives here -- see SimContext's own comment for why.
using TopologyMap = std::unordered_map<const void*, domain::JointTopology>;

namespace detail {

// Stand-in LayoutPolicy purely for probing SimContextLike below -- it only
// needs to satisfy LayoutPolicyLike (std::is_empty_v), the same way
// TraversalProbe (engine/pass/traversals.hpp) stands in for a JointTopology
// when checking TraversalLike. TopologyFor is a template, so SimContextLike
// can't ask "does this type have a TopologyFor" without naming some concrete
// Policy to instantiate it with; this one is never meant to be looked up
// against a real TopologyMap, only to typecheck.
struct LayoutPolicyProbe {};
static_assert(topology::LayoutPolicyLike<LayoutPolicyProbe>);

// Steps one Algorithm, unless it declares NoStep (see algorithm_contract.hpp)
// -- an Algorithm with nothing to step on its own account is silently
// skipped, rather than requiring every caller to filter it out of the
// Algorithms... pack by hand. Config is a template parameter, not a
// concrete type named here, purely so this generic engine layer never has
// to know what a real config looks like (see algorithms::SimConfig for the
// one this codebase actually uses) -- it only ever forwards `config` on to
// AlgorithmT::Step.
template <typename AlgorithmT, typename SimContextT, typename Config>
void StepAlgorithm(
    const SimContextT& sim_context, const Config& config, float dt
) {
  using StepT = typename AlgorithmT::Step;
  if constexpr (!std::is_same_v<StepT, NoStep>) {
    StepT::Step(
        sim_context.template ViewFor<AlgorithmT>(), sim_context, config, dt
    );
  }
}

}  // namespace detail

// Structural counterpart to SimContext, for constraining a Step function's
// SimStateT (e.g. ABAStep::Step, algorithms/aba/aba_step.hpp) without that
// Step function naming SimContext<Algorithms...> directly -- it has no
// reason to know the full Algorithms... pack of whichever sim hosts it (see
// ABAStep's own comment on why SimStateT stays a template parameter). Only
// checks TopologyFor, the one member a Step function actually calls itself:
// ViewFor is called on a Step function's behalf, by SimContext::Step, before
// a Step function ever sees sim_state (detail::StepAlgorithm above), so it
// isn't part of the surface a Step function needs constrained here. Probed
// with LayoutPolicyProbe the same way TraversalLike
// (engine/pass/traversals.hpp) probes Apply/InitOp with a TraversalProbe --
// TopologyFor is templated on the caller's Policy, so checking it at all
// means picking some concrete stand-in Policy to instantiate with.
template <typename T>
concept SimContextLike = requires(const T& ctx) {
  {
    ctx.template TopologyFor<detail::LayoutPolicyProbe>()
  } -> std::same_as<const domain::JointTopology&>;
};

// The allocator-built half of what a Step function needs: every hosted
// Algorithm's own View, plus the per-ordering-policy JointTopology map.
// Holds its own copies of both (a small map plus a tuple of lightweight
// View handles into Arena memory -- cheap to copy, since a View is itself
// just a few pointers/strides into that memory, never the memory itself),
// rather than borrowing SimAllocator's own storage -- so a SimContext
// legitimately outlives the SimAllocator that built it (see
// SimAllocator::State()/Extract()); the only thing it actually depends on
// staying alive is the Arena backing the Views it holds.
//
// Lives in engine::pass (not engine::memory, where it's built, or domain,
// which stays a dependency-free value-type layer) so it can sit next to
// Step, its one real consumer. Still templated on the same Algorithms...
// pack as whichever SimAllocator built it, so View<A>() can return A::View
// directly.
template <AlgorithmLike... Algorithms>
class SimContext {
 public:
  SimContext(
      TopologyMap topologies, std::tuple<typename Algorithms::View...> views
  )
      : topologies_(std::move(topologies)), views_(std::move(views)) {}

  template <topology::LayoutPolicyLike Policy>
  const domain::JointTopology& TopologyFor() const {
    return topologies_.at(memory::SlotId<Policy>());
  }

  template <AlgorithmLike A>
  typename A::View ViewFor() const {
    return std::get<typename A::View>(views_);
  }

  // Runs every hosted Algorithm's own Step exactly once, each over its own
  // View plus this same context and `config` -- whatever caller-supplied,
  // whole-simulation config value a real Step actually reads (see
  // algorithms::SimConfig), left as a template parameter here so
  // engine::pass never has to name a concrete config type. `dt` is the
  // tick's own timestep. One shared shape (View, SimContext, Config, dt)
  // for every Step::Step, regardless of whether it uses all of it: state
  // only some algorithms would receive is exactly the kind of ambient
  // global that's easy to wire wrong silently.
  template <typename Config>
  void Step(float dt, const Config& config) const {
    (detail::StepAlgorithm<Algorithms>(*this, config, dt), ...);
  }

 private:
  TopologyMap topologies_;
  std::tuple<typename Algorithms::View...> views_;
};
static_assert(SimContextLike<SimContext<>>);

// Builds SimContext<Algorithms...> from a util::TypeList<Algorithms...>
// instead of a caller restating the pack -- same shape as, and for the same
// reason as, memory::SimAllocatorFor (engine/memory/sim_allocator.hpp):
// registries like algorithms::RegisteredAlgorithms hand out a TypeList, and
// a caller storing a SimContext (e.g. interface::Simulation, past
// SimAllocator::State()) needs to name its type the same
// TypeList-unwrapped way.
template <typename List>
struct SimContextFor;

template <AlgorithmLike... Algorithms>
struct SimContextFor<util::TypeList<Algorithms...>> {
  using Type = SimContext<Algorithms...>;
};

template <typename List>
using SimContextForT = typename SimContextFor<List>::Type;

}  // namespace achilles::engine::pass
