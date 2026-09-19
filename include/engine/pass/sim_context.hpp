#pragma once

#include <concepts>
#include <tuple>
#include <type_traits>
#include <unordered_map>

#include "domain/joint_topology.hpp"
#include "engine/algorithm_contract.hpp"
#include "engine/memory/binding.hpp"
#include "engine/topology/layout_policy.hpp"
#include "engine/view/view_contract.hpp"
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
    StepT::Step(sim_context, config, dt);
  }
}

}  // namespace detail

// Structural counterpart to SimContext, for constraining a Step function's
// SimStateT (e.g. ABAStep::Step, algorithms/aba/aba_step.hpp) without that
// Step function naming SimContext<Algorithms...> directly -- it has no
// reason to know the full Algorithms... pack of whichever sim hosts it (see
// ABAStep's own comment on why SimStateT stays a template parameter). Only
// checks TopologyFor, not ViewFor, even though a Step function calls both on
// itself now (detail::StepAlgorithm above no longer looks up the view on a
// Step function's behalf). TopologyFor can be probed generically because
// TopologyMap is keyed at runtime by memory::SlotId<Policy>() -- any Policy
// type typechecks against it, whether or not that policy is actually
// registered. ViewFor, by contrast, is keyed at compile time by std::get on
// the concrete std::tuple<Algorithms::View...> a real SimContext holds: only
// a View type that's actually a member of *that* tuple compiles, so there's
// no single probe View that would typecheck against every possible
// SimContext<...> instantiation the way LayoutPolicyProbe does for
// TopologyFor. A Step function naming a View that its host sim doesn't
// actually carry simply fails to compile where it's really instantiated,
// same as it would for any other mistyped member access -- just not
// something this concept can catch structurally ahead of that. Probed with
// LayoutPolicyProbe the same way TraversalLike (engine/pass/traversals.hpp)
// probes Apply/InitOp with a TraversalProbe -- TopologyFor is templated on
// the caller's Policy, so checking it at all means picking some concrete
// stand-in Policy to instantiate with.
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
// pack as whichever SimAllocator built it, so ViewFor<ViewT>() has a real
// std::tuple<Algorithms::View...> to std::get out of.
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

  // The original, Algorithm-keyed overload -- kept for existing external
  // callers that only ever have the whole AlgorithmT in hand and shouldn't
  // have to also know its View type by name (SimAllocator::ViewFor,
  // interface::Simulation::ViewFor, and every test that reaches a View
  // through one of those). Just resolves A::View and hands off to the
  // View-keyed overload below.
  template <AlgorithmLike A>
  typename A::View ViewFor() const {
    return ViewFor<typename A::View>();
  }

  // Keyed by the View type itself, via std::get on views_ -- mirrors
  // TopologyFor<Policy> being keyed by the Policy type, so a Step function
  // can name its own View the same way it already names its own Policy
  // (sim_state.template ViewFor<ABAView>()), rather than having its View
  // resolved and handed to it up front. Constrained to !AlgorithmLike<ViewT>
  // purely to stay unambiguous against the overload above -- a View type is
  // never itself AlgorithmLike (it has no ::Enum/::View/::Step), so every
  // real call resolves to exactly one overload. A Step naming a View type
  // its host sim doesn't actually carry fails to compile here, not at
  // runtime -- see SimContextLike's own comment on why that can't be
  // checked structurally ahead of time.
  template <typename ViewT>
    requires(!AlgorithmLike<ViewT>)
  ViewT ViewFor() const {
    return std::get<ViewT>(views_);
  }

  // Runs every hosted Algorithm's own Step exactly once, passing along this
  // same context and `config` -- whatever caller-supplied, whole-simulation
  // config value a real Step actually reads (see algorithms::SimConfig),
  // left as a template parameter here so engine::pass never has to name a
  // concrete config type. `dt` is the tick's own timestep. One shared shape
  // (SimContext, Config, dt) for every Step::Step, regardless of whether it
  // uses all of it: state only some algorithms would receive is exactly the
  // kind of ambient global that's easy to wire wrong silently. Each Step
  // looks up its own View via sim_context.ViewFor<ViewT>(), the same way it
  // looks up its own Topology.
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
