#pragma once

#include <cstddef>
#include <tuple>
#include <type_traits>
#include <utility>

#include "domain/joint_topology.hpp"
#include "engine/algorithm_contract.hpp"
#include "engine/memory/binding.hpp"
#include "engine/topology/layout_policy.hpp"
#include "engine/view/view_contract.hpp"
#include "util/tmp.hpp"

namespace achilles::engine::pass {

namespace detail {

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

// Collects FieldOrderingT<EnumT, Traits, F> for every field of one Algorithm
// into a TypeList -- UsedOrderingsFor below concatenates and de-dups these
// across a whole Algorithms... pack to find every distinct ordering policy
// actually in play. Lives here, not engine::memory (where this computation
// used to live, back when only SimAllocator needed it): SimContext now needs
// the same result to size its own topology tuple below, and SimAllocator --
// which already depends on this header -- reuses UsedOrderingsFor itself
// rather than keeping a second copy that could silently drift out of sync
// with it.
template <typename EnumT, template <EnumT> class Traits, std::size_t... Is>
auto CollectFieldOrderings(std::index_sequence<Is...>)
    -> util::TypeList<
        memory::FieldOrderingT<EnumT, Traits, static_cast<EnumT>(Is)>...>;

template <typename AlgorithmT>
struct AlgorithmOrderings;

template <typename EnumT, template <EnumT> class Traits, typename StepT>
struct AlgorithmOrderings<Algorithm<EnumT, Traits, StepT>> {
  using Type = decltype(CollectFieldOrderings<EnumT, Traits>(
      std::make_index_sequence<static_cast<std::size_t>(EnumT::kCount)>{}
  ));
};

}  // namespace detail

// Every distinct ordering policy named by any field of any of Algorithms...,
// de-duplicated -- two algorithms sharing the same policy (e.g. ABA and VI
// both naming TopologicalOrdering, see algorithms/vi/vi_data.hpp) collapse
// to one entry, since TopologyFor<Policy> only ever needs one JointTopology
// per policy no matter how many fields share it. Public, not detail: both
// SimContext (to size its own topology tuple below) and SimAllocator (to
// build the matching real JointTopology values, engine/memory/
// sim_allocator.hpp) need to name this exact same list.
template <AlgorithmLike... Algorithms>
using UsedOrderingsFor = util::UniqueT<
    util::ConcatT<typename detail::AlgorithmOrderings<Algorithms>::Type...>>;

// One JointTopology per distinct ordering policy, wrapped so each entry gets
// its own distinct tuple-element type even though every wrapped value is the
// exact same domain::JointTopology type underneath -- std::get<T> needs T to
// be unique within the tuple, and multiple ordering policies routinely share
// a JointTopology's shape without being the same policy. View itself never
// needed a wrapper like this: ViewFactory<EnumT, Traits> already produces a
// distinct type per Algorithm, so std::tuple<Algorithms::View...> has no
// such collision to begin with.
template <typename Policy>
struct TopologySlot {
  domain::JointTopology topology;
};

// The concrete tuple type SimContext<Algorithms...> stores its topologies
// in -- one TopologySlot<Policy> per entry of UsedOrderingsFor<Algorithms...>,
// in the same order.
template <AlgorithmLike... Algorithms>
using TopologyTupleFor = util::ToTupleT<
    util::TransformT<TopologySlot, UsedOrderingsFor<Algorithms...>>>;

// The allocator-built half of what a Step function needs: every hosted
// Algorithm's own View, plus one JointTopology per distinct ordering policy.
// Holds its own copies of both -- a tuple of TopologySlots plus a tuple of
// lightweight View handles into Arena memory, both cheap to copy since
// neither owns the memory it points/refers into -- rather than borrowing
// SimAllocator's own storage, so a SimContext legitimately outlives the
// SimAllocator that built it (see SimAllocator::State()/Extract()); the only
// thing it actually depends on staying alive is the Arena backing the Views
// and JointTopology instances it holds.
//
// Lives in engine::pass (not engine::memory, where it's built, or domain,
// which stays a dependency-free value-type layer) so it can sit next to
// Step, its one real consumer. Still templated on the same Algorithms...
// pack as whichever SimAllocator built it, so both ViewFor<ViewT>() and
// TopologyFor<Policy>() have a real tuple to std::get out of -- both keyed
// by std::get at compile time now, rather than TopologyFor being the odd
// one out on a runtime std::unordered_map: naming a Policy or a View type
// the host sim doesn't actually carry fails to compile here, not at
// runtime, for either one.
template <AlgorithmLike... Algorithms>
class SimContext {
 public:
  SimContext(
      TopologyTupleFor<Algorithms...> topologies,
      std::tuple<typename Algorithms::View...> views
  )
      : topologies_(std::move(topologies)), views_(std::move(views)) {}

  // Keyed by the Policy type itself, via std::get on topologies_ -- a Step
  // function names its own ordering policy the same way it names its own
  // View type below (sim_state.template TopologyFor<TopologicalOrdering>()),
  // rather than having its Topology resolved and handed to it up front.
  template <topology::LayoutPolicyLike Policy>
  const domain::JointTopology& TopologyFor() const {
    return std::get<TopologySlot<Policy>>(topologies_).topology;
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
  // TopologyFor<Policy> above being keyed by the Policy type, so a Step
  // function can name its own View the same way it already names its own
  // Policy, rather than having its View resolved and handed to it up front.
  // Constrained to !AlgorithmLike<ViewT> purely to stay unambiguous against
  // the overload above -- a View type is never itself AlgorithmLike (it has
  // no ::Enum/::View/::Step), so every real call resolves to exactly one
  // overload.
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
  // looks up its own View and Topology from sim_context itself
  // (ViewFor<ViewT>()/TopologyFor<Policy>()), rather than either being
  // resolved and handed to it up front.
  template <typename Config>
  void Step(float dt, const Config& config) const {
    (detail::StepAlgorithm<Algorithms>(*this, config, dt), ...);
  }

 private:
  TopologyTupleFor<Algorithms...> topologies_;
  std::tuple<typename Algorithms::View...> views_;
};

// Nominal (not duck-typed) check that SimStateT is actually some
// SimContext<Algorithms...>. This concept used to duck-type-probe
// TopologyFor with a stand-in Policy the same way TraversalLike probes
// Apply/InitOp -- that worked only because TopologyFor was, at the time, a
// runtime std::unordered_map lookup that type-checked for any Policy at
// all. Now that both TopologyFor and ViewFor are std::get on real tuples,
// no single stand-in Policy/ViewT compiles against every possible
// SimContext<...> instantiation (not even SimContext<> itself, whose
// tuples are empty), so that style of probe is gone for good -- see the
// git history on this file if the old approach is ever worth revisiting.
//
// This still doesn't require a Step function to name the Algorithms...
// pack (it matches any pack via the partial specialization below), so it
// keeps the original decoupling ABAStep's own comment describes. It just
// checks something coarser: is this literally a SimContext, not merely
// something shaped like one. That still catches the class of mistake the
// old check would have -- the wrong object entirely (a typo, sim_config
// where sim.SimContext() was meant, ...) -- with a clean "constraint not
// satisfied" error at Step's own template parameter, rather than a deep
// cascading error from inside Step's body. It can't tell you *which*
// View/Policy is missing, though -- that's still only ever caught where
// ViewFor/TopologyFor are actually called, same as any other unconstrained
// member access.
template <typename T>
struct IsSimContext : std::false_type {};

template <AlgorithmLike... Algorithms>
struct IsSimContext<SimContext<Algorithms...>> : std::true_type {};

template <typename T>
concept SimContextLike = IsSimContext<T>::value;

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
