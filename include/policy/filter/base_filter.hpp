// The base every pair filter derives from, and its concept.

#pragma once

#include <cstddef>
#include <functional>
#include <span>
#include <string_view>
#include <type_traits>
#include <vector>

#include "domain/archetype/archetype.hpp"
#include "domain/topology/joint_topology.hpp"
#include "util/tmp.hpp"

namespace achilles::policy::filter {

// Which rows ever pair up, decided from their archetypes' own data:
// Key(archetype, joint) reads whatever one row's answer depends on, once
// per row, and Admits(a, b) is the pair test on two rows' keys.
template <typename F>
concept PairFilterLike =
    std::is_empty_v<F> && requires(const Archetype& archetype) {
      {
        F::Admits(F::Key(archetype, 0), F::Key(archetype, 0))
      } -> std::same_as<bool>;
    };

}  // namespace achilles::policy::filter
