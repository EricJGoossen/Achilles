#pragma once

#include <array>
#include <cstddef>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <xsimd/xsimd.hpp>

#include "engine/field_contract.hpp"
#include "engine/view/planar_view.hpp"

namespace achilles::test_support {

// PlanarView itself never allocates -- its constructor just wraps
// externally-provided, per-field base pointers (see
// engine/view/planar_view.hpp). Achilles' actual allocator, which is
// supposed to produce those pointers for real, hasn't been written yet.
// This is NOT a stand-in for it: it has none of a real allocator's
// responsibilities (pooling, growth, coordinating multiple views sharing
// memory, lifetime tracking beyond its own RAII) -- it exists only to
// hand a test one correctly-sized, correctly-(xsimd-batch-)aligned buffer
// per field, the same shape PlanarView::FieldStrideBytes computes
// internally, so PlanarView/OpInvoker/RunPass etc. can be exercised for
// real instead of only at the type level.
//
// -- Porting plan, once the real allocator exists --
//
// Every test that uses this fixture only ever calls two things on it:
// the constructor (EnumT + Assemblers... + num_instances) and
// MakeView()/NumInstances(). That surface is deliberately small so the
// port below can be close to mechanical:
//
// 1. Check the one hard requirement first: can the real allocator, given
//    EnumT and the Assemblers... pack (or the Traits template
//    ViewFactory builds them from), produce exactly what PlanarView's
//    constructor needs -- a std::array<std::byte*, kNumTypes> of
//    per-field base pointers, each field's block big enough for
//    kNumFields leaf-arrays of num_instances elements, aligned to
//    alignof(xsimd::batch<ScalarType>)? If the allocator instead hands
//    out one shared arena with a different offset scheme, PlanarView's
//    own FieldZero/FieldStrideBytes -- not just this fixture -- need to
//    change first; that's a prerequisite to this port, not part of it.
// 2. Reimplement this constructor's body to call the real allocator
//    instead of std::aligned_alloc/std::memset, keeping MakeView() and
//    NumInstances() as the only public surface -- every existing test
//    file that uses PlanarViewFixture (engine_view_planar_view.cpp,
//    engine_op_invoker.cpp, engine_algorithm_step.cpp, ...) then needs
//    zero changes. Prefer this adapter approach over rewriting every call
//    site to talk to the allocator directly; only do the latter once the
//    adapter has been proven to work and calling the allocator directly
//    is clearly simpler at the call site than going through this type.
// 3. Delete the manual std::aligned_alloc/std::memset/Buffer machinery
//    once step 2 is in and passing -- don't leave both paths around.
// 4. Re-run the full suite. Nothing should change behaviorally: every
//    existing test only depends on MakeView()/NumInstances(), so this
//    should be a no-observable-diff port, not a rewrite.
// 5. Add a dedicated test file for the real allocator's *own*
//    responsibilities that this fixture was explicitly never built to
//    cover -- pooling, growth/resizing, multiple views sharing memory,
//    lifetime/ownership edge cases. This fixture proved PlanarView works
//    given valid memory; it says nothing about whether the allocator
//    itself is correct.
template <engine::FieldEnumLike EnumT, engine::AssemblerLike... Assemblers>
class PlanarViewFixture {
  static constexpr std::size_t kNumTypes = sizeof...(Assemblers);
  using View = engine::view::PlanarView<EnumT, Assemblers...>;

  struct FreeDeleter {
    void operator()(std::byte* p) const { std::free(p); }
  };
  using Buffer = std::unique_ptr<std::byte[], FreeDeleter>;

 public:
  explicit PlanarViewFixture(std::size_t num_instances)
      : num_instances_(num_instances) {
    std::size_t i = 0;
    (
        [&] {
          std::size_t alignment =
              alignof(xsimd::batch<typename Assemblers::ScalarType>);
          std::size_t raw_bytes =
              num_instances_ * sizeof(typename Assemblers::ScalarType);
          std::size_t stride_bytes =
              (raw_bytes + alignment - 1) / alignment * alignment;
          // A field's buffer holds one leaf-array per child of its own
          // Assembler (e.g. 3 for Vector3Assembler: X, Y, Z), each
          // stride_bytes apart -- not just one array's worth. Matches
          // PlanarView::ReadChild/WriteChild indexing
          // (field0 + FieldOffset<Is>() * field_stride_bytes for each of
          // kNumFields children).
          std::size_t total_bytes = stride_bytes * Assemblers::kNumFields;
          buffers_[i] = Buffer(static_cast<std::byte*>(
              std::aligned_alloc(alignment, total_bytes)
          ));
          // aligned_alloc doesn't zero-initialize -- do it explicitly so
          // reading an index nothing has written to yet is deterministic
          // (zero) instead of whatever was in that memory before,
          // matching what tests reasonably expect from "fresh" storage.
          std::memset(buffers_[i].get(), 0, total_bytes);
          type_bases_[i] = buffers_[i].get();
          ++i;
        }(),
        ...
    );
  }

  View MakeView() const { return View(type_bases_, num_instances_); }
  std::size_t NumInstances() const { return num_instances_; }

 private:
  std::size_t num_instances_;
  std::array<Buffer, kNumTypes> buffers_;
  std::array<std::byte*, kNumTypes> type_bases_{};
};

// Picks PlanarViewFixture's Assemblers... pack back out of an
// already-resolved PlanarView type (e.g. ABAView = ViewFactory<PlanarView,
// ABAField, ABAFieldTraits>, algorithms/aba/aba_data.hpp) via partial
// specialization, instead of a caller hand-spelling every one of a large
// FieldEnum's fields' Assembler types again in the same order Traits
// already lists them -- for ABAField's 18 fields that would be as
// error-prone to keep in sync as it is tedious to write. Usage:
// `FixtureFor<ABAView>::Type fixture(num_instances);`.
template <typename View>
struct FixtureFor;

template <typename EnumT, typename... Assemblers>
struct FixtureFor<engine::view::PlanarView<EnumT, Assemblers...>> {
  using Type = PlanarViewFixture<EnumT, Assemblers...>;
};

}  // namespace achilles::test_support
