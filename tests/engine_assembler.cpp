#include <gtest/gtest.h>

#include <cstddef>
#include <new>
#include <tuple>
#include <xsimd/xsimd.hpp>

#include "domain/math/vector3.hpp"
#include "domain/spatial/inertia.hpp"
#include "engine/assembler.hpp"

using achilles::domain::math::Vector3;
using achilles::domain::math::Vector3Assembler;
using achilles::domain::spatial::Inertia;
using achilles::domain::spatial::InertiaAssembler;
using achilles::engine::AssemblableLike;
using achilles::engine::Assembler;
using achilles::engine::AssemblerLike;
using achilles::engine::RepeatTypes;
using achilles::engine::Templated;

namespace {

// Owns a runtime-sized, over-aligned raw buffer (the alignment xsimd
// batches require) for a standalone Assembler test, strided the same way
// Assembler::Read/Write expect (FieldOffset<I>() * stride bytes between
// consecutive children) -- the same math View uses per field, just for a
// single Assembler in isolation, with no View/allocator involved.
// A hand-rolled RAII owner instead of unique_ptr<std::byte[]> because the
// buffer needs an alignment beyond what plain new/delete guarantee --
// the [] form's default deleter can't carry that alignment through to
// delete, and std::aligned_alloc/free (the C answer to the same problem)
// is manual memory management, not RAII.
class Buffer {
 public:
  Buffer() = default;
  Buffer(std::size_t size, std::size_t alignment)
      : data_(static_cast<std::byte*>(
            ::operator new(size, std::align_val_t{alignment})
        )),
        alignment_(alignment) {}
  Buffer(const Buffer&) = delete;
  Buffer& operator=(const Buffer&) = delete;
  Buffer(Buffer&& other) noexcept
      : data_(other.data_), alignment_(other.alignment_) {
    other.data_ = nullptr;
  }
  Buffer& operator=(Buffer&& other) noexcept {
    if (this != &other) {
      Release();
      data_ = other.data_;
      alignment_ = other.alignment_;
      other.data_ = nullptr;
    }
    return *this;
  }
  ~Buffer() { Release(); }

  std::byte* Get() const { return data_; }

 private:
  void Release() {
    if (data_ != nullptr) {
      ::operator delete(data_, std::align_val_t{alignment_});
    }
  }

  std::byte* data_ = nullptr;
  std::size_t alignment_ = 0;
};

template <typename A>
Buffer MakeAssemblerBuffer(std::size_t num_instances) {
  std::size_t alignment = alignof(xsimd::batch<typename A::ScalarType>);
  std::size_t raw_bytes = num_instances * sizeof(typename A::ScalarType);
  std::size_t stride = (raw_bytes + alignment - 1) / alignment * alignment;
  return Buffer(stride * A::kNumFields, alignment);
}

template <typename A>
std::size_t StrideFor(std::size_t num_instances) {
  std::size_t alignment = alignof(xsimd::batch<typename A::ScalarType>);
  std::size_t raw_bytes = num_instances * sizeof(typename A::ScalarType);
  return (raw_bytes + alignment - 1) / alignment * alignment;
}

}  // namespace

// Read/Write round-trip through a real, single-leaf-group Assembler
// (Vector3Assembler), scalar and batched -- the same shape already
// exercised indirectly via every domain/math type's own
// static_assert(AssemblerLike<...>), checked here directly against a raw
// buffer instead.
TEST(AssemblerRoundTrip, ScalarLeafGroup) {
  using A = Vector3Assembler<float>;
  std::size_t n = 4;
  Buffer buf = MakeAssemblerBuffer<A>(n);
  std::size_t stride = StrideFor<A>(n);

  Vector3<float> value(1.0F, 2.0F, 3.0F);
  A::Write<float>(buf.Get(), stride, value);
  Vector3<float> read_back = A::Read<float>(buf.Get(), stride);

  EXPECT_TRUE(read_back.IsApprox(value));
}

TEST(AssemblerRoundTrip, BatchedLeafGroup) {
  using A = Vector3Assembler<float>;
  using Batch = xsimd::batch<float>;
  std::size_t n = Batch::size;
  Buffer buf = MakeAssemblerBuffer<A>(n);
  std::size_t stride = StrideFor<A>(n);

  Vector3<Batch> value(Batch(1.0F), Batch(2.0F), Batch(3.0F));
  A::Write<Batch>(buf.Get(), stride, value);
  Vector3<Batch> read_back = A::Read<Batch>(buf.Get(), stride);

  EXPECT_FLOAT_EQ(read_back.X().get(0), 1.0F);
  EXPECT_FLOAT_EQ(read_back.Y().get(0), 2.0F);
  EXPECT_FLOAT_EQ(read_back.Z().get(0), 3.0F);
}

// Read/Write round trip through a real composite Assembler
// (InertiaAssembler = Assembler<Inertia, T, T, Vector3Assembler<T>,
// RepeatTypes<6>>): a bare leaf (mass), a nested Assembler (h), and a
// RepeatTypes block (the 6 inertia components) all in one tree -- proves
// the flattening/recursion machinery (FlattenOne, ReadChild/WriteChild's
// leaf-vs-nested dispatch) against a real, already-shipping composite
// type instead of an invented one.
TEST(AssemblerRoundTrip, CompositeAssemblerWithNestedAndRepeatedChildren) {
  using A = InertiaAssembler<float>;
  std::size_t n = 4;
  Buffer buf = MakeAssemblerBuffer<A>(n);
  std::size_t stride = StrideFor<A>(n);

  Inertia<float> value(
      2.0F,
      Vector3<float>(0.1F, -0.2F, 0.3F),
      2.0F,
      3.0F,
      4.0F,
      0.0F,
      0.0F,
      0.0F
  );
  A::Write<float>(buf.Get(), stride, value);
  Inertia<float> read_back = A::Read<float>(buf.Get(), stride);

  EXPECT_TRUE(read_back.IsApprox(value));
}

// Zero<T>() is what an accumulating output starts from (View::LoadOutput at
// a BroadcastIndex): every leaf exactly zero, scalar and batched alike.
TEST(AssemblerZero, EveryLeafIsZeroScalarAndBatched) {
  using A = Vector3Assembler<float>;
  using Batch = xsimd::batch<float>;

  EXPECT_TRUE(A::Zero<float>().IsZero());
  Vector3<Batch> batched = A::Zero<Batch>();
  for (std::size_t lane = 0; lane < Batch::size; ++lane) {
    EXPECT_FLOAT_EQ(batched.X().get(lane), 0.0F);
    EXPECT_FLOAT_EQ(batched.Y().get(lane), 0.0F);
    EXPECT_FLOAT_EQ(batched.Z().get(lane), 0.0F);
  }
}

// Regression test: Zero() used to build every Target from zero children,
// which for Inertia meant its constructor's physical-validity assert firing
// on an all-zero body -- any reduced Inertia output (View::LoadOutput at a
// BroadcastIndex) would abort. A Target with its own Zero() now builds it
// that way.
TEST(AssemblerZero, TargetWithAValidatingConstructorUsesItsOwnZero) {
  using A = InertiaAssembler<float>;
  Inertia<float> zero = A::Zero<float>();

  EXPECT_FLOAT_EQ(zero.Mass(), 0.0F);
  EXPECT_TRUE(zero.H().IsZero());
  EXPECT_TRUE(zero.IsApprox(Inertia<float>::Zero()));
}


// Accumulate<float> is one plain += per leaf onto what the buffer holds.
TEST(AssemblerAccumulate, ScalarAddsOntoTheStoredValue) {
  using A = Vector3Assembler<float>;
  std::size_t n = 4;
  Buffer buf = MakeAssemblerBuffer<A>(n);
  std::size_t stride = StrideFor<A>(n);

  A::Write<float>(buf.Get(), stride, Vector3<float>(1.0F, 2.0F, 3.0F));
  A::Accumulate<float>(buf.Get(), stride, Vector3<float>(0.5F, -1.0F, 4.0F));

  EXPECT_TRUE(A::Read<float>(buf.Get(), stride)
                  .IsApprox(Vector3<float>(1.5F, 1.0F, 7.0F)));
}

// Accumulate<batch> folds every lane of each leaf into that leaf's one
// stored instance, and only that instance: instance 1 (the next element
// of each leaf's run) must be untouched.
TEST(AssemblerAccumulate, BatchSumsEveryLaneIntoTheOneInstance) {
  using A = Vector3Assembler<float>;
  using Batch = xsimd::batch<float>;
  std::size_t n = 2 * Batch::size;
  Buffer buf = MakeAssemblerBuffer<A>(n);
  std::size_t stride = StrideFor<A>(n);
  auto lanes = static_cast<float>(Batch::size);

  A::Write<float>(buf.Get(), stride, Vector3<float>(10.0F, 20.0F, 30.0F));
  A::Write<float>(
      buf.Get() + sizeof(float), stride, Vector3<float>(7.0F, 7.0F, 7.0F)
  );
  A::Accumulate<Batch>(
      buf.Get(), stride, Vector3<Batch>(Batch(1.0F), Batch(2.0F), Batch(-3.0F))
  );

  EXPECT_TRUE(A::Read<float>(buf.Get(), stride)
                  .IsApprox(Vector3<float>(
                      10.0F + lanes, 20.0F + (2.0F * lanes), 30.0F - (3.0F * lanes)
                  )));
  EXPECT_TRUE(A::Read<float>(buf.Get() + sizeof(float), stride)
                  .IsApprox(Vector3<float>(7.0F, 7.0F, 7.0F)));
}

// The nested-Assembler half of AccumulateChild's dispatch: every leaf of a
// composite, nested ones included, lands at its own field offset. Every
// lane holds the stored body itself, so the result is that body scaled by
// (1 + lanes) -- still physically valid for Inertia's constructor to read
// back, and still distinct per leaf.
TEST(AssemblerAccumulate, CompositeAssemblerAccumulatesEveryLeaf) {
  using A = InertiaAssembler<float>;
  using Batch = xsimd::batch<float>;
  std::size_t n = Batch::size;
  Buffer buf = MakeAssemblerBuffer<A>(n);
  std::size_t stride = StrideFor<A>(n);
  float scale = 1.0F + static_cast<float>(Batch::size);

  Inertia<float> stored(
      2.0F, Vector3<float>(0.1F, -0.2F, 0.3F), 2.0F, 3.0F, 4.0F, 0.1F, 0.2F, 0.3F
  );
  A::Write<float>(buf.Get(), stride, stored);
  A::Accumulate<Batch>(
      buf.Get(),
      stride,
      Inertia<Batch>(
          Batch(2.0F),
          Vector3<Batch>(Batch(0.1F), Batch(-0.2F), Batch(0.3F)),
          Batch(2.0F),
          Batch(3.0F),
          Batch(4.0F),
          Batch(0.1F),
          Batch(0.2F),
          Batch(0.3F)
      )
  );

  Inertia<float> read_back = A::Read<float>(buf.Get(), stride);
  EXPECT_FLOAT_EQ(read_back.Mass(), 2.0F * scale);
  EXPECT_TRUE(read_back.H().IsApprox(Vector3<float>(0.1F, -0.2F, 0.3F) * scale));
  EXPECT_FLOAT_EQ(read_back.Ixx(), 2.0F * scale);
  EXPECT_FLOAT_EQ(read_back.Iyy(), 3.0F * scale);
  EXPECT_FLOAT_EQ(read_back.Izz(), 4.0F * scale);
  EXPECT_FLOAT_EQ(read_back.Ixy(), 0.1F * scale);
  EXPECT_FLOAT_EQ(read_back.Ixz(), 0.2F * scale);
  EXPECT_FLOAT_EQ(read_back.Iyz(), 0.3F * scale);
}

TEST(AssemblerKNumFields, CountsLeavesNotChildren) {
  // Vector3Assembler: 3 leaves from one RepeatTypes<3>.
  EXPECT_EQ(Vector3Assembler<float>::kNumFields, 3U);
  // InertiaAssembler: 1 (mass) + 3 (h, a nested Vector3Assembler) + 6
  // (RepeatTypes<6>) = 10 -- kNumFields counts flattened leaves, not
  // Children pack entries (which number only 3: T, Vector3Assembler<T>,
  // RepeatTypes<6>).
  EXPECT_EQ(InertiaAssembler<float>::kNumFields, 10U);
}

namespace {

// RepeatTypes<N, ChildType> (the explicit-ChildType form, as opposed to
// the bare RepeatTypes<N> every production Assembler alias actually
// uses -- see domain/math/*.hpp and domain/spatial/*.hpp, all of which
// pass just RepeatTypes<N>) is currently unused anywhere in production.
// Exercised directly here since nothing else compiles it: N copies of a
// *nested Assembler*, not of a scalar. (A bare differently-scalar-typed
// leaf, e.g. RepeatTypes<2, std::int32_t> under a T=float Assembler,
// isn't actually constructible -- AllMatchScalar requires every leaf's
// own type equal T exactly, so ChildType has to either be T itself or an
// Assembler whose own ScalarType is T.)
template <typename T>
struct TwoVector3s {
  using ScalarType = T;
  Vector3<T> a;
  Vector3<T> b;

  TwoVector3s() : a(), b() {}
  TwoVector3s(Vector3<T> a_in, Vector3<T> b_in) : a(a_in), b(b_in) {}

  std::tuple<Vector3<T>, Vector3<T>> ToTuple() const { return {a, b}; }

  // SeedableLike (engine/assembler.hpp) requires this of anything
  // AssemblerLike reads/writes; both members are plain Vector3s, whose own
  // Zero() is already inert, so there's no degenerate-value concern like
  // Quaternion::Zero() to work around here.
  static TwoVector3s PaddingSeed() {
    return TwoVector3s(Vector3<T>::Zero(), Vector3<T>::Zero());
  }
};

using TwoVector3sAssembler =
    Assembler<TwoVector3s, float, RepeatTypes<2, Vector3Assembler<float>>>;
static_assert(AssemblerLike<TwoVector3sAssembler>);

}  // namespace

// The fallback for a Target with no Zero() of its own: built from each
// child's zero, recursing into the nested Vector3Assemblers -- TwoVector3s
// declares no Zero().
TEST(AssemblerZero, TargetWithoutItsOwnZeroIsBuiltFromZeroChildren) {
  TwoVector3s<float> zero = TwoVector3sAssembler::Zero<float>();
  EXPECT_TRUE(zero.a.IsZero());
  EXPECT_TRUE(zero.b.IsZero());
}

TEST(AssemblerRepeatTypesExplicitChildType, RoundTrips) {
  using A = TwoVector3sAssembler;
  std::size_t n = 4;
  Buffer buf = MakeAssemblerBuffer<A>(n);
  std::size_t stride = StrideFor<A>(n);

  TwoVector3s<float> value(
      Vector3<float>(1.0F, 2.0F, 3.0F), Vector3<float>(4.0F, 5.0F, 6.0F)
  );
  A::Write<float>(buf.Get(), stride, value);
  TwoVector3s<float> read_back = A::Read<float>(buf.Get(), stride);

  EXPECT_TRUE(read_back.a.IsApprox(value.a));
  EXPECT_TRUE(read_back.b.IsApprox(value.b));
}

namespace {

// Templated<TT> (wrapping a template-template Children entry so it picks
// up the enclosing Assembler's own T automatically) is, like
// RepeatTypes<N, ChildType> above, unused anywhere in production: every
// composite Assembler that nests another Assembler (InertiaAssembler
// nesting Vector3Assembler<T>, see domain/spatial/inertia.hpp) spells the
// nested Assembler's T out explicitly instead of writing
// Templated<Vector3Assembler>. Builds the InertiaAssembler-equivalent
// shape (a leaf plus a nested Vector3-shaped Assembler) via Templated<>
// instead, to prove the documented capability actually works.
template <typename T>
struct LeafAndVector {
  using ScalarType = T;
  T scale;
  Vector3<T> offset;

  LeafAndVector() : scale(), offset() {}
  LeafAndVector(T scale_in, Vector3<T> offset_in)
      : scale(scale_in), offset(offset_in) {}

  std::tuple<T, Vector3<T>> ToTuple() const { return {scale, offset}; }

  // See TwoVector3s::PaddingSeed above -- same reasoning, a zero scale and
  // a zero offset are both inert.
  static LeafAndVector PaddingSeed() {
    return LeafAndVector(T{0}, Vector3<T>::Zero());
  }
};

using LeafAndVectorAssembler =
    Assembler<LeafAndVector, float, float, Templated<Vector3Assembler>>;
static_assert(AssemblerLike<LeafAndVectorAssembler>);

}  // namespace

TEST(AssemblerTemplatedMarker, RoundTrips) {
  using A = LeafAndVectorAssembler;
  std::size_t n = 4;
  Buffer buf = MakeAssemblerBuffer<A>(n);
  std::size_t stride = StrideFor<A>(n);

  LeafAndVector<float> value(5.0F, Vector3<float>(1.0F, 2.0F, 3.0F));
  A::Write<float>(buf.Get(), stride, value);
  LeafAndVector<float> read_back = A::Read<float>(buf.Get(), stride);

  EXPECT_FLOAT_EQ(read_back.scale, 5.0F);
  EXPECT_TRUE(read_back.offset.IsApprox(value.offset));
}

namespace {

// A ToTuple() with the wrong shape -- here, an extra unused field folded
// into the tuple -- must be rejected by AssemblableLike (and therefore
// AssemblerLike) even though every individual Read/Write call still
// type-checks; see the comment on AssemblableLike in assembler.hpp for
// why that's a separate check from "does a call to Write resolve".
template <typename T>
struct WrongShapeTarget {
  using ScalarType = T;
  T x, y, z, unused;

  WrongShapeTarget() : x(), y(), z(), unused() {}
  WrongShapeTarget(T x_in, T y_in, T z_in)
      : x(x_in), y(y_in), z(z_in), unused() {}

  std::tuple<T, T, T, T> ToTuple() const { return {x, y, z, unused}; }
};

using WrongShapeAssembler = Assembler<WrongShapeTarget, float, RepeatTypes<3>>;
static_assert(!AssemblableLike<WrongShapeAssembler>);
static_assert(!AssemblerLike<WrongShapeAssembler>);

}  // namespace

TEST(AssemblerLikeRejection, WrongToTupleShapeFailsAssemblableLike) {
  // Compile-time-only: the static_asserts above are the test. Nothing to
  // run.
  SUCCEED();
}
