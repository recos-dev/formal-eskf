// SPDX-License-Identifier: BSD-3-Clause

#include <array>
#include <cstddef>
#ifdef NATIVE
#include <cassert>
#define ASSERT(CONDITION, MESSAGE) assert(CONDITION)
#else
#define ASSERT(CONDITION, MESSAGE) __ESBMC_assert(CONDITION, MESSAGE)
#endif
#ifndef MUTATION
#define MUTATION 0
#endif
#define CHECK(CONDITION, MESSAGE) ASSERT(MUTATION ? !(CONDITION) : (CONDITION), MESSAGE)
#if FORMAL_ESKF_PROOF_BINARY64
using Scalar = double;
#else
using Scalar = float;
#endif
struct Rotation
{
    constexpr Rotation() = default;
    Scalar w{1};
    Scalar x{0};
};
struct Sample
{
    Rotation q{};
    unsigned tag{37};
};
struct Owner
{
    std::array<Sample, 64> samples{};
    Sample current{};
};
unsigned calls;
struct Counted
{
    unsigned id = ++calls;
};
struct Constructed
{
    unsigned id;
    Constructed() : id(++calls) {}
};
void verify_default()
{
    Owner owner;
    bool valid = owner.current.q.w == Scalar{1} && owner.current.tag == 37;
    for (std::size_t i = 0; i < 64; ++i)
    {
        valid = valid && owner.samples[i].q.w == Scalar{1} && owner.samples[i].q.x == Scalar{0} &&
                owner.samples[i].tag == 37;
    }
    CHECK(valid, "Array filler default member values");
}
void verify_partial()
{
    std::array<Sample, 4> samples{{Sample{{}, 91}}};
    bool valid = samples[0].tag == 91 && samples[0].q.w == Scalar{1};
    for (std::size_t i = 1; i < 4; ++i)
    {
        valid = valid && samples[i].tag == 37 && samples[i].q.w == Scalar{1};
    }
    CHECK(valid, "Array filler preserves explicit prefix and initializes suffix");
}
void verify_nested()
{
    std::array<std::array<Sample, 3>, 2> samples{};
    bool valid = true;
    for (std::size_t i = 0; i < 2; ++i)
    {
        for (std::size_t j = 0; j < 3; ++j)
        {
            valid = valid && samples[i][j].q.w == Scalar{1} && samples[i][j].tag == 37;
        }
    }
    CHECK(valid, "Nested array filler initializes every record");
}
void verify_effects()
{
    calls = 0;
    std::array<Counted, 4> samples{};
    CHECK(calls == 4 && samples[0].id == 1 && samples[1].id == 2 && samples[2].id == 3 && samples[3].id == 4,
          "Each default member initializer executes separately and in order");
}
void verify_partial_effects()
{
    calls = 0;
    std::array<Counted, 4> samples{{{91}}};
    CHECK(calls == 3 && samples[0].id == 91 && samples[1].id == 1 && samples[2].id == 2 && samples[3].id == 3,
          "Only omitted elements execute the default member initializer");
}
void verify_constructor_effects()
{
    calls = 0;
    std::array<Constructed, 4> samples{};
    CHECK(calls == 4 && samples[0].id == 1 && samples[1].id == 2 && samples[2].id == 3 && samples[3].id == 4,
          "Each omitted element calls a distinct default constructor");
}
void verify_empty()
{
    calls = 0;
    std::array<Constructed, 0> samples{};
    CHECK(calls == 0 && samples.size() == 0, "Empty array invokes no element constructors");
}
void verify_builtin()
{
    Sample samples[4]{};
    int integers[4]{9};
    CHECK(samples[0].tag == 37 && samples[3].q.w == Scalar{1} && integers[0] == 9 && integers[1] == 0 &&
              integers[2] == 0 && integers[3] == 0,
          "Builtin record and scalar arrays retain their initialization semantics");
}
unsigned counter;
struct Record
{
    unsigned value = ++counter;
};
void verify_explicit_order()
{
    counter = 0;
    std::array<Record, 4> values{{{++counter}, {++counter}}};
    CHECK(counter == 4 && values[0].value == 1 && values[1].value == 2 && values[2].value == 3 && values[3].value == 4,
          "Explicit prefix and omitted suffix preserve evaluation order");
}
void verify_scalar_order()
{
    counter = 0;
    int const values[4] = {static_cast<int>(++counter), static_cast<int>(++counter)};
    CHECK(counter == 2 && values[0] == 1 && values[1] == 2 && values[2] == 0 && values[3] == 0,
          "Const scalar arrays preserve explicit order and zero filler");
}
void verify_nested_partial()
{
    std::array<std::array<int, 3>, 2> const values{{{{1, 2}}, {{3}}}};
    CHECK(values[0][0] == 1 && values[0][1] == 2 && values[0][2] == 0 && values[1][0] == 3 && values[1][1] == 0 &&
              values[1][2] == 0,
          "Nested partial scalar arrays preserve all values");
}
void verify_initialization()
{
    verify_default();
    verify_partial();
    verify_nested();
    verify_effects();
    verify_partial_effects();
    verify_constructor_effects();
    verify_empty();
    verify_builtin();
    verify_explicit_order();
    verify_scalar_order();
    verify_nested_partial();
}
#ifdef NATIVE
int main()
{
#ifdef ENTRY
    ENTRY();
#else
    verify_initialization();
#endif
}
#endif
