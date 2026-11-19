// SPDX-License-Identifier: BSD-3-Clause

// Tool qualification, not an ESKF requirement proof. Exercise address-of
// reference-returning calls, including argument side effects and writes.
#include <cstddef>
extern void __ESBMC_assume(bool);
extern void __ESBMC_assert(bool, char const *);

struct Object
{
    int data[64];
    unsigned index;
    unsigned calls;
    int * pointer;
    int & get()
    {
        ++calls;
        return data[index];
    }
    int const & get_const() const { return data[index]; }
    int * view(bool visible) { return visible ? &get() : nullptr; }
    int * view_const(bool visible) { return visible ? const_cast<int *>(&get_const()) : nullptr; }
    int *& pointer_ref() { return pointer; }
    int (&array_ref())[64] { return data; }
};

void verify_reference_identity(Object value, bool visible)
{
    __ESBMC_assume(value.index < 64);
    value.calls = 0;
    auto * p = value.view(visible);
    __ESBMC_assert(p == (visible ? &value.data[value.index] : nullptr), "identity");
    __ESBMC_assert(value.calls == (visible ? 1U : 0U), "single evaluation");
    __ESBMC_assert(value.view_const(visible) == p, "const identity");
}

void verify_reference_write(Object value, int replacement)
{
    __ESBMC_assume(value.index < 64);
    value.calls = 0;
    int * p = &value.get();
    *p = replacement;
    __ESBMC_assert(value.data[value.index] == replacement && value.calls == 1, "write through reference");
    int & named = value.get();
    __ESBMC_assert(&named == &value.data[value.index] && value.calls == 2, "named reference");
}

void verify_reference_pointer(Object value)
{
    value.pointer = &value.data[0];
    int ** p = &value.pointer_ref();
    __ESBMC_assert(p == &value.pointer, "pointer reference address");
    *p = &value.data[63];
    __ESBMC_assert(value.pointer == &value.data[63], "pointer reference write");
}

void verify_reference_array(Object value)
{
    auto * p = &value.array_ref();
    __ESBMC_assert(p == &value.data, "array reference address");
    (*p)[63] = 42;
    __ESBMC_assert(value.data[63] == 42, "array reference write");
}

int function(int x) { return x; }
using Function = int(int);
Function & function_ref() { return function; }
void verify_reference_function()
{
    auto * p = &function_ref();
    __ESBMC_assert(p == &function && p(42) == 42, "function reference address");
}

void verify_negative_identity(Object value)
{
    __ESBMC_assume(value.index < 64);
    value.calls = 0;
    __ESBMC_assert(&value.get() == &value.data[(value.index + 1U) % 64U], "negative identity");
}
void verify_negative_write(Object value)
{
    value.index = 0;
    value.calls = 0;
    value.data[1] = 0;
    *(&value.get()) = 42;
    __ESBMC_assert(value.data[1] == 42, "negative write");
}
void verify_negative_calls(Object value)
{
    __ESBMC_assume(value.index < 64);
    value.calls = 0;
    auto * p = &value.get();
    (void)p;
    __ESBMC_assert(value.calls == 0, "negative calls");
}
