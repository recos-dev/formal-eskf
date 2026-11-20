// SPDX-License-Identifier: BSD-3-Clause

#ifdef NATIVE
#include <assert.h>
#define CHECK(C, M) assert(C)
#else
#define CHECK(C, M) __ESBMC_assert(C, M)
#endif
struct Record
{
    int a;
    int b;
};
void verify_c_arrays(void)
{
    struct Record records[3] = {{1, 2}};
    int indexed[5] = {[3] = 7};
    int nested[2][3] = {{1}, {2, 3}};
    CHECK(records[0].a == 1 && records[0].b == 2 && records[1].a == 0 && records[2].b == 0,
          "C record arrays retain zero-fill semantics");
    CHECK(indexed[0] == 0 && indexed[1] == 0 && indexed[2] == 0 && indexed[3] == 7 && indexed[4] == 0,
          "C designated array initializers retain indices and zero-fill");
    CHECK(nested[0][0] == 1 && nested[0][1] == 0 && nested[0][2] == 0 && nested[1][0] == 2 && nested[1][1] == 3 &&
              nested[1][2] == 0,
          "C nested arrays retain explicit values and zero-fill");
}
int main(void)
{
    verify_c_arrays();
    return 0;
}
