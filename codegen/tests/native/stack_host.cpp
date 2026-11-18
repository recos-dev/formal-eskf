/*
 * formal-eskf is freely redistributable under the BSD 3-Clause License.
 * See the file "LICENSE" for information on usage and redistribution of this
 * file.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <ucontext.h>
#include <sys/mman.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstddef>
extern "C" int setup(unsigned);
extern "C" int call_predict();
extern "C" int call_kernel();
static ucontext_t caller, callee;
static unsigned char * arena;
static unsigned operation;
static int status;
constexpr std::size_t size = 1024U * 1024U;
static void invoke() { status = operation == 0U ? 0 : (operation == 1U ? call_kernel() : call_predict()); }
int main()
{
    arena =
        static_cast<unsigned char *>(mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0));
    if (arena == MAP_FAILED)
    {
        return 1;
    }
    for (operation = 0U; operation < 3U; ++operation)
    {
        std::size_t highest = 0U;
        for (unsigned seed = 0U; seed < 16U; ++seed)
        {
            if (setup(seed))
            {
                return 2;
            }
            // Warm libm/loader before the measurement; no I/O on the alternate stack.
            if (call_predict() || call_kernel())
            {
                return 3;
            }
            std::memset(arena, 0xa5, size);
            if (getcontext(&callee))
            {
                return 4;
            }
            callee.uc_link = &caller;
            callee.uc_stack.ss_sp = arena;
            callee.uc_stack.ss_size = size;
            makecontext(&callee, invoke, 0);
            if (swapcontext(&caller, &callee))
            {
                return 4;
            }
            if (status)
            {
                return 5;
            }
            std::size_t untouched = 0U;
            while (untouched < size && arena[untouched] == 0xa5)
            {
                ++untouched;
            }
            if (size - untouched > highest)
            {
                highest = size - untouched;
            }
        }
        std::printf("{\"operation\":%u,\"observed_stack_write_bytes\":%zu}\n", operation, highest);
    }
    munmap(arena, size);
}
