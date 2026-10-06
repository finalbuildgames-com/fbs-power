# fbs-power

Power and resource networks for games in C99: connect nodes, tick, and get exact integer allocations per network.

## What it does

You build an undirected graph of nodes and edges. Each node has integer per-tick `production`, per-tick `demand`, storage `capacity` and `stored` energy, so one node can be a generator, a consumer, a battery or any mix. Every connected component is a network, and the library keeps networks up to date for you:

- `fbs_power_node_add` / `fbs_power_edge_add` create nodes and wires. Adding an edge merges two networks.
- `fbs_power_edge_remove` / `fbs_power_node_remove` split a network into as many networks as there are components left.
- `fbs_power_tick` advances every network by one tick (`fbs_power_tick_network` advances one). Surplus charges batteries by free capacity and the rest is reported as `wasted`. A deficit is covered by discharging batteries, then the available supply is shared by the context's policy:
  - `FBS_POWER_POLICY_PROPORTIONAL`: each consumer gets its share of demand, the leftover units go out by largest remainder.
  - `FBS_POWER_POLICY_PRIORITY`: consumers are filled in full in ascending `priority` order, the first unsatisfied one gets the partial remainder.
  - `FBS_POWER_POLICY_ALL_OR_NOTHING`: everyone gets full demand or nobody gets anything, and a blackout drains no battery.
- `fbs_power_node_allocation` returns what a node received last tick. `fbs_power_network_stats_get` returns sums plus last-tick `supplied`, `consumed`, `charged`, `discharged`, `wasted` and a `satisfaction_num/satisfaction_den` fraction.
- `fbs_power_network_members`, `fbs_power_node_list`, `fbs_power_edge_list` and `fbs_power_network_list` return handles in ascending index order.
- `fbs_power_serialize` / `fbs_power_deserialize` save and load the graph as a little-endian blob with a CRC-32. Node and edge handles still resolve after a load.

Every tick holds `production + discharged == consumed + charged + wasted` per network, exactly, in integers.

## When to use it

- Base-building, factory or colony games where machines share power (or water, fuel, any per-tick quantity) through connected wiring.
- Games that need lockstep or replay determinism: there is no floating point anywhere in the library.
- Hosts that want stable handles to keep in their own entity data (`tag` is an opaque `uint64_t` for your id).

## When not to use it

- You need per-edge capacity, line loss, geometry or placement rules, or several resource channels on one graph. The header lists these as out of scope for 0.1.0. Use one context per resource type.
- You need fractional quantities. All rates and storage are `int64_t` units you define.
- You need more than (1<<20)-1 nodes, edges or networks per context. Capacities are fixed at `fbs_power_create`, and every live node uses a network slot (a lone node is a network of one), so `max_networks` also limits how many unconnected nodes you can have.
- You want different policies per network. The policy is set once per context in `fbs_power_config`.
- You want callbacks during a tick, or parallel ticking. A tick runs on the calling thread and the context takes no locks.
- Handles use a 12-bit generation, so a handle can alias after one slot is reused 4095 times. Drop stale handles promptly if you recycle nodes heavily.
- Removing an edge or node relabels the affected network with a breadth-first search, so the cost grows with that network's size.

## Example

A generator (10 per tick), a lamp (wants 25) and a battery holding 60 on one network:

```c
#include <fbs/power.h>
#include <stdio.h>

#define TRY(call) do { st = (call); if (st != FBS_POWER_OK) goto fail; } while (0)

static fbs_power_status add(fbs_power_context *ctx, int64_t production, int64_t demand,
                            int64_t capacity, int64_t stored, fbs_power_node *out) {
  fbs_power_node_desc d;
  d.production = production;
  d.demand = demand;
  d.capacity = capacity;
  d.stored = stored;
  d.priority = 0;
  d.tag = 0;
  return fbs_power_node_add(ctx, &d, out);
}

int main(void) {
  fbs_power_context *ctx = NULL;
  fbs_power_node generator, lamp, battery;
  fbs_power_edge e;
  fbs_power_network net;
  fbs_power_network_stats s;
  fbs_power_status st = FBS_POWER_OK;
  int64_t got;
  int tick;

  TRY(fbs_power_create(NULL, NULL, &ctx)); /* default config, libc allocator */
  TRY(add(ctx, 10, 0, 0, 0, &generator));
  TRY(add(ctx, 0, 25, 0, 0, &lamp));
  TRY(add(ctx, 0, 0, 100, 60, &battery));
  TRY(fbs_power_edge_add(ctx, generator, lamp, &e));
  TRY(fbs_power_edge_add(ctx, lamp, battery, &e));
  TRY(fbs_power_node_network(ctx, generator, &net));
  for (tick = 1; tick <= 6; ++tick) {
    TRY(fbs_power_tick(ctx));
    TRY(fbs_power_node_allocation(ctx, lamp, &got));
    TRY(fbs_power_network_stats_get(ctx, net, &s));
    printf("tick %d: lamp %lld/25, battery %lld, satisfaction %u/%u\n", tick, (long long)got,
           (long long)s.stored, (unsigned)s.satisfaction_num, (unsigned)s.satisfaction_den);
  }
  fbs_power_destroy(ctx);
  return 0;
fail:
  fprintf(stderr, "power error: %s\n", fbs_power_status_name(st));
  fbs_power_destroy(ctx); /* NULL is a no-op */
  return 1;
}
```

The battery covers the 15-unit gap for four ticks (lamp 25/25, battery 45, 30, 15, 0), then the lamp gets 10/25 (satisfaction 2/5). The same network is the `{4,5,6}` case checked in `test_fixtures` in `tests/test_power.c`. Build it with `cc -std=c99 -Iinclude main.c src/power.c -o power_demo`, or link `fbs::power` from CMake.

## Build and test

Run from this repository's root. In addition to CMake and the compiler named
below, install the build tool selected by your generator (for example Make or
Ninja).

Requires CMake 3.16+ and a C99 compiler. No other dependency; the library uses only the C standard library (no libm, though the CMake target links `m` on non-MSVC toolchains). No third-party code is vendored.

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
cmake --build build --parallel 1
(cd build && ctest --output-on-failure)
```

CMake options: `FBS_BUILD_TESTS` and `FBS_BUILD_EXAMPLES` (both `ON`). This registers two tests: `power` runs `tests/test_power.c`, and `power_example` runs `fbs_power_example`, built from `examples/basic.c`, which creates and destroys a context and prints the API version.

`tests/test_power.c` is a self-contained test program (exit code is the failure count) covering:

- Merges, splits and node removal, with membership checked node by node, plus stale network, node and edge handles returning `FBS_POWER_E_NOT_FOUND`.
- Conservation and storage bounds on every network for 1000 ticks of a seeded 24-node graph that keeps changing, under each policy.
- Hand-computed allocations for all three policies and the largest-remainder rule, the same results across different node and edge insertion orders, and a blackout that drains nothing.
- Capacity exhaustion, int64 overflow and invalid arguments leaving state and outputs untouched, and exactly one allocation per context.
- Save/load round trips with byte-identical re-serialization. Every truncated blob and every single-byte corruption is rejected with `FBS_POWER_E_SCHEMA`. A 4000-node blob loads under a loose 2000 ms ceiling (the test prints the time).
- Golden fixtures in `tests/fixtures/power/` (`grid.bin`, `allocations.txt`). `fbs_test_power --write-fixtures` rewrites them.

Use it from another CMake project with `add_subdirectory` or FetchContent (no `find_package` config is installed):

```cmake
include(FetchContent)
FetchContent_Declare(fbs_power
  GIT_REPOSITORY https://github.com/finalbuildgames-com/fbs-power.git
  GIT_TAG <full-commit-sha>) # pin a reviewed commit
FetchContent_MakeAvailable(fbs_power)
target_link_libraries(your_target PRIVATE fbs::power)
```

Engine adapters: none. This repository ships the C library only.

## Build modes and installation

`BUILD_SHARED_LIBS=ON` builds a shared library; the default is static.
`FBS_BUILD_TESTS` and `BUILD_TESTING` together enable the core test.
`FBS_BUILD_EXAMPLES` controls `fbs_power_example`; its CTest entry also requires
`BUILD_TESTING`. For a library-only build, set `FBS_BUILD_TESTS=OFF` and
`FBS_BUILD_EXAMPLES=OFF`.

```sh
cmake --install build --prefix "$PWD/install"
```

Installation supplies [the public header](include/fbs/power.h), the library,
license notices and `FinalBuildPowerTargets.cmake` under
`${CMAKE_INSTALL_LIBDIR}/cmake/FinalBuildPower`. It supplies no package config or
version config, so `find_package(FinalBuildPower)` is unavailable. A consumer may
include the installed targets file explicitly and link `fbs::power`, or use
the source integration above. The [minimal program](examples/basic.c) and
[core tests](tests/test_power.c) show the implemented entry points.

## Design notes

- **Determinism.** Integer arithmetic only. Shares are computed with an exact 64x64 to 128-bit multiply and divide in portable C, and ties go to the lower node index. Results do not depend on edge insertion order. Changing node creation order can change index tie breaks.
- **Memory.** `fbs_power_create` makes one allocation sized from the config, and nothing is allocated after that, including during ticks (`test_allocator` checks this). Pass an `fbs_power_allocator` (`alloc`, `free`, `user`) or `NULL` for `malloc`/`free`. `fbs_power_memory` reports the block size. `fbs_power_clear` empties a context without freeing it.
- **Loading saves.** `fbs_power_deserialize` checks magic, version, length, CRC and every record before it allocates. With a `NULL` config, table sizes come from the blob, so load untrusted saves with an explicit config to cap the allocation. A blob is accepted only if its version equals `FBS_POWER_VERSION`. The policy is stored in the blob. Per-tick results are not saved, so stats and allocations read zero until the next tick.
- **Threading.** No globals and no static mutable state, so separate contexts are independent. A single context has no internal locking.
- **Errors.** Functions return `fbs_power_status`; negative values are errors and `fbs_power_status_name` gives a string. On error, outputs are left untouched except `FBS_POWER_E_TRUNCATED`, which writes the required count, and no mutator changes state. A tick that would overflow int64 is refused as a whole with `FBS_POWER_E_OVERFLOW`.
- **Versioning.** `FBS_POWER_VERSION` (major*10000 + minor*100 + patch, currently 100 for 0.1.0) and `fbs_power_version()`. The context is opaque, the API has no struct size fields, and the header has `extern "C"` guards for C++.

## License

MIT for Final Build Games' original code, see [LICENSE](LICENSE). This is an original C implementation: marketplace products were used only as specification references and none of their source or assets are included. No third-party runtime code is included. See [THIRD_PARTY_NOTICES](THIRD_PARTY_NOTICES).
