
/*
 * fbs/power.h — FinalBuildSystems power / resource network library.
 * C99, engine independent, no libm, no allocation after fbs_power_create.
 *
 * Model: an undirected graph of nodes joined by edges. Every maximal connected
 * component is a network. Each node carries integer per-tick production,
 * per-tick demand, storage capacity and stored energy; a node may be any
 * combination (a pure producer has demand 0, a battery has production 0 and
 * capacity > 0). One tick balances every network independently and exactly:
 *
 *     produced + discharged  ==  consumed + charged + wasted (per network, per tick)
 *
 * Units are caller-defined integers ("energy units per tick"). Nothing is
 * floating point anywhere in the library, so a tick is bit-identical on every
 * platform and in WASM.
 * Out of scope for 0.1.0: geometry/placement, per-edge capacity or loss,
 * power channels, callbacks during a tick, a resource/build ledger.
 * Errors leave outputs untouched except FBS_POWER_E_TRUNCATED (required count
 * written); no mutator changes state on an error.
 */
#ifndef FBS_POWER_H
#define FBS_POWER_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FBS_POWER_VERSION_MAJOR 0
#define FBS_POWER_VERSION_MINOR 1
#define FBS_POWER_VERSION_PATCH 0
#define FBS_POWER_VERSION 100   /* major*10000 + minor*100 + patch */

/* Status codes. Negative values are errors. */
typedef enum fbs_power_status {
  FBS_POWER_OK = 0,
  FBS_POWER_E_INVALID   = -1, /* NULL, malformed handle, bad enum, a == b edge */
  FBS_POWER_E_RANGE     = -2, /* negative rate or capacity, stored outside [0, capacity], config out of range */
  FBS_POWER_E_FULL      = -3, /* a config capacity is exhausted (nodes, edges, networks) */
  FBS_POWER_E_MEMORY    = -4, /* allocator returned NULL */
  FBS_POWER_E_STATE     = -5, /* reserved for lifecycle misuse */
  FBS_POWER_E_NOT_FOUND = -6, /* node/edge/network handle does not exist or was freed (stale generation) */
  FBS_POWER_E_EXISTS    = -7, /* duplicate edge between the same pair */
  FBS_POWER_E_OVERFLOW  = -8, /* an accumulator would exceed int64 range; the tick or mutation is refused */
  FBS_POWER_E_SCHEMA    = -9, /* blob magic/version/length/crc/consistency mismatch */
  FBS_POWER_E_TRUNCATED = -10 /* output too small; *out_count = required count */
} fbs_power_status;

const char *fbs_power_status_name(int status); /* never NULL */

/* ------------------------------------------------------------------------ */
/* Handles                                                                   */
/* ------------------------------------------------------------------------ */
/* Handles are index + generation packed into 32 bits (20-bit index, 12-bit
 * generation), so a freed slot that is reused is not mistaken for the old
 * occupant: a stale handle yields FBS_POWER_E_NOT_FOUND. The generation wraps
 * after 4095 reuses of one slot, so a handle older than that many recycles of
 * its slot can alias; hosts that recycle nodes that heavily must drop stale
 * handles promptly. FBS_POWER_INVALID is the null handle. */
typedef uint32_t fbs_power_node;
typedef uint32_t fbs_power_edge;
typedef uint32_t fbs_power_network;
#define FBS_POWER_INVALID ((uint32_t)0)

/* ------------------------------------------------------------------------ */
/* Configuration                                                             */
/* ------------------------------------------------------------------------ */

typedef struct fbs_power_allocator {
  void *(*alloc)(void *user, size_t bytes);
  void  (*free)(void *user, void *ptr);
  void *user;
} fbs_power_allocator;

/* Scarcity policies use exact integer allocations. Results are deterministic
 * for fixed node IDs, rates and priorities, independent of edge insertion
 * order. Reordering node creation can change node-index tie breaks. */
typedef enum fbs_power_policy {
  /* Each consumer receives floor(demand * supply / total_demand); the
   * remainder (total_supply - sum of floors) is handed out one unit at a time
   * by the largest-remainder rule, ties broken by ascending node index. Sums
   * to exactly the available supply. */
  FBS_POWER_POLICY_PROPORTIONAL = 0,
  /* Consumers are served in full in ascending (priority, node index) order
   * until supply runs out; the first unsatisfied consumer takes the partial
   * remainder. */
  FBS_POWER_POLICY_PRIORITY = 1,
  /* All consumers get their full demand or every consumer gets zero
   * (technic's rule). */
  FBS_POWER_POLICY_ALL_OR_NOTHING = 2
} fbs_power_policy;

typedef struct fbs_power_config {
  uint32_t max_nodes;      /* 1..(1<<20)-1 */
  uint32_t max_edges;      /* 0..(1<<20)-1 */
  uint32_t max_networks;   /* 1..(1<<20)-1; config_default gives 256, not derived from max_nodes */
  fbs_power_policy policy; /* default FBS_POWER_POLICY_PROPORTIONAL */
} fbs_power_config;

/* 256 nodes, 512 edges, 256 networks, PROPORTIONAL. */
fbs_power_config fbs_power_config_default(void);

typedef struct fbs_power_context fbs_power_context;

/* One allocation, sized from cfg. alloc may be NULL for the libc allocator. */
fbs_power_status fbs_power_create(const fbs_power_config *cfg,
                                  const fbs_power_allocator *alloc,
                                  fbs_power_context **out);
void fbs_power_destroy(fbs_power_context *ctx);
/* Removes every node, edge and network; generations keep counting. */
void fbs_power_clear(fbs_power_context *ctx);
size_t fbs_power_memory(const fbs_power_context *ctx);
uint32_t fbs_power_node_count(const fbs_power_context *ctx);
uint32_t fbs_power_edge_count(const fbs_power_context *ctx);
unsigned fbs_power_version(void);

/* ------------------------------------------------------------------------ */
/* Topology                                                                  */
/* ------------------------------------------------------------------------ */

typedef struct fbs_power_node_desc {
  int64_t  production;   /* units produced per tick, >= 0 */
  int64_t  demand;       /* units consumed per tick when fully supplied, >= 0 */
  int64_t  capacity;     /* storage capacity in units, >= 0 (0 = not a battery) */
  int64_t  stored;       /* initial stored energy, 0..capacity */
  uint32_t priority;     /* lower is served first under FBS_POWER_POLICY_PRIORITY */
  uint64_t tag;          /* opaque host id, never interpreted */
} fbs_power_node_desc;

fbs_power_status fbs_power_node_add(fbs_power_context *ctx,
                                    const fbs_power_node_desc *desc,
                                    fbs_power_node *out);
/* Removes the node and every edge touching it, then re-splits the component
 * it belonged to. */
fbs_power_status fbs_power_node_remove(fbs_power_context *ctx, fbs_power_node n);
fbs_power_status fbs_power_node_get(const fbs_power_context *ctx, fbs_power_node n,
                                    fbs_power_node_desc *out);
/* Rates may change at runtime (a machine switching on); storage is changed
 * only by a tick or by fbs_power_node_set_stored. */
fbs_power_status fbs_power_node_set_rates(fbs_power_context *ctx, fbs_power_node n,
                                          int64_t production, int64_t demand);
fbs_power_status fbs_power_node_set_stored(fbs_power_context *ctx, fbs_power_node n,
                                           int64_t stored); /* E_RANGE outside [0, capacity]; never clamped */
fbs_power_status fbs_power_node_set_capacity(fbs_power_context *ctx, fbs_power_node n,
                                             int64_t capacity); /* E_RANGE if capacity < 0 or < stored */
fbs_power_status fbs_power_node_set_priority(fbs_power_context *ctx, fbs_power_node n, uint32_t priority);

/* Undirected; a == b is FBS_POWER_E_INVALID, a duplicate pair is
 * FBS_POWER_E_EXISTS. Merges the two networks (union by size). */
fbs_power_status fbs_power_edge_add(fbs_power_context *ctx,
                                    fbs_power_node a, fbs_power_node b,
                                    fbs_power_edge *out);
/* Removes the edge and re-splits: after this call every maximal connected
 * component is exactly one network. */
fbs_power_status fbs_power_edge_remove(fbs_power_context *ctx, fbs_power_edge e);
fbs_power_status fbs_power_edge_get(const fbs_power_context *ctx, fbs_power_edge e,
                                    fbs_power_node *out_a, fbs_power_node *out_b);
/* Edges in ascending edge index order; E_TRUNCATED with *out_count = total when cap is too small. */
fbs_power_status fbs_power_edge_list(const fbs_power_context *ctx, fbs_power_edge *out, size_t cap,
                                     size_t *out_count);
/* Nodes in ascending node index order; same truncation rule. */
fbs_power_status fbs_power_node_list(const fbs_power_context *ctx, fbs_power_node *out, size_t cap,
                                     size_t *out_count);

/* ------------------------------------------------------------------------ */
/* Networks                                                                  */
/* ------------------------------------------------------------------------ */

fbs_power_status fbs_power_node_network(const fbs_power_context *ctx,
                                        fbs_power_node n, fbs_power_network *out);
uint32_t fbs_power_network_count(const fbs_power_context *ctx);
/* Networks in ascending slot index order; always sets *out_count to the true
 * count and writes the ids only when they all fit (FBS_POWER_E_TRUNCATED
 * otherwise, nothing written). */
fbs_power_status fbs_power_network_list(const fbs_power_context *ctx,
                                        fbs_power_network *out, size_t cap,
                                        size_t *out_count);
/* Members in ascending node index order — a stable, testable membership
 * (same truncation rule). */
fbs_power_status fbs_power_network_members(const fbs_power_context *ctx,
                                           fbs_power_network net,
                                           fbs_power_node *out, size_t cap,
                                           size_t *out_count);

/* satisfaction_num/den is the reduced fraction consumed/demand when it fits
 * two uint32 (0/1 when demand is 0). Otherwise the numerator is
 * floor(consumed * UINT32_MAX / demand), the denominator is UINT32_MAX,
 * and the pair is reduced: a conservative approximation, never overstated. */
typedef struct fbs_power_network_stats {
  uint32_t node_count;
  int64_t  production;   /* sum of member production */
  int64_t  demand;       /* sum of member demand */
  int64_t  stored;       /* sum of member stored */
  int64_t  capacity;     /* sum of member capacity */
  /* Filled by the last tick: what actually happened. */
  int64_t  supplied;     /* production + discharged; includes any waste */
  int64_t  consumed;     /* sum of per-node allocations */
  int64_t  charged;      /* energy that entered storage */
  int64_t  discharged;   /* energy that left storage */
  int64_t  wasted;       /* production that could not be consumed or stored */
  uint32_t satisfaction_num, satisfaction_den; /* reduced fraction or conservative approximation as above */
} fbs_power_network_stats;

fbs_power_status fbs_power_network_stats_get(const fbs_power_context *ctx,
                                             fbs_power_network net,
                                             fbs_power_network_stats *out);

/* ------------------------------------------------------------------------ */
/* Tick                                                                      */
/* ------------------------------------------------------------------------ */

/* Advances every network by exactly one tick. Fixed order of operations,
 * documented so a host can reason about it and a test can assert it:
 *
 *   1. supply  = sum(production), demand = sum(demand)
 *   2. if supply >= demand: every consumer is fully served; the surplus charges
 *      batteries, distributed by free capacity with the largest-remainder rule
 *      (ascending node index breaks ties); any remainder is `wasted`.
 *   3. if supply < demand: batteries discharge, distributed by stored amount
 *      with the same remainder rule, up to the deficit; the total available
 *      supply is then allocated by ctx->policy. Discharge never exceeds what
 *      the policy actually consumes beyond production (an ALL_OR_NOTHING
 *      blackout drains nothing and wastes the production).
 *   4. stored is clamped to [0, capacity] by construction, never by a fix-up.
 *
 * Conservation holds exactly, in integers, for every network:
 *   production + discharged == consumed + charged + wasted
 *   sum(stored_after) - sum(stored_before) == charged - discharged
 */
fbs_power_status fbs_power_tick(fbs_power_context *ctx);
/* Ticks a single network only (others untouched). */
fbs_power_status fbs_power_tick_network(fbs_power_context *ctx, fbs_power_network net);

/* Per-node result of the last tick. */
fbs_power_status fbs_power_node_allocation(const fbs_power_context *ctx,
                                           fbs_power_node n, int64_t *out_allocated);

/* ------------------------------------------------------------------------ */
/* Serialization (byte layout described in src/power.c, SCHEMA)            */
/* ------------------------------------------------------------------------ */

size_t fbs_power_serialized_size(const fbs_power_context *ctx);
fbs_power_status fbs_power_serialize(const fbs_power_context *ctx, void *buf,
                                     size_t cap, size_t *out_len);
/* cfg NULL: capacities are the larger of the defaults and what the blob needs
 * (a blob's slot indices size the tables, so untrusted saves should be loaded
 * with an explicit cfg to bound the allocation); with cfg everything must fit
 * (E_FULL). Networks are rebuilt by the same BFS the runtime uses. */
fbs_power_status fbs_power_deserialize(const void *buf, size_t len,
                                       const fbs_power_config *cfg,
                                       const fbs_power_allocator *alloc,
                                       fbs_power_context **out);

#ifdef __cplusplus
}
#endif
#endif /* FBS_POWER_H */
