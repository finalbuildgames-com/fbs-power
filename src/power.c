/*
 * src/power.c — FinalBuildSystems power / resource networks. Implements
 * include/fbs/power.h (v0.1.0, frozen; this file never edits it).
 *
 * Original work. Nothing is ported, wrapped or vendored: per
 * docs/decisions/power.md §10 the owned Fab pack ("Base Building and Power
 * Grids", Blueprint only, Fab Standard Licence) is a *specification input
 * only* — read for research, no line of it ships, and none of the GPL/AGPL/
 * LGPL games read as semantic references (Mindustry, /tg/station, technic)
 * contributed code. The pack's structural defects P1–P7 (§3 of the decision)
 * are fixed structurally here, not copied:
 *
 *   P1 non-conservation  -> both sides are summed with overflow detection and
 *                           `production + discharged == consumed + charged +
 *                           wasted` holds exactly, per network, per tick.
 *   P2 order dependence  -> every policy is a pure function of the member set,
 *                           with ties broken by ascending node index; nothing
 *                           depends on insertion order.
 *   P3 missed split      -> removal BFSes the affected component and emits as
 *                           many networks as there are components.
 *   P4 position-dependent membership -> membership is a per-network list kept
 *                           in ascending node index order at all times.
 *   P5 stale network ids -> networks are derived state with generational
 *                           handles; a freed network's handle is E_NOT_FOUND.
 *   P6 no-op disconnect  -> edge removal always unlinks both endpoints.
 *   P7 asymmetric ends   -> an edge is one record on two intrusive adjacency
 *                           lists; a duplicate pair is E_EXISTS.
 *
 * C99. Standard library only (no libm), no floating point, no globals, no
 * static mutable state, no allocation after fbs_power_create. One allocation
 * per context.
 *
 * ---------------------------------------------------------------------------
 * Internal contracts that the public header only implies
 * ---------------------------------------------------------------------------
 *
 * HANDLES. A handle is (generation << 20) | index: a 20-bit slot index in the
 * low bits and a 12-bit generation in the high bits, exactly as the header
 * states, with the same layout for nodes, edges and networks. A generation is
 * never 0, so FBS_POWER_INVALID (0) is not a representable live handle. Each
 * slot owns its generation, initialised to 1 and bumped (wrapping 0xFFF -> 1)
 * every time the slot is freed, so the next occupant of a slot never answers to
 * the previous occupant's handle. fbs_power_clear frees every slot and
 * therefore bumps every live generation ("generations keep counting"). With 12
 * bits a slot must be recycled 4095 times before a handle can alias; that is
 * the header's stated budget, not a defect of this file.
 *
 * SLOT ALLOCATION. Each table has a high-water mark and an intrusive LIFO free
 * list. A fresh context therefore issues indices 0, 1, 2, ... in order; after
 * frees, the most recently freed slot is reused first. Nothing in the contract
 * depends on which free slot is chosen, and serialization is written in
 * ascending slot index order, so the choice is not observable in a blob.
 *
 * NETWORKS. Every live node belongs to exactly one network (a lone node is a
 * network of size 1), so fbs_power_node_add can fail with FBS_POWER_E_FULL when
 * max_networks is exhausted. Each network owns a singly linked member list
 * threaded through the node records and kept in ascending node index order at
 * all times: node_add seeds a singleton, edge_add merges two ascending lists,
 * and a split partitions one ascending list into several, all order preserving.
 * That is what makes fbs_power_network_members a stable, testable membership.
 *
 * MERGE. fbs_power_edge_add unions by size: the larger network survives and the
 * smaller one's members are relabelled; on a tie the lower network *slot index*
 * survives. The absorbed network's slot is freed, so its handle is immediately
 * stale.
 *
 * SPLIT. Removal never mutates before it knows the answer. A removal first
 * labels the components of the affected network with a BFS that skips the edge
 * (or node) being removed, checks that max_networks can hold the result, and
 * only then unlinks and rebuilds the member lists. So a refused removal changes
 * nothing.
 *   - fbs_power_edge_remove BFSes from the *lower-index endpoint*, so the
 *     component containing that endpoint keeps the original network id and
 *     generation; any other component becomes a new network with a fresh
 *     generation. Removing a non-bridging edge yields one component and changes
 *     no network at all.
 *   - fbs_power_node_remove BFSes from the lowest-index *surviving* member
 *     first, so that component keeps the id; further components are discovered
 *     by scanning the surviving member list in ascending order and each gets a
 *     new network. Removing an isolated node frees its network outright.
 *
 * TICK. The order of operations is the header's, with one refinement that the
 * header's own field comments force. In the deficit branch the header says
 * batteries discharge up to the deficit and the available supply is then
 * allocated by the policy. Under FBS_POWER_POLICY_ALL_OR_NOTHING the allocation
 * can be zero even though supply was available, and a literal reading would
 * then drain the batteries into `wasted` — contradicting `wasted`'s documented
 * meaning ("production that could not be consumed or stored") and draining
 * storage for no consumer. So the discharge is capped by what the policy
 * actually consumes: discharged = max(0, consumed - production). Under
 * PROPORTIONAL and PRIORITY the allocation always sums to exactly the available
 * supply, so discharged == min(deficit, total_stored) and the refinement is
 * invisible; it only bites the all-or-nothing blackout, where nothing moves at
 * all and wasted == production. Both conservation identities hold either way.
 *
 * A tick is refused as a whole. fbs_power_tick validates every network's sums
 * for int64 overflow before it mutates anything, so FBS_POWER_E_OVERFLOW leaves
 * the context byte-identical.
 *
 * The five "filled by the last tick" statistics and the satisfaction fraction
 * belong to the network record, so a network that survives a merge or keeps its
 * id through a split keeps the numbers from the last tick it actually ran,
 * while a network created by a split or a load starts at zero with a
 * satisfaction of 0/1. The sums the same struct reports (node_count,
 * production, demand, stored, capacity) are never cached: they are recomputed
 * from the members on every read, so they are always current.
 *
 * FBS_POWER_E_STATE is never returned by this file, and cannot be: the frozen
 * 0.1.0 contract has no callbacks, so there is no way to be inside a tick when
 * a mutator is called. The header already calls the code "reserved for
 * lifecycle misuse"; docs/decisions/power.md §6.2 item 6 describes the case it
 * is reserved for, which arrives only if callbacks ever do.
 *
 * LIST OUTPUTS. The header's opening paragraph is the house rule — "errors
 * leave outputs untouched except FBS_POWER_E_TRUNCATED (required count
 * written)" — and it is what the four list functions implement: on truncation
 * only *out_count is written and the caller's array is not touched. The
 * sentence on fbs_power_network_list that reads "writes min(cap, count) ids" is
 * a description of the success path, where min(cap, count) is count; a partial
 * prefix write would contradict both the opening paragraph and every other
 * module in this repository (fbs_inv_list_items, fbs_faction_edges, …).
 *
 * ARITHMETIC. Every apportionment is floor + largest remainder with ties broken
 * by ascending node index, and the products involved (weight * total) do not
 * fit in int64 in general. They are computed exactly in a portable 64x64->128
 * multiply followed by a 128/64 shift-subtract division (pwr_muldiv), never in
 * floating point and never with a compiler extension such as __int128.
 *
 * BLOB VALIDATION COST. A load is O(N + E log N) before it allocates and
 * O(V + E) after: endpoints are resolved by binary search over the node
 * records (which the schema requires to be in strictly ascending index order),
 * and the "no pair joined twice" rule is checked once the adjacency lists
 * exist by stamping each node's neighbours. Neither is a scan per edge, so a
 * four-thousand-node save loads in about a millisecond rather than seconds.
 *
 * SCHEMA. docs/decisions/power.md §6.1, magic "FBSPWR\0\0", version
 * FBS_POWER_VERSION, little-endian, field by field, no padding. Networks,
 * per-network tick statistics and per-node allocations are *not* stored: they
 * are derived, and the loader rebuilds networks with the same BFS the runtime
 * uses. The crc32 is the standard CRC-32/ISO-HDLC (reflected polynomial
 * 0xEDB88320, init/final 0xFFFFFFFF) computed bit by bit — no lookup table,
 * because a table would either be static mutable state or cost a rebuild per
 * call, and blobs here are small.
 */

#include "fbs/power.h"

#include <stdlib.h>
#include <string.h>

/* ------------------------------------------------------------------------- */
/* Limits and layout constants                                               */
/* ------------------------------------------------------------------------- */

#define PWR_NIL 0xFFFFFFFFu

#define PWR_INDEX_BITS 20
#define PWR_MAX_SLOTS ((uint32_t)1u << PWR_INDEX_BITS)          /* 1048576      */
#define PWR_INDEX_MASK (PWR_MAX_SLOTS - 1u)                     /* 0xFFFFF      */
#define PWR_MAX_CAPACITY (PWR_MAX_SLOTS - 1u)                   /* header range */
#define PWR_GEN_MIN 1u
#define PWR_GEN_MAX 0xFFFu

#define PWR_HEADER_BYTES 24u
#define PWR_NODE_BYTES 48u
#define PWR_EDGE_BYTES 12u
#define PWR_CRC_BYTES 4u

#define PWR_BLOCK_ALIGN 8u

/* Number of separately aligned regions carved out of the single allocation. */
#define PWR_REGIONS 10

static size_t pwr_align_up(size_t v) {
  return (v + (PWR_BLOCK_ALIGN - 1u)) & ~(size_t)(PWR_BLOCK_ALIGN - 1u);
}

static void *pwr_default_alloc(void *user, size_t bytes) {
  (void)user;
  return malloc(bytes);
}

static void pwr_default_free(void *user, void *ptr) {
  (void)user;
  free(ptr);
}

/* ------------------------------------------------------------------------- */
/* Exact integer arithmetic                                                  */
/* ------------------------------------------------------------------------- */

/* a + b for non-negative int64 with overflow detection. */
static int pwr_add_checked(int64_t a, int64_t b, int64_t *out) {
  if (b > INT64_MAX - a) return 0;
  *out = a + b;
  return 1;
}

/* Exact 64x64 -> 128 product of two non-negative values. */
typedef struct pwr_u128 {
  uint64_t hi, lo;
} pwr_u128;

static pwr_u128 pwr_mul_u64(uint64_t a, uint64_t b) {
  uint64_t a_lo = a & 0xFFFFFFFFu, a_hi = a >> 32;
  uint64_t b_lo = b & 0xFFFFFFFFu, b_hi = b >> 32;
  uint64_t ll = a_lo * b_lo;
  uint64_t lh = a_lo * b_hi;
  uint64_t hl = a_hi * b_lo;
  uint64_t hh = a_hi * b_hi;
  uint64_t mid = (ll >> 32) + (lh & 0xFFFFFFFFu) + (hl & 0xFFFFFFFFu);
  pwr_u128 r;
  r.lo = (ll & 0xFFFFFFFFu) | (mid << 32);
  r.hi = hh + (lh >> 32) + (hl >> 32) + (mid >> 32);
  return r;
}

/* q = floor(a*b/m) and r = a*b - q*m, exactly, for 0 <= a <= m, 0 <= b and
 * m > 0, all within the non-negative int64 range. Because a <= m the quotient
 * is at most b and the remainder is below m, so both results fit in int64.
 * Shift-subtract long division: m < 2^63 and the running remainder stays below
 * m, so the doubling step never overflows 64 bits. */
static void pwr_muldiv(int64_t a, int64_t b, int64_t m, int64_t *q, int64_t *r) {
  pwr_u128 n = pwr_mul_u64((uint64_t)a, (uint64_t)b);
  uint64_t rem = 0u, quot = 0u, d = (uint64_t)m;
  int i;
  for (i = 127; i >= 0; --i) {
    uint64_t bit = (i >= 64) ? ((n.hi >> (i - 64)) & 1u) : ((n.lo >> i) & 1u);
    rem = (rem << 1) | bit;
    quot <<= 1;
    if (rem >= d) {
      rem -= d;
      quot |= 1u;
    }
  }
  *q = (int64_t)quot;
  *r = (int64_t)rem;
}

static int64_t pwr_gcd(int64_t a, int64_t b) {
  while (b != 0) {
    int64_t t = a % b;
    a = b;
    b = t;
  }
  return a;
}

/* The exact reduced fraction num/den == consumed/demand, in the uint32 pair the
 * header declares. demand == 0 is 0/1. When the reduced denominator does not
 * fit in uint32 — only reachable for a network demand above 2^32 whose consumed
 * share shares almost no factor with it — the value is reported at the largest
 * representable denominator instead, rounded down, so it never overstates
 * satisfaction. That fallback is the single inexact number this library can
 * produce and is recorded in docs/lanes/power-impl.md. */
static void pwr_satisfaction(int64_t consumed, int64_t demand, uint32_t *num, uint32_t *den) {
  int64_t g, n, d;
  if (demand <= 0) {
    *num = 0u;
    *den = 1u;
    return;
  }
  /* 0 <= consumed and demand > 0 here, so pwr_gcd is positive in both places:
     gcd(0, demand) == demand and gcd(n, 0xFFFFFFFF) >= 1. No guard needed. */
  g = pwr_gcd(consumed, demand);
  n = consumed / g;
  d = demand / g;
  if (d > (int64_t)0xFFFFFFFFu) {
    int64_t q, r;
    pwr_muldiv(consumed, (int64_t)0xFFFFFFFFu, demand, &q, &r);
    n = q;
    d = (int64_t)0xFFFFFFFFu;
    g = pwr_gcd(n, d);
    n /= g;
    d /= g;
  }
  *num = (uint32_t)n;
  *den = (uint32_t)d;
}

/* CRC-32/ISO-HDLC, reflected polynomial 0xEDB88320, init and final xor
 * 0xFFFFFFFF. Bitwise: no table, hence no static mutable state. */
static uint32_t pwr_crc32(const unsigned char *p, size_t len) {
  uint32_t crc = 0xFFFFFFFFu;
  size_t i;
  unsigned b;
  for (i = 0u; i < len; ++i) {
    crc ^= (uint32_t)p[i];
    for (b = 0u; b < 8u; ++b) crc = (crc & 1u) ? ((crc >> 1) ^ 0xEDB88320u) : (crc >> 1);
  }
  return crc ^ 0xFFFFFFFFu;
}

/* ------------------------------------------------------------------------- */
/* Little-endian field access                                                */
/* ------------------------------------------------------------------------- */

static void pwr_put_u32(unsigned char *p, uint32_t v) {
  p[0] = (unsigned char)(v & 0xffu);
  p[1] = (unsigned char)((v >> 8) & 0xffu);
  p[2] = (unsigned char)((v >> 16) & 0xffu);
  p[3] = (unsigned char)((v >> 24) & 0xffu);
}

static void pwr_put_u64(unsigned char *p, uint64_t v) {
  pwr_put_u32(p, (uint32_t)(v & 0xffffffffu));
  pwr_put_u32(p + 4, (uint32_t)((v >> 32) & 0xffffffffu));
}

static void pwr_put_i64(unsigned char *p, int64_t v) { pwr_put_u64(p, (uint64_t)v); }

static uint32_t pwr_get_u32(const unsigned char *p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static uint64_t pwr_get_u64(const unsigned char *p) {
  return (uint64_t)pwr_get_u32(p) | ((uint64_t)pwr_get_u32(p + 4) << 32);
}

static int64_t pwr_get_i64(const unsigned char *p) { return (int64_t)pwr_get_u64(p); }

/* ------------------------------------------------------------------------- */
/* Internal representation                                                   */
/* ------------------------------------------------------------------------- */

typedef struct pwr_node_rec {
  int64_t production;
  int64_t demand;
  int64_t capacity;
  int64_t stored;
  int64_t allocation; /* result of the last tick that touched this node */
  uint64_t tag;
  uint32_t priority;
  uint32_t generation;  /* PWR_GEN_MIN .. PWR_GEN_MAX, never 0            */
  uint32_t network;     /* owning network slot, PWR_NIL when not live     */
  uint32_t next_member; /* next member of that network, ascending order   */
  uint32_t edge_head;   /* first incident edge slot, PWR_NIL when none    */
  uint32_t degree;      /* number of incident edges                       */
  uint32_t next_free;   /* free-list link when !live                      */
  uint32_t live;
} pwr_node_rec;

typedef struct pwr_edge_rec {
  uint32_t a, b;       /* node slots; a != b                              */
  uint32_t next[2];    /* intrusive adjacency: [0] for a, [1] for b       */
  uint32_t prev[2];
  uint32_t generation;
  uint32_t next_free;
  uint32_t live;
} pwr_edge_rec;

typedef struct pwr_net_rec {
  uint32_t generation;
  uint32_t size;
  uint32_t first_member;
  uint32_t next_free;
  uint32_t live;
  /* Filled by the last tick of this network; the sums the header calls
     production/demand/stored/capacity are recomputed on read, never cached. */
  int64_t supplied, consumed, charged, discharged, wasted;
  uint32_t sat_num, sat_den;
} pwr_net_rec;

struct fbs_power_context {
  fbs_power_allocator alloc;
  size_t block_size;

  uint32_t max_nodes, max_edges, max_networks;
  fbs_power_policy policy;

  uint32_t node_top, edge_top, net_top;       /* high-water marks       */
  uint32_t node_count, edge_count, net_count; /* live counts            */
  uint32_t node_free, edge_free, net_free;    /* free-list heads        */

  uint32_t stamp_gen; /* monotone BFS labelling counter */

  pwr_node_rec *nodes;
  pwr_edge_rec *edges;
  pwr_net_rec *nets;

  uint32_t *queue; /* BFS queue,                       max_nodes */
  uint32_t *stamp; /* BFS component label per node,    max_nodes */
  uint32_t *chead; /* component list heads,            max_nodes */
  uint32_t *ctail; /* component list tails,            max_nodes */
  uint32_t *mem;   /* members of the network in hand,  max_nodes */
  uint32_t *ord;   /* ordering permutation,            max_nodes */
  int64_t *val;    /* per-member share,                max_nodes */
  int64_t *rem;    /* per-member remainder,            max_nodes */
};

/* ------------------------------------------------------------------------- */
/* Status names, version, config                                             */
/* ------------------------------------------------------------------------- */

const char *fbs_power_status_name(int status) {
  switch (status) {
    case FBS_POWER_OK: return "ok";
    case FBS_POWER_E_INVALID: return "invalid";
    case FBS_POWER_E_RANGE: return "range";
    case FBS_POWER_E_FULL: return "full";
    case FBS_POWER_E_MEMORY: return "memory";
    case FBS_POWER_E_STATE: return "state";
    case FBS_POWER_E_NOT_FOUND: return "not_found";
    case FBS_POWER_E_EXISTS: return "exists";
    case FBS_POWER_E_OVERFLOW: return "overflow";
    case FBS_POWER_E_SCHEMA: return "schema";
    case FBS_POWER_E_TRUNCATED: return "truncated";
    default: return "unknown";
  }
}

unsigned fbs_power_version(void) { return FBS_POWER_VERSION; }

fbs_power_config fbs_power_config_default(void) {
  fbs_power_config cfg;
  cfg.max_nodes = 256u;
  cfg.max_edges = 512u;
  cfg.max_networks = 256u;
  cfg.policy = FBS_POWER_POLICY_PROPORTIONAL;
  return cfg;
}

static int pwr_policy_valid(fbs_power_policy p) {
  return p == FBS_POWER_POLICY_PROPORTIONAL || p == FBS_POWER_POLICY_PRIORITY ||
         p == FBS_POWER_POLICY_ALL_OR_NOTHING;
}

/* Header ranges: max_nodes 1..(1<<20)-1, max_edges 0..(1<<20)-1,
 * max_networks >= 1 (and, like every other table, at most (1<<20)-1 so its
 * handles stay representable). A bad policy is a bad enum, not a range error. */
static fbs_power_status pwr_config_check(const fbs_power_config *cfg) {
  if (cfg->max_nodes < 1u || cfg->max_nodes > PWR_MAX_CAPACITY) return FBS_POWER_E_RANGE;
  if (cfg->max_edges > PWR_MAX_CAPACITY) return FBS_POWER_E_RANGE;
  if (cfg->max_networks < 1u || cfg->max_networks > PWR_MAX_CAPACITY) return FBS_POWER_E_RANGE;
  if (!pwr_policy_valid(cfg->policy)) return FBS_POWER_E_INVALID;
  return FBS_POWER_OK;
}

/* ------------------------------------------------------------------------- */
/* Handles                                                                   */
/* ------------------------------------------------------------------------- */

static uint32_t pwr_handle(uint32_t generation, uint32_t index) {
  return (generation << PWR_INDEX_BITS) | (index & PWR_INDEX_MASK);
}

static uint32_t pwr_handle_index(uint32_t h) { return h & PWR_INDEX_MASK; }
static uint32_t pwr_handle_gen(uint32_t h) { return h >> PWR_INDEX_BITS; }

static uint32_t pwr_next_gen(uint32_t g) { return (g >= PWR_GEN_MAX) ? PWR_GEN_MIN : g + 1u; }

/* E_INVALID for a null or structurally impossible handle, E_NOT_FOUND for a
 * slot that was never issued or has been freed (stale generation). */
static fbs_power_status pwr_resolve_node(const fbs_power_context *ctx, fbs_power_node n,
                                         uint32_t *out) {
  uint32_t idx, gen;
  if (n == FBS_POWER_INVALID) return FBS_POWER_E_INVALID;
  gen = pwr_handle_gen(n);
  if (gen < PWR_GEN_MIN || gen > PWR_GEN_MAX) return FBS_POWER_E_INVALID;
  idx = pwr_handle_index(n);
  if (idx >= ctx->max_nodes) return FBS_POWER_E_INVALID;
  if (!ctx->nodes[idx].live || ctx->nodes[idx].generation != gen) return FBS_POWER_E_NOT_FOUND;
  *out = idx;
  return FBS_POWER_OK;
}

static fbs_power_status pwr_resolve_edge(const fbs_power_context *ctx, fbs_power_edge e,
                                         uint32_t *out) {
  uint32_t idx, gen;
  if (e == FBS_POWER_INVALID) return FBS_POWER_E_INVALID;
  gen = pwr_handle_gen(e);
  if (gen < PWR_GEN_MIN || gen > PWR_GEN_MAX) return FBS_POWER_E_INVALID;
  idx = pwr_handle_index(e);
  if (idx >= ctx->max_edges) return FBS_POWER_E_INVALID;
  if (!ctx->edges[idx].live || ctx->edges[idx].generation != gen) return FBS_POWER_E_NOT_FOUND;
  *out = idx;
  return FBS_POWER_OK;
}

static fbs_power_status pwr_resolve_net(const fbs_power_context *ctx, fbs_power_network net,
                                        uint32_t *out) {
  uint32_t idx, gen;
  if (net == FBS_POWER_INVALID) return FBS_POWER_E_INVALID;
  gen = pwr_handle_gen(net);
  if (gen < PWR_GEN_MIN || gen > PWR_GEN_MAX) return FBS_POWER_E_INVALID;
  idx = pwr_handle_index(net);
  if (idx >= ctx->max_networks) return FBS_POWER_E_INVALID;
  if (!ctx->nets[idx].live || ctx->nets[idx].generation != gen) return FBS_POWER_E_NOT_FOUND;
  *out = idx;
  return FBS_POWER_OK;
}

static fbs_power_node pwr_node_handle(const fbs_power_context *ctx, uint32_t idx) {
  return pwr_handle(ctx->nodes[idx].generation, idx);
}

static fbs_power_edge pwr_edge_handle(const fbs_power_context *ctx, uint32_t idx) {
  return pwr_handle(ctx->edges[idx].generation, idx);
}

static fbs_power_network pwr_net_handle(const fbs_power_context *ctx, uint32_t idx) {
  return pwr_handle(ctx->nets[idx].generation, idx);
}

/* ------------------------------------------------------------------------- */
/* Lifetime                                                                  */
/* ------------------------------------------------------------------------- */

static size_t pwr_block_layout(const fbs_power_config *cfg, size_t *off) {
  size_t at = pwr_align_up(sizeof(struct fbs_power_context));
  size_t n = (size_t)cfg->max_nodes;
  off[0] = at; /* nodes */
  at = pwr_align_up(at + n * sizeof(pwr_node_rec));
  off[1] = at; /* edges */
  at = pwr_align_up(at + (size_t)cfg->max_edges * sizeof(pwr_edge_rec));
  off[2] = at; /* networks */
  at = pwr_align_up(at + (size_t)cfg->max_networks * sizeof(pwr_net_rec));
  off[3] = at; /* queue */
  at = pwr_align_up(at + n * sizeof(uint32_t));
  off[4] = at; /* stamp */
  at = pwr_align_up(at + n * sizeof(uint32_t));
  off[5] = at; /* chead */
  at = pwr_align_up(at + n * sizeof(uint32_t));
  off[6] = at; /* ctail */
  at = pwr_align_up(at + n * sizeof(uint32_t));
  off[7] = at; /* mem */
  at = pwr_align_up(at + n * sizeof(uint32_t));
  off[8] = at; /* ord */
  at = pwr_align_up(at + n * sizeof(uint32_t));
  off[9] = at; /* val and rem, both int64 */
  at = pwr_align_up(at + 2u * n * sizeof(int64_t));
  return at;
}

static void pwr_reset(fbs_power_context *ctx) {
  uint32_t i;
  ctx->node_top = 0u;
  ctx->edge_top = 0u;
  ctx->net_top = 0u;
  ctx->node_count = 0u;
  ctx->edge_count = 0u;
  ctx->net_count = 0u;
  ctx->node_free = PWR_NIL;
  ctx->edge_free = PWR_NIL;
  ctx->net_free = PWR_NIL;
  ctx->stamp_gen = 0u;
  for (i = 0u; i < ctx->max_nodes; ++i) {
    ctx->nodes[i].generation = PWR_GEN_MIN;
    ctx->nodes[i].network = PWR_NIL;
    ctx->nodes[i].next_member = PWR_NIL;
    ctx->nodes[i].edge_head = PWR_NIL;
    ctx->nodes[i].next_free = PWR_NIL;
    ctx->stamp[i] = 0u;
  }
  for (i = 0u; i < ctx->max_edges; ++i) {
    ctx->edges[i].generation = PWR_GEN_MIN;
    ctx->edges[i].next_free = PWR_NIL;
  }
  for (i = 0u; i < ctx->max_networks; ++i) {
    ctx->nets[i].generation = PWR_GEN_MIN;
    ctx->nets[i].first_member = PWR_NIL;
    ctx->nets[i].next_free = PWR_NIL;
    ctx->nets[i].sat_den = 1u;
  }
}

static fbs_power_status pwr_context_alloc(const fbs_power_config *cfg,
                                          const fbs_power_allocator *alloc,
                                          fbs_power_context **out) {
  fbs_power_allocator a;
  size_t off[PWR_REGIONS], total;
  unsigned char *block;
  fbs_power_context *ctx;

  if (alloc) {
    if (!alloc->alloc || !alloc->free) return FBS_POWER_E_INVALID;
    a = *alloc;
  } else {
    a.alloc = pwr_default_alloc;
    a.free = pwr_default_free;
    a.user = NULL;
  }

  total = pwr_block_layout(cfg, off);
  block = (unsigned char *)a.alloc(a.user, total);
  if (!block) return FBS_POWER_E_MEMORY;
  memset(block, 0, total);

  ctx = (fbs_power_context *)(void *)block;
  ctx->alloc = a;
  ctx->block_size = total;
  ctx->max_nodes = cfg->max_nodes;
  ctx->max_edges = cfg->max_edges;
  ctx->max_networks = cfg->max_networks;
  ctx->policy = cfg->policy;

  ctx->nodes = (pwr_node_rec *)(void *)(block + off[0]);
  ctx->edges = (pwr_edge_rec *)(void *)(block + off[1]);
  ctx->nets = (pwr_net_rec *)(void *)(block + off[2]);
  ctx->queue = (uint32_t *)(void *)(block + off[3]);
  ctx->stamp = (uint32_t *)(void *)(block + off[4]);
  ctx->chead = (uint32_t *)(void *)(block + off[5]);
  ctx->ctail = (uint32_t *)(void *)(block + off[6]);
  ctx->mem = (uint32_t *)(void *)(block + off[7]);
  ctx->ord = (uint32_t *)(void *)(block + off[8]);
  ctx->val = (int64_t *)(void *)(block + off[9]);
  ctx->rem = ctx->val + cfg->max_nodes;

  pwr_reset(ctx);
  *out = ctx;
  return FBS_POWER_OK;
}

fbs_power_status fbs_power_create(const fbs_power_config *cfg, const fbs_power_allocator *alloc,
                                  fbs_power_context **out) {
  fbs_power_config c;
  fbs_power_context *ctx = NULL;
  fbs_power_status st;

  if (!out) return FBS_POWER_E_INVALID;
  c = cfg ? *cfg : fbs_power_config_default();
  st = pwr_config_check(&c);
  if (st != FBS_POWER_OK) return st;

  st = pwr_context_alloc(&c, alloc, &ctx);
  if (st != FBS_POWER_OK) return st;
  *out = ctx;
  return FBS_POWER_OK;
}

void fbs_power_destroy(fbs_power_context *ctx) {
  fbs_power_allocator a;
  if (!ctx) return;
  a = ctx->alloc;
  a.free(a.user, ctx);
}

void fbs_power_clear(fbs_power_context *ctx) {
  uint32_t i;
  if (!ctx) return;
  /* Every live slot is freed, which bumps its generation: handles issued
     before the clear can never resolve again. */
  for (i = 0u; i < ctx->node_top; ++i) {
    if (ctx->nodes[i].live) ctx->nodes[i].generation = pwr_next_gen(ctx->nodes[i].generation);
  }
  for (i = 0u; i < ctx->edge_top; ++i) {
    if (ctx->edges[i].live) ctx->edges[i].generation = pwr_next_gen(ctx->edges[i].generation);
  }
  for (i = 0u; i < ctx->net_top; ++i) {
    if (ctx->nets[i].live) ctx->nets[i].generation = pwr_next_gen(ctx->nets[i].generation);
  }
  for (i = 0u; i < ctx->node_top; ++i) {
    pwr_node_rec *r = &ctx->nodes[i];
    uint32_t gen = r->generation;
    memset(r, 0, sizeof *r);
    r->generation = gen;
    r->network = PWR_NIL;
    r->next_member = PWR_NIL;
    r->edge_head = PWR_NIL;
    r->next_free = PWR_NIL;
  }
  for (i = 0u; i < ctx->edge_top; ++i) {
    pwr_edge_rec *r = &ctx->edges[i];
    uint32_t gen = r->generation;
    memset(r, 0, sizeof *r);
    r->generation = gen;
    r->next_free = PWR_NIL;
  }
  for (i = 0u; i < ctx->net_top; ++i) {
    pwr_net_rec *r = &ctx->nets[i];
    uint32_t gen = r->generation;
    memset(r, 0, sizeof *r);
    r->generation = gen;
    r->first_member = PWR_NIL;
    r->next_free = PWR_NIL;
    r->sat_den = 1u;
  }
  ctx->node_top = 0u;
  ctx->edge_top = 0u;
  ctx->net_top = 0u;
  ctx->node_count = 0u;
  ctx->edge_count = 0u;
  ctx->net_count = 0u;
  ctx->node_free = PWR_NIL;
  ctx->edge_free = PWR_NIL;
  ctx->net_free = PWR_NIL;
}

size_t fbs_power_memory(const fbs_power_context *ctx) { return ctx ? ctx->block_size : 0u; }
uint32_t fbs_power_node_count(const fbs_power_context *ctx) { return ctx ? ctx->node_count : 0u; }
uint32_t fbs_power_edge_count(const fbs_power_context *ctx) { return ctx ? ctx->edge_count : 0u; }
uint32_t fbs_power_network_count(const fbs_power_context *ctx) { return ctx ? ctx->net_count : 0u; }

/* ------------------------------------------------------------------------- */
/* Slot allocation                                                           */
/* ------------------------------------------------------------------------- */

static uint32_t pwr_node_alloc(fbs_power_context *ctx) {
  uint32_t idx;
  if (ctx->node_free != PWR_NIL) {
    idx = ctx->node_free;
    ctx->node_free = ctx->nodes[idx].next_free;
  } else if (ctx->node_top < ctx->max_nodes) {
    idx = ctx->node_top++;
  } else {
    return PWR_NIL;
  }
  ctx->nodes[idx].next_free = PWR_NIL;
  ctx->nodes[idx].live = 1u;
  ctx->node_count += 1u;
  return idx;
}

static void pwr_node_release(fbs_power_context *ctx, uint32_t idx) {
  pwr_node_rec *r = &ctx->nodes[idx];
  uint32_t gen = pwr_next_gen(r->generation);
  memset(r, 0, sizeof *r);
  r->generation = gen;
  r->network = PWR_NIL;
  r->next_member = PWR_NIL;
  r->edge_head = PWR_NIL;
  r->next_free = ctx->node_free;
  ctx->node_free = idx;
  ctx->node_count -= 1u;
}

static uint32_t pwr_edge_alloc(fbs_power_context *ctx) {
  uint32_t idx;
  if (ctx->edge_free != PWR_NIL) {
    idx = ctx->edge_free;
    ctx->edge_free = ctx->edges[idx].next_free;
  } else if (ctx->edge_top < ctx->max_edges) {
    idx = ctx->edge_top++;
  } else {
    return PWR_NIL;
  }
  ctx->edges[idx].next_free = PWR_NIL;
  ctx->edges[idx].live = 1u;
  ctx->edge_count += 1u;
  return idx;
}

static void pwr_edge_release(fbs_power_context *ctx, uint32_t idx) {
  pwr_edge_rec *r = &ctx->edges[idx];
  uint32_t gen = pwr_next_gen(r->generation);
  memset(r, 0, sizeof *r);
  r->generation = gen;
  r->next_free = ctx->edge_free;
  ctx->edge_free = idx;
  ctx->edge_count -= 1u;
}

static uint32_t pwr_net_alloc(fbs_power_context *ctx) {
  uint32_t idx;
  pwr_net_rec *r;
  if (ctx->net_free != PWR_NIL) {
    idx = ctx->net_free;
    ctx->net_free = ctx->nets[idx].next_free;
  } else if (ctx->net_top < ctx->max_networks) {
    idx = ctx->net_top++;
  } else {
    return PWR_NIL;
  }
  r = &ctx->nets[idx];
  r->size = 0u;
  r->first_member = PWR_NIL;
  r->next_free = PWR_NIL;
  r->live = 1u;
  r->supplied = 0;
  r->consumed = 0;
  r->charged = 0;
  r->discharged = 0;
  r->wasted = 0;
  r->sat_num = 0u;
  r->sat_den = 1u;
  ctx->net_count += 1u;
  return idx;
}

static void pwr_net_release(fbs_power_context *ctx, uint32_t idx) {
  pwr_net_rec *r = &ctx->nets[idx];
  uint32_t gen = pwr_next_gen(r->generation);
  memset(r, 0, sizeof *r);
  r->generation = gen;
  r->first_member = PWR_NIL;
  r->sat_den = 1u;
  r->next_free = ctx->net_free;
  ctx->net_free = idx;
  ctx->net_count -= 1u;
}

/* ------------------------------------------------------------------------- */
/* Adjacency (intrusive doubly linked lists, O(degree) removal)              */
/* ------------------------------------------------------------------------- */

static uint32_t pwr_side_of(const pwr_edge_rec *e, uint32_t node) { return e->a == node ? 0u : 1u; }

static void pwr_adj_link(fbs_power_context *ctx, uint32_t ei, uint32_t node, uint32_t side) {
  pwr_edge_rec *e = &ctx->edges[ei];
  uint32_t head = ctx->nodes[node].edge_head;
  e->prev[side] = PWR_NIL;
  e->next[side] = head;
  if (head != PWR_NIL) {
    pwr_edge_rec *h = &ctx->edges[head];
    h->prev[pwr_side_of(h, node)] = ei;
  }
  ctx->nodes[node].edge_head = ei;
  ctx->nodes[node].degree += 1u;
}

static void pwr_adj_unlink(fbs_power_context *ctx, uint32_t ei, uint32_t node, uint32_t side) {
  pwr_edge_rec *e = &ctx->edges[ei];
  uint32_t pv = e->prev[side], nx = e->next[side];
  if (pv != PWR_NIL) {
    pwr_edge_rec *p = &ctx->edges[pv];
    p->next[pwr_side_of(p, node)] = nx;
  } else {
    ctx->nodes[node].edge_head = nx;
  }
  if (nx != PWR_NIL) {
    pwr_edge_rec *n = &ctx->edges[nx];
    n->prev[pwr_side_of(n, node)] = pv;
  }
  e->prev[side] = PWR_NIL;
  e->next[side] = PWR_NIL;
  ctx->nodes[node].degree -= 1u;
}

/* The edge slot joining a and b, or PWR_NIL. Walks the shorter list. */
static uint32_t pwr_find_edge(const fbs_power_context *ctx, uint32_t a, uint32_t b) {
  uint32_t from = ctx->nodes[a].degree <= ctx->nodes[b].degree ? a : b;
  uint32_t other = (from == a) ? b : a;
  uint32_t ei = ctx->nodes[from].edge_head;
  while (ei != PWR_NIL) {
    const pwr_edge_rec *e = &ctx->edges[ei];
    uint32_t side = pwr_side_of(e, from);
    if ((side ? e->a : e->b) == other) return ei;
    ei = e->next[side];
  }
  return PWR_NIL;
}

/* ------------------------------------------------------------------------- */
/* Membership lists (always ascending by node index)                         */
/* ------------------------------------------------------------------------- */

static uint32_t pwr_merge_members(fbs_power_context *ctx, uint32_t ha, uint32_t hb) {
  uint32_t head = PWR_NIL, tail = PWR_NIL, pick;
  while (ha != PWR_NIL || hb != PWR_NIL) {
    if (hb == PWR_NIL || (ha != PWR_NIL && ha < hb)) {
      pick = ha;
      ha = ctx->nodes[ha].next_member;
    } else {
      pick = hb;
      hb = ctx->nodes[hb].next_member;
    }
    if (tail == PWR_NIL)
      head = pick;
    else
      ctx->nodes[tail].next_member = pick;
    tail = pick;
  }
  if (tail != PWR_NIL) ctx->nodes[tail].next_member = PWR_NIL;
  return head;
}

/* ------------------------------------------------------------------------- */
/* Component labelling and splitting                                         */
/* ------------------------------------------------------------------------- */

static void pwr_stamp_rebase(fbs_power_context *ctx) {
  /* Room for one label per node plus the base itself. */
  if (ctx->stamp_gen <= 0xFFFFFFFFu - (ctx->max_nodes + 2u)) return;
  memset(ctx->stamp, 0, (size_t)ctx->max_nodes * sizeof(uint32_t));
  ctx->stamp_gen = 0u;
}

static void pwr_bfs(fbs_power_context *ctx, uint32_t start, uint32_t base, uint32_t label,
                    uint32_t skip_edge, uint32_t skip_node) {
  size_t qh = 0u, qt = 0u;
  ctx->stamp[start] = label;
  ctx->queue[qt++] = start;
  while (qh < qt) {
    uint32_t n = ctx->queue[qh++];
    uint32_t ei = ctx->nodes[n].edge_head;
    while (ei != PWR_NIL) {
      const pwr_edge_rec *e = &ctx->edges[ei];
      uint32_t side = pwr_side_of(e, n);
      uint32_t other = side ? e->a : e->b;
      uint32_t next = e->next[side];
      if (ei != skip_edge && other != skip_node && ctx->stamp[other] <= base) {
        ctx->stamp[other] = label;
        ctx->queue[qt++] = other;
      }
      ei = next;
    }
  }
}

/* Labels the components of `net` (optionally as if `skip_edge` / `skip_node`
 * were already gone) into ctx->stamp as base+1 .. base+components, discovering
 * `first_start`'s component first when it is given and the rest in ascending
 * member order. Mutates no graph state. */
static uint32_t pwr_label_components(fbs_power_context *ctx, uint32_t net, uint32_t first_start,
                                     uint32_t skip_edge, uint32_t skip_node, uint32_t *out_base) {
  uint32_t base, components = 0u, m;
  pwr_stamp_rebase(ctx);
  base = ctx->stamp_gen;
  if (first_start != PWR_NIL) {
    components = 1u;
    pwr_bfs(ctx, first_start, base, base + components, skip_edge, skip_node);
  }
  for (m = ctx->nets[net].first_member; m != PWR_NIL; m = ctx->nodes[m].next_member) {
    if (m == skip_node) continue;
    if (ctx->stamp[m] > base) continue;
    components += 1u;
    pwr_bfs(ctx, m, base, base + components, skip_edge, skip_node);
  }
  ctx->stamp_gen = base + components;
  *out_base = base;
  return components;
}

/* Rebuilds `net`'s membership from a labelling. Component 0 keeps `net`;
 * components 1.. take the network slots in `extra`. `skip_node` is dropped. */
static void pwr_apply_components(fbs_power_context *ctx, uint32_t net, uint32_t base,
                                 uint32_t components, uint32_t skip_node, const uint32_t *extra) {
  uint32_t c, m = ctx->nets[net].first_member;
  for (c = 0u; c < components; ++c) {
    ctx->chead[c] = PWR_NIL;
    ctx->ctail[c] = PWR_NIL;
  }
  while (m != PWR_NIL) {
    uint32_t next = ctx->nodes[m].next_member;
    if (m != skip_node) {
      uint32_t k = ctx->stamp[m] - base - 1u;
      ctx->nodes[m].next_member = PWR_NIL;
      if (ctx->ctail[k] == PWR_NIL)
        ctx->chead[k] = m;
      else
        ctx->nodes[ctx->ctail[k]].next_member = m;
      ctx->ctail[k] = m;
      ctx->nodes[m].network = (k == 0u) ? net : extra[k - 1u];
    }
    m = next;
  }
  for (c = 0u; c < components; ++c) {
    uint32_t target = (c == 0u) ? net : extra[c - 1u];
    uint32_t size = 0u, walk = ctx->chead[c];
    while (walk != PWR_NIL) {
      size += 1u;
      walk = ctx->nodes[walk].next_member;
    }
    ctx->nets[target].first_member = ctx->chead[c];
    ctx->nets[target].size = size;
  }
}

/* ------------------------------------------------------------------------- */
/* Nodes                                                                     */
/* ------------------------------------------------------------------------- */

static fbs_power_status pwr_desc_check(const fbs_power_node_desc *d) {
  if (d->production < 0 || d->demand < 0 || d->capacity < 0) return FBS_POWER_E_RANGE;
  if (d->stored < 0 || d->stored > d->capacity) return FBS_POWER_E_RANGE;
  return FBS_POWER_OK;
}

fbs_power_status fbs_power_node_add(fbs_power_context *ctx, const fbs_power_node_desc *desc,
                                    fbs_power_node *out) {
  fbs_power_status st;
  uint32_t idx, net;
  pwr_node_rec *r;

  if (!ctx || !desc || !out) return FBS_POWER_E_INVALID;
  st = pwr_desc_check(desc);
  if (st != FBS_POWER_OK) return st;
  if (ctx->node_free == PWR_NIL && ctx->node_top >= ctx->max_nodes) return FBS_POWER_E_FULL;
  if (ctx->net_free == PWR_NIL && ctx->net_top >= ctx->max_networks) return FBS_POWER_E_FULL;

  idx = pwr_node_alloc(ctx);
  if (idx == PWR_NIL) return FBS_POWER_E_FULL;
  net = pwr_net_alloc(ctx);
  if (net == PWR_NIL) { /* unreachable: checked above, but never leave a leak */
    pwr_node_release(ctx, idx);
    return FBS_POWER_E_FULL;
  }

  r = &ctx->nodes[idx];
  r->production = desc->production;
  r->demand = desc->demand;
  r->capacity = desc->capacity;
  r->stored = desc->stored;
  r->allocation = 0;
  r->tag = desc->tag;
  r->priority = desc->priority;
  r->network = net;
  r->next_member = PWR_NIL;
  r->edge_head = PWR_NIL;
  r->degree = 0u;

  ctx->nets[net].first_member = idx;
  ctx->nets[net].size = 1u;

  *out = pwr_node_handle(ctx, idx);
  return FBS_POWER_OK;
}

fbs_power_status fbs_power_node_get(const fbs_power_context *ctx, fbs_power_node n,
                                    fbs_power_node_desc *out) {
  fbs_power_status st;
  uint32_t idx;
  const pwr_node_rec *r;
  if (!ctx || !out) return FBS_POWER_E_INVALID;
  st = pwr_resolve_node(ctx, n, &idx);
  if (st != FBS_POWER_OK) return st;
  r = &ctx->nodes[idx];
  out->production = r->production;
  out->demand = r->demand;
  out->capacity = r->capacity;
  out->stored = r->stored;
  out->priority = r->priority;
  out->tag = r->tag;
  return FBS_POWER_OK;
}

fbs_power_status fbs_power_node_set_rates(fbs_power_context *ctx, fbs_power_node n,
                                          int64_t production, int64_t demand) {
  fbs_power_status st;
  uint32_t idx;
  if (!ctx) return FBS_POWER_E_INVALID;
  st = pwr_resolve_node(ctx, n, &idx);
  if (st != FBS_POWER_OK) return st;
  if (production < 0 || demand < 0) return FBS_POWER_E_RANGE;
  ctx->nodes[idx].production = production;
  ctx->nodes[idx].demand = demand;
  return FBS_POWER_OK;
}

fbs_power_status fbs_power_node_set_stored(fbs_power_context *ctx, fbs_power_node n,
                                           int64_t stored) {
  fbs_power_status st;
  uint32_t idx;
  if (!ctx) return FBS_POWER_E_INVALID;
  st = pwr_resolve_node(ctx, n, &idx);
  if (st != FBS_POWER_OK) return st;
  if (stored < 0 || stored > ctx->nodes[idx].capacity) return FBS_POWER_E_RANGE;
  ctx->nodes[idx].stored = stored;
  return FBS_POWER_OK;
}

fbs_power_status fbs_power_node_set_capacity(fbs_power_context *ctx, fbs_power_node n,
                                             int64_t capacity) {
  fbs_power_status st;
  uint32_t idx;
  if (!ctx) return FBS_POWER_E_INVALID;
  st = pwr_resolve_node(ctx, n, &idx);
  if (st != FBS_POWER_OK) return st;
  if (capacity < 0 || capacity < ctx->nodes[idx].stored) return FBS_POWER_E_RANGE;
  ctx->nodes[idx].capacity = capacity;
  return FBS_POWER_OK;
}

fbs_power_status fbs_power_node_set_priority(fbs_power_context *ctx, fbs_power_node n,
                                             uint32_t priority) {
  fbs_power_status st;
  uint32_t idx;
  if (!ctx) return FBS_POWER_E_INVALID;
  st = pwr_resolve_node(ctx, n, &idx);
  if (st != FBS_POWER_OK) return st;
  ctx->nodes[idx].priority = priority;
  return FBS_POWER_OK;
}

fbs_power_status fbs_power_node_remove(fbs_power_context *ctx, fbs_power_node n) {
  fbs_power_status st;
  uint32_t idx, net, base = 0u, components, start, i, ei;

  if (!ctx) return FBS_POWER_E_INVALID;
  st = pwr_resolve_node(ctx, n, &idx);
  if (st != FBS_POWER_OK) return st;
  net = ctx->nodes[idx].network;

  /* Lowest-index surviving member of the component, if any. */
  start = PWR_NIL;
  for (i = ctx->nets[net].first_member; i != PWR_NIL; i = ctx->nodes[i].next_member) {
    if (i != idx) {
      start = i;
      break;
    }
  }

  if (start == PWR_NIL) {
    /* Isolated node: its network goes with it. */
    ctx->nets[net].first_member = PWR_NIL;
    ctx->nets[net].size = 0u;
    pwr_net_release(ctx, net);
    pwr_node_release(ctx, idx);
    return FBS_POWER_OK;
  }

  components = pwr_label_components(ctx, net, start, PWR_NIL, idx, &base);
  if (ctx->net_count + components - 1u > ctx->max_networks) return FBS_POWER_E_FULL;

  /* Commit: drop every incident edge, then rebuild the membership. */
  ei = ctx->nodes[idx].edge_head;
  while (ei != PWR_NIL) {
    pwr_edge_rec *e = &ctx->edges[ei];
    uint32_t side = pwr_side_of(e, idx);
    uint32_t other = side ? e->a : e->b;
    uint32_t next = e->next[side];
    pwr_adj_unlink(ctx, ei, other, side ^ 1u);
    pwr_adj_unlink(ctx, ei, idx, side);
    pwr_edge_release(ctx, ei);
    ei = next;
  }

  for (i = 1u; i < components; ++i) {
    uint32_t fresh = pwr_net_alloc(ctx);
    ctx->ord[i - 1u] = fresh; /* ord doubles as the new-network scratch here */
  }
  pwr_apply_components(ctx, net, base, components, idx, ctx->ord);
  pwr_node_release(ctx, idx);
  return FBS_POWER_OK;
}

fbs_power_status fbs_power_node_network(const fbs_power_context *ctx, fbs_power_node n,
                                        fbs_power_network *out) {
  fbs_power_status st;
  uint32_t idx;
  if (!ctx || !out) return FBS_POWER_E_INVALID;
  st = pwr_resolve_node(ctx, n, &idx);
  if (st != FBS_POWER_OK) return st;
  *out = pwr_net_handle(ctx, ctx->nodes[idx].network);
  return FBS_POWER_OK;
}

fbs_power_status fbs_power_node_allocation(const fbs_power_context *ctx, fbs_power_node n,
                                           int64_t *out_allocated) {
  fbs_power_status st;
  uint32_t idx;
  if (!ctx || !out_allocated) return FBS_POWER_E_INVALID;
  st = pwr_resolve_node(ctx, n, &idx);
  if (st != FBS_POWER_OK) return st;
  *out_allocated = ctx->nodes[idx].allocation;
  return FBS_POWER_OK;
}

/* ------------------------------------------------------------------------- */
/* Edges                                                                     */
/* ------------------------------------------------------------------------- */

fbs_power_status fbs_power_edge_add(fbs_power_context *ctx, fbs_power_node a, fbs_power_node b,
                                    fbs_power_edge *out) {
  fbs_power_status st;
  uint32_t ia, ib, ei, na, nb, keep, drop;

  if (!ctx || !out) return FBS_POWER_E_INVALID;
  st = pwr_resolve_node(ctx, a, &ia);
  if (st != FBS_POWER_OK) return st;
  st = pwr_resolve_node(ctx, b, &ib);
  if (st != FBS_POWER_OK) return st;
  if (ia == ib) return FBS_POWER_E_INVALID;
  if (pwr_find_edge(ctx, ia, ib) != PWR_NIL) return FBS_POWER_E_EXISTS;
  if (ctx->edge_free == PWR_NIL && ctx->edge_top >= ctx->max_edges) return FBS_POWER_E_FULL;

  ei = pwr_edge_alloc(ctx);
  if (ei == PWR_NIL) return FBS_POWER_E_FULL;
  ctx->edges[ei].a = ia;
  ctx->edges[ei].b = ib;
  pwr_adj_link(ctx, ei, ia, 0u);
  pwr_adj_link(ctx, ei, ib, 1u);

  na = ctx->nodes[ia].network;
  nb = ctx->nodes[ib].network;
  if (na != nb) {
    /* Union by size; the lower network slot wins a tie. */
    if (ctx->nets[na].size > ctx->nets[nb].size) {
      keep = na;
      drop = nb;
    } else if (ctx->nets[nb].size > ctx->nets[na].size) {
      keep = nb;
      drop = na;
    } else {
      keep = na < nb ? na : nb;
      drop = na < nb ? nb : na;
    }
    {
      uint32_t m;
      for (m = ctx->nets[drop].first_member; m != PWR_NIL; m = ctx->nodes[m].next_member)
        ctx->nodes[m].network = keep;
    }
    ctx->nets[keep].first_member =
        pwr_merge_members(ctx, ctx->nets[keep].first_member, ctx->nets[drop].first_member);
    ctx->nets[keep].size += ctx->nets[drop].size;
    ctx->nets[drop].first_member = PWR_NIL;
    ctx->nets[drop].size = 0u;
    pwr_net_release(ctx, drop);
  }

  *out = pwr_edge_handle(ctx, ei);
  return FBS_POWER_OK;
}

fbs_power_status fbs_power_edge_remove(fbs_power_context *ctx, fbs_power_edge e) {
  fbs_power_status st;
  uint32_t ei, ia, ib, net, base = 0u, components, start, i;

  if (!ctx) return FBS_POWER_E_INVALID;
  st = pwr_resolve_edge(ctx, e, &ei);
  if (st != FBS_POWER_OK) return st;
  ia = ctx->edges[ei].a;
  ib = ctx->edges[ei].b;
  net = ctx->nodes[ia].network;
  start = ia < ib ? ia : ib; /* the id stays with the lower-index endpoint */

  components = pwr_label_components(ctx, net, start, ei, PWR_NIL, &base);
  if (ctx->net_count + components - 1u > ctx->max_networks) return FBS_POWER_E_FULL;

  pwr_adj_unlink(ctx, ei, ia, 0u);
  pwr_adj_unlink(ctx, ei, ib, 1u);
  pwr_edge_release(ctx, ei);

  if (components > 1u) {
    for (i = 1u; i < components; ++i) ctx->ord[i - 1u] = pwr_net_alloc(ctx);
    pwr_apply_components(ctx, net, base, components, PWR_NIL, ctx->ord);
  }
  return FBS_POWER_OK;
}

fbs_power_status fbs_power_edge_get(const fbs_power_context *ctx, fbs_power_edge e,
                                    fbs_power_node *out_a, fbs_power_node *out_b) {
  fbs_power_status st;
  uint32_t ei;
  if (!ctx || !out_a || !out_b) return FBS_POWER_E_INVALID;
  st = pwr_resolve_edge(ctx, e, &ei);
  if (st != FBS_POWER_OK) return st;
  *out_a = pwr_node_handle(ctx, ctx->edges[ei].a);
  *out_b = pwr_node_handle(ctx, ctx->edges[ei].b);
  return FBS_POWER_OK;
}

/* The house truncation rule (see the header's opening paragraph): on
 * FBS_POWER_E_TRUNCATED only the required count is written and the caller's
 * array is left untouched. */
fbs_power_status fbs_power_edge_list(const fbs_power_context *ctx, fbs_power_edge *out, size_t cap,
                                     size_t *out_count) {
  size_t n = 0u;
  uint32_t i;
  if (!ctx || !out_count) return FBS_POWER_E_INVALID;
  if (!out && cap > 0u) return FBS_POWER_E_INVALID;
  if ((size_t)ctx->edge_count > cap) {
    *out_count = (size_t)ctx->edge_count;
    return FBS_POWER_E_TRUNCATED;
  }
  for (i = 0u; i < ctx->edge_top; ++i) {
    if (ctx->edges[i].live) out[n++] = pwr_edge_handle(ctx, i);
  }
  *out_count = n;
  return FBS_POWER_OK;
}

fbs_power_status fbs_power_node_list(const fbs_power_context *ctx, fbs_power_node *out, size_t cap,
                                     size_t *out_count) {
  size_t n = 0u;
  uint32_t i;
  if (!ctx || !out_count) return FBS_POWER_E_INVALID;
  if (!out && cap > 0u) return FBS_POWER_E_INVALID;
  if ((size_t)ctx->node_count > cap) {
    *out_count = (size_t)ctx->node_count;
    return FBS_POWER_E_TRUNCATED;
  }
  for (i = 0u; i < ctx->node_top; ++i) {
    if (ctx->nodes[i].live) out[n++] = pwr_node_handle(ctx, i);
  }
  *out_count = n;
  return FBS_POWER_OK;
}

/* ------------------------------------------------------------------------- */
/* Networks                                                                  */
/* ------------------------------------------------------------------------- */

fbs_power_status fbs_power_network_list(const fbs_power_context *ctx, fbs_power_network *out,
                                        size_t cap, size_t *out_count) {
  size_t n = 0u;
  uint32_t i;
  if (!ctx || !out_count) return FBS_POWER_E_INVALID;
  if (!out && cap > 0u) return FBS_POWER_E_INVALID;
  if ((size_t)ctx->net_count > cap) {
    *out_count = (size_t)ctx->net_count;
    return FBS_POWER_E_TRUNCATED;
  }
  for (i = 0u; i < ctx->net_top; ++i) {
    if (ctx->nets[i].live) out[n++] = pwr_net_handle(ctx, i);
  }
  *out_count = n;
  return FBS_POWER_OK;
}

fbs_power_status fbs_power_network_members(const fbs_power_context *ctx, fbs_power_network net,
                                           fbs_power_node *out, size_t cap, size_t *out_count) {
  fbs_power_status st;
  uint32_t idx, m;
  size_t n = 0u;
  if (!ctx || !out_count) return FBS_POWER_E_INVALID;
  if (!out && cap > 0u) return FBS_POWER_E_INVALID;
  st = pwr_resolve_net(ctx, net, &idx);
  if (st != FBS_POWER_OK) return st;
  if ((size_t)ctx->nets[idx].size > cap) {
    *out_count = (size_t)ctx->nets[idx].size;
    return FBS_POWER_E_TRUNCATED;
  }
  for (m = ctx->nets[idx].first_member; m != PWR_NIL; m = ctx->nodes[m].next_member)
    out[n++] = pwr_node_handle(ctx, m);
  *out_count = n;
  return FBS_POWER_OK;
}

/* Sums over a network's members, refusing rather than wrapping. total_free is
 * the sum of (capacity - stored), which is what a surplus can absorb. */
static fbs_power_status pwr_net_sums(const fbs_power_context *ctx, uint32_t net, int64_t *supply,
                                     int64_t *demand, int64_t *stored, int64_t *capacity,
                                     int64_t *total_free) {
  int64_t sp = 0, dm = 0, so = 0, ca = 0, fr = 0;
  uint32_t m;
  for (m = ctx->nets[net].first_member; m != PWR_NIL; m = ctx->nodes[m].next_member) {
    const pwr_node_rec *r = &ctx->nodes[m];
    if (!pwr_add_checked(sp, r->production, &sp)) return FBS_POWER_E_OVERFLOW;
    if (!pwr_add_checked(dm, r->demand, &dm)) return FBS_POWER_E_OVERFLOW;
    if (!pwr_add_checked(so, r->stored, &so)) return FBS_POWER_E_OVERFLOW;
    if (!pwr_add_checked(ca, r->capacity, &ca)) return FBS_POWER_E_OVERFLOW;
    if (!pwr_add_checked(fr, r->capacity - r->stored, &fr)) return FBS_POWER_E_OVERFLOW;
  }
  if (supply) *supply = sp;
  if (demand) *demand = dm;
  if (stored) *stored = so;
  if (capacity) *capacity = ca;
  if (total_free) *total_free = fr;
  return FBS_POWER_OK;
}

fbs_power_status fbs_power_network_stats_get(const fbs_power_context *ctx, fbs_power_network net,
                                             fbs_power_network_stats *out) {
  fbs_power_status st;
  uint32_t idx;
  int64_t supply, demand, stored, capacity;
  const pwr_net_rec *r;
  if (!ctx || !out) return FBS_POWER_E_INVALID;
  st = pwr_resolve_net(ctx, net, &idx);
  if (st != FBS_POWER_OK) return st;
  st = pwr_net_sums(ctx, idx, &supply, &demand, &stored, &capacity, NULL);
  if (st != FBS_POWER_OK) return st;
  r = &ctx->nets[idx];
  out->node_count = r->size;
  out->production = supply;
  out->demand = demand;
  out->stored = stored;
  out->capacity = capacity;
  out->supplied = r->supplied;
  out->consumed = r->consumed;
  out->charged = r->charged;
  out->discharged = r->discharged;
  out->wasted = r->wasted;
  out->satisfaction_num = r->sat_num;
  out->satisfaction_den = r->sat_den;
  return FBS_POWER_OK;
}

/* ------------------------------------------------------------------------- */
/* Tick                                                                      */
/* ------------------------------------------------------------------------- */

/* Sorts positions 0..n-1. `before` returns nonzero when x must come first. */
typedef int (*pwr_cmp)(const void *key, uint32_t x, uint32_t y);

static void pwr_sift(uint32_t *a, size_t n, size_t root, pwr_cmp before, const void *key) {
  for (;;) {
    size_t child = root * 2u + 1u, pick;
    if (child >= n) return;
    pick = child;
    if (child + 1u < n && before(key, a[child], a[child + 1u])) pick = child + 1u;
    if (!before(key, a[root], a[pick])) return;
    {
      uint32_t t = a[root];
      a[root] = a[pick];
      a[pick] = t;
    }
    root = pick;
  }
}

/* Heapsort: no recursion, no allocation, and the comparators below are strict
 * total orders (positions are unique), so the result is unique. */
static void pwr_sort(uint32_t *a, size_t n, pwr_cmp before, const void *key) {
  size_t i;
  if (n < 2u) return;
  for (i = n / 2u; i-- > 0u;) pwr_sift(a, n, i, before, key);
  for (i = n; i-- > 1u;) {
    uint32_t t = a[0];
    a[0] = a[i];
    a[i] = t;
    pwr_sift(a, i, 0u, before, key);
  }
}

static int pwr_before_remainder(const void *key, uint32_t x, uint32_t y) {
  const int64_t *rem = (const int64_t *)key;
  if (rem[x] != rem[y]) return rem[x] > rem[y];
  return x < y;
}

typedef struct pwr_prio_key {
  const pwr_node_rec *nodes;
  const uint32_t *mem;
} pwr_prio_key;

static int pwr_before_priority(const void *key, uint32_t x, uint32_t y) {
  const pwr_prio_key *k = (const pwr_prio_key *)key;
  uint32_t px = k->nodes[k->mem[x]].priority, py = k->nodes[k->mem[y]].priority;
  if (px != py) return px < py;
  return k->mem[x] < k->mem[y];
}

/* Hands out exactly `total` over ctx->val[0..n-1] read as weights, by
 * floor(weight * total / weight_sum) plus one unit each to the largest
 * remainders, ties by ascending position (== ascending node index, because
 * ctx->mem is ascending). ctx->val is overwritten with the shares. */
static void pwr_apportion(fbs_power_context *ctx, size_t n, int64_t weight_sum, int64_t total) {
  size_t i;
  int64_t handed = 0, leftover;
  if (weight_sum <= 0 || total <= 0) {
    for (i = 0u; i < n; ++i) ctx->val[i] = 0;
    return;
  }
  for (i = 0u; i < n; ++i) {
    int64_t q, r;
    pwr_muldiv(ctx->val[i], total, weight_sum, &q, &r);
    ctx->val[i] = q;
    ctx->rem[i] = r;
    handed += q;
  }
  leftover = total - handed;
  if (leftover <= 0) return;
  for (i = 0u; i < n; ++i) ctx->ord[i] = (uint32_t)i;
  pwr_sort(ctx->ord, n, pwr_before_remainder, ctx->rem);
  for (i = 0u; i < n && (int64_t)i < leftover; ++i) ctx->val[ctx->ord[i]] += 1;
}

static void pwr_allocate(fbs_power_context *ctx, size_t n, int64_t available, int64_t demand_total,
                         int64_t *out_consumed) {
  size_t i;
  int64_t consumed = 0;

  switch (ctx->policy) {
    case FBS_POWER_POLICY_ALL_OR_NOTHING:
      for (i = 0u; i < n; ++i)
        ctx->val[i] = (available >= demand_total) ? ctx->nodes[ctx->mem[i]].demand : 0;
      break;
    case FBS_POWER_POLICY_PRIORITY: {
      pwr_prio_key key;
      int64_t left = available;
      key.nodes = ctx->nodes;
      key.mem = ctx->mem;
      for (i = 0u; i < n; ++i) {
        ctx->val[i] = 0;
        ctx->ord[i] = (uint32_t)i;
      }
      pwr_sort(ctx->ord, n, pwr_before_priority, &key);
      for (i = 0u; i < n; ++i) {
        uint32_t pos = ctx->ord[i];
        int64_t want = ctx->nodes[ctx->mem[pos]].demand;
        int64_t give = want < left ? want : left;
        ctx->val[pos] = give;
        left -= give;
      }
      break;
    }
    case FBS_POWER_POLICY_PROPORTIONAL:
    default:
      for (i = 0u; i < n; ++i) ctx->val[i] = ctx->nodes[ctx->mem[i]].demand;
      pwr_apportion(ctx, n, demand_total, available);
      break;
  }
  for (i = 0u; i < n; ++i) consumed += ctx->val[i];
  *out_consumed = consumed;
}

/* One network, one tick. Sums have already been validated for overflow. */
static void pwr_tick_one(fbs_power_context *ctx, uint32_t net) {
  pwr_net_rec *nr = &ctx->nets[net];
  size_t n = 0u, i;
  int64_t supply = 0, demand = 0, stored = 0, total_free = 0;
  int64_t consumed = 0, charged = 0, discharged = 0, wasted = 0;
  uint32_t m;

  for (m = nr->first_member; m != PWR_NIL; m = ctx->nodes[m].next_member) {
    const pwr_node_rec *r = &ctx->nodes[m];
    ctx->mem[n++] = m;
    supply += r->production;
    demand += r->demand;
    stored += r->stored;
    total_free += r->capacity - r->stored;
  }

  if (supply >= demand) {
    /* Step 2: everyone is served, the surplus charges batteries by free
       capacity, and whatever storage cannot take is wasted. */
    int64_t surplus = supply - demand;
    int64_t to_charge = surplus < total_free ? surplus : total_free;
    for (i = 0u; i < n; ++i) {
      pwr_node_rec *r = &ctx->nodes[ctx->mem[i]];
      r->allocation = r->demand;
    }
    consumed = demand;
    for (i = 0u; i < n; ++i) {
      const pwr_node_rec *r = &ctx->nodes[ctx->mem[i]];
      ctx->val[i] = r->capacity - r->stored;
    }
    pwr_apportion(ctx, n, total_free, to_charge);
    for (i = 0u; i < n; ++i) ctx->nodes[ctx->mem[i]].stored += ctx->val[i];
    charged = to_charge;
    discharged = 0;
    wasted = surplus - to_charge;
  } else {
    /* Step 3: batteries cover the deficit as far as they can, the available
       supply is allocated by the policy, and storage is only actually drawn
       down for what the policy consumed (see the file header). */
    int64_t deficit = demand - supply;
    int64_t draw = deficit < stored ? deficit : stored;
    int64_t available = supply + draw;
    pwr_allocate(ctx, n, available, demand, &consumed);
    discharged = consumed > supply ? consumed - supply : 0;
    for (i = 0u; i < n; ++i) ctx->nodes[ctx->mem[i]].allocation = ctx->val[i];
    for (i = 0u; i < n; ++i) ctx->val[i] = ctx->nodes[ctx->mem[i]].stored;
    pwr_apportion(ctx, n, stored, discharged);
    for (i = 0u; i < n; ++i) ctx->nodes[ctx->mem[i]].stored -= ctx->val[i];
    charged = 0;
    wasted = supply + discharged - consumed;
  }

  nr->supplied = supply + discharged;
  nr->consumed = consumed;
  nr->charged = charged;
  nr->discharged = discharged;
  nr->wasted = wasted;
  pwr_satisfaction(consumed, demand, &nr->sat_num, &nr->sat_den);
}

/* Everything a tick will need, checked before a single byte changes. */
static fbs_power_status pwr_tick_check(const fbs_power_context *ctx, uint32_t net) {
  int64_t supply, demand, stored, capacity, total_free, tmp;
  fbs_power_status st = pwr_net_sums(ctx, net, &supply, &demand, &stored, &capacity, &total_free);
  if (st != FBS_POWER_OK) return st;
  if (supply >= demand) return FBS_POWER_OK;
  /* supply + draw <= demand, so the available supply cannot overflow. */
  if (!pwr_add_checked(supply, demand - supply < stored ? demand - supply : stored, &tmp))
    return FBS_POWER_E_OVERFLOW;
  return FBS_POWER_OK;
}

fbs_power_status fbs_power_tick(fbs_power_context *ctx) {
  uint32_t i;
  if (!ctx) return FBS_POWER_E_INVALID;
  for (i = 0u; i < ctx->net_top; ++i) {
    if (!ctx->nets[i].live) continue;
    {
      fbs_power_status st = pwr_tick_check(ctx, i);
      if (st != FBS_POWER_OK) return st;
    }
  }
  for (i = 0u; i < ctx->net_top; ++i) {
    if (ctx->nets[i].live) pwr_tick_one(ctx, i);
  }
  return FBS_POWER_OK;
}

fbs_power_status fbs_power_tick_network(fbs_power_context *ctx, fbs_power_network net) {
  fbs_power_status st;
  uint32_t idx;
  if (!ctx) return FBS_POWER_E_INVALID;
  st = pwr_resolve_net(ctx, net, &idx);
  if (st != FBS_POWER_OK) return st;
  st = pwr_tick_check(ctx, idx);
  if (st != FBS_POWER_OK) return st;
  pwr_tick_one(ctx, idx);
  return FBS_POWER_OK;
}

/* ------------------------------------------------------------------------- */
/* Serialization (docs/decisions/power.md §6.1)                              */
/* ------------------------------------------------------------------------- */

size_t fbs_power_serialized_size(const fbs_power_context *ctx) {
  if (!ctx) return 0u;
  return (size_t)PWR_HEADER_BYTES + (size_t)ctx->node_count * PWR_NODE_BYTES +
         (size_t)ctx->edge_count * PWR_EDGE_BYTES + (size_t)PWR_CRC_BYTES;
}

fbs_power_status fbs_power_serialize(const fbs_power_context *ctx, void *buf, size_t cap,
                                     size_t *out_len) {
  unsigned char *p;
  size_t need, off;
  uint32_t i;

  if (!ctx || !out_len) return FBS_POWER_E_INVALID;
  if (!buf && cap > 0u) return FBS_POWER_E_INVALID;
  need = fbs_power_serialized_size(ctx);
  if (cap < need) {
    *out_len = need;
    return FBS_POWER_E_TRUNCATED;
  }

  p = (unsigned char *)buf;
  p[0] = 'F';
  p[1] = 'B';
  p[2] = 'S';
  p[3] = 'P';
  p[4] = 'W';
  p[5] = 'R';
  p[6] = '\0';
  p[7] = '\0';
  pwr_put_u32(p + 8, (uint32_t)FBS_POWER_VERSION);
  pwr_put_u32(p + 12, (uint32_t)ctx->policy);
  pwr_put_u32(p + 16, ctx->node_count);
  pwr_put_u32(p + 20, ctx->edge_count);

  off = PWR_HEADER_BYTES;
  for (i = 0u; i < ctx->node_top; ++i) {
    const pwr_node_rec *r = &ctx->nodes[i];
    unsigned char *w = p + off;
    if (!r->live) continue;
    pwr_put_u32(w, pwr_handle(r->generation, i));
    pwr_put_i64(w + 4, r->production);
    pwr_put_i64(w + 12, r->demand);
    pwr_put_i64(w + 20, r->capacity);
    pwr_put_i64(w + 28, r->stored);
    pwr_put_u32(w + 36, r->priority);
    pwr_put_u64(w + 40, r->tag);
    off += PWR_NODE_BYTES;
  }
  for (i = 0u; i < ctx->edge_top; ++i) {
    const pwr_edge_rec *r = &ctx->edges[i];
    unsigned char *w = p + off;
    if (!r->live) continue;
    pwr_put_u32(w, pwr_handle(r->generation, i));
    pwr_put_u32(w + 4, pwr_node_handle(ctx, r->a));
    pwr_put_u32(w + 8, pwr_node_handle(ctx, r->b));
    off += PWR_EDGE_BYTES;
  }
  pwr_put_u32(p + off, pwr_crc32(p, off));

  *out_len = need;
  return FBS_POWER_OK;
}

/* Position of the node record whose handle is exactly `h`, or PWR_NIL. Node
 * records are written in strictly ascending index order (and that order has
 * already been verified when this runs), so an endpoint is found by binary
 * search in O(log N); the generation has to match too, or the edge names a
 * node that is not in this blob. */
static uint32_t pwr_blob_find_node(const unsigned char *nodes, uint32_t node_count, uint32_t h) {
  uint32_t lo = 0u, hi = node_count, want = pwr_handle_index(h);
  while (lo < hi) {
    uint32_t mid = lo + (hi - lo) / 2u;
    uint32_t nh = pwr_get_u32(nodes + (size_t)mid * PWR_NODE_BYTES);
    uint32_t idx = pwr_handle_index(nh);
    if (idx == want) return (nh == h) ? mid : PWR_NIL;
    if (idx < want)
      lo = mid + 1u;
    else
      hi = mid;
  }
  return PWR_NIL;
}

/* Validates the blob before anything is allocated, in O(N + E log N). Reports
 * the slot counts the blob needs (one past the highest index used, not the live
 * count, because handles carry their index).
 *
 * One rule is deliberately *not* checked here: that no pair of nodes is joined
 * twice. Detecting that from the blob alone is quadratic without scratch
 * memory, and this library allocates exactly once. It is instead checked in
 * O(V + E) by fbs_power_deserialize after the adjacency lists exist, using the
 * context's own BFS stamp array; that path destroys the context before
 * returning FBS_POWER_E_SCHEMA, so a rejected blob still allocates nothing on
 * net and leaks nothing. Every cheap rejection — magic, version, policy,
 * counts, length, crc, handle ordering and ranges, rates, endpoints — happens
 * before a byte is allocated, as it must for an untrusted save. */
static fbs_power_status pwr_blob_check(const unsigned char *p, size_t len, uint32_t *out_nodes,
                                       uint32_t *out_edges, uint32_t *out_networks,
                                       uint32_t *out_policy) {
  uint32_t node_count, edge_count, policy, i;
  uint32_t node_slots = 0u, edge_slots = 0u, networks = 0u;
  size_t need, node_off, edge_off;
  uint32_t last_index = 0u;
  int have_last = 0;

  if (len < (size_t)PWR_HEADER_BYTES + PWR_CRC_BYTES) return FBS_POWER_E_SCHEMA;
  if (p[0] != 'F' || p[1] != 'B' || p[2] != 'S' || p[3] != 'P' || p[4] != 'W' || p[5] != 'R' ||
      p[6] != 0u || p[7] != 0u)
    return FBS_POWER_E_SCHEMA;
  if (pwr_get_u32(p + 8) != (uint32_t)FBS_POWER_VERSION) return FBS_POWER_E_SCHEMA;
  policy = pwr_get_u32(p + 12);
  if (!pwr_policy_valid((fbs_power_policy)policy)) return FBS_POWER_E_SCHEMA;
  node_count = pwr_get_u32(p + 16);
  edge_count = pwr_get_u32(p + 20);
  if (node_count > PWR_MAX_CAPACITY || edge_count > PWR_MAX_CAPACITY) return FBS_POWER_E_SCHEMA;

  need = (size_t)PWR_HEADER_BYTES + (size_t)node_count * PWR_NODE_BYTES +
         (size_t)edge_count * PWR_EDGE_BYTES + (size_t)PWR_CRC_BYTES;
  if (len != need) return FBS_POWER_E_SCHEMA;
  if (pwr_get_u32(p + len - PWR_CRC_BYTES) != pwr_crc32(p, len - PWR_CRC_BYTES))
    return FBS_POWER_E_SCHEMA;

  node_off = PWR_HEADER_BYTES;
  edge_off = node_off + (size_t)node_count * PWR_NODE_BYTES;

  for (i = 0u; i < node_count; ++i) {
    const unsigned char *r = p + node_off + (size_t)i * PWR_NODE_BYTES;
    uint32_t h = pwr_get_u32(r), idx = pwr_handle_index(h), gen = pwr_handle_gen(h);
    int64_t production = pwr_get_i64(r + 4), demand = pwr_get_i64(r + 12);
    int64_t capacity = pwr_get_i64(r + 20), stored = pwr_get_i64(r + 28);
    if (h == FBS_POWER_INVALID) return FBS_POWER_E_SCHEMA;
    if (gen < PWR_GEN_MIN || gen > PWR_GEN_MAX) return FBS_POWER_E_SCHEMA;
    if (idx > PWR_MAX_CAPACITY - 1u) return FBS_POWER_E_SCHEMA;
    if (have_last && idx <= last_index) return FBS_POWER_E_SCHEMA; /* strictly ascending */
    last_index = idx;
    have_last = 1;
    if (production < 0 || demand < 0 || capacity < 0) return FBS_POWER_E_SCHEMA;
    if (stored < 0 || stored > capacity) return FBS_POWER_E_SCHEMA;
    if (idx + 1u > node_slots) node_slots = idx + 1u;
  }

  have_last = 0;
  last_index = 0u;
  for (i = 0u; i < edge_count; ++i) {
    const unsigned char *r = p + edge_off + (size_t)i * PWR_EDGE_BYTES;
    uint32_t h = pwr_get_u32(r), idx = pwr_handle_index(h), gen = pwr_handle_gen(h);
    uint32_t ha = pwr_get_u32(r + 4), hb = pwr_get_u32(r + 8);
    if (h == FBS_POWER_INVALID) return FBS_POWER_E_SCHEMA;
    if (gen < PWR_GEN_MIN || gen > PWR_GEN_MAX) return FBS_POWER_E_SCHEMA;
    if (idx > PWR_MAX_CAPACITY - 1u) return FBS_POWER_E_SCHEMA;
    if (have_last && idx <= last_index) return FBS_POWER_E_SCHEMA;
    last_index = idx;
    have_last = 1;
    if (ha == hb) return FBS_POWER_E_SCHEMA;
    /* Both endpoints must name a live node record, generation included. The
       node records are in strictly ascending index order, so this is a binary
       search rather than a rescan of the whole node table per endpoint. */
    if (pwr_blob_find_node(p + node_off, node_count, ha) == PWR_NIL) return FBS_POWER_E_SCHEMA;
    if (pwr_blob_find_node(p + node_off, node_count, hb) == PWR_NIL) return FBS_POWER_E_SCHEMA;
    if (idx + 1u > edge_slots) edge_slots = idx + 1u;
  }

  /* A network per node in the worst case (no edges at all). The exact count is
     only known once the graph is walked, which happens after the context is
     allocated; this bound is what a NULL cfg sizes the network table with. */
  networks = node_count < 1u ? 1u : node_count;

  *out_nodes = node_slots < 1u ? 1u : node_slots;
  *out_edges = edge_slots;
  *out_networks = networks;
  *out_policy = policy;
  return FBS_POWER_OK;
}

fbs_power_status fbs_power_deserialize(const void *buf, size_t len, const fbs_power_config *cfg,
                                       const fbs_power_allocator *alloc,
                                       fbs_power_context **out) {
  const unsigned char *p = (const unsigned char *)buf;
  fbs_power_config c;
  fbs_power_context *ctx = NULL;
  fbs_power_status st;
  uint32_t node_slots = 0u, edge_slots = 0u, networks = 0u, policy = 0u;
  uint32_t node_count, edge_count, i, base = 0u, components;
  size_t node_off, edge_off;

  if (!buf || !out) return FBS_POWER_E_INVALID;
  if (cfg) {
    st = pwr_config_check(cfg);
    if (st != FBS_POWER_OK) return st;
  }

  st = pwr_blob_check(p, len, &node_slots, &edge_slots, &networks, &policy);
  if (st != FBS_POWER_OK) return st;

  if (cfg) {
    c = *cfg;
    if (c.max_nodes < node_slots || c.max_edges < edge_slots) return FBS_POWER_E_FULL;
  } else {
    c = fbs_power_config_default();
    if (c.max_nodes < node_slots) c.max_nodes = node_slots;
    if (c.max_edges < edge_slots) c.max_edges = edge_slots;
    if (c.max_networks < networks) c.max_networks = networks;
  }
  c.policy = (fbs_power_policy)policy; /* the blob's policy is part of the state */

  st = pwr_context_alloc(&c, alloc, &ctx);
  if (st != FBS_POWER_OK) return st;

  node_count = pwr_get_u32(p + 16);
  edge_count = pwr_get_u32(p + 20);
  node_off = PWR_HEADER_BYTES;
  edge_off = node_off + (size_t)node_count * PWR_NODE_BYTES;

  /* Nodes land back on their own slots, with their own generations, so every
     handle held across the save still resolves. */
  for (i = 0u; i < node_count; ++i) {
    const unsigned char *r = p + node_off + (size_t)i * PWR_NODE_BYTES;
    uint32_t h = pwr_get_u32(r), idx = pwr_handle_index(h);
    pwr_node_rec *t = &ctx->nodes[idx];
    t->generation = pwr_handle_gen(h);
    t->production = pwr_get_i64(r + 4);
    t->demand = pwr_get_i64(r + 12);
    t->capacity = pwr_get_i64(r + 20);
    t->stored = pwr_get_i64(r + 28);
    t->priority = pwr_get_u32(r + 36);
    t->tag = pwr_get_u64(r + 40);
    t->allocation = 0;
    t->network = PWR_NIL;
    t->next_member = PWR_NIL;
    t->edge_head = PWR_NIL;
    t->degree = 0u;
    t->live = 1u;
    if (idx + 1u > ctx->node_top) ctx->node_top = idx + 1u;
  }
  ctx->node_count = node_count;
  for (i = ctx->node_top; i-- > 0u;) {
    if (!ctx->nodes[i].live) {
      ctx->nodes[i].next_free = ctx->node_free;
      ctx->node_free = i;
    }
  }

  for (i = 0u; i < edge_count; ++i) {
    const unsigned char *r = p + edge_off + (size_t)i * PWR_EDGE_BYTES;
    uint32_t h = pwr_get_u32(r), idx = pwr_handle_index(h);
    uint32_t ia = pwr_handle_index(pwr_get_u32(r + 4));
    uint32_t ib = pwr_handle_index(pwr_get_u32(r + 8));
    pwr_edge_rec *t = &ctx->edges[idx];
    t->generation = pwr_handle_gen(h);
    t->a = ia;
    t->b = ib;
    t->live = 1u;
    if (idx + 1u > ctx->edge_top) ctx->edge_top = idx + 1u;
    pwr_adj_link(ctx, idx, ia, 0u);
    pwr_adj_link(ctx, idx, ib, 1u);
  }
  ctx->edge_count = edge_count;
  for (i = ctx->edge_top; i-- > 0u;) {
    if (!ctx->edges[i].live) {
      ctx->edges[i].next_free = ctx->edge_free;
      ctx->edge_free = i;
    }
  }

  /* The one rule pwr_blob_check leaves to us: no pair of nodes joined twice
     (the pack's P7). Now that the adjacency lists exist it is O(V + E) — walk
     each node's neighbours once, stamping them with a per-node label. */
  pwr_stamp_rebase(ctx);
  for (i = 0u; i < ctx->node_top; ++i) {
    uint32_t ei = ctx->nodes[i].edge_head;
    if (!ctx->nodes[i].live) continue;
    ctx->stamp_gen += 1u;
    while (ei != PWR_NIL) {
      const pwr_edge_rec *e = &ctx->edges[ei];
      uint32_t side = pwr_side_of(e, i);
      uint32_t other = side ? e->a : e->b;
      if (ctx->stamp[other] == ctx->stamp_gen) {
        fbs_power_destroy(ctx);
        return FBS_POWER_E_SCHEMA;
      }
      ctx->stamp[other] = ctx->stamp_gen;
      ei = e->next[side];
    }
  }

  /* Networks are derived: the same BFS the runtime uses, started from the
     lowest unassigned node, so membership after a load equals membership
     before it. The components are counted first, because an explicit cfg has
     to be told FBS_POWER_E_FULL when its network table cannot hold them. */
  pwr_stamp_rebase(ctx);
  base = ctx->stamp_gen;
  components = 0u;
  for (i = 0u; i < ctx->node_top; ++i) {
    if (!ctx->nodes[i].live || ctx->stamp[i] > base) continue;
    components += 1u;
    pwr_bfs(ctx, i, base, base + components, PWR_NIL, PWR_NIL);
  }
  ctx->stamp_gen = base + components;
  if (components > ctx->max_networks) {
    fbs_power_destroy(ctx);
    return FBS_POWER_E_FULL;
  }
  for (i = 0u; i < components; ++i) {
    ctx->ord[i] = pwr_net_alloc(ctx);
    ctx->chead[i] = PWR_NIL;
    ctx->ctail[i] = PWR_NIL;
  }
  for (i = 0u; i < ctx->node_top; ++i) {
    uint32_t comp, net;
    if (!ctx->nodes[i].live) continue;
    comp = ctx->stamp[i] - base - 1u;
    net = ctx->ord[comp];
    ctx->nodes[i].network = net;
    ctx->nodes[i].next_member = PWR_NIL;
    if (ctx->ctail[comp] == PWR_NIL)
      ctx->chead[comp] = i;
    else
      ctx->nodes[ctx->ctail[comp]].next_member = i;
    ctx->ctail[comp] = i;
    ctx->nets[net].size += 1u;
  }
  for (i = 0u; i < components; ++i) ctx->nets[ctx->ord[i]].first_member = ctx->chead[i];

  *out = ctx;
  return FBS_POWER_OK;
}
