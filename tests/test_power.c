/*
 * tests/test_power.c — witnesses for include/fbs/power.h.
 *
 * Self-contained: no test framework. Exit code = number of failures (clamped
 * to 100 so it survives the 8-bit exit status; the true count is printed).
 *
 * Covers docs/decisions/power.md §7 items 1..11 with the exact witnesses
 * stated there — every one of them is the negative test for one of the pack's
 * structural bugs P1..P7 recorded in §3 — plus NULL/bad-enum validation and
 * 0xA5 output-untouched checks on every entry point, every E_TRUNCATED path,
 * config validation, the status-name/version functions and two committed
 * golden fixtures.
 *
 * §7 item 12's cross-platform half ("run native and under WASM and compare")
 * is out of scope for this C binary: this repository builds WASM only for the
 * trace module (integrations/wasm/build-trace.sh), and the decision's §10 says
 * the fixture on native satisfies the milestone. The golden files below are
 * exactly the artifacts a WASM replay would be compared against byte for byte.
 *
 *   ./fbs_test_power                      compare against the committed fixtures
 *   ./fbs_test_power --write-fixtures     rewrite tests/fixtures/power/grid.bin
 *                                         and tests/fixtures/power/allocations.txt
 *   ./fbs_test_power --fixture-dir DIR    look for fixtures under DIR
 */

#include "fbs/power.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ------------------------------------------------------------------------- */
/* Harness                                                                   */
/* ------------------------------------------------------------------------- */

static int g_checks = 0;
static int g_fails = 0;

static void check_impl(int cond, const char *expr, const char *file, int line) {
  ++g_checks;
  if (!cond) {
    ++g_fails;
    printf("FAIL %s:%d: %s\n", file, line, expr);
  }
}

#define CHECK(expr) check_impl((expr) ? 1 : 0, #expr, __FILE__, __LINE__)

static const char *g_fixture_dir = "fixtures/power";
static int g_write_fixtures = 0;

static int untouched(const void *p, size_t n, unsigned char fill) {
  const unsigned char *b = (const unsigned char *)p;
  size_t i;
  for (i = 0; i < n; ++i)
    if (b[i] != fill) return 0;
  return 1;
}

/* Deterministic 32-bit xorshift; no rand() anywhere (§7 item 4 requires a
 * fixed, documented seed). */
typedef struct {
  uint32_t state;
} rng;

static void rng_seed(rng *r, uint32_t seed) { r->state = seed ? seed : 1u; }

static uint32_t rng_next(rng *r) {
  uint32_t x = r->state;
  x = (uint32_t)(x ^ (x << 13));
  x = (uint32_t)(x ^ (x >> 17));
  x = (uint32_t)(x ^ (x << 5));
  r->state = x;
  return x;
}

static unsigned rng_below(rng *r, unsigned n) { return n ? (unsigned)(rng_next(r) % n) : 0u; }

/* Counting allocator. */
typedef struct {
  int allocs;
  int frees;
  size_t bytes;
  int budget; /* < 0: unlimited, else the number of allocations still allowed */
} counting_alloc;

static void *ca_alloc(void *user, size_t bytes) {
  counting_alloc *c = (counting_alloc *)user;
  if (c->budget == 0) return NULL;
  if (c->budget > 0) --c->budget;
  ++c->allocs;
  c->bytes += bytes;
  return malloc(bytes);
}

static void ca_free(void *user, void *ptr) {
  counting_alloc *c = (counting_alloc *)user;
  if (!ptr) return;
  ++c->frees;
  free(ptr);
}

static void write_or_compare(const char *path, const unsigned char *bytes, size_t len,
                             const char *what) {
  FILE *fp;
  if (g_write_fixtures) {
    fp = fopen(path, "wb");
    ++g_checks;
    if (!fp) {
      ++g_fails;
      printf("FAIL cannot open %s for writing\n", path);
      return;
    }
    {
      size_t wrote = fwrite(bytes, 1u, len, fp);
      fclose(fp);
      CHECK(wrote == len);
      printf("wrote %s (%lu bytes)\n", path, (unsigned long)len);
    }
    return;
  }
  fp = fopen(path, "rb");
  ++g_checks;
  if (!fp) {
    ++g_fails;
    printf("FAIL cannot open fixture %s (run with --write-fixtures to create it)\n", path);
    return;
  }
  {
    unsigned char golden[4096];
    size_t got = fread(golden, 1u, sizeof golden, fp);
    fclose(fp);
    ++g_checks;
    if (got != len) {
      ++g_fails;
      printf("FAIL %s: %s is %lu bytes, expected %lu\n", path, what, (unsigned long)got,
             (unsigned long)len);
      return;
    }
    CHECK(memcmp(golden, bytes, len) == 0);
  }
}

/* ------------------------------------------------------------------------- */
/* Helpers                                                                   */
/* ------------------------------------------------------------------------- */

#define IDX(h) ((uint32_t)((h) & 0xFFFFFu))
#define GEN(h) ((uint32_t)((h) >> 20))

static fbs_power_context *make_ctx(const fbs_power_config *cfg) {
  fbs_power_context *ctx = NULL;
  CHECK(fbs_power_create(cfg, NULL, &ctx) == FBS_POWER_OK);
  return ctx;
}

static fbs_power_context *make_policy_ctx(fbs_power_policy p) {
  fbs_power_config cfg = fbs_power_config_default();
  cfg.policy = p;
  return make_ctx(&cfg);
}

static fbs_power_node add_node(fbs_power_context *ctx, int64_t production, int64_t demand,
                               int64_t capacity, int64_t stored, uint32_t priority, uint64_t tag) {
  fbs_power_node_desc d;
  fbs_power_node n = FBS_POWER_INVALID;
  d.production = production;
  d.demand = demand;
  d.capacity = capacity;
  d.stored = stored;
  d.priority = priority;
  d.tag = tag;
  CHECK(fbs_power_node_add(ctx, &d, &n) == FBS_POWER_OK);
  return n;
}

static fbs_power_node plain(fbs_power_context *ctx) { return add_node(ctx, 0, 0, 0, 0, 0u, 0u); }

static fbs_power_edge add_edge(fbs_power_context *ctx, fbs_power_node a, fbs_power_node b) {
  fbs_power_edge e = FBS_POWER_INVALID;
  CHECK(fbs_power_edge_add(ctx, a, b, &e) == FBS_POWER_OK);
  return e;
}

static fbs_power_network net_of(fbs_power_context *ctx, fbs_power_node n) {
  fbs_power_network net = FBS_POWER_INVALID;
  CHECK(fbs_power_node_network(ctx, n, &net) == FBS_POWER_OK);
  return net;
}

static int64_t alloc_of(fbs_power_context *ctx, fbs_power_node n) {
  int64_t v = -1;
  CHECK(fbs_power_node_allocation(ctx, n, &v) == FBS_POWER_OK);
  return v;
}

static int64_t stored_of(fbs_power_context *ctx, fbs_power_node n) {
  fbs_power_node_desc d;
  memset(&d, 0, sizeof d);
  CHECK(fbs_power_node_get(ctx, n, &d) == FBS_POWER_OK);
  return d.stored;
}

/* Membership as the sorted node-index list §7 items 1..3 ask for. */
static size_t members_of(fbs_power_context *ctx, fbs_power_network net, uint32_t *out, size_t cap) {
  fbs_power_node buf[64];
  size_t count = 0u, i;
  fbs_power_status st = fbs_power_network_members(ctx, net, buf, sizeof buf / sizeof buf[0], &count);
  CHECK(st == FBS_POWER_OK);
  if (st != FBS_POWER_OK || count > cap) return (size_t)-1;
  for (i = 0u; i < count; ++i) out[i] = IDX(buf[i]);
  return count;
}

static void expect_members(fbs_power_context *ctx, fbs_power_network net, const uint32_t *want,
                           size_t n) {
  uint32_t got[64];
  size_t count = members_of(ctx, net, got, sizeof got / sizeof got[0]);
  size_t i;
  CHECK(count == n);
  if (count != n) return;
  for (i = 0u; i < n; ++i) CHECK(got[i] == want[i]);
  /* ascending by construction */
  for (i = 1u; i < n; ++i) CHECK(got[i - 1u] < got[i]);
}

static unsigned char *serialize_alloc(fbs_power_context *ctx, size_t *len) {
  size_t need = fbs_power_serialized_size(ctx);
  unsigned char *buf = (unsigned char *)malloc(need ? need : 1u);
  size_t out = 0u;
  CHECK(buf != NULL);
  if (!buf) return NULL;
  CHECK(fbs_power_serialize(ctx, buf, need, &out) == FBS_POWER_OK);
  CHECK(out == need);
  *len = out;
  return buf;
}

static int blobs_equal(const unsigned char *a, size_t la, const unsigned char *b, size_t lb) {
  return la == lb && memcmp(a, b, la) == 0;
}

/* Rewrites the trailing crc32 so that a deliberately malformed blob is rejected
 * by a *field* rule rather than by the checksum. CRC-32/ISO-HDLC, independently
 * written here so the test does not share an implementation with the library. */
static void fix_crc(unsigned char *buf, size_t len) {
  uint32_t crc = 0xFFFFFFFFu;
  size_t i;
  unsigned bit;
  for (i = 0u; i + 4u < len; ++i) {
    crc ^= (uint32_t)buf[i];
    for (bit = 0u; bit < 8u; ++bit) crc = (crc & 1u) ? ((crc >> 1) ^ 0xEDB88320u) : (crc >> 1);
  }
  crc ^= 0xFFFFFFFFu;
  buf[len - 4u] = (unsigned char)(crc & 0xffu);
  buf[len - 3u] = (unsigned char)((crc >> 8) & 0xffu);
  buf[len - 2u] = (unsigned char)((crc >> 16) & 0xffu);
  buf[len - 1u] = (unsigned char)((crc >> 24) & 0xffu);
}

/* Membership compared without depending on which network id a rebuild hands
 * out: two contexts agree when every node sees the same member set. */
static void same_membership(fbs_power_context *a, fbs_power_context *b, const fbs_power_node *nodes,
                            unsigned n) {
  unsigned i;
  for (i = 0u; i < n; ++i) {
    uint32_t ma[64], mb[64];
    size_t ca = members_of(a, net_of(a, nodes[i]), ma, 64u);
    size_t cb = members_of(b, net_of(b, nodes[i]), mb, 64u);
    CHECK(ca == cb);
    if (ca == cb && ca != (size_t)-1) CHECK(memcmp(ma, mb, ca * sizeof(uint32_t)) == 0);
  }
}

static fbs_power_network_stats stats_of(fbs_power_context *ctx, fbs_power_network net) {
  fbs_power_network_stats s;
  memset(&s, 0, sizeof s);
  CHECK(fbs_power_network_stats_get(ctx, net, &s) == FBS_POWER_OK);
  return s;
}

/* ------------------------------------------------------------------------- */
/* §7.1 Merge with membership, and union by size                             */
/* ------------------------------------------------------------------------- */

static void test_merge(void) {
  fbs_power_context *ctx = make_ctx(NULL);
  fbs_power_node a = plain(ctx), b = plain(ctx), c = plain(ctx), d = plain(ctx);
  fbs_power_network na, nc, merged;
  const uint32_t ab[2] = {0u, 1u};
  const uint32_t cd[2] = {2u, 3u};
  const uint32_t abcd[4] = {0u, 1u, 2u, 3u};
  size_t count = 0u;
  fbs_power_network list[8];

  CHECK(fbs_power_network_count(ctx) == 4u); /* every lone node is a network */
  (void)add_edge(ctx, a, b);
  (void)add_edge(ctx, c, d);
  CHECK(fbs_power_network_count(ctx) == 2u);
  CHECK(fbs_power_node_count(ctx) == 4u);
  CHECK(fbs_power_edge_count(ctx) == 2u);

  na = net_of(ctx, a);
  nc = net_of(ctx, c);
  CHECK(na != nc);
  CHECK(net_of(ctx, b) == na);
  CHECK(net_of(ctx, d) == nc);
  expect_members(ctx, na, ab, 2u);
  expect_members(ctx, nc, cd, 2u);

  CHECK(fbs_power_network_list(ctx, list, 8u, &count) == FBS_POWER_OK);
  CHECK(count == 2u);
  CHECK(IDX(list[0]) < IDX(list[1])); /* ascending id order */

  /* Equal sizes: the lower network slot survives the tie. */
  (void)add_edge(ctx, b, c);
  CHECK(fbs_power_network_count(ctx) == 1u);
  merged = net_of(ctx, a);
  CHECK(merged == (IDX(na) < IDX(nc) ? na : nc));
  CHECK(net_of(ctx, b) == merged && net_of(ctx, c) == merged && net_of(ctx, d) == merged);
  expect_members(ctx, merged, abcd, 4u);
  /* the absorbed id is gone, not silently aliased (the pack's P5) */
  CHECK(fbs_power_network_members(ctx, (IDX(na) < IDX(nc) ? nc : na), list, 8u, &count) ==
        FBS_POWER_E_NOT_FOUND);
  fbs_power_destroy(ctx);

  /* Different sizes: the larger network survives, whichever endpoint is a. */
  {
    fbs_power_context *c2 = make_ctx(NULL);
    fbs_power_node n[5];
    fbs_power_network big, small_net;
    unsigned i;
    for (i = 0u; i < 5u; ++i) n[i] = plain(c2);
    (void)add_edge(c2, n[0], n[1]);
    (void)add_edge(c2, n[1], n[2]); /* {0,1,2} */
    (void)add_edge(c2, n[3], n[4]); /* {3,4}   */
    big = net_of(c2, n[0]);
    small_net = net_of(c2, n[3]);
    CHECK(stats_of(c2, big).node_count == 3u);
    CHECK(stats_of(c2, small_net).node_count == 2u);
    /* a is in the *smaller* network, so a naive "first caller absorbs" would
       keep the wrong id (the pack's merge rule). */
    (void)add_edge(c2, n[4], n[2]);
    CHECK(net_of(c2, n[0]) == big);
    CHECK(net_of(c2, n[4]) == big);
    CHECK(stats_of(c2, big).node_count == 5u);
    CHECK(fbs_power_network_members(c2, small_net, list, 8u, &count) == FBS_POWER_E_NOT_FOUND);
    fbs_power_destroy(c2);
  }
}

/* ------------------------------------------------------------------------- */
/* §7.2 Split with membership (the pack's P3)                                */
/* ------------------------------------------------------------------------- */

static void test_split(void) {
  fbs_power_context *ctx = make_ctx(NULL);
  fbs_power_node a = plain(ctx), b = plain(ctx), c = plain(ctx), d = plain(ctx);
  fbs_power_network n_b0 = net_of(ctx, b), n_c0 = net_of(ctx, c), n_d0 = net_of(ctx, d);
  fbs_power_network keep, fresh;
  fbs_power_edge e_ab, e_bc, e_cd;
  const uint32_t ab[2] = {0u, 1u};
  const uint32_t cd[2] = {2u, 3u};
  const uint32_t abcd[4] = {0u, 1u, 2u, 3u};
  fbs_power_node buf[8];
  size_t count = 0u;

  e_ab = add_edge(ctx, a, b);
  e_bc = add_edge(ctx, b, c);
  e_cd = add_edge(ctx, c, d);
  keep = net_of(ctx, a);
  CHECK(fbs_power_network_count(ctx) == 1u);
  expect_members(ctx, keep, abcd, 4u);

  /* the three absorbed ids are stale, not reusable as aliases */
  CHECK(fbs_power_network_members(ctx, n_b0, buf, 8u, &count) == FBS_POWER_E_NOT_FOUND);
  CHECK(fbs_power_network_members(ctx, n_c0, buf, 8u, &count) == FBS_POWER_E_NOT_FOUND);
  CHECK(fbs_power_network_members(ctx, n_d0, buf, 8u, &count) == FBS_POWER_E_NOT_FOUND);
  CHECK(fbs_power_tick_network(ctx, n_c0) == FBS_POWER_E_NOT_FOUND);

  /* Bridge removal: two networks, and the id stays with the side holding the
     lower-index endpoint of the removed edge (b, index 1). */
  CHECK(fbs_power_edge_remove(ctx, e_bc) == FBS_POWER_OK);
  CHECK(fbs_power_network_count(ctx) == 2u);
  CHECK(net_of(ctx, a) == keep);
  CHECK(net_of(ctx, b) == keep);
  fresh = net_of(ctx, c);
  CHECK(fresh != keep);
  CHECK(net_of(ctx, d) == fresh);
  expect_members(ctx, keep, ab, 2u);
  expect_members(ctx, fresh, cd, 2u);
  CHECK(fbs_power_edge_remove(ctx, e_bc) == FBS_POWER_E_NOT_FOUND); /* freed edge handle */
  CHECK(fbs_power_edge_count(ctx) == 2u);

  /* Reconnect: one network again, the tie keeping the lower slot. */
  (void)add_edge(ctx, b, c);
  CHECK(fbs_power_network_count(ctx) == 1u);
  CHECK(net_of(ctx, d) == keep);
  expect_members(ctx, keep, abcd, 4u);
  CHECK(fbs_power_network_members(ctx, fresh, buf, 8u, &count) == FBS_POWER_E_NOT_FOUND);
  (void)e_ab;
  (void)e_cd;
  fbs_power_destroy(ctx);

  /* Both ends of a removed edge are really unlinked (the pack's P6, a
     Disconnect that quietly does nothing). After the removal each former
     endpoint must be removable in turn without the other side's adjacency list
     still naming the dead edge, and the pair must be joinable again. */
  {
    fbs_power_context *c3 = make_ctx(NULL);
    fbs_power_node w = plain(c3), x = plain(c3), y = plain(c3);
    fbs_power_edge wx = add_edge(c3, w, x);
    fbs_power_edge xy = add_edge(c3, x, y);
    fbs_power_node ea = FBS_POWER_INVALID, eb = FBS_POWER_INVALID;
    CHECK(fbs_power_edge_remove(c3, wx) == FBS_POWER_OK);
    CHECK(fbs_power_network_count(c3) == 2u);
    CHECK(fbs_power_edge_count(c3) == 1u);
    /* w is isolated now; x still holds exactly its edge to y */
    CHECK(fbs_power_node_remove(c3, w) == FBS_POWER_OK);
    CHECK(fbs_power_edge_count(c3) == 1u);
    CHECK(fbs_power_network_count(c3) == 1u);
    CHECK(fbs_power_edge_get(c3, xy, &ea, &eb) == FBS_POWER_OK);
    CHECK(fbs_power_node_remove(c3, x) == FBS_POWER_OK);
    CHECK(fbs_power_edge_count(c3) == 0u);
    CHECK(fbs_power_network_count(c3) == 1u);
    CHECK(fbs_power_edge_get(c3, xy, &ea, &eb) == FBS_POWER_E_NOT_FOUND);
    /* the freed pair joins again from scratch, so no stale link survived */
    {
      fbs_power_node w2 = plain(c3);
      CHECK(fbs_power_network_count(c3) == 2u);
      (void)add_edge(c3, w2, y);
      CHECK(fbs_power_network_count(c3) == 1u);
      CHECK(fbs_power_edge_count(c3) == 1u);
    }
    fbs_power_destroy(c3);
  }

  /* Non-bridging removal from a cycle changes nothing at all. */
  {
    fbs_power_context *c2 = make_ctx(NULL);
    fbs_power_node x = plain(c2), y = plain(c2), z = plain(c2);
    fbs_power_edge e_ca;
    fbs_power_network before;
    const uint32_t xyz[3] = {0u, 1u, 2u};
    unsigned char *b0, *b1;
    size_t l0 = 0u, l1 = 0u;
    (void)add_edge(c2, x, y);
    (void)add_edge(c2, y, z);
    e_ca = add_edge(c2, z, x);
    before = net_of(c2, x);
    b0 = serialize_alloc(c2, &l0);
    CHECK(fbs_power_edge_remove(c2, e_ca) == FBS_POWER_OK);
    CHECK(fbs_power_network_count(c2) == 1u);
    CHECK(net_of(c2, x) == before);
    CHECK(net_of(c2, z) == before);
    expect_members(c2, before, xyz, 3u);
    b1 = serialize_alloc(c2, &l1);
    CHECK(l1 == l0 - 12u); /* exactly one edge record fewer */
    free(b0);
    free(b1);
    fbs_power_destroy(c2);
  }
}

/* ------------------------------------------------------------------------- */
/* §7.3 Node removal splits too                                              */
/* ------------------------------------------------------------------------- */

static void test_node_removal(void) {
  fbs_power_context *ctx = make_ctx(NULL);
  fbs_power_node a = plain(ctx), b = plain(ctx), c = plain(ctx);
  fbs_power_edge e_ab, e_bc;
  fbs_power_network keep, other;
  const uint32_t only_a[1] = {0u};
  const uint32_t only_c[1] = {2u};
  fbs_power_node ea = FBS_POWER_INVALID, eb = FBS_POWER_INVALID;

  e_ab = add_edge(ctx, a, b);
  e_bc = add_edge(ctx, b, c);
  keep = net_of(ctx, a);
  CHECK(fbs_power_network_count(ctx) == 1u);

  CHECK(fbs_power_node_remove(ctx, b) == FBS_POWER_OK);
  CHECK(fbs_power_node_count(ctx) == 2u);
  CHECK(fbs_power_edge_count(ctx) == 0u); /* every edge that touched b is gone */
  CHECK(fbs_power_edge_get(ctx, e_ab, &ea, &eb) == FBS_POWER_E_NOT_FOUND);
  CHECK(fbs_power_edge_get(ctx, e_bc, &ea, &eb) == FBS_POWER_E_NOT_FOUND);
  CHECK(fbs_power_network_count(ctx) == 2u);
  CHECK(net_of(ctx, a) == keep); /* the lowest surviving member keeps the id */
  other = net_of(ctx, c);
  CHECK(other != keep);
  expect_members(ctx, keep, only_a, 1u);
  expect_members(ctx, other, only_c, 1u);
  CHECK(fbs_power_node_network(ctx, b, &ea) == FBS_POWER_E_NOT_FOUND);

  /* Removing an isolated node frees its network. */
  {
    size_t count = 0u;
    CHECK(fbs_power_node_remove(ctx, c) == FBS_POWER_OK);
    CHECK(fbs_power_network_count(ctx) == 1u);
    CHECK(fbs_power_network_members(ctx, other, &ea, 1u, &count) == FBS_POWER_E_NOT_FOUND);
  }
  fbs_power_destroy(ctx);

  /* A hub of degree three splits into three networks in one call. */
  {
    fbs_power_context *c2 = make_ctx(NULL);
    fbs_power_node hub, leaf[3];
    unsigned i;
    leaf[0] = plain(c2);
    leaf[1] = plain(c2);
    leaf[2] = plain(c2);
    hub = plain(c2);
    for (i = 0u; i < 3u; ++i) (void)add_edge(c2, hub, leaf[i]);
    CHECK(fbs_power_network_count(c2) == 1u);
    CHECK(fbs_power_node_remove(c2, hub) == FBS_POWER_OK);
    CHECK(fbs_power_network_count(c2) == 3u);
    CHECK(fbs_power_edge_count(c2) == 0u);
    for (i = 0u; i < 3u; ++i) CHECK(stats_of(c2, net_of(c2, leaf[i])).node_count == 1u);
    CHECK(net_of(c2, leaf[0]) != net_of(c2, leaf[1]));
    CHECK(net_of(c2, leaf[1]) != net_of(c2, leaf[2]));
    fbs_power_destroy(c2);
  }
}

/* ------------------------------------------------------------------------- */
/* §7.4 Exact conservation per tick (the pack's P1)                          */
/* ------------------------------------------------------------------------- */

#define FUZZ_NODES 24u
#define FUZZ_TICKS 1000u

static void fuzz_one_policy(fbs_power_policy policy, uint32_t seed) {
  fbs_power_config cfg = fbs_power_config_default();
  fbs_power_context *ctx;
  fbs_power_node nodes[FUZZ_NODES];
  fbs_power_edge edges[64];
  unsigned edge_count = 0u, i;
  rng r;
  unsigned t;

  cfg.max_nodes = 32u;
  cfg.max_edges = 64u;
  cfg.max_networks = 32u;
  cfg.policy = policy;
  ctx = make_ctx(&cfg);
  rng_seed(&r, seed);

  for (i = 0u; i < FUZZ_NODES; ++i) {
    unsigned kind = rng_below(&r, 4u);
    int64_t prod = (kind == 0u || kind == 3u) ? (int64_t)rng_below(&r, 200u) : 0;
    int64_t dem = (kind == 1u || kind == 3u) ? (int64_t)rng_below(&r, 200u) : 0;
    int64_t cap = (kind == 2u || kind == 3u) ? (int64_t)rng_below(&r, 500u) : 0;
    int64_t sto = cap ? (int64_t)rng_below(&r, (unsigned)cap + 1u) : 0;
    nodes[i] = add_node(ctx, prod, dem, cap, sto, rng_below(&r, 4u), (uint64_t)i);
  }
  for (i = 0u; i < 18u; ++i) {
    unsigned x = rng_below(&r, FUZZ_NODES), y = rng_below(&r, FUZZ_NODES);
    fbs_power_edge e = FBS_POWER_INVALID;
    if (x == y) continue;
    if (fbs_power_edge_add(ctx, nodes[x], nodes[y], &e) == FBS_POWER_OK) edges[edge_count++] = e;
  }

  /* The inner assertions are aggregated into one flag each: a thousand ticks
     times two dozen nodes would otherwise drown the check count, and a failing
     flag still names the tick. */
  for (t = 0u; t < FUZZ_TICKS; ++t) {
    fbs_power_network list[32];
    fbs_power_network_stats before[32];
    size_t count = 0u, k;
    int api_ok = 1, conserve_ok = 1, storage_ok = 1, alloc_ok = 1, bounds_ok = 1;

    if (fbs_power_network_list(ctx, list, 32u, &count) != FBS_POWER_OK) api_ok = 0;
    for (k = 0u; k < count; ++k) {
      if (fbs_power_network_stats_get(ctx, list[k], &before[k]) != FBS_POWER_OK) api_ok = 0;
    }
    if (fbs_power_tick(ctx) != FBS_POWER_OK) api_ok = 0;

    for (k = 0u; k < count; ++k) {
      fbs_power_network_stats s;
      fbs_power_node mem[32];
      size_t mcount = 0u, m;
      int64_t sum = 0;
      if (fbs_power_network_stats_get(ctx, list[k], &s) != FBS_POWER_OK) {
        api_ok = 0;
        continue;
      }
      if (s.production + s.discharged != s.consumed + s.charged + s.wasted) conserve_ok = 0;
      if (s.stored - before[k].stored != s.charged - s.discharged) storage_ok = 0;
      if (s.supplied != s.production + s.discharged) conserve_ok = 0;
      if (s.satisfaction_den == 0u) conserve_ok = 0;
      if (fbs_power_network_members(ctx, list[k], mem, 32u, &mcount) != FBS_POWER_OK) {
        api_ok = 0;
        continue;
      }
      for (m = 0u; m < mcount; ++m) {
        fbs_power_node_desc d;
        int64_t got = 0;
        memset(&d, 0, sizeof d);
        if (fbs_power_node_get(ctx, mem[m], &d) != FBS_POWER_OK) api_ok = 0;
        if (fbs_power_node_allocation(ctx, mem[m], &got) != FBS_POWER_OK) api_ok = 0;
        sum += got;
        if (got < 0 || got > d.demand) alloc_ok = 0;
        if (d.stored < 0 || d.stored > d.capacity) bounds_ok = 0;
      }
      if (sum != s.consumed) alloc_ok = 0;
    }
    CHECK(api_ok);
    CHECK(conserve_ok);
    CHECK(storage_ok);
    CHECK(alloc_ok);
    CHECK(bounds_ok);

    /* Keep the topology and the rates moving so splits, merges and policy
       branches all run under the invariants. */
    if (t % 11u == 0u) {
      unsigned x = rng_below(&r, FUZZ_NODES);
      CHECK(fbs_power_node_set_rates(ctx, nodes[x], (int64_t)rng_below(&r, 200u),
                                     (int64_t)rng_below(&r, 200u)) == FBS_POWER_OK);
    }
    if (t % 37u == 0u && edge_count > 0u) {
      unsigned pick = rng_below(&r, edge_count);
      CHECK(fbs_power_edge_remove(ctx, edges[pick]) == FBS_POWER_OK);
      edges[pick] = edges[--edge_count];
    }
    if (t % 23u == 0u) {
      unsigned x = rng_below(&r, FUZZ_NODES), y = rng_below(&r, FUZZ_NODES);
      fbs_power_edge e = FBS_POWER_INVALID;
      if (x != y && edge_count < 64u &&
          fbs_power_edge_add(ctx, nodes[x], nodes[y], &e) == FBS_POWER_OK)
        edges[edge_count++] = e;
    }
  }
  fbs_power_destroy(ctx);
}

static void test_conservation(void) {
  fuzz_one_policy(FBS_POWER_POLICY_PROPORTIONAL, 0x9E3779B9u);
  fuzz_one_policy(FBS_POWER_POLICY_PRIORITY, 0x9E3779B9u);
  fuzz_one_policy(FBS_POWER_POLICY_ALL_OR_NOTHING, 0x9E3779B9u);
}

/* ------------------------------------------------------------------------- */
/* §7.5 Deterministic distribution, order independent (the pack's P2)        */
/* ------------------------------------------------------------------------- */

/* supply 7 over three consumers demanding 3 each: floor(3*7/9) = 2 with an
 * equal remainder of 3 everywhere, so the single spare unit goes to the lowest
 * node index. Hand computed, not self-consistency. */
static void test_largest_remainder(void) {
  fbs_power_context *ctx = make_ctx(NULL);
  fbs_power_node p = add_node(ctx, 7, 0, 0, 0, 0u, 0u);
  fbs_power_node c0 = add_node(ctx, 0, 3, 0, 0, 0u, 1u);
  fbs_power_node c1 = add_node(ctx, 0, 3, 0, 0, 0u, 2u);
  fbs_power_node c2 = add_node(ctx, 0, 3, 0, 0, 0u, 3u);
  fbs_power_network_stats s;
  (void)add_edge(ctx, p, c0);
  (void)add_edge(ctx, c0, c1);
  (void)add_edge(ctx, c1, c2);
  CHECK(fbs_power_tick(ctx) == FBS_POWER_OK);
  CHECK(alloc_of(ctx, c0) == 3);
  CHECK(alloc_of(ctx, c1) == 2);
  CHECK(alloc_of(ctx, c2) == 2);
  CHECK(alloc_of(ctx, p) == 0);
  s = stats_of(ctx, net_of(ctx, p));
  CHECK(s.production == 7 && s.demand == 9);
  CHECK(s.consumed == 7 && s.charged == 0 && s.discharged == 0 && s.wasted == 0);
  CHECK(s.supplied == 7);
  CHECK(s.satisfaction_num == 7u && s.satisfaction_den == 9u);
  fbs_power_destroy(ctx);
}

/* The same logical network built in several insertion orders. Node identity is
 * the tag, and the demands are chosen so no two remainders tie: the outcome is
 * then a function of the graph alone. */
typedef struct {
  int64_t production, demand;
  uint64_t tag;
} order_node;

static void build_in_order(fbs_power_context *ctx, const order_node *spec, const unsigned *node_ord,
                           const unsigned *edge_ord, unsigned n, unsigned edges,
                           fbs_power_node *by_tag) {
  fbs_power_node made[8];
  unsigned i;
  for (i = 0u; i < n; ++i) {
    unsigned k = node_ord[i];
    made[k] = add_node(ctx, spec[k].production, spec[k].demand, 0, 0, 0u, spec[k].tag);
    by_tag[k] = made[k];
  }
  /* A path 0-1-2-...-(n-1), inserted in the given order. */
  for (i = 0u; i < edges; ++i) {
    unsigned k = edge_ord[i];
    (void)add_edge(ctx, made[k], made[k + 1u]);
  }
}

static void test_order_independence(void) {
  static const order_node spec[4] = {{5, 0, 100u}, {0, 1, 101u}, {0, 2, 102u}, {0, 3, 103u}};
  static const unsigned fwd_nodes[4] = {0u, 1u, 2u, 3u};
  static const unsigned rev_nodes[4] = {3u, 2u, 1u, 0u};
  static const unsigned shuffled[4] = {2u, 0u, 3u, 1u};
  static const unsigned fwd_edges[3] = {0u, 1u, 2u};
  static const unsigned rev_edges[3] = {2u, 1u, 0u};
  static const unsigned mix_edges[3] = {1u, 2u, 0u};
  const unsigned *node_orders[3];
  const unsigned *edge_orders[3];
  int64_t reference[4];
  unsigned run, i;

  node_orders[0] = fwd_nodes;
  node_orders[1] = rev_nodes;
  node_orders[2] = shuffled;
  edge_orders[0] = fwd_edges;
  edge_orders[1] = rev_edges;
  edge_orders[2] = mix_edges;

  for (run = 0u; run < 3u; ++run) {
    fbs_power_context *ctx = make_ctx(NULL);
    fbs_power_node by_tag[4];
    fbs_power_network_stats s;
    build_in_order(ctx, spec, node_orders[run], edge_orders[run], 4u, 3u, by_tag);
    CHECK(fbs_power_network_count(ctx) == 1u);
    CHECK(fbs_power_tick(ctx) == FBS_POWER_OK);
    s = stats_of(ctx, net_of(ctx, by_tag[0]));
    CHECK(s.production == 5 && s.demand == 6 && s.consumed == 5);
    CHECK(s.satisfaction_num == 5u && s.satisfaction_den == 6u);
    /* floor(1*5/6)=0 r5, floor(2*5/6)=1 r4, floor(3*5/6)=2 r3; two spare units
       to the two largest remainders: (1, 2, 2). */
    CHECK(alloc_of(ctx, by_tag[1]) == 1);
    CHECK(alloc_of(ctx, by_tag[2]) == 2);
    CHECK(alloc_of(ctx, by_tag[3]) == 2);
    for (i = 0u; i < 4u; ++i) {
      int64_t got = alloc_of(ctx, by_tag[i]);
      if (run == 0u)
        reference[i] = got;
      else
        CHECK(got == reference[i]);
    }
    fbs_power_destroy(ctx);
  }

  /* Edge insertion order alone must not change membership, allocations or
     statistics. The serialized bytes are deliberately *not* asserted equal
     here: an edge record carries its slot index, and slots are handed out in
     insertion order so that an edge handle survives a save. Everything the
     model computes is invariant; only the names of the edges differ. */
  {
    fbs_power_context *a = make_ctx(NULL);
    fbs_power_context *b = make_ctx(NULL);
    fbs_power_node by_tag_a[4], by_tag_b[4];
    fbs_power_network_stats sa, sb;
    unsigned char *ba, *bb;
    size_t la = 0u, lb = 0u;
    build_in_order(a, spec, fwd_nodes, fwd_edges, 4u, 3u, by_tag_a);
    build_in_order(b, spec, fwd_nodes, rev_edges, 4u, 3u, by_tag_b);
    CHECK(fbs_power_tick(a) == FBS_POWER_OK);
    CHECK(fbs_power_tick(b) == FBS_POWER_OK);
    for (i = 0u; i < 4u; ++i) {
      CHECK(by_tag_a[i] == by_tag_b[i]); /* identical node slots */
      CHECK(alloc_of(a, by_tag_a[i]) == alloc_of(b, by_tag_b[i]));
    }
    same_membership(a, b, by_tag_a, 4u);
    sa = stats_of(a, net_of(a, by_tag_a[0]));
    sb = stats_of(b, net_of(b, by_tag_b[0]));
    CHECK(memcmp(&sa, &sb, sizeof sa) == 0);
    /* the node half of the blob is byte identical; only the edge records,
       which name slots, are permuted */
    ba = serialize_alloc(a, &la);
    bb = serialize_alloc(b, &lb);
    CHECK(la == lb);
    CHECK(memcmp(ba, bb, 24u + 4u * 48u) == 0);
    free(ba);
    free(bb);
    fbs_power_destroy(a);
    fbs_power_destroy(b);
  }
}

/* ------------------------------------------------------------------------- */
/* §7.6 Priority and all-or-nothing                                          */
/* ------------------------------------------------------------------------- */

/* One producer (7) and three consumers (3 each) on one network, so the three
 * policies can be compared on the same fixture. */
static fbs_power_context *policy_fixture(fbs_power_policy policy, uint32_t p0, uint32_t p1,
                                         uint32_t p2, fbs_power_node *out) {
  fbs_power_context *ctx = make_policy_ctx(policy);
  out[0] = add_node(ctx, 7, 0, 0, 0, 0u, 0u);
  out[1] = add_node(ctx, 0, 3, 0, 0, p0, 1u);
  out[2] = add_node(ctx, 0, 3, 0, 0, p1, 2u);
  out[3] = add_node(ctx, 0, 3, 0, 0, p2, 3u);
  (void)add_edge(ctx, out[0], out[1]);
  (void)add_edge(ctx, out[1], out[2]);
  (void)add_edge(ctx, out[2], out[3]);
  return ctx;
}

static void test_policies(void) {
  fbs_power_node n[4];
  fbs_power_network_stats s;
  fbs_power_context *ctx;

  /* PROPORTIONAL is priority blind. */
  ctx = policy_fixture(FBS_POWER_POLICY_PROPORTIONAL, 9u, 0u, 4u, n);
  CHECK(fbs_power_tick(ctx) == FBS_POWER_OK);
  CHECK(alloc_of(ctx, n[1]) == 3 && alloc_of(ctx, n[2]) == 2 && alloc_of(ctx, n[3]) == 2);
  fbs_power_destroy(ctx);

  /* PRIORITY: ascending (priority, node index). n[2] first, then n[1], then the
     partial remainder to n[3]. */
  ctx = policy_fixture(FBS_POWER_POLICY_PRIORITY, 5u, 1u, 5u, n);
  CHECK(fbs_power_tick(ctx) == FBS_POWER_OK);
  CHECK(alloc_of(ctx, n[1]) == 3);
  CHECK(alloc_of(ctx, n[2]) == 3);
  CHECK(alloc_of(ctx, n[3]) == 1);
  s = stats_of(ctx, net_of(ctx, n[0]));
  CHECK(s.consumed == 7 && s.wasted == 0 && s.charged == 0 && s.discharged == 0);
  CHECK(s.satisfaction_num == 7u && s.satisfaction_den == 9u);
  fbs_power_destroy(ctx);

  /* PRIORITY with equal priorities falls back to the node index. */
  ctx = policy_fixture(FBS_POWER_POLICY_PRIORITY, 2u, 2u, 2u, n);
  CHECK(fbs_power_tick(ctx) == FBS_POWER_OK);
  CHECK(alloc_of(ctx, n[1]) == 3 && alloc_of(ctx, n[2]) == 3 && alloc_of(ctx, n[3]) == 1);
  fbs_power_destroy(ctx);

  /* ALL_OR_NOTHING: 7 < 9, so nobody is served and the production is wasted.
     Nothing is drawn from storage for a blackout. */
  ctx = policy_fixture(FBS_POWER_POLICY_ALL_OR_NOTHING, 0u, 0u, 0u, n);
  CHECK(fbs_power_tick(ctx) == FBS_POWER_OK);
  CHECK(alloc_of(ctx, n[1]) == 0 && alloc_of(ctx, n[2]) == 0 && alloc_of(ctx, n[3]) == 0);
  s = stats_of(ctx, net_of(ctx, n[0]));
  CHECK(s.consumed == 0 && s.charged == 0 && s.discharged == 0 && s.wasted == 7);
  CHECK(s.supplied == 7);
  CHECK(s.satisfaction_num == 0u && s.satisfaction_den == 1u);
  /* Raise supply to exactly the demand and everyone is served in full. */
  CHECK(fbs_power_node_set_rates(ctx, n[0], 9, 0) == FBS_POWER_OK);
  CHECK(fbs_power_tick(ctx) == FBS_POWER_OK);
  CHECK(alloc_of(ctx, n[1]) == 3 && alloc_of(ctx, n[2]) == 3 && alloc_of(ctx, n[3]) == 3);
  s = stats_of(ctx, net_of(ctx, n[0]));
  CHECK(s.consumed == 9 && s.wasted == 0);
  CHECK(s.satisfaction_num == 1u && s.satisfaction_den == 1u);
  fbs_power_destroy(ctx);

  /* ALL_OR_NOTHING with a battery that can just close the gap: the batteries
     discharge and everyone is served; one unit short and nothing moves. */
  {
    fbs_power_node a, b, bat;
    ctx = make_policy_ctx(FBS_POWER_POLICY_ALL_OR_NOTHING);
    a = add_node(ctx, 4, 0, 0, 0, 0u, 0u);
    b = add_node(ctx, 0, 10, 0, 0, 0u, 1u);
    bat = add_node(ctx, 0, 0, 100, 6, 0u, 2u);
    (void)add_edge(ctx, a, b);
    (void)add_edge(ctx, b, bat);
    CHECK(fbs_power_tick(ctx) == FBS_POWER_OK);
    CHECK(alloc_of(ctx, b) == 10);
    CHECK(stored_of(ctx, bat) == 0);
    s = stats_of(ctx, net_of(ctx, a));
    CHECK(s.discharged == 6 && s.consumed == 10 && s.wasted == 0 && s.charged == 0);
    /* now storage is empty: 4 < 10, blackout, and the 4 units are wasted */
    CHECK(fbs_power_tick(ctx) == FBS_POWER_OK);
    CHECK(alloc_of(ctx, b) == 0);
    CHECK(stored_of(ctx, bat) == 0);
    s = stats_of(ctx, net_of(ctx, a));
    CHECK(s.discharged == 0 && s.consumed == 0 && s.wasted == 4);
    fbs_power_destroy(ctx);
  }
}

/* PW-1. An all-or-nothing blackout must drain *nothing*: the batteries could
 * have contributed, the policy refused to serve anyone, and so no energy leaves
 * storage. This is the live witness for the one place where the header's tick
 * order is refined (see the file comment in src/power.c). It is written so the
 * obvious mutant — discharging min(deficit, stored) regardless of what the
 * policy consumed — fails on stored, on discharged and on wasted. */
static void test_blackout_drains_nothing(void) {
  fbs_power_context *ctx = make_policy_ctx(FBS_POWER_POLICY_ALL_OR_NOTHING);
  fbs_power_node p = add_node(ctx, 4, 0, 0, 0, 0u, 0u);
  fbs_power_node c = add_node(ctx, 0, 10, 0, 0, 0u, 1u);
  fbs_power_node bat = add_node(ctx, 0, 0, 100, 3, 0u, 2u);
  fbs_power_network_stats s;
  unsigned t;

  (void)add_edge(ctx, p, c);
  (void)add_edge(ctx, c, bat);
  /* production 4 + storage 3 == 7 < demand 10, so the gap cannot be closed. */
  CHECK(fbs_power_tick(ctx) == FBS_POWER_OK);
  CHECK(stored_of(ctx, bat) == 3); /* the mutant leaves 0 here */
  CHECK(alloc_of(ctx, c) == 0);
  s = stats_of(ctx, net_of(ctx, p));
  CHECK(s.discharged == 0); /* the mutant reports 3 */
  CHECK(s.consumed == 0);
  CHECK(s.charged == 0);
  CHECK(s.wasted == 4); /* == production; the mutant reports 7 */
  CHECK(s.supplied == 4);
  CHECK(s.production + s.discharged == s.consumed + s.charged + s.wasted);
  CHECK(s.satisfaction_num == 0u && s.satisfaction_den == 1u);

  /* And it stays put: a blackout is a steady state, not a slow drain. */
  for (t = 0u; t < 50u; ++t) CHECK(fbs_power_tick(ctx) == FBS_POWER_OK);
  CHECK(stored_of(ctx, bat) == 3);

  /* One unit more of storage closes the gap, and then it does discharge —
     so the test above is not passing merely because discharge never fires. */
  CHECK(fbs_power_node_set_stored(ctx, bat, 6) == FBS_POWER_OK);
  CHECK(fbs_power_tick(ctx) == FBS_POWER_OK);
  CHECK(stored_of(ctx, bat) == 0);
  CHECK(alloc_of(ctx, c) == 10);
  s = stats_of(ctx, net_of(ctx, p));
  CHECK(s.discharged == 6 && s.consumed == 10 && s.wasted == 0);
  fbs_power_destroy(ctx);
}

/* fbs_power_tick_network advances one network and leaves the others alone. */
static void test_tick_network(void) {
  fbs_power_context *ctx = make_ctx(NULL);
  fbs_power_node p0 = add_node(ctx, 4, 0, 0, 0, 0u, 0u);
  fbs_power_node c0 = add_node(ctx, 0, 10, 0, 0, 0u, 1u);
  fbs_power_node p1 = add_node(ctx, 9, 0, 0, 0, 0u, 2u);
  fbs_power_node c1 = add_node(ctx, 0, 10, 0, 0, 0u, 3u);
  fbs_power_node bat = add_node(ctx, 0, 0, 100, 0, 0u, 4u);
  fbs_power_network_stats s;
  (void)add_edge(ctx, p0, c0);
  (void)add_edge(ctx, p1, c1);
  (void)add_edge(ctx, c1, bat);
  CHECK(fbs_power_network_count(ctx) == 2u);

  CHECK(fbs_power_tick_network(ctx, net_of(ctx, p0)) == FBS_POWER_OK);
  CHECK(alloc_of(ctx, c0) == 4);
  CHECK(alloc_of(ctx, c1) == 0); /* the other network was not advanced */
  s = stats_of(ctx, net_of(ctx, c1));
  CHECK(s.consumed == 0 && s.supplied == 0 && s.satisfaction_den == 1u);

  CHECK(fbs_power_tick_network(ctx, net_of(ctx, c1)) == FBS_POWER_OK);
  CHECK(alloc_of(ctx, c1) == 9);
  CHECK(alloc_of(ctx, c0) == 4); /* and the first one was not advanced twice */
  fbs_power_destroy(ctx);
}

/* ------------------------------------------------------------------------- */
/* §7.7 Storage limits                                                       */
/* ------------------------------------------------------------------------- */

static void test_storage(void) {
  /* Charging stops exactly at capacity; the surplus is wasted. */
  {
    fbs_power_context *ctx = make_ctx(NULL);
    fbs_power_node p = add_node(ctx, 100, 0, 0, 0, 0u, 0u);
    fbs_power_node bat = add_node(ctx, 0, 0, 30, 0, 0u, 1u);
    fbs_power_network_stats s;
    (void)add_edge(ctx, p, bat);
    CHECK(fbs_power_tick(ctx) == FBS_POWER_OK);
    CHECK(stored_of(ctx, bat) == 30);
    s = stats_of(ctx, net_of(ctx, p));
    CHECK(s.charged == 30 && s.wasted == 70 && s.consumed == 0 && s.discharged == 0);
    CHECK(s.satisfaction_num == 0u && s.satisfaction_den == 1u); /* demand 0 -> 0/1 */
    CHECK(fbs_power_tick(ctx) == FBS_POWER_OK);
    CHECK(stored_of(ctx, bat) == 30);
    s = stats_of(ctx, net_of(ctx, p));
    CHECK(s.charged == 0 && s.wasted == 100);
    fbs_power_destroy(ctx);
  }

  /* Discharging stops exactly at 0 and the shortfall shows up as a reduced
     allocation, not as a negative store. */
  {
    fbs_power_context *ctx = make_ctx(NULL);
    fbs_power_node c = add_node(ctx, 0, 10, 0, 0, 0u, 0u);
    fbs_power_node bat = add_node(ctx, 0, 0, 100, 3, 0u, 1u);
    fbs_power_network_stats s;
    (void)add_edge(ctx, c, bat);
    CHECK(fbs_power_tick(ctx) == FBS_POWER_OK);
    CHECK(alloc_of(ctx, c) == 3);
    CHECK(stored_of(ctx, bat) == 0);
    s = stats_of(ctx, net_of(ctx, c));
    CHECK(s.discharged == 3 && s.consumed == 3 && s.charged == 0 && s.wasted == 0);
    CHECK(s.satisfaction_num == 3u && s.satisfaction_den == 10u);
    CHECK(fbs_power_tick(ctx) == FBS_POWER_OK);
    CHECK(alloc_of(ctx, c) == 0);
    CHECK(stored_of(ctx, bat) == 0);
    fbs_power_destroy(ctx);
  }

  /* Two batteries with different stored amounts discharge proportionally:
     stored 5 and 7, deficit 5, floor(5*5/12)=2 r1 and floor(7*5/12)=2 r11, so
     the spare unit goes to the larger remainder: 2 and 3. */
  {
    fbs_power_context *ctx = make_ctx(NULL);
    fbs_power_node c = add_node(ctx, 0, 5, 0, 0, 0u, 0u);
    fbs_power_node b0 = add_node(ctx, 0, 0, 100, 5, 0u, 1u);
    fbs_power_node b1 = add_node(ctx, 0, 0, 100, 7, 0u, 2u);
    fbs_power_network_stats s;
    (void)add_edge(ctx, c, b0);
    (void)add_edge(ctx, b0, b1);
    CHECK(fbs_power_tick(ctx) == FBS_POWER_OK);
    CHECK(alloc_of(ctx, c) == 5);
    CHECK(stored_of(ctx, b0) == 3);
    CHECK(stored_of(ctx, b1) == 4);
    s = stats_of(ctx, net_of(ctx, c));
    CHECK(s.discharged == 5 && s.consumed == 5 && s.stored == 7);
    fbs_power_destroy(ctx);
  }

  /* Charging is apportioned by *free* capacity, with the same rule: free 5 and
     7, surplus 5 -> 2 and 3. */
  {
    fbs_power_context *ctx = make_ctx(NULL);
    fbs_power_node p = add_node(ctx, 5, 0, 0, 0, 0u, 0u);
    fbs_power_node b0 = add_node(ctx, 0, 0, 10, 5, 0u, 1u);
    fbs_power_node b1 = add_node(ctx, 0, 0, 10, 3, 0u, 2u);
    (void)add_edge(ctx, p, b0);
    (void)add_edge(ctx, b0, b1);
    CHECK(fbs_power_tick(ctx) == FBS_POWER_OK);
    CHECK(stored_of(ctx, b0) == 7);
    CHECK(stored_of(ctx, b1) == 6);
    fbs_power_destroy(ctx);
  }

  /* A long charge/discharge cycle never leaves [0, capacity]. */
  {
    fbs_power_context *ctx = make_ctx(NULL);
    fbs_power_node p = add_node(ctx, 13, 0, 0, 0, 0u, 0u);
    fbs_power_node c = add_node(ctx, 0, 0, 0, 0, 0u, 1u);
    fbs_power_node bat = add_node(ctx, 0, 0, 40, 0, 0u, 2u);
    unsigned t;
    int ok = 1;
    (void)add_edge(ctx, p, c);
    (void)add_edge(ctx, c, bat);
    for (t = 0u; t < 200u; ++t) {
      int64_t sto;
      CHECK(fbs_power_node_set_rates(ctx, c, 0, (t / 7u) % 2u ? 40 : 0) == FBS_POWER_OK);
      CHECK(fbs_power_tick(ctx) == FBS_POWER_OK);
      sto = stored_of(ctx, bat);
      if (sto < 0 || sto > 40) ok = 0;
    }
    CHECK(ok);
    fbs_power_destroy(ctx);
  }

  /* set_stored / set_capacity are checked, never clamped. */
  {
    fbs_power_context *ctx = make_ctx(NULL);
    fbs_power_node bat = add_node(ctx, 0, 0, 10, 4, 0u, 0u);
    CHECK(fbs_power_node_set_stored(ctx, bat, 11) == FBS_POWER_E_RANGE);
    CHECK(fbs_power_node_set_stored(ctx, bat, -1) == FBS_POWER_E_RANGE);
    CHECK(stored_of(ctx, bat) == 4);
    CHECK(fbs_power_node_set_stored(ctx, bat, 10) == FBS_POWER_OK);
    CHECK(fbs_power_node_set_capacity(ctx, bat, 9) == FBS_POWER_E_RANGE); /* below stored */
    CHECK(fbs_power_node_set_capacity(ctx, bat, -1) == FBS_POWER_E_RANGE);
    CHECK(fbs_power_node_set_capacity(ctx, bat, 10) == FBS_POWER_OK);
    CHECK(fbs_power_node_set_capacity(ctx, bat, 50) == FBS_POWER_OK);
    CHECK(fbs_power_node_set_rates(ctx, bat, -1, 0) == FBS_POWER_E_RANGE);
    CHECK(fbs_power_node_set_rates(ctx, bat, 0, -1) == FBS_POWER_E_RANGE);
    fbs_power_destroy(ctx);
  }
}

/* ------------------------------------------------------------------------- */
/* §7.8 Capacity and allocator failures                                      */
/* ------------------------------------------------------------------------- */

static void test_capacity_failures(void) {
  fbs_power_config cfg = fbs_power_config_default();
  fbs_power_context *ctx;
  fbs_power_node a, b, c;
  fbs_power_node made = 0xA5A5A5A5u;
  fbs_power_edge e = 0xA5A5A5A5u;
  unsigned char *b0, *b1;
  size_t l0 = 0u, l1 = 0u;

  /* max_nodes exhaustion leaves the graph exactly as it was. */
  cfg.max_nodes = 2u;
  cfg.max_edges = 4u;
  cfg.max_networks = 4u;
  ctx = make_ctx(&cfg);
  a = plain(ctx);
  b = plain(ctx);
  b0 = serialize_alloc(ctx, &l0);
  {
    fbs_power_node_desc d;
    memset(&d, 0, sizeof d);
    CHECK(fbs_power_node_add(ctx, &d, &made) == FBS_POWER_E_FULL);
    CHECK(made == 0xA5A5A5A5u); /* output untouched on error */
  }
  b1 = serialize_alloc(ctx, &l1);
  CHECK(blobs_equal(b0, l0, b1, l1));
  free(b0);
  free(b1);
  fbs_power_destroy(ctx);

  /* max_edges exhaustion, same witness. */
  cfg.max_nodes = 4u;
  cfg.max_edges = 1u;
  ctx = make_ctx(&cfg);
  a = plain(ctx);
  b = plain(ctx);
  c = plain(ctx);
  (void)add_edge(ctx, a, b);
  b0 = serialize_alloc(ctx, &l0);
  CHECK(fbs_power_edge_add(ctx, b, c, &e) == FBS_POWER_E_FULL);
  CHECK(e == 0xA5A5A5A5u);
  b1 = serialize_alloc(ctx, &l1);
  CHECK(blobs_equal(b0, l0, b1, l1));
  free(b0);
  free(b1);
  fbs_power_destroy(ctx);

  /* max_networks exhaustion: every live node needs a network, so the second
     lone node cannot be created. */
  cfg.max_nodes = 4u;
  cfg.max_edges = 4u;
  cfg.max_networks = 1u;
  ctx = make_ctx(&cfg);
  a = plain(ctx);
  b0 = serialize_alloc(ctx, &l0);
  {
    fbs_power_node_desc d;
    memset(&d, 0, sizeof d);
    made = 0xA5A5A5A5u;
    CHECK(fbs_power_node_add(ctx, &d, &made) == FBS_POWER_E_FULL);
    CHECK(made == 0xA5A5A5A5u);
  }
  b1 = serialize_alloc(ctx, &l1);
  CHECK(blobs_equal(b0, l0, b1, l1));
  CHECK(fbs_power_node_count(ctx) == 1u);
  free(b0);
  free(b1);
  (void)a;
  fbs_power_destroy(ctx);

  /* A split that would need one network more than the table holds is refused
     and changes nothing. The state is reached through a blob, because building
     it by hand would need the same network budget. */
  {
    fbs_power_context *src = make_ctx(NULL);
    fbs_power_context *dst = NULL;
    fbs_power_config small = fbs_power_config_default();
    fbs_power_node w = plain(src), x = plain(src), y = plain(src), z = plain(src);
    fbs_power_edge bridge;
    unsigned char *blob;
    size_t len = 0u;
    (void)add_edge(src, w, x);
    (void)add_edge(src, y, z);
    blob = serialize_alloc(src, &len);
    small.max_nodes = 4u;
    small.max_edges = 4u;
    small.max_networks = 2u;
    CHECK(fbs_power_deserialize(blob, len, &small, NULL, &dst) == FBS_POWER_OK);
    CHECK(fbs_power_network_count(dst) == 2u);
    {
      fbs_power_edge list[4];
      size_t count = 0u;
      unsigned char *d0, *d1;
      size_t dl0 = 0u, dl1 = 0u;
      CHECK(fbs_power_edge_list(dst, list, 4u, &count) == FBS_POWER_OK);
      CHECK(count == 2u);
      bridge = list[0];
      d0 = serialize_alloc(dst, &dl0);
      CHECK(fbs_power_edge_remove(dst, bridge) == FBS_POWER_E_FULL);
      d1 = serialize_alloc(dst, &dl1);
      CHECK(blobs_equal(d0, dl0, d1, dl1));
      CHECK(fbs_power_network_count(dst) == 2u);
      free(d0);
      free(d1);
    }
    /* And the same blob refuses to load at all when the table is one short. */
    {
      fbs_power_context *too_small = (fbs_power_context *)0xA5;
      small.max_networks = 1u;
      CHECK(fbs_power_deserialize(blob, len, &small, NULL, &too_small) == FBS_POWER_E_FULL);
      CHECK(too_small == (fbs_power_context *)0xA5);
      small.max_networks = 2u;
      small.max_nodes = 3u;
      CHECK(fbs_power_deserialize(blob, len, &small, NULL, &too_small) == FBS_POWER_E_FULL);
      small.max_nodes = 4u;
      small.max_edges = 1u;
      CHECK(fbs_power_deserialize(blob, len, &small, NULL, &too_small) == FBS_POWER_E_FULL);
      CHECK(too_small == (fbs_power_context *)0xA5);
    }
    free(blob);
    fbs_power_destroy(dst);
    fbs_power_destroy(src);
  }
}

static void test_allocator(void) {
  counting_alloc ca;
  fbs_power_allocator alloc;
  fbs_power_context *ctx = (fbs_power_context *)0xA5;
  fbs_power_config cfg = fbs_power_config_default();

  memset(&ca, 0, sizeof ca);
  ca.budget = 0;
  alloc.alloc = ca_alloc;
  alloc.free = ca_free;
  alloc.user = &ca;
  CHECK(fbs_power_create(&cfg, &alloc, &ctx) == FBS_POWER_E_MEMORY);
  CHECK(ctx == (fbs_power_context *)0xA5);
  CHECK(ca.allocs == 0 && ca.frees == 0);

  /* One allocation for the whole context, one free, nothing else. */
  ca.budget = -1;
  ctx = NULL;
  CHECK(fbs_power_create(&cfg, &alloc, &ctx) == FBS_POWER_OK);
  CHECK(ca.allocs == 1);
  CHECK(ctx != NULL);
  CHECK(fbs_power_memory(ctx) == ca.bytes);
  {
    fbs_power_node n[8];
    unsigned i;
    for (i = 0u; i < 8u; ++i) n[i] = plain(ctx);
    for (i = 1u; i < 8u; ++i) (void)add_edge(ctx, n[0], n[i]);
    CHECK(fbs_power_tick(ctx) == FBS_POWER_OK);
    CHECK(ca.allocs == 1); /* no allocation after create */
  }
  fbs_power_destroy(ctx);
  CHECK(ca.frees == 1);

  /* A NULL allocator function is a bad argument, not a crash. */
  {
    fbs_power_allocator bad;
    fbs_power_context *out = (fbs_power_context *)0xA5;
    bad.alloc = NULL;
    bad.free = ca_free;
    bad.user = &ca;
    CHECK(fbs_power_create(&cfg, &bad, &out) == FBS_POWER_E_INVALID);
    bad.alloc = ca_alloc;
    bad.free = NULL;
    CHECK(fbs_power_create(&cfg, &bad, &out) == FBS_POWER_E_INVALID);
    CHECK(out == (fbs_power_context *)0xA5);
  }
}

/* ------------------------------------------------------------------------- */
/* §7.9 Handle hygiene                                                       */
/* ------------------------------------------------------------------------- */

static void test_handles(void) {
  fbs_power_context *ctx = make_ctx(NULL);
  fbs_power_node a = plain(ctx), b, reused;
  fbs_power_node_desc d;
  fbs_power_network net = 0xA5A5A5A5u;
  fbs_power_edge e = FBS_POWER_INVALID, dup = 0xA5A5A5A5u;
  int64_t got = -7;

  CHECK(GEN(a) >= 1u); /* generation is never 0: the null handle is not live */
  CHECK(IDX(a) == 0u);
  b = plain(ctx);

  /* Free the slot and take it again: the old handle must not resolve. */
  CHECK(fbs_power_node_remove(ctx, a) == FBS_POWER_OK);
  reused = plain(ctx);
  CHECK(IDX(reused) == IDX(a));
  CHECK(GEN(reused) != GEN(a));
  CHECK(reused != a);
  memset(&d, 0xA5, sizeof d);
  CHECK(fbs_power_node_get(ctx, a, &d) == FBS_POWER_E_NOT_FOUND);
  CHECK(untouched(&d, sizeof d, 0xA5));
  CHECK(fbs_power_node_remove(ctx, a) == FBS_POWER_E_NOT_FOUND);
  CHECK(fbs_power_node_network(ctx, a, &net) == FBS_POWER_E_NOT_FOUND);
  CHECK(net == 0xA5A5A5A5u);
  CHECK(fbs_power_node_allocation(ctx, a, &got) == FBS_POWER_E_NOT_FOUND);
  CHECK(got == -7);
  CHECK(fbs_power_node_set_rates(ctx, a, 1, 1) == FBS_POWER_E_NOT_FOUND);
  CHECK(fbs_power_node_set_stored(ctx, a, 0) == FBS_POWER_E_NOT_FOUND);
  CHECK(fbs_power_node_set_capacity(ctx, a, 0) == FBS_POWER_E_NOT_FOUND);
  CHECK(fbs_power_node_set_priority(ctx, a, 0u) == FBS_POWER_E_NOT_FOUND);
  CHECK(fbs_power_edge_add(ctx, a, b, &dup) == FBS_POWER_E_NOT_FOUND);
  CHECK(dup == 0xA5A5A5A5u);

  /* Duplicate edges (the pack's P7), in both directions. */
  e = add_edge(ctx, reused, b);
  CHECK(fbs_power_edge_add(ctx, reused, b, &dup) == FBS_POWER_E_EXISTS);
  CHECK(fbs_power_edge_add(ctx, b, reused, &dup) == FBS_POWER_E_EXISTS);
  CHECK(dup == 0xA5A5A5A5u);
  CHECK(fbs_power_edge_add(ctx, b, b, &dup) == FBS_POWER_E_INVALID); /* a == b */
  CHECK(dup == 0xA5A5A5A5u);
  CHECK(fbs_power_edge_count(ctx) == 1u);

  /* A freed edge slot behaves the same way. */
  CHECK(fbs_power_edge_remove(ctx, e) == FBS_POWER_OK);
  {
    fbs_power_edge again = add_edge(ctx, reused, b);
    fbs_power_node ea = 0xA5A5A5A5u, eb = 0xA5A5A5A5u;
    CHECK(IDX(again) == IDX(e));
    CHECK(again != e);
    CHECK(fbs_power_edge_get(ctx, e, &ea, &eb) == FBS_POWER_E_NOT_FOUND);
    CHECK(ea == 0xA5A5A5A5u && eb == 0xA5A5A5A5u);
    CHECK(fbs_power_edge_get(ctx, again, &ea, &eb) == FBS_POWER_OK);
    CHECK((ea == reused && eb == b) || (ea == b && eb == reused));
  }

  /* fbs_power_clear keeps counting generations. */
  {
    fbs_power_node before = reused;
    fbs_power_network net_before = net_of(ctx, reused);
    fbs_power_node after = FBS_POWER_INVALID;
    size_t count = 0u;
    fbs_power_clear(ctx);
    CHECK(fbs_power_node_count(ctx) == 0u);
    CHECK(fbs_power_edge_count(ctx) == 0u);
    CHECK(fbs_power_network_count(ctx) == 0u);
    CHECK(fbs_power_node_get(ctx, before, &d) == FBS_POWER_E_NOT_FOUND);
    CHECK(fbs_power_network_members(ctx, net_before, &after, 1u, &count) == FBS_POWER_E_NOT_FOUND);
    after = plain(ctx);
    CHECK(IDX(after) == 0u);
    CHECK(after != before);
    CHECK(fbs_power_node_get(ctx, before, &d) == FBS_POWER_E_NOT_FOUND);
  }

  /* Malformed handles are E_INVALID, not E_NOT_FOUND. */
  CHECK(fbs_power_node_get(ctx, FBS_POWER_INVALID, &d) == FBS_POWER_E_INVALID);
  CHECK(fbs_power_node_get(ctx, 3u, &d) == FBS_POWER_E_INVALID); /* generation 0 */
  CHECK(fbs_power_node_get(ctx, 0xFFFFFFFFu, &d) == FBS_POWER_E_INVALID);
  CHECK(fbs_power_edge_get(ctx, FBS_POWER_INVALID, &b, &reused) == FBS_POWER_E_INVALID);
  CHECK(fbs_power_tick_network(ctx, FBS_POWER_INVALID) == FBS_POWER_E_INVALID);
  fbs_power_destroy(ctx);
}

/* ------------------------------------------------------------------------- */
/* §7.10 Serialization round trip                                            */
/* ------------------------------------------------------------------------- */

/* The graph the golden fixture is built from: a producer/consumer/battery
 * network, a network whose battery runs out, and a lone hybrid node. */
#define RICH_NODES 8u

static fbs_power_context *build_rich(fbs_power_node *out) {
  fbs_power_context *ctx = make_ctx(NULL);
  out[0] = add_node(ctx, 100, 0, 0, 0, 0u, 0xA0u);
  out[1] = add_node(ctx, 0, 30, 0, 0, 2u, 0xA1u);
  out[2] = add_node(ctx, 0, 55, 0, 0, 1u, 0xA2u);
  out[3] = add_node(ctx, 0, 0, 200, 0, 0u, 0xA3u);
  out[4] = add_node(ctx, 10, 0, 0, 0, 0u, 0xA4u);
  out[5] = add_node(ctx, 0, 25, 0, 0, 3u, 0xA5u);
  out[6] = add_node(ctx, 0, 0, 100, 60, 0u, 0xA6u);
  out[7] = add_node(ctx, 5, 2, 50, 10, 1u, 0xA7u);
  (void)add_edge(ctx, out[0], out[1]);
  (void)add_edge(ctx, out[1], out[2]);
  (void)add_edge(ctx, out[2], out[3]);
  (void)add_edge(ctx, out[4], out[5]);
  (void)add_edge(ctx, out[5], out[6]);
  return ctx;
}

static void test_round_trip(void) {
  fbs_power_node h[RICH_NODES];
  fbs_power_context *ctx = build_rich(h);
  fbs_power_context *back = NULL;
  unsigned char *b0, *b1, *b2, *b3;
  size_t l0 = 0u, l1 = 0u, l2 = 0u, l3 = 0u;
  unsigned i;

  for (i = 0u; i < 10u; ++i) CHECK(fbs_power_tick(ctx) == FBS_POWER_OK);
  b0 = serialize_alloc(ctx, &l0);
  CHECK(l0 == 24u + RICH_NODES * 48u + 5u * 12u + 4u);

  CHECK(fbs_power_deserialize(b0, l0, NULL, NULL, &back) == FBS_POWER_OK);
  b1 = serialize_alloc(back, &l1);
  CHECK(blobs_equal(b0, l0, b1, l1)); /* serialize -> deserialize -> serialize */

  /* Handles survive the trip, and so do the descriptors. */
  for (i = 0u; i < RICH_NODES; ++i) {
    fbs_power_node_desc da, db;
    memset(&da, 0, sizeof da);
    memset(&db, 0, sizeof db);
    CHECK(fbs_power_node_get(ctx, h[i], &da) == FBS_POWER_OK);
    CHECK(fbs_power_node_get(back, h[i], &db) == FBS_POWER_OK);
    CHECK(memcmp(&da, &db, sizeof da) == 0);
  }
  CHECK(fbs_power_node_count(back) == RICH_NODES);
  CHECK(fbs_power_edge_count(back) == 5u);
  CHECK(fbs_power_network_count(back) == 3u);
  same_membership(ctx, back, h, RICH_NODES);

  /* The blob carries no derived tick results, so the assertion the decision
     asks for is that the *next* tick agrees: same stats, same allocations,
     same bytes afterwards. */
  CHECK(fbs_power_tick(ctx) == FBS_POWER_OK);
  CHECK(fbs_power_tick(back) == FBS_POWER_OK);
  for (i = 0u; i < RICH_NODES; ++i) {
    fbs_power_network_stats sa = stats_of(ctx, net_of(ctx, h[i]));
    fbs_power_network_stats sb = stats_of(back, net_of(back, h[i]));
    CHECK(memcmp(&sa, &sb, sizeof sa) == 0);
    CHECK(alloc_of(ctx, h[i]) == alloc_of(back, h[i]));
  }
  b2 = serialize_alloc(ctx, &l2);
  b3 = serialize_alloc(back, &l3);
  CHECK(blobs_equal(b2, l2, b3, l3));

  free(b1);
  free(b2);
  free(b3);
  fbs_power_destroy(back);
  fbs_power_destroy(ctx);

  /* An empty context is a legal blob too. */
  {
    fbs_power_context *empty = make_ctx(NULL);
    fbs_power_context *loaded = NULL;
    unsigned char *e0, *e1;
    size_t el0 = 0u, el1 = 0u;
    e0 = serialize_alloc(empty, &el0);
    CHECK(el0 == 28u);
    CHECK(fbs_power_deserialize(e0, el0, NULL, NULL, &loaded) == FBS_POWER_OK);
    CHECK(fbs_power_node_count(loaded) == 0u);
    CHECK(fbs_power_network_count(loaded) == 0u);
    e1 = serialize_alloc(loaded, &el1);
    CHECK(blobs_equal(e0, el0, e1, el1));
    free(e0);
    free(e1);
    fbs_power_destroy(loaded);
    fbs_power_destroy(empty);
  }

  /* The policy travels with the blob. */
  {
    fbs_power_context *pri = make_policy_ctx(FBS_POWER_POLICY_PRIORITY);
    fbs_power_context *loaded = NULL;
    fbs_power_node p = add_node(pri, 7, 0, 0, 0, 0u, 0u);
    fbs_power_node c0 = add_node(pri, 0, 3, 0, 0, 5u, 1u);
    fbs_power_node c1 = add_node(pri, 0, 3, 0, 0, 1u, 2u);
    fbs_power_node c2 = add_node(pri, 0, 3, 0, 0, 5u, 3u);
    unsigned char *pb;
    size_t pl = 0u;
    (void)add_edge(pri, p, c0);
    (void)add_edge(pri, c0, c1);
    (void)add_edge(pri, c1, c2);
    pb = serialize_alloc(pri, &pl);
    CHECK(fbs_power_deserialize(pb, pl, NULL, NULL, &loaded) == FBS_POWER_OK);
    CHECK(fbs_power_tick(loaded) == FBS_POWER_OK);
    CHECK(alloc_of(loaded, c0) == 3);
    CHECK(alloc_of(loaded, c1) == 3);
    CHECK(alloc_of(loaded, c2) == 1);
    free(pb);
    fbs_power_destroy(loaded);
    fbs_power_destroy(pri);
  }

  /* Sparse slots: a blob whose node indices have holes must land back on the
     same slots, so the handles keep resolving. */
  {
    fbs_power_context *sparse = make_ctx(NULL);
    fbs_power_context *loaded = NULL;
    fbs_power_node keep[3];
    fbs_power_node drop[2];
    unsigned char *s0, *s1;
    size_t sl0 = 0u, sl1 = 0u;
    keep[0] = add_node(sparse, 4, 0, 0, 0, 0u, 1u);
    drop[0] = plain(sparse);
    keep[1] = add_node(sparse, 0, 3, 0, 0, 0u, 2u);
    drop[1] = plain(sparse);
    keep[2] = add_node(sparse, 0, 0, 20, 5, 0u, 3u);
    CHECK(fbs_power_node_remove(sparse, drop[0]) == FBS_POWER_OK);
    CHECK(fbs_power_node_remove(sparse, drop[1]) == FBS_POWER_OK);
    (void)add_edge(sparse, keep[0], keep[2]);
    s0 = serialize_alloc(sparse, &sl0);
    CHECK(fbs_power_deserialize(s0, sl0, NULL, NULL, &loaded) == FBS_POWER_OK);
    for (i = 0u; i < 3u; ++i) {
      fbs_power_node_desc dd;
      memset(&dd, 0, sizeof dd);
      CHECK(fbs_power_node_get(loaded, keep[i], &dd) == FBS_POWER_OK);
      CHECK(dd.tag == (uint64_t)(i + 1u));
    }
    {
      fbs_power_node_desc gone;
      memset(&gone, 0xA5, sizeof gone);
      CHECK(fbs_power_node_get(loaded, drop[0], &gone) == FBS_POWER_E_NOT_FOUND);
      CHECK(fbs_power_node_get(loaded, drop[1], &gone) == FBS_POWER_E_NOT_FOUND);
      CHECK(untouched(&gone, sizeof gone, 0xA5));
    }
    CHECK(fbs_power_network_count(loaded) == 2u);
    s1 = serialize_alloc(loaded, &sl1);
    CHECK(blobs_equal(s0, sl0, s1, sl1));
    free(s0);
    free(s1);
    fbs_power_destroy(loaded);
    fbs_power_destroy(sparse);
  }
  free(b0);
}

static void test_corruption(void) {
  fbs_power_node h[RICH_NODES];
  fbs_power_context *ctx = build_rich(h);
  fbs_power_context *out;
  counting_alloc ca;
  fbs_power_allocator alloc;
  unsigned char *blob;
  size_t len = 0u, i;
  int all_schema = 1, all_clean = 1;

  memset(&ca, 0, sizeof ca);
  ca.budget = -1;
  alloc.alloc = ca_alloc;
  alloc.free = ca_free;
  alloc.user = &ca;

  CHECK(fbs_power_tick(ctx) == FBS_POWER_OK);
  blob = serialize_alloc(ctx, &len);

  /* Every truncation is a schema error and allocates nothing. */
  for (i = 0u; i < len; ++i) {
    out = (fbs_power_context *)0xA5;
    if (fbs_power_deserialize(blob, i, NULL, &alloc, &out) != FBS_POWER_E_SCHEMA) all_schema = 0;
    if (out != (fbs_power_context *)0xA5) all_clean = 0;
  }
  CHECK(all_schema);
  CHECK(all_clean);
  CHECK(ca.allocs == 0);

  /* So is every single-byte corruption: the crc32 catches whatever the field
     checks do not. */
  all_schema = 1;
  for (i = 0u; i < len; ++i) {
    unsigned char save = blob[i];
    blob[i] = (unsigned char)(save ^ 0xFFu);
    out = (fbs_power_context *)0xA5;
    if (fbs_power_deserialize(blob, len, NULL, &alloc, &out) != FBS_POWER_E_SCHEMA) all_schema = 0;
    if (out != (fbs_power_context *)0xA5) all_clean = 0;
    blob[i] = save;
  }
  CHECK(all_schema);
  CHECK(all_clean);
  CHECK(ca.allocs == 0);

  /* A flipped crc alone, with every other byte intact. */
  {
    unsigned char save = blob[len - 1u];
    blob[len - 1u] = (unsigned char)(save ^ 0x01u);
    out = (fbs_power_context *)0xA5;
    CHECK(fbs_power_deserialize(blob, len, NULL, &alloc, &out) == FBS_POWER_E_SCHEMA);
    CHECK(out == (fbs_power_context *)0xA5);
    blob[len - 1u] = save;
    out = NULL;
    CHECK(fbs_power_deserialize(blob, len, NULL, &alloc, &out) == FBS_POWER_OK);
    fbs_power_destroy(out);
  }

  /* Structural corruptions the crc is recomputed for, so only the field checks
     can catch them. */
  {
    unsigned char *bad = (unsigned char *)malloc(len);
    const size_t node_off = 24u;
    const size_t edge_off = node_off + RICH_NODES * 48u;
    int base_allocs = ca.allocs; /* a successful load happened just above */
    CHECK(bad != NULL);
    if (bad) {
      /* two node records with the same index: not strictly ascending */
      memcpy(bad, blob, len);
      memcpy(bad + node_off + 48u, bad + node_off, 4u);
      fix_crc(bad, len);
      out = (fbs_power_context *)0xA5;
      CHECK(fbs_power_deserialize(bad, len, NULL, &alloc, &out) == FBS_POWER_E_SCHEMA);
      CHECK(out == (fbs_power_context *)0xA5);
      CHECK(ca.allocs == base_allocs); /* still rejected before a byte is allocated */

      /* an edge naming a node handle that is not in the blob (right index,
         wrong generation) must not resolve to the node at that index */
      memcpy(bad, blob, len);
      {
        uint32_t ep = (uint32_t)bad[edge_off + 4] | ((uint32_t)bad[edge_off + 5] << 8) |
                      ((uint32_t)bad[edge_off + 6] << 16) | ((uint32_t)bad[edge_off + 7] << 24);
        uint32_t wrong = (ep ^ 0x00100000u); /* flip a generation bit, keep the index */
        bad[edge_off + 4] = (unsigned char)(wrong & 0xffu);
        bad[edge_off + 5] = (unsigned char)((wrong >> 8) & 0xffu);
        bad[edge_off + 6] = (unsigned char)((wrong >> 16) & 0xffu);
        bad[edge_off + 7] = (unsigned char)((wrong >> 24) & 0xffu);
      }
      fix_crc(bad, len);
      out = (fbs_power_context *)0xA5;
      CHECK(fbs_power_deserialize(bad, len, NULL, &alloc, &out) == FBS_POWER_E_SCHEMA);
      CHECK(out == (fbs_power_context *)0xA5);
      CHECK(ca.allocs == base_allocs);

      /* the same pair joined twice (the pack's P7). This one is caught after
         the adjacency lists are built, so the context is allocated and then
         destroyed: the blob is still refused and nothing leaks. */
      memcpy(bad, blob, len);
      memcpy(bad + edge_off + 12u + 4u, bad + edge_off + 4u, 8u); /* copy both endpoints */
      fix_crc(bad, len);
      out = (fbs_power_context *)0xA5;
      CHECK(fbs_power_deserialize(bad, len, NULL, &alloc, &out) == FBS_POWER_E_SCHEMA);
      CHECK(out == (fbs_power_context *)0xA5);
      CHECK(ca.allocs == ca.frees); /* allocated and released, never leaked */

      /* and the mirrored pair is caught too, not just the identical one */
      memcpy(bad, blob, len);
      memcpy(bad + edge_off + 12u + 4u, bad + edge_off + 8u, 4u);
      memcpy(bad + edge_off + 12u + 8u, bad + edge_off + 4u, 4u);
      fix_crc(bad, len);
      out = (fbs_power_context *)0xA5;
      CHECK(fbs_power_deserialize(bad, len, NULL, &alloc, &out) == FBS_POWER_E_SCHEMA);
      CHECK(out == (fbs_power_context *)0xA5);
      CHECK(ca.allocs == ca.frees);
      free(bad);
    }
  }

  CHECK(ca.allocs == ca.frees);
  free(blob);
  fbs_power_destroy(ctx);
}

/* PW-2. Blob validation is O(N + E log N) before the allocation and O(V + E)
 * after it, so a four-thousand-node save loads promptly instead of quadratically.
 * The elapsed time is *reported*, not asserted tightly: a sanitizer build is an
 * order of magnitude slower and this is a smoke signal, not a budget. The loose
 * ceiling below only catches a return to quadratic behaviour, which took
 * seconds. */
#define BIG_NODES 4000u

static void test_large_blob_load(void) {
  fbs_power_config cfg = fbs_power_config_default();
  fbs_power_context *ctx, *back = NULL;
  fbs_power_node *nodes;
  unsigned char *blob = NULL;
  size_t len = 0u, need;
  unsigned i;
  int build_ok = 1;
  clock_t t0, t1;
  long ms;

  cfg.max_nodes = BIG_NODES;
  cfg.max_edges = BIG_NODES;
  cfg.max_networks = BIG_NODES;
  ctx = make_ctx(&cfg);
  nodes = (fbs_power_node *)malloc((size_t)BIG_NODES * sizeof(fbs_power_node));
  CHECK(nodes != NULL);
  if (!nodes) {
    fbs_power_destroy(ctx);
    return;
  }

  /* A path of 4000 nodes: 3999 edges, one network, all three node kinds. */
  for (i = 0u; i < BIG_NODES; ++i) {
    fbs_power_node_desc d;
    d.production = (i % 3u) == 0u ? 7 : 0;
    d.demand = (i % 3u) == 1u ? 5 : 0;
    d.capacity = (i % 3u) == 2u ? 50 : 0;
    d.stored = (i % 3u) == 2u ? 20 : 0;
    d.priority = i % 4u;
    d.tag = (uint64_t)i;
    if (fbs_power_node_add(ctx, &d, &nodes[i]) != FBS_POWER_OK) build_ok = 0;
  }
  for (i = 1u; i < BIG_NODES; ++i) {
    fbs_power_edge e = FBS_POWER_INVALID;
    if (fbs_power_edge_add(ctx, nodes[i - 1u], nodes[i], &e) != FBS_POWER_OK) build_ok = 0;
  }
  CHECK(build_ok);
  CHECK(fbs_power_node_count(ctx) == BIG_NODES);
  CHECK(fbs_power_edge_count(ctx) == BIG_NODES - 1u);
  CHECK(fbs_power_network_count(ctx) == 1u);

  need = fbs_power_serialized_size(ctx);
  CHECK(need == 24u + (size_t)BIG_NODES * 48u + (size_t)(BIG_NODES - 1u) * 12u + 4u);
  blob = (unsigned char *)malloc(need);
  CHECK(blob != NULL);
  if (blob) {
    CHECK(fbs_power_serialize(ctx, blob, need, &len) == FBS_POWER_OK);

    t0 = clock();
    CHECK(fbs_power_deserialize(blob, len, NULL, NULL, &back) == FBS_POWER_OK);
    t1 = clock();
    ms = (long)((t1 - t0) * 1000 / CLOCKS_PER_SEC);
    printf("power: %u-node / %u-edge blob (%lu bytes) loaded in %ld ms\n", BIG_NODES,
           BIG_NODES - 1u, (unsigned long)len, ms);
    CHECK(ms < 2000L); /* quadratic validation took seconds; this must not */

    if (back) {
      unsigned char *again;
      size_t len2 = 0u;
      CHECK(fbs_power_node_count(back) == BIG_NODES);
      CHECK(fbs_power_edge_count(back) == BIG_NODES - 1u);
      CHECK(fbs_power_network_count(back) == 1u);
      CHECK(stats_of(back, net_of(back, nodes[0])).node_count == BIG_NODES);
      again = serialize_alloc(back, &len2);
      CHECK(blobs_equal(again, len2, blob, len));
      free(again);
      fbs_power_destroy(back);
    }
    free(blob);
  }
  free(nodes);
  fbs_power_destroy(ctx);
}

/* ------------------------------------------------------------------------- */
/* §7.11 Overflow                                                            */
/* ------------------------------------------------------------------------- */

static void test_overflow(void) {
  fbs_power_context *ctx = make_ctx(NULL);
  fbs_power_node a = add_node(ctx, INT64_MAX / 2 + 1, 0, 0, 0, 0u, 0u);
  fbs_power_node b = add_node(ctx, INT64_MAX / 2 + 1, 0, 0, 0, 0u, 1u);
  fbs_power_node c = add_node(ctx, 3, 0, 0, 0, 0u, 2u);
  fbs_power_network_stats s;
  unsigned char *b0, *b1;
  size_t l0 = 0u, l1 = 0u;

  /* Separate networks are fine. */
  CHECK(fbs_power_tick(ctx) == FBS_POWER_OK);
  (void)add_edge(ctx, a, b);
  b0 = serialize_alloc(ctx, &l0);

  memset(&s, 0xA5, sizeof s);
  CHECK(fbs_power_network_stats_get(ctx, net_of(ctx, a), &s) == FBS_POWER_E_OVERFLOW);
  CHECK(untouched(&s, sizeof s, 0xA5));
  CHECK(fbs_power_tick_network(ctx, net_of(ctx, a)) == FBS_POWER_E_OVERFLOW);
  CHECK(fbs_power_tick(ctx) == FBS_POWER_E_OVERFLOW);
  b1 = serialize_alloc(ctx, &l1);
  CHECK(blobs_equal(b0, l0, b1, l1)); /* a refused tick changes nothing */
  /* the healthy network was not ticked either: the whole call is refused */
  CHECK(alloc_of(ctx, c) == 0);
  free(b0);
  free(b1);

  /* Demand overflows the same way. */
  CHECK(fbs_power_node_set_rates(ctx, a, 0, INT64_MAX - 1) == FBS_POWER_OK);
  CHECK(fbs_power_node_set_rates(ctx, b, 0, INT64_MAX - 1) == FBS_POWER_OK);
  CHECK(fbs_power_tick(ctx) == FBS_POWER_E_OVERFLOW);

  /* So does the sum of free capacity. */
  CHECK(fbs_power_node_set_rates(ctx, a, 1, 0) == FBS_POWER_OK);
  CHECK(fbs_power_node_set_rates(ctx, b, 1, 0) == FBS_POWER_OK);
  CHECK(fbs_power_node_set_capacity(ctx, a, INT64_MAX - 1) == FBS_POWER_OK);
  CHECK(fbs_power_node_set_capacity(ctx, b, INT64_MAX - 1) == FBS_POWER_OK);
  CHECK(fbs_power_tick(ctx) == FBS_POWER_E_OVERFLOW);

  /* Back under the limit and it ticks again. */
  CHECK(fbs_power_node_set_capacity(ctx, a, 10) == FBS_POWER_OK);
  CHECK(fbs_power_node_set_capacity(ctx, b, 10) == FBS_POWER_OK);
  CHECK(fbs_power_tick(ctx) == FBS_POWER_OK);
  fbs_power_destroy(ctx);

  /* Very large but legal values still balance exactly, and the satisfaction
     fraction stays exact when it fits in 32 bits. */
  {
    fbs_power_context *big = make_ctx(NULL);
    fbs_power_node p = add_node(big, 3000000000LL, 0, 0, 0, 0u, 0u);
    fbs_power_node q = add_node(big, 0, 5000000000LL, 0, 0, 0u, 1u);
    fbs_power_network_stats t;
    (void)add_edge(big, p, q);
    CHECK(fbs_power_tick(big) == FBS_POWER_OK);
    CHECK(alloc_of(big, q) == 3000000000LL);
    t = stats_of(big, net_of(big, p));
    CHECK(t.satisfaction_num == 3u && t.satisfaction_den == 5u);
    CHECK(t.production + t.discharged == t.consumed + t.charged + t.wasted);
    fbs_power_destroy(big);
  }

  /* A denominator that cannot be reduced into 32 bits falls back to the
     largest representable denominator, rounded down, never overstated. */
  {
    fbs_power_context *huge = make_ctx(NULL);
    fbs_power_node p = add_node(huge, 1, 0, 0, 0, 0u, 0u);
    fbs_power_node q = add_node(huge, 0, 4294967311LL, 0, 0, 0u, 1u); /* prime > 2^32 */
    fbs_power_network_stats t;
    (void)add_edge(huge, p, q);
    CHECK(fbs_power_tick(huge) == FBS_POWER_OK);
    t = stats_of(huge, net_of(huge, p));
    CHECK(t.consumed == 1);
    CHECK(t.satisfaction_num == 0u && t.satisfaction_den == 1u);
    CHECK((int64_t)t.satisfaction_num * t.demand <= t.consumed * (int64_t)t.satisfaction_den);
    fbs_power_destroy(huge);
  }
}

/* ------------------------------------------------------------------------- */
/* Status names, version, config validation                                  */
/* ------------------------------------------------------------------------- */

static void test_status_names(void) {
  CHECK(strcmp(fbs_power_status_name(FBS_POWER_OK), "ok") == 0);
  CHECK(strcmp(fbs_power_status_name(FBS_POWER_E_INVALID), "invalid") == 0);
  CHECK(strcmp(fbs_power_status_name(FBS_POWER_E_RANGE), "range") == 0);
  CHECK(strcmp(fbs_power_status_name(FBS_POWER_E_FULL), "full") == 0);
  CHECK(strcmp(fbs_power_status_name(FBS_POWER_E_MEMORY), "memory") == 0);
  CHECK(strcmp(fbs_power_status_name(FBS_POWER_E_STATE), "state") == 0);
  CHECK(strcmp(fbs_power_status_name(FBS_POWER_E_NOT_FOUND), "not_found") == 0);
  CHECK(strcmp(fbs_power_status_name(FBS_POWER_E_EXISTS), "exists") == 0);
  CHECK(strcmp(fbs_power_status_name(FBS_POWER_E_OVERFLOW), "overflow") == 0);
  CHECK(strcmp(fbs_power_status_name(FBS_POWER_E_SCHEMA), "schema") == 0);
  CHECK(strcmp(fbs_power_status_name(FBS_POWER_E_TRUNCATED), "truncated") == 0);
  CHECK(strcmp(fbs_power_status_name(-99), "unknown") == 0);
  CHECK(strcmp(fbs_power_status_name(42), "unknown") == 0);
  CHECK(fbs_power_status_name(-1) != NULL);

  CHECK(fbs_power_version() == FBS_POWER_VERSION);
  CHECK(fbs_power_version() ==
        (unsigned)(FBS_POWER_VERSION_MAJOR * 10000 + FBS_POWER_VERSION_MINOR * 100 +
                   FBS_POWER_VERSION_PATCH));
}

static void test_config(void) {
  fbs_power_config cfg = fbs_power_config_default();
  fbs_power_context *ctx = (fbs_power_context *)0xA5;

  CHECK(cfg.max_nodes == 256u);
  CHECK(cfg.max_edges == 512u);
  CHECK(cfg.max_networks == 256u);
  CHECK(cfg.policy == FBS_POWER_POLICY_PROPORTIONAL);

  cfg.max_nodes = 0u;
  CHECK(fbs_power_create(&cfg, NULL, &ctx) == FBS_POWER_E_RANGE);
  CHECK(ctx == (fbs_power_context *)0xA5);
  cfg.max_nodes = (1u << 20);
  CHECK(fbs_power_create(&cfg, NULL, &ctx) == FBS_POWER_E_RANGE);
  cfg = fbs_power_config_default();
  cfg.max_edges = (1u << 20);
  CHECK(fbs_power_create(&cfg, NULL, &ctx) == FBS_POWER_E_RANGE);
  cfg = fbs_power_config_default();
  cfg.max_networks = 0u;
  CHECK(fbs_power_create(&cfg, NULL, &ctx) == FBS_POWER_E_RANGE);
  cfg = fbs_power_config_default();
  cfg.policy = (fbs_power_policy)7;
  CHECK(fbs_power_create(&cfg, NULL, &ctx) == FBS_POWER_E_INVALID);
  cfg.policy = (fbs_power_policy)-1;
  CHECK(fbs_power_create(&cfg, NULL, &ctx) == FBS_POWER_E_INVALID);
  CHECK(ctx == (fbs_power_context *)0xA5);

  /* max_edges 0 is legal: a graph of lone nodes. */
  cfg = fbs_power_config_default();
  cfg.max_edges = 0u;
  ctx = NULL;
  CHECK(fbs_power_create(&cfg, NULL, &ctx) == FBS_POWER_OK);
  {
    fbs_power_node a = plain(ctx), b = plain(ctx);
    fbs_power_edge e = FBS_POWER_INVALID;
    CHECK(fbs_power_edge_add(ctx, a, b, &e) == FBS_POWER_E_FULL);
    CHECK(fbs_power_memory(ctx) > 0u);
  }
  fbs_power_destroy(ctx);

  /* cfg == NULL means the default config. */
  ctx = NULL;
  CHECK(fbs_power_create(NULL, NULL, &ctx) == FBS_POWER_OK);
  CHECK(fbs_power_memory(ctx) > 0u);
  fbs_power_destroy(ctx);
  fbs_power_destroy(NULL); /* must be a no-op */
  fbs_power_clear(NULL);

  /* Deserialize validates the config it is handed. */
  {
    fbs_power_context *src = make_ctx(NULL);
    fbs_power_context *out = (fbs_power_context *)0xA5;
    unsigned char *blob;
    size_t len = 0u;
    (void)plain(src);
    blob = serialize_alloc(src, &len);
    cfg = fbs_power_config_default();
    cfg.max_nodes = 0u;
    CHECK(fbs_power_deserialize(blob, len, &cfg, NULL, &out) == FBS_POWER_E_RANGE);
    cfg = fbs_power_config_default();
    cfg.policy = (fbs_power_policy)9;
    CHECK(fbs_power_deserialize(blob, len, &cfg, NULL, &out) == FBS_POWER_E_INVALID);
    CHECK(out == (fbs_power_context *)0xA5);
    free(blob);
    fbs_power_destroy(src);
  }
}

/* ------------------------------------------------------------------------- */
/* NULL / bad-argument coverage on every entry point                         */
/* ------------------------------------------------------------------------- */

static void test_invalid_arguments(void) {
  fbs_power_context *ctx = make_ctx(NULL);
  fbs_power_node a = plain(ctx), b = plain(ctx), got = 0xA5A5A5A5u;
  fbs_power_edge e = add_edge(ctx, a, b), egot = 0xA5A5A5A5u;
  fbs_power_network net = net_of(ctx, a), ngot = 0xA5A5A5A5u;
  fbs_power_node_desc desc;
  fbs_power_network_stats stats;
  size_t count = 0xA5A5u;
  int64_t v = -3;
  unsigned char buf[64];

  memset(&desc, 0xA5, sizeof desc);
  memset(&stats, 0xA5, sizeof stats);

  CHECK(fbs_power_node_add(NULL, &desc, &got) == FBS_POWER_E_INVALID);
  CHECK(fbs_power_node_add(ctx, NULL, &got) == FBS_POWER_E_INVALID);
  CHECK(fbs_power_node_add(ctx, &desc, NULL) == FBS_POWER_E_INVALID);
  CHECK(got == 0xA5A5A5A5u);
  {
    fbs_power_node_desc bad;
    memset(&bad, 0, sizeof bad);
    bad.production = -1;
    CHECK(fbs_power_node_add(ctx, &bad, &got) == FBS_POWER_E_RANGE);
    bad.production = 0;
    bad.demand = -1;
    CHECK(fbs_power_node_add(ctx, &bad, &got) == FBS_POWER_E_RANGE);
    bad.demand = 0;
    bad.capacity = -1;
    CHECK(fbs_power_node_add(ctx, &bad, &got) == FBS_POWER_E_RANGE);
    bad.capacity = 5;
    bad.stored = 6;
    CHECK(fbs_power_node_add(ctx, &bad, &got) == FBS_POWER_E_RANGE);
    bad.stored = -1;
    CHECK(fbs_power_node_add(ctx, &bad, &got) == FBS_POWER_E_RANGE);
    CHECK(got == 0xA5A5A5A5u);
  }

  CHECK(fbs_power_node_remove(NULL, a) == FBS_POWER_E_INVALID);
  CHECK(fbs_power_node_get(NULL, a, &desc) == FBS_POWER_E_INVALID);
  CHECK(fbs_power_node_get(ctx, a, NULL) == FBS_POWER_E_INVALID);
  CHECK(untouched(&desc, sizeof desc, 0xA5));
  CHECK(fbs_power_node_set_rates(NULL, a, 0, 0) == FBS_POWER_E_INVALID);
  CHECK(fbs_power_node_set_stored(NULL, a, 0) == FBS_POWER_E_INVALID);
  CHECK(fbs_power_node_set_capacity(NULL, a, 0) == FBS_POWER_E_INVALID);
  CHECK(fbs_power_node_set_priority(NULL, a, 0u) == FBS_POWER_E_INVALID);
  CHECK(fbs_power_node_network(NULL, a, &ngot) == FBS_POWER_E_INVALID);
  CHECK(fbs_power_node_network(ctx, a, NULL) == FBS_POWER_E_INVALID);
  CHECK(ngot == 0xA5A5A5A5u);
  CHECK(fbs_power_node_allocation(NULL, a, &v) == FBS_POWER_E_INVALID);
  CHECK(fbs_power_node_allocation(ctx, a, NULL) == FBS_POWER_E_INVALID);
  CHECK(v == -3);

  CHECK(fbs_power_edge_add(NULL, a, b, &egot) == FBS_POWER_E_INVALID);
  CHECK(fbs_power_edge_add(ctx, a, b, NULL) == FBS_POWER_E_INVALID);
  CHECK(fbs_power_edge_add(ctx, FBS_POWER_INVALID, b, &egot) == FBS_POWER_E_INVALID);
  CHECK(fbs_power_edge_add(ctx, a, FBS_POWER_INVALID, &egot) == FBS_POWER_E_INVALID);
  CHECK(egot == 0xA5A5A5A5u);
  CHECK(fbs_power_edge_remove(NULL, e) == FBS_POWER_E_INVALID);
  CHECK(fbs_power_edge_get(NULL, e, &got, &egot) == FBS_POWER_E_INVALID);
  CHECK(fbs_power_edge_get(ctx, e, NULL, &egot) == FBS_POWER_E_INVALID);
  CHECK(fbs_power_edge_get(ctx, e, &got, NULL) == FBS_POWER_E_INVALID);
  CHECK(got == 0xA5A5A5A5u && egot == 0xA5A5A5A5u);

  CHECK(fbs_power_edge_list(NULL, &egot, 1u, &count) == FBS_POWER_E_INVALID);
  CHECK(fbs_power_edge_list(ctx, &egot, 1u, NULL) == FBS_POWER_E_INVALID);
  CHECK(fbs_power_edge_list(ctx, NULL, 1u, &count) == FBS_POWER_E_INVALID);
  CHECK(fbs_power_node_list(NULL, &got, 1u, &count) == FBS_POWER_E_INVALID);
  CHECK(fbs_power_node_list(ctx, &got, 1u, NULL) == FBS_POWER_E_INVALID);
  CHECK(fbs_power_node_list(ctx, NULL, 1u, &count) == FBS_POWER_E_INVALID);
  CHECK(fbs_power_network_list(NULL, &ngot, 1u, &count) == FBS_POWER_E_INVALID);
  CHECK(fbs_power_network_list(ctx, &ngot, 1u, NULL) == FBS_POWER_E_INVALID);
  CHECK(fbs_power_network_list(ctx, NULL, 1u, &count) == FBS_POWER_E_INVALID);
  CHECK(fbs_power_network_members(NULL, net, &got, 1u, &count) == FBS_POWER_E_INVALID);
  CHECK(fbs_power_network_members(ctx, net, &got, 1u, NULL) == FBS_POWER_E_INVALID);
  CHECK(fbs_power_network_members(ctx, net, NULL, 1u, &count) == FBS_POWER_E_INVALID);
  CHECK(count == 0xA5A5u);
  CHECK(got == 0xA5A5A5A5u && egot == 0xA5A5A5A5u && ngot == 0xA5A5A5A5u);

  CHECK(fbs_power_network_stats_get(NULL, net, &stats) == FBS_POWER_E_INVALID);
  CHECK(fbs_power_network_stats_get(ctx, net, NULL) == FBS_POWER_E_INVALID);
  CHECK(fbs_power_network_stats_get(ctx, FBS_POWER_INVALID, &stats) == FBS_POWER_E_INVALID);
  CHECK(untouched(&stats, sizeof stats, 0xA5));

  CHECK(fbs_power_tick(NULL) == FBS_POWER_E_INVALID);
  CHECK(fbs_power_tick_network(NULL, net) == FBS_POWER_E_INVALID);

  CHECK(fbs_power_serialized_size(NULL) == 0u);
  CHECK(fbs_power_serialize(NULL, buf, sizeof buf, &count) == FBS_POWER_E_INVALID);
  CHECK(fbs_power_serialize(ctx, buf, sizeof buf, NULL) == FBS_POWER_E_INVALID);
  CHECK(fbs_power_serialize(ctx, NULL, sizeof buf, &count) == FBS_POWER_E_INVALID);
  {
    fbs_power_context *out = (fbs_power_context *)0xA5;
    CHECK(fbs_power_deserialize(NULL, 10u, NULL, NULL, &out) == FBS_POWER_E_INVALID);
    CHECK(fbs_power_deserialize(buf, 10u, NULL, NULL, NULL) == FBS_POWER_E_INVALID);
    CHECK(out == (fbs_power_context *)0xA5);
  }

  /* Handles from another context are rejected, not silently honoured. */
  {
    fbs_power_context *other = make_ctx(NULL);
    fbs_power_node far_node = FBS_POWER_INVALID;
    unsigned i;
    for (i = 0u; i < 5u; ++i) far_node = plain(other);
    CHECK(fbs_power_node_get(ctx, far_node, &desc) == FBS_POWER_E_NOT_FOUND);
    fbs_power_destroy(other);
  }
  fbs_power_destroy(ctx);
}

/* ------------------------------------------------------------------------- */
/* Truncated list outputs                                                    */
/* ------------------------------------------------------------------------- */

static void test_truncation(void) {
  fbs_power_context *ctx = make_ctx(NULL);
  fbs_power_node n[4];
  fbs_power_node nbuf[4];
  fbs_power_edge ebuf[4];
  fbs_power_network sbuf[4];
  size_t count = 0u;
  unsigned i;
  unsigned char blob[8];

  for (i = 0u; i < 4u; ++i) n[i] = plain(ctx);
  (void)add_edge(ctx, n[0], n[1]);
  (void)add_edge(ctx, n[2], n[3]);

  memset(nbuf, 0xA5, sizeof nbuf);
  CHECK(fbs_power_node_list(ctx, nbuf, 3u, &count) == FBS_POWER_E_TRUNCATED);
  CHECK(count == 4u);
  CHECK(untouched(nbuf, sizeof nbuf, 0xA5));
  CHECK(fbs_power_node_list(ctx, nbuf, 4u, &count) == FBS_POWER_OK);
  CHECK(count == 4u);
  for (i = 0u; i < 4u; ++i) CHECK(IDX(nbuf[i]) == i); /* ascending node index */

  memset(ebuf, 0xA5, sizeof ebuf);
  CHECK(fbs_power_edge_list(ctx, ebuf, 1u, &count) == FBS_POWER_E_TRUNCATED);
  CHECK(count == 2u);
  CHECK(untouched(ebuf, sizeof ebuf, 0xA5));
  CHECK(fbs_power_edge_list(ctx, ebuf, 4u, &count) == FBS_POWER_OK);
  CHECK(count == 2u);
  CHECK(IDX(ebuf[0]) < IDX(ebuf[1]));

  memset(sbuf, 0xA5, sizeof sbuf);
  CHECK(fbs_power_network_list(ctx, sbuf, 1u, &count) == FBS_POWER_E_TRUNCATED);
  CHECK(count == 2u);
  CHECK(untouched(sbuf, sizeof sbuf, 0xA5));

  memset(nbuf, 0xA5, sizeof nbuf);
  CHECK(fbs_power_network_members(ctx, net_of(ctx, n[0]), nbuf, 1u, &count) ==
        FBS_POWER_E_TRUNCATED);
  CHECK(count == 2u);
  CHECK(untouched(nbuf, sizeof nbuf, 0xA5));

  /* cap 0 with a NULL array is the legal "how many?" query. */
  count = 0u;
  CHECK(fbs_power_node_list(ctx, NULL, 0u, &count) == FBS_POWER_E_TRUNCATED);
  CHECK(count == 4u);
  count = 0u;
  CHECK(fbs_power_edge_list(ctx, NULL, 0u, &count) == FBS_POWER_E_TRUNCATED);
  CHECK(count == 2u);
  count = 0u;
  CHECK(fbs_power_network_list(ctx, NULL, 0u, &count) == FBS_POWER_E_TRUNCATED);
  CHECK(count == 2u);
  count = 0u;
  CHECK(fbs_power_network_members(ctx, net_of(ctx, n[0]), NULL, 0u, &count) ==
        FBS_POWER_E_TRUNCATED);
  CHECK(count == 2u);

  /* serialize reports the required length and writes nothing. */
  memset(blob, 0xA5, sizeof blob);
  count = 0u;
  CHECK(fbs_power_serialize(ctx, blob, sizeof blob, &count) == FBS_POWER_E_TRUNCATED);
  CHECK(count == fbs_power_serialized_size(ctx));
  CHECK(untouched(blob, sizeof blob, 0xA5));
  fbs_power_destroy(ctx);
}

/* ------------------------------------------------------------------------- */
/* Golden fixtures                                                           */
/* ------------------------------------------------------------------------- */

static void test_fixtures(void) {
  fbs_power_node h[RICH_NODES];
  fbs_power_context *ctx = build_rich(h);
  unsigned char *blob;
  size_t len = 0u;
  char path[512];
  char text[512];
  size_t text_len = 0u;
  unsigned i;

  for (i = 0u; i < 10u; ++i) CHECK(fbs_power_tick(ctx) == FBS_POWER_OK);

  /* Hand-computed after ten ticks:
     net {0,1,2,3}: supply 100 >= demand 85, so 15 charges the only battery
                    every tick -> stored 150, allocations 30 and 55.
     net {4,5,6}:   supply 10 < demand 25; the battery covers 15 a tick until
                    tick 4 empties it, after which the consumer gets 10.
     net {7}:       supply 5 >= demand 2, so 3 charges its own store -> 40. */
  CHECK(alloc_of(ctx, h[0]) == 0);
  CHECK(alloc_of(ctx, h[1]) == 30);
  CHECK(alloc_of(ctx, h[2]) == 55);
  CHECK(stored_of(ctx, h[3]) == 150);
  CHECK(alloc_of(ctx, h[5]) == 10);
  CHECK(stored_of(ctx, h[6]) == 0);
  CHECK(alloc_of(ctx, h[7]) == 2);
  CHECK(stored_of(ctx, h[7]) == 40);

  blob = serialize_alloc(ctx, &len);
  sprintf(path, "%s/grid.bin", g_fixture_dir);
  write_or_compare(path, blob, len, "power grid blob");

  for (i = 0u; i < RICH_NODES; ++i) {
    text_len += (size_t)sprintf(text + text_len, "%u %lld %lld\n", (unsigned)IDX(h[i]),
                                (long long)alloc_of(ctx, h[i]), (long long)stored_of(ctx, h[i]));
  }
  sprintf(path, "%s/allocations.txt", g_fixture_dir);
  write_or_compare(path, (const unsigned char *)text, text_len, "power allocations");

  /* The committed bytes must still load, still mean the same thing and still
     re-serialize identically. */
  if (!g_write_fixtures) {
    fbs_power_context *g = NULL;
    sprintf(path, "%s/grid.bin", g_fixture_dir);
    {
      FILE *fp = fopen(path, "rb");
      if (fp) {
        unsigned char golden[2048];
        size_t got = fread(golden, 1u, sizeof golden, fp);
        fclose(fp);
        CHECK(got == len);
        if (got == len) {
          size_t l2 = 0u;
          unsigned char *b2;
          CHECK(fbs_power_deserialize(golden, got, NULL, NULL, &g) == FBS_POWER_OK);
          if (g) {
            b2 = serialize_alloc(g, &l2);
            CHECK(blobs_equal(b2, l2, golden, got));
            CHECK(fbs_power_network_count(g) == 3u);
            for (i = 0u; i < RICH_NODES; ++i) CHECK(stored_of(g, h[i]) == stored_of(ctx, h[i]));
            free(b2);
            fbs_power_destroy(g);
          }
        }
      }
    }
  }

  free(blob);
  fbs_power_destroy(ctx);
}

/* ------------------------------------------------------------------------- */

int main(int argc, char **argv) {
  int i;
  for (i = 1; i < argc; ++i) {
    if (strcmp(argv[i], "--write-fixtures") == 0) {
      g_write_fixtures = 1;
    } else if (strcmp(argv[i], "--fixture-dir") == 0 && i + 1 < argc) {
      g_fixture_dir = argv[++i];
    } else {
      printf("usage: %s [--write-fixtures] [--fixture-dir DIR]\n", argv[0]);
      return 2;
    }
  }

  test_merge();
  test_split();
  test_node_removal();
  test_conservation();
  test_largest_remainder();
  test_order_independence();
  test_policies();
  test_blackout_drains_nothing();
  test_tick_network();
  test_storage();
  test_capacity_failures();
  test_allocator();
  test_handles();
  test_round_trip();
  test_corruption();
  test_large_blob_load();
  test_overflow();
  test_status_names();
  test_config();
  test_invalid_arguments();
  test_truncation();
  test_fixtures();

  printf("power: %d checks passed, %d failed\n", g_checks - g_fails, g_fails);
  return g_fails > 100 ? 100 : g_fails;
}
