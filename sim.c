/* sim.c — implementation of the horde fluid core. See sim.h for the model. */
#include "sim.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* ------------------------------------------------------------------ */
/* allocation                                                          */
/* ------------------------------------------------------------------ */
Sim *sim_create(int W, int H) {
    Sim *s = (Sim *)calloc(1, sizeof(Sim));
    s->W = W; s->H = H;
    size_t n = (size_t)W * H;
    s->rho  = (float *)calloc(n, sizeof(float));
    s->bed  = (float *)calloc(n, sizeof(float));
    s->phi  = (float *)malloc(n * sizeof(float));
    s->vx   = (float *)calloc(n, sizeof(float));
    s->vy   = (float *)calloc(n, sizeof(float));
    s->d    = (float *)calloc(n, sizeof(float));
    s->flag = (unsigned char *)calloc(n, sizeof(unsigned char));
    for (size_t i = 0; i < n; i++) s->phi[i] = SIM_INF;

    /* sealed border so nothing leaks off-grid */
    for (int x = 0; x < W; x++) { s->bed[sim_idx(s,x,0)] = s->bed[sim_idx(s,x,H-1)] = 1e6f; }
    for (int y = 0; y < H; y++) { s->bed[sim_idx(s,0,y)] = s->bed[sim_idx(s,W-1,y)] = 1e6f; }

    /* defaults */
    s->flow        = 0.18f;
    s->relax_rate  = 0.18f;
    s->relax_iters = 6;
    s->spawn_amt   = 0.6f;
    s->spawn_cap   = 14.0f;
    s->drain_rate  = 0.35f;
    s->spawn_enabled = 1;
    s->v_damp      = 0.95f;
    s->v_max       = 0.90f;
    s->phi_dirty   = 1;

    /* particle horde -------------------------------------------------- */
    s->pcap  = 200000;
    s->px    = (float *)malloc(s->pcap * sizeof(float));
    s->py    = (float *)malloc(s->pcap * sizeof(float));
    s->pvx   = (float *)calloc(s->pcap, sizeof(float));
    s->pvy   = (float *)calloc(s->pcap, sizeof(float));
    s->px2   = (float *)malloc(s->pcap * sizeof(float));
    s->py2   = (float *)malloc(s->pcap * sizeof(float));
    s->psx   = (float *)malloc(s->pcap * sizeof(float));
    s->psy   = (float *)malloc(s->pcap * sizeof(float));
    s->pseed = (unsigned char *)malloc(s->pcap);
    s->cell_start = (int *)malloc((n + 1) * sizeof(int));
    s->cell_count = (int *)malloc(n * sizeof(int));
    s->pidx  = (int *)malloc(s->pcap * sizeof(int));
    s->pcount = 0;
    s->p_radius        = 0.40f;
    s->p_repel         = 0.80f;
    s->p_collide_iters = 4;
    s->p_goal_speed    = 0.25f;
    s->p_friction      = 0.60f;
    s->p_jitter        = 0.02f;
    s->spawn_per_step  = 0.06f;
    s->spawn_accum     = 0.0f;
    s->spawn_max       = 10000;
    s->p_press_k       = 0.05f;
    s->p_rho_target    = 3.0f;
    s->p_jam_damp      = 0.85f;
    return s;
}

void sim_free(Sim *s) {
    if (!s) return;
    free(s->rho); free(s->bed); free(s->phi);
    free(s->vx);  free(s->vy);
    free(s->d);   free(s->flag);
    free(s->px);  free(s->py);  free(s->pvx); free(s->pvy);
    free(s->px2); free(s->py2); free(s->psx); free(s->psy); free(s->pseed);
    free(s->cell_start); free(s->cell_count); free(s->pidx);
    free(s);
}

static inline int is_wall(const Sim *s, int i) { return s->bed[i] > 0.0f; }

/* ------------------------------------------------------------------ */
/* editors                                                             */
/* ------------------------------------------------------------------ */
void sim_set_wall(Sim *s, int x, int y, float height) {
    if (x <= 0 || y <= 0 || x >= s->W-1 || y >= s->H-1) return; /* keep border */
    s->bed[sim_idx(s,x,y)] = height > 0.0f ? height : 0.0f;
    s->phi_dirty = 1;
}
void sim_set_flag(Sim *s, int x, int y, unsigned char flag, int on) {
    int i = sim_idx(s,x,y);
    if (on) s->flag[i] |= flag; else s->flag[i] &= (unsigned char)~flag;
    if (flag & CELL_GOAL) s->phi_dirty = 1;
}
void sim_add_rho(Sim *s, int x, int y, float a) { s->rho[sim_idx(s,x,y)] += a; }
void sim_clear(Sim *s) {
    size_t n = (size_t)s->W * s->H;
    memset(s->rho, 0, n * sizeof(float));
    memset(s->vx,  0, n * sizeof(float));
    memset(s->vy,  0, n * sizeof(float));
    s->pcount = 0;                 /* clear the horde, keep terrain */
    s->spawn_accum = 0.0f;
}

void sim_add_velocity(Sim *s, int x, int y, float vx, float vy) {
    if (x <= 0 || y <= 0 || x >= s->W-1 || y >= s->H-1) return;
    int i = sim_idx(s,x,y);
    if (is_wall(s,i)) return;
    s->vx[i] += vx; s->vy[i] += vy;
}

void sim_add_impulse(Sim *s, int cx, int cy, int radius, float strength) {
    if (radius <= 0) return;
    int W = s->W, H = s->H;
    float r2 = (float)(radius * radius);
    for (int dy = -radius; dy <= radius; dy++)
    for (int dx = -radius; dx <= radius; dx++) {
        float d2 = (float)(dx*dx + dy*dy);
        if (d2 > r2) continue;
        int x = cx + dx, y = cy + dy;
        if (x <= 0 || y <= 0 || x >= W-1 || y >= H-1) continue;
        int i = sim_idx(s, x, y);
        if (is_wall(s, i)) continue;
        float f = 1.0f - sqrtf(d2) / (float)radius;          /* 1 at center, 0 at rim */
        if (d2 < 1e-6f) continue;                             /* no direction at exact center */
        float len = sqrtf(d2);
        s->vx[i] += strength * f * ((float)dx / len);
        s->vy[i] += strength * f * ((float)dy / len);
    }
}

/* ------------------------------------------------------------------ */
/* potential field: multi-source Dijkstra from all GOAL cells.         */
/* 8-neighbourhood (ortho cost 1, diagonal sqrt2). Walls are blocked,  */
/* so phi guides the crowd AROUND obstacles and THROUGH gaps. Piling   */
/* and overflow are handled separately by surface equalization.        */
/* ------------------------------------------------------------------ */
typedef struct { float dist; int idx; } HeapNode;
typedef struct { HeapNode *a; int n, cap; } Heap;

static void heap_push(Heap *h, float dist, int idx) {
    if (h->n == h->cap) { h->cap = h->cap ? h->cap*2 : 256;
        h->a = (HeapNode*)realloc(h->a, h->cap*sizeof(HeapNode)); }
    int i = h->n++; h->a[i].dist = dist; h->a[i].idx = idx;
    while (i > 0) { int p = (i-1)/2; if (h->a[p].dist <= h->a[i].dist) break;
        HeapNode t = h->a[p]; h->a[p] = h->a[i]; h->a[i] = t; i = p; }
}
static HeapNode heap_pop(Heap *h) {
    HeapNode top = h->a[0]; h->a[0] = h->a[--h->n];
    int i = 0;
    for (;;) { int l = 2*i+1, r = 2*i+2, m = i;
        if (l < h->n && h->a[l].dist < h->a[m].dist) m = l;
        if (r < h->n && h->a[r].dist < h->a[m].dist) m = r;
        if (m == i) break;
        HeapNode t = h->a[m]; h->a[m] = h->a[i]; h->a[i] = t; i = m; }
    return top;
}

void sim_recompute_phi(Sim *s) {
    int W = s->W, H = s->H; size_t n = (size_t)W*H;
    for (size_t i = 0; i < n; i++) s->phi[i] = SIM_INF;

    Heap h = {0};
    for (size_t i = 0; i < n; i++)
        if ((s->flag[i] & CELL_GOAL) && !is_wall(s,(int)i)) {
            s->phi[i] = 0.0f; heap_push(&h, 0.0f, (int)i);
        }

    static const int dx[8] = {1,-1,0,0, 1,1,-1,-1};
    static const int dy[8] = {0,0,1,-1, 1,-1,1,-1};
    static const float dc[8] = {1,1,1,1, 1.41421356f,1.41421356f,1.41421356f,1.41421356f};

    /* Walls are passable at a HUGE cost rather than blocked. Where any open
     * route exists it always wins (a detour costs < this), so phi still guides
     * the horde AROUND obstacles. But a fully walled-off goal is no longer
     * unreachable: phi keeps decreasing toward it through the walls, so the
     * horde is drawn to PRESS against the surrounding walls (siege) instead of
     * ignoring the goal. Map detours top out well under WALL_ENTER.            */
    const float WALL_ENTER = 5000.0f;

    while (h.n) {
        HeapNode cur = heap_pop(&h);
        if (cur.dist > s->phi[cur.idx]) continue; /* stale */
        int cx = cur.idx % W, cy = cur.idx / W;
        for (int k = 0; k < 8; k++) {
            int nx = cx + dx[k], ny = cy + dy[k];
            if (nx < 0 || ny < 0 || nx >= W || ny >= H) continue;
            int ni = ny*W + nx;
            float nd = cur.dist + dc[k] + (is_wall(s, ni) ? WALL_ENTER : 0.0f);
            if (nd < s->phi[ni]) { s->phi[ni] = nd; heap_push(&h, nd, ni); }
        }
    }
    free(h.a);
    s->phi_dirty = 0;
}

/* ------------------------------------------------------------------ */
/* one step                                                            */
/* ------------------------------------------------------------------ */
static inline float frand01(void) { return (float)rand() * (1.0f / (float)RAND_MAX); }

/* Unit vector down -grad(phi) at cell (ix,iy): "which way to the goal". Zero in
 * unreachable pockets / flat spots. Border is sealed so neighbours are valid. */
static void goal_dir(const Sim *s, int ix, int iy, float *ox, float *oy) {
    int W = s->W, i = iy*W + ix;
    float pc = s->phi[i];
    if (pc >= SIM_INF) { *ox = 0.0f; *oy = 0.0f; return; }
    float pl = s->phi[i-1], pr = s->phi[i+1];
    float pu = s->phi[i-W], pd = s->phi[i+W];
    /* neutralize wall / unreachable neighbours: the slope comes from passable
     * cells, so we steer along the basin without a spurious "flee the wall"
     * kick (the wall's own phi is huge by design). */
    if (pl >= SIM_INF || is_wall(s, i-1)) pl = pc;
    if (pr >= SIM_INF || is_wall(s, i+1)) pr = pc;
    if (pu >= SIM_INF || is_wall(s, i-W)) pu = pc;
    if (pd >= SIM_INF || is_wall(s, i+W)) pd = pc;
    float gx = pl - pr, gy = pu - pd;     /* >0 => goal right / down */
    float m  = sqrtf(gx*gx + gy*gy);
    if (m < 1e-6f) { *ox = 0.0f; *oy = 0.0f; return; }
    *ox = gx / m; *oy = gy / m;
}

/* Append one particle inside cell (x,y), jittered. Returns 1 if emitted. */
static int emit_at(Sim *s, int x, int y) {
    if (s->pcount >= s->pcap) return 0;
    int k = s->pcount++;
    s->px[k]  = (float)x + frand01();
    s->py[k]  = (float)y + frand01();
    s->pvx[k] = 0.0f; s->pvy[k] = 0.0f;
    s->pseed[k] = (unsigned char)(rand() & 0xff);
    return 1;
}

int sim_emit(Sim *s, int x, int y, int n) {
    if (x < 1 || y < 1 || x >= s->W-1 || y >= s->H-1) return 0;
    if (is_wall(s, sim_idx(s,x,y))) return 0;
    int e = 0;
    for (int k = 0; k < n; k++) e += emit_at(s, x, y);
    return e;
}

/* Births: every enabled spawner cell emits spawn_per_step particles/step, with
 * fractional carryover so low rates still work. Conserved thereafter. */
static void particles_spawn(Sim *s) {
    /* stop entirely at the cap (and don't keep accumulating, or we'd dump a
     * burst the moment a drain frees a slot). */
    if (!s->spawn_enabled || s->pcount >= s->spawn_max) return;
    int W = s->W, H = s->H;
    float acc = s->spawn_accum;
    for (int y = 1; y < H-1; y++)
    for (int x = 1; x < W-1; x++) {
        int i = y*W + x;
        if (!(s->flag[i] & CELL_SPAWN) || is_wall(s, i)) continue;
        acc += s->spawn_per_step;
        while (acc >= 1.0f && s->pcount < s->spawn_max) {
            if (!emit_at(s, x, y)) { acc = 0.0f; goto done; }  /* pool full */
            acc -= 1.0f;
        }
    }
done:
    s->spawn_accum = acc;
}

/* Spatial hash over cells (counting sort), rebuilt each step from current
 * positions. Gives, per cell c, the contiguous range pidx[cell_start[c] ..
 * cell_start[c+1]) of particle indices in it. GPU-port-friendly. */
static void particles_hash(Sim *s) {
    int W = s->W; size_t n = (size_t)W * s->H;
    memset(s->cell_count, 0, n * sizeof(int));
    for (int p = 0; p < s->pcount; p++) {
        int c = (int)s->py[p] * W + (int)s->px[p];
        s->cell_count[c]++;
    }
    int acc = 0;
    for (size_t c = 0; c < n; c++) { s->cell_start[c] = acc; acc += s->cell_count[c]; }
    s->cell_start[n] = acc;
    memset(s->cell_count, 0, n * sizeof(int));    /* reuse as running cursor */
    for (int p = 0; p < s->pcount; p++) {
        int c = (int)s->py[p] * W + (int)s->px[p];
        s->pidx[s->cell_start[c] + s->cell_count[c]++] = p;
    }
}

/* DERIVED density: bin particle counts per cell. Read by the game layer
 * (turret targeting, economy) and by the debug field view. */
static void particles_rasterize(Sim *s) {
    size_t n = (size_t)s->W * s->H;
    memset(s->rho, 0, n * sizeof(float));
    for (int p = 0; p < s->pcount; p++)
        s->rho[(int)s->py[p] * s->W + (int)s->px[p]] += 1.0f;
}

/* Deaths: a particle sitting on a drain cell is consumed at drain_rate. This is
 * the ONLY way (besides nothing) a particle leaves the world. swap-remove. */
static void particles_drain(Sim *s) {
    int W = s->W;
    for (int p = 0; p < s->pcount; ) {
        int c = (int)s->py[p] * W + (int)s->px[p];
        if ((s->flag[c] & CELL_DRAIN) && frand01() < s->drain_rate) {
            int last = --s->pcount;
            s->px[p]  = s->px[last];  s->py[p]  = s->py[last];
            s->pvx[p] = s->pvx[last]; s->pvy[p] = s->pvy[last];
            s->pseed[p] = s->pseed[last];
            s->drained_total += 1.0;
            continue;                                  /* re-check swapped-in */
        }
        p++;
    }
}

/* Swap the working position buffers (px<->px2). */
static inline void swap_pos(Sim *s) {
    float *t;
    t = s->px; s->px = s->px2; s->px2 = t;
    t = s->py; s->py = s->py2; s->py2 = t;
}

/* The heart of the granular horde. For every particle:
 *   1) steer toward the goal (-grad phi) at a cruise speed, keep some momentum,
 *      add the explosion impulse field (vx,vy) and a little jitter; clamp;
 *   2) tentative move, sliding along any wall-blocked axis;
 *   3) PBD collision resolve: short-range repulsion pushes overlapping
 *      neighbours apart (this is the packing / angle-of-repose / spreading),
 *      and keeps everyone out of walls. Jacobi double-buffered (px<->px2),
 *      so it is order-independent and compute-shader-friendly.
 * Velocity is taken as the tentative velocity; friction in step 1 settles it.
 * The hash (cell_start/pidx) is built once per step on the pre-step positions;
 * since speed < 1 cell/step, neighbours stay within the searched 3x3 block. */
static void particles_step(Sim *s) {
    int W = s->W, H = s->H;
    const float PMAX  = 0.85f;                 /* < 1 cell/step keeps hash valid */
    const float twoR  = 2.0f * s->p_radius;
    const float twoR2 = twoR * twoR;
    const int   R     = (int)ceilf(twoR);   /* neighbour-search half-width, in cells */
    const float fr    = s->p_friction;
    /* jam factor maps ABSOLUTE local density 1..target -> 0..1 (a packed cell is
     * "jammed" even once it is AT target, not only when over it — that was the
     * bug that left the settled centre trembling at full jitter). */
    const float jden_inv = 1.0f / fmaxf(0.5f, s->p_rho_target - 1.0f);

    /* remember where everyone started, to recover true velocity post-collision */
    memcpy(s->psx, s->px, (size_t)s->pcount * sizeof(float));
    memcpy(s->psy, s->py, (size_t)s->pcount * sizeof(float));

    /* Grid pressure field P = excess density over target (uses last step's rho,
     * a one-step lag is fine). Pairwise repulsion only spreads a pile from its
     * rim; this propagates through the INTERIOR, so an over-packed blob relaxes
     * to ~p_rho_target everywhere. Below target P=0 => free flow, unaffected. */
    size_t n = (size_t)W * H;
    for (size_t c = 0; c < n; c++) {
        float e = s->rho[c] - s->p_rho_target;
        s->d[c] = e > 0.0f ? e : 0.0f;
    }

    /* 1+2) forces + tentative integration -> px2/py2 */
    for (int p = 0; p < s->pcount; p++) {
        int ix = (int)s->px[p], iy = (int)s->py[p];
        int i  = iy*W + ix;
        float dx, dy; goal_dir(s, ix, iy, &dx, &dy);
        float Dx = dx * s->p_goal_speed, Dy = dy * s->p_goal_speed;

        /* jam factor from local over-density: 0 in free flow, 1 deep in a pile.
         * A jammed pile should act like high static friction (freeze), not
         * tremble — so we damp the steering (momentum+goal) and kill the random
         * jitter where packed. Pressure & impulses below are LEFT UNDAMPED, so a
         * packed blob can still decompress/spread and explosions still fling it. */
        float jam = (s->rho[i] - 1.0f) * jden_inv;
        if (jam < 0.0f) jam = 0.0f; else if (jam > 1.0f) jam = 1.0f;
        float steer = 1.0f - s->p_jam_damp * jam;
        float jit   = s->p_jitter * (1.0f - jam);

        float tvx = (s->pvx[p]*fr + Dx*(1.0f - fr))*steer + s->vx[i] + (frand01()-0.5f)*jit;
        float tvy = (s->pvy[p]*fr + Dy*(1.0f - fr))*steer + s->vy[i] + (frand01()-0.5f)*jit;

        /* grid pressure: push toward lower excess-density (-grad P), wall
         * neighbours neutralized so we don't get pushed into walls. */
        float Pc = s->d[i];
        float Pl = is_wall(s,i-1) ? Pc : s->d[i-1];
        float Pr = is_wall(s,i+1) ? Pc : s->d[i+1];
        float Pu = is_wall(s,i-W) ? Pc : s->d[i-W];
        float Pd = is_wall(s,i+W) ? Pc : s->d[i+W];
        tvx += s->p_press_k * (Pl - Pr);
        tvy += s->p_press_k * (Pu - Pd);

        float sp2 = tvx*tvx + tvy*tvy;
        if (sp2 > PMAX*PMAX) { float k = PMAX / sqrtf(sp2); tvx *= k; tvy *= k; }

        float nx = s->px[p] + tvx, ny = s->py[p] + tvy;
        if (is_wall(s, iy*W + (int)nx)) { nx = s->px[p]; }
        if (is_wall(s, (int)ny*W + ix)) { ny = s->py[p]; }
        s->px2[p] = nx; s->py2[p] = ny;
    }
    swap_pos(s);                                /* px now holds tentative */

    /* 3) PBD collision resolve. The hash is rebuilt EACH iteration on the
     * current positions: as grains shove apart they cross cell boundaries, so
     * stale neighbour lists would let overlaps slip through and the density
     * would never converge to the packing limit. Rebuilding makes the grains
     * behave rigidly => per-cell density is capped => a blocked pile SPREADS
     * laterally instead of compressing into an ever-denser blob. */
    for (int it = 0; it < s->p_collide_iters; it++) {
        particles_hash(s);
        for (int p = 0; p < s->pcount; p++) {
            float ax = s->px[p], ay = s->py[p];
            int cx = (int)ax, cy = (int)ay;
            float pushx = 0.0f, pushy = 0.0f;
            for (int oy = -R; oy <= R; oy++)
            for (int ox = -R; ox <= R; ox++) {
                int c = (cy+oy)*W + (cx+ox);
                if (c < 0 || c >= W*H) continue;
                int beg = s->cell_start[c], end = s->cell_start[c+1];
                for (int q = beg; q < end; q++) {
                    int b = s->pidx[q];
                    if (b == p) continue;
                    float ddx = ax - s->px[b], ddy = ay - s->py[b];
                    float d2 = ddx*ddx + ddy*ddy;
                    if (d2 >= twoR2 || d2 < 1e-8f) continue;
                    float d = sqrtf(d2), inv = 1.0f / d;
                    float ov = (twoR - d) * 0.5f * s->p_repel;
                    pushx += ddx * inv * ov;
                    pushy += ddy * inv * ov;
                }
            }
            /* cap the per-iteration shove so a large radius can't fling a
             * particle past its neighbour list; overlaps resolve over iters. */
            float pm2 = pushx*pushx + pushy*pushy;
            if (pm2 > twoR2) { float k = twoR / sqrtf(pm2); pushx *= k; pushy *= k; }
            float nx = ax + pushx, ny = ay + pushy;
            if (nx < 1.0f) nx = 1.0f; else if (nx > (float)(W-1)) nx = (float)(W-1) - 1e-3f;
            if (ny < 1.0f) ny = 1.0f; else if (ny > (float)(H-1)) ny = (float)(H-1) - 1e-3f;
            if (is_wall(s, (int)ny*W + (int)nx)) { nx = ax; ny = ay; }  /* don't enter walls */
            s->px2[p] = nx; s->py2[p] = ny;
        }
        swap_pos(s);                            /* px holds the resolved positions */
    }

    /* true velocity = net displacement this step (back-pressure: a particle the
     * pile won't let move gets ~0 velocity). We also damp the carried velocity
     * where it's dense, so a settled centre stops oscillating — pressure is a
     * fresh force each step, so decompression of an OVER-packed blob still works. */
    for (int p = 0; p < s->pcount; p++) {
        int c = (int)s->py[p]*W + (int)s->px[p];
        float jam = (s->rho[c] - 1.0f) * jden_inv;
        if (jam < 0.0f) jam = 0.0f; else if (jam > 1.0f) jam = 1.0f;
        float k = 1.0f - s->p_jam_damp * jam;
        s->pvx[p] = (s->px[p] - s->psx[p]) * k;
        s->pvy[p] = (s->py[p] - s->psy[p]) * k;
    }
}

/* Multiplicative friction + CFL clamp + tiny-value cull (avoids jitter).   */
static void damp_velocity(Sim *s) {
    size_t n = (size_t)s->W * s->H;
    float k = s->v_damp, vmax = s->v_max;
    for (size_t i = 0; i < n; i++) {
        float vx = s->vx[i] * k, vy = s->vy[i] * k;
        if (vx >  vmax) vx =  vmax; else if (vx < -vmax) vx = -vmax;
        if (vy >  vmax) vy =  vmax; else if (vy < -vmax) vy = -vmax;
        if (fabsf(vx) < 1e-4f) vx = 0.0f;
        if (fabsf(vy) < 1e-4f) vy = 0.0f;
        s->vx[i] = vx; s->vy[i] = vy;
    }
}

void sim_step(Sim *s) {
    if (s->phi_dirty) sim_recompute_phi(s);
    particles_spawn(s);         /* births at spawners                   */
    particles_step(s);          /* steer + integrate + granular collide */
    particles_drain(s);         /* deaths at drains                     */
    damp_velocity(s);           /* decay the explosion impulse field    */
    particles_rasterize(s);     /* derive rho for gameplay / debug view */
}
