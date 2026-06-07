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
    return s;
}

void sim_free(Sim *s) {
    if (!s) return;
    free(s->rho); free(s->bed); free(s->phi);
    free(s->vx);  free(s->vy);
    free(s->d);   free(s->flag);
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
        s->rho[i] *= (1.0f - 0.9f * f);                       /* carve up to -90%      */
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

    while (h.n) {
        HeapNode cur = heap_pop(&h);
        if (cur.dist > s->phi[cur.idx]) continue; /* stale */
        int cx = cur.idx % W, cy = cur.idx / W;
        for (int k = 0; k < 8; k++) {
            int nx = cx + dx[k], ny = cy + dy[k];
            if (nx < 0 || ny < 0 || nx >= W || ny >= H) continue;
            int ni = ny*W + nx;
            if (is_wall(s, ni)) continue;              /* cannot path through walls */
            float nd = cur.dist + dc[k];
            if (nd < s->phi[ni]) { s->phi[ni] = nd; heap_push(&h, nd, ni); }
        }
    }
    free(h.a);
    s->phi_dirty = 0;
}

/* ------------------------------------------------------------------ */
/* one step                                                            */
/* ------------------------------------------------------------------ */
static void advect_toward_goal(Sim *s) {
    int W = s->W, H = s->H; size_t n = (size_t)W*H;
    memset(s->d, 0, n * sizeof(float));

    for (int y = 1; y < H-1; y++)
    for (int x = 1; x < W-1; x++) {
        int i = y*W + x;
        if (is_wall(s,i)) continue;
        float r = s->rho[i];
        if (r <= 1e-6f) continue;
        float pc = s->phi[i];
        if (pc >= SIM_INF) continue;       /* unreachable pocket: just pile */

        /* desired direction = -grad(phi), pointing toward the goal */
        float pl = s->phi[i-1],  pr = s->phi[i+1];
        float pu = s->phi[i-W],  pd = s->phi[i+W];
        if (pl >= SIM_INF) pl = pc;
        if (pr >= SIM_INF) pr = pc;
        if (pu >= SIM_INF) pu = pc;
        if (pd >= SIM_INF) pd = pc;
        float gx = pl - pr;        /* >0 => goal is to the right */
        float gy = pu - pd;        /* >0 => goal is downward     */
        float mag = fabsf(gx) + fabsf(gy);
        if (mag < 1e-6f) continue;

        float total = s->flow * r;
        float wx = fabsf(gx) / mag, wy = fabsf(gy) / mag;

        /* x component: only move if the target neighbour is passable & downhill */
        float mx = 0.0f, my = 0.0f;
        int xn = (gx > 0) ? i+1 : i-1;
        if (!is_wall(s,xn) && s->phi[xn] < pc) mx = total * wx;
        int yn = (gy > 0) ? i+W : i-W;
        if (!is_wall(s,yn) && s->phi[yn] < pc) my = total * wy;

        float out = mx + my;
        if (out <= 0.0f) continue;
        s->d[i]  -= out;            /* whatever we can't push simply stays (conservative) */
        s->d[xn] += mx;
        s->d[yn] += my;
    }
    for (size_t i = 0; i < n; i++) { s->rho[i] += s->d[i]; if (s->rho[i] < 0) s->rho[i] = 0; }
}

/* Shallow-water surface equalization with a bed/sill. This is what makes the
 * horde pile against walls and SPILL OVER once deep enough. Jacobi sweeps with
 * a small rate => unconditionally stable. */
static void equalize_surface(Sim *s) {
    int W = s->W, H = s->H; size_t n = (size_t)W*H;
    for (int it = 0; it < s->relax_iters; it++) {
        memset(s->d, 0, n * sizeof(float));
        for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++) {
            int i = y*W + x;
            /* process the right and the down interface once each */
            for (int axis = 0; axis < 2; axis++) {
                int j = axis ? i + W : i + 1;
                if (axis == 0 && x >= W-1) continue;
                if (axis == 1 && y >= H-1) continue;

                float sa = s->bed[i] + s->rho[i];
                float sb = s->bed[j] + s->rho[j];
                if (sa == sb) continue;

                /* the sill between the two cells: only water ABOVE the higher
                 * bed can pass -> tall walls hold the horde back until overtopped */
                float sill = s->bed[i] > s->bed[j] ? s->bed[i] : s->bed[j];

                float move; /* signed: +ve means i -> j */
                if (sa > sb) {
                    float head = sa - sill;            /* spillable column in i */
                    if (head <= 0) continue;
                    move = s->relax_rate * (sa - sb);
                    if (move > head)      move = head;
                    if (move > s->rho[i]) move = s->rho[i];
                } else {
                    float head = sb - sill;            /* spillable column in j */
                    if (head <= 0) continue;
                    move = -(s->relax_rate * (sb - sa));
                    if (-move > head)      move = -head;
                    if (-move > s->rho[j]) move = -s->rho[j];
                }
                s->d[i] -= move; s->d[j] += move;
            }
        }
        for (size_t i = 0; i < n; i++) { s->rho[i] += s->d[i]; if (s->rho[i] < 0) s->rho[i] = 0; }
    }
}

/* Upwind advection of rho along the velocity field. CFL-safe as long as
 * |vx|,|vy| <= v_max < 1: each cell can dump at most (|vx|+|vy|)*rho, which
 * we additionally rescale if the two outflows together would exceed rho.
 * Hitting a wall flips that velocity component (half-elastic bounce).      */
static void advect_by_velocity(Sim *s) {
    int W = s->W, H = s->H; size_t n = (size_t)W*H;
    memset(s->d, 0, n * sizeof(float));
    for (int y = 1; y < H-1; y++)
    for (int x = 1; x < W-1; x++) {
        int i = y*W + x;
        if (is_wall(s,i)) continue;
        float r = s->rho[i];
        if (r <= 1e-6f) continue;
        float vx = s->vx[i], vy = s->vy[i];
        if (fabsf(vx) + fabsf(vy) < 1e-6f) continue;

        int xn = (vx > 0) ? i+1 : i-1;
        int yn = (vy > 0) ? i+W : i-W;
        float out_x = fabsf(vx) * r;
        float out_y = fabsf(vy) * r;
        if (is_wall(s, xn)) { out_x = 0.0f; s->vx[i] = -0.5f * s->vx[i]; }
        if (is_wall(s, yn)) { out_y = 0.0f; s->vy[i] = -0.5f * s->vy[i]; }
        float out = out_x + out_y;
        if (out > r) { float k = r / out; out_x *= k; out_y *= k; }

        s->d[i]  -= (out_x + out_y);
        s->d[xn] += out_x;
        s->d[yn] += out_y;
    }
    for (size_t i = 0; i < n; i++) { s->rho[i] += s->d[i]; if (s->rho[i] < 0) s->rho[i] = 0; }
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

static void apply_sources_drains(Sim *s) {
    size_t n = (size_t)s->W * s->H;
    for (size_t i = 0; i < n; i++) {
        if (s->spawn_enabled && (s->flag[i] & CELL_SPAWN)) {
            if (s->rho[i] < s->spawn_cap) s->rho[i] += s->spawn_amt;
        }
        if (s->flag[i] & CELL_DRAIN) {
            float removed = s->rho[i] * s->drain_rate;
            s->rho[i] -= removed;
            s->drained_total += removed;
        }
    }
}

void sim_step(Sim *s) {
    if (s->phi_dirty) sim_recompute_phi(s);
    apply_sources_drains(s);    /* inject / consume                     */
    advect_toward_goal(s);      /* directed flow down the potential     */
    advect_by_velocity(s);      /* impulse-driven kick layer            */
    damp_velocity(s);           /* friction + CFL clamp                 */
    equalize_surface(s);        /* pile, spread, overflow               */
}
