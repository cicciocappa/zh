/* test_dump.c — headless verification harness (no SDL).
 * Builds a scene exercising the three target behaviours, runs the sim, and
 * writes PPM frames so we can confirm stability + correct flow/pile/overflow.
 *
 * Scene layout (W x H grid):
 *   - SPAWNER strip top-left  -> a steady supply of horde
 *   - GOAL strip bottom-right -> what the horde streams toward
 *   - a WALL with a narrow GAP across the upper-middle  -> chokepoint/congestion
 *   - a fully WALLED BOX with its own spawner inside     -> fills & packs (sealed)
 *   - a DRAIN patch near the goal                        -> consumes particles
 *
 * Model note: the horde is now a persistent PARTICLE population (sim.c). rho is
 * a DERIVED per-cell count, so total rho == live particle count. We verify the
 * particle invariants: none inside walls, none out of bounds, no NaN.
 */
#include "sim.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

static void heat(float t, unsigned char *r, unsigned char *g, unsigned char *b) {
    /* 0 -> dark, mid -> green/amber, high -> hot red (a "biomass" ramp) */
    if (t < 0) t = 0; if (t > 1) t = 1;
    float rr, gg, bb;
    if (t < 0.5f) { float u = t/0.5f; rr = 0.10f+0.20f*u; gg = 0.15f+0.70f*u; bb = 0.10f; }
    else          { float u = (t-0.5f)/0.5f; rr = 0.30f+0.70f*u; gg = 0.85f-0.70f*u; bb = 0.10f-0.10f*u; }
    *r=(unsigned char)(rr*255); *g=(unsigned char)(gg*255); *b=(unsigned char)(bb*255);
}

static void dump_ppm(const Sim *s, const char *path) {
    FILE *f = fopen(path, "wb");
    fprintf(f, "P6\n%d %d\n255\n", s->W, s->H);
    for (int y = 0; y < s->H; y++)
    for (int x = 0; x < s->W; x++) {
        int i = sim_idx(s,x,y);
        unsigned char r,g,b;
        if (s->bed[i] > 0.0f && s->bed[i] < 1e5f) { r=70; g=70; b=85; }       /* wall  */
        else if (s->bed[i] >= 1e5f)               { r=30; g=30; b=38; }       /* border*/
        else if (s->flag[i] & CELL_GOAL)          { r=60; g=120; b=255; }     /* goal  */
        else if (s->flag[i] & CELL_DRAIN)         { r=20; g=20; b=20; }       /* drain */
        else {
            float t = s->rho[i] / 4.0f;           /* rho = particles-per-cell now */
            heat(sqrtf(t > 1 ? 1 : t), &r,&g,&b); /* sqrt for visible low densities */
        }
        fputc(r,f); fputc(g,f); fputc(b,f);
    }
    fclose(f);
}

static void hwall(Sim *s, int y, int x0, int x1, float h, int gap0, int gap1) {
    for (int x = x0; x <= x1; x++) if (x < gap0 || x > gap1) sim_set_wall(s, x, y, h);
}

int main(void) {
    const int W = 200, H = 200;
    Sim *s = sim_create(W, H);

    /* goal: bottom-right block */
    for (int y = H-14; y < H-6; y++) for (int x = W-30; x < W-10; x++) sim_set_flag(s,x,y,CELL_GOAL,1);

    /* spawner: top-left strip */
    for (int y = 8; y < 16; y++) for (int x = 10; x < 40; x++) sim_set_flag(s,x,y,CELL_SPAWN,1);

    /* chokepoint: long wall across with a 10-cell gap */
    hwall(s, 70, 6, W-7, 9.0f, 96, 106);

    /* overflow demo: a small sealed box (walls all four sides) with a spawner inside */
    int bx0=130, bx1=170, by0=20, by1=55;
    for (int x = bx0; x <= bx1; x++) { sim_set_wall(s,x,by0,9.0f); sim_set_wall(s,x,by1,9.0f); }
    for (int y = by0; y <= by1; y++) { sim_set_wall(s,bx0,y,9.0f); sim_set_wall(s,bx1,y,9.0f); }
    for (int y = by0+6; y < by0+14; y++) for (int x = bx0+6; x < bx0+20; x++) sim_set_flag(s,x,y,CELL_SPAWN,1);

    /* drain patch guarding the goal */
    for (int y = H-26; y < H-20; y++) for (int x = W-40; x < W-20; x++) sim_set_flag(s,x,y,CELL_DRAIN,1);

    sim_recompute_phi(s);

    const int STEPS = 900, EVERY = 18;
    int frame = 0;
    for (int step = 0; step <= STEPS; step++) {
        if (step % EVERY == 0) {
            char path[64]; snprintf(path, sizeof path, "frames/f%04d.ppm", frame++);
            dump_ppm(s, path);
        }
        sim_step(s);
    }

    /* particle invariants: no NaN, in bounds, never inside a wall */
    int bad = 0, oob = 0, in_wall = 0; float mx = 0;
    for (int p = 0; p < s->pcount; p++) {
        float x = s->px[p], y = s->py[p];
        if (!(x == x) || !(y == y)) { bad++; continue; }
        if (x < 1.0f || y < 1.0f || x >= W-1 || y >= H-1) { oob++; continue; }
        if (s->bed[(int)y*W + (int)x] > 0.0f) in_wall++;
    }
    /* density stats. The sealed box (bx0..bx1,by0..by1) is a pathological
     * spawn-into-full case; report max OUTSIDE it + a histogram so we can see
     * the typical open-flow density, not the outlier. */
    float mx_open = 0; long occ = 0; long hist[8] = {0};
    for (int y = 0; y < H; y++)
    for (int x = 0; x < W; x++) {
        float v = s->rho[y*W + x];
        if (v > mx) mx = v;
        int inbox = (x >= bx0 && x <= bx1 && y >= by0 && y <= by1);
        if (!inbox && v > mx_open) mx_open = v;
        if (v > 0.5f) { occ++; int b = (int)v; if (b > 7) b = 7; hist[b]++; }
    }
    printf("frames=%d  pop=%d  max_cell=%.0f  max_open=%.0f  drained=%.0f  in_wall=%d oob=%d nan=%d\n",
           frame, s->pcount, mx, mx_open, s->drained_total, in_wall, oob, bad);
    printf("occupied_cells=%ld  density histogram (cells with N particles):\n", occ);
    for (int b = 1; b <= 7; b++)
        printf("  N=%d%s : %ld\n", b, b==7?"+":" ", hist[b]);
    sim_free(s);
    return (bad || oob || in_wall) ? 1 : 0;
}
