/* sim.h — Horde-as-granular-fluid simulation core.
 *
 * Renderer-agnostic. No external dependencies. Single-precision floats.
 *
 * Model in one breath:
 *   The horde is a CONTINUUM (a density/depth field on a grid), not a set of
 *   pathfinding agents. Two forces move it:
 *     1) GOAL FORCING  — a potential field phi (cost-to-goal). Density is
 *        advected DOWN the gradient of phi. This is "gravity toward objectives"
 *        and replaces real gravity in our top-down world. -> where they WANT to go.
 *     2) SURFACE PRESSURE — a shallow-water-style equalization over a bed
 *        elevation. Walls are just cells with a high bed; the crowd piles
 *        against them and, once its surface (bed+depth) exceeds the wall top,
 *        SPILLS OVER (weir overflow). -> how they physically pile / spread / overflow.
 *
 * Lineage: a deliberately simplified cousin of Continuum Crowds (Treuille 2006)
 * and shallow-water/pipe fluid models. We drop momentum and the Poisson
 * incompressibility solve on purpose: the medium must be COMPRESSIBLE up to a
 * packing limit (it piles), and the relaxation form is unconditionally stable,
 * which is what you want in a game loop.
 */
#ifndef SIM_H
#define SIM_H

#include <stddef.h>

#define SIM_INF 1e30f

/* Per-cell flags (bitmask). */
enum {
    CELL_SPAWN = 1u << 0, /* injects density each step                    */
    CELL_DRAIN = 1u << 1, /* removes density each step (pit/water/turret) */
    CELL_GOAL  = 1u << 2  /* potential-field source (objective / attractor)*/
};

typedef struct {
    int   W, H;        /* grid dimensions                                  */
    float *rho;        /* density / pile depth, >= 0                       */
    float *bed;        /* bed/barrier elevation; floor=0, wall=tall (>0)   */
    float *phi;        /* potential: cost-to-goal (SIM_INF = unreachable)  */
    float *vx, *vy;    /* collocated velocity field (cell/step). Fed by
                          impulses (explosions); decays each step. Does NOT
                          self-advect — it is just a kick layer riding on
                          top of the relaxation model.                     */
    unsigned char *flag;

    /* scratch buffers (size W*H) */
    float *d;          /* per-step delta accumulator                       */

    /* tunables -------------------------------------------------------- */
    float flow;        /* goal-advection rate per step,   ~0.05..0.25      */
    float relax_rate;  /* surface equalization rate,      ~0.05..0.22      */
    int   relax_iters; /* equalization sweeps per step. FEW = granular /
                          steep piles (high angle of repose);
                          MANY = watery / self-leveling.  This single knob
                          is your "granularity" dial.                      */
    float spawn_amt;   /* density added per spawn cell per step            */
    float spawn_cap;   /* spawn stops above this local depth               */
    float drain_rate;  /* fraction of depth removed at drain cells/step    */
    int   spawn_enabled; /* master switch: 0 stops all CELL_SPAWN injection.
                            Drains keep running. Game layer flips this when
                            a survival timer expires.                       */
    float v_damp;      /* per-step multiplicative friction on (vx,vy)      */
    float v_max;       /* CFL clamp: |vx|,|vy| <= v_max < 1 cell/step      */

    /* bookkeeping */
    int   phi_dirty;   /* set when bed/goals change -> recompute phi       */
    double drained_total; /* cumulative removed mass (future: biomass/score)*/
} Sim;

Sim *sim_create(int W, int H);
void sim_free(Sim *s);

/* Convenience cell editors (mark phi_dirty where needed). */
void sim_set_wall (Sim *s, int x, int y, float height); /* height<=0 clears */
void sim_set_flag (Sim *s, int x, int y, unsigned char flag, int on);
void sim_add_rho  (Sim *s, int x, int y, float amount);
void sim_clear    (Sim *s); /* zero rho and velocity, keep terrain */

/* Velocity injectors. Used by explosions, knock-back, wind, etc.          */
void sim_add_velocity(Sim *s, int x, int y, float vx, float vy);
/* Radial outward kick + density carve in a disk. `strength` is the peak
 * velocity magnitude (cell/step); falloff is linear to zero at `radius`.  */
void sim_add_impulse (Sim *s, int cx, int cy, int radius, float strength);

/* Recompute the potential field (Dijkstra, 8-neighbourhood, walls blocked). */
void sim_recompute_phi(Sim *s);

/* Advance one simulation step. */
void sim_step(Sim *s);

static inline int sim_idx(const Sim *s, int x, int y) { return y * s->W + x; }

#endif /* SIM_H */
