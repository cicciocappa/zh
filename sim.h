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

    /* -------------------------------------------------------------------- *
     * PARTICLE HORDE — the authoritative agents (granular, persistent).
     * The grid above is now infrastructure: phi = shared goal force,
     * cells = spatial hash, rho = density DERIVED from particle binning.
     * Particles are born only at spawners, die only at drains; otherwise
     * conserved. Packing/angle-of-repose comes from short-range repulsion,
     * not from surface equalization.  SoA layout, GPU-port-friendly.        */
    int    pcount, pcap;          /* live count / allocated capacity         */
    float *px,  *py;              /* positions (grid coords, float)          */
    float *pvx, *pvy;             /* velocities (cells/step)                 */
    float *px2, *py2;             /* double buffer for Jacobi collision pass */
    float *psx, *psy;             /* start-of-step positions (PBD velocity)  */
    unsigned char *pseed;         /* per-particle random byte (sprite/jitter)*/

    /* spatial hash over cells (counting sort, rebuilt each step) */
    int   *cell_start;            /* size n+1: prefix offsets into pidx      */
    int   *cell_count;            /* size n:   transient per-cell tally      */
    int   *pidx;                  /* size pcap: particle indices, cell-sorted*/

    /* particle tunables */
    float p_radius;     /* interaction/packing radius (grid units)          */
    float p_repel;      /* repulsion stiffness (fraction of overlap pushed) */
    int   p_collide_iters; /* Jacobi resolve sweeps per step                */
    float p_goal_speed; /* target cruise speed down -grad(phi)              */
    float p_friction;   /* velocity retention per step (granular damping)   */
    float p_jitter;     /* small random walk amplitude                      */
    float spawn_per_step; /* particles emitted per spawner cell per step    */
    float spawn_accum;  /* fractional spawn carryover                       */
    int   spawn_max;    /* stop spawning once pcount reaches this. With no
                           sink the horde would otherwise grow forever and a
                           pile never settles; this caps it.                 */
    float p_press_k;    /* grid-pressure strength: push down -grad(excess)  */
    float p_rho_target; /* density (particles/cell) above which pressure acts*/
    float p_jam_damp;   /* density-driven freezing: 0=off, ~1=dense piles lock
                           up (high static friction) instead of trembling     */
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

/* Manually emit n particles inside cell (x,y) (jittered). Used by tools/tests;
 * the normal source of births is spawners during sim_step. Returns # emitted. */
int  sim_emit(Sim *s, int x, int y, int n);

/* Advance one simulation step. */
void sim_step(Sim *s);

static inline int sim_idx(const Sim *s, int x, int y) { return y * s->W + x; }

#endif /* SIM_H */
