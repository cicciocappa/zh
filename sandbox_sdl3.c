/* sandbox_sdl3.c — interactive sandbox over the same sim core.
 *
 * Paint terrain, drop spawners/goals/drains, turn the knobs live, and watch
 * the horde flow. The simulation core (sim.c/sim.h) has NO SDL dependency;
 * this file is purely the visualization + input layer, so the model ports
 * straight into your C/OpenGL pipeline (upload `rho` as a texture instead).
 *
 * Build (Linux, with SDL3 installed):
 *     cc -O2 -o sandbox sandbox_sdl3.c sim.c -lm $(pkg-config --cflags --libs sdl3)
 *
 * Controls:
 *     LMB            paint the current brush
 *     RMB            erase (clears walls + flags under the brush)
 *     1              brush = WALL
 *     2              brush = SPAWNER
 *     3              brush = GOAL
 *     4              brush = DRAIN
 *     [ / ]          brush size down / up
 *     - / =          fewer / more equalization iters (granular <-> watery)
 *     , / .          flow rate down / up
 *     SPACE          pause / resume
 *     N              single step (while paused)
 *     C              clear density (keep terrain)
 *     R              reset everything
 *     T              cycle survival timer (OFF -> 60s -> 180s -> OFF);
 *                    when it expires, all spawners stop injecting.
 *     E / MMB        explosion at cursor (radial impulse + density carve)
 *     V              toggle render: density field <-> particle sprites
 *     ESC            quit
 */
#include "sim.h"
#include <SDL3/SDL.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define GW 220
#define GH 220
#define CELL 3                 /* pixels per cell */
#define WINW (GW*CELL)
#define WINH (GH*CELL)
#define BOOM_RADIUS   16
#define BOOM_STRENGTH 0.90f

/* Particle view: persistent pool. Each cell has target = rho * SPRITES_PER_UNIT,
 * particles are spawned/killed to track it. Movement = field v + small phi
 * descent + jitter. Two-frame bobbing for life. Field is the truth; sprites
 * are just a view. */
#define MAX_PARTICLES    10000
#define SPRITES_PER_UNIT 0.30f
#define RHO_VISIBLE_MIN  0.5f
#define PHI_BIAS         0.12f
#define JITTER_AMP       0.05f

typedef struct { float x, y; unsigned char phase; } Particle;
static Particle parts[MAX_PARTICLES];
static int      n_alive = 0;
static int      cell_counts[GW*GH];

static float frand(void) { return (float)rand() / (float)RAND_MAX; }

enum { B_WALL, B_SPAWN, B_GOAL, B_DRAIN };
static const float WALL_H = 9.0f;

static void put_ramp(float t, Uint8 *r, Uint8 *g, Uint8 *b) {
    if (t < 0) t = 0;
    if (t > 1) t = 1;
    float rr, gg, bb;
    if (t < 0.5f) { float u = t/0.5f;       rr=0.10f+0.20f*u; gg=0.15f+0.70f*u; bb=0.10f; }
    else          { float u = (t-0.5f)/0.5f; rr=0.30f+0.70f*u; gg=0.85f-0.70f*u; bb=0.10f-0.10f*u; }
    *r=(Uint8)(rr*255); *g=(Uint8)(gg*255); *b=(Uint8)(bb*255);
}

static void render(Sim *s, Uint32 *px, int show_rho) {
    for (int y = 0; y < s->H; y++)
    for (int x = 0; x < s->W; x++) {
        int i = sim_idx(s,x,y);
        Uint8 r,g,b;
        if (s->bed[i] >= 1e5f)            { r=30; g=30; b=38; }
        else if (s->bed[i] > 0.0f)        { r=70; g=70; b=85; }
        else if (s->flag[i] & CELL_GOAL)  { r=60; g=120; b=255; }
        else if (s->flag[i] & CELL_DRAIN) { r=18; g=18; b=22; }
        else if (show_rho) {
            float t = s->rho[i] / 12.0f;
            put_ramp(sqrtf(t > 1 ? 1 : t), &r,&g,&b);
            if (s->flag[i] & CELL_SPAWN) { r=(Uint8)(r/2+90); }  /* tint spawners */
        }
        else {
            r=20; g=24; b=22;                                   /* dark floor */
            if (s->flag[i] & CELL_SPAWN) { r=90; g=40; b=40; }   /* spawner tint */
        }
        px[i] = 0xFF000000u | (r<<16) | (g<<8) | b;
    }
}

static void particles_update(Sim *s) {
    memset(cell_counts, 0, sizeof cell_counts);

    /* 1) advance alive particles; drop OOB / wall-hits; tally by cell. */
    for (int p = 0; p < n_alive; ) {
        Particle *pt = &parts[p];
        int cx = (int)pt->x, cy = (int)pt->y;
        if (cx < 1 || cy < 1 || cx >= GW-1 || cy >= GH-1 ||
            s->bed[cy*GW + cx] > 0.0f) {
            parts[p] = parts[--n_alive]; continue;
        }
        int ci = cy*GW + cx;
        float vx = s->vx[ci], vy = s->vy[ci];
        float pc = s->phi[ci];
        if (pc < SIM_INF) {
            float pl = s->phi[ci-1],  pr = s->phi[ci+1];
            float pu = s->phi[ci-GW], pd = s->phi[ci+GW];
            if (pl >= SIM_INF) pl = pc;
            if (pr >= SIM_INF) pr = pc;
            if (pu >= SIM_INF) pu = pc;
            if (pd >= SIM_INF) pd = pc;
            float gx = pl - pr, gy = pu - pd;
            float mag = fabsf(gx) + fabsf(gy);
            if (mag > 1e-6f) {
                float k = PHI_BIAS / mag;
                vx += gx * k; vy += gy * k;
            }
        }
        vx += (frand() - 0.5f) * JITTER_AMP;
        vy += (frand() - 0.5f) * JITTER_AMP;
        float nx = pt->x + vx, ny = pt->y + vy;
        int ncx = (int)nx, ncy = (int)ny;
        if (ncx < 1 || ncy < 1 || ncx >= GW-1 || ncy >= GH-1 ||
            s->bed[ncy*GW + ncx] > 0.0f) {
            /* stay put rather than tunneling into a wall */
            nx = (float)cx + 0.5f; ny = (float)cy + 0.5f;
            ncx = cx; ncy = cy;
        }
        pt->x = nx; pt->y = ny;
        cell_counts[ncy*GW + ncx]++;
        p++;
    }

    /* 2) cull cells whose population exceeds the target. */
    for (int p = 0; p < n_alive; ) {
        int cx = (int)parts[p].x, cy = (int)parts[p].y;
        int ci = cy*GW + cx;
        int target = (s->rho[ci] < RHO_VISIBLE_MIN)
                     ? 0
                     : (int)(s->rho[ci] * SPRITES_PER_UNIT);
        if (cell_counts[ci] > target) {
            cell_counts[ci]--;
            parts[p] = parts[--n_alive];
            continue;
        }
        p++;
    }

    /* 3) spawn into cells below target. */
    for (int y = 1; y < GH-1; y++)
    for (int x = 1; x < GW-1; x++) {
        int i = y*GW + x;
        if (s->bed[i] > 0.0f) continue;
        if (s->rho[i] < RHO_VISIBLE_MIN) continue;
        int target = (int)(s->rho[i] * SPRITES_PER_UNIT);
        while (cell_counts[i] < target && n_alive < MAX_PARTICLES) {
            Particle *pt = &parts[n_alive++];
            pt->x = (float)x + frand();
            pt->y = (float)y + frand();
            pt->phase = (unsigned char)(rand() & 0xff);
            cell_counts[i]++;
        }
    }
}

static void particles_render(SDL_Renderer *ren, int tick) {
    static SDL_FRect rects[MAX_PARTICLES];
    int n = n_alive;
    for (int p = 0; p < n; p++) {
        const Particle *pt = &parts[p];
        int frame = ((tick + pt->phase) >> 3) & 1;          /* swap every 8 ticks */
        float bob = frame ? 0.0f : -1.0f;
        rects[p].x = pt->x * (float)CELL - 1.5f;
        rects[p].y = pt->y * (float)CELL - 1.5f + bob;
        rects[p].w = 3.0f;
        rects[p].h = 3.0f;
    }
    SDL_SetRenderDrawColor(ren, 210, 60, 60, 255);
    SDL_RenderFillRects(ren, rects, n);
}

static void paint(Sim *s, int cx, int cy, int brush, int size, int erase) {
    for (int dy = -size; dy <= size; dy++)
    for (int dx = -size; dx <= size; dx++) {
        int x = cx+dx, y = cy+dy;
        if (x<=0||y<=0||x>=s->W-1||y>=s->H-1) continue;
        if (dx*dx+dy*dy > size*size) continue;
        if (erase) {
            sim_set_wall(s,x,y,0);
            sim_set_flag(s,x,y,CELL_SPAWN|CELL_GOAL|CELL_DRAIN,0);
            continue;
        }
        switch (brush) {
            case B_WALL:  sim_set_wall(s,x,y,WALL_H); break;
            case B_SPAWN: sim_set_flag(s,x,y,CELL_SPAWN,1); break;
            case B_GOAL:  sim_set_flag(s,x,y,CELL_GOAL,1);  break;
            case B_DRAIN: sim_set_flag(s,x,y,CELL_DRAIN,1); break;
        }
    }
}

int main(void) {
    if (!SDL_Init(SDL_INIT_VIDEO)) { SDL_Log("init: %s", SDL_GetError()); return 1; }
    SDL_Window   *win = SDL_CreateWindow("Horde fluid sandbox", WINW, WINH, 0);
    SDL_Renderer *ren = SDL_CreateRenderer(win, NULL);
    SDL_Texture  *tex = SDL_CreateTexture(ren, SDL_PIXELFORMAT_ARGB8888,
                                          SDL_TEXTUREACCESS_STREAMING, GW, GH);
    SDL_SetTextureScaleMode(tex, SDL_SCALEMODE_NEAREST);

    Sim *s = sim_create(GW, GH);
    static Uint32 px[GW*GH];

    int brush = B_WALL, size = 3, paused = 0, running = 1;
    int painting = 0, erasing = 0;
    int render_mode = 0;        /* 0 = density field, 1 = particle sprites */
    int tick = 0;               /* frame counter for sprite animation       */

    /* survival timer (wall-clock, frozen while paused). -1 = disabled. */
    int countdown_total_ms = -1, countdown_left_ms = 0;
    Uint64 prev_ticks = SDL_GetTicks();

    while (running) {
        Uint64 now_ticks = SDL_GetTicks();
        int dt_ms = (int)(now_ticks - prev_ticks);
        prev_ticks = now_ticks;
        if (countdown_total_ms > 0 && !paused) {
            countdown_left_ms -= dt_ms;
            if (countdown_left_ms <= 0) {
                countdown_left_ms = 0;
                s->spawn_enabled = 0;
            }
        }
        SDL_Event e;
        while (SDL_PollEvent(&e)) {
            switch (e.type) {
            case SDL_EVENT_QUIT: running = 0; break;
            case SDL_EVENT_MOUSE_BUTTON_DOWN:
                if (e.button.button == SDL_BUTTON_MIDDLE) {
                    sim_add_impulse(s, (int)e.button.x/CELL, (int)e.button.y/CELL,
                                    BOOM_RADIUS, BOOM_STRENGTH);
                    break;
                }
                if (e.button.button == SDL_BUTTON_LEFT)  painting = 1;
                if (e.button.button == SDL_BUTTON_RIGHT) erasing = 1;
                paint(s, (int)e.button.x/CELL, (int)e.button.y/CELL, brush, size,
                      e.button.button == SDL_BUTTON_RIGHT);
                break;
            case SDL_EVENT_MOUSE_BUTTON_UP:
                if (e.button.button == SDL_BUTTON_LEFT)  painting = 0;
                if (e.button.button == SDL_BUTTON_RIGHT) erasing = 0;
                break;
            case SDL_EVENT_MOUSE_MOTION:
                if (painting || erasing)
                    paint(s, (int)e.motion.x/CELL, (int)e.motion.y/CELL, brush, size, erasing);
                break;
            case SDL_EVENT_KEY_DOWN:
                switch (e.key.key) {
                case SDLK_ESCAPE: running = 0; break;
                case SDLK_1: brush = B_WALL;  break;
                case SDLK_2: brush = B_SPAWN; break;
                case SDLK_3: brush = B_GOAL;  break;
                case SDLK_4: brush = B_DRAIN; break;
                case SDLK_LEFTBRACKET:  if (size > 0) size--; break;
                case SDLK_RIGHTBRACKET: if (size < 20) size++; break;
                case SDLK_MINUS:  if (s->relax_iters > 1) s->relax_iters--; break;
                case SDLK_EQUALS: if (s->relax_iters < 30) s->relax_iters++; break;
                case SDLK_COMMA:  s->flow = fmaxf(0.02f, s->flow - 0.02f); break;
                case SDLK_PERIOD: s->flow = fminf(0.25f, s->flow + 0.02f); break;
                case SDLK_SPACE: paused = !paused; break;
                case SDLK_N: if (paused) sim_step(s); break;
                case SDLK_C: sim_clear(s); break;
                case SDLK_R:
                    sim_free(s); s = sim_create(GW, GH);
                    countdown_total_ms = -1; countdown_left_ms = 0;
                    break;
                case SDLK_T:
                    if      (countdown_total_ms < 0)      countdown_total_ms = 60000;
                    else if (countdown_total_ms == 60000) countdown_total_ms = 180000;
                    else                                  countdown_total_ms = -1;
                    countdown_left_ms = countdown_total_ms > 0 ? countdown_total_ms : 0;
                    s->spawn_enabled = 1;
                    break;
                case SDLK_E: {
                    float mx, my; SDL_GetMouseState(&mx, &my);
                    sim_add_impulse(s, (int)mx/CELL, (int)my/CELL,
                                    BOOM_RADIUS, BOOM_STRENGTH);
                    break;
                }
                case SDLK_V: render_mode = !render_mode; break;
                default: break;
                }
                break;
            default: break;
            }
        }

        if (!paused) { sim_step(s); tick++; }

        if (render_mode == 1 && !paused) particles_update(s);

        render(s, px, render_mode == 0);
        SDL_UpdateTexture(tex, NULL, px, GW * sizeof(Uint32));
        SDL_RenderClear(ren);
        SDL_RenderTexture(ren, tex, NULL, NULL);
        if (render_mode == 1) particles_render(ren, tick);
        SDL_RenderPresent(ren);

        char timer_str[40];
        if (countdown_total_ms < 0) {
            snprintf(timer_str, sizeof timer_str, "timer:OFF");
        } else {
            int secs = (countdown_left_ms + 999) / 1000;
            snprintf(timer_str, sizeof timer_str, "timer:%d:%02d%s",
                     secs/60, secs%60, s->spawn_enabled ? "" : " SPAWN-OFF");
        }
        char view_str[40];
        if (render_mode) snprintf(view_str, sizeof view_str, "PARTICLES(%d/%d)", n_alive, MAX_PARTICLES);
        else             snprintf(view_str, sizeof view_str, "FIELD");
        char title[256];
        snprintf(title, sizeof title,
                 "Horde fluid  |  brush:%s size:%d  flow:%.2f  relax_iters:%d  %s  %s  view:%s  drained:%.0f",
                 (const char*[]){"WALL","SPAWN","GOAL","DRAIN"}[brush], size,
                 s->flow, s->relax_iters, paused ? "PAUSED" : "running", timer_str,
                 view_str, s->drained_total);
        SDL_SetWindowTitle(win, title);
        SDL_Delay(8);
    }

    sim_free(s);
    SDL_DestroyTexture(tex);
    SDL_DestroyRenderer(ren);
    SDL_DestroyWindow(win);
    SDL_Quit();
    return 0;
}
