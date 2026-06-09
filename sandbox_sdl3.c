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
 *     - / =          horde cruise speed down / up        (p_goal_speed)
 *     , / .          spawn rate down / up                (spawn_per_step)
 *     Z / X          repulsion / packing down / up        (p_repel)
 *     O / P          collision radius down / up            (p_radius)
 *     9 / 0          friction down / up (sloshy<->settled)(p_friction)
 *     K / L          jam freezing down / up (dense piles lock)(p_jam_damp)
 *     SPACE          pause / resume
 *     N              single step (while paused)
 *     C              clear the horde (keep terrain)
 *     R              reset everything
 *     T              cycle survival timer (OFF -> 60s -> 180s -> OFF);
 *                    when it expires, all spawners stop emitting.
 *     E / MMB        explosion at cursor (radial velocity impulse)
 *     V              toggle render: zombie sprites <-> derived-density field
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

/* The horde is now the sim's PERSISTENT particle population (sim.c owns it):
 * one sprite per live particle, read straight from s->px/s->py/s->pseed. The
 * old density-tracking marker pool is gone — particles are born at spawners,
 * die at drains, and never pop in/out to chase a density field. */

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

/* Procedural stylized zombie sprite: an 8x8 top-down humanoid (head, splayed
 * arms, legs), generated in code so we depend on nothing but core SDL3 (no
 * SDL_image). The sheet is a grid of SPR_FRAMES (shuffle cycle) x SPR_SHADES
 * (tonal variants): picking a shade per particle breaks the "rubber-stamp" look
 * so a dense field reads as many individuals — all from one texture, so the
 * draws still batch. Placeholder art; swap for loaded art later. */
#define SPR_W      8
#define SPR_H      8
#define SPR_FRAMES 2
#define SPR_SHADES 3
#define SPR_COLS   (SPR_FRAMES * SPR_SHADES)
#define SHEET_W    (SPR_W * SPR_COLS)

static Uint32 spr_palette(char c, float k) {   /* k = per-shade brightness */
    int r, g, b;
    switch (c) {
        case 'K': r = 0x32; g = 0x52; b = 0x1E; break;   /* outline: DARK GREEN, not black
                                                            (near-black blackens dense piles) */
        case 'G': r = 0x5C; g = 0x8A; b = 0x38; break;   /* rotting-green body */
        case 'H': r = 0xA8; g = 0xCC; b = 0x68; break;   /* sickly pale head   */
        default:  return 0x00000000u;                    /* transparent        */
    }
    r = (int)(r * k); g = (int)(g * k); b = (int)(b * k);
    if (r > 255) r = 255;
    if (g > 255) g = 255;
    if (b > 255) b = 255;
    return 0xFF000000u | (r << 16) | (g << 8) | b;
}

/* Build the SHEET_W x SPR_H spritesheet as a STATIC texture. Column index is
 * shade*SPR_FRAMES + frame, so src.x = col*SPR_W picks one sub-image. */
static SDL_Texture *make_zombie_tex(SDL_Renderer *ren) {
    static const char *frames[SPR_FRAMES][SPR_H] = {
        {   /* frame 0: legs together, arms wide */
            "..KKKK..", ".KHHHHK.", ".KHHHHK.", "KKGGGGKK",
            ".KGGGGK.", ".KGGGGK.", ".KG..GK.", ".K....K.",
        },
        {   /* frame 1: arms tucked, feet shifted */
            "..KKKK..", ".KHHHHK.", ".KHHHHK.", ".KGGGGK.",
            "KKGGGGKK", ".KGGGGK.", ".KG..GK.", "..K..K..",
        },
    };
    static const float shade_k[SPR_SHADES] = { 0.85f, 1.0f, 1.15f };
    static Uint32 buf[SPR_H * SHEET_W];
    for (int sh = 0; sh < SPR_SHADES; sh++)
    for (int fr = 0; fr < SPR_FRAMES; fr++) {
        int colx = (sh * SPR_FRAMES + fr) * SPR_W;
        for (int y = 0; y < SPR_H; y++)
        for (int x = 0; x < SPR_W; x++)
            buf[y*SHEET_W + colx + x] = spr_palette(frames[fr][y][x], shade_k[sh]);
    }
    SDL_Texture *t = SDL_CreateTexture(ren, SDL_PIXELFORMAT_ARGB8888,
                                       SDL_TEXTUREACCESS_STATIC, SHEET_W, SPR_H);
    SDL_SetTextureBlendMode(t, SDL_BLENDMODE_BLEND);
    SDL_SetTextureScaleMode(t, SDL_SCALEMODE_NEAREST);
    SDL_UpdateTexture(t, NULL, buf, SHEET_W*(int)sizeof(Uint32));
    return t;
}

/* A render-only copy of a particle (so we can depth-sort without touching the
 * core's SoA arrays). */
#define RENDER_CAP 200000
typedef struct { float x, y; unsigned char seed; } RSprite;

static int cmp_rsprite_y(const void *a, const void *b) {
    float ya = ((const RSprite*)a)->y, yb = ((const RSprite*)b)->y;
    return (ya > yb) - (ya < yb);
}

static void particles_render(SDL_Renderer *ren, SDL_Texture *zt, Sim *s, int tick) {
    static RSprite buf[RENDER_CAP];
    int n = s->pcount < RENDER_CAP ? s->pcount : RENDER_CAP;
    for (int p = 0; p < n; p++) {
        buf[p].x = s->px[p]; buf[p].y = s->py[p]; buf[p].seed = s->pseed[p];
    }
    /* draw back-to-front so lower (nearer) zombies occlude those behind them */
    qsort(buf, n, sizeof(RSprite), cmp_rsprite_y);
    for (int p = 0; p < n; p++) {
        int frame = ((tick + buf[p].seed) >> 3) & 1;        /* swap every 8 ticks */
        int shade = buf[p].seed % SPR_SHADES;               /* stable per particle */
        float bob = frame ? 0.0f : -1.0f;
        SDL_FRect src = { (float)((shade * SPR_FRAMES + frame) * SPR_W),
                          0.0f, SPR_W, SPR_H };
        SDL_FRect dst = { buf[p].x * (float)CELL - SPR_W * 0.5f,
                          buf[p].y * (float)CELL - SPR_H * 0.5f + bob,
                          SPR_W, SPR_H };
        SDL_RenderTexture(ren, zt, &src, &dst);
    }
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

/* Headless-ish preview: seed a scene, run the sim in particle view, dump one
 * frame to shot.bmp. Reuses the real render path so it shows exactly what the
 * interactive view does. Invoke as `./sandbox shot`. */
static void run_shot(SDL_Renderer *ren, SDL_Texture *tex, SDL_Texture *ztex, Sim *s,
                     int siege, const char *outfile) {
    if (!siege) {
        /* spawner top-left, goal bottom-right, a wall bar so the horde has to
         * stream around it — exercises the flowing front, not just a pile. */
        paint(s, 55, 45, B_SPAWN, 9, 0);
        paint(s, 165, 175, B_GOAL, 7, 0);
        for (int y = 25; y <= 150; y++) {
            sim_set_wall(s, 110, y, WALL_H);
            sim_set_wall(s, 111, y, WALL_H);
        }
    } else {
        /* goal sealed inside a wall box; spawner outside. The horde must be
         * drawn to SIEGE the walls instead of ignoring the unreachable goal. */
        paint(s, 110, 30, B_SPAWN, 10, 0);
        paint(s, 110, 150, B_GOAL, 6, 0);
        int x0 = 90, x1 = 130, y0 = 130, y1 = 170;
        for (int x = x0; x <= x1; x++) { sim_set_wall(s, x, y0, WALL_H); sim_set_wall(s, x, y1, WALL_H); }
        for (int y = y0; y <= y1; y++) { sim_set_wall(s, x0, y, WALL_H); sim_set_wall(s, x1, y, WALL_H); }
    }
    s->spawn_enabled = 1;
    for (int i = 0; i < 1200; i++) sim_step(s);

    static Uint32 px[GW*GH];
    render(s, px, 0);
    SDL_UpdateTexture(tex, NULL, px, GW * sizeof(Uint32));
    SDL_RenderClear(ren);
    SDL_RenderTexture(ren, tex, NULL, NULL);
    particles_render(ren, ztex, s, 4);

    SDL_Surface *surf = SDL_RenderReadPixels(ren, NULL);  /* read before present */
    if (surf) {
        if (!SDL_SaveBMP(surf, outfile)) SDL_Log("SaveBMP: %s", SDL_GetError());
        else SDL_Log("wrote %s  (%d particles alive)", outfile, s->pcount);
        SDL_DestroySurface(surf);
    } else {
        SDL_Log("ReadPixels: %s", SDL_GetError());
    }
    SDL_RenderPresent(ren);
}

int main(int argc, char **argv) {
    if (!SDL_Init(SDL_INIT_VIDEO)) { SDL_Log("init: %s", SDL_GetError()); return 1; }
    SDL_Window   *win = SDL_CreateWindow("Horde fluid sandbox", WINW, WINH, 0);
    SDL_Renderer *ren = SDL_CreateRenderer(win, NULL);
    SDL_Texture  *tex = SDL_CreateTexture(ren, SDL_PIXELFORMAT_ARGB8888,
                                          SDL_TEXTUREACCESS_STREAMING, GW, GH);
    SDL_SetTextureScaleMode(tex, SDL_SCALEMODE_NEAREST);
    SDL_Texture  *ztex = make_zombie_tex(ren);

    Sim *s = sim_create(GW, GH);
    static Uint32 px[GW*GH];

    if (argc > 1 && (SDL_strcmp(argv[1], "shot") == 0 || SDL_strcmp(argv[1], "siege") == 0)) {
        int siege = SDL_strcmp(argv[1], "siege") == 0;
        run_shot(ren, tex, ztex, s, siege, siege ? "siege.bmp" : "shot.bmp");
        sim_free(s);
        SDL_DestroyTexture(ztex); SDL_DestroyTexture(tex);
        SDL_DestroyRenderer(ren); SDL_DestroyWindow(win); SDL_Quit();
        return 0;
    }

    int brush = B_WALL, size = 3, paused = 0, running = 1;
    int painting = 0, erasing = 0;
    int render_mode = 1;        /* 0 = derived-density field, 1 = zombie sprites */
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
                case SDLK_MINUS:  s->p_goal_speed = fmaxf(0.02f, s->p_goal_speed - 0.02f); break;
                case SDLK_EQUALS: s->p_goal_speed = fminf(0.80f, s->p_goal_speed + 0.02f); break;
                case SDLK_COMMA:  s->spawn_per_step = fmaxf(0.00f, s->spawn_per_step - 0.01f); break;
                case SDLK_PERIOD: s->spawn_per_step = fminf(1.00f, s->spawn_per_step + 0.01f); break;
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
                case SDLK_Z:  s->p_repel = fmaxf(0.05f, s->p_repel - 0.05f); break;
                case SDLK_X: s->p_repel = fminf(1.00f, s->p_repel + 0.05f); break;
                case SDLK_9: s->p_friction = fmaxf(0.00f, s->p_friction - 0.05f); break;
                case SDLK_0: s->p_friction = fminf(0.95f, s->p_friction + 0.05f); break;
                case SDLK_O: s->p_radius = fmaxf(0.20f, s->p_radius - 0.05f); break;
                case SDLK_P: s->p_radius = fminf(2.50f, s->p_radius + 0.05f); break;
                case SDLK_K: s->p_jam_damp = fmaxf(0.00f, s->p_jam_damp - 0.05f); break;
                case SDLK_L: s->p_jam_damp = fminf(0.98f, s->p_jam_damp + 0.05f); break;
                default: break;
                }
                break;
            default: break;
            }
        }

        if (!paused) { sim_step(s); tick++; }

        render(s, px, render_mode == 0);
        SDL_UpdateTexture(tex, NULL, px, GW * sizeof(Uint32));
        SDL_RenderClear(ren);
        SDL_RenderTexture(ren, tex, NULL, NULL);
        if (render_mode == 1) particles_render(ren, ztex, s, tick);
        SDL_RenderPresent(ren);

        char timer_str[40];
        if (countdown_total_ms < 0) {
            snprintf(timer_str, sizeof timer_str, "timer:OFF");
        } else {
            int secs = (countdown_left_ms + 999) / 1000;
            snprintf(timer_str, sizeof timer_str, "timer:%d:%02d%s",
                     secs/60, secs%60, s->spawn_enabled ? "" : " SPAWN-OFF");
        }
        char view_str[32];
        snprintf(view_str, sizeof view_str, render_mode ? "ZOMBIES" : "FIELD");
        char title[256];
        snprintf(title, sizeof title,
                 "Horde  |  brush:%s size:%d  speed:%.2f spawn:%.2f repel:%.2f frict:%.2f rad:%.2f jam:%.2f  "
                 "%s  %s  view:%s  pop:%d/%d  drained:%.0f",
                 (const char*[]){"WALL","SPAWN","GOAL","DRAIN"}[brush], size,
                 s->p_goal_speed, s->spawn_per_step, s->p_repel, s->p_friction, s->p_radius, s->p_jam_damp,
                 paused ? "PAUSED" : "running", timer_str,
                 view_str, s->pcount, s->spawn_max, s->drained_total);
        SDL_SetWindowTitle(win, title);
        SDL_Delay(8);
    }

    sim_free(s);
    SDL_DestroyTexture(ztex);
    SDL_DestroyTexture(tex);
    SDL_DestroyRenderer(ren);
    SDL_DestroyWindow(win);
    SDL_Quit();
    return 0;
}
