# TODO — Horde fluid game

Tracker dei progressi. Riferimento al modello e alle decisioni: vedi `CLAUDE.md`.

Legenda: `[ ]` da fare · `[~]` in corso · `[x]` fatto

---

## ✅ Fatto (POC verificato)

- [x] Fluid core (`sim.h`/`sim.c`): goal forcing (Dijkstra `phi`) + surface pressure (shallow-water)
- [x] Regression headless (200×200, 900 step): conservativo, niente NaN, `bad_cells=0`
- [x] Sandbox SDL3 interattivo (pennelli muro/spawner/goal/drain, manopole live)
- [x] Survival timer (`spawn_enabled`, tasto `T`: OFF → 60s → 180s)
- [x] Campo di velocità (`vx,vy` collocated, additivo, no self-advect) + clamp CFL + friction
- [x] Esplosioni `sim_add_impulse()` (carve `rho` + kick radiale; tasto `E` / MMB)
- [x] Particle view placeholder (tasto `V`, pool 10k, `sprites_per_unit=0.3`)

---

## 🎯 Prossimi candidati (dall'agenda)

### 1. Sprite veri al posto dei quadratini
- [ ] Procurare/disegnare spritesheet zombie top-down (walk cycle; 4 direzioni o solo bobbing)
- [ ] Sostituire `SDL_RenderFillRects` con `SDL_RenderTexture` per sprite
- [ ] Orientamento sprite dedotto da `-∇phi`
- [ ] Se count > ~10–20k: valutare backend `SDL_GPU` con instancing

### 2. Primo gameplay — torrette
- [ ] Rendere i drain "attivi": sparano in base al `rho` adiacente
- [ ] HP delle torrette + costo in biomassa
- [ ] Ciclo economia: mass drained → currency → build

### 3. Muri costruibili / scavabili dal player
- [ ] Layer di UI sopra `sim_set_wall` (già pronto)
- [ ] Bilanciamento bed = altezza/HP

---

## 🔮 Futuri (predisposti nel data model)

- [ ] Muri con HP che cedono
- [ ] Fuoco: campo `temperature` + propagazione cellulare
- [ ] Attrattori per rumore/luce (fase offensiva/stealth)
- [ ] **Sloshing autentico**: `equalize_surface` genera `v` invece di muovere `rho` direttamente
      (upgrade fisico più grande; costoso in CFL ma look spettacolare)

---

## 🛠️ Tecnico / infra

- [ ] Port del core su OpenGL/compute shader (caricare `rho` come texture)
- [ ] Pacchettizzare/distribuire SDL3 (oggi compilato a mano in `~/.local`)
