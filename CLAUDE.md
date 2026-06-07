# Horde fluid game — project memory

Progetto: gioco top-down 2D di zombie dove **l'orda è un fluido granulare simulato**
(campo di densità su griglia), non una folla di agenti con pathfinding. Riferimento
creativo: *Creeper World*. Stack: **C + SDL3** (il core sim è renderer-agnostico e
porta su OpenGL/compute shader). Si lavora dal terminale per compilare e testare.

Conversazioni in **italiano**. Codice e commenti in inglese.

## Stato attuale (POC verificato)

Il core della simulazione e il sandbox SDL3 girano entrambi. Tutti i pezzi sono stati
verificati a mano nel sandbox e via regression headless (200×200, 900 step → 
`total_mass=116534.4 drained=171.6 bad_cells=0`, conservativo, niente NaN).

Pezzi presenti, oltre al fluid core:

- **Survival timer** (master switch `s->spawn_enabled`): il game layer disattiva tutti
  gli spawner allo scadere di un countdown. Nel sandbox è il tasto `T` (OFF → 60s → 180s).
- **Campo di velocità** (`vx, vy` collocated, additivo): impulse layer per esplosioni.
  Advezione upwind con clamp CFL (`v_max=0.9`) e bounce semi-elastico sui muri; non
  self-advect. Friction multiplicativa (`v_damp=0.95`). Goal forcing resta com'era
  (non vogliamo inerzia "verso il goal" che rovini il curvare attorno ai muri).
- **Esplosioni**: `sim_add_impulse(cx,cy,r,strength)` — carve radiale di `rho`
  (fino a −90% al centro) + kick radiale di velocità. Nel sandbox tasto `E` o MMB.
- **Particle view**: rendering alternativo nel sandbox (tasto `V`). Pool persistente
  di 10k particelle; target per cella = `rho × 0.30` con soglia di visibilità
  `rho ≥ 0.5`. Le particelle si muovono col campo `v` reale + bias verso `-∇phi`
  + jitter, con bobbing 2-frame. *Solo placeholder visivo* (quadratini 3×3 rossi):
  validato il flusso campo→sprite, asset veri da fare.

## File

- `sim.h` / `sim.c` — **core della simulazione**, C puro, zero dipendenze. È il pezzo
  che porterà nel motore OpenGL (caricando `rho` come texture invece di disegnare con SDL).
- `test_dump.c` — verifica headless (no SDL): scena → run → frame PPM in `frames/`.
- `sandbox_sdl3.c` — sandbox interattivo SDL3 (pennelli muro/spawner/goal/drain, manopole
  live, survival timer `T`, esplosioni `E`/MMB, particle view `V`).
- `Makefile` — `make test` (no deps) · `make sandbox` (richiede SDL3) · `make`.
- `README.md` — descrizione estesa del modello e dei controlli.

## Build

```sh
make test      # headless, nessuna dipendenza
make sandbox   # interattivo, richiede SDL3 (pkg-config --libs sdl3)
```

Su questa macchina (Linux Mint 22.3 = Ubuntu noble) SDL3 NON è nei repo. È stato
compilato dai sorgenti (libsdl-org/SDL, branch `release-3.2.x` → v3.2.31) e
installato in `~/.local` senza sudo. Per ricompilare il sandbox:

```sh
export PKG_CONFIG_PATH=$HOME/.local/lib/pkgconfig
make sandbox
```

L'rpath verso `~/.local/lib` è già incorporato nel binario, quindi `./sandbox`
basta senza `LD_LIBRARY_PATH`. Sorgenti SDL3 in `~/src/SDL3`.

## Il modello (riferimento)

L'orda è un campo di densità/profondità `rho` su griglia. Due forze:

1. **Goal forcing** — potential field `phi` (Dijkstra 8-vicini, costo-verso-obiettivo,
   muri impassabili). La densità è avvettata lungo `-grad(phi)`: "gravità verso gli
   obiettivi". Ricalcolato solo quando terreno/goal cambiano (`phi_dirty`).
2. **Surface pressure** — equalizzazione shallow-water su `bed` (quota fondo; muri = bed
   alto). L'orda preme contro i muri e, quando `bed+rho` supera la cima, sfiora oltre (weir).

Scelte deliberate: **niente momento, niente solve di incomprimibilità (Poisson)**. Il
mezzo è comprimibile fino all'impaccamento (si impila) e la forma a rilassamento è
incondizionatamente stabile (nessun vincolo CFL). È un cugino semplificato di Continuum
Crowds (Treuille 2006) + shallow-water.

**Manopola chiave:** `relax_iters` = dial granulare↔acquoso. Poche iterazioni → pile
ripide / alto angolo di riposo (sabbioso); molte → auto-livellamento (acquoso).

`sim_step()` = `apply_sources_drains` → `advect_toward_goal` → `advect_by_velocity`
→ `damp_velocity` → `equalize_surface`.

I due step centrali (`advect_by_velocity` + `damp_velocity`) sono il layer di velocità
descritto sopra: con `v=0` ovunque sono no-op, quindi il modello "puro" resta
recuperabile semplicemente non iniettando impulsi.

## Convenzioni

- Buffer per-cella `s->d` per i delta → aggiornamenti conservativi e order-independent.
- Indicizzazione `i = y*W + x`; bordo sigillato con `bed` enorme.
- Tutto data-parallel per-cella → pensare già "compute-shader-friendly" (no dipendenze
  seriali tra celle dentro uno sweep; usare Jacobi/double-buffer, non Gauss-Seidel in-place).
- Mantenere il core SIM senza alcuna dipendenza da SDL/OpenGL.

## Decisioni aperte / agenda

Cose fatte: **campo di velocità ✓** (additivo, no self-advect, no sloshing nativo —
si può aggiungere generando velocità dall'equalize_surface se servirà);
**particle view ✓** ma come placeholder (vedi sotto, va completata).

Prossimi candidati naturali:

1. **Sprite veri al posto dei quadratini.** Il flusso campo→sprite è validato; manca:
   (a) procurare/disegnare uno spritesheet zombie top-down (walk cycle, magari 4 direzioni
   o solo bobbing top-down); (b) sostituire `SDL_RenderFillRects` con `SDL_RenderTexture`
   per ogni sprite, eventualmente con orientamento dedotto da `-∇phi`; (c) se il count
   supera ~10-20k pensare al backend `SDL_GPU` con instancing. Decisione presa nel POC:
   visualizziamo *tutta* l'orda come sprite (non solo la frontiera), `sprites_per_unit=0.3`.
2. **Primo elemento di gameplay — torrette.** I drain già consumano massa e contano
   `drained_total`; il passo successivo è renderli "attivi": una torretta è un drain
   che spara in base al `rho` adiacente, ha HP, costa biomassa accumulata. Definire il
   ciclo economia (mass drained → currency → build).
3. **Muri costruibili / scavabili** dal player (bed = altezza/HP). API già pronta
   (`sim_set_wall`), manca solo il layer di UI e bilanciamento.

Game layer più futuri (predisposti nel data model): muri con HP che cedono;
fuoco = campo `temperature` + propagazione cellulare; attrattori per rumore/luce
(fase offensiva/stealth). Sloshing autentico (equalize_surface che genera `v` invece
di muovere `rho` direttamente) è il "upgrade fisico" più grande ancora da fare —
costoso in CFL ma il look sarebbe spettacolare.
