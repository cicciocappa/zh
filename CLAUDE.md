# Horde game — project memory

Progetto: gioco top-down 2D di tower-defense dove **l'orda di zombie è una popolazione
di particelle granulari persistenti** (tipo sabbia/SPH), non una folla di agenti con
pathfinding individuale né — più — un campo di densità. Riferimento creativo:
*"Sir, We Have an Orc Problem"* (TD 2D physics-based, ~100k nemici, corpi individuali
che si accatastano come una massa che inonda la mappa). Stack: **C + SDL3** (il core sim
è renderer-agnostico e pensato per portare su GPU/compute shader). Si lavora dal terminale.

Conversazioni in **italiano**. Codice e commenti in inglese.

> **Nota storica importante.** Il progetto è nato come "orda = fluido granulare" (campo di
> densità `rho` su griglia, shallow-water). Quel modello è stato **ritirato** (commit
> `149419f`/`822bdaf` se serve un confronto) perché derivare gli sprite dalla densità
> istantanea li faceva apparire/sparire invece di seguire traiettorie. Vedi il pivot
> all'Opzione B (particelle granulari) nella memoria di sessione. Il **grid-fluido non è
> stato buttato**: è diventato l'infrastruttura (campo-obiettivo + spatial-hash + densità
> derivata).

## Stato attuale (verificato)

Core a particelle e sandbox SDL3 girano entrambi. Verificato headless (200×200) — invarianti
particellari OK: `in_wall=0 oob=0 nan=0`, nessuna fuga dai muri, niente NaN. Verificato a
vista con `./sandbox shot` (flusso a fiume che si accumula e aggira un muro) e
`./sandbox siege` (goal sigillato dai muri → l'orda assedia le pareti).

Pezzi presenti:

- **Orda = particelle** (SoA in `Sim`): nascono SOLO agli spawner, muoiono SOLO ai drain,
  per il resto **conservate**. ~18-23k in flusso tipico; cap `pcap=200000`.
- **Forza-obiettivo**: potential field `phi` (Dijkstra 8-vicini). I muri sono
  **attraversabili a costo enorme** (`WALL_ENTER=5000`, ≫ qualunque giro sulla mappa): dove
  c'è un varco vince il giro, dove il goal è sigillato il gradiente punta comunque verso di
  esso → **assedio** (le particelle premono sui muri; con HP futuri li sfonderanno).
- **Impaccamento granulare**: repulsione PBD a corto raggio (Jacobi double-buffer), niente
  più pressione shallow-water. La velocità è ricavata dallo **spostamento reale**
  (`pos_finale − pos_iniziale`) → contropressione, niente sovra-compressione.
- **Campo di velocità** (`vx,vy`): layer di impulsi additivo per le esplosioni; le particelle
  lo campionano. `sim_add_impulse(cx,cy,r,strength)` dà un kick radiale (tasto `E`/MMB).
- **`rho` DERIVATA**: binning del conteggio particelle per cella. Serve al gameplay
  (mira torrette, economia) e alla vista-campo di debug (`V`).
- **Survival timer** (`s->spawn_enabled`): disattiva tutti gli spawner allo scadere
  (sandbox: tasto `T`, OFF → 60s → 180s).
- **Sprite zombie procedurale**: spritesheet 8×8 generato in codice (2 frame shuffle × 3
  sfumature, una sola texture → draw batchati), 1 sprite per particella, depth-sort per y,
  bobbing. *Ancora placeholder* (omino verde stilizzato), asset veri da fare in seguito.

## File

- `sim.h` / `sim.c` — **core della simulazione a particelle**, C puro, zero dipendenze. È il
  pezzo che porterà su GPU (particelle + grid → entrambi compute-shader-friendly).
- `test_dump.c` — verifica headless (no SDL): scena → run → invarianti + frame PPM in `frames/`
  (la cartella `frames/` deve esistere, non è versionata).
- `sandbox_sdl3.c` — sandbox interattivo SDL3 + modalità screenshot `shot`/`siege`.
- `Makefile` — `make test` (no deps) · `make sandbox` (richiede SDL3) · `make`.
- `README.md` — **probabilmente stale** (descriveva il vecchio modello fluido); da aggiornare.

## Build

```sh
make test      # headless, nessuna dipendenza
make sandbox   # interattivo, richiede SDL3 (pkg-config --libs sdl3)
```

Su questa macchina (Linux Mint 22.3 = Ubuntu noble) SDL3 NON è nei repo. È stato compilato
dai sorgenti (libsdl-org/SDL, **branch `release-3.2.x`** che riporta v3.2.31 — il tag
`v3.2.31` NON esiste, l'ultimo tag rilasciato è `release-3.2.30`) e installato in `~/.local`
senza sudo. Per ricompilare il sandbox:

```sh
export PKG_CONFIG_PATH=$HOME/.local/lib/pkgconfig
make sandbox
```

L'rpath verso `~/.local/lib` è già nel binario, quindi `./sandbox` basta senza
`LD_LIBRARY_PATH`. Sorgenti SDL3 in `~/src/SDL3`. Anteprime: `./sandbox shot` (o `siege`)
salva un BMP, poi `convert shot.bmp shot.png`.

## Il modello (riferimento)

L'orda è una popolazione di particelle. Per ogni `sim_step`:

```
phi (se dirty) → particles_spawn → particles_hash → particles_step
                 → particles_drain → damp_velocity → particles_rasterize
```

- **`particles_spawn`** — ogni cella `CELL_SPAWN` emette `spawn_per_step` particelle/step
  (accumulo frazionario), se `spawn_enabled`.
- **`particles_hash`** — spatial hash sulle celle via counting sort (`cell_start`/`pidx`):
  vicini in O(1). **Ricostruito a OGNI iterazione di collisione** (non una volta sola): senza,
  i grani che si spingono escono dalle liste di vicini stantie e la densità non converge al
  limite di packing → sovra-compressione. Costo O(N)×iters, trascurabile.
- **`particles_step`** — il cuore granulare. Per ogni particella: (1) sterza lungo `-∇phi`
  a una `p_goal_speed` di crociera, conservando momento (`p_friction`), + campo impulsi
  (`vx,vy`) + jitter, clamp; (2) mossa tentativa, scivolando sull'asse bloccato dai muri;
  (3) **resolve PBD**: repulsione a corto raggio (`p_radius`, `p_repel`, `p_collide_iters`
  sweep Jacobi) che impacca/sparge e tiene tutti fuori dai muri. Velocità finale = spostamento
  netto (contropressione).
- **`particles_drain`** — una particella su cella `CELL_DRAIN` viene consumata a `drain_rate`
  (swap-remove), accumula `drained_total`.
- **`particles_rasterize`** — `rho[cella] += 1` per particella (densità derivata).

`goal_dir()` campiona `-∇phi`; **neutralizza i vicini-muro** (e `INF`) clampandoli al valore
della cella, così la pendenza viene dai vicini liberi e non c'è una spinta spuria "scappa dal
muro" (il flusso aperto resta corretto, l'assedio nasce dal pendio verso il goal).

Scelte deliberate: **niente solve di incomprimibilità (Poisson)** — l'impaccamento è
posizionale (PBD), comprimibile fino al packing, stabile. Cugino di un solver granulare
position-based guidato da un campo-obiettivo condiviso (à la Continuum Crowds per il phi).

### Tunable principali (in `Sim`, regolabili a caldo nel sandbox)

- `p_goal_speed` (crociera, tasti `-`/`=`) · `spawn_per_step` (`,`/`.`) ·
  `p_repel` (impaccamento/larghezza onda, `;`/`'`) · `p_friction` (sloshy↔assestato, `9`/`0`).
- `p_radius`, `p_collide_iters`, `p_jitter`: default ragionevoli in `sim_create`, da tarare in
  codice. **Attenzione**: tenere `2·p_radius ≤ 1` e velocità `< 1` cella/step, altrimenti i
  vicini escono dal blocco 3×3 dell'hash (`PMAX=0.85` in `particles_step`).

## Convenzioni

- Indicizzazione `i = y*W + x`; bordo sigillato con `bed` enorme (i muri sono `bed>0`).
- Tutto pensato **data-parallel / compute-shader-friendly**: hash via counting sort, collisioni
  Jacobi **double-buffered** (`px`↔`px2`, mai Gauss-Seidel in-place), nessuna dipendenza
  seriale tra particelle dentro uno sweep.
- Il core SIM resta **senza alcuna dipendenza da SDL/OpenGL**. Il sandbox è solo rendering:
  legge `s->px/s->py/s->pseed` e disegna; per il depth-sort copia in un buffer locale.

## Decisioni aperte / agenda

Fatto: **pivot a particelle granulari persistenti ✓** (conservazione, traiettorie continue,
assedio dei muri, contropressione). **Sprite procedurale ✓** ma placeholder.

Prossimi candidati:

1. **Scaling a 100k → port GPU/compute.** Su CPU si valida a ~20-50k; per i 100k del
   riferimento serve spostare hash + collide + integrazione su compute shader (la pipeline è
   già scelta apposta GPU-friendly).
2. **Torrette (primo gameplay).** I drain già consumano particelle e contano `drained_total`;
   renderli "attivi": una torretta è un drain che spara in base alle particelle/`rho` adiacenti,
   ha HP, costa biomassa. Definire l'economia (drained → currency → build).
3. **Muri con HP, costruibili/scavabili.** `sim_set_wall` c'è (bed = altezza/HP); l'assedio già
   preme contro i muri — manca far cedere il muro sotto pressione + UI/bilanciamento.
4. **Asset sprite veri** al posto dell'omino verde (eventuale orientamento da `-∇phi`,
   cadaveri/sangue persistenti come nel riferimento).

Più avanti (predisposti): fuoco = campo `temperature` + propagazione; attrattori per
rumore/luce (fase offensiva/stealth).
