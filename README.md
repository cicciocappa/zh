# Horde fluid — proof of concept

POC del modello di simulazione: **l'orda come fluido granulare**, non come folla di
agenti. È il sandbox da cui far crescere il gioco.

## File

| File | Cosa |
|---|---|
| `sim.h` / `sim.c` | **Core della simulazione**, C puro, zero dipendenze. È la parte che porti nella tua pipeline C/OpenGL: invece di disegnare con SDL, carichi `rho` come texture. |
| `test_dump.c` | Verifica headless (niente SDL): monta una scena, gira, scrive frame PPM in `frames/`. Serve a confermare stabilità e comportamento. |
| `sandbox_sdl3.c` | Sandbox **interattivo** (SDL3): dipingi terreno, lasci cadere spawner/goal/drain, giri le manopole dal vivo. |

## Build

```sh
make test       # solo headless, nessuna dipendenza
make sandbox    # interattivo, richiede SDL3 (pkg-config --libs sdl3)
make            # entrambi
```

Il `test` non visualizza: per vedere i PPM convertili (es. `ffmpeg -i frames/f%04d.ppm out.gif`)
o aprili con un viewer. Il `sandbox` apre una finestra.

## Controlli del sandbox

- **LMB** dipingi · **RMB** cancella
- **1** muro · **2** spawner · **3** goal · **4** drain
- **[ ]** dimensione pennello · **- =** iterazioni di equalizzazione · **, .** flow rate
- **SPACE** pausa · **N** step singolo · **C** azzera densità · **R** reset · **ESC** esci

## Il modello in breve

L'orda è un **campo di densità/profondità** su griglia. Due forze la muovono:

1. **Goal forcing** — un campo di potenziale `phi` (costo-verso-obiettivo, calcolato con
   Dijkstra). La densità viene avvettata lungo `-grad(phi)`. È la "gravità verso gli
   obiettivi": dove l'orda *vuole* andare. I muri sono impassabili per `phi`, quindi il
   flusso aggira gli ostacoli e si infila nei varchi.
2. **Surface pressure** — equalizzazione shallow-water su un `bed` (quota del fondo). I
   muri sono semplici celle con `bed` alto: l'orda ci si accumula contro e, quando la sua
   superficie (`bed + rho`) supera la cima del muro, **trabocca** (sfioro/weir). È il
   *come* l'orda si impila / si spande / esonda.

Niente momento e niente solve di incomprimibilità (Poisson): il mezzo dev'essere
**comprimibile** fino a un limite di impaccamento (si impila), e la forma a rilassamento è
incondizionatamente stabile — ciò che serve in un game loop. È un cugino volutamente
semplificato di *Continuum Crowds* (Treuille 2006) e dei modelli shallow-water.

### La manopola che conta

`relax_iters` (iterazioni di equalizzazione per step) è il tuo **dial di "granularità"**:
- **poche** → la densità non si livella → pile ripide, alto angolo di riposo → *sabbioso*
- **molte** → si auto-livella → *acquoso*

## Comportamenti verificati

Headless, 900 step, scena con spawner/goal/muro-con-varco/box-sigillato/drain:
- stabile (nessuna cella NaN o esplosa), profondità limitata, massa conservata salvo sorgenti/drain
- flusso diretto verso l'obiettivo ✓
- congestione al collo di bottiglia ✓
- traboccamento oltre i muri ✓
- consumo di massa ai drain ✓

## Dove si aggancia il gioco (prossimi strati)

- **Drain = kill-zone**: torrette/trappole rimuovono `rho` e accumulano `drained_total` → biomassa/risorse.
- **Muri costruibili**: `bed` è già l'altezza del muro → "HP/altezza" e overflow gratis.
- **Esplosioni/mortaio**: sottrai `rho` in un raggio (+ impulso, quando aggiungeremo la velocità).
- **Fuoco**: campo `temperature` + propagazione cellulare sull'impaccamento denso.
- **Attrattori (fase offensiva/stealth)**: aggiungi sorgenti di potenziale per rumore/luce; il
  giocatore diventa un attrattore mobile, lo stealth = gestire la propria impronta in `phi`.
- **Sprite di frontiera**: dove `rho` supera una soglia adiacente a una difesa, spawni agenti
  sprite discreti (instanced) per il combattimento leggibile, mentre il grosso resta campo.

## Note tecniche

- Risoluzione/passo: lo schema sposta solo una *frazione* di densità per cella, quindi è
  stabile a prescindere dal `dt`. Per la versione di gioco: timestep fisso, render disaccoppiato.
- Scaling: `sim_step` è interamente data-parallel per-cella → si porta su **compute shader**
  (ping-pong di texture per `rho`/`bed`, Dijkstra ricalcolato solo on-edit lato CPU o con un
  eikonal solver su GPU). Il loop CPU qui regge comodamente griglie ~256² in tempo reale.
