# TODO — Horde game

Tracker dei progressi. Modello e decisioni: vedi `CLAUDE.md`.

Legenda: `[ ]` da fare · `[~]` in corso · `[x]` fatto

---

## ✅ Fatto

### Core fluido originale (poi declassato a infrastruttura)
- [x] Goal forcing (Dijkstra `phi`) + surface pressure shallow-water
- [x] Campo di velocità (`vx,vy`) + esplosioni `sim_add_impulse`
- [x] Sandbox SDL3 (pennelli, manopole live, survival timer `T`)

### Pivot a ORDA = PARTICELLE GRANULARI PERSISTENTI (Opzione B)
Rif. creativo: *Sir, We Have an Orc Problem*. Le particelle nascono solo agli spawner,
muoiono solo ai drain, mai pop-in/out. Il grid resta come infrastruttura.
- [x] Popolazione particellare SoA in `sim.c` (`particles_spawn/hash/step/drain/rasterize`)
- [x] Spatial hash counting-sort (ricostruito a ogni iter di collisione)
- [x] Repulsione PBD Jacobi double-buffer + collisione muri
- [x] `phi` con muri attraversabili a costo enorme → goal sigillato = **assedio**, non ignorato
- [x] Velocità = spostamento reale (contropressione)
- [x] **Pressione su griglia** (`-∇` eccesso densità) → mette un tetto alla densità *anche all'interno*
- [x] Jam damping density-based (smorza sterzata/jitter/velocità dove denso) — *vedi problema aperto*
- [x] **Cap allo spawn** (`spawn_max`, default 10k) — niente sink ⇒ serve un tetto
- [x] `rho` derivata dal binning (per gameplay/vista debug)
- [x] Sprite zombie procedurale 8×8 (2 frame × 3 sfumature, una texture batchata, depth-sort)
- [x] Contorno verde-scuro (non near-black) → masse fitte restano verdi, non anneriscono
- [x] Knob live: `-/=` speed · `,/.` spawn · `Z/X` repel · `9/0` friction · `O/P` raggio · `K/L` jam
- [x] Modalità anteprima `./sandbox shot` e `./sandbox siege`
- [x] Test headless aggiornato: invarianti (no NaN, no fuori-griglia, no in-muro) + istogramma densità

---

## ⚠️ PROBLEMA APERTO — riprendere da qui

**Il centro denso di una pila trema; i bordi si assestano.** Persiste anche a massa ferma
(cap raggiunto, nessun afflusso) e **anche alzando `jam` (`L`) non cambia nulla**.

Diagnosi/pista: lo smorzamento jam agisce sullo stage delle forze (sterzata/jitter/velocità),
ma il **resolve di collisione (stage 3 di `particles_step`) muove le posizioni DIRETTAMENTE**
(`px2 = ax + pushx`), scavalcando la velocità. Quindi se il tremolio è l'**oscillazione
Jacobi delle collisioni** al centro (tanti vicini sovrapposti, spinte sommate, `repel`,
4 iter con re-hash) → le posizioni ping-pongano e niente di ciò che ho toccato lo intercetta.
Il fatto che `jam` non abbia avuto effetto **conferma** che la causa è a livello posizione, non velocità.

Leve da provare:
- [ ] Sotto-rilassare/smorzare la **spinta di collisione** in funzione della densità (o globalmente)
- [ ] Rilevamento convergenza / clamp dello spostamento netto per-step nelle zone dense
- [ ] Valutare se la pressione (1-step lag, Jacobi) contribuisce all'oscillazione → eventuale sotto-rilassamento
- [ ] Ricordare: `repel > 1` = sovra-rilassamento = rimbalzo (tenerlo ≤ 1; per rigidità usare più iterazioni)

---

## 🎯 Prossimo gameplay
- [ ] **Torrette**: drain "attivi" che sparano in base alle particelle adiacenti; HP; costo biomassa; economia (drained → currency → build)
- [ ] **Muri con HP**: l'assedio già preme contro i muri — farli cedere sotto pressione + UI/bilanciamento
- [ ] **Asset sprite veri** al posto dell'omino verde (cadaveri/sangue persistenti come nel riferimento)

---

## 🛠️ Tecnico / infra
- [ ] **Scaling a 100k → port GPU/compute** (hash + collide + integrazione su compute shader; pipeline già scelta GPU-friendly). Su CPU si valida a ~10–50k.
- [ ] Aggiornare `README.md` (ancora sul vecchio modello fluido)
- [ ] Pacchettizzare/distribuire SDL3 (oggi compilato a mano in `~/.local`)
