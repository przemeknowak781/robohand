# RoboHand — parametryczny symulator dłoni robota napędzanej ścięgnami

Aplikacja 3D (przeglądarka, Three.js) do parametrycznego symulowania manipulatora —
dłoni robota humanoidalnego o architekturze ścięgnowej (tendon-driven), z pełną
telemetrią naprężeń w ścięgnach i interfejsem przygotowanym pod integrację z ML/RL.

![RoboHand](docs/screenshot.png)

## Szybki start

```bash
npm install
npm run dev            # aplikacja 3D na http://localhost:5173
npm run demo:headless  # rdzeń fizyki w Node, bez przeglądarki (ścieżka ML)
npm run build          # bundel produkcyjny do dist/
npm run typecheck      # kontrola typów
```

## Architektura

```
src/
  sim/            RDZEŃ FIZYKI — czysty TypeScript, ZERO zależności od DOM/Three.js
    types.ts        typy parametrów, obserwacji i specyfikacji ML
    math3.ts        minimalna algebra 3D (mat3/vec3)
    defaultHand.ts  parametryczny builder dłoni (proporcje ludzkie, 14 ścięgien)
    simulator.ts    kinematyka + dynamika + model ścięgien, step API
    presets.ts      mapowanie "curl" palców → akcje ścięgien, gotowe pozy
    recorder.ts     rejestrator datasetów JSONL
  viz/            wizualizacja Three.js (scena, dłoń, HUD, wykres naprężeń)
  ui/             panel sterowania (lil-gui), pobieranie plików
  main.ts         pętla stałego kroku czasowego, spięcie całości
scripts/
  headless-demo.ts  przykład użycia rdzenia w Node (pętla jak w środowisku RL)
```

Rozdział `sim/` od `viz/` jest celowy: **ten sam kod fizyki** działa w przeglądarce
i headless w Node — środowisko treningowe RL importuje `HandSimulator` bezpośrednio,
bez przeglądarki i bez renderowania.

## Model fizyczny

Każdy palec to szeregowy łańcuch przegubów obrotowych (zgięciowych):
kciuk CMC–MP–IP, pozostałe palce MCP–PIP–DIP (15 przegubów łącznie).

Ścięgna modelowane są jako elastyczne cięgna prowadzone przez bloczki (pulleys)
o stałych ramionach momentu, napinane przez aktuatory liniowe (silnik + szpula):

| Wielkość | Wzór |
|---|---|
| wycieczka ścięgna | `e(q) = Σⱼ signⱼ · rⱼ · qⱼ` |
| rozciągnięcie | `s = pretension + xₐ − e(q)` |
| naprężenie | `T = max(0, k·s + c·ṡ)` — ścięgno nie pcha |
| moment w stawie | `τⱼ += signⱼ · rⱼ · T` |
| aktuator | `ẋₐ = (u·maxPull − xₐ)/τ_act` — człon inercyjny I rzędu |

Do tego: pasywna sztywność i tłumienie torebki stawowej, miękkie limity stawów,
momenty grawitacyjne liczone z kinematyki prostej (masy paliczków), efektywna
bezwładność stawu z mas dystalnych. Całkowanie: semi-implicit Euler ze stałym
krokiem (domyślnie fizyka 480 Hz, krok sterowania 60 Hz) — **w pełni
deterministyczne** (ten sam ciąg akcji ⇒ identyczna trajektoria, co potwierdza
zgodność wyników headless i w przeglądarce co do bitów).

### Układ ścięgien (antropomorficzny)

- każdy palec: **FDP** (zginacz głęboki, MCP+PIP+DIP), **FDS** (zginacz
  powierzchowny, MCP+PIP), **EDC** (prostownik, grzbietowo przez wszystkie stawy),
- kciuk: **FPL** + **EPL**,
- razem **14 ścięgien = 14-wymiarowa przestrzeń akcji**.

Prostowniki mają napięcie wstępne (pretension) i działają jak sprężyny powrotne —
sprzężenie zgięcia stawów wynika naturalnie ze wspólnego prowadzenia ścięgna,
tak jak w rzeczywistych dłoniach ścięgnowych.

## Parametry (panel „RoboHand — controls")

- **Pose presets** — Open / Fist / Pinch / Point / OK sign,
- **Finger curl** — zadawanie zgięcia per palec (mapowane na antagonistyczne pary ścięgien),
- **Tendon actuators (advanced)** — bezpośrednie zadawanie 14 aktuatorów (0–1),
- **Hand geometry** — skala globalna, długość/grubość palców, promienie bloczków,
- **Tendon physics** — sztywności zginaczy/prostowników, tłumienie, pretension, próg bezpiecznego naprężenia,
- **Joints & actuators** — sztywność/tłumienie stawów, stała czasowa i skok aktuatorów,
- **Environment** — grawitacja (włącznik + wartość g),
- **Simulation** — pauza, skala czasu, liczba podkroków fizyki, reset pozy,
- **Data / ML** — nagrywanie datasetu, pobranie `.jsonl` i specyfikacji `.json`.

Zmiany parametrów przebudowują model **na żywo z zachowaniem stanu** (o ile
topologia stawów/ścięgien się nie zmienia).

## Pomiary naprężeń

- HUD (lewy górny róg): naprężenie każdego ścięgna w N + pasek względem progu
  bezpieczeństwa (czerwony kolor przy przeciążeniu),
- ścięgna w 3D kolorowane naprężeniem: niebieski → żółty → czerwony,
- dolny panel: przebieg czasowy naprężeń wszystkich ścięgien
  (prostowniki linią przerywaną).

## Integracja z ML

### API symulatora (przeglądarka lub Node)

```ts
import { buildHandParams } from './src/sim/defaultHand';
import { HandSimulator } from './src/sim/simulator';

const sim = new HandSimulator(buildHandParams());
const spec = sim.getSpec();       // pełna specyfikacja przestrzeni akcji/obserwacji
let obs = sim.reset();
obs = sim.step(action);           // action: number[14] w [0,1] — cele aktuatorów
// obs: { time, q[15], qd[15], tendonTension[14], tendonExcursion[14],
//        tendonStretch[14], actuatorPosition[14], actuatorTarget[14],
//        fingertipPositions[15] }
const flat = HandSimulator.flattenObservation(obs); // wektor dla sieci
```

W przeglądarce symulator jest dostępny z konsoli / zewnętrznego harnessa jako
`window.robohand` (m.in. `sim`, `recorder`, `setCurls({...})`).

### Specyfikacja (przycisk „download sim spec")

`SimSpec` zawiera: `actionSpace` (box [0,1]¹⁴ z nazwami ścięgien),
`observationSpace` (pola + etykiety każdego wymiaru), kroki czasowe oraz
**pełne drzewo parametrów dłoni** — dataset jest samoopisujący się i
odtwarzalny.

### Format datasetu (JSONL)

```
{"type":"spec","spec":{...}}                        ← nagłówek
{"type":"step","t":0.0167,"action":[...],"obs":{...}}
{"type":"step","t":0.0333,"action":[...],"obs":{...}}
```

## Mapa rozwoju

- kontakt i chwytanie obiektów (kolizje palce–obiekt, siły chwytu),
- abdukcja/addukcja MCP i kciuka (dodatkowe DOF + ścięgna międzykostne),
- mostek WebSocket/Python (gymnasium env) nad rdzeniem `sim/`,
- tarcie ścięgno–bloczek (model Capstan), elastyczność nieliniowa,
- eksport/import konfiguracji dłoni jako JSON.
