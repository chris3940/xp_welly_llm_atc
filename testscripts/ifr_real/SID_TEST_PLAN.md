# SID climb — in-sim test plan (beta-56)

Covers the two changes that have **never been flown**: the lateral sector handoff
(Phase 2.8, decoupled from the departure hold) and the SID climb **floor** now read
from the earliest constrained fix. Headless coverage is in `afis_lflu_lflp.sh`;
this plan is what the REPL structurally cannot check — real DataRefs, real STT/TTS,
real timing.

For each flight, drop `Log.txt` **and** `transcript.log` in `~/Téléchargements/`.
`transcript.log` carries the per-event coordinates/altitude/heading that `Log.txt`
does not — both are needed.

## Log anchors to grep

```
IFR SID climb: probe10nm tma_ceil=… cta_ceil=… (…) dep=… -> step1 FL… step2 FL… hold … NM
[cifp] <ICAO> rwy <RW> binding min -> … at <FIX>; intermediate floor -> … at <FIX>
[DBG] sid-handoff probe: lateral='…' above='…' -> <ctrl> <freq> (lateral|above|none)
IFR SID climb: sector handoff -> <ctrl> <freq> (openair '…', lateral|above) … FL… <held|queued>
IFR SID climb: FL140 (step2)
```

---

## 1. LFLP → LFMN, SID **ROMA2A** (ouest) — vol de référence

The regression flight. Everything below is what beta-55 got wrong.

| # | Attendu | Piège guardé |
|---|---|---|
| 1.1 | step1 = **FL110**, hold **30 NM** (override ouest actif) | le scoping ouest ne doit PAS avoir désactivé la retenue |
| 1.2 | Un seul appel au check-in : « radar contact, climb flight level 110 » | double appel séparé |
| 1.3 | Handoff **Lyon 120.230** en entrant dans LYON CTA (~21 NM) | Genève à 13,5 NM (bug sonde verticale) |
| 1.4 | Le log dit `(… lateral)`, pas `(… above)` | retour au modèle vertical |
| 1.5 | Le nouveau contrôleur accuse le check-in **sec** : « radar contact » | FL140 annoncé trop tôt, pendant la retenue |
| 1.6 | **FL140 seulement après 30 NM**, dit par **Lyon** | FL140 par Chambéry / avant la libération |
| 1.7 | Puis palier FL140, handoff Marseille, montée croisière | échelle bloquée à FL140 |

## 2. LFLP → est, SID **ESAP2A** ou **ODIK2A** — le nouveau plancher

Le cœur du correctif. Départ vers l'est, terrain élevé.

| # | Attendu | Piège guardé |
|---|---|---|
| 2.1 | `intermediate floor -> 13000 ft at LP620` (ESAP2A) ou `at LP610` (ODIK2A) | plancher lu au fixe de sortie (FL150) puis annulé |
| 2.2 | step1 = **FL130**, pas FL110 | l'avion buste LP620 à 4,9 NM |
| 2.3 | `hold 10 NM` (générique) — **pas** 30 NM | la retenue ouest s'applique encore à l'est |
| 2.4 | Passage LP620/LP610 **à FL130 ou au-dessus** | à vérifier sur `transcript.log` (alt + coords) |
| 2.5 | Puis step2 / croisière normalement | échelle cassée par le plancher relevé |

## 3. LFLP → **VENA2A** — non-régression

step1 = **FL130** comme avant le correctif (son plus haut minimum était déjà à un
fixe intermédiaire, LP610). Rien ne doit changer. Vol court suffisant.

## 4. LFLL → sud (plaine) — Phase 2.8 en empilement vertical

Le cas où le transfert est réellement vertical. Vérifié au REPL, jamais en sim.
Pile attendue (capture utilisateur 2026-08-10) :

```
ST EXUPERY CTR    GND   – 2500       LYON TMA S1   2500  – 4500
LYON TMA S3       4500  – FL065      LYON TMA S4   FL065 – FL115
LYON CTA          FL115 – FL145      MARSEILLE CTA S7  FL145 – FL195
```

| # | Attendu |
|---|---|
| 4.1 | step1 = **FL110** (sous le plafond FL115 de LYON TMA S4) |
| 4.2 | step2 = **FL140** (sous le plafond FL145 de LYON CTA) |
| 4.3 | Handoff **latéral** Lyon Approach 121.205 → 120.230, pas de silence |
| 4.4 | Puis Marseille pour la croisière |

## 5. LIMF → **KUKE1Z** — non-régression escalier

Le contre-exemple qui a dicté la forme du correctif. SID en escalier
(+2000 / +5000 / +6000 / +FL100 / +FL190 / +FL200 à la sortie).

| # | Attendu | Piège guardé |
|---|---|---|
| 5.1 | `intermediate floor -> 2000 ft at MF702` | plancher = FL190 @MATOG |
| 5.2 | step1 reste **FL110** | « climb flight level 190 » dès le palier de 2000 ft |
| 5.3 | Échelle FL110 → FL140 → croisière intacte | échelle écrasée en une seule clearance |

**Défaut connu, hors périmètre :** l'avion traverse MATOG (+FL190, 11,4 NM) autorisé
FL110. Corriger cela demande un plancher continu suivi fixe par fixe le long de la
route, pas un palier statique. À ne pas signaler comme régression.

## 6. LFLP **PAS2A** (nord) — à caractériser

Volontairement laissé hors du `match_fixes` ouest. Aucune attente ferme : noter le
`step1` / `hold` obtenus et si le résultat est plausible vis-à-vis du relief au nord
d'Annecy. Sert à décider s'il faut sa propre entrée `departure_holds`.
