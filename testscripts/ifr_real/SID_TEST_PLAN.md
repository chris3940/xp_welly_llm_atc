# SID climb — in-sim test plan (beta-56)

> **Superseded by [TEST_PLAN.md](TEST_PLAN.md) (beta-61).** Kept for its
> SID-by-SID detail, which the newer plan does not repeat. Several items here are
> now covered headless by `make test-afis` — check that plan first so you do not
> spend a flight on something the harness already proves.

Covers the two changes that have **never been flown**: the lateral sector handoff
(Phase 2.8, decoupled from the departure hold) and the SID climb **floor** now read
from the earliest constrained fix. Headless coverage is in `afis_lflu_lflp.sh`;
this plan is what the REPL structurally cannot check — real DataRefs, real STT/TTS,
real timing.

For each flight, drop `Log.txt` **and** `transcript.log` in `~/Téléchargements/`.
`transcript.log` carries the per-event coordinates/altitude/heading that `Log.txt`
does not — both are needed.

## Le raccourci direct-to doit être forcé dans les deux sens

Le direct-to SID a **deux points de déclenchement** et, sans forçage, un tirage à
20 % — donc un vol pris au hasard ne dit pas quel chemin a été exercé. Réglage
`shortcut_always` (Settings, « SHORTCUTS ALWAYS ») :

| Réglage | Chemin exercé |
|---|---|
| **OFF** | Phase 1 / 1b ne tirent pas (4 fois sur 5) → montée SID nominale, aucun direct |
| **ON** | Le direct part à coup sûr : au check-in si déjà ≥15 NM (Phase 1), sinon au franchissement des 15 NM (Phase 1b, `engine.cpp:5082`) |

**Chaque cas ci-dessous marqué « ×2 » se vole deux fois, une fois par position.**

**Point de vigilance (défaut antérieur, `kSidDirectMinNm`, 2026-08-07) :** Phase 1b
n'a **aucune garde vérifiant que le fixe de sortie est encore devant l'avion**. Sur
les SID est de LFLP, ESAPI est à 8,1 NM et ODIKI à 10,0 NM du terrain alors que
l'offre se déclenche à 15 NM. Le tracker de route ne reculera pas (il cherche vers
l'avant), mais le **texte parlé** peut annoncer un direct vers un point déjà passé.
À surveiller explicitement au cas 2b. Ne pas l'imputer aux commits `1e8710b` /
`9f70a0f`.

## Log anchors to grep

```
IFR SID climb: probe10nm tma_ceil=… cta_ceil=… (…) dep=… -> step1 FL… step2 FL… hold … NM
[cifp] <ICAO> rwy <RW> binding min -> … at <FIX>; intermediate floor -> … at <FIX>
[DBG] sid-handoff probe: lateral='…' above='…' -> <ctrl> <freq> (lateral|above|none)
IFR SID climb: sector handoff -> <ctrl> <freq> (openair '…', lateral|above) … FL… <held|queued>
IFR SID climb: FL140 (step2)
[route] ATC direct: <FIX> (idx=…, SID)            ← raccourci au check-in (Phase 1)
[route] ATC direct: <FIX> (idx=…, SID deferred)   ← raccourci différé (Phase 1b, 15 NM)
IFR SID climb: deferred direct <FIX> (… NM crossing…)
```

---

## 1. LFLP → LFMN, SID **ROMA2A** (ouest) — vol de référence ×2

The regression flight. Everything below is what beta-55 got wrong.

ROMAM est à 63,6 NM du terrain, donc largement devant l'avion au franchissement des
15 NM : c'est le cas **sain** du raccourci, celui qui sert de référence au cas 2b.

| # | Attendu | Piège guardé |
|---|---|---|
| 1.1 | step1 = **FL110**, hold **30 NM** (override ouest actif) | le scoping ouest ne doit PAS avoir désactivé la retenue |
| 1.2 | Un seul appel au check-in : « radar contact, climb flight level 110 » | double appel séparé |
| 1.3 | Handoff **Lyon 120.230** en entrant dans LYON CTA (~21 NM) | Genève à 13,5 NM (bug sonde verticale) |
| 1.4 | Le log dit `(… lateral)`, pas `(… above)` | retour au modèle vertical |
| 1.5 | Le nouveau contrôleur accuse le check-in **sec** : « radar contact » | FL140 annoncé trop tôt, pendant la retenue |
| 1.6 | **FL140 seulement après 30 NM**, dit par **Lyon** | FL140 par Chambéry / avant la libération |
| 1.7 | Puis palier FL140, handoff Marseille, montée croisière | échelle bloquée à FL140 |
| 1.8 | **SHORTCUTS OFF** : aucun `[route] ATC direct:` dans le log | le raccourci part quand même |
| 1.9 | **SHORTCUTS ON** : « direct ROMAM, when able » **une seule fois**, et le handoff Lyon + la retenue FL110 se déroulent identiquement | le direct casse l'échelle ou la retenue |

## 2. LFLP → est, SID **ESAP2A** ou **ODIK2A** — le nouveau plancher

Le cœur du correctif. Départ vers l'est, terrain élevé.

| # | Attendu | Piège guardé |
|---|---|---|
| 2.1 | `intermediate floor -> 13000 ft at LP620` (ESAP2A) ou `at LP610` (ODIK2A) | plancher lu au fixe de sortie (FL150) puis annulé |
| 2.2 | step1 = **FL130**, pas FL110 | l'avion buste LP620 à 4,9 NM |
| 2.3 | `hold 10 NM` (générique) — **pas** 30 NM | la retenue ouest s'applique encore à l'est |
| 2.4 | Passage LP620/LP610 **à FL130 ou au-dessus** | à vérifier sur `transcript.log` (alt + coords) |
| 2.5 | Puis step2 / croisière normalement | échelle cassée par le plancher relevé |

### 2b. Le même, **SHORTCUTS ON** — le cas à risque

ESAPI est à 8,1 NM (ODIKI 10,0 NM) alors que l'offre différée se déclenche à 15 NM,
et Phase 1b ne vérifie pas que la cible est devant.

| # | À observer |
|---|---|
| 2b.1 | Le direct part-il, et **à quelle distance** ? (`deferred direct … NM crossing`) |
| 2b.2 | À cet instant, l'avion a-t-il **déjà passé ESAPI/ODIKI** ? (`transcript.log`, coords) |
| 2b.3 | Si oui → défaut confirmé : garde « cible encore devant » à ajouter en Phase 1b |
| 2b.4 | Le plancher FL130 tient-il malgré le direct ? (il doit : step1 est calculé à l'init, indépendamment du raccourci) |

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
