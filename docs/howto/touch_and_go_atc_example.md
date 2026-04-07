# Touch and Go / Go-Around — ATC Test Script

Towered Airport, z.B. **LSZG Grenchen**
Callsign: **HB-LUK** (Hotel Bravo Lima Uniform Kilo)

---

## Voraussetzungen

- Rundflug-Flow (siehe `rundflug_atc_example.md`) muss funktionieren
- Flugzeug am Gate/Parking oder bereits im Pattern
- COM1 auf Tower-Frequenz (LSZG: 120.100 MHz)
- PTT-Taste konfiguriert
- API Key gespeichert

## LSZG Frequenzen

| Service | Frequenz |
|---------|----------|
| ATIS    | 121.100  |
| Ground  | 121.800  |
| Tower   | 120.100  |

---

## Übersicht: Touch and Go

Touch and Go = Aufsetzen + sofort wieder durchstarten, ohne die Piste zu verlassen.
Typisch für Landetraining. Der Pilot bleibt im Pattern und fliegt weitere Runden.

```
Taxi → Takeoff → Pattern → Touch and Go → Pattern → Touch and Go → ... → Full Stop Landing
```

### State-Flow:

```
TOWER_CONTACT → REQUEST_TOUCH_AND_GO → TOUCH_AND_GO_CLEARED
TOUCH_AND_GO_CLEARED → REPORT_POSITION → PATTERN_ENTRY → REPORT_FINAL → LANDING_CLEARED
                                                        → REQUEST_TOUCH_AND_GO → TOUCH_AND_GO_CLEARED (weitere Runde)
```

---

## Ablauf: Touch and Go

*Voraussetzung: Taxi + Takeoff wie im Rundflug-Script (Schritte 1–5). Hier geht es ab TOWER_CONTACT weiter.*

### 1. Touch and Go anfordern (TOWER_CONTACT → TOUCH_AND_GO_CLEARED)

*Nach dem Abheben, noch auf Tower-Frequenz*

**Pilot:**
> "Grenchen Tower, Hotel Bravo Lima Uniform Kilo, request touch and go runway zero six"

**Erwartete ATC-Antwort:**
> "Hotel Bravo Lima Uniform Kilo, runway 06, cleared touch and go, wind calm."

**Pilot-Antwort: READBACK** (Touch-and-Go-Clearance)
> "Cleared touch and go runway 06, Hotel Bravo Lima Uniform Kilo"

**ATC:** Stille

---

*Touch and Go durchführen: Aufsetzen, Vollgas, wieder abheben, Platzrunde fliegen*

---

### 2. Downwind melden nach Touch and Go (TOUCH_AND_GO_CLEARED → PATTERN_ENTRY)

*Querab der Pistenmitte, auf Platzrundenhöhe*

**Pilot:**
> "Grenchen Tower, Hotel Bravo Lima Uniform Kilo, left downwind runway zero six, touch and go"

**Erwartete ATC-Antwort:**
> "Hotel Bravo Lima Uniform Kilo, number one, runway 06, continue approach, report final."

**Pilot-Antwort: ROGER/WILCO**
> "Will report final, Hotel Bravo Lima Uniform Kilo"

**ATC:** Stille

---

### 3a. Final melden — Full Stop (PATTERN_ENTRY → LANDING_CLEARED)

*Wenn du jetzt landen willst (kein weiterer Touch and Go):*

**Pilot:**
> "Hotel Bravo Lima Uniform Kilo, final runway zero six"

**Erwartete ATC-Antwort:**
> "Hotel Bravo Lima Uniform Kilo, runway 06, cleared to land, wind calm."

**Pilot-Antwort: READBACK**
> "Cleared to land runway 06, Hotel Bravo Lima Uniform Kilo"

*→ Weiter mit Runway verlassen wie im Rundflug-Script (Schritt 8)*

---

### 3b. Noch ein Touch and Go anfordern (PATTERN_ENTRY → TOUCH_AND_GO_CLEARED)

*Wenn du eine weitere Runde fliegen willst:*

**Pilot:**
> "Hotel Bravo Lima Uniform Kilo, request touch and go runway zero six"

**Erwartete ATC-Antwort:**
> "Hotel Bravo Lima Uniform Kilo, runway 06, cleared touch and go, wind calm."

**Pilot-Antwort: READBACK**
> "Cleared touch and go runway 06, Hotel Bravo Lima Uniform Kilo"

*→ Zurück zu Schritt 2 (Downwind melden nach Touch and Go)*

---

## Ablauf: Go-Around

Go-Around = Durchstarten aus dem Anflug. Kann jederzeit im Pattern oder auf dem Final passieren.

### State-Flow:

```
PATTERN_ENTRY   → GO_AROUND → PATTERN_ENTRY (re-enter pattern)
LANDING_CLEARED → GO_AROUND → PATTERN_ENTRY (re-enter pattern)
TOUCH_AND_GO_CLEARED → GO_AROUND → PATTERN_ENTRY (re-enter pattern)
```

---

### 4a. Go-Around aus dem Pattern (PATTERN_ENTRY → PATTERN_ENTRY)

*Auf Base oder im Pattern, du entscheidest dich zum Durchstarten*

**Pilot:**
> "Grenchen Tower, Hotel Bravo Lima Uniform Kilo, going around"

**Erwartete ATC-Antwort:**
> "Hotel Bravo Lima Uniform Kilo, roger, fly runway heading, climb and maintain pattern altitude, re-enter left downwind runway 06."

**Pilot-Antwort: READBACK**
> "Fly runway heading, re-enter left downwind runway 06, Hotel Bravo Lima Uniform Kilo"

*→ Platzrundenhöhe steigen, Downwind erneut anfliegen, weiter mit Schritt 2 oder Rundflug Schritt 6*

---

### 4b. Go-Around nach Landing Clearance (LANDING_CLEARED → PATTERN_ENTRY)

*Auf dem Final, bereits "cleared to land", aber Durchstarten nötig (z.B. Piste blockiert)*

**Pilot:**
> "Hotel Bravo Lima Uniform Kilo, going around"

**Erwartete ATC-Antwort:**
> "Hotel Bravo Lima Uniform Kilo, roger, fly runway heading, climb and maintain pattern altitude, re-enter left downwind runway 06."

**Pilot-Antwort: READBACK**
> "Fly runway heading, re-enter left downwind runway 06, Hotel Bravo Lima Uniform Kilo"

*→ Durchstarten, Platzrundenhöhe steigen, Pattern erneut fliegen*

---

### 4c. Go-Around nach Touch-and-Go Clearance (TOUCH_AND_GO_CLEARED → PATTERN_ENTRY)

*Du hast Touch-and-Go Clearance, entscheidest dich aber zum Durchstarten ohne aufzusetzen*

**Pilot:**
> "Hotel Bravo Lima Uniform Kilo, going around"

**Erwartete ATC-Antwort:**
> "Hotel Bravo Lima Uniform Kilo, roger, fly runway heading, re-enter left downwind runway 06."

**Pilot-Antwort: READBACK**
> "Fly runway heading, re-enter left downwind runway 06, Hotel Bravo Lima Uniform Kilo"

---

## Kompletter Testdurchlauf: Multiple Circuits

Ein typisches Landetraining mit mehreren Touch-and-Gos:

```
1. Taxi + Takeoff (Rundflug-Script Schritte 1–5)
2. Touch and Go anfordern (Schritt 1)
3. Touch and Go durchführen
4. Downwind melden (Schritt 2)
5. Nochmal Touch and Go anfordern (Schritt 3b)
6. Touch and Go durchführen
7. Downwind melden (Schritt 2)
8. Final melden — Full Stop (Schritt 3a)
9. Landen, Runway verlassen (Rundflug-Script Schritt 8)
10. Taxi to Parking (Rundflug-Script Schritt 9)
```

**Erwartete State-Transitions:**
```
IDLE → TAXI_CLEARED → TOWER_CONTACT → TOUCH_AND_GO_CLEARED
→ PATTERN_ENTRY → TOUCH_AND_GO_CLEARED
→ PATTERN_ENTRY → LANDING_CLEARED → IDLE
```

---

## Edge Cases zum Testen

### Go-Around mit anschliessendem Touch and Go

1. Cleared to land (LANDING_CLEARED)
2. "Going around" → PATTERN_ENTRY
3. Downwind melden → PATTERN_ENTRY
4. "Request touch and go" → TOUCH_AND_GO_CLEARED
5. Touch and Go durchführen
6. Downwind melden → PATTERN_ENTRY
7. Final melden → LANDING_CLEARED
8. Landen

### Unverständliche Eingabe im Touch-and-Go State

**Pilot (in TOUCH_AND_GO_CLEARED):**
> "Irgendwas unverständliches"

**Erwartet:**
> "Hotel Bravo Lima Uniform Kilo, report your position."

### Touch and Go direkt aus PATTERN_ENTRY

**Pilot (in PATTERN_ENTRY, ohne vorherigen T&G Request):**
> "Hotel Bravo Lima Uniform Kilo, request touch and go runway zero six"

**Erwartet:**
> "Hotel Bravo Lima Uniform Kilo, runway 06, cleared touch and go, wind calm."

---

## Log prüfen

In X-Plane `Log.txt` nach `[xp_wellys_atc]` suchen:

```
[xp_wellys_atc] ATC state: TOWER_CONTACT -> TOUCH_AND_GO_CLEARED  ← T&G Clearance
[xp_wellys_atc] ATC state: TOUCH_AND_GO_CLEARED -> PATTERN_ENTRY  ← nach Downwind Report
[xp_wellys_atc] ATC state: PATTERN_ENTRY -> TOUCH_AND_GO_CLEARED  ← nächster T&G
[xp_wellys_atc] ATC state: LANDING_CLEARED -> PATTERN_ENTRY       ← Go-Around
```
