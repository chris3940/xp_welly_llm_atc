# Rundflug (Traffic Pattern) — ATC Test Script

Towered Airport, z.B. **LSZG Grenchen**
Callsign: **HB-LUK** (Hotel Bravo Lima Uniform Kilo)

---

## Voraussetzungen

- Flugzeug am Gate/Parking, Avionics ON
- COM1 auf Ground-Frequenz (z.B. 121.9)
- PTT-Taste konfiguriert
- API Key gespeichert

---

## Ablauf

### 1. Ground kontaktieren (IDLE → GROUND_CONTACT)

**Pilot:**
> "Grenchen Ground, Hotel Bravo Lima Uniform Kilo"

**Erwartete ATC-Antwort:**
> "Hotel Bravo Lima Uniform Kilo, LSZG Ground, information Alpha, runway 26, QNH 1013."

---

### 2. Taxi anfordern (GROUND_CONTACT → TAXI_CLEARED)

**Pilot:**
> "Request taxi, Hotel Bravo Lima Uniform Kilo"

**Erwartete ATC-Antwort:**
> "Hotel Bravo Lima Uniform Kilo, taxi to holding point runway 26 via Alpha, QNH 1013."

---

### 3. Readback Taxi-Clearance (bleibt TAXI_CLEARED)

**Pilot:**
> "Taxi to holding point runway two six via Alpha, QNH one zero one three, Hotel Bravo Lima Uniform Kilo"

**Erwartete ATC-Antwort:**
> "Hotel Bravo Lima Uniform Kilo, readback correct, contact tower when ready."

---

### 4. Tower kontaktieren (TAXI_CLEARED → TOWER_CONTACT)

*COM1 auf Tower-Frequenz wechseln*

**Pilot:**
> "Grenchen Tower, Hotel Bravo Lima Uniform Kilo, holding short runway two six"

**Erwartete ATC-Antwort:**
> "Hotel Bravo Lima Uniform Kilo, LSZG Tower, runway 26, hold short, number one."

---

### 5. Ready for Departure (TOWER_CONTACT → DEPARTURE_CLEARED)

**Pilot:**
> "Hotel Bravo Lima Uniform Kilo, ready for departure"

**Erwartete ATC-Antwort:**
> "Hotel Bravo Lima Uniform Kilo, runway 26, cleared for takeoff, wind 240 degrees 08 knots."

---

### 6. Readback Takeoff-Clearance (DEPARTURE_CLEARED → IDLE)

**Pilot:**
> "Cleared for takeoff runway two six, Hotel Bravo Lima Uniform Kilo"

**Erwartete ATC-Antwort:**
> "Hotel Bravo Lima Uniform Kilo, readback correct, frequency change approved, good day."

---

*Abheben, Platzrunde fliegen (Crosswind → Downwind → Base → Final)*

---

### 7. Downwind melden (IDLE → PATTERN_ENTRY)

**Pilot:**
> "Grenchen Tower, Hotel Bravo Lima Uniform Kilo, inbound for landing, midfield left downwind runway two six"

**Erwartete ATC-Antwort (INITIAL_CALL_INBOUND):**
> "Hotel Bravo Lima Uniform Kilo, LSZG Tower, enter left downwind runway 26, report midfield downwind."

*Alternativ, falls als REPORT_POSITION_DOWNWIND im PATTERN_ENTRY erkannt:*
> "Hotel Bravo Lima Uniform Kilo, number one, runway 26, continue approach, report final."

---

### 8. Final melden (PATTERN_ENTRY → LANDING_CLEARED)

**Pilot:**
> "Hotel Bravo Lima Uniform Kilo, final runway two six"

**Erwartete ATC-Antwort:**
> "Hotel Bravo Lima Uniform Kilo, runway 26, cleared to land, wind 240 degrees 08 knots."

---

### 9. Runway verlassen (LANDING_CLEARED → IDLE)

*Landen, Runway via Taxiway verlassen*

**Pilot:**
> "Hotel Bravo Lima Uniform Kilo, clear of runway two six"

**Erwartete ATC-Antwort:**
> "Hotel Bravo Lima Uniform Kilo, contact ground on 121.9, good day."

---

## Edge Cases zum Testen

### Radio Check (IDLE, beliebiger State)

**Pilot:**
> "Radio check"

**Erwartet:**
> "Hotel Bravo Lima Uniform Kilo, LSZG Tower, reading you five by five."

### Unverständliche Eingabe (_INVALID)

**Pilot:**
> "Ich möchte eine Pizza bestellen"

**Erwartet (je nach State):**
> "Hotel Bravo Lima Uniform Kilo, say again your request."

### Falscher State (z.B. Ready for Departure im IDLE)

**Pilot:**
> "Ready for departure"

**Erwartet:**
> _INVALID-Antwort für den aktuellen State

---

## Log prüfen

In X-Plane `Log.txt` nach `[xp_wellys_atc]` suchen:

```
[xp_wellys_atc][DEBUG] Whisper response: "..."     ← Transkription korrekt?
[xp_wellys_atc][DEBUG] Intent: ... (confidence=...) ← Intent + Confidence
[xp_wellys_atc] ATC state: IDLE -> GROUND_CONTACT  ← State-Transition
[xp_wellys_atc] Routing to GPT intent classification ← GPT-Fallback aktiv?
```
