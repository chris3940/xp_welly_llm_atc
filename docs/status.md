# Project Status — xp_wellys_atc

Letzte Aktualisierung: 2026-04-04

## Milestone-Ubersicht

| # | Milestone | Status |
|---|-----------|--------|
| M1 | Settings, ImGui UI, Keychain API Key | Done |
| M2 | Core Audio Recording, PTT, WAV Encoding | Done |
| M3 | Whisper STT — async transcription, transcript display | Done |
| M4 | Intent Parser — rule-based transcript → PilotIntent | Done |
| M5 | ATC State Machine + GPT Fallback + Full Text Pipeline | Done |
| M6 | TTS + Audio Playback — OpenAI TTS + Core Audio MP3 | Offen |
| M7 | Polish — final testing, edge cases, cleanup | Offen |

## M5 — Was wurde gemacht

- `atc_state_machine.hpp/.cpp`: ATCState Enum (8 States), ATCResponse Struct, `process()` mit allen VFR Transitions und Phraseologie-Templates
  - IDLE → GROUND_CONTACT → TAXI_CLEARED → TOWER_CONTACT → DEPARTURE_CLEARED → IDLE
  - TOWER_CONTACT → PATTERN_ENTRY → LANDING_CLEARED → IDLE
  - UNICOM_ACTIVE fuer nicht-kontrollierte Flugplaetze
- `gpt_client.hpp/.cpp`: Async POST an `/v1/chat/completions` (gpt-4o-mini), gleicher callback-queue Pattern wie whisper_client, ATC System-Prompt mit ICAO Phraseologie
- `atc_session.cpp`: Nach Intent-Parse → State Machine → bei leerem Response/low confidence/UNKNOWN → GPT Fallback (oder "Say again" wenn disabled) → ATC Response ins Transcript
- `atc_ui.cpp`: ATCState-Anzeige + Reset-Button im Status-Tab, Airport-Prefix bei ATC-Lines im Transcript
- `main.cpp`: init/stop fuer atc_state_machine + gpt_client, drain gpt callback queue im Flight Loop
- `xplane_context`: QNH (barometer_sealevel_inhg), Wind Direction/Speed DataRefs hinzugefuegt

## Bugfixes und Erweiterungen (zwischen M1-M5)

### ImGui UI komplett umgebaut
- **Problem:** ImGui Fenster war leer, dann ausserhalb des Dialogs, nicht klickbar
- **Loesung:** Kompletter Umbau nach xp_pilot Pattern:
  - Fullscreen unsichtbares XPLM-Fenster (`DecorationNone`) fuer Mouse/Keyboard Capture
  - Separater `XPLMRegisterDrawCallback` (`xplm_Phase_Window`) fuer ImGui Rendering
  - Eigene GL State Verwaltung (Viewport, Projection Matrix, Save/Restore)
  - Mouse-Forwarding (X-Plane Screen-Coords → ImGui Coords)
  - Keyboard-Forwarding (Printable chars + Backspace/Delete/Enter/Escape)
  - `XPLMTakeKeyboardFocus` fuer Input-Capture
  - `XPLMGetMouseLocationGlobal` im Draw-Callback fuer Hover-Support
  - ImGui Fenster zentriert, draggable, resizable

### HFS-Pfad-Fix (macOS)
- **Problem:** `XPLMGetSystemPath` und `XPLMGetPluginInfo` geben auf macOS HFS-Pfade zurueck (Doppelpunkte statt Slashes: `Macintosh HD:Users:...`)
- **Betroffen:** settings.json konnte nicht geschrieben/gelesen werden, apt.dat nicht geoeffnet
- **Loesung:** HFS → POSIX Konvertierung in `xplane_context.cpp` und `settings.cpp`

### Towered Airport Detection via apt.dat
- **Problem:** ILS-basierte Heuristik war unzuverlaessig (ILS-IDs ≠ Airport-ICAO, LSZB als "Uncontrolled")
- **Loesung:** apt.dat Parse beim Plugin-Start (Background-Thread), sucht TWR-Frequenzen (Code 54 + 1054 fuer X-Plane 12 Format), cached in `unordered_set` fuer O(1) Lookup
- Default `towered=true` bis Cache geladen (~1-2s)

### PTT Click-Sound
- 880Hz Sinus-Pip, 80ms, Fade-in/out Envelope
- Wird bei PTT-Press abgespielt ueber Core Audio AudioQueue
- Lautstaerke folgt Volume-Setting

### Audio Output Device Selection
- Core Audio Device-Enumeration (alle Output-Devices)
- Dropdown im Settings-Tab mit Refresh-Button
- AudioQueue wird auf gewaehltes Device geroutet (`kAudioQueueProperty_CurrentDevice`)
- Gespeichert in settings.json als Device-UID
- Zweck: ATC-Audio aufs Headset, X-Plane Engine-Sound auf Hauptlautsprecher

### Avionics-Check fuer PTT
- PTT blockiert wenn `sim/cockpit/electrical/avionics_on == 0` (Cold & Dark)
- Log-Meldung: "PTT blocked — avionics off"

### API Key Paste-Button
- "Paste" Button neben API Key Input (Cmd+V geht nicht in ImGui/X-Plane)
- Liest aus macOS Clipboard via `pbpaste`

### PTT via X-Plane Command
- PTT-Zuweisung ueber X-Plane Key/Joystick Binding statt settings.json

### Sonstige Fixes
- `compile_commands.json` Symlink fuer LSP/Editor Support
- clang-tidy clean (alle Warnings behoben)

## M4 — Was wurde gemacht

- `intent_parser.hpp`: PilotIntent-Enum (12 Intents), PilotMessage-Struct mit raw_transcript/intent/confidence/callsign/runway
- `intent_parser.cpp`: Rule-based Keyword-Matching in Prioritaetsreihenfolge:
  - UNABLE (0.95), SELF_ANNOUNCE (0.90), READY_FOR_DEPARTURE (0.90), RUNWAY_VACATED (0.90), REQUEST_TAXI (0.90), REPORT_POSITION (0.90), REQUEST_LANDING (0.85), REQUEST_FREQUENCY (0.80), INITIAL_CALL (0.85), READBACK (0.75)
- Callsign-Extraktion: Phonetic-Alphabet-Sequenzen, N-Number ("november ..."), Vergleich mit settings::pilot_callsign
- Runway-Extraktion: Spoken Numbers ("two six" → "26"), Suffixe (left/right/center → L/R/C), direkte Ziffern
- Context Weighting: REQUEST_TAXI bei !on_ground → 0.3, INITIAL_CALL mit "tower" bei !is_towered → 0.4, SELF_ANNOUNCE bei towered → 0.3
- `atc_session.cpp`: Nach Whisper-Callback wird intent_parser::parse() aufgerufen, PilotMessage gespeichert, Intent+Confidence geloggt
- `atc_ui.cpp`: Status-Tab zeigt "Last Parsed Intent" mit Intent-Name, Confidence, Callsign, Runway, Raw Transcript

## M3 — Was wurde gemacht

- `whisper_client.cpp`: Async POST an OpenAI `/v1/audio/transcriptions` via libcurl multipart, detached thread, thread-safe callback queue (drained im flight loop)
- `atc_session.cpp`: `on_ptt_released()` ruft jetzt Whisper auf, Minimum-Duration-Gate (< 0.5s wird verworfen), State → PROCESSING waehrend API-Call
- `atc_ui.cpp`: Neuer "Transcript"-Tab mit scrollbarer Liste (Pilot = weiss, ATC = cyan vorbereitet), Auto-Scroll, Clear-Button
- `main.cpp`: `whisper_client::init/stop` + `drain_callback_queue()` im Flight Loop

## Naechster Schritt

- M5 in X-Plane testen (PTT → sprechen → ATC Response im Transcript pruefen)
- M6 (TTS + Audio Playback) beginnen
