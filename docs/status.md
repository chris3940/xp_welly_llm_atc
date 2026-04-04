# Project Status — xp_wellys_atc

Letzte Aktualisierung: 2026-04-04

## Milestone-Ubersicht

| # | Milestone | Status |
|---|-----------|--------|
| M1 | Settings, ImGui UI, Keychain API Key | Done |
| M2 | Core Audio Recording, PTT, WAV Encoding | Done |
| M3 | Whisper STT — async transcription, transcript display | Done (nicht committed) |
| M4 | Intent Parser — rule-based transcript → PilotIntent | Done (nicht committed) |
| M5 | ATC State Machine — VFR ATC logic + response generation | Offen |
| M6 | GPT Fallback — GPT-4o-mini for unknown intents | Offen |
| M7 | TTS + Audio Playback — OpenAI TTS + Core Audio MP3 | Offen |
| M8 | Polish — final testing, edge cases, cleanup | Offen |

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

- M3+M4 in X-Plane testen (PTT → sprechen → Intent im UI pruefen)
- Commit erstellen
- M5 (ATC State Machine) beginnen
