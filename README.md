# xp_wellys_atc

AI-powered ATC voice communication plugin for X-Plane 12 VFR flights.

Talk to ATC using your microphone via push-to-talk. The plugin transcribes your speech (OpenAI Whisper), interprets your intent through a rule-based ATC state machine, and plays back realistic ATC responses (OpenAI TTS).

## Features

- **Push-to-Talk** — keyboard or joystick button
- **Speech-to-Text** — OpenAI Whisper transcription
- **ATC State Machine** — VFR phraseology for towered and non-towered airports
- **GPT Fallback** — GPT-4o-mini handles ambiguous or unrecognized intents
- **Text-to-Speech** — natural ATC voice responses via OpenAI TTS
- **ImGui UI** — in-sim transcript panel, status bar, and settings

## Requirements

- macOS 12.0+ (ARM64 / x86_64 universal binary)
- X-Plane 12
- CMake 3.21+
- Homebrew LLVM (`brew install llvm`)
- OpenAI API key

## Quick Start

```bash
# 1. Clone and setup dependencies
git clone <repo-url>
cd xp_wellys_atc
make setup      # Downloads X-Plane SDK, Dear ImGui, nlohmann/json

# 2. Build
make build      # CMake Release build → build/xp_wellys_atc.xpl

# 3. Install into X-Plane
make install    # Code-sign + deploy to X-Plane plugins directory
```

## Configuration

On first launch, open the plugin menu in X-Plane and enter your OpenAI API key in the settings tab. The key is stored securely in the macOS Keychain — it never touches disk.

Settings are stored in `data/settings.json`:

| Setting | Default | Description |
|---|---|---|
| `ptt_key_vk` | `49` (Space) | Virtual key code for push-to-talk |
| `ptt_joystick_button` | `-1` (off) | Joystick button index for PTT |
| `tts_voice` | `onyx` | OpenAI TTS voice |
| `pilot_callsign` | `November One Two Three Alpha Bravo` | Your callsign |
| `volume` | `1.0` | Playback volume (0.0–1.0) |
| `gpt_fallback_enabled` | `true` | Use GPT when intent confidence is low |

## Usage

1. Tune COM1/COM2 to the appropriate frequency in X-Plane
2. Hold the PTT key and speak your radio call (e.g., *"Zurich Tower, November 123AB, request taxi"*)
3. Release PTT — the plugin transcribes, processes, and plays back the ATC response
4. Check the ImGui overlay for transcript history and current ATC state

## Other Make Targets

```bash
make format     # Run clang-format on all source files
make lint       # Run clang-tidy
make clean      # Remove build artifacts
```

## License

Private project — not licensed for redistribution.
