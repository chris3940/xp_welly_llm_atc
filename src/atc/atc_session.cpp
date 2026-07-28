/*
 * xp_wellys_atc - AI-powered ATC voice communication for X-Plane 12
 * Copyright (C) 2026 thWelly & Claude (Anthropic)
 * Copyright (C) 2026 Christopher P. Potter (Linux port + IFR extensions)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 */

#include "atc/atc_session.hpp"
#include "atc/atc_state_machine.hpp"
#include "atc/atis_generator.hpp"
#include "atc/phonetic.hpp"
#include "atc/engine.hpp"
#include "atc/flight_phase.hpp"
#include "atc/atc_templates.hpp"
#include "atc/intent_parser.hpp"
#include "audio/audio_player.hpp"
#include "audio/audio_recorder.hpp"
#include "backends/manager.hpp"
#include "core/logging.hpp"
#include "core/xplane_context.hpp"
#include "data/airspace_db.hpp"
#include "data/cifp_reader.hpp"
#include "data/simbrief_ofp.hpp"
#include "persistence/model_manifest.hpp"
#include "persistence/model_paths.hpp"
#include "persistence/settings.hpp"

#include <XPLMProcessing.h>
#include <XPLMUtilities.h>

#include <cctype>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <functional>
#include <sstream>
#include <string>
#include <unordered_set>
#include <utility>

namespace atc_session {

static PTTState state_ = PTTState::IDLE;

// True between speak_response() submitting a TTS job and the async
// callback firing. Without this flag, update() flips PLAYING->IDLE in
// the window where TTS is still synthesising (audio_player::is_playing()
// returns false because play_pcm hasn't been called yet) — which let
// the per-tick traffic advisor poll fire and stack a fresh advisory on
// top of the ack TTS that hadn't been spoken yet. Drained on the main
// thread by the manager's callback queue, so no atomic is needed.
static bool tts_pending_ = false;

static float last_duration_ = 0.0f;
static size_t last_samples_ = 0;
static size_t last_wav_bytes_ = 0;

static std::vector<TranscriptEntry> transcript_;
static intent_parser::PilotMessage last_pilot_message_;

// Transcript log file — opened (overwritten) at each session init.
// Path: <plugin>/Resources/transcript.log
static FILE *g_transcript_log_ = nullptr;
static std::string g_last_stt_model_;

// Returns the human-readable name of the active STT model.
static std::string current_stt_model_label() {
  const std::string &mode = settings::backend_mode();
  if (mode == "local" || mode == "local_stt_mistral")
    return settings::local_stt_model();
  if (mode == "mistral")
    return settings::mistral_stt_model();
  if (mode == "openai")
    return settings::openai_stt_model();
  return mode;
}

static void write_stt_header_if_changed() {
  if (!g_transcript_log_)
    return;
  const std::string label = current_stt_model_label();
  if (label == g_last_stt_model_)
    return;
  std::fprintf(g_transcript_log_, "-- STT: %s --\n", label.c_str());
  std::fflush(g_transcript_log_);
  g_last_stt_model_ = label;
}

static void push_transcript(TranscriptEntry e) {
  const auto &ctx = xplane_context::get();
  e.lat = ctx.latitude;
  e.lon = ctx.longitude;
  e.alt_ft = ctx.altitude_ft_msl;
  e.heading = ctx.heading_true;

  if (g_transcript_log_) {
    int mins = static_cast<int>(e.sim_time) / 60;
    int secs = static_cast<int>(e.sim_time) % 60;
    const char *freq = e.frequency.empty() ? "" : e.frequency.c_str();
    // Position suffix appended to every line for post-flight debugging.
    int qnh_hpa = ctx.qnh_hpa;
    char pos[120];
    std::snprintf(pos, sizeof(pos),
                  " @(%.4f,%.4f alt=%.0fft pa=%.0fft hdg=%.0f qnh=%d sqk=%04d mode=%d)",
                  e.lat, e.lon, e.alt_ft, ctx.pressure_alt_ft, e.heading, qnh_hpa,
                  ctx.transponder_code, ctx.transponder_mode);
    switch (e.kind) {
    case TranscriptKind::Pilot:
      write_stt_header_if_changed();
      std::fprintf(g_transcript_log_, "[%02d:%02d%s%s] You: %s%s\n", mins, secs,
                   e.frequency.empty() ? "" : " ", freq, e.text.c_str(), pos);
      break;
    case TranscriptKind::Tower:
      std::fprintf(g_transcript_log_, "[%02d:%02d%s%s] %s: %s%s\n", mins, secs,
                   e.frequency.empty() ? "" : " ", freq,
                   e.label.empty() ? "ATC" : e.label.c_str(), e.text.c_str(),
                   pos);
      break;
    case TranscriptKind::System:
      std::fprintf(g_transcript_log_, "[%02d:%02d] -- %s --%s\n", mins, secs,
                   e.text.c_str(), pos);
      break;
    }
    std::fflush(g_transcript_log_);
  }
  transcript_.push_back(std::move(e));
}

// Capture the controller label that should appear in the transcript for the
// current ATC message — stored per-entry so that historical messages are not
// retroactively relabelled when the active controller changes.
static std::string current_tower_label() {
  // Post-landing on the GROUND frequency: after the Tower->Ground handoff the
  // engine's controller label is still "Tower" (stale). Label ground responses as
  // "<airport> Ground" (user 2026-07-19: transcript showed "Tower" on the taxi-in
  // call). State is IDLE by then, so key on airborne-then-on-ground + GROUND freq.
  {
    const auto &cx0 = xplane_context::get();
    if (cx0.on_ground && atc_state_machine::was_airborne() &&
        cx0.frequency_type == xplane_context::FrequencyType::GROUND) {
      std::string apt = !cx0.nearest_airport_name.empty()
                            ? cx0.nearest_airport_name
                            : cx0.nearest_airport_id;
      auto sep = apt.find_first_of(" -");
      if (sep != std::string::npos)
        apt = apt.substr(0, sep);
      return apt.empty() ? "Ground" : apt + " Ground";
    }
  }
  const std::string &ctrl = engine::current_controller_label();
  if (!ctrl.empty())
    return ctrl;
  // For IFR airborne states: use the pending departure label stored when the
  // takeoff clearance was issued. This covers the window between takeoff and
  // when poll_departure_handoff() fires and activates the label officially.
  using AS = atc_state_machine::ATCState;
  const auto st = atc_state_machine::get_state();
  if (st == AS::IFR_FREQ_HANDOFF || st == AS::IFR_EN_ROUTE ||
      st == AS::IFR_RADAR_CONTACT || st == AS::IFR_ENROUTE_CRUISE) {
    const std::string &pending = engine::pending_departure_label();
    if (!pending.empty())
      return pending;
  }
  // En-route IFR states: nearest airport is irrelevant to the sector —
  // return "Control" until the real centre label is populated by polling.
  if (st == AS::IFR_ENROUTE_CRUISE || st == AS::IFR_EN_ROUTE ||
      st == AS::IFR_FREQ_HANDOFF || st == AS::IFR_RADAR_CONTACT ||
      st == AS::IFR_APPROACH_CONTACT || st == AS::IFR_APPROACH_DESCENT ||
      st == AS::IFR_APPROACH_TOWER)
    return "Control";
  const auto &cx = xplane_context::get();
  const std::string &name = cx.nearest_airport_name;
  const std::string &id = cx.nearest_airport_id;
  std::string apt = !name.empty() ? name : id;
  // City name only — strip local suffix ("Annecy Meythet" → "Annecy")
  auto sep = apt.find_first_of(" -");
  if (sep != std::string::npos)
    apt = apt.substr(0, sep);
  return apt.empty() ? "ATC" : apt + " ATC";
}
static int total_transcriptions_ = 0;
static int total_inferences_ = 0;
static constexpr float kMinRecordingDuration = 0.5f;
// Extra mic-open time after PTT release to avoid cutting the last syllable.
// Bumped from 0.60 to 0.90 in 4.2.2 — Voxtral was still truncating trailing
// callsigns ("...Romeo Charlie") at 600 ms for pilots who let the key go
// while the final syllable was still leaving their lips.
static constexpr float kPttTailSec = 0.90f;
static float ptt_tail_remaining_ = 0.0f;

// ATIS playback state
static bool atis_playing_ = false;
// COM (1 or 2) the active ATIS broadcast started on. Pinned at trigger
// time so the abort check stays consistent if the pilot rapidly cycles
// COM2 while COM1 is also on ATIS — we abort the stream that's playing,
// not whichever COM still happens to be on the ATIS freq.
static int atis_active_com_ = 0;
static float atis_cooldown_ = 0.0f;
// 120 s cooldown: an inbound VFR pilot at LSZB tunes ATIS, then APP, then
// TWR within ~30-90 s. With a 30 s cooldown the previous-airport's ATIS
// re-fires when the pilot retunes ATIS during that sequence — annoying
// and unrealistic. 120 s is long enough to span a normal arrival
// sequence but short enough that a pilot who deliberately retunes ATIS
// minutes later (e.g. checking for a new letter) still gets fresh
// playback.
static constexpr float kAtisCooldownSec = 120.0f;
static float atis_tuned_timer_ = 0.0f;           // how long tuned to ATIS freq
static constexpr float kAtisTuneDelaySec = 2.0f; // wait before playing

// Map the pilot's currently-tuned frequency to a logical voice role.
// The transmitting controller is whichever one owns the freq the pilot
// is listening to — *not* the state machine's next_state. Without this,
// a Ground handoff message ("contact Tower on 120.100") gets spoken
// with the Tower voice, because the state has already advanced to
// TOWER_CONTACT by the time speak_response runs.
//
// Tower-only airports collapse Ground/Approach onto the Tower voice
// (one controller handles everything on the tower freq). Unknown /
// Center / unicom-class freqs fall back to the Center voice — that's
// the en-route facility a pilot would talk to between airports.
static model_manifest::VoiceRole
role_for_frequency(const xplane_context::XPlaneContext &ctx) {
  using FT = xplane_context::FrequencyType;
  using R = model_manifest::VoiceRole;
  if (ctx.tower_only)
    return R::Tower;
  switch (ctx.frequency_type) {
  case FT::ATIS:
    return R::Atis;
  case FT::DELIVERY:
  case FT::GROUND:
    return R::Ground;
  case FT::TOWER:
    return R::Tower;
  case FT::APPROACH:
  case FT::DEPARTURE:
  case FT::UNICOM:
  case FT::CTAF:
  case FT::UNKNOWN:
    return R::Center;
  }
  return R::Center;
}

// 5-letter all-uppercase tokens are ICAO waypoint fixes (e.g. "BULOL", "ROMAM").
// espeak-ng spells out all-caps sequences letter-by-letter; lowercasing forces
// phonetic pronunciation, which is how controllers pronounce 5-letter fixes.
// 2-4 letter uppercase codes (VOR/NDB, ILS, etc.) are left as-is — those are
// correctly spelled out letter-by-letter in real ATC phraseology.
static std::string expand_navfix_names(std::string s) {
  size_t i = 0;
  while (i < s.size()) {
    if (!std::isupper(static_cast<unsigned char>(s[i]))) { ++i; continue; }
    size_t j = i;
    while (j < s.size() && std::isupper(static_cast<unsigned char>(s[j]))) ++j;
    bool left_ok  = (i == 0) || !std::isalnum(static_cast<unsigned char>(s[i - 1]));
    bool right_ok = (j >= s.size()) || !std::isalnum(static_cast<unsigned char>(s[j]));
    if (left_ok && right_ok && (j - i) == 5) {
      for (size_t k = i; k < j; ++k)
        s[k] = static_cast<char>(std::tolower(static_cast<unsigned char>(s[k])));
    }
    i = (j > i) ? j : i + 1;
  }
  return s;
}

// Spell a runway designator into its spoken word form for the STT bias:
// "22L" -> "two two left", "04L" -> "zero four left", "09" -> "zero nine".
// Pilots SAY the words, and Voxtral mishears the number words ("two two") as
// the function word "to" ("to to left"); biasing the spoken form anchors the
// correct homophone. Used ALONGSIDE the digit form ("runway 22L") -- Voxtral
// still outputs digits, but the word bias fixes the "to to left" slip (LFMN
// 22L 2026-07-12).
static std::string spell_runway(const std::string &rwy) {
  static const char *digit[] = {"zero", "one", "two",   "three", "four",
                                "five", "six", "seven", "eight", "nine"};
  std::string out;
  auto add = [&](const char *w) {
    if (!out.empty()) out += ' ';
    out += w;
  };
  for (char c : rwy) {
    if (c >= '0' && c <= '9') add(digit[c - '0']);
    else if (c == 'L' || c == 'l') add("left");
    else if (c == 'R' || c == 'r') add("right");
    else if (c == 'C' || c == 'c') add("center");
  }
  return out;
}

// Runway designator expansion: "runway 04L" -> "runway 04 Left", "22R" -> "22 Right",
// "12C" -> "12 Center". Only expands after "runway " so waypoint idents
// (e.g. ABDI8R, FN04A) are never affected.
static std::string expand_runways(std::string s) {
  const std::string kw = "runway ";
  size_t pos = 0;
  while ((pos = s.find(kw, pos)) != std::string::npos) {
    size_t j = pos + kw.size();
    while (j < s.size() && std::isdigit(static_cast<unsigned char>(s[j])))
      ++j;
    if (j > pos + kw.size() && j < s.size()) {
      char c = static_cast<char>(
          std::toupper(static_cast<unsigned char>(s[j])));
      bool word_end = (j + 1 >= s.size()) ||
                      !std::isalnum(static_cast<unsigned char>(s[j + 1]));
      if (word_end) {
        if (c == 'L') { s.replace(j, 1, " Left");   pos = j + 5; continue; }
        if (c == 'R') { s.replace(j, 1, " Right");  pos = j + 6; continue; }
        if (c == 'C') { s.replace(j, 1, " Center"); pos = j + 7; continue; }
      }
    }
    pos += kw.size();
  }
  return s;
}

// Flight-level digit expansion for TTS. ICAO Annex 10 / Doc 4444 (and
// EUROCONTROL, UK CAP413, FAA identically): flight levels are spoken
// DIGIT BY DIGIT — FL210 = "flight level two one zero", never the cardinal
// "two hundred ten" that TTS produces from the numeral "210". Also
// normalises the "FL210" / "FL 210" abbreviation into the spoken form.
// Altitudes in feet are deliberately left as cardinals ("two thousand five
// hundred feet") per ICAO, so this only rewrites the flight-level phrase.
// Applied to the SPOKEN text only; the transcript keeps the compact "210".
static std::string spell_digits(const std::string &num) {
  static const char *kDigit[] = {"zero", "one", "two",   "three", "four",
                                  "five", "six", "seven", "eight", "nine"};
  std::string out;
  for (char c : num) {
    if (!std::isdigit(static_cast<unsigned char>(c)))
      continue;
    if (!out.empty())
      out += ' ';
    out += kDigit[c - '0'];
  }
  return out;
}

// Spell a speed in CARDINAL words the way ATC says it: 210 -> "two hundred
// ten", 250 -> "two hundred fifty", 200 -> "two hundred". Speed is NOT spoken
// digit-by-digit (unlike squawk/QNH/altitude); the controller and pilot both
// use the grouped form, so the STT bias needs the cardinal words to match the
// read-back (user 2026-07-25). See coding_atc_number_spelling.
static std::string spell_cardinal_speed(int kt) {
  static const char *kOnes[] = {"zero", "one",  "two", "three", "four",
                                "five", "six",  "seven", "eight", "nine"};
  static const char *kTeens[] = {"ten",      "eleven",  "twelve",   "thirteen",
                                 "fourteen", "fifteen", "sixteen",  "seventeen",
                                 "eighteen", "nineteen"};
  static const char *kTens[] = {"",      "",      "twenty",  "thirty",
                                "forty", "fifty", "sixty",   "seventy",
                                "eighty", "ninety"};
  if (kt <= 0)
    return {};
  const int h = kt / 100, r = kt % 100;
  std::string s;
  if (h > 0)
    s = std::string(kOnes[h]) + " hundred";
  if (r > 0) {
    std::string rem;
    if (r < 10)
      rem = kOnes[r];
    else if (r < 20)
      rem = kTeens[r - 10];
    else {
      rem = kTens[r / 10];
      if (r % 10)
        rem += std::string(" ") + kOnes[r % 10];
    }
    s = s.empty() ? rem : s + " " + rem;
  }
  return s;
}

// Spell a frequency digit-by-digit the way ATC says it, with "decimal" for the
// dot (ICAO/EU): "120.230" -> "one two zero decimal two three zero",
// "121.205" -> "one two one decimal two zero five". The pilot reads the handoff
// frequency back this way, so the STT bias needs the spelled form alongside the
// compact "120.230" (user 2026-07-25). See coding_atc_number_spelling.
static std::string spell_freq(const std::string &mhz) {
  const auto dot = mhz.find('.');
  if (dot == std::string::npos)
    return spell_digits(mhz);
  const std::string whole = spell_digits(mhz.substr(0, dot));
  const std::string frac = spell_digits(mhz.substr(dot + 1));
  if (whole.empty() && frac.empty())
    return {};
  std::string s = whole;
  s += (s.empty() ? "" : " ") + std::string("decimal");
  if (!frac.empty())
    s += " " + frac;
  return s;
}

static std::string expand_flight_levels(std::string s) {
  auto lc = [](char c) {
    return static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  };
  static const std::string kFl = "flight level ";
  std::string out;
  out.reserve(s.size() + 24);
  size_t i = 0;
  while (i < s.size()) {
    bool matched = false;

    // Pattern A: "flight level <digits>" (case-insensitive phrase).
    if (i + kFl.size() <= s.size()) {
      bool eq = true;
      for (size_t k = 0; k < kFl.size(); ++k)
        if (lc(s[i + k]) != kFl[k]) { eq = false; break; }
      if (eq) {
        size_t d = i + kFl.size(), e = d;
        while (e < s.size() && std::isdigit(static_cast<unsigned char>(s[e])))
          ++e;
        if (e > d) {
          out += s.substr(i, kFl.size());      // keep original "flight level "
          out += spell_digits(s.substr(d, e - d));
          i = e;
          matched = true;
        }
      }
    }

    // Pattern B: "FL<digits>" / "FL <digits>" abbreviation at a word start.
    if (!matched && (s[i] == 'F' || s[i] == 'f') && i + 1 < s.size() &&
        (s[i + 1] == 'L' || s[i + 1] == 'l')) {
      bool left_ok =
          (i == 0) || !std::isalnum(static_cast<unsigned char>(s[i - 1]));
      if (left_ok) {
        size_t d = i + 2;
        if (d < s.size() && s[d] == ' ')
          ++d;
        size_t e = d;
        while (e < s.size() && std::isdigit(static_cast<unsigned char>(s[e])))
          ++e;
        if (e > d) {
          out += "flight level ";
          out += spell_digits(s.substr(d, e - d));
          i = e;
          matched = true;
        }
      }
    }

    if (!matched) {
      out += s[i];
      ++i;
    }
  }
  return out;
}

// Spell the transponder code after "squawk" digit-by-digit for TTS ("squawk 2370"
// -> "squawk two three seven zero", "verify squawk 3412 mode Charlie" -> "... three
// four one two ..."), the ICAO way. The FL pass already spells flight levels, but a
// squawk is bare digits with no keyword it recognises, so it went to the TTS engine
// as a raw number ("twenty-three seventy"). Case-insensitive on the keyword; only a
// run of digits immediately after it is spelled. (user 2026-07-27) [C. P. Potter]
static std::string expand_squawk(std::string s) {
  auto lc = [](char c) {
    return static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  };
  static const std::string kKey = "squawk";
  std::string out;
  out.reserve(s.size() + 16);
  size_t i = 0;
  while (i < s.size()) {
    bool matched = false;
    bool left_ok =
        (i == 0) || !std::isalnum(static_cast<unsigned char>(s[i - 1]));
    if (left_ok && i + kKey.size() <= s.size()) {
      bool eq = true;
      for (size_t k = 0; k < kKey.size(); ++k)
        if (lc(s[i + k]) != kKey[k]) { eq = false; break; }
      if (eq) {
        size_t sp = i + kKey.size();
        while (sp < s.size() && s[sp] == ' ')
          ++sp;
        size_t e = sp;
        while (e < s.size() && std::isdigit(static_cast<unsigned char>(s[e])))
          ++e;
        if (e > sp) {
          out += s.substr(i, kKey.size()); // keep "squawk" as spoken
          out += ' ';
          out += spell_digits(s.substr(sp, e - sp));
          i = e;
          matched = true;
        }
      }
    }
    if (!matched) {
      out += s[i];
      ++i;
    }
  }
  return out;
}

// Speak ATC response via local TTS, then transition to PLAYING → IDLE.
// `length_scale` > 1.0 makes Piper speak slower (used for ATIS).
// `on_playback_starting` (optional) fires on the main thread the moment
// audio is about to play — only after a successful synthesis. Used by
// the ATIS path to delay the transcript line until the user actually
// hears the broadcast, so a silent TTS failure does not leave a ghost
// entry in the history.
// Record substantive tower speech as the REQUEST_REPEAT ("say again") replay
// target. IFR clearances (check-in ack, "cleared ... approach", descents) are
// generated engine-side and spoken directly, bypassing atc_state_machine::
// process (the only other writer of last_tower_response_text_), so this is the
// only place they get captured. Skips ATIS broadcasts and corrective /
// reminder / repeat-reply lines -- a pilot who says "say again" wants the last
// real clearance, not the "negative" correction or the "no previous clearance"
// reply (LFLP->LFMN 2026-07-12: "say again" returned "nothing to repeat"
// because IFR clearances were never recorded).
static void record_repeatable_response(const std::string &text,
                                       model_manifest::VoiceRole role) {
  if (text.empty() || role == model_manifest::VoiceRole::Atis)
    return;
  auto has = [&](const char *s) { return text.find(s) != std::string::npos; };
  if (has("negative") || has("say again") || has("no previous clearance"))
    return;
  atc_state_machine::set_last_tower_response(text);
}

static void
speak_response(const std::string &text, model_manifest::VoiceRole role,
               float length_scale = 1.0f, int com_override = 0,
               std::function<void()> on_playback_starting = nullptr) {
  record_repeatable_response(text, role);
  state_ = PTTState::PLAYING;
  tts_pending_ = true;
  ++total_inferences_; // TTS inference

  std::string final_text = expand_navfix_names(expand_runways(expand_flight_levels(expand_squawk(text))));

  backends::tts::synthesize_async(
      final_text, role, length_scale,
      [com_override, on_playback_starting = std::move(on_playback_starting)](
          backends::tts::Audio audio, bool success) {
        tts_pending_ = false;
        if (success && !audio.pcm16.empty()) {
          if (settings::debug_logging()) {
            char dbg[160];
            std::snprintf(dbg, sizeof(dbg),
                          "[xp_wellys_atc][DEBUG] TTS produced %zu samples "
                          "@ %u Hz\n",
                          audio.pcm16.size(), audio.sample_rate_hz);
            XPLMDebugString(dbg);
          }
          if (on_playback_starting)
            on_playback_starting();
          int com = com_override > 0 ? com_override : settings::active_com();
          audio_player::play_pcm_on_com(com, std::move(audio.pcm16),
                                        audio.sample_rate_hz, audio.channels,
                                        settings::volume());
        } else {
          XPLMDebugString(
              "[xp_wellys_atc][ERROR] TTS failed, skipping playback\n");
          state_ = PTTState::IDLE;
        }
      });
}

// Speak a tower response with the state-revert guard active. Used for
// engine output that committed a semantic state transition — if the
// TTS playback fails, the pilot never heard the clearance, so the
// state must be rolled back (or, when a later mutation makes the
// rollback unsafe, the clearance text must remain available for
// REQUEST_REPEAT replay).
//
//   pre_snap     = atc_state_machine::capture_snapshot() taken BEFORE
//                  the process() call that produced `text`.
//   expected_gen = atc_state_machine::current_gen() taken IMMEDIATELY
//                  AFTER that process() call.
//
// On success: nothing else happens — the response plays, state stays.
// On TTS failure: a squelch burst is played on the active COM, then
//   either (a) restore — state rolled back, system entry suggests
//   re-issuing the request, OR (b) stale — a later mutation already
//   bumped gen past expected_gen, the unsent clearance text still
//   lives in last_tower_response_text_, system entry steers the pilot
//   toward "say again" so REQUEST_REPEAT replays it.
static void speak_response_guarded(const std::string &text,
                                   model_manifest::VoiceRole role,
                                   float length_scale,
                                   atc_state_machine::AtcStateSnapshot pre_snap,
                                   uint64_t expected_gen) {
  record_repeatable_response(text, role);
  state_ = PTTState::PLAYING;
  tts_pending_ = true;
  ++total_inferences_;

  std::string final_text = expand_navfix_names(expand_runways(expand_flight_levels(expand_squawk(text))));

  backends::tts::synthesize_async(
      final_text, role, length_scale,
      [pre_snap = std::move(pre_snap), expected_gen](backends::tts::Audio audio,
                                                     bool success) mutable {
        tts_pending_ = false;
        if (success && !audio.pcm16.empty()) {
          if (settings::debug_logging()) {
            char dbg[160];
            std::snprintf(dbg, sizeof(dbg),
                          "[xp_wellys_atc][DEBUG] TTS produced %zu samples "
                          "@ %u Hz\n",
                          audio.pcm16.size(), audio.sample_rate_hz);
            XPLMDebugString(dbg);
          }
          int com = settings::active_com();
          audio_player::play_pcm_on_com(com, std::move(audio.pcm16),
                                        audio.sample_rate_hz, audio.channels,
                                        settings::volume());
          return;
        }
        // TTS failed — engage the revert guard.
        XPLMDebugString("[xp_wellys_atc][ERROR] TTS failed, applying revert "
                        "guard (squelch + state check)\n");
        const int com = settings::active_com();
        audio_player::play_squelch_burst(com);

        const bool restored =
            atc_state_machine::restore_snapshot_if_gen(pre_snap, expected_gen);
        const char *sys_text =
            restored ? "Radio failure - please repeat your transmission"
                     : "Radio failure - say 'say again' for the missed "
                       "instruction";
        push_transcript(TranscriptEntry{
            static_cast<double>(XPLMGetElapsedTime()),
            TranscriptKind::System,
            sys_text,
            "",
            "",
        });
        state_ = PTTState::IDLE;
      });
}

// Shared "got a pilot transcript, run it through the engine and speak the
// reply" path. Used by:
//   - the STT callback in on_ptt_released() — voice path; quality comes
//     from the STT result and may be < 0.3 (triggers the engine's
//     "say again" branch without writing a pilot transcript row).
//   - submit_text() — Debug-Texteingabe; quality is always 1.0.
//
// Pre-conditions: state_ == PTTState::PROCESSING. ++total_transcriptions_
// has been incremented by the caller.
static void dispatch_pilot_transcript(const std::string &text, float quality) {
  ++total_inferences_;

  const auto &ctx = xplane_context::get();
  float active_freq =
      (ctx.active_com == 1) ? ctx.com1_freq_mhz : ctx.com2_freq_mhz;
  char freq_str[16];
  std::snprintf(freq_str, sizeof(freq_str), "%.3f", active_freq);

  // Transcript writing is a UI concern, done here before handing off to
  // the engine. Low-quality transcripts skip the pilot row and go
  // straight to a "say again" response from the engine.
  bool is_pilot_row_written = false;
  if (quality >= 0.3f) {
    push_transcript(TranscriptEntry{
        static_cast<double>(XPLMGetElapsedTime()),
        TranscriptKind::Pilot,
        text,
        freq_str,
        "",
    });
    is_pilot_row_written = true;
  }

  std::string freq_str_copy = freq_str;

  engine::Input in{
      text,
      quality,
      &ctx,
      settings::pilot_callsign(),
      static_cast<double>(XPLMGetElapsedTime()),
  };

  // Snapshot the ATC state BEFORE process() runs so the TTS revert
  // guard can roll it back on synthesis failure. The matching
  // expected_gen is captured inside the engine callback — directly
  // after process() — so a later auto-correction between the synthesise
  // call and the TTS error reply is detected and switches us to the
  // "stale" branch (clearance text stays accessible via REQUEST_REPEAT).
  auto pre_snap = atc_state_machine::capture_snapshot();

  engine::process_transcript(
      std::move(in),
      [freq_str_copy, is_pilot_row_written,
       pre_snap = std::move(pre_snap)](const engine::Output &out) mutable {
        last_pilot_message_ = out.parsed;
        if (out.response_text.empty()) {
          state_ = PTTState::IDLE;
          return;
        }
        // expected_gen lives between process() and TTS callback —
        // captured here, the instant the engine returns.
        const uint64_t expected_gen = atc_state_machine::current_gen();
        // Quality-rejection path didn't write a pilot row — the ATC
        // "say again" still deserves a transcript entry with the active
        // frequency.
        std::string freq_for_atc =
            is_pilot_row_written ? freq_str_copy : std::string();
        push_transcript(TranscriptEntry{
            static_cast<double>(XPLMGetElapsedTime()),
            TranscriptKind::Tower,
            out.response_text,
            freq_for_atc,
            current_tower_label(),
        });
        // Role follows the frequency the pilot is currently tuned to —
        // that's the controller actually transmitting. Tying it to the
        // state machine misroutes handoff messages (Ground saying
        // "contact Tower" would speak with Tower's voice).
        const auto &c = xplane_context::get();
        auto role = role_for_frequency(c);
        speak_response_guarded(out.response_text, role, 1.0f,
                               std::move(pre_snap), expected_gen);
      });
}

void init() {
  state_ = PTTState::IDLE;
  tts_pending_ = false;
  last_duration_ = 0.0f;
  last_samples_ = 0;
  last_wav_bytes_ = 0;
  transcript_.clear();
  last_pilot_message_ = {};
  // (Re-)open the transcript log — truncate so each session starts fresh.
  if (g_transcript_log_) {
    std::fclose(g_transcript_log_);
    g_transcript_log_ = nullptr;
  }
  std::string log_path =
      model_paths::plugin_root() + "/Resources/transcript.log";
  g_transcript_log_ = std::fopen(log_path.c_str(), "w");
  if (g_transcript_log_) {
    int xp_ver = 0, xplm_ver = 0;
    XPLMHostApplicationID host = 0;
    XPLMGetVersions(&xp_ver, &xplm_ver, &host);
    const char *os =
#if defined(__linux__)
        "Linux"
#elif defined(__APPLE__)
        "macOS"
#elif defined(_WIN32)
        "Windows"
#else
        "Unknown"
#endif
        ;
    std::time_t now = std::time(nullptr);
    char ts[32] = {0};
    std::strftime(ts, sizeof(ts), "%Y-%m-%d %H:%M:%S", std::localtime(&now));
#ifdef XP_WELLYS_ATC_VERSION
    const char *plugin_ver = XP_WELLYS_ATC_VERSION;
#else
    const char *plugin_ver = "unknown";
#endif
    std::fprintf(g_transcript_log_,
                 "=== Welly's ATC v%s | OS=%s | X-Plane %d | XPLM SDK %d "
                 "| session %s ===\n",
                 plugin_ver, os, xp_ver, xplm_ver, ts);
    std::fflush(g_transcript_log_);
    logging::info("Transcript log: %s", log_path.c_str());
    g_last_stt_model_.clear(); // force header on first transcription
    write_stt_header_if_changed();
  }
  total_transcriptions_ = 0;
  total_inferences_ = 0;
  engine::reset();
  atis_playing_ = false;
  atis_active_com_ = 0;
  atis_cooldown_ = 0.0f;
  atis_tuned_timer_ = 0.0f;
}

void stop() {
  state_ = PTTState::IDLE;
  tts_pending_ = false;
  if (g_transcript_log_) {
    std::fclose(g_transcript_log_);
    g_transcript_log_ = nullptr;
  }
}

void on_ptt_pressed() {
  if (state_ != PTTState::IDLE) {
    char buf[128];
    std::snprintf(buf, sizeof(buf), "[xp_wellys_atc] PTT blocked, state=%d\n",
                  static_cast<int>(state_));
    XPLMDebugString(buf);
    return;
  }

  // Radio requires power (checks COM radio power DataRef, handles
  // avionics master, battery, and individual radio switches)
  const auto &ctx = xplane_context::get();
  if (!ctx.com_radio_powered) {
    XPLMDebugString("[xp_wellys_atc] PTT blocked - COM radio not powered\n");
    return;
  }

  // Backends must be loaded — without STT we cannot transcribe and
  // without LM we cannot reliably resolve low-confidence transcripts.
  // The plugin's startup path surfaces the model-download dialog when
  // anything is missing; this gate prevents PTT from doing nothing
  // visible.
  if (!backends::stt_ready() || !backends::lm_ready() ||
      !backends::tts_ready()) {
    XPLMDebugString("[xp_wellys_atc][ERROR] PTT blocked - local models not "
                    "loaded (open the plugin window to download)\n");
    return;
  }

  state_ = PTTState::RECORDING;
  audio_player::play_ptt_click();
  audio_recorder::start_recording();
  if (settings::debug_logging())
    XPLMDebugString("[xp_wellys_atc][DEBUG] PTT pressed\n");
}

// Called once the mic is stopped (after the tail expires). Handles the
// minimum-duration gate, takes the PCM buffer, and dispatches to STT.
static void submit_recording_to_stt() {
  last_duration_ = audio_recorder::duration_seconds();
  last_samples_ = audio_recorder::buffer_samples();

  if (last_duration_ < kMinRecordingDuration) {
    char buf[128];
    std::snprintf(buf, sizeof(buf),
                  "[xp_wellys_atc] Recording too short (%.2fs), discarding\n",
                  last_duration_);
    XPLMDebugString(buf);
    state_ = PTTState::IDLE;
    return;
  }

  std::vector<int16_t> pcm = audio_recorder::take_pcm();
  unsigned src_rate = audio_recorder::sample_rate_hz();
  last_wav_bytes_ = pcm.size() * sizeof(int16_t);

  if (settings::debug_logging()) {
    char buf[256];
    std::snprintf(buf, sizeof(buf),
                  "[xp_wellys_atc][DEBUG] Recording stopped: %.1fs, %zu "
                  "samples @ %u Hz\n",
                  last_duration_, last_samples_, src_rate);
    XPLMDebugString(buf);
  }

  state_ = PTTState::PROCESSING;

  // Build the STT initial-prompt context — biases transcription of
  // local proper nouns ("Grenchen", "Speck") AND of the pilot's own
  // phonetic callsign (long NATO sequences like "November One Two
  // Three Alpha Bravo" are otherwise a frequent mishear source —
  // problem #4 in the planning doc). The string is consumed three
  // different ways depending on backend:
  //   - whisper_stt → whisper_full_params.initial_prompt (freeform)
  //   - openai_stt  → "prompt" multipart field (freeform)
  //   - mistral_stt → context_bias[] (split on whitespace; more
  //                   tokens = more bias entries)
  // — so adding the callsign tokens here biases all three.
  // Start with the static ATC vocabulary prompt (full NATO phonetic alphabet
  // + common ATC phrases from atc_prompt_templates.json). This forms the
  // foundation for all three STT backends:
  //   - whisper_stt  → whisper_full_params.initial_prompt
  //   - openai_stt   → "prompt" multipart field
  //   - mistral_stt  → split on whitespace/commas → context_bias[] entries
  // Dynamic tokens appended below further anchor the specific flight.
  const auto &ctx_for_whisper = xplane_context::get();

  // Phase-aware STT bias: pick the phrase set matching the current phase so
  // Voxtral's context_bias[] (one entry per token) is not seeded with irrelevant
  // words -- ground/pattern phrases (taxi, holding point, downwind, ...) mid-cruise
  // pull mishears, and en-route phrases are noise on short final. En-route IFR ->
  // climb/descend/contact/direct...; approach+landing -> cleared to land / report
  // established / runway in sight / vacated; ground+VFR -> the full list (user
  // 2026-07-22). Falls back to the full "prompt" when a variant is absent.
  std::string stt_variant;
  {
    using S = atc_state_machine::ATCState;
    const auto s = atc_state_machine::get_state();
    if (s == S::IFR_RADAR_CONTACT || s == S::IFR_ENROUTE_CRUISE ||
        s == S::IFR_DESCENT || s == S::IFR_ARRIVAL)
      stt_variant = "prompt_enroute";
    else if (s == S::IFR_APPROACH_CONTACT || s == S::IFR_APPROACH_DESCENT ||
             s == S::IFR_APPROACH_TOWER || s == S::IFR_LANDING_CLEARED)
      stt_variant = "prompt_approach";
  }
  std::string airport_ctx =
      atc_templates::get_prompt("whisper_prompt", stt_variant);
  if (!airport_ctx.empty())
    airport_ctx += " ";
  // Determine whether the aircraft is airborne mid-IFR (drift-prone state)
  // so we can prefer the assigned destination ICAO over
  // ctx.nearest_airport_id — the latter drifts to whatever airfield is
  // physically closest (small strips, heliports like XLF00DH Hopital de
  // Chamonix, unrelated Approach airports like LSGG/LFLI). See
  // feedback_nearest_airport_ifr — this is the tightest scope of that
  // rule that fits in v4.3.1 without a wider refactor.
  const auto ctx_state = atc_state_machine::get_state();
  const bool airborne_ifr_drift_risk =
      ctx_state == atc_state_machine::ATCState::IFR_RADAR_CONTACT ||
      ctx_state == atc_state_machine::ATCState::IFR_ENROUTE_CRUISE ||
      ctx_state == atc_state_machine::ATCState::IFR_DESCENT ||
      ctx_state == atc_state_machine::ATCState::IFR_ARRIVAL ||
      ctx_state == atc_state_machine::ATCState::IFR_APPROACH_CONTACT ||
      ctx_state == atc_state_machine::ATCState::IFR_APPROACH_DESCENT ||
      // On final after the Tower handoff (still airborne) the full airport
      // freq list must ALSO be suppressed: LFMN has ~20 Tower/Ground freqs and
      // dumping them all floods the Voxtral context_bias with numeric tokens,
      // garbling every read-back once cleared to land ("Pirto Land", "X-Robot",
      // LFMN R22LZ 2026-07-12). Only the pending-handoff freq belongs in the
      // bias here. Also biases the live Tower label instead of the airport name.
      ctx_state == atc_state_machine::ATCState::IFR_APPROACH_TOWER ||
      ctx_state == atc_state_machine::ATCState::IFR_LANDING_CLEARED;
  if (!airborne_ifr_drift_risk) {
    airport_ctx += ctx_for_whisper.nearest_airport_id;
    if (!ctx_for_whisper.nearest_airport_name.empty())
      airport_ctx += " " + ctx_for_whisper.nearest_airport_name;
    // Facilities the PILOT ADDRESSES on the ground/departure ("Torino Ground",
    // "Torino Tower") -- the airport NAME ("Torino Caselle") is never spoken, so
    // the city + facility forms must be in the prompt too, not just the BIAS
    // (user 2026-07-26, "Torino Ground/Tower not in CTX at the stand").
    std::string city = ctx_for_whisper.nearest_airport_name;
    const auto sep = city.find_first_of(" /-");
    if (sep != std::string::npos)
      city = city.substr(0, sep);
    if (!city.empty())
      airport_ctx += " " + city + " Ground " + city + " Tower";
  }
  // Aircraft registration (e.g. "N111RC", "F-HABC") from X-Plane's acf_tailnum
  // DataRef — anchors the short-form tail number the pilot uses in radio calls.
  if (!ctx_for_whisper.aircraft_tail_number.empty())
    airport_ctx += " " + ctx_for_whisper.aircraft_tail_number;
  // Runway to anchor in the STT bias ("runway 22L", "R-NAV 22L", ... below).
  // Prefer the CIFP-assigned LANDING runway once an approach is cleared -- it
  // persists through landing + taxi-in, so "runway 22L" stays biased through
  // the vacated / ground calls (LFMN 2026-07-12: "22L" heard "to toL" after it
  // dropped from the bias post-landing). Fall back to the state machine's
  // assigned_runway() (the departure runway before any approach is assigned).
  std::string locked_rwy = engine::assigned_landing_runway();
  if (locked_rwy.empty())
    locked_rwy = atc_state_machine::assigned_runway();
  if (!locked_rwy.empty()) {
    airport_ctx += " runway " + locked_rwy;               // digit form ("22L")
    const std::string spoken = spell_runway(locked_rwy);  // spoken form
    if (!spoken.empty())
      airport_ctx += " runway " + spoken;                 // ("two two left")
  }
  // Landing-phase read-back anchors (IFR final approach / Tower / landing).
  // Voxtral garbles the landing read-back hard -- "cleared to land" -> "Climb to
  // 9", "established" / "report established" -> "Establish 1-0-4" -- because the
  // airport freq list is suppressed here and ONLY the runway is biased, so there
  // is no anchor for the actual phrases (LFLP build-77 2026-07-20). Bias the exact
  // spoken read-backs (digit + word runway forms) so Voxtral has something to lock
  // onto. Targeted (a handful of tokens), so it does not re-flood the context that
  // was deliberately thinned at these states.
  {
    const atc_state_machine::ATCState st_now = atc_state_machine::get_state();
    if (st_now == atc_state_machine::ATCState::IFR_APPROACH_DESCENT ||
        st_now == atc_state_machine::ATCState::IFR_APPROACH_TOWER ||
        st_now == atc_state_machine::ATCState::IFR_LANDING_CLEARED) {
      airport_ctx += " report established cleared to land go around";
      if (!locked_rwy.empty()) {
        airport_ctx += " cleared to land runway " + locked_rwy +
                       " established runway " + locked_rwy;
        const std::string spoken_rwy = spell_runway(locked_rwy);
        if (!spoken_rwy.empty())
          airport_ctx += " cleared to land runway " + spoken_rwy +
                         " established runway " + spoken_rwy;
      }
    }
  }
  // Prefer the locked session callsign once Tower has accepted one —
  // that's the exact phrasing the controller will use back. Before
  // the lock fires, fall back to the user's configured phonetic
  // expansion. Always also append the raw tail-number form (e.g.
  // "N123AB") so single-token utterances like "N-one-two-three" stay
  // anchored even when Whisper drops the phonetic spelling.
  const std::string &sess_cs = atc_state_machine::session_callsign();
  const std::string phonetic =
      !sess_cs.empty() ? sess_cs : settings::pilot_callsign();
  if (!phonetic.empty())
    airport_ctx += " " + phonetic;
  const std::string raw_cs = settings::pilot_callsign_raw();
  if (!raw_cs.empty())
    airport_ctx += " " + raw_cs;
  // Abbreviated callsign — last two NATO letters (ICAO Doc 4444 §5.2).
  // After the initial exchange, ATC and pilot both drop the middle digits
  // (e.g. "November One One One Romeo Charlie" -> "Romeo Charlie"). Biasing
  // the trailing pair explicitly stops Voxtral from garbling it into
  // "runway Charlie", "Romo Charlie", "on the road to Charlie", etc.
  if (!phonetic.empty()) {
    std::vector<std::string> nato_words;
    std::string cur;
    for (char c : phonetic) {
      if (c == ' ') { if (!cur.empty()) nato_words.push_back(cur); cur.clear(); }
      else cur += c;
    }
    if (!cur.empty()) nato_words.push_back(cur);
    if (nato_words.size() >= 2) {
      airport_ctx += " " + nato_words[nato_words.size() - 2] +
                     " " + nato_words[nato_words.size() - 1];
    }
    // Digit-form callsign variant, e.g. "November 750 X-Ray Papa": pilots say the
    // number block as one group ("seven-fifty") and Voxtral renders it as digits,
    // so biasing the digit form alongside the NATO-word form anchors partial
    // callsigns like "750 X-Ray Papa" / "November 750 ..." that otherwise garble
    // to "750XR" / "750X3" / "X River Park" (user 2026-07-19). Number WORDS are
    // collapsed into a contiguous digit run; the prefix + trailing letters stay.
    {
      auto nato_digit = [](const std::string &w) -> char {
        static const std::pair<const char *, char> kN[] = {
            {"zero", '0'},  {"one", '1'},  {"two", '2'},   {"three", '3'},
            {"four", '4'},  {"five", '5'}, {"six", '6'},   {"seven", '7'},
            {"eight", '8'}, {"nine", '9'}, {"niner", '9'}};
        std::string lw = w;
        for (char &c : lw)
          c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        for (const auto &p : kN)
          if (lw == p.first)
            return p.second;
        return 0;
      };
      std::string digit_form;
      for (const auto &word : nato_words) {
        const char d = nato_digit(word);
        if (d) {
          if (!digit_form.empty() &&
              std::isdigit(static_cast<unsigned char>(digit_form.back())))
            digit_form += d; // continue the digit run (Seven Five Zero -> 750)
          else {
            if (!digit_form.empty())
              digit_form += ' ';
            digit_form += d;
          }
        } else {
          if (!digit_form.empty())
            digit_form += ' ';
          digit_form += word;
        }
      }
      if (!digit_form.empty() && digit_form != phonetic)
        airport_ctx += " " + digit_form;
    }
  }
  // Arrival-ground pruning: once the aircraft has flown and is back on the
  // ground (post-landing at the destination), the enroute route content —
  // SID, STAR, and every navlog fix — is pure noise for a taxi/parking/
  // vacated call. It floods the bias with ~20+ irrelevant tokens (KUKEV,
  // BANKO, SALE3P, ...) that dilute the callsign and wreck recognition
  // (LIMF -> LFLP 2026-07-10: "X-Ray Papa" -> "rubber power", "Annecy" ->
  // "anti" on the ground). Skip the whole route block there. Departure
  // ground (was_airborne()==false) keeps it — the SID + fixes matter for
  // the clearance/taxi readback.
  const bool arrival_ground =
      ctx_for_whisper.on_ground && atc_state_machine::was_airborne();
  // SID, STAR, destination ICAO + name, and all FPL fix idents from SimBrief OFP.
  // Built fresh every PTT so the STAR name appears as soon as ATC assigns it.
  // Cap at 60 fixes to avoid inflating the prompt on long-haul routes.
  {
    const auto &ofp = simbrief_ofp::get();
    // Destination: ICAO code + apt.dat name so the pilot's readback is
    // recognised regardless of whether they say "LFSR" or "Reims-Prunay".
    // (destination stays useful even on arrival ground — it's the local field.)
    if (!ofp.destination_icao.empty())
      airport_ctx += " " + ofp.destination_icao;
    if (!ctx_for_whisper.ifr_destination.empty())
      airport_ctx += " " + ctx_for_whisper.ifr_destination;
    if (!arrival_ground) {
      if (!ofp.sid_name.empty())
        airport_ctx += " " + ofp.sid_name;
      const std::string &star = engine::assigned_star_name();
      if (!star.empty())
        airport_ctx += " " + star;
      // Spoken plain-language STAR ("SALEV THREE PAPA"): ATC speaks this form
      // (alpha-64) so the pilot reads it back that way -- bias it or the coded
      // form alone makes Voxtral hear "side of 3 Papa" (user 2026-07-19).
      const std::string star_spoken = engine::assigned_star_spoken();
      if (!star_spoken.empty() && star_spoken != star) {
        airport_ctx += " " + star_spoken;
        // Digit-form variant too ("SALEV 3 Papa"): Voxtral renders the validity
        // number as a DIGIT, so bias the digit form alongside the word form
        // ("SALEV THREE PAPA") to anchor the readback (user 2026-07-19). Convert
        // the single number WORD in the spoken designator to a digit.
        static const std::pair<const char *, char> kW2D[] = {
            {"Zero", '0'},  {"One", '1'},  {"Two", '2'},   {"Three", '3'},
            {"Four", '4'},  {"Five", '5'}, {"Six", '6'},   {"Seven", '7'},
            {"Eight", '8'}, {"Nine", '9'}};
        std::string digit_form;
        std::string word;
        auto flush = [&](const std::string &w) {
          char d = 0;
          for (const auto &p : kW2D)
            if (w == p.first) { d = p.second; break; }
          if (!digit_form.empty())
            digit_form += ' ';
          if (d)
            digit_form += d;
          else
            digit_form += w;
        };
        for (char c : star_spoken) {
          if (c == ' ') { if (!word.empty()) { flush(word); word.clear(); } }
          else word += c;
        }
        if (!word.empty())
          flush(word);
        if (!digit_form.empty() && digit_form != star_spoken)
          airport_ctx += " " + digit_form;
      }
      // Spoken approach identity ("RNAV Zulu approach runway 04"): ATC speaks the
      // NATO variant word so bias it or the readback garbles ("Zulu" -> "zero",
      // "04" -> "zero for"; user 2026-07-19).
      const std::string appr_spoken =
          engine::assigned_approach_spoken(ctx_for_whisper);
      if (!appr_spoken.empty())
        airport_ctx += " " + appr_spoken;
      // Callsign-dilution guard (user 2026-07-19: N750XP is recognised well in
      // DEPARTURE / ENROUTE but worse in ARRIVAL / APPROACH). Once in the terminal
      // arrival/approach phase the whole ENROUTE navlog is behind the aircraft --
      // ~60 stale fix tokens that dilute the callsign and the live approach vocab.
      // Drop them there and rely on the tracker-FORWARD upcoming fixes (STAR +
      // approach) below, which are the actual readback vocabulary. Cruise/descent
      // keeps the full navlog (upcoming enroute fixes still matter there).
      const auto st = atc_state_machine::get_state();
      using AS = atc_state_machine::ATCState;
      const auto upcoming = engine::upcoming_route_fix_idents();
      const bool prune_navlog =
          !upcoming.empty() &&
          (st == AS::IFR_ARRIVAL || st == AS::IFR_APPROACH_CONTACT ||
           st == AS::IFR_APPROACH_DESCENT || st == AS::IFR_APPROACH_TOWER ||
           st == AS::IFR_LANDING_CLEARED);
      int fix_count = 0;
      if (!prune_navlog) {
        for (const auto &fix : ofp.navlog) {
          if (!fix.ident.empty() && fix_count < 60) {
            airport_ctx += " " + fix.ident;
            ++fix_count;
          }
        }
      }
      // CIFP STAR + approach procedure waypoints from the engine's route table.
      // These are NOT in the filed navlog (SimBrief lists the enroute FPL, not
      // the STAR/APPCH fixes), so "direct <STAR fix>" readbacks garble without
      // them (AMFOU -> "I'm full", LFMN ABDI8R 2026-07-13). Deduped against what
      // the navlog already added. Only the upcoming fixes (tracker-forward).
      for (const auto &id : upcoming) {
        if ((" " + airport_ctx + " ").find(" " + id + " ") == std::string::npos) {
          airport_ctx += " " + id;
          if (++fix_count >= 90)
            break;
        }
      }
    }
    // Destination arrival controller (Approach or Information/FIS).
    // Try TRACON first (proper Approach); fall back to CTR which is how
    // XP12 atc.dat encodes FIS/Information services (e.g. "REIMS" for LFSR).
    // Skipped on arrival ground — the Approach controller (Geneva/Chambery
    // for LFLP) is stale once landed; you're on Ground/Tower now.
    if (!arrival_ground && !ofp.destination_icao.empty()) {
      const auto dest_pos =
          xplane_context::airport_pos_for(ofp.destination_icao);
      if (dest_pos.first != 0.0 || dest_pos.second != 0.0) {
        const airspace_db::Controller *arr_ctrl =
            airspace_db::find_by_role_near(airspace_db::ControllerRole::TRACON,
                                           dest_pos.first, dest_pos.second, 0);
        if (!arr_ctrl)
          arr_ctrl = airspace_db::find_by_role_near(
              airspace_db::ControllerRole::CTR, dest_pos.first,
              dest_pos.second, 0);
        if (arr_ctrl && !arr_ctrl->name.empty()) {
          // Raw atc.dat name tokens (e.g. "REIMS" "INFORMATION") — case-insensitive coverage.
          std::istringstream iss(arr_ctrl->name);
          std::string tok;
          while (iss >> tok)
            airport_ctx += " " + tok;
          // Human-readable city from the controller's facility airport (e.g. LFSM → "Reims"),
          // pre-loaded before the handoff fires so the readback is biased from the first PTT.
          if (!arr_ctrl->facility_id.empty()) {
            const std::string apt =
                xplane_context::airport_name_for(arr_ctrl->facility_id);
            if (!apt.empty()) {
              // Stop at a space OR '/' so "Nice/Cote d'Azur" -> "Nice".
              auto sp = apt.find_first_of(" /");
              std::string city =
                  (sp == std::string::npos) ? apt : apt.substr(0, sp);
              if (!city.empty()) {
                city[0] = static_cast<char>(
                    std::toupper(static_cast<unsigned char>(city[0])));
                for (std::size_t i = 1; i < city.size(); ++i)
                  city[i] = static_cast<char>(
                      std::tolower(static_cast<unsigned char>(city[i])));
                airport_ctx += " " + city;
              }
            }
          }
        }
      }
    }
  }

  // Live ACC/Approach controller the pilot is actually talking to (Milan,
  // France, Marseille, Geneva, Chambery...) during airborne IFR. The static
  // arrival-controller guess above can differ from the live handoff target, so
  // bias the ACTUAL current label -- otherwise Voxtral mis-transcribes the
  // controller name in check-ins ("Genieball" for "Geneva"; LIMF -> LFLP
  // 2026-07-11).
  if (airborne_ifr_drift_risk) {
    // Both the LIVE controller and the PENDING one (the target of an
    // outstanding "contact X" handoff). The readback of "contact Geneva" is
    // biased by the PENDING label -- without it Voxtral garbled Geneva to
    // "Jimmy Van" while only Marseille (live) was in context (LIMF -> LFLP
    // 2026-07-11).
    for (const std::string &lbl : {engine::current_controller_label(),
                                   engine::pending_controller_label()}) {
      if (lbl.empty())
        continue;
      std::istringstream iss(lbl);
      std::string tok;
      while (iss >> tok)
        airport_ctx += " " + tok;
    }
  }

  // Add departure controller name so Voxtral recognises French/German city
  // names in pilot readbacks (e.g. "Chambery" → not "chamber",
  // "Marseille" → correct, "Strasbourg" → not "Strasbourg Control").
  // Use the pending label (set at takeoff clearance time) if the active
  // label has not yet been activated by poll_departure_handoff().
  // Guard: skip during arrival/approach phase — the departure label is
  // irrelevant late-flight noise (LIMF -> LFLP retest: "Torino" was
  // still being added to the CTX during Chambéry Approach handoff). Also
  // skip on arrival ground (post-landing), where the departure controller
  // is doubly stale.
  if (!arrival_ground &&
      (!airborne_ifr_drift_risk ||
       ctx_state == atc_state_machine::ATCState::IFR_RADAR_CONTACT)) {
    const std::string &dep_label = engine::current_controller_label().empty()
                                       ? engine::pending_departure_label()
                                       : engine::current_controller_label();
    if (!dep_label.empty()) {
      std::istringstream iss(dep_label);
      std::string tok;
      while (iss >> tok)
        airport_ctx += " " + tok;
    }
  }

  // Add the pending handoff frequency and known airport frequencies as
  // numeric tokens (e.g. "120.230").  When these are in context_bias, Voxtral
  // outputs them as digits directly ("120.230") rather than phonetic words
  // ("one to zero decimal two three zero"), making frequency readbacks reliable.
  // Guard: during airborne IFR the airport_freqs.all iteration dumps every
  // freq of whatever airport nearest_airport_id currently points at — for
  // large TRACON airports like LSGG that's 14+ freqs (Ground / Tower /
  // Approach East / Approach West / Delivery / ATIS / Departure / ...).
  // Voxtral treats them all equally and gets confused about which is the
  // active working freq. Only add the pending-handoff freq (the one the
  // plugin just told the pilot to switch to) during airborne states —
  // that's the ONLY freq the pilot is about to say. Full apt.dat freq
  // list only during ground / initial-clearance / taxi phases where
  // multi-freq context is genuinely useful.
  {
    char freq_buf[16];
    if (!airborne_ifr_drift_risk) {
      // Dedupe: apt.dat lists the same freq under several role codes, so the
      // raw list repeats (LFMN dumped 118.700 five times). Duplicate numeric
      // tokens add nothing but bias-flood risk for Voxtral.
      std::unordered_set<uint32_t> seen_khz;
      for (const auto &af : ctx_for_whisper.airport_freqs.all) {
        if (af.freq_khz > 0 && seen_khz.insert(af.freq_khz).second) {
          std::snprintf(freq_buf, sizeof(freq_buf), "%.3f",
                        static_cast<float>(af.freq_khz) / 1000.0f);
          airport_ctx += " ";
          airport_ctx += freq_buf;
        }
      }
    }
    const float ph = engine::pending_handoff_freq();
    if (ph > 0.0f) {
      std::snprintf(freq_buf, sizeof(freq_buf), "%.3f", ph);
      airport_ctx += " ";
      airport_ctx += freq_buf;
    }
  }

  // Extra individual words not in the static whisper_prompt template —
  // these are the specific Voxtral/WhisperATC mishearing hotspots
  // identified in flight testing (see project_voxtral_stt_errors.md).
  // Each token becomes a separate context_bias[] entry for Voxtral and
  // extends the initial_prompt for whisper.cpp / OpenAI.
  // Acronym separations: pilots pronounce "R-NAV" as two syllables (or
  // "AR-NAV"). Voxtral often outputs "arm of / arnold / armature" when the
  // single-token "RNAV" fails to match. We add TWO forms:
  //   - "R-NAV" (hyphenated) — single Mistral context_bias entry that
  //     matches ^[^,\s]+$; teaches Voxtral the two-syllable spelling.
  //   - "R NAV" (spaced) — for whisper.cpp / OpenAI freeform initial_prompt,
  //     which does use adjacent tokens as a bigram hint. Mistral splits
  //     this into "R" and "NAV" separately (harmless — short tokens have
  //     low bias weight; the "R-NAV" entry is the useful one).
  // The kPhraseAliases pass converts either output form back to "rnav".
  airport_ctx +=
      " Romeo IFR QNH Mode Charlie holding point squawk vacated"
      " taxi RNAV R-NAV R NAV report ground tower wilco roger startup"
      " climb climbing departure ready"
      " filed maintain cleared descend identified request"
      " contact frequency approach control radar information centre"
      // "flight level" biased explicitly: Voxtral mishears it as "flat level"
      // in readbacks (LIMF -> LFLP 2026-07-11 "flat level 65"). Both the phrase
      // and the standalone word anchor it.
      " flight level flight"
      // Speed-restriction phrase WITHOUT a hardcoded value: a literal number here
      // (was "250") competes with the ACTIVE limit added dynamically below and
      // pulls Voxtral toward the wrong digits -- "reduce speed 210" was read back
      // as "200 10" while the prompt still shouted "250" (LFLP 2026-07-18). Keep
      // only the phraseology words; the real number is anchored at the dynamic
      // speed bias further down.
      " reduce speed knots or less";
  // Approach + runway bigram: after Approach clears the aircraft, the pilot
  // will say "R-NAV 25" or "RNAV 25" (or ILS/RNP/etc + rwy). Voxtral often
  // mishears "RNAV 25" as "RNAV to 5" — the "2" becomes "to". Biasing the
  // exact "R-NAV NN" phrase anchors the digit pair as a unit. Uses the
  // CIFP-assigned landing runway (set at Approach check-in via
  // set_assigned_runway) rather than ctx.active_runway (which can be
  // wind-favoured for the opposite end).
  if (!locked_rwy.empty()) {
    // "R-NAV NN" and "RNAV NN" — anchor the digit pair to the assigned
    // runway. Without this, Voxtral turns "RNAV 25" into "RNAV to 5"
    // (the "2" becomes "to"). "runway NN" is already added at line ~622.
    airport_ctx += " R-NAV " + locked_rwy;
    airport_ctx += " RNAV "  + locked_rwy;
  }
  airport_ctx += " cancelling cancel cancellation";

  // ATC-assigned altitude / FL — Voxtral's number tokenizer struggles with
  // round-thousand altitudes and can emit "2015" for spoken "two thousand".
  // Adding the exact spoken phrasing of the currently-cleared altitude
  // strongly anchors the pilot's next readback digit token.
  // See project_voxtral_stt_errors.md ("2000 feet" -> "2015" row).
  const int cleared_alt_ft = engine::current_cleared_alt_ft();
  if (cleared_alt_ft > 0) {
    const int ta_ft =
        ctx_for_whisper.transition_alt_ft > 0 ? ctx_for_whisper.transition_alt_ft
                                              : 5000;
    if (cleared_alt_ft >= ta_ft) {
      // FL notation — pilots can say either "flight level NNN" or "FL NNN"
      // ("F L two five zero"). Bias both forms since Voxtral produces
      // different tokens for each.
      const int fl = cleared_alt_ft / 100;
      airport_ctx += " flight level " + std::to_string(fl);
      airport_ctx += " FL " + std::to_string(fl);
    } else {
      // Feet notation — the digit form is what Voxtral must emit correctly.
      airport_ctx += " " + std::to_string(cleared_alt_ft) + " feet";
    }
  }

  // Speed-restriction context bias (analog of the altitude bias above): speak the
  // ACTIVE speed limit so Voxtral anchors the digits in the pilot's readback --
  // "reduce speed 210 knots" was misheard as "110 knots", failing readback
  // verification and looping (LFLP 2026-07-17). Only when a restriction is in force.
  const int cur_spd_kt = engine::current_speed_restriction_kt();
  if (cur_spd_kt > 0) {
    const std::string kt = std::to_string(cur_spd_kt);
    // Anchor the value inside the full phrase AND standalone, so both the whole
    // read-back ("reduce speed 210 knots or less") and a terse "210 knots" match.
    airport_ctx += " reduce speed " + kt + " knots or less " + kt + " knots";
    // Also bias the SPOKEN word forms ("two hundred ten" / "two hundred and ten"):
    // Voxtral garbles the bare digits ("210" -> "to 110"), and a word anchor gives it
    // the phrase to reach for (LFLP 2026-07-18). Only when there is a tens remainder
    // (a round "two hundred" is already covered by the digit form). The readback
    // verifier parses these forms back to the value (extract_speed compound rule).
    const int h = cur_spd_kt / 100, r = cur_spd_kt % 100;
    if (h > 0 && r > 0) {
      static const char *kOnes[] = {"zero", "one",   "two",   "three", "four",
                                    "five", "six",   "seven", "eight", "nine"};
      static const char *kTeens[] = {"ten",      "eleven",  "twelve",
                                     "thirteen", "fourteen", "fifteen",
                                     "sixteen",  "seventeen", "eighteen",
                                     "nineteen"};
      static const char *kTens[] = {"",      "",      "twenty",  "thirty",
                                    "forty", "fifty", "sixty",   "seventy",
                                    "eighty", "ninety"};
      std::string rem;
      if (r >= 10 && r < 20)
        rem = kTeens[r - 10];
      else if (r >= 20) {
        rem = kTens[r / 10];
        if (r % 10)
          rem += std::string(" ") + kOnes[r % 10];
      } else
        rem = kOnes[r];
      const std::string base = std::string(kOnes[h]) + " hundred";
      airport_ctx += " " + base + " " + rem + " knots";
      airport_ctx += " " + base + " and " + rem + " knots";
    }
  }

  // ── Voxtral context_bias: curated, READBACK-FIRST list of <=100 phrases ──
  // Rebuilt every PTT (like airport_ctx). Unlike the freeform prompt above,
  // Mistral Voxtral's context_bias is an ARRAY of <=100 whole-word OR multi-word
  // strings -- it must NOT be whitespace-split. Order = what the pilot is most
  // likely to READ BACK from the LAST ATC call (assigned level, freq, runway,
  // controller) first, then the callsign, then route proper nouns, then core
  // vocab phrases. split_context() (mistral_stt) dedupes + caps 100, so overflow
  // drops the least-critical generic vocab, never the readback anchors.
  std::string context_bias;
  {
    std::vector<std::string> bias;
    auto add = [&bias](const std::string &p) {
      if (p.empty())
        return;
      for (const auto &e : bias)
        if (e == p)
          return;
      if (bias.size() < 100)
        bias.push_back(p);
    };
    // 1) Readback anchors from the last ATC transmission.
    if (cleared_alt_ft > 0) {
      const int ta = ctx_for_whisper.transition_alt_ft > 0
                         ? ctx_for_whisper.transition_alt_ft
                         : 5000;
      if (cleared_alt_ft >= ta) {
        const std::string fl = std::to_string(cleared_alt_ft / 100);
        add("flight level " + fl);
        add("FL " + fl);
      } else {
        add(std::to_string(cleared_alt_ft) + " feet");
      }
    }
    if (cur_spd_kt > 0) {
      add("reduce speed " + std::to_string(cur_spd_kt) + " knots"); // compact
      const std::string card = spell_cardinal_speed(cur_spd_kt);
      if (!card.empty())
        add("reduce speed " + card + " knots"); // cardinal (as ATC speaks it)
    }
    // Pending-handoff frequency -- the pilot reads it back digit-by-digit ("one
    // two zero decimal two three zero"), so bias the SPELLED form plus compact.
    // ONLY when it is a NEW freq to switch TO (differs from the current COM): once
    // the pilot has checked in, pending_handoff_freq() still returns that freq, and
    // keeping it in the bias made Voxtral SUBSTITUTE it for an unrelated number --
    // "climb flight level 140" read back as "flight level 125.630" (the freq!)
    // because 125.630 was a strong competing number anchor (user 2026-07-26).
    {
      const float ph = engine::pending_handoff_freq();
      const float acom =
          (ctx_for_whisper.active_com == 2) ? ctx_for_whisper.com2_freq_mhz
                                            : ctx_for_whisper.com1_freq_mhz;
      if (ph > 100.0f && std::fabs(ph - acom) >= 0.010f) {
        char fb[16];
        std::snprintf(fb, sizeof(fb), "%.3f", ph);
        add(spell_freq(fb));
        add(fb);
      }
    }
    // Squawk (IFR clearance read-back anchor -- a 4-digit garble hotspot). ATC
    // SPELLS the code digit-by-digit ("squawk six zero seven one") and the pilot
    // reads it back the same way, so bias the SPELLED form -- plus the compact
    // digit form (Voxtral may emit either), mirroring the runway digit+word pattern.
    {
      const std::string &sq = atc_state_machine::session_squawk();
      if (!sq.empty()) {
        add("squawk " + spell_digits(sq));
        add("squawk " + sq);
      }
    }
    // QNH value -- also spelled digit-by-digit ("QNH one zero one zero"); the bare
    // digits garble to "QLH". Bias spelled + compact forms.
    if (ctx_for_whisper.qnh_hpa > 0) {
      const std::string q = std::to_string(ctx_for_whisper.qnh_hpa);
      add("QNH " + spell_digits(q));
      add("QNH " + q);
    }
    // Departure "report reaching/passing N feet": in report-then-transfer mode the
    // pilot reports this after takeoff; "passing 3000 feet" garbled to "3, 2015"
    // because it was unanchored (user 2026-07-26). Use the CAPPED report altitude +
    // VERB from engine::departure_report_alt_ft -- the raw config gave "passing 3000"
    // while LIMF's clearance said "reaching 2000", so the bias never matched the
    // readback and it garbled to "passing 2000" (user 2026-07-27). Only when
    // departing.
    {
      using SD = atc_state_machine::ATCState;
      const bool departing = ctx_state == SD::IFR_DEPARTURE_CLEARED ||
                             ctx_state == SD::IFR_FREQ_HANDOFF ||
                             ctx_state == SD::IFR_EN_ROUTE;
      bool reaching = false;
      const int rep = engine::departure_report_alt_ft(ctx_for_whisper, &reaching);
      if (rep > 0 && departing) {
        const std::string r = std::to_string(rep);
        // Bias BOTH verbs and the bare number -- the pilot may read back "reaching
        // 2000 feet", "passing 2000 feet", or just "2000 feet" (user 2026-07-27).
        // The KEY was the ALTITUDE: the raw config biased 3000 while the clearance
        // said 2000, so nothing matched and it garbled.
        add("reaching " + r + " feet");
        add("passing " + r + " feet");
        add(r + " feet");
      }
    }
    add(engine::current_controller_label());
    // NEXT station the pilot is handed to ("contact Lyon Approach on ...") -- the
    // pilot reads back that station name, so it MUST be anchored or it garbles
    // ("Lyon" -> "Laren", user 2026-07-26). current_controller_label alone only had
    // the station he was LEAVING.
    add(engine::pending_controller_label());
    add(engine::pending_departure_label());
    if (!locked_rwy.empty()) {
      add("runway " + locked_rwy);
      const std::string sp = spell_runway(locked_rwy);
      if (!sp.empty())
        add("runway " + sp);
    }
    // Holding point: the pilot reads back "holding point Delta" -- we KNOW it (the taxi
    // clearance assigned it from ctx.runway_holding_points, apt.dat-derived). Bias the
    // SPECIFIC spoken phrase, exactly like "runway 22" -- the generic "holding point" in
    // kCoreVocab is a weak anchor and garbled to "appeared in point" (user 2026-07-28,
    // LFLP: "holding point Delta"). On the ground only (irrelevant airborne). [C. P.
    // Potter]
    if (ctx_for_whisper.on_ground && !ctx_for_whisper.active_runway.empty()) {
      auto hp_it =
          ctx_for_whisper.runway_holding_points.find(ctx_for_whisper.active_runway);
      if (hp_it != ctx_for_whisper.runway_holding_points.end() &&
          !hp_it->second.empty()) {
        // "D" -> "Delta", "C1" -> "Charlie One" -- match the spoken clearance.
        add("holding point " + atc_phonetic::spell_holding_point(hp_it->second));
      }
    }
    // 2) Callsign forms (the single biggest garble source).
    add(phonetic);
    add(raw_cs);
    if (!phonetic.empty()) {
      std::vector<std::string> w;
      std::string cur;
      for (char c : phonetic) {
        if (c == ' ') {
          if (!cur.empty())
            w.push_back(cur);
          cur.clear();
        } else {
          cur += c;
        }
      }
      if (!cur.empty())
        w.push_back(cur);
      if (w.size() >= 2)
        add(w[w.size() - 2] + " " + w[w.size() - 1]); // short callsign
    }
    // 3) Route / airport proper nouns (skip enroute content on arrival ground).
    {
      const auto &ofp = simbrief_ofp::get();
      add(ofp.destination_icao);
      add(ctx_for_whisper.ifr_destination);
      // DESTINATION facilities the pilot addresses on arrival ("Annecy Tower",
      // "Annecy Ground") -- ONLY in the approach/arrival phases (he doesn't call
      // the dest tower before then; keeps the term count down, user 2026-07-26).
      // Airborne the ground-facility block above is skipped (airborne_ifr_drift_risk),
      // so without this the dest tower name was unanchored -> "Annecy Tower" garbled.
      {
        using S = atc_state_machine::ATCState;
        const bool near_dest = ctx_state == S::IFR_ARRIVAL ||
                               ctx_state == S::IFR_APPROACH_CONTACT ||
                               ctx_state == S::IFR_APPROACH_DESCENT ||
                               ctx_state == S::IFR_APPROACH_TOWER ||
                               ctx_state == S::IFR_LANDING_CLEARED;
        if (near_dest) {
          std::string dcity = ctx_for_whisper.ifr_destination;
          const auto dsp = dcity.find_first_of(" /-");
          if (dsp != std::string::npos)
            dcity = dcity.substr(0, dsp);
          if (!dcity.empty()) {
            add(dcity);            // "Annecy"
            add(dcity + " Tower"); // the facility the pilot calls on final
            add(dcity + " Ground"); // for the taxi-in call after landing
          }
          // Approach IAF / FAF idents -- the pilot is cleared to the IAF and
          // reports "established" at the FAF, so anchor those two fix names.
          add(engine::approach_iaf_ident());
          add(engine::approach_faf_ident());
        }
      }
      if (!arrival_ground) {
        // SID: the pilot reads back the SID designator + its terminating fix at
        // clearance ("via ROMAM 2 Alpha"). SimBrief sid_name is often "none", so
        // prefer the CIFP-derived SID (ctx.ifr_sid) and always bias the last-fix
        // NAME (ROMAM -- the garble-prone token) -- the clearance/departure phase.
        add(ctx_for_whisper.ifr_sid);          // e.g. "ROMA2A" (CIFP)
        add(ctx_for_whisper.ifr_sid_last_fix); // e.g. "ROMAM" (spoken fix)
        add(ofp.sid_name);                     // SimBrief SID if filed
        // SID intermediate fixes (SOCOF, TOLNA, BELUS, GIRED for ROMA2A). On the
        // ground the BIAS carried only the ENROUTE fixes, never the SID's own
        // waypoints (user 2026-07-26). Read from CIFP; ground-only (file read).
        if (ctx_for_whisper.on_ground && !ctx_for_whisper.ifr_sid.empty()) {
          const auto sidwp = cifp_reader::sid_waypoints(
              ctx_for_whisper.cifp_dir, ctx_for_whisper.nearest_airport_id,
              ctx_for_whisper.ifr_sid, ctx_for_whisper.active_runway,
              /*constrained_only=*/false);
          int nsid = 0;
          for (const auto &w : sidwp) {
            if (w.ident.empty())
              continue;
            add(w.ident);
            if (++nsid >= 8)
              break;
          }
        }
        const std::string star = engine::assigned_star_name();
        add(star);
        const std::string star_sp = engine::assigned_star_spoken();
        if (star_sp != star)
          add(star_sp);
        // Route fixes -- MIRROR the CTX prompt logic (align BIAS with CTX, user
        // 2026-07-26): in the arrival/approach phase the enroute FPL fixes are
        // behind the aircraft (noise), so PRUNE them and rely on the tracker-
        // FORWARD fixes (STAR + approach procedure, incl. IAF/FAF). Cruise/descent
        // keeps the enroute navlog (upcoming enroute fixes still matter there).
        using S3 = atc_state_machine::ATCState;
        const auto upcoming = engine::upcoming_route_fix_idents();
        const bool prune_navlog =
            !upcoming.empty() &&
            (ctx_state == S3::IFR_ARRIVAL ||
             ctx_state == S3::IFR_APPROACH_CONTACT ||
             ctx_state == S3::IFR_APPROACH_DESCENT ||
             ctx_state == S3::IFR_APPROACH_TOWER ||
             ctx_state == S3::IFR_LANDING_CLEARED);
        // "direct <fix>" prefix too: ATC issues "confirm direct <fix>" course
        // corrections and the pilot reads back "direct <fix>"; unanchored, "direct"
        // garbled to "Diet KUKEV" (user 2026-07-27). Forward fixes are the direct-to
        // targets.
        for (const auto &id : upcoming) { // STAR + approach forward fixes (IAF/FAF)
          add(id);
          add("direct " + id);
        }
        if (!prune_navlog) {
          int nfix = 0;
          for (const auto &f : ofp.navlog) {
            if (f.ident.empty())
              continue;
            add(f.ident);
            add("direct " + f.ident);
            if (++nfix >= 8)
              break;
          }
        }
      }
    }
    if (!airborne_ifr_drift_risk) {
      add(ctx_for_whisper.nearest_airport_id);
      add(ctx_for_whisper.nearest_airport_name);
      // Facilities the PILOT ADDRESSES ("Annecy Ground", then "Annecy Tower") --
      // the airport NAME ("Annecy Meythet") is never spoken, so "Annecy" in the
      // spoken form was unanchored and garbled to "9C" (user 2026-07-26). Anchor
      // the city alone + the facility forms the pilot actually says.
      std::string city = ctx_for_whisper.nearest_airport_name;
      const auto sep = city.find_first_of(" /-");
      if (sep != std::string::npos)
        city = city.substr(0, sep);
      if (!city.empty()) {
        add(city);
        add(city + " Ground");
        add(city + " Tower");
        add(city + " Approach");
      }
    }
    // 4) Core ATC readback vocab -- MULTI-WORD phrases only (single words like
    //    "climb"/"QNH"/"squawk" are already well recognised and would waste the
    //    <=100 budget). Fills the remainder after the flight-specific anchors.
    static const char *kCoreVocab[] = {
        "flight level", "cleared for takeoff", "cleared to land",
        "report established", "holding point", "report passing", "radar contact",
        "climb flight level", "descend flight level", "maintain flight level",
        "contact approach", "contact tower", "contact ground", "runway vacated",
        "line up and wait", "go around", "squawk ident", "reduce speed",
        "request taxi", "ready for departure",
        // "as filed" garbled to "Asphalt" in the clearance read-back (user
        // 2026-07-26); anchor both the phrase and the fuller form.
        "as filed", "cleared as filed",
        // Delivery-phase opening call: "request IFR clearance" garbled to "request
        // high factorance" (user 2026-07-26) -- anchor the request + the phrase.
        "IFR clearance", "request IFR clearance", "startup approved", nullptr};
    for (int i = 0; kCoreVocab[i]; ++i)
      add(kCoreVocab[i]);
    // Join comma-separated for the mistral context_bias[] path.
    for (size_t i = 0; i < bias.size(); ++i) {
      if (i)
        context_bias += ", ";
      context_bias += bias[i];
    }
  }

  if (g_transcript_log_) {
    static std::string s_last_logged_ctx;
    if (airport_ctx != s_last_logged_ctx) {
      std::fprintf(g_transcript_log_, "-- CTX: %s --\n", airport_ctx.c_str());
      s_last_logged_ctx = airport_ctx;
    } else {
      std::fprintf(g_transcript_log_, "-- CTX: (unchanged) --\n");
    }
    // Tag the configured encoding mode so A/B runs are traceable in transcript.log
    // (underscore|comma|quote|off|auto). For "auto" the ACTUAL discovered method is
    // in Log.txt "[STT-MISTRAL] context_bias method '...'".
    std::fprintf(g_transcript_log_, "-- BIAS [%s]: %s --\n",
                 settings::mistral_context_bias_encoding().c_str(),
                 context_bias.c_str());
    std::fflush(g_transcript_log_);
  }

  backends::stt::transcribe_async(
      std::move(pcm), src_rate,
      [](const backends::stt::TranscriptResult &wr) {
        if (!wr.success) {
          // Prefer the structured error_message (set by the cloud
          // backends via openai_common::interpret) so the user sees
          // the actual cause ("STT-OPENAI: Operation timed out").
          // Fall back to wr.text for older callers that still pack
          // the human-readable message there.
          const std::string display =
              !wr.error_message.empty() ? wr.error_message : wr.text;
          logging::error("STT error: %s", display.c_str());
          push_transcript(TranscriptEntry{
              static_cast<double>(XPLMGetElapsedTime()),
              TranscriptKind::System,
              display,
              "",
              "",
          });
          state_ = PTTState::IDLE;
          return;
        }

        ++total_transcriptions_;
        dispatch_pilot_transcript(wr.text, wr.quality);
      },
      airport_ctx, context_bias);
}

void on_ptt_released() {
  if (state_ != PTTState::RECORDING)
    return;
  // Keep the mic open for kPttTailSec so the last syllable is captured
  // before the buffer is handed to STT.
  state_ = PTTState::TAIL_RECORDING;
  ptt_tail_remaining_ = kPttTailSec;
}

// Debug-Texteingabe convenience: replace the keyword "REG" (case-
// insensitive) with the configured phonetic pilot callsign. Lets the
// user type "Bern Tower REG, ready for departure" instead of spelling
// "November One Two Three Alpha Bravo" every time. Voice path is
// untouched — Whisper produces phonetic words, this expansion only
// runs for typed input.
//
// Word-boundary matching: the keyword must be its own token (after
// stripping surrounding punctuation), so "REGION" stays unchanged.
// All occurrences in the text are replaced.
static std::string expand_callsign_placeholder(const std::string &text) {
  const std::string phonetic = settings::pilot_callsign();
  if (phonetic.empty())
    return text;

  auto eq_ci = [](const std::string &a, const char *b) {
    size_t i = 0;
    for (; i < a.size() && b[i] != '\0'; ++i) {
      if (std::tolower(static_cast<unsigned char>(a[i])) !=
          std::tolower(static_cast<unsigned char>(b[i])))
        return false;
    }
    return i == a.size() && b[i] == '\0';
  };

  std::string out;
  out.reserve(text.size() + phonetic.size());
  size_t i = 0;
  const size_t n = text.size();
  while (i < n) {
    if (std::isspace(static_cast<unsigned char>(text[i]))) {
      out += text[i++];
      continue;
    }
    size_t tok_start = i;
    while (i < n && !std::isspace(static_cast<unsigned char>(text[i])))
      ++i;
    std::string tok = text.substr(tok_start, i - tok_start);

    // Split off leading and trailing punctuation around the core.
    size_t lead = 0;
    while (lead < tok.size() &&
           !std::isalnum(static_cast<unsigned char>(tok[lead])))
      ++lead;
    size_t trail = tok.size();
    while (trail > lead &&
           !std::isalnum(static_cast<unsigned char>(tok[trail - 1])))
      --trail;
    std::string core = tok.substr(lead, trail - lead);
    if (eq_ci(core, "REG")) {
      out += tok.substr(0, lead);
      out += phonetic;
      out += tok.substr(trail);
    } else {
      out += tok;
    }
  }
  return out;
}

void submit_text(const std::string &text) {
  if (state_ != PTTState::IDLE) {
    char buf[128];
    std::snprintf(buf, sizeof(buf),
                  "[xp_wellys_atc] submit_text blocked, state=%d\n",
                  static_cast<int>(state_));
    XPLMDebugString(buf);
    return;
  }
  if (text.empty())
    return;
  // LM + TTS still need to be loaded — text bypasses STT, but the
  // downstream stages are identical to the voice path.
  if (!backends::lm_ready() || !backends::tts_ready()) {
    XPLMDebugString("[xp_wellys_atc][ERROR] submit_text blocked - LM/TTS not "
                    "ready (open the plugin window to download)\n");
    return;
  }
  state_ = PTTState::PROCESSING;
  ++total_transcriptions_;
  // quality = 1.0 — typed text skips the Whisper quality gate; the engine
  // will run the full intent-parse + state-machine path with no "say
  // again" short-circuit. Expand the "REG" placeholder keyword to the
  // configured phonetic callsign so the user doesn't have to spell out
  // the full registration in every test message.
  dispatch_pilot_transcript(expand_callsign_placeholder(text), 1.0f);
}

void update() {
  if (state_ == PTTState::PLAYING && !tts_pending_ &&
      !audio_player::is_playing()) {
    if (atis_playing_) {
      atis_playing_ = false;
      if (settings::debug_logging())
        XPLMDebugString(
            "[xp_wellys_atc][DEBUG] ATIS playback finished, state -> IDLE\n");
    } else {
      if (settings::debug_logging())
        XPLMDebugString(
            "[xp_wellys_atc][DEBUG] Playback finished, state -> IDLE\n");
    }
    state_ = PTTState::IDLE;
  }

  // PTT tail: keep recording for kPttTailSec after key release, then submit.
  float dt = 1.0f / 60.0f; // approximate per-frame at ~60fps
  if (state_ == PTTState::TAIL_RECORDING) {
    ptt_tail_remaining_ -= dt;
    if (ptt_tail_remaining_ <= 0.0f) {
      audio_recorder::stop_recording();
      state_ = PTTState::PROCESSING;
      submit_recording_to_stt();
    }
  }

  // ATIS cooldown timer
  if (atis_cooldown_ > 0.0f)
    atis_cooldown_ -= dt;

  // Flight-phase auto-correction of ATC state
  double now_secs_for_state = static_cast<double>(XPLMGetElapsedTime());
  atc_state_machine::check_auto_correction(flight_phase::get(), dt,
                                           now_secs_for_state);

  // Airport-change reset of EN_ROUTE → IDLE. Runs every frame so the
  // UI hint pipeline reflects a new airport's options the moment the
  // airport lock changes, instead of waiting for the next PTT call.
  atc_state_machine::check_airport_change(xplane_context::get(),
                                          now_secs_for_state);

  // Route fix tracker — logging only, no audio. Runs every frame regardless
  // of PTT state. Writes a System entry to transcript + Log.txt when the
  // aircraft enters the 5 NM zone around the next fix on the route.
  {
    const auto &ctx_rft = xplane_context::get();
    const std::string track_event = engine::poll_route_tracker(ctx_rft);
    if (!track_event.empty()) {
      push_transcript(TranscriptEntry{
          static_cast<double>(XPLMGetElapsedTime()),
          TranscriptKind::System,
          track_event,
          {},
          {},
      });
    }
    // Informational engine note (e.g. weather forced a non-preferred approach)
    // -> transcript System line. Log.txt already has the detailed reason.
    const std::string note = engine::take_pending_transcript_note();
    if (!note.empty()) {
      push_transcript(TranscriptEntry{
          static_cast<double>(XPLMGetElapsedTime()),
          TranscriptKind::System,
          note,
          {},
          {},
      });
    }
  }

  // Phase-2 traffic advisory poll. SDK-free engine helper consumes the
  // live traffic_context snapshot + state-machine state and may emit a
  // synthetic advisory transition. Only run while idle so a controller
  // utterance never overlaps with a pilot-driven exchange.
  if (state_ == PTTState::IDLE && backends::tts_ready()) {
    const auto &ctx_now = xplane_context::get();
    double now_secs = static_cast<double>(XPLMGetElapsedTime());

    // Ground runway-change notification: fires before any other poll so the
    // pilot is warned immediately when the active runway changes on the ground.
    std::string runway_change_text;
    if (engine::poll_ground_runway_change(ctx_now, &runway_change_text) &&
        !runway_change_text.empty()) {
      float active_freq = (ctx_now.active_com == 1) ? ctx_now.com1_freq_mhz
                                                    : ctx_now.com2_freq_mhz;
      char freq_str[16];
      std::snprintf(freq_str, sizeof(freq_str), "%.3f", active_freq);
      push_transcript(TranscriptEntry{
          static_cast<double>(XPLMGetElapsedTime()),
          TranscriptKind::Tower,
          runway_change_text,
          freq_str,
          current_tower_label(),
      });
      auto role = role_for_frequency(ctx_now);
      speak_response(runway_change_text, role, 1.0f);
      return; // one tower utterance per frame
    }

    // Readback-reminder poll runs FIRST (among pilot-dialog polls). The pilot was supposed to
    // read a clearance back and went silent — the tower nudge takes
    // precedence over traffic advisories and even over an unsolicited
    // go-around (which can't happen anyway without an active
    // clearance in PROCESSING). Cancellation case is handled inside
    // atc_state_machine before the call returns; here we just speak.
    std::string readback_reminder_text;
    if (engine::poll_readback_reminder(ctx_now, now_secs,
                                       &readback_reminder_text) &&
        !readback_reminder_text.empty()) {
      float active_freq = (ctx_now.active_com == 1) ? ctx_now.com1_freq_mhz
                                                    : ctx_now.com2_freq_mhz;
      char freq_str[16];
      std::snprintf(freq_str, sizeof(freq_str), "%.3f", active_freq);
      push_transcript(TranscriptEntry{
          static_cast<double>(XPLMGetElapsedTime()),
          TranscriptKind::Tower,
          readback_reminder_text,
          freq_str,
          current_tower_label(),
      });
      auto role = role_for_frequency(ctx_now);
      speak_response(readback_reminder_text, role, 1.0f);
      return; // one tower utterance per frame
    }

    // IFR departure handoff: Tower tells the pilot to contact Departure or
    // Approach ~10 s into climb. Fires before go-around/traffic checks.
    // Capture label before poll_departure_handoff() updates
    // s_current_controller_label.
    std::string label_pre_handoff = current_tower_label();
    std::string departure_handoff_text;
    if (engine::poll_departure_handoff(ctx_now, dt, &departure_handoff_text) &&
        !departure_handoff_text.empty()) {
      float active_freq = (ctx_now.active_com == 1) ? ctx_now.com1_freq_mhz
                                                    : ctx_now.com2_freq_mhz;
      char freq_str[16];
      std::snprintf(freq_str, sizeof(freq_str), "%.3f", active_freq);
      push_transcript(TranscriptEntry{
          static_cast<double>(XPLMGetElapsedTime()),
          TranscriptKind::Tower,
          departure_handoff_text,
          freq_str,
          label_pre_handoff,
      });
      auto role = role_for_frequency(ctx_now);
      speak_response(departure_handoff_text, role, 1.0f);
      return; // one tower utterance per frame
    }

    // IFR SID climb management: ATC-initiated step climbs and direct-to
    // shortcut while in IFR_RADAR_CONTACT state.
    // Capture label before poll_sid_climb() may update
    // s_current_controller_label (TMA exit).
    std::string label_pre_climb = current_tower_label();
    std::string sid_climb_text;
    if (engine::poll_sid_climb(ctx_now, dt, &sid_climb_text) &&
        !sid_climb_text.empty()) {
      float active_freq = (ctx_now.active_com == 1) ? ctx_now.com1_freq_mhz
                                                    : ctx_now.com2_freq_mhz;
      char freq_str[16];
      std::snprintf(freq_str, sizeof(freq_str), "%.3f", active_freq);
      push_transcript(TranscriptEntry{
          static_cast<double>(XPLMGetElapsedTime()),
          TranscriptKind::Tower,
          sid_climb_text,
          freq_str,
          label_pre_climb,
      });
      auto role = role_for_frequency(ctx_now);
      speak_response(sid_climb_text, role, 1.0f);
      return; // one tower utterance per frame
    }

    // Unified in-front-profile enforcement: altitude crossing + ICAO/procedure
    // speed + cleared-level compliance, run in EVERY airborne IFR phase (replaces
    // the old per-phase poll_speed_restriction / poll_altitude_compliance / CIFP
    // crossing patchwork). Altitude + speed clearances arm a readback; the
    // cleared-level courtesy nag does not (out_rb stays false).
    std::string profile_text;
    bool profile_rb = false;
    if (engine::poll_profile_enforcement(ctx_now, dt, &profile_text, &profile_rb) &&
        !profile_text.empty()) {
      float active_freq = (ctx_now.active_com == 1) ? ctx_now.com1_freq_mhz
                                                    : ctx_now.com2_freq_mhz;
      char freq_str[16];
      std::snprintf(freq_str, sizeof(freq_str), "%.3f", active_freq);
      push_transcript(TranscriptEntry{
          static_cast<double>(XPLMGetElapsedTime()),
          TranscriptKind::Tower,
          profile_text,
          freq_str,
          engine::current_controller_label(),
      });
      speak_response(profile_text, role_for_frequency(ctx_now), 1.0f);
      if (profile_rb)
        atc_state_machine::arm_readback(profile_text);
      return;
    }

    // IFR en-route management: Centre direct-to shortcut, TMA entry descent
    // clearance (proactive — ATC does not wait for pilot request), and
    // cross-track deviation alert.
    std::string enroute_text;
    std::string label_pre_enroute = current_tower_label();
    bool enroute_rb = false;
    if (engine::poll_enroute(ctx_now, dt, &enroute_text, &enroute_rb) &&
        !enroute_text.empty()) {
      float active_freq = (ctx_now.active_com == 1) ? ctx_now.com1_freq_mhz
                                                    : ctx_now.com2_freq_mhz;
      char freq_str[16];
      std::snprintf(freq_str, sizeof(freq_str), "%.3f", active_freq);
      push_transcript(TranscriptEntry{
          static_cast<double>(XPLMGetElapsedTime()),
          TranscriptKind::Tower,
          enroute_text,
          freq_str,
          label_pre_enroute,
      });
      auto role = role_for_frequency(ctx_now);
      speak_response(enroute_text, role, 1.0f);
      if (enroute_rb)
        atc_state_machine::arm_readback(enroute_text);
      return; // one tower utterance per frame
    }

    // IFR descent phase: TMA/CTR boundary detection and Approach handoff.
    std::string descent_text;
    bool descent_rb = false;
    if (engine::poll_descent(ctx_now, dt, &descent_text, &descent_rb) &&
        !descent_text.empty()) {
      float active_freq = (ctx_now.active_com == 1) ? ctx_now.com1_freq_mhz
                                                    : ctx_now.com2_freq_mhz;
      char freq_str[16];
      std::snprintf(freq_str, sizeof(freq_str), "%.3f", active_freq);
      push_transcript(TranscriptEntry{
          static_cast<double>(XPLMGetElapsedTime()),
          TranscriptKind::Tower,
          descent_text,
          freq_str,
          engine::current_controller_label(),
      });
      auto role = role_for_frequency(ctx_now);
      speak_response(descent_text, role, 1.0f);
      if (descent_rb)
        atc_state_machine::arm_readback(descent_text);
      return;
    }

    // IFR arrival phase: flying the STAR under ACC/arrival; fires the real
    // Approach handoff (TMA/CTR boundary or 50 NM fallback).
    std::string arrival_text;
    bool arrival_rb = false;
    if (engine::poll_arrival(ctx_now, dt, &arrival_text, &arrival_rb) &&
        !arrival_text.empty()) {
      float active_freq = (ctx_now.active_com == 1) ? ctx_now.com1_freq_mhz
                                                    : ctx_now.com2_freq_mhz;
      char freq_str[16];
      std::snprintf(freq_str, sizeof(freq_str), "%.3f", active_freq);
      push_transcript(TranscriptEntry{
          static_cast<double>(XPLMGetElapsedTime()),
          TranscriptKind::Tower,
          arrival_text,
          freq_str,
          engine::current_controller_label(),
      });
      auto role = role_for_frequency(ctx_now);
      speak_response(arrival_text, role, 1.0f);
      if (arrival_rb)
        atc_state_machine::arm_readback(arrival_text);
      return;
    }

    // (altitude-compliance courtesy prompt now folded into
    // poll_profile_enforcement above, so it runs in the approach too.)

    // IFR approach STAR constraint management: step-down clearances + final alt.
    std::string approach_text;
    bool approach_rb = false;
    if (engine::poll_approach(ctx_now, dt, &approach_text, &approach_rb) &&
        !approach_text.empty()) {
      float active_freq = (ctx_now.active_com == 1) ? ctx_now.com1_freq_mhz
                                                    : ctx_now.com2_freq_mhz;
      char freq_str[16];
      std::snprintf(freq_str, sizeof(freq_str), "%.3f", active_freq);
      push_transcript(TranscriptEntry{
          static_cast<double>(XPLMGetElapsedTime()),
          TranscriptKind::Tower,
          approach_text,
          freq_str,
          engine::current_controller_label(),
      });
      auto role = role_for_frequency(ctx_now);
      speak_response(approach_text, role, 1.0f);
      if (approach_rb)
        atc_state_machine::arm_readback(approach_text);
      return; // one utterance per frame
    }

    // After-FAF lateral deviation: "confirm established on the approach".
    std::string alignment_text;
    if (engine::poll_approach_alignment(ctx_now, dt, &alignment_text) &&
        !alignment_text.empty()) {
      float active_freq = (ctx_now.active_com == 1) ? ctx_now.com1_freq_mhz
                                                    : ctx_now.com2_freq_mhz;
      char freq_str[16];
      std::snprintf(freq_str, sizeof(freq_str), "%.3f", active_freq);
      push_transcript(TranscriptEntry{
          static_cast<double>(XPLMGetElapsedTime()),
          TranscriptKind::Tower,
          alignment_text,
          freq_str,
          engine::current_controller_label(),
      });
      auto role = role_for_frequency(ctx_now);
      speak_response(alignment_text, role, 1.0f);
      return;
    }

    // Phase-4 go-around trigger runs *before* the traffic advisory so a
    // single tick can never produce both: when the runway is occupied,
    // the go-around call is the more urgent of the two.
    std::string go_around_text;
    if (engine::poll_go_around(ctx_now, now_secs, &go_around_text) &&
        !go_around_text.empty()) {
      float active_freq = (ctx_now.active_com == 1) ? ctx_now.com1_freq_mhz
                                                    : ctx_now.com2_freq_mhz;
      char freq_str[16];
      std::snprintf(freq_str, sizeof(freq_str), "%.3f", active_freq);
      push_transcript(TranscriptEntry{
          static_cast<double>(XPLMGetElapsedTime()),
          TranscriptKind::Tower,
          go_around_text,
          freq_str,
          current_tower_label(),
      });
      auto role = role_for_frequency(ctx_now);
      speak_response(go_around_text, role, 1.0f);
    } else {
      std::string advisory_text;
      if (engine::poll_traffic_advisory(ctx_now, now_secs, &advisory_text) &&
          !advisory_text.empty()) {
        float active_freq = (ctx_now.active_com == 1) ? ctx_now.com1_freq_mhz
                                                      : ctx_now.com2_freq_mhz;
        char freq_str[16];
        std::snprintf(freq_str, sizeof(freq_str), "%.3f", active_freq);
        push_transcript(TranscriptEntry{
            static_cast<double>(XPLMGetElapsedTime()),
            TranscriptKind::Tower,
            advisory_text,
            freq_str,
            current_tower_label(),
        });
        auto role = role_for_frequency(ctx_now);
        speak_response(advisory_text, role, 1.0f);
      }
    }
  }

  // Airport-change detection lives in atc_state_machine::process now —
  // it fires off the next pilot transmission. No per-frame loop here.

  // ATIS playback trigger — requires COM radio power + tuning delay.
  // ATIS reception works on EITHER COM1 or COM2: pilots commonly park
  // ATIS on COM2 (standby) while keeping COM1 on the controller's freq.
  const auto &ctx = xplane_context::get();
  int atis_com =
      ctx.com_radio_powered ? atis_generator::which_com_tuned_to_atis(ctx) : 0;
  bool tuned = atis_com != 0;

  if (tuned) {
    atis_tuned_timer_ += dt;
  } else {
    atis_tuned_timer_ = 0.0f;
  }

  // Pilot retuned (or powered off) the specific COM that's playing ATIS
  // — abort. Aborts even if the OTHER COM is still on ATIS, because we
  // can't switch buses mid-broadcast. The cooldown stays intact so re-
  // tuning ATIS within the cooldown window stays silent (we already
  // announced this letter).
  if (atis_playing_ && atis_com != atis_active_com_) {
    audio_player::abort_playback();
    atis_playing_ = false;
    state_ = PTTState::IDLE;
    if (settings::debug_logging())
      XPLMDebugString("[xp_wellys_atc][DEBUG] ATIS aborted: pilot retuned "
                      "the COM that was playing ATIS\n");
  }

  // ATIS is a side-channel like Traffic — independent of ATCState.
  // Pilot can re-tune ATIS at any point (e.g. holding point) to refresh
  // the broadcast. Only PTT state and TTS readiness gate playback so we
  // never overlap an active pilot/controller exchange.
  if (state_ == PTTState::IDLE && atis_cooldown_ <= 0.0f && tuned &&
      atis_tuned_timer_ >= kAtisTuneDelaySec && backends::tts_ready()) {
    std::string atis_text = atis_generator::generate_atis_text(ctx);
    if (atis_text.empty())
      return; // no ATIS letter assigned yet -- skip playback this frame

    if (settings::debug_logging()) {
      char dbg[64];
      std::snprintf(dbg, sizeof(dbg), " (COM%d)", atis_com);
      XPLMDebugString(("[xp_wellys_atc][DEBUG] ATIS triggered" +
                       std::string(dbg) + ": " + atis_text + "\n")
                          .c_str());
    }

    // Log the ATIS broadcast against the COM it actually plays on, not
    // the active COM — pilot may be on Tower with active=COM1 while
    // ATIS streams through COM2.
    float atis_com_freq =
        (atis_com == 2) ? ctx.com2_freq_mhz : ctx.com1_freq_mhz;
    char freq_str[16];
    std::snprintf(freq_str, sizeof(freq_str), "%.3f", atis_com_freq);

    // Stage the transcript entry but do not publish it yet — the push
    // happens in the TTS success callback so a silent synthesis
    // failure (bad voice id, OpenAI error, network) leaves no ghost
    // line in the history. Cooldown + atis_playing_ ARE set up-front
    // to block a retrigger loop while synthesis is in flight.
    TranscriptEntry pending_entry{
        static_cast<double>(XPLMGetElapsedTime()),
        TranscriptKind::Tower,
        atis_text,
        freq_str,
        current_tower_label(),
    };

    atis_playing_ = true;
    atis_active_com_ = atis_com;
    atis_cooldown_ = kAtisCooldownSec;
    // ATIS reads slower than tower/ground — Piper length_scale > 1
    // produces the slower rate the OpenAI path used to get from
    // speed=0.85.
    speak_response(atis_text, model_manifest::VoiceRole::Atis, 1.18f, atis_com,
                   [entry = std::move(pending_entry)]() mutable {
                     push_transcript(std::move(entry));
                   });
  }
}

void reset_atis_cooldown() {
  atis_cooldown_ = 0.0f;
  atis_tuned_timer_ = 0.0f;
}

PTTState ptt_state() { return state_; }

std::string ptt_state_label() {
  switch (state_) {
  case PTTState::IDLE:
    return "Ready";
  case PTTState::RECORDING:
  case PTTState::TAIL_RECORDING:
    return "[REC]";
  case PTTState::PROCESSING:
    return "[Processing...]";
  case PTTState::PLAYING:
    return "[ATC speaking...]";
  }
  return "UNKNOWN";
}

float last_recording_duration() { return last_duration_; }
size_t last_recording_samples() { return last_samples_; }
size_t last_wav_bytes() { return last_wav_bytes_; }

const intent_parser::PilotMessage &last_pilot_message() {
  return last_pilot_message_;
}

const std::vector<TranscriptEntry> &transcript_entries() { return transcript_; }

void clear_transcript() { transcript_.clear(); }

std::string last_atc_response() {
  // Tower entries only — System entries (e.g. "Funkstoerung") do not
  // count as ATC replies, even though they appear in the transcript.
  for (auto it = transcript_.rbegin(); it != transcript_.rend(); ++it) {
    if (it->kind == TranscriptKind::Tower)
      return it->text;
  }
  return "";
}

int total_transcriptions() { return total_transcriptions_; }
int total_api_calls() { return total_inferences_ + engine::lm_inferences(); }

} // namespace atc_session
