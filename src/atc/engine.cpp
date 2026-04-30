/*
 * xp_wellys_atc - AI-powered ATC voice communication for X-Plane 12
 * Copyright (C) 2026 thWelly & Claude (Anthropic)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#include "engine.hpp"

#include "atc/atc_state_machine.hpp"
#include "atc/atc_templates.hpp"
#include "backends/manager.hpp"
#include "core/logging.hpp"
#include "persistence/settings.hpp"

namespace engine {

static int profanity_warnings_ = 0;
static int lm_inferences_ = 0;

void reset() {
  profanity_warnings_ = 0;
  lm_inferences_ = 0;
}

int lm_inferences() { return lm_inferences_; }

static std::string build_say_again(const std::string &callsign) {
  return callsign.empty() ? "Say again." : callsign + ", say again.";
}

static std::string build_profanity_response(int warning_number,
                                            const std::string &callsign) {
  if (warning_number == 1) {
    return callsign + ", maintain proper radio discipline. Use standard "
                      "phraseology on this frequency.";
  }
  if (warning_number == 2) {
    return callsign + ", this is your final warning. Continued inappropriate "
                      "language on this frequency will be reported to the "
                      "civil aviation authority. Use standard phraseology.";
  }
  return callsign + ", your conduct has been noted and will be reported to "
                    "the aviation authority. Maintain radio discipline "
                    "immediately.";
}

static Output run_state_machine(const intent_parser::PilotMessage &msg,
                                const xplane_context::XPlaneContext &ctx_now) {
  auto atc_resp = atc_state_machine::process(msg, ctx_now);
  if (settings::debug_logging())
    logging::debug("ATC response text: %s",
                   atc_resp.text.empty() ? "(silent)" : atc_resp.text.c_str());
  Output out;
  out.parsed = msg;
  out.response_text = atc_resp.text;
  return out;
}

void process_transcript(Input in, Done done) {
  if (settings::debug_logging())
    logging::debug("Whisper response (quality=%.2f): \"%s\"", in.quality,
                   in.transcript.c_str());

  // Poor transcript quality — likely noise or engine sounds
  if (in.quality < 0.3f) {
    logging::info("Transcript quality too low, requesting say again");
    Output out;
    out.response_text = build_say_again(in.pilot_callsign);
    done(std::move(out));
    return;
  }

  const auto &ctx = *in.ctx;

  // Parse intent
  auto parsed = intent_parser::parse(in.transcript, ctx);

  if (settings::debug_logging())
    logging::debug("Intent: %s (confidence=%.2f), callsign=%s",
                   intent_parser::intent_name(parsed.intent), parsed.confidence,
                   parsed.callsign.empty() ? "(none)"
                                           : parsed.callsign.c_str());

  // Inappropriate language — intercept before state machine.
  // Does NOT change ATC state, pilot can continue normally after.
  if (parsed.intent == intent_parser::PilotIntent::INAPPROPRIATE_LANGUAGE) {
    ++profanity_warnings_;
    std::string cs =
        parsed.callsign.empty() ? in.pilot_callsign : parsed.callsign;
    logging::info("Radio discipline warning #%d", profanity_warnings_);
    Output out;
    out.parsed = parsed;
    out.response_text = build_profanity_response(profanity_warnings_, cs);
    out.is_warning = true;
    done(std::move(out));
    return;
  }

  // Sub-variant disambiguation: rule-based parser matches parent
  // intents via keywords, but semantic sub-variants (pattern vs
  // cross-country departure) are better resolved by the LLM. If the
  // parsed intent has a sharper variant AND the local LM is loaded,
  // ask it to pick between them.
  using PI = intent_parser::PilotIntent;
  bool is_departure_variant = parsed.intent == PI::READY_FOR_DEPARTURE ||
                              parsed.intent == PI::READY_FOR_DEPARTURE_VFR;
  if (is_departure_variant && backends::lm_ready()) {
    std::string prompt =
        "You are an ATC intent classifier. A VFR pilot at a towered "
        "airport just said they are ready for departure. Classify "
        "the pilot's intent as EXACTLY ONE of:\n"
        "- READY_FOR_DEPARTURE: pattern/local flight. Pilot stays "
        "in the traffic pattern, does touch-and-go, or will come "
        "BACK INBOUND for landing. Keywords: \"inbound\", "
        "\"pattern\", \"local\", \"touch and go\", \"training\", "
        "\"closed traffic\", \"remain in the pattern\".\n"
        "- READY_FOR_DEPARTURE_VFR: cross-country departure. Pilot "
        "leaves the airport area to another destination and will "
        "NOT come back. Keywords: \"on course\", \"en route\", "
        "\"northbound\", \"southbound\", \"eastbound\", "
        "\"westbound\", \"VFR to <destination>\", "
        "\"cross-country\", \"departure to <place>\".\n"
        "IMPORTANT: \"inbound\" always means PATTERN (pilot will "
        "return to land), never cross-country.\n"
        "Respond with ONLY the intent name. Nothing else.";
    std::string transcript = in.transcript;
    xplane_context::XPlaneContext ctx_snapshot = ctx;
    ++lm_inferences_;
    backends::lm::classify_intent_async(
        transcript, prompt,
        // NOLINTNEXTLINE(bugprone-exception-escape)
        [parsed, ctx_snapshot, done = std::move(done)](
            const std::string &result, bool success) mutable {
          auto msg = parsed;
          if (success) {
            auto new_intent = intent_parser::intent_from_key(result);
            bool valid = new_intent == PI::READY_FOR_DEPARTURE ||
                         new_intent == PI::READY_FOR_DEPARTURE_VFR;
            if (valid && new_intent != msg.intent) {
              logging::info("Intent disambiguation: %s -> %s (via local LM)",
                            intent_parser::intent_name(msg.intent),
                            intent_parser::intent_name(new_intent));
              msg.intent = new_intent;
            } else if (!valid) {
              logging::info(
                  "LM disambig inconclusive (returned \"%s\"), keeping "
                  "rule-based: %s",
                  result.c_str(), intent_parser::intent_name(msg.intent));
            }
          } else {
            logging::info("LM disambig failed, keeping rule-based result");
          }
          done(run_state_machine(msg, ctx_snapshot));
        });
    return;
  }

  // Two-stage intent resolution
  bool needs_lm = parsed.confidence < 0.7f ||
                  parsed.intent == intent_parser::PilotIntent::UNKNOWN;

  if (!needs_lm) {
    // High confidence: process through state machine directly
    done(run_state_machine(parsed, ctx));
    return;
  }

  if (!backends::lm_ready()) {
    // LM not loaded (headless tools, model still downloading) — use
    // state machine with _INVALID fallback.
    auto atc_resp = atc_state_machine::process(parsed, ctx);
    std::string response = atc_resp.text;
    if (response.empty()) {
      std::string cs =
          parsed.callsign.empty() ? in.pilot_callsign : parsed.callsign;
      response = "Say again, " + cs + ".";
    }
    logging::info("ATC (fallback): %s", response.c_str());
    Output out;
    out.parsed = parsed;
    out.response_text = response;
    done(std::move(out));
    return;
  }

  // LM intent classification
  using FT = xplane_context::FrequencyType;
  bool is_towered = ctx.is_towered_airport &&
                    ctx.frequency_type != FT::UNICOM &&
                    ctx.frequency_type != FT::CTAF;

  std::string state_str =
      atc_state_machine::state_name(atc_state_machine::get_state());
  auto valid = atc_templates::valid_intents(is_towered, state_str);

  std::string valid_list;
  for (const auto &v : valid) {
    if (!valid_list.empty())
      valid_list += ", ";
    valid_list += v;
  }

  std::string sys_prompt = atc_templates::get_prompt("gpt_classify_prompt");
  if (sys_prompt.empty()) {
    sys_prompt = "You are an ATC intent classifier. The pilot is in "
                 "state {state}. Valid intents: {valid_intents}. The "
                 "pilot said: \"{transcript}\"\nRespond with ONLY the "
                 "intent name, nothing else. If none match, respond "
                 "with \"_INVALID\".";
  }
  sys_prompt = atc_templates::fill(
      sys_prompt,
      {{"state", state_str},
       {"valid_intents", valid_list},
       {"transcript", in.transcript},
       {"frequency_type",
        xplane_context::frequency_type_name(ctx.frequency_type)},
       {"on_ground", ctx.on_ground ? "true" : "false"},
       {"altitude_ft", std::to_string(static_cast<int>(ctx.altitude_ft_msl))},
       {"groundspeed_kts",
        std::to_string(static_cast<int>(ctx.groundspeed_kts))},
       {"airport", ctx.nearest_airport_id}});

  logging::info("Routing to local LM intent classification");

  // Snapshot ctx so the async callback sees the state at the moment the
  // pilot spoke, not whatever ctx contains when the LM responds.
  xplane_context::XPlaneContext ctx_snapshot = ctx;
  ++lm_inferences_;
  backends::lm::classify_intent_async(
      in.transcript, sys_prompt,
      // NOLINTNEXTLINE(bugprone-exception-escape)
      [parsed, ctx_snapshot, done = std::move(done)](std::string intent_key,
                                                     bool lm_success) mutable {
        if (!lm_success)
          intent_key = "_INVALID";

        if (settings::debug_logging())
          logging::debug("LM classified intent: %s", intent_key.c_str());

        // Build a PilotMessage with LM-classified intent and route
        // through the state machine for full frequency validation.
        auto lm_msg = parsed;
        lm_msg.intent = intent_parser::intent_from_key(intent_key);
        lm_msg.confidence = 0.85f;

        auto atc_resp = atc_state_machine::process(lm_msg, ctx_snapshot);

        if (settings::debug_logging())
          logging::debug("ATC response text: %s", atc_resp.text.empty()
                                                      ? "(silent)"
                                                      : atc_resp.text.c_str());

        Output out;
        out.parsed = lm_msg;
        out.response_text = atc_resp.text;
        done(std::move(out));
      });
}

} // namespace engine
