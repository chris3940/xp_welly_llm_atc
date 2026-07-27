/*
 * xp_wellys_atc - AI-powered ATC voice communication for X-Plane 12
 * Copyright (C) 2026 thWelly & Claude (Anthropic)
 *
 * Licensed under the GNU GPL-3.0-or-later. See LICENSE.
 */

#include "backends/mistral_stt.hpp"

#include "backends/openai_common.hpp"
#include "core/logging.hpp"
#include "persistence/models_catalog.hpp"
#include "persistence/settings.hpp"

#include <curl/curl.h>
#include <json.hpp>

#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <sstream>
#include <thread>
#include <utility>

namespace backends {

namespace {
constexpr const char *kBackendTag = "STT-MISTRAL";

size_t write_to_string(char *ptr, size_t size, size_t nmemb, void *userdata) {
  auto *response = static_cast<std::string *>(userdata);
  const size_t bytes = size * nmemb;
  response->append(ptr, bytes);
  return bytes;
}

// Return true if the transcript is a Voxtral hallucination loop — a single
// word repeated more than kMaxConsecutive times in a row (e.g. "climb" ×100).
// Threshold of 5 avoids false positives on normal aviation repetition
// ("say again, say again, say again" = 3).
bool is_loop_hallucination(const std::string &text) {
  constexpr int kMaxConsecutive = 5;
  std::istringstream ss(text);
  std::string tok, prev;
  int run = 0;
  while (ss >> tok) {
    // lowercase + strip trailing punctuation for comparison
    std::transform(tok.begin(), tok.end(), tok.begin(),
                   [](unsigned char c) { return std::tolower(c); });
    while (!tok.empty() && std::ispunct(static_cast<unsigned char>(tok.back())))
      tok.pop_back();
    if (tok.empty())
      continue;
    run = (tok == prev) ? run + 1 : 1;
    if (run > kMaxConsecutive)
      return true;
    prev = tok;
  }
  return false;
}

// Split the curated context_bias list on COMMAS ONLY into raw phrases (internal
// spaces PRESERVED); trim, drop empties, dedupe, cap 100. The whitespace-free
// encoding is applied later per method (build_bias_field).
std::vector<std::string> split_context(const std::string &s) {
  constexpr size_t kMaxEntries = 100;
  std::vector<std::string> out;
  size_t lo = 0;
  auto flush = [&](size_t hi) {
    size_t a = lo, b = hi;
    while (a < b && std::isspace(static_cast<unsigned char>(s[a])))
      ++a;
    while (b > a && std::isspace(static_cast<unsigned char>(s[b - 1])))
      --b;
    if (b <= a || out.size() >= kMaxEntries)
      return;
    std::string tok = s.substr(a, b - a);
    for (const auto &e : out)
      if (e == tok)
        return;
    out.emplace_back(std::move(tok));
  };
  for (size_t i = 0; i <= s.size(); ++i) {
    if (i == s.size() || s[i] == ',') {
      flush(i);
      lo = i + 1;
    }
  }
  return out;
}

// Voxtral's context_bias is ONE field of COMMA-separated values
// (context_bias_input_method=comma_separated); an item may not contain a raw
// comma or whitespace. Build that single field from the curated phrases, with a
// per-method encoding for MULTI-WORD phrases (the exact accepted encoding is
// auto-discovered at runtime by transcribe(), then cached):
//   0 QUOTE     -- wrap a multi-word phrase in double quotes: ...,"flight level",...
//   1 UNDERSCORE-- join its words with underscores:           ...,flight_level,...
//   2 COMMA     -- flatten to individual words (always accepted; the user's
//                  "one,two,one"):                            ...,flight,level,...
// Single-word items (no space: "6071", "120.230", "X-Ray") pass through unchanged
// in every method. Returns "" when there are no phrases.
std::string build_bias_field(const std::vector<std::string> &phrases,
                             int method) {
  auto has_ws = [](const std::string &p) {
    for (char c : p)
      if (std::isspace(static_cast<unsigned char>(c)))
        return true;
    return false;
  };
  std::string field;
  auto append = [&](const std::string &item) {
    if (item.empty())
      return;
    if (!field.empty())
      field += ',';
    field += item;
  };
  for (const auto &p : phrases) {
    if (!has_ws(p)) {
      append(p);
      continue;
    }
    if (method == 0) { // quote-wrap
      append("\"" + p + "\"");
    } else if (method == 1) { // underscore
      std::string t;
      bool ps = false;
      for (char c : p) {
        if (std::isspace(static_cast<unsigned char>(c))) {
          ps = true;
          continue;
        }
        if (ps && !t.empty())
          t += '_';
        ps = false;
        t += c;
      }
      append(t);
    } else { // 2: comma -- flatten to words (each becomes its own CSV item)
      size_t lo = 0;
      for (size_t i = 0; i <= p.size(); ++i) {
        if (i == p.size() || std::isspace(static_cast<unsigned char>(p[i]))) {
          if (i > lo)
            append(p.substr(lo, i - lo));
          lo = i + 1;
        }
      }
    }
  }
  return field;
}
} // namespace

MistralStt::MistralStt(std::string api_key, std::string model,
                       std::string base_url)
    : api_key_(std::move(api_key)), model_(std::move(model)),
      base_url_(std::move(base_url)) {}

std::string MistralStt::transcribe(const std::vector<float> &pcm_16k_mono,
                                   const std::string &airport_context,
                                   const std::string &context_bias) {
  last_error_.clear();
  if (api_key_.empty()) {
    logging::error("[%s] No API key configured", kBackendTag);
    last_error_ = std::string(kBackendTag) + ": No API key configured";
    return {};
  }
  if (pcm_16k_mono.empty())
    return {};

  std::string language = settings::backend_language();
  if (language.empty())
    language = "en";

  std::vector<uint8_t> wav = openai_common::pcm_float32_to_wav(pcm_16k_mono);
  const std::string key_tail = openai_common::last4(api_key_);
  logging::info("[%s] POST /v1/audio/transcriptions, %zu samples, model %s, "
                "lang=%s, key ...%s",
                kBackendTag, pcm_16k_mono.size(), model_.c_str(),
                language.c_str(), key_tail.c_str());

  CURL *curl = curl_easy_init();
  if (!curl) {
    logging::error("[%s] curl_easy_init failed", kBackendTag);
    last_error_ = std::string(kBackendTag) + ": curl_easy_init failed";
    return {};
  }

  const std::string url = base_url_ + "/v1/audio/transcriptions";
  const std::string auth = "Authorization: Bearer " + api_key_;
  struct curl_slist *headers = curl_slist_append(nullptr, auth.c_str());

  const bool supports_bias =
      models_catalog::mistral_stt_supports_context_bias(model_);
  const std::vector<std::string> bias_phrases =
      supports_bias ? split_context(context_bias) : std::vector<std::string>{};

  // Multi-word context_bias encoding is AUTO-DISCOVERED at runtime: the Voxtral
  // comma_separated input method rejects an item with whitespace/commas, and the
  // exact accepted multi-word encoding is unknown, so try QUOTE -> UNDERSCORE ->
  // COMMA and CACHE the first the server accepts (COMMA flattens to words and can
  // never be rejected). Cached for the session so later calls skip to the winner,
  // avoiding a rebuild/retry per hypothesis (user 2026-07-25).
  static std::atomic<int> s_bias_method{-1};
  static const char *const kMethodName[] = {"quote", "underscore", "comma"};

  auto build_mime = [&](int method) -> curl_mime * {
    curl_mime *m = curl_mime_init(curl);
    curl_mimepart *p = curl_mime_addpart(m);
    curl_mime_name(p, "file");
    curl_mime_data(p, reinterpret_cast<const char *>(wav.data()), wav.size());
    curl_mime_filename(p, "audio.wav");
    curl_mime_type(p, "audio/wav");
    p = curl_mime_addpart(m);
    curl_mime_name(p, "model");
    curl_mime_data(p, model_.c_str(), CURL_ZERO_TERMINATED);
    p = curl_mime_addpart(m);
    curl_mime_name(p, "language");
    curl_mime_data(p, language.c_str(), CURL_ZERO_TERMINATED);
    // Freeform prompt -- the full aviation vocabulary + phonetic callsign as
    // coherent text; consumed by all Voxtral models and carries the multi-word /
    // spelled phrasing regardless of the context_bias encoding.
    if (!airport_context.empty()) {
      p = curl_mime_addpart(m);
      curl_mime_name(p, "prompt");
      curl_mime_data(p, airport_context.c_str(), CURL_ZERO_TERMINATED);
    }
    if (supports_bias && method >= 0 && !bias_phrases.empty()) {
      const std::string field = build_bias_field(bias_phrases, method);
      if (!field.empty()) {
        p = curl_mime_addpart(m);
        curl_mime_name(p, "context_bias");
        curl_mime_data(p, field.c_str(), CURL_ZERO_TERMINATED);
      }
    }
    return m;
  };

  std::string response_body;
  curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
  curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_to_string);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response_body);
  openai_common::apply_default_timeouts(curl, openai_common::CallKind::Stt);

  // Config switch: force a specific encoding (A/B in-sim without rebuild), else
  // "auto" = discover-and-cache. quote=0, underscore=1, comma=2; "off"/"none"
  // disables context_bias entirely (prompt-only -- the pre-fix behaviour, to A/B
  // whether the dedicated bias helps or hurts).
  int forced = -2; // -2 = auto
  bool bias_off = false;
  {
    const std::string enc = settings::mistral_context_bias_encoding();
    if (enc == "quote")
      forced = 0;
    else if (enc == "underscore")
      forced = 1;
    else if (enc == "comma")
      forced = 2;
    else if (enc == "off" || enc == "none")
      bias_off = true;
  }
  std::vector<int> methods;
  if (!supports_bias || bias_phrases.empty() || bias_off)
    methods = {-1}; // no context_bias field at all
  else if (forced >= 0)
    methods = {forced}; // config-forced encoding (no discovery/cache)
  else if (s_bias_method.load() >= 0)
    methods = {s_bias_method.load()}; // cached winner (auto)
  else
    methods = {0, 1, 2}; // auto discovery order

  openai_common::HttpResult res;
  for (size_t mi = 0; mi < methods.size(); ++mi) {
    const int method = methods[mi];
    curl_mime *mime = build_mime(method);
    curl_easy_setopt(curl, CURLOPT_MIMEPOST, mime);
    long http_code = 0;
    for (int attempt = 0; attempt < 2; ++attempt) {
      response_body.clear();
      CURLcode rc = curl_easy_perform(curl);
      http_code = 0;
      curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);
      res = openai_common::interpret(static_cast<int>(rc), http_code,
                                     response_body, kBackendTag);
      if (res.success || !res.transient)
        break;
      logging::info("[%s] transient error, retrying once: %s", kBackendTag,
                    res.error_message.c_str());
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    curl_mime_free(mime);

    // context_bias encoding rejected -> try the next method (discovery only).
    const bool ctxbias_rejected =
        http_code == 400 && method >= 0 &&
        response_body.find("context_bias") != std::string::npos;
    if (ctxbias_rejected && s_bias_method.load() < 0 &&
        mi + 1 < methods.size()) {
      logging::info(
          "[%s] context_bias method '%s' rejected (HTTP 400), trying '%s'",
          kBackendTag, kMethodName[method], kMethodName[methods[mi + 1]]);
      continue;
    }
    if (res.success && method >= 0) {
      if (forced >= 0) {
        logging::info("[%s] context_bias method '%s' (config-forced, %zu phrases)",
                      kBackendTag, kMethodName[method], bias_phrases.size());
      } else if (s_bias_method.load() < 0) {
        s_bias_method.store(method);
        logging::info("[%s] context_bias method '%s' ACCEPTED (%zu phrases) -- "
                      "cached for session",
                      kBackendTag, kMethodName[method], bias_phrases.size());
      }
    }
    break;
  }

  curl_slist_free_all(headers);
  curl_easy_cleanup(curl);

  if (!res.success) {
    logging::error("[%s] %s", kBackendTag, res.error_message.c_str());
    last_error_ = res.error_message;
    return {};
  }

  try {
    const auto j = nlohmann::json::parse(response_body);
    std::string text = j.value("text", std::string{});
    // Strip underscores that the "underscore" context_bias encoding leaks into
    // the transcript (Voxtral echoes the bias term verbatim: "November Seven Five
    // Zero X-Ray Papa" -> "November_Seven_Five_Zero_X-Ray_Papa", "squawk 2373" ->
    // "squawk_2373"). Underscores never occur in real ATC speech, so replacing
    // '_' -> ' ' is always safe (no-op for the comma/quote encodings) and is
    // ESSENTIAL: the raw transcript feeds the rule intent-parser, the readback
    // verifier, AND the Mistral LM classifier -- underscored tokens broke keyword
    // matching, number read-back, and LM classification (user 2026-07-26).
    std::replace(text.begin(), text.end(), '_', ' ');
    if (is_loop_hallucination(text)) {
      logging::error("[%s] hallucination loop detected, rejecting transcript",
                     kBackendTag);
      last_error_ = std::string(kBackendTag) + ": hallucination loop rejected";
      return {};
    }
    return text;
  } catch (const std::exception &e) {
    logging::error("[%s] JSON parse error: %s", kBackendTag, e.what());
    last_error_ = std::string(kBackendTag) + ": JSON parse error: " + e.what();
    return {};
  }
}

} // namespace backends
