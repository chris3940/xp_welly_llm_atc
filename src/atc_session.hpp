#ifndef ATC_SESSION_HPP
#define ATC_SESSION_HPP

#include "intent_parser.hpp"

#include <cstddef>
#include <string>
#include <vector>

namespace atc_session {

enum class PTTState { IDLE, RECORDING, PROCESSING, PLAYING };

struct TranscriptEntry {
  double sim_time;
  bool is_pilot;
  std::string text;
  std::string frequency;
};

void init();
void stop();

void on_ptt_pressed();
void on_ptt_released();

PTTState ptt_state();
std::string ptt_state_label();

// Last recording info (populated after stop_recording)
float last_recording_duration();
size_t last_recording_samples();
size_t last_wav_bytes();

// Last parsed intent
const intent_parser::PilotMessage &last_pilot_message();

// Transcript access
const std::vector<TranscriptEntry> &transcript_entries();
void clear_transcript();

} // namespace atc_session

#endif // ATC_SESSION_HPP
