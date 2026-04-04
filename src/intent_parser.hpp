#ifndef INTENT_PARSER_HPP
#define INTENT_PARSER_HPP

#include "xplane_context.hpp"

#include <string>

namespace intent_parser {

enum class PilotIntent {
  UNKNOWN,
  INITIAL_CALL,
  REQUEST_TAXI,
  READY_FOR_DEPARTURE,
  REPORT_POSITION,
  REQUEST_LANDING,
  RUNWAY_VACATED,
  READBACK,
  REQUEST_FREQUENCY,
  UNABLE,
  SELF_ANNOUNCE,
};

struct PilotMessage {
  std::string raw_transcript;
  PilotIntent intent = PilotIntent::UNKNOWN;
  float confidence = 0.0f;
  std::string callsign;
  std::string runway;
};

void init();
void stop();

PilotMessage parse(const std::string &transcript,
                   const xplane_context::XPlaneContext &ctx);

const char *intent_name(PilotIntent intent);

} // namespace intent_parser

#endif // INTENT_PARSER_HPP
