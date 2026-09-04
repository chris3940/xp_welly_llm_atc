// Every exit of process_transcript must complete the transmission.
//
// A bare `return` in the handoff-readback suppression path dropped the
// completion callback: atc_session was never told the transcript had been
// handled, stayed in PROCESSING, and refused every later push-to-talk. Two
// arrivals into Innsbruck were flown with a dead radio (2026-08-28, 2026-08-29)
// before the PTT state trace showed which state it was stuck in.
// [C. P. Potter]
#include "atc/atc_state_machine.hpp"
#include "atc/engine.hpp"
#include "atc/intent_parser.hpp"
#include "atc/flight_phase.hpp"
#include "atc/atc_templates.hpp"
#include "core/xplane_context.hpp"
#include <catch2/catch_amalgamated.hpp>

TEST_CASE("the handoff readback completes the transmission",
          "[engine][done][handoff]") {
  atc_templates::init();
  flight_phase::init();
  intent_parser::init();
  atc_state_machine::init();
  engine::reset();

  xplane_context::XPlaneContext ctx;
  ctx.nearest_airport_id = "LOWI";
  ctx.latitude = 47.48; ctx.longitude = 12.17;
  ctx.altitude_ft_msl = 15600.0f; ctx.pressure_alt_ft = 15000.0f;
  ctx.on_ground = false; ctx.groundspeed_kts = 280.0f;
  ctx.com1_freq_mhz = 118.525f;   // the pilot is STILL on the old frequency
  ctx.active_com = 1;

  // A handoff is outstanding to 128.975 and the pilot reads it back.
  engine::set_pending_handoff_freq(128.975f);
  engine::set_sector_checkin_pending(true);   // he has not called the new sector yet
  atc_state_machine::set_state(atc_state_machine::ATCState::IFR_APPROACH_CONTACT);

  engine::Input in{};
  in.transcript = "one-two-eight decimal nine seven five in November, Romeo Charlie.";
  in.pilot_callsign = "November Romeo Charlie";
  in.quality = 1.0f;
  in.ctx = &ctx;
  in.now_secs = 0.0;

  bool completed = false;
  engine::process_transcript(in, [&](engine::Output o) {
    completed = true;
    (void)o;
  });

  // The reply may well be silence -- reading back is not staying, and ATC owes
  // nothing. But the callback MUST fire, or the radio dies.
  REQUIRE(completed);

  atc_state_machine::stop();
  intent_parser::stop();
  flight_phase::stop();
}
