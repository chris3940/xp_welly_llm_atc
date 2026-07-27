// Tests for IFR departure handoff (poll_departure_handoff) and the
// frequency guard in process_transcript.
//
// Constraints verified:
//   1. Departure handoff does NOT fire below CTR upper altitude (2481 ft AGL
//      for LFLP — lower than the 2500 ft fallback, so LFLP hands off first).
//   2. Frequency guard: wrong frequency silently drops the transcript.
//   3. Frequency guard: IFR_EN_ROUTE rejects TOWER frequency.
//   4. Frequency guard: IFR_EN_ROUTE accepts APPROACH frequency.

#include "atc/atc_state_machine.hpp"
#include "atc/engine.hpp"
#include "atc/flight_phase.hpp"
#include "atc/intent_parser.hpp"
#include "core/xplane_context.hpp"
#include "data/openair_db.hpp"

#include <catch2/catch_amalgamated.hpp>

using atc_state_machine::ATCState;
using xplane_context::AirportFrequency;
using xplane_context::FrequencyType;
using xplane_context::XPlaneContext;

// LFLP Annecy Meythet: elevation 1519 ft MSL, ANNECY CTR upper 4000 ft MSL.
static constexpr float LFLP_ELEVATION_FT     = 1519.0f;
static constexpr float LFLP_CTR_UPPER_MSL_FT = 4000.0f;
static constexpr float LFLP_CTR_AGL          = LFLP_CTR_UPPER_MSL_FT - LFLP_ELEVATION_FT; // 2481 ft

namespace {

XPlaneContext make_ifr_ctx(float height_agl) {
    XPlaneContext ctx{};
    ctx.nearest_airport_id   = "LFLP";
    ctx.nearest_airport_name = "ANNECY MEYTHET";
    ctx.airport_lat = 45.9292;
    ctx.airport_lon = 6.1028;
    ctx.latitude    = 45.9292;
    ctx.longitude   = 6.1028;

    ctx.height_agl_ft    = height_agl;
    ctx.altitude_ft_msl  = LFLP_ELEVATION_FT + height_agl;
    ctx.is_towered_airport = true;
    ctx.on_ground = (height_agl < 1.0f);
    ctx.groundspeed_kts = 120.0f;

    // LFLP: APPROACH on 121.200, TOWER on 118.200
    AirportFrequency app{};
    app.freq_khz = 121200;
    app.type = FrequencyType::APPROACH;
    app.name = "CHAMBERY APP";
    ctx.airport_freqs.all.push_back(app);

    AirportFrequency twr{};
    twr.freq_khz = 118200;
    twr.type = FrequencyType::TOWER;
    twr.name = "ANNECY TWR";
    ctx.airport_freqs.all.push_back(twr);

    ctx.frequency_type = FrequencyType::TOWER; // default: on Tower
    return ctx;
}

} // namespace

// ── Altitude threshold sanity ─────────────────────────────────────────────

TEST_CASE("ifr departure: LFLP CTR upper altitude is below 2500 ft AGL fallback",
          "[ifr_departure]")
{
    // ANNECY CTR upper 4000 ft MSL, elevation 1519 ft => 2481 ft AGL.
    // This means if the plugin uses the OpenAir ceiling instead of the 2500 ft
    // fallback, the handoff fires ~19 ft AGL earlier — confirming the OpenAir
    // path is more accurate than the fallback.
    REQUIRE(LFLP_CTR_AGL < 2500.0f);
    REQUIRE(LFLP_CTR_AGL == Catch::Approx(2481.0f).margin(1.0f));
}

// ── Departure handoff altitude guard ─────────────────────────────────────

TEST_CASE("ifr departure: handoff does not fire below the report altitude",
          "[ifr_departure]")
{
    engine::reset();
    flight_phase::init();
    atc_state_machine::init();
    openair_db::init(""); // disabled — no file in unit tests

    atc_state_machine::set_state(ATCState::IFR_DEPARTURE_CLEARED);

    // Report-then-transfer mode (EU IFR tower_report_alt_ft = 3000 ft MSL): the
    // Tower keeps the aircraft until it passes the report altitude, THEN hands
    // off. Below it -> handoff must NOT fire. LFLP elevation 1519 ft, so 1400 ft
    // AGL = 2919 ft MSL, just under the 3000 ft report altitude. (This gate is
    // altitude-only; the phase guard was dropped 2026-07-26 because the geometric
    // detector mislabels a low, turning IFR climb-out as PATTERN/FINAL_APPROACH
    // and used to block the handoff for ~1 min at LIMF -- the pilot's repeated
    // "reaching 2000 feet" reports met silence.)
    XPlaneContext ctx = make_ifr_ctx(1400.0f);
    std::string text;
    bool fired = engine::poll_departure_handoff(ctx, 0.0f, &text);
    REQUIRE_FALSE(fired);
    // State unchanged
    REQUIRE(atc_state_machine::get_state() == ATCState::IFR_DEPARTURE_CLEARED);

    atc_state_machine::stop();
    flight_phase::stop();
    openair_db::stop();
}

// ── Frequency guard ───────────────────────────────────────────────────────

TEST_CASE("freq guard: APPROACH frequency rejected in GROUND_CONTACT state",
          "[ifr_departure][freq_guard]")
{
    engine::reset();
    atc_state_machine::init();
    intent_parser::init();
    flight_phase::init();
    openair_db::init("");

    atc_state_machine::set_state(ATCState::GROUND_CONTACT);

    XPlaneContext ctx = make_ifr_ctx(0.0f);
    ctx.on_ground = true;
    ctx.frequency_type = FrequencyType::APPROACH; // wrong for ground state

    engine::Input in{};
    in.transcript      = "Annecy Tower, November 111, ready for departure";
    in.pilot_callsign  = "November 111";
    in.quality         = 0.85f;
    in.ctx             = &ctx;
    in.now_secs        = 0.0;

    engine::Output out;
    engine::process_transcript(in, [&](engine::Output o) { out = std::move(o); });

    // Wrong frequency -> silent drop: no response text, state unchanged
    REQUIRE(out.response_text.empty());
    REQUIRE(atc_state_machine::get_state() == ATCState::GROUND_CONTACT);

    atc_state_machine::stop();
    intent_parser::stop();
    flight_phase::stop();
    openair_db::stop();
}

TEST_CASE("freq guard: TOWER frequency rejected in IFR_EN_ROUTE state",
          "[ifr_departure][freq_guard]")
{
    engine::reset();
    atc_state_machine::init();
    intent_parser::init();
    flight_phase::init();
    openair_db::init("");

    atc_state_machine::set_state(ATCState::IFR_EN_ROUTE);

    XPlaneContext ctx = make_ifr_ctx(3000.0f);
    ctx.frequency_type = FrequencyType::TOWER; // wrong: en-route needs APPROACH

    engine::Input in{};
    in.transcript     = "Chambery Approach, November 111, passing 3000";
    in.pilot_callsign = "November 111";
    in.quality        = 0.85f;
    in.ctx            = &ctx;
    in.now_secs       = 0.0;

    engine::Output out;
    engine::process_transcript(in, [&](engine::Output o) { out = std::move(o); });

    REQUIRE(out.response_text.empty());
    REQUIRE(atc_state_machine::get_state() == ATCState::IFR_EN_ROUTE);

    atc_state_machine::stop();
    intent_parser::stop();
    flight_phase::stop();
    openair_db::stop();
}

TEST_CASE("freq guard: APPROACH frequency accepted in IFR_EN_ROUTE state",
          "[ifr_departure][freq_guard]")
{
    engine::reset();
    atc_state_machine::init();
    intent_parser::init();
    flight_phase::init();
    openair_db::init("");

    atc_state_machine::set_state(ATCState::IFR_EN_ROUTE);

    XPlaneContext ctx = make_ifr_ctx(3000.0f);
    ctx.frequency_type = FrequencyType::APPROACH; // correct for en-route

    engine::Input in{};
    in.transcript     = "Chambery Approach, November 111, passing 3000";
    in.pilot_callsign = "November 111";
    in.quality        = 0.85f;
    in.ctx            = &ctx;
    in.now_secs       = 0.0;

    engine::Output out;
    engine::process_transcript(in, [&](engine::Output o) { out = std::move(o); });

    // Correct frequency -> ATC replies with something
    REQUIRE_FALSE(out.response_text.empty());

    atc_state_machine::stop();
    intent_parser::stop();
    flight_phase::stop();
    openair_db::stop();
}

// ── Pending-handoff bypass (Fix B v2, v4.3.1) ─────────────────────────────

// The plugin issues intermediate handoffs like "contact Milan on 118.675"
// during SID climb (poll_sid_climb Phase 2.8) to controllers whose frequency
// lives in atc.dat only (CTR role — no matching apt.dat entry). Result:
// ctx.frequency_type = UNKNOWN even though the pilot is legitimately on the
// frequency the plugin just told them to switch to. Without a bypass, the
// wrong-freq guard silently drops the check-in transmission — reproduced
// in-sim by pilot's three unanswered calls to Milan on 118.675 (2026-07-07).

TEST_CASE("freq guard: pending-handoff freq bypasses UNKNOWN drop "
          "in IFR_RADAR_CONTACT",
          "[ifr_departure][freq_guard][pending_handoff]")
{
    engine::reset();
    atc_state_machine::init();
    intent_parser::init();
    flight_phase::init();
    openair_db::init("");

    atc_state_machine::set_state(ATCState::IFR_RADAR_CONTACT);

    // Simulate Phase 2.8 having fired: "contact Milan on 118.675".
    engine::set_pending_handoff_freq(118.675f);

    XPlaneContext ctx = make_ifr_ctx(14000.0f);
    ctx.com1_freq_mhz  = 118.675f;   // pilot switched to the assigned freq
    ctx.com2_freq_mhz  = 121.100f;   // (previous Approach)
    ctx.active_com     = 1;
    ctx.frequency_type = FrequencyType::UNKNOWN; // atc.dat-only CTR freq

    engine::Input in{};
    in.transcript     = "Milan Radar, November 750XP, climbing to flight level 210";
    in.pilot_callsign = "N750XP";
    in.quality        = 0.85f;
    in.ctx            = &ctx;
    in.now_secs       = 0.0;

    engine::Output out;
    engine::process_transcript(in, [&](engine::Output o) { out = std::move(o); });

    // Bypass fired → some ATC response emitted (not the silent drop).
    INFO("Response was: '" << out.response_text << "'");
    REQUIRE_FALSE(out.response_text.empty());

    atc_state_machine::stop();
    intent_parser::stop();
    flight_phase::stop();
    openair_db::stop();
}

// In-sim regression: after Phase 2.8 fires "contact Milan on 118.675" at
// 24:37, the pilot READS BACK the instruction at 24:55 while still on the
// OLD frequency (121.100), then switches to 118.675 and checks in at 25:26.
// The readback on the OLD freq must NOT clear s_pending_handoff_freq_mhz
// or s_sector_checkin_pending, otherwise the subsequent check-in on the
// NEW freq gets dropped by the wrong-freq guard. This reproduces the
// LIMF→LFLP 2026-07-07 in-sim regression: 3 unanswered check-ins.

TEST_CASE("pending handoff survives readback on OLD freq before switch",
          "[ifr_departure][freq_guard][pending_handoff]")
{
    engine::reset();
    atc_state_machine::init();
    intent_parser::init();
    flight_phase::init();
    openair_db::init("");

    atc_state_machine::set_state(ATCState::IFR_RADAR_CONTACT);
    engine::set_pending_handoff_freq(118.675f);
    REQUIRE(engine::pending_handoff_freq() == Catch::Approx(118.675f));

    // Step 1: pilot reads back on OLD Torino APP freq 121.100.
    XPlaneContext ctx_readback = make_ifr_ctx(12000.0f);
    ctx_readback.com1_freq_mhz  = 121.100f;
    ctx_readback.active_com     = 1;
    ctx_readback.frequency_type = FrequencyType::APPROACH; // known APP freq

    engine::Input in_rb{};
    in_rb.transcript     = "Contact Milan on 118.675, November 750XP";
    in_rb.pilot_callsign = "N750XP";
    in_rb.quality        = 0.85f;
    in_rb.ctx            = &ctx_readback;

    engine::Output out_rb;
    engine::process_transcript(in_rb, [&](engine::Output o) { out_rb = std::move(o); });

    // ═══ Critical assertion: readback processing MUST NOT overwrite the
    // ═══ pending handoff (previously Phase 2.8 set it to 118.675). If this
    // ═══ fails, a template lambda somewhere in build_vars() is calling
    // ═══ set_pending_handoff_freq with the airport's own APP freq — the
    // ═══ LIMF→LFLP 2026-07-07 in-sim regression root cause.
    INFO("Pending handoff after readback: " << engine::pending_handoff_freq());
    REQUIRE(engine::pending_handoff_freq() == Catch::Approx(118.675f));

    // Step 2: pilot switches to 118.675 and checks in.
    XPlaneContext ctx_checkin = make_ifr_ctx(14000.0f);
    ctx_checkin.com1_freq_mhz  = 118.675f;
    ctx_checkin.active_com     = 1;
    ctx_checkin.frequency_type = FrequencyType::UNKNOWN; // Milan Radar CTR

    // After the CTX change, the pending handoff MUST still be 118.675
    // (nothing between step 1 and step 2 touches it).
    REQUIRE(engine::pending_handoff_freq() == Catch::Approx(118.675f));

    engine::Input in_ck{};
    in_ck.transcript     = "Milan Radar, November 750XP, climbing FL210";
    in_ck.pilot_callsign = "N750XP";
    in_ck.quality        = 0.85f;
    in_ck.ctx            = &ctx_checkin;

    engine::Output out_ck;
    engine::process_transcript(in_ck, [&](engine::Output o) { out_ck = std::move(o); });

    // The check-in on the NEW freq must NOT be silently dropped.
    INFO("Check-in response: '" << out_ck.response_text << "'");
    INFO("Pending handoff after check-in: " << engine::pending_handoff_freq());
    REQUIRE_FALSE(out_ck.response_text.empty());

    atc_state_machine::stop();
    intent_parser::stop();
    flight_phase::stop();
    openair_db::stop();
}

TEST_CASE("freq guard: pending-handoff bypass does NOT apply to a mismatched "
          "frequency (guard still drops truly-wrong freq)",
          "[ifr_departure][freq_guard][pending_handoff]")
{
    engine::reset();
    atc_state_machine::init();
    intent_parser::init();
    flight_phase::init();
    openair_db::init("");

    atc_state_machine::set_state(ATCState::IFR_RADAR_CONTACT);

    // Plugin issued handoff to 118.675, but pilot tuned 121.100 instead.
    engine::set_pending_handoff_freq(118.675f);

    XPlaneContext ctx = make_ifr_ctx(14000.0f);
    ctx.com1_freq_mhz  = 121.100f;   // WRONG — not the pending freq
    ctx.active_com     = 1;
    ctx.frequency_type = FrequencyType::UNKNOWN;

    engine::Input in{};
    in.transcript     = "Milan Radar, November 111, climbing to flight level 210";
    in.pilot_callsign = "November 111";
    in.quality        = 0.85f;
    in.ctx            = &ctx;
    in.now_secs       = 0.0;

    engine::Output out;
    engine::process_transcript(in, [&](engine::Output o) { out = std::move(o); });

    // Bypass must NOT fire — the guard should still silently drop this call
    // (pilot on the wrong frequency, not the one the plugin handed them off to).
    REQUIRE(out.response_text.empty());

    atc_state_machine::stop();
    intent_parser::stop();
    flight_phase::stop();
    openair_db::stop();
}
