#include "ptt_input.hpp"
#include "atc_session.hpp"
#include "settings.hpp"

#include <XPLMDisplay.h>
#include <XPLMUtilities.h>

#include <imgui.h>

namespace ptt_input {

static bool registered_ = false;
static bool key_held_    = false;

static int key_sniffer(char key, XPLMKeyFlags flags, char virtual_key, void* /*refcon*/) {
    int ptt_vk = settings::ptt_key_vk();
    if (ptt_vk < 0) return 1;  // PTT not configured, pass through

    // Don't capture keys when an ImGui text input is focused
    if (ImGui::GetCurrentContext() && ImGui::GetIO().WantTextInput) return 1;

    if (static_cast<int>(static_cast<unsigned char>(virtual_key)) != ptt_vk) return 1;

    bool is_down = (flags & xplm_DownFlag) != 0;
    bool is_up   = (flags & xplm_UpFlag) != 0;

    if (is_down && !key_held_) {
        key_held_ = true;
        atc_session::on_ptt_pressed();
    } else if (is_up && key_held_) {
        key_held_ = false;
        atc_session::on_ptt_released();
    }

    return 1;  // pass key through to other plugins
}

void init() {
    int ptt_vk = settings::ptt_key_vk();
    if (ptt_vk < 0) {
        XPLMDebugString("[xp_wellys_atc] Warning: PTT key not configured (ptt_key_vk=-1)\n");
    } else {
        char buf[128];
        std::snprintf(buf, sizeof(buf),
                      "[xp_wellys_atc] PTT key configured: VK code %d\n", ptt_vk);
        XPLMDebugString(buf);
    }

    XPLMRegisterKeySniffer(key_sniffer, 1 /* before windows */, nullptr);
    registered_ = true;
    key_held_ = false;
}

void stop() {
    if (registered_) {
        XPLMUnregisterKeySniffer(key_sniffer, 1, nullptr);
        registered_ = false;
    }
    key_held_ = false;
}

}  // namespace ptt_input
