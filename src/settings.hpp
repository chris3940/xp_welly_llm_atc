#ifndef SETTINGS_HPP
#define SETTINGS_HPP

#include <string>

namespace settings {

void init();
void stop();
void save();

// Keychain operations
bool save_api_key(const std::string &key);
std::string load_api_key();
void delete_api_key();
std::string get_api_key();

// Getters
bool api_key_saved();
int ptt_key_vk();
int ptt_joystick_button();
std::string tts_voice();
std::string tts_model();
std::string whisper_model();
std::string gpt_model();
bool gpt_fallback_enabled();
std::string pilot_callsign();
int active_com();
float volume();
bool debug_logging();
std::string audio_output_device();

// Setters
void set_tts_voice(const std::string &v);
std::string pilot_callsign_raw();
void set_pilot_callsign_raw(const std::string &raw);
std::string to_icao_phonetic(const std::string &raw);
void set_volume(float v);
void set_gpt_fallback_enabled(bool v);
void set_debug_logging(bool v);
void set_active_com(int com);
void set_audio_output_device(const std::string &uid);
void set_ptt_key_vk(int vk);
void set_ptt_joystick_button(int btn);

// Window geometry (-1 = use default/center)
float window_x();
float window_y();
float window_w();
float window_h();
void set_window_geometry(float x, float y, float w, float h);
void reset_window_geometry();

} // namespace settings

#endif // SETTINGS_HPP
