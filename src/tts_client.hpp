#ifndef TTS_CLIENT_HPP
#define TTS_CLIENT_HPP

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace tts_client {

void init();
void stop();

void speak_async(
    const std::string &text,
    std::function<void(std::vector<uint8_t> mp3_data, bool success)> callback);

// Called from flight loop to drain pending callbacks on main thread
void drain_callback_queue();

} // namespace tts_client

#endif // TTS_CLIENT_HPP
