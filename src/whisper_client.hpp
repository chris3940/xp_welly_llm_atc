#ifndef WHISPER_CLIENT_HPP
#define WHISPER_CLIENT_HPP

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace whisper_client {

void init();
void stop();

void transcribe_async(
    std::vector<uint8_t> wav_data,
    std::function<void(std::string transcript, bool success)> callback);

// Called from flight loop to drain pending callbacks on main thread
void drain_callback_queue();

} // namespace whisper_client

#endif // WHISPER_CLIENT_HPP
