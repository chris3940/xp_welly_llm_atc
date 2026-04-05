#ifndef PTT_INPUT_HPP
#define PTT_INPUT_HPP

namespace ptt_input {

void init();
void stop();

// Called every flight loop frame — joystick button polling + capture
void update();

// Capture mode for UI binding
enum class CaptureMode { NONE, KEYBOARD, JOYSTICK };

void start_capture(CaptureMode mode);
void cancel_capture();
CaptureMode capture_mode();
float capture_elapsed(); // seconds since capture started (for timeout)

// Returns captured value (VK or button index), -1 if still waiting
// Resets capture mode to NONE after returning a valid result
int poll_capture_result();

} // namespace ptt_input

#endif // PTT_INPUT_HPP
