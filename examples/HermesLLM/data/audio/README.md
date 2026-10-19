# MAVLink notification audio

Place pre-generated 16-bit PCM WAV files here and upload the SPIFFS image.

Expected file names:

- `mav_arm.wav`
- `mav_disarm.wav`
- `mav_armed.wav`
- `mav_disarmed.wav`
- `mav_mode_manual.wav`
- `mav_mode_acro.wav`
- `mav_mode_steering.wav`
- `mav_mode_hold.wav`
- `mav_mode_loiter.wav`
- `mav_mode_follow.wav`
- `mav_mode_simple.wav`
- `mav_mode_dock.wav`
- `mav_mode_circle.wav`
- `mav_mode_auto.wav`
- `mav_mode_rtl.wav`
- `mav_mode_smart_rtl.wav`
- `mav_mode_guided.wav`
- `mav_mode_initialising.wav`
- `mav_mode_autotune.wav`
- `mav_mode_land.wav`

If a file is missing, the firmware falls back to the existing TTS path.

## Presentation greetings (slide mode)

- `intro.wav` - played by the first double tap on the head
- `outro.wav` - played by the next double tap (then back to `intro.wav`, alternating)

Format: 16-bit PCM, mono, 24 kHz WAV. Only active while Slide mode is ON; otherwise a double tap behaves as before.
