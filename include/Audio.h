#pragma once

#if defined(BOARD_JC4880P443C_I_W)

#include <Arduino.h>
#include <FS.h>

class Audio {
public:
    Audio(bool, uint8_t, uint8_t) {}

    void setBufsize(int, int) {}
    void forceMono(bool) {}
    void setVolume(uint8_t) {}
    void stopSong() {}
    bool connecttoFS(fs::FS &, const char *) { return false; }
    bool isRunning() const { return false; }
    void pauseResume() {}
    void loop() {}
    uint32_t getAudioCurrentTime() const { return 0; }
    uint32_t getAudioFileDuration() const { return 0; }
};

#else

#include_next <Audio.h>

#endif