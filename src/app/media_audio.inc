void mediaNormalizeDir(String &dir)
{
    if (dir.isEmpty()) dir = "/";
    if (!dir.startsWith("/")) dir = "/" + dir;
    while (dir.length() > 1 && dir.endsWith("/")) dir.remove(dir.length() - 1);
}

String mediaDisplayNameFromPath(const String &path)
{
    const int slash = path.lastIndexOf('/');
    if (slash < 0) return path;
    return path.substring(slash + 1);
}

String mediaDirFromPath(const String &path)
{
    const int slash = path.lastIndexOf('/');
    if (slash <= 0) return "/";
    String dir = path.substring(0, slash);
    mediaNormalizeDir(dir);
    return dir;
}

uint8_t audioVolumeLevelFromPercent(uint8_t percent)
{
    if (percent > 100) percent = 100;
    if (percent == 0) return 0;
    const float normalized = static_cast<float>(percent) / 100.0f;
    const float curved = powf(normalized, AUDIO_VOLUME_CURVE_EXPONENT);
    const long mapped = lroundf(curved * static_cast<float>(AUDIO_VOLUME_TARGET));
    return static_cast<uint8_t>(constrain(mapped, 1L, static_cast<long>(AUDIO_VOLUME_TARGET)));
}

void mediaFormatSeconds(uint32_t sec, char *out, size_t outLen)
{
    if (!out || outLen == 0) return;
    const uint32_t mins = sec / 60U;
    const uint32_t secs = sec % 60U;
    snprintf(out, outLen, "%02lu:%02lu", static_cast<unsigned long>(mins), static_cast<unsigned long>(secs));
}

String mediaFindAdjacentTrack(const String &sourcePath, bool nextDir)
{
    String dir = sourcePath.isEmpty() ? mediaCurrentDir : mediaDirFromPath(sourcePath);
    mediaNormalizeDir(dir);
    String currentKey = mediaDisplayNameFromPath(sourcePath);
    currentKey.toLowerCase();
    const bool hasCurrent = !sourcePath.isEmpty();

    if (!sdEnsureMounted() && !sdEnsureMounted(true)) return "";
    if (!sdLock()) return "";

    String firstPath;
    String firstKey;
    String lastPath;
    String lastKey;
    String bestPath;
    String bestKey;
    int scanned = 0;

    File folder = SD.open(dir);
    if (!folder || !folder.isDirectory()) {
        if (folder) folder.close();
        sdUnlock();
        return "";
    }

    File entry = folder.openNextFile();
    while (entry) {
        const char *rawName = entry.name();
        const char *baseName = mediaBaseNamePtr(rawName);
        if (!entry.isDirectory() && mediaAllowedExtC(baseName)) {
            String fullPath = mediaBuildFullPath(dir, rawName);
            if (!fullPath.isEmpty()) {
                String key(baseName);
                key.toLowerCase();

                if (firstPath.isEmpty() || key.compareTo(firstKey) < 0) {
                    firstPath = fullPath;
                    firstKey = key;
                }
                if (lastPath.isEmpty() || key.compareTo(lastKey) > 0) {
                    lastPath = fullPath;
                    lastKey = key;
                }

                if (hasCurrent) {
                    const int cmp = key.compareTo(currentKey);
                    if (nextDir) {
                        if (cmp > 0 && (bestPath.isEmpty() || key.compareTo(bestKey) < 0)) {
                            bestPath = fullPath;
                            bestKey = key;
                        }
                    } else {
                        if (cmp < 0 && (bestPath.isEmpty() || key.compareTo(bestKey) > 0)) {
                            bestPath = fullPath;
                            bestKey = key;
                        }
                    }
                }
            }
        }

        entry.close();
        entry = folder.openNextFile();
        scanned++;
        if ((scanned & 0x0F) == 0) delay(0);
    }
    if (entry) entry.close();
    folder.close();
    sdUnlock();

    if (!bestPath.isEmpty()) return bestPath;
    return nextDir ? firstPath : lastPath;
}

void rgbApplyNow(bool rOn, bool gOn, bool bOn)
{
    if (!RGB_OUTPUT_SUPPORTED) return;
    auto level = [](bool on) -> uint8_t {
        const uint8_t scaled = on ? static_cast<uint8_t>((static_cast<uint16_t>(255U) * rgbLedPercent) / 100U) : 0U;
        return RGB_ACTIVE_LOW ? static_cast<uint8_t>(255U - scaled) : scaled;
    };
    // Board LED channels are physically swapped: logical R<->G.
    analogWrite(RGB_PIN_R, level(gOn));
    analogWrite(RGB_PIN_G, level(rOn));
    analogWrite(RGB_PIN_B, level(bOn));
}

void rgbSetPersistent(bool rOn, bool gOn, bool bOn)
{
    rgbPersistR = rOn;
    rgbPersistG = gOn;
    rgbPersistB = bOn;
    if (rgbFlashUntilMs == 0) rgbApplyNow(rOn, gOn, bOn);
}

void rgbFlashAcknowledge()
{
    rgbApplyNow(true, true, true);
    rgbFlashUntilMs = millis() + 150;
}

void rgbApplyPwm(uint8_t r, uint8_t g, uint8_t b)
{
    if (!RGB_OUTPUT_SUPPORTED) return;
    auto out = [](uint8_t val) -> uint8_t {
        const uint8_t scaled = static_cast<uint8_t>((static_cast<uint16_t>(val) * rgbLedPercent) / 100U);
        return RGB_ACTIVE_LOW ? static_cast<uint8_t>(255U - scaled) : scaled;
    };
    // Board LED channels are physically swapped: logical R<->G.
    analogWrite(RGB_PIN_R, out(g));
    analogWrite(RGB_PIN_G, out(r));
    analogWrite(RGB_PIN_B, out(b));
}

void rgbService()
{
    if (!batteryCharging && batteryPercent < 5) {
        unsigned long now = millis();
        if (now - rgbPulseLastMs >= 20) {
            rgbPulseLastMs = now;
            int v = static_cast<int>(rgbPulseLevel) + (rgbPulseDir * 5);
            if (v >= 255) {
                v = 255;
                rgbPulseDir = -1;
            } else if (v <= 20) {
                v = 20;
                rgbPulseDir = 1;
            }
            rgbPulseLevel = static_cast<uint8_t>(v);
        }
        rgbApplyPwm(rgbPulseLevel, 0, 0);
        return;
    }
    if (batteryCharging) {
        if (batteryPercent >= 98) {
            // On this board revision, physical GREEN is on logical B channel.
            rgbApplyNow(false, false, true);
            return;
        }
        unsigned long now = millis();
        if (now - rgbPulseLastMs >= 20) {
            rgbPulseLastMs = now;
            int v = static_cast<int>(rgbPulseLevel) + (rgbPulseDir * 5);
            if (v >= 255) {
                v = 255;
                rgbPulseDir = -1;
            } else if (v <= 20) {
                v = 20;
                rgbPulseDir = 1;
            }
            rgbPulseLevel = static_cast<uint8_t>(v);
        }
        // On this board revision, physical GREEN is on logical B channel.
        rgbApplyPwm(0, 0, rgbPulseLevel);
        return;
    }
    if (!displayAwake) {
        rgbApplyNow(false, false, false);
        return;
    }
    if (rgbFlashUntilMs != 0) return;
    if (mediaIsPlaying && !mediaPaused) {
        unsigned long now = millis();
        if (now - rgbPulseLastMs >= 20) {
            rgbPulseLastMs = now;
            int v = static_cast<int>(rgbPulseLevel) + (rgbPulseDir * 5);
            if (v >= 255) {
                v = 255;
                rgbPulseDir = -1;
            } else if (v <= 20) {
                v = 20;
                rgbPulseDir = 1;
            }
            rgbPulseLevel = static_cast<uint8_t>(v);
        }
        rgbApplyPwm(0, rgbPulseLevel, 0);
        return;
    }
    rgbApplyNow(rgbPersistR, rgbPersistG, rgbPersistB);
}

void rgbRefreshByMediaState()
{
    if (mediaIsPlaying && !mediaPaused) {
        rgbSetPersistent(false, true, false); // Green: playing
    } else {
        rgbSetPersistent(true, false, false); // Red: idle/stopped
    }
}

static inline bool chatMessageBeepCanPlay()
{
    return !mediaIsPlaying && !mediaPaused;
}

static void chatMessageBeepRampDuty(uint32_t fromDuty, uint32_t toDuty, uint16_t rampMs)
{
    if (!chatMessageBeepPinAttached) return;

    const uint8_t steps = 8;

    if (rampMs == 0 || fromDuty == toDuty) {
        ledcWrite(CHAT_NOTIFY_LEDC_CHANNEL, toDuty);
        return;
    }

    const uint16_t stepDelay = (rampMs >= steps) ? (rampMs / steps) : 0;
    const int32_t delta = (int32_t)toDuty - (int32_t)fromDuty;

    for (uint8_t i = 1; i <= steps; ++i) {
        uint32_t duty = fromDuty + (delta * i) / steps;
        ledcWrite(CHAT_NOTIFY_LEDC_CHANNEL, duty);

        if (stepDelay > 0) delay(stepDelay);
    }

    // guarantee exact final value
    ledcWrite(CHAT_NOTIFY_LEDC_CHANNEL, toDuty);
}

static inline uint32_t chatMessageBeepDuty()
{
    if (mediaVolumePercent == 0) return 0;
    const float normalized = static_cast<float>(mediaVolumePercent) / 100.0f;
    const float curved = powf(normalized, AUDIO_VOLUME_CURVE_EXPONENT);
    const long mapped = lroundf(curved * 1023.0f);
    return static_cast<uint32_t>(constrain(mapped, 1L, 1023L));
}

static void chatMessageBeepStop(bool clearQueue)
{
    if (chatMessageBeepPinAttached) {
        const uint32_t duty = chatMessageBeepDuty();
        chatMessageBeepRampDuty(duty, 0, CHAT_NOTIFY_FADE_OUT_MS);
        ledcWriteTone(CHAT_NOTIFY_LEDC_CHANNEL, 0);
        ledcWrite(CHAT_NOTIFY_LEDC_CHANNEL, 0);

        ledcDetachPin(I2S_SPK_PIN);

        pinMode(I2S_SPK_PIN, INPUT);
        gpio_set_pull_mode((gpio_num_t)I2S_SPK_PIN, GPIO_FLOATING);

        chatMessageBeepPinAttached = false;
    }

    chatMessageBeepPhase = 0;
    chatMessageBeepDeadlineMs = 0;
    if (clearQueue) chatMessageBeepQueue = 0;
}

static bool chatMessageBeepEnsurePinAttached()
{
    if (chatMessageBeepPinAttached) return true;

    gpio_set_pull_mode((gpio_num_t)I2S_SPK_PIN, GPIO_FLOATING);
    pinMode(I2S_SPK_PIN, OUTPUT);
    digitalWrite(I2S_SPK_PIN, LOW);

    ledcSetup(CHAT_NOTIFY_LEDC_CHANNEL, CHAT_NOTIFY_FREQ_PRIMARY, 10);
    ledcAttachPin(I2S_SPK_PIN, CHAT_NOTIFY_LEDC_CHANNEL);
    ledcWrite(CHAT_NOTIFY_LEDC_CHANNEL, 0);

    chatMessageBeepPinAttached = true;
    return true;
}

static void chatMessageBeepStartStep(uint8_t stepIndex)
{
    const ChatBeepPattern pattern = chatMessageBeepPattern(messageBeepTone);
    if (stepIndex >= pattern.count || pattern.count == 0) {
        if (chatMessageBeepQueue > 0) chatMessageBeepQueue--;
        chatMessageBeepStop(false);
        chatMessageBeepDeadlineMs = millis() + CHAT_NOTIFY_REPEAT_GAP_MS;
        return;
    }

    if (!chatMessageBeepEnsurePinAttached()) return;

    const ChatBeepStep &step = pattern.steps[stepIndex];
    if (step.freq == 0) {
        const uint32_t duty = chatMessageBeepDuty();
        chatMessageBeepRampDuty(duty, 0, CHAT_NOTIFY_FADE_OUT_MS);
        ledcWriteTone(CHAT_NOTIFY_LEDC_CHANNEL, 0);
        ledcWrite(CHAT_NOTIFY_LEDC_CHANNEL, 0);
    } else {
        ledcWrite(CHAT_NOTIFY_LEDC_CHANNEL, 0);
        delayMicroseconds(150);
        ledcWriteTone(CHAT_NOTIFY_LEDC_CHANNEL, step.freq);
        chatMessageBeepRampDuty(0, chatMessageBeepDuty(), CHAT_NOTIFY_FADE_IN_MS);
    }

    chatMessageBeepPhase = static_cast<uint8_t>(stepIndex + 1U);
    chatMessageBeepDeadlineMs = millis() + step.durationMs;
}

static void chatMessageBeepPreview()
{
    if (!chatMessageBeepCanPlay()) return;
    chatMessageBeepStop(true);
    chatMessageBeepQueue = 1;
    chatMessageBeepDeadlineMs = 0;
    chatMessageBeepStartStep(0);
}

void chatQueueIncomingMessageBeep()
{
    if (!chatMessageBeepCanPlay()) return;
    if (chatMessageBeepQueue < CHAT_NOTIFY_QUEUE_MAX) chatMessageBeepQueue++;
}

void chatQueueIncomingMessageVibration()
{
#if defined(BOARD_ESP32S3_3248S035_N16R8)
    if (!vibrationEnabled) return;
    if (chatMessageVibrationQueue < VIBRATION_QUEUE_MAX) chatMessageVibrationQueue++;
#endif
}

static void touchFeedbackQueue()
{
    if (touchSoundEnabled && chatMessageBeepCanPlay()) {
        touchFeedbackBeepActive = true;
        touchFeedbackBeepDeadlineMs = 0;
    }
#if defined(BOARD_ESP32S3_3248S035_N16R8)
    if (touchVibrationEnabled && vibrationEnabled) {
        touchFeedbackVibrationActive = true;
        touchFeedbackVibrationDeadlineMs = 0;
    }
#endif
}

#if defined(BOARD_ESP32S3_3248S035_N16R8)
static uint32_t chatMessageVibrationDuty()
{
    if (!vibrationEnabled) return 0;
    switch (vibrationIntensity) {
        case VIBRATION_INTENSITY_LOW: return 96;
        case VIBRATION_INTENSITY_MEDIUM: return 168;
        case VIBRATION_INTENSITY_HIGH:
        default: return 255;
    }
}

static bool chatMessageVibrationEnsurePinAttached()
{
    if (chatMessageVibrationPinAttached) return true;
    ledcSetup(VIBRATION_LEDC_CHANNEL, VIBRATION_PWM_HZ, VIBRATION_PWM_RES_BITS);
    ledcAttachPin(VIBRATION_MOTOR_PIN, VIBRATION_LEDC_CHANNEL);
    chatMessageVibrationPinAttached = true;
    return true;
}

static void chatMessageVibrationStop(bool clearQueue)
{
    ledcWrite(VIBRATION_LEDC_CHANNEL, 0);
    if (chatMessageVibrationPinAttached) {
        ledcDetachPin(VIBRATION_MOTOR_PIN);
        pinMode(VIBRATION_MOTOR_PIN, OUTPUT);
        digitalWrite(VIBRATION_MOTOR_PIN, LOW);
        chatMessageVibrationPinAttached = false;
    }
    chatMessageVibrationPinOutput = false;
    chatMessageVibrationPhase = 0;
    chatMessageVibrationDeadlineMs = 0;
    if (clearQueue) chatMessageVibrationQueue = 0;
}

static void chatMessageVibrationStartOutput()
{
    if (!chatMessageVibrationEnsurePinAttached()) return;
    ledcWrite(VIBRATION_LEDC_CHANNEL, chatMessageVibrationDuty());
    chatMessageVibrationPinOutput = true;
}

static void chatMessageVibrationPreview()
{
    if (!vibrationEnabled) {
        chatMessageVibrationStop(true);
        return;
    }
    chatMessageVibrationStop(true);
    chatMessageVibrationQueue = 1;
    chatMessageVibrationDeadlineMs = 0;
    chatMessageVibrationService();
}
#endif

void chatMessageBeepService()
{
    if (!chatMessageBeepCanPlay()) {
        if (chatMessageBeepPhase != 0 || chatMessageBeepPinAttached) chatMessageBeepStop(true);
        return;
    }

    const unsigned long now = millis();
    if (chatMessageBeepPhase == 0) {
        if (chatMessageBeepQueue == 0) return;
        if (chatMessageBeepDeadlineMs != 0 && static_cast<long>(now - chatMessageBeepDeadlineMs) < 0) return;
        chatMessageBeepStartStep(0);
        return;
    }

    if (static_cast<long>(now - chatMessageBeepDeadlineMs) < 0) return;
    chatMessageBeepStartStep(chatMessageBeepPhase);
}

static void touchFeedbackBeepService()
{
    if (!touchFeedbackBeepActive) return;
    if (!touchSoundEnabled || !chatMessageBeepCanPlay()) {
        touchFeedbackBeepActive = false;
        touchFeedbackBeepDeadlineMs = 0;
        return;
    }
    if (chatMessageBeepPhase != 0 || chatMessageBeepQueue > 0) {
        touchFeedbackBeepActive = false;
        touchFeedbackBeepDeadlineMs = 0;
        return;
    }

    const unsigned long now = millis();
    if (touchFeedbackBeepDeadlineMs == 0) {
        if (!chatMessageBeepEnsurePinAttached()) {
            touchFeedbackBeepActive = false;
            return;
        }
        ledcWrite(CHAT_NOTIFY_LEDC_CHANNEL, 0);
        delayMicroseconds(150);
        ledcWriteTone(CHAT_NOTIFY_LEDC_CHANNEL, TOUCH_TICK_FREQ);
        ledcWrite(CHAT_NOTIFY_LEDC_CHANNEL, 88U);
        touchFeedbackBeepDeadlineMs = now + TOUCH_TICK_MS;
        return;
    }

    if (static_cast<long>(now - touchFeedbackBeepDeadlineMs) < 0) return;
    chatMessageBeepStop(false);
    touchFeedbackBeepActive = false;
    touchFeedbackBeepDeadlineMs = 0;
}

void chatMessageVibrationService()
{
#if defined(BOARD_ESP32S3_3248S035_N16R8)
    const unsigned long now = millis();
    if (chatMessageVibrationPhase == 0) {
        if (chatMessageVibrationQueue == 0) return;
        if (chatMessageVibrationDeadlineMs != 0 && static_cast<long>(now - chatMessageVibrationDeadlineMs) < 0) return;
        chatMessageVibrationStartOutput();
        chatMessageVibrationPhase = 1;
        chatMessageVibrationDeadlineMs = now + VIBRATION_PULSE_MS;
        return;
    }

    if (static_cast<long>(now - chatMessageVibrationDeadlineMs) < 0) return;

    switch (chatMessageVibrationPhase) {
        case 1:
            ledcWrite(VIBRATION_LEDC_CHANNEL, 0);
            chatMessageVibrationPhase = 2;
            chatMessageVibrationDeadlineMs = now + VIBRATION_GAP_MS;
            break;
        case 2:
            ledcWrite(VIBRATION_LEDC_CHANNEL, chatMessageVibrationDuty());
            chatMessageVibrationPhase = 3;
            chatMessageVibrationDeadlineMs = now + VIBRATION_PULSE_MS;
            break;
        case 3:
            if (chatMessageVibrationQueue > 0) chatMessageVibrationQueue--;
            chatMessageVibrationStop(false);
            break;
        default:
            chatMessageVibrationStop(true);
            break;
    }
#endif
}

#if defined(BOARD_ESP32S3_3248S035_N16R8)
static void touchFeedbackVibrationService()
{
    if (!touchFeedbackVibrationActive) return;
    if (!touchVibrationEnabled || !vibrationEnabled) {
        touchFeedbackVibrationActive = false;
        touchFeedbackVibrationDeadlineMs = 0;
        return;
    }
    if (chatMessageVibrationPhase != 0 || chatMessageVibrationQueue > 0) {
        touchFeedbackVibrationActive = false;
        touchFeedbackVibrationDeadlineMs = 0;
        return;
    }

    const unsigned long now = millis();
    if (touchFeedbackVibrationDeadlineMs == 0) {
        if (!chatMessageVibrationEnsurePinAttached()) {
            touchFeedbackVibrationActive = false;
            return;
        }
        ledcWrite(VIBRATION_LEDC_CHANNEL, TOUCH_VIBRATION_DUTY);
        touchFeedbackVibrationDeadlineMs = now + TOUCH_VIBRATION_PULSE_MS;
        return;
    }

    if (static_cast<long>(now - touchFeedbackVibrationDeadlineMs) < 0) return;
    ledcWrite(VIBRATION_LEDC_CHANNEL, 0);
    ledcDetachPin(VIBRATION_MOTOR_PIN);
    pinMode(VIBRATION_MOTOR_PIN, OUTPUT);
    digitalWrite(VIBRATION_MOTOR_PIN, LOW);
    chatMessageVibrationPinAttached = false;
    chatMessageVibrationPinOutput = false;
    touchFeedbackVibrationActive = false;
    touchFeedbackVibrationDeadlineMs = 0;
}
#endif

bool audioEnsureBackendReady(const char *reason)
{
#if defined(BOARD_ESP32S3_3248S035_N16R8)
    Serial.printf("[AUDIO] backend unavailable on this build: GPIO %d mapped but internal DAC mode is ESP32-only reason=%s\n",
                  I2S_SPK_PIN,
                  (reason && reason[0]) ? reason : "-");
    return false;
#else
    if (audioBackendReady && audio) return true;
    const unsigned long now = millis();
    if (static_cast<unsigned long>(now - audioLastInitAttemptMs) < AUDIO_INIT_RETRY_MS) return false;
    audioLastInitAttemptMs = now;

    const uint32_t freeHeap = static_cast<uint32_t>(ESP.getFreeHeap());
    const uint32_t dmaLargest = static_cast<uint32_t>(heap_caps_get_largest_free_block(MALLOC_CAP_DMA));
    if (freeHeap < AUDIO_INIT_MIN_FREE_HEAP || dmaLargest < AUDIO_INIT_MIN_DMA_BLOCK) {
        Serial.printf("[AUDIO] init skipped: heap=%lu dma=%lu reason=%s\n",
                      static_cast<unsigned long>(freeHeap),
                      static_cast<unsigned long>(dmaLargest),
                      (reason && reason[0]) ? reason : "-");
        return false;
    }

    Audio *created = new (std::nothrow) Audio(true, AUDIO_INTERNAL_DAC_CHANNEL, AUDIO_I2S_PORT);
    if (!created) {
        Serial.printf("[AUDIO] init failed: new Audio null reason=%s\n", (reason && reason[0]) ? reason : "-");
        return false;
    }

    audio = created;
    audio->setBufsize(AUDIO_INPUT_RAM_BUFFER_BYTES, -1);
    audio->forceMono(AUDIO_FORCE_MONO_INTERNAL_DAC);
    audioBackendReady = true;
    audio->setVolume(audioVolumeLevelFromPercent(mediaVolumePercent));
    audioVolumeCurrent = audioVolumeLevelFromPercent(mediaVolumePercent);
    Serial.printf("[AUDIO] backend ready: internal_dac=1 channel=%u i2s_port=%u heap=%lu dma=%lu reason=%s\n",
                  static_cast<unsigned int>(AUDIO_INTERNAL_DAC_CHANNEL),
                  static_cast<unsigned int>(AUDIO_I2S_PORT),
                  static_cast<unsigned long>(freeHeap),
                  static_cast<unsigned long>(dmaLargest),
                  (reason && reason[0]) ? reason : "-");
    return true;
#endif
}

void audioSetVolumeImmediate(uint8_t v)
{
    if (!audio) return;
    audio->setVolume(v);
    audioVolumeCurrent = v;
}

String mediaAliasTargetExt(const String &path)
{
    String p = path;
    p.toLowerCase();
    if (p.endsWith(".mpga") || p.endsWith(".mpeg")) return ".mp3";
    if (p.endsWith(".wave")) return ".wav";
    if (p.endsWith(".adts")) return ".aac";
    if (p.endsWith(".m4b") || p.endsWith(".f4a")) return ".m4a";
    return "";
}

bool mediaPathIsFlac(const String &path)
{
    String n = path;
    n.toLowerCase();
    return n.endsWith(".flac");
}

bool audioFlacSupportedNow(uint32_t *freeHeapOut, uint32_t *largestOut)
{
    const uint32_t freeHeap = static_cast<uint32_t>(ESP.getFreeHeap());
    const uint32_t largest = static_cast<uint32_t>(heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
    if (freeHeapOut) *freeHeapOut = freeHeap;
    if (largestOut) *largestOut = largest;
    if (psramFound()) return true;
    return (freeHeap >= AUDIO_FLAC_MIN_FREE_HEAP) && (largest >= AUDIO_FLAC_MIN_LARGEST_8BIT);
}

bool copyFileToPath(const String &srcPath, const String &dstPath)
{
    uint8_t *buf = reinterpret_cast<uint8_t *>(allocPreferPsram(1024));
    if (!buf) return false;
    for (int attempt = 0; attempt < 2; attempt++) {
        if (!sdEnsureMounted(attempt > 0)) {
            free(buf);
            return false;
        }
        if (!sdLock()) {
            free(buf);
            return false;
        }

        bool ok = true;
        File src = SD.open(srcPath, FILE_READ);
        if (!src || src.isDirectory()) {
            if (src) src.close();
            ok = false;
        }

        File dst;
        if (ok) {
            if (SD.exists(dstPath)) SD.remove(dstPath);
            dst = SD.open(dstPath, FILE_WRITE);
            if (!dst) ok = false;
        }

        if (ok) {
            while (src.available()) {
                int n = src.read(buf, 1024);
                if (n <= 0) break;
                if (dst.write(buf, n) != static_cast<size_t>(n)) {
                    ok = false;
                    break;
                }
            }
            if (ok) {
                dst.flush();
                ok = (dst.getWriteError() == 0);
            }
        }

        if (src) src.close();
        if (dst) dst.close();
        if (!ok && SD.exists(dstPath)) SD.remove(dstPath);
        sdUnlock();

        if (ok) {
            free(buf);
            return true;
        }
        sdMarkFault("copyFileToPath");
        delay(15);
    }
    free(buf);
    return false;
}

String mediaResolvePlaybackPath(const String &originalPath)
{
    String aliasExt = mediaAliasTargetExt(originalPath);
    if (aliasExt.isEmpty()) return originalPath;

    String aliasPath = "/.__play_alias" + aliasExt;
    if (!copyFileToPath(originalPath, aliasPath)) return "";
    return aliasPath;
}

void audioStopPlayback(bool smooth = true)
{
    (void)smooth;
    if (audioBackendReady && audio) audio->stopSong();
    if (mediaPlaybackPath.startsWith("/.__play_alias")) {
        if (sdLock()) {
            SD.remove(mediaPlaybackPath);
            sdUnlock();
        }
        mediaPlaybackPath = "";
    }
    mediaIsPlaying = false;
    mediaPaused = false;
    rgbRefreshByMediaState();
}

bool audioStartFile(const String &path)
{
    audioLastError = "";
    if (mediaPathIsFlac(path)) {
        uint32_t freeHeap = 0;
        uint32_t largest = 0;
        if (!audioFlacSupportedNow(&freeHeap, &largest)) {
            audioLastError = "flac_mem";
            Serial.printf("[AUDIO] FLAC rejected: no_psram heap=%lu largest=%lu need_heap>=%lu need_largest>=%lu path=%s\n",
                          static_cast<unsigned long>(freeHeap),
                          static_cast<unsigned long>(largest),
                          static_cast<unsigned long>(AUDIO_FLAC_MIN_FREE_HEAP),
                          static_cast<unsigned long>(AUDIO_FLAC_MIN_LARGEST_8BIT),
                          path.c_str());
            return false;
        }
    }
    if (!sdEnsureMounted() && !sdEnsureMounted(true)) return false;
    if (!audioEnsureBackendReady("playback")) return false;
    if (!networkSuspendedForAudio && !psramFound()) {
        const uint32_t largestNow = static_cast<uint32_t>(heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
        if (largestNow < AUDIO_PREEMPTIVE_NET_SUSPEND_LARGEST_8BIT) {
            Serial.printf("[AUDIO] preemptive network suspend: no_psram largest=%lu < %lu\n",
                          static_cast<unsigned long>(largestNow),
                          static_cast<unsigned long>(AUDIO_PREEMPTIVE_NET_SUSPEND_LARGEST_8BIT));
            networkSuspendForAudio();
            delay(20);
        }
    }
    audio->stopSong();
    const uint32_t freeHeapBefore = static_cast<uint32_t>(ESP.getFreeHeap());
    const uint32_t largestBefore = static_cast<uint32_t>(heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
    if (!audio->connecttoFS(SD, path.c_str())) {
        bool fileExists = false;
        if (sdLock()) {
            fileExists = SD.exists(path);
            sdUnlock();
        }

        const uint32_t freeHeapFail = static_cast<uint32_t>(ESP.getFreeHeap());
        const uint32_t largestFail = static_cast<uint32_t>(heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
        Serial.printf("[AUDIO] connecttoFS failed path=%s exists=%d heap_before=%lu largest_before=%lu heap_fail=%lu largest_fail=%lu\n",
                      path.c_str(),
                      fileExists ? 1 : 0,
                      static_cast<unsigned long>(freeHeapBefore),
                      static_cast<unsigned long>(largestBefore),
                      static_cast<unsigned long>(freeHeapFail),
                      static_cast<unsigned long>(largestFail));

        // Decoder OOM (for example mp3/flac) can fail connecttoFS while SD remains healthy.
        // Only trigger SD fault/recovery when the file itself is not accessible.
        if (!fileExists) {
            audioLastError = "sd_access";
            sdMarkFault("audioStartFile");
            if (!sdEnsureMounted(true) || !audio->connecttoFS(SD, path.c_str())) return false;
        } else {
            if (!networkSuspendedForAudio) {
                Serial.println("[AUDIO] decoder allocation failed, retry with network suspended");
                networkSuspendForAudio();
                delay(60);
                audio->stopSong();
                if (audio->connecttoFS(SD, path.c_str())) {
                    Serial.printf("[AUDIO] retry ok after network suspend path=%s\n", path.c_str());
                } else {
                    audioLastError = "decoder_mem";
                    return false;
                }
            } else {
                audioLastError = "decoder_mem";
                return false;
            }
        }
    }
    audioSetVolumeImmediate(audioVolumeLevelFromPercent(mediaVolumePercent));
    mediaIsPlaying = true;
    mediaPaused = false;
    rgbRefreshByMediaState();
    return true;
}

bool mediaStartTrack(const String &sourcePath, const String &displayName)
{
    if (sourcePath.isEmpty()) return false;
    audioStopPlayback(false);

    String playPath = mediaResolvePlaybackPath(sourcePath);
    if (playPath.isEmpty()) playPath = sourcePath;
    if (!audioStartFile(playPath)) {
        if (audioLastError.isEmpty()) audioLastError = "start_failed";
        return false;
    }

    mediaSelectedSourcePath = sourcePath;
    mediaSelectedTrackName = displayName.isEmpty() ? mediaDisplayNameFromPath(sourcePath) : displayName;
    mediaPlaybackPath = playPath;
    mediaNowPlaying = mediaSelectedTrackName;
    uiStatusLine = "Playing: " + mediaSelectedTrackName;
    lvglSyncStatusLine();
    return true;
}

static void mediaEnsureStorageReadyForUi()
{
#if defined(BOARD_ESP32S3_3248S035_N16R8)
    if (!sdMounted) {
        setupSd();
    } else {
        sdEnsureMounted();
    }
#else
    sdEnsureMounted();
#endif
}

void audioSetPaused(bool pause)
{
    if (!audioBackendReady || !audio) return;
    if (!audio->isRunning()) return;
    if (mediaPaused != pause) audio->pauseResume();
    mediaPaused = pause;
    mediaIsPlaying = !pause;
    rgbRefreshByMediaState();
}

void audioService()
{
    if (!audioBackendReady || !audio) return;
    audio->loop();
}

void loadMediaEntries()
{
    mediaCount = 0;
    mediaHasMore = false;
    mediaNormalizeDir(mediaCurrentDir);
    mediaScanRuns++;
    const unsigned long scanStartMs = millis();
    int scanned = 0;
    int eligibleSeen = 0;

    if (!sdEnsureMounted() && !sdEnsureMounted(true)) {
        uiStatusLine = "SD unavailable";
        return;
    }
    if (!sdLock()) {
        uiStatusLine = "SD busy";
        return;
    }

    bool finished = false;
    for (int pass = 0; pass < 2 && !finished; pass++) {
        scanned = 0;
        eligibleSeen = 0;
        mediaCount = 0;
        mediaHasMore = false;

        File dir = SD.open(mediaCurrentDir);
        if (!dir || !dir.isDirectory()) {
            if (dir) dir.close();
            if (mediaCurrentDir != "/") {
                mediaCurrentDir = "/";
                mediaOffset = 0;
                continue;
            }
            uiStatusLine = "Invalid dir: " + mediaCurrentDir;
            sdUnlock();
            return;
        }

        File entry = dir.openNextFile();
        while (entry) {
            const char *rawName = entry.name();
            const char *baseName = mediaBaseNamePtr(rawName);
            const bool isDir = entry.isDirectory();
            const size_t size = isDir ? 0 : entry.size();
            const bool eligible = isDir || mediaAllowedExtC(baseName);

            if (eligible) {
                if (eligibleSeen < mediaOffset) {
                    eligibleSeen++;
                } else if (mediaCount < MEDIA_PAGE_SIZE && mediaCount < MAX_MEDIA_ENTRIES) {
                    String full = mediaBuildFullPath(mediaCurrentDir, rawName);
                    if (full.length() > 0) {
                        mediaEntries[mediaCount] = {String(baseName), full, isDir, size};
                        mediaCount++;
                    }
                    eligibleSeen++;
                } else {
                    mediaHasMore = true;
                    break;
                }
            }

            entry.close();
            entry = dir.openNextFile();
            scanned++;
            if ((scanned % MEDIA_SCAN_YIELD_EVERY) == 0) {
                if (mediaIsPlaying && !mediaPaused) audioService();
                delay(0);
            }
        }
        if (entry) entry.close();
        dir.close();

        if (mediaOffset > 0 && mediaCount == 0) {
            mediaOffset = 0;
            continue;
        }
        finished = true;
    }
    sdUnlock();

    const int page = (mediaOffset / MEDIA_PAGE_SIZE) + 1;
    uiStatusLine = "Media p" + String(page) + ": " + String(mediaCount) + " item(s)";
    if (mediaHasMore) uiStatusLine += " +";
    const unsigned long scanMs = static_cast<unsigned long>(millis() - scanStartMs);
    if (scanMs >= MEDIA_SCAN_SLOW_MS) {
        mediaScanSlowRuns++;
        Serial.printf("[MEDIA] slow scan dir=%s page=%d ms=%lu scanned=%d eligible=%d count=%d more=%d slow_runs=%lu/%lu\n",
                      mediaCurrentDir.c_str(),
                      page,
                      scanMs,
                      scanned,
                      eligibleSeen,
                      mediaCount,
                      mediaHasMore ? 1 : 0,
                      static_cast<unsigned long>(mediaScanSlowRuns),
                      static_cast<unsigned long>(mediaScanRuns));
    } else {
        Serial.printf("[MEDIA] scan dir=%s page=%d ms=%lu scanned=%d eligible=%d count=%d more=%d\n",
                      mediaCurrentDir.c_str(),
                      page,
                      scanMs,
                      scanned,
                      eligibleSeen,
                      mediaCount,
                      mediaHasMore ? 1 : 0);
    }
    rgbRefreshByMediaState();
}

void displaySetAwake(bool awake)
{
    displayAwake = awake;
    displayBacklightSet(awake ? displayBacklightLevelFromPercent(displayBrightnessPercent) : TFT_BL_LEVEL_OFF);
    if (awake) {
        wakeTouchConfirmCount = 0;
        rgbRefreshByMediaState();
        if (lvglReady) {
            // Charge animation is drawn directly to TFT; force LVGL to repaint active menu screen on wake.
            lv_obj_invalidate(lv_scr_act());
            lvglNextHandlerDueMs = 0;
            lv_timer_handler();
            lvglRefreshInfoPanel(false);
            lvglRefreshTopIndicators();
        }
    } else {
        wakeTouchReleaseGuard = false;
        if (!batteryCharging) rgbApplyNow(false, false, false);
    }
}

static void wakeDisplayForIncomingNotification()
{
    if (displayAwake && !screensaverActive) return;
    if (screensaverActive) screensaverSetActive(false);
    if (!displayAwake) displaySetAwake(true);
    lastUserActivityMs = millis();
}

bool canEnterLowPowerSleep(bool touchDownNow)
{
#if defined(BOARD_ESP32S3_3248S035_N16R8)
    (void)touchDownNow;
    return false;
#else
    if (touchDownNow) return false;
    if (uiScreenKeepsDisplayAwake()) return false;
    if (wifiConnectedSafe()) return false; // requested: sleep only when not connected
    if (batteryCharging) return false;
    if (batteryPercent < 5) return false;
    if (fsWriteBusy()) return false;
    if (bootStaConnectInProgress) return false;
    if (mediaIsPlaying || mediaPaused) return false;
    if (displayAwake) return false;
    if (millis() - lastUserActivityMs < LIGHT_SLEEP_AFTER_IDLE_MS) return false;
    return true;
#endif
}

void enterLowPowerSleep()
{
    // Wake on CST820 IRQ low level touch signal when available.
    if (TOUCH_USE_IRQ) {
        gpio_wakeup_enable(static_cast<gpio_num_t>(TOUCH_IRQ), GPIO_INTR_LOW_LEVEL);
        esp_sleep_enable_gpio_wakeup();
    }
    // Optional safety wake in case touch IRQ is not wired on this board revision.
    if (LIGHT_SLEEP_TIMER_FALLBACK || !TOUCH_USE_IRQ) esp_sleep_enable_timer_wakeup(LIGHT_SLEEP_TIMER_US);
    esp_light_sleep_start();
}

static bool normalizeSavedApCreds()
{
    bool changed = false;
    savedApSsid.trim();
    savedApPass.trim();
    if (savedApSsid.isEmpty()) {
        savedApSsid = AP_SSID;
        changed = true;
    }
    if (savedApPass.isEmpty() || savedApPass.length() < 8) {
        savedApPass = AP_PASS;
        changed = true;
    }
    return changed;
}

void loadSavedStaCreds()
{
    wifiPrefs.begin("wifi", true);
    savedStaSsid = wifiPrefs.getString("sta_ssid", "");
    savedStaPass = wifiPrefs.getString("sta_pass", "");
    savedApSsid = wifiPrefs.getString("ap_ssid", AP_SSID);
    savedApPass = wifiPrefs.getString("ap_pass", AP_PASS);
    wifiSessionApMode = wifiPrefs.getBool("ap_mode", false);
    wifiPrefs.end();
    if (normalizeSavedApCreds()) {
        wifiPrefs.begin("wifi", false);
        wifiPrefs.putString("ap_ssid", savedApSsid);
        wifiPrefs.putString("ap_pass", savedApPass);
        wifiPrefs.end();
    }
}

bool loadBatterySnapshot(BatterySnapshot &snapshotOut)
{
    batteryPrefs.begin("battery", true);
    bool valid = batteryPrefs.getBool("valid", false);
    float rawV = batteryPrefs.getFloat("raw_v", 0.0f);
    if (rawV <= 0.0f) rawV = batteryPrefs.getFloat("vbat", 0.0f);
    bool charging = batteryPrefs.getBool("charging", false);
    uint32_t uptimeMs = batteryPrefs.getULong("uptime_ms", 0UL);
    batteryPrefs.end();
    if (!valid || rawV <= 0.0f) return false;
    snapshotOut.valid = true;
    snapshotOut.rawV = rawV;
    snapshotOut.charging = charging;
    snapshotOut.uptimeMs = uptimeMs;
    return true;
}

void saveBatterySnapshot(float rawV, bool charging, uint32_t uptimeMs)
{
    batteryPrefs.begin("battery", false);
    batteryPrefs.putBool("valid", true);
    batteryPrefs.putFloat("raw_v", rawV);
    batteryPrefs.putFloat("vbat", batteryCalibratedVoltageFromRaw(rawV));
    batteryPrefs.putBool("charging", charging);
    batteryPrefs.putULong("uptime_ms", uptimeMs);
    batteryPrefs.end();
}

void saveStaCreds(const String &ssid, const String &pass)
{
    wifiPrefs.begin("wifi", false);
    wifiPrefs.putString("sta_ssid", ssid);
    wifiPrefs.putString("sta_pass", pass);
    wifiPrefs.end();
    savedStaSsid = ssid;
    savedStaPass = pass;
}

void clearStaCreds()
{
    wifiPrefs.begin("wifi", false);
    wifiPrefs.remove("sta_ssid");
    wifiPrefs.remove("sta_pass");
    wifiPrefs.end();
    savedStaSsid = "";
    savedStaPass = "";
}

void loadGamePrefs()
{
    gamePrefs.begin("games", true);
    snakeHighScore = static_cast<uint16_t>(gamePrefs.getUInt("snake_best", 0));
    snakeLastScore = static_cast<uint16_t>(gamePrefs.getUInt("snake_last", 0));
    tetrisHighScore = static_cast<uint16_t>(gamePrefs.getUInt("tetris_best", 0));
    tetrisLastScore = static_cast<uint16_t>(gamePrefs.getUInt("tetris_last", 0));
    checkersLocalWins = static_cast<uint16_t>(gamePrefs.getUInt("checkers_local_wins", 0));
    checkersRemoteWins = static_cast<uint16_t>(gamePrefs.getUInt("checkers_remote_wins", 0));
#if defined(BOARD_ESP32S3_3248S035_N16R8)
    snake3dHighScore = static_cast<uint16_t>(gamePrefs.getUInt("snake3d_best", 0));
    snake3dLastScore = static_cast<uint16_t>(gamePrefs.getUInt("snake3d_last", 0));
#endif
    gamePrefs.end();
}

static void saveGameHighScore(const char *key, uint16_t score)
{
    if (!key) return;
    gamePrefs.begin("games", false);
    gamePrefs.putUInt(key, score);
    gamePrefs.end();
}

static void saveGameValue(const char *key, uint16_t value)
{
    if (!key) return;
    gamePrefs.begin("games", false);
    gamePrefs.putUInt(key, value);
    gamePrefs.end();
}

static void snakeMaybeStoreHighScore(bool persist)
{
    if (snakeScore <= snakeHighScore) return;
    snakeHighScore = snakeScore;
    if (persist) saveGameHighScore("snake_best", snakeHighScore);
}

static void tetrisMaybeStoreHighScore(bool persist)
{
    if (tetrisScore <= tetrisHighScore) return;
    tetrisHighScore = tetrisScore;
    if (persist) saveGameHighScore("tetris_best", tetrisHighScore);
}

#if defined(BOARD_ESP32S3_3248S035_N16R8)
static void snake3dMaybeStoreHighScore(bool persist)
{
    if (snake3dScore <= snake3dHighScore) return;
    snake3dHighScore = snake3dScore;
    if (persist) saveGameHighScore("snake3d_best", snake3dHighScore);
}
#endif

