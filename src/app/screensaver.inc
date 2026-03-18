struct ScreensaverEyePreset {
    int16_t offsetX;
    int16_t offsetY;
    int16_t height;
    int16_t width;
    float slopeTop;
    float slopeBottom;
    int16_t radiusTop;
    int16_t radiusBottom;
    int16_t inverseRadiusTop;
    int16_t inverseRadiusBottom;
    int16_t inverseOffsetTop;
    int16_t inverseOffsetBottom;
    bool pupilsVisible;
};

struct ScreensaverPose {
    const ScreensaverEyePreset *preset;
    float faceScale;
    float lookX;
    float lookY;
};

static constexpr ScreensaverEyePreset SCREENSAVER_PRESET_NORMAL = {0, 0, 40, 40, 0.0f, 0.0f, 8, 8, 0, 0, 0, 0, true};
static constexpr ScreensaverEyePreset SCREENSAVER_PRESET_HAPPY = {0, 0, 10, 40, 0.0f, 0.0f, 10, 0, 0, 0, 0, 0, true};
static constexpr ScreensaverEyePreset SCREENSAVER_PRESET_GLEE = {0, 0, 8, 40, 0.0f, 0.0f, 8, 0, 0, 5, 0, 0, true};
static constexpr ScreensaverEyePreset SCREENSAVER_PRESET_SAD = {0, 0, 15, 40, -0.5f, 0.0f, 1, 10, 0, 0, 0, 0, true};
static constexpr ScreensaverEyePreset SCREENSAVER_PRESET_FOCUSED = {0, 0, 14, 40, 0.2f, 0.0f, 3, 1, 0, 0, 0, 0, true};
static constexpr ScreensaverEyePreset SCREENSAVER_PRESET_SURPRISED = {-2, 0, 45, 45, 0.0f, 0.0f, 16, 16, 0, 0, 0, 0, true};
static constexpr ScreensaverEyePreset SCREENSAVER_PRESET_SKEPTIC_ALT = {0, -6, 26, 40, 0.3f, 0.0f, 1, 10, 0, 0, 0, 0, true};
static constexpr ScreensaverEyePreset SCREENSAVER_PRESET_SLEEPY = {0, -2, 14, 40, -0.5f, -0.5f, 3, 3, 0, 0, 0, 0, false};
static constexpr ScreensaverEyePreset SCREENSAVER_PRESET_SUSPICIOUS = {0, 0, 22, 40, 0.0f, 0.0f, 8, 3, 0, 0, 0, 0, true};
static constexpr ScreensaverEyePreset SCREENSAVER_PRESET_BLINK = {0, 0, 5, 40, 0.0f, 0.0f, 3, 3, 0, 0, 0, 0, false};

static const ScreensaverEyePreset *const SCREENSAVER_PRESETS[] = {
    &SCREENSAVER_PRESET_NORMAL,
    &SCREENSAVER_PRESET_HAPPY,
    &SCREENSAVER_PRESET_GLEE,
    &SCREENSAVER_PRESET_SAD,
    &SCREENSAVER_PRESET_FOCUSED,
    &SCREENSAVER_PRESET_SURPRISED,
    &SCREENSAVER_PRESET_SKEPTIC_ALT,
    &SCREENSAVER_PRESET_SLEEPY,
    &SCREENSAVER_PRESET_SUSPICIOUS,
};

static ScreensaverPose screensaverRandomPose(bool blinkOnly)
{
    const float scale = min(DISPLAY_WIDTH / 128.0f, DISPLAY_HEIGHT / 64.0f) * 0.9f;
    const ScreensaverEyePreset *preset = blinkOnly
                                             ? &SCREENSAVER_PRESET_BLINK
                                             : SCREENSAVER_PRESETS[random(0, static_cast<int>(sizeof(SCREENSAVER_PRESETS) / sizeof(SCREENSAVER_PRESETS[0])))];
    ScreensaverPose pose = {
        preset,
        scale,
        blinkOnly ? 0.0f : static_cast<float>(random(-50, 51)) / 100.0f,
        blinkOnly ? 0.0f : static_cast<float>(random(-40, 41)) / 100.0f,
    };
    return pose;
}

enum ScreensaverCornerType : uint8_t { SC_T_R, SC_T_L, SC_B_L, SC_B_R };

static void screensaverFillRect(int32_t x0, int32_t y0, int32_t x1, int32_t y1, uint16_t color)
{
    const int32_t l = min(x0, x1);
    const int32_t r = max(x0, x1);
    const int32_t t = min(y0, y1);
    const int32_t b = max(y0, y1);
    const int32_t w = max<int32_t>(0, r - l);
    const int32_t h = max<int32_t>(0, b - t);
    if (w <= 0 || h <= 0) return;
    tft.fillRect(l, t, w, h, color);
}

static inline uint16_t screensaverDrawColor(uint8_t drawValue, uint16_t eyeColor)
{
    return drawValue ? eyeColor : TFT_BLACK;
}

static void screensaverFillRectTriangle(int32_t x0, int32_t y0, int32_t x1, int32_t y1, uint8_t drawValue, uint16_t eyeColor)
{
    tft.fillTriangle(x0, y0, x1, y1, x1, y0, screensaverDrawColor(drawValue, eyeColor));
}

static void screensaverFillEllipseCorner(ScreensaverCornerType corner, int16_t x0, int16_t y0, int32_t rx, int32_t ry, uint16_t color)
{
    if (rx < 2 || ry < 2) return;
    int32_t x, y;
    const int32_t rx2 = rx * rx;
    const int32_t ry2 = ry * ry;
    const int32_t fx2 = 4 * rx2;
    const int32_t fy2 = 4 * ry2;
    int32_t s;

    auto hline = [&](int32_t sx, int32_t sy, int32_t len) {
        if (len <= 0) return;
        tft.drawFastHLine(static_cast<int16_t>(sx), static_cast<int16_t>(sy), static_cast<int16_t>(len), color);
    };

    if (corner == SC_T_R) {
        for (x = 0, y = ry, s = 2 * ry2 + rx2 * (1 - 2 * ry); ry2 * x <= rx2 * y; x++) {
            hline(x0, y0 - y, x);
            if (s >= 0) { s += fx2 * (1 - y); y--; }
            s += ry2 * ((4 * x) + 6);
        }
        for (x = rx, y = 0, s = 2 * rx2 + ry2 * (1 - 2 * rx); rx2 * y <= ry2 * x; y++) {
            hline(x0, y0 - y, x);
            if (s >= 0) { s += fy2 * (1 - x); x--; }
            s += rx2 * ((4 * y) + 6);
        }
    } else if (corner == SC_B_R) {
        for (x = 0, y = ry, s = 2 * ry2 + rx2 * (1 - 2 * ry); ry2 * x <= rx2 * y; x++) {
            hline(x0, y0 + y - 1, x);
            if (s >= 0) { s += fx2 * (1 - y); y--; }
            s += ry2 * ((4 * x) + 6);
        }
        for (x = rx, y = 0, s = 2 * rx2 + ry2 * (1 - 2 * rx); rx2 * y <= ry2 * x; y++) {
            hline(x0, y0 + y - 1, x);
            if (s >= 0) { s += fy2 * (1 - x); x--; }
            s += rx2 * ((4 * y) + 6);
        }
    } else if (corner == SC_T_L) {
        for (x = 0, y = ry, s = 2 * ry2 + rx2 * (1 - 2 * ry); ry2 * x <= rx2 * y; x++) {
            hline(x0 - x, y0 - y, x);
            if (s >= 0) { s += fx2 * (1 - y); y--; }
            s += ry2 * ((4 * x) + 6);
        }
        for (x = rx, y = 0, s = 2 * rx2 + ry2 * (1 - 2 * rx); rx2 * y <= ry2 * x; y++) {
            hline(x0 - x, y0 - y, x);
            if (s >= 0) { s += fy2 * (1 - x); x--; }
            s += rx2 * ((4 * y) + 6);
        }
    } else {
        for (x = 0, y = ry, s = 2 * ry2 + rx2 * (1 - 2 * ry); ry2 * x <= rx2 * y; x++) {
            hline(x0 - x, y0 + y - 1, x);
            if (s >= 0) { s += fx2 * (1 - y); y--; }
            s += ry2 * ((4 * x) + 6);
        }
        for (x = rx, y = 0, s = 2 * rx2 + ry2 * (1 - 2 * rx); rx2 * y <= ry2 * x; y++) {
            hline(x0 - x, y0 + y, x);
            if (s >= 0) { s += fy2 * (1 - x); x--; }
            s += rx2 * ((4 * y) + 6);
        }
    }
}

static void screensaverDrawEyeExact(int16_t centerX, int16_t centerY, const ScreensaverEyePreset &srcPreset,
                                    bool mirrored, float scaleX, float scaleY, uint16_t color)
{
    ScreensaverEyePreset config = srcPreset;
    config.offsetX = mirrored ? -config.offsetX : config.offsetX;
    config.offsetY = -config.offsetY;
    config.slopeTop = mirrored ? config.slopeTop : -config.slopeTop;
    config.slopeBottom = mirrored ? config.slopeBottom : -config.slopeBottom;

    const int32_t offsetX = lroundf(config.offsetX * scaleX);
    const int32_t offsetY = lroundf(config.offsetY * scaleY);
    const int32_t width = max<int32_t>(2, lroundf(config.width * scaleX));
    const int32_t height = max<int32_t>(2, lroundf(config.height * scaleY));
    int32_t radiusTop = max<int32_t>(0, lroundf(config.radiusTop * min(scaleX, scaleY)));
    int32_t radiusBottom = max<int32_t>(0, lroundf(config.radiusBottom * min(scaleX, scaleY)));

    int32_t deltaYTop = lroundf(height * config.slopeTop / 2.0f);
    int32_t deltaYBottom = lroundf(height * config.slopeBottom / 2.0f);
    const int32_t totalHeight = height + deltaYTop - deltaYBottom;
    if (radiusBottom > 0 && radiusTop > 0 && totalHeight - 1 < radiusBottom + radiusTop) {
        const int32_t correctedTop = static_cast<int32_t>(static_cast<float>(radiusTop) * (totalHeight - 1) / max<int32_t>(1, radiusBottom + radiusTop));
        const int32_t correctedBottom = static_cast<int32_t>(static_cast<float>(radiusBottom) * (totalHeight - 1) / max<int32_t>(1, radiusBottom + radiusTop));
        radiusTop = correctedTop;
        radiusBottom = correctedBottom;
    }

    const int32_t TLcY = centerY + offsetY - height / 2 + radiusTop - deltaYTop;
    const int32_t TLcX = centerX + offsetX - width / 2 + radiusTop;
    const int32_t TRcY = centerY + offsetY - height / 2 + radiusTop + deltaYTop;
    const int32_t TRcX = centerX + offsetX + width / 2 - radiusTop;
    const int32_t BLcY = centerY + offsetY + height / 2 - radiusBottom - deltaYBottom;
    const int32_t BLcX = centerX + offsetX - width / 2 + radiusBottom;
    const int32_t BRcY = centerY + offsetY + height / 2 - radiusBottom + deltaYBottom;
    const int32_t BRcX = centerX + offsetX + width / 2 - radiusBottom;

    const int32_t minCX = min(TLcX, BLcX);
    const int32_t maxCX = max(TRcX, BRcX);
    const int32_t minCY = min(TLcY, TRcY);
    const int32_t maxCY = max(BLcY, BRcY);

    screensaverFillRect(minCX, minCY, maxCX, maxCY, color);
    screensaverFillRect(TRcX, TRcY, BRcX + radiusBottom, BRcY, color);
    screensaverFillRect(TLcX - radiusTop, TLcY, BLcX, BLcY, color);
    screensaverFillRect(TLcX, TLcY - radiusTop, TRcX, TRcY, color);
    screensaverFillRect(BLcX, BLcY, BRcX, BRcY + radiusBottom, color);

    if (config.slopeTop > 0) {
        screensaverFillRectTriangle(TLcX, TLcY - radiusTop, TRcX, TRcY - radiusTop, 0, color);
        screensaverFillRectTriangle(TRcX, TRcY - radiusTop, TLcX, TLcY - radiusTop, 1, color);
    } else if (config.slopeTop < 0) {
        screensaverFillRectTriangle(TRcX, TRcY - radiusTop, TLcX, TLcY - radiusTop, 0, color);
        screensaverFillRectTriangle(TLcX, TLcY - radiusTop, TRcX, TRcY - radiusTop, 1, color);
    }
    if (config.slopeBottom > 0) {
        screensaverFillRectTriangle(BRcX + radiusBottom, BRcY + radiusBottom, BLcX - radiusBottom, BLcY + radiusBottom, 0, color);
        screensaverFillRectTriangle(BLcX - radiusBottom, BLcY + radiusBottom, BRcX + radiusBottom, BRcY + radiusBottom, 1, color);
    } else if (config.slopeBottom < 0) {
        screensaverFillRectTriangle(BLcX - radiusBottom, BLcY + radiusBottom, BRcX + radiusBottom, BRcY + radiusBottom, 0, color);
        screensaverFillRectTriangle(BRcX + radiusBottom, BRcY + radiusBottom, BLcX - radiusBottom, BLcY + radiusBottom, 1, color);
    }

    if (radiusTop > 0) {
        screensaverFillEllipseCorner(SC_T_L, TLcX, TLcY, radiusTop, radiusTop, color);
        screensaverFillEllipseCorner(SC_T_R, TRcX, TRcY, radiusTop, radiusTop, color);
    }
    if (radiusBottom > 0) {
        screensaverFillEllipseCorner(SC_B_L, BLcX, BLcY, radiusBottom, radiusBottom, color);
        screensaverFillEllipseCorner(SC_B_R, BRcX, BRcY, radiusBottom, radiusBottom, color);
    }
}

static void screensaverApplyPose(const ScreensaverPose &pose)
{
    const ScreensaverEyePreset &preset = *pose.preset;
    const float eyeScale = pose.faceScale;
    const int16_t eyeSize = max<int16_t>(18, lroundf(40.0f * eyeScale));
    const int16_t interDistance = max<int16_t>(4, lroundf(4.0f * eyeScale));
    const int16_t centerX = DISPLAY_WIDTH / 2 + lroundf(-25.0f * pose.lookX * eyeScale);
    const int16_t centerY = DISPLAY_HEIGHT / 2 + lroundf(20.0f * pose.lookY * eyeScale);
    const float scaleYY = 1.0f - fabsf(pose.lookY) * 0.4f;
    const float leftScaleY = (1.0f + pose.lookX * 0.2f) * scaleYY;
    const float rightScaleY = (1.0f - pose.lookX * 0.2f) * scaleYY;
    const uint16_t eyeColor = tft.color565(
        static_cast<uint8_t>((SCREENSAVER_EYE_COLOR_RGB >> 16) & 0xFF),
        static_cast<uint8_t>((SCREENSAVER_EYE_COLOR_RGB >> 8) & 0xFF),
        static_cast<uint8_t>(SCREENSAVER_EYE_COLOR_RGB & 0xFF));

    tft.fillScreen(TFT_BLACK);
    screensaverDrawEyeExact(centerX - eyeSize / 2 - interDistance, centerY, preset, true, eyeScale, eyeScale * leftScaleY, eyeColor);
    screensaverDrawEyeExact(centerX + eyeSize / 2 + interDistance, centerY, preset, false, eyeScale, eyeScale * rightScaleY, eyeColor);
}

void screensaverSetActive(bool active)
{
    if (screensaverActive == active) return;
    screensaverActive = active;
    if (active) {
        screensaverReturnScreen = uiScreen;
        lvglEnsureScreenBuilt(UI_SCREENSAVER);
        if (lvglTopBarRoot) lv_obj_add_flag(lvglTopBarRoot, LV_OBJ_FLAG_HIDDEN);
        screensaverLastPoseMs = 0;
        screensaverNextPoseDelayMs = SCREENSAVER_POSE_MIN_MS;
        screensaverApplyPose(screensaverRandomPose(false));
        lvglOpenScreen(UI_SCREENSAVER, LV_SCR_LOAD_ANIM_FADE_ON);
    } else {
        if (lvglTopBarRoot) lv_obj_clear_flag(lvglTopBarRoot, LV_OBJ_FLAG_HIDDEN);
        lvglOpenScreen(screensaverReturnScreen, LV_SCR_LOAD_ANIM_FADE_ON);
        lvglSuppressClicksAfterGesture();
    }
}

void screensaverService()
{
    if (!screensaverActive || !lvglReady) return;
    const unsigned long now = millis();
    if (screensaverLastPoseMs != 0 && static_cast<unsigned long>(now - screensaverLastPoseMs) < screensaverNextPoseDelayMs) return;
    const bool blinkNow = random(0, 100) < 24;
    screensaverApplyPose(screensaverRandomPose(blinkNow));
    screensaverLastPoseMs = now;
    screensaverNextPoseDelayMs = blinkNow ? SCREENSAVER_BLINK_MS : static_cast<unsigned long>(random(static_cast<long>(SCREENSAVER_POSE_MIN_MS), static_cast<long>(SCREENSAVER_POSE_MAX_MS + 1UL)));
}
