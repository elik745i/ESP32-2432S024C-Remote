#if defined(BOARD_ESP32S3_3248S035_N16R8)
static bool snake3dCellOccupied(int x, int y, int z)
{
    for (int i = 0; i < snake3dLen; ++i) {
        if (snake3dX[i] == x && snake3dY[i] == y && snake3dZ[i] == z) return true;
    }
    return false;
}

static void snake3dSpawnFood()
{
    for (int tries = 0; tries < 400; ++tries) {
        const int fx = random(0, SNAKE3D_COLS);
        const int fy = random(0, SNAKE3D_ROWS);
        const int fz = random(0, SNAKE3D_LAYERS);
        if (!snake3dCellOccupied(fx, fy, fz)) {
            snake3dFoodX = static_cast<int8_t>(fx);
            snake3dFoodY = static_cast<int8_t>(fy);
            snake3dFoodZ = static_cast<int8_t>(fz);
            return;
        }
    }
    snake3dFoodX = 0;
    snake3dFoodY = 0;
    snake3dFoodZ = 0;
}

void snake3dResetGame()
{
    snake3dLen = 4;
    snake3dDir = 1;
    snake3dNextDir = 1;
    snake3dViewDir = 1;
    snake3dStarted = true;
    snake3dPaused = false;
    snake3dGameOver = false;
    snake3dScore = 0;
    const int cy = SNAKE3D_ROWS / 2;
    const int cz = SNAKE3D_LAYERS / 2;
    for (int i = 0; i < snake3dLen; ++i) {
        snake3dX[i] = static_cast<int8_t>((SNAKE3D_COLS - 2) - i);
        snake3dY[i] = static_cast<int8_t>(cy);
        snake3dZ[i] = static_cast<int8_t>(cz);
    }
    snake3dSpawnFood();
    snake3dLastStepMs = millis();
}

void snake3dPrepareGame()
{
    snake3dResetGame();
    snake3dStarted = false;
    snake3dPaused = false;
    snake3dGameOver = false;
}

void snake3dStartGame()
{
    snake3dResetGame();
    snake3dStarted = true;
    snake3dPaused = false;
    snake3dLastStepMs = millis();
}

static unsigned long snake3dCurrentStepMs()
{
    const unsigned long foods = static_cast<unsigned long>(snake3dScore / 10U);
    const unsigned long reduction = foods * SNAKE3D_STEP_MS_DELTA_PER_FOOD;
    if (reduction >= (SNAKE3D_STEP_MS_START - SNAKE3D_STEP_MS_MIN)) return SNAKE3D_STEP_MS_MIN;
    return SNAKE3D_STEP_MS_START - reduction;
}

static void snake3dStep()
{
    if (!snake3dStarted || snake3dPaused || snake3dGameOver) return;
    snake3dDir = snake3dNextDir;
    int nx = snake3dX[0];
    int ny = snake3dY[0];
    int nz = snake3dZ[0];
    if (snake3dDir == 0) ny--;
    else if (snake3dDir == 1) nx++;
    else if (snake3dDir == 2) ny++;
    else if (snake3dDir == 3) nx--;
    else if (snake3dDir == 4) nz++;
    else nz--;

    if (nx < 0 || nx >= SNAKE3D_COLS || ny < 0 || ny >= SNAKE3D_ROWS || nz < 0 || nz >= SNAKE3D_LAYERS || snake3dCellOccupied(nx, ny, nz)) {
        snake3dGameOver = true;
        snake3dStarted = false;
        snake3dLastScore = snake3dScore;
        saveGameValue("snake3d_last", snake3dLastScore);
        snake3dMaybeStoreHighScore(true);
        return;
    }

    const bool ate = (nx == snake3dFoodX && ny == snake3dFoodY && nz == snake3dFoodZ);
    const int oldLen = snake3dLen;
    if (ate && snake3dLen < SNAKE3D_MAX_CELLS) snake3dLen++;
    for (int i = snake3dLen - 1; i > 0; --i) {
        snake3dX[i] = snake3dX[i - 1];
        snake3dY[i] = snake3dY[i - 1];
        snake3dZ[i] = snake3dZ[i - 1];
    }
    snake3dX[0] = static_cast<int8_t>(nx);
    snake3dY[0] = static_cast<int8_t>(ny);
    snake3dZ[0] = static_cast<int8_t>(nz);

    if (ate) {
        snake3dScore += 10;
        snake3dMaybeStoreHighScore(false);
        snake3dSpawnFood();
    } else if (oldLen < snake3dLen) {
        snake3dLen = oldLen;
    }
}

void snake3dTick()
{
    if (uiScreen != UI_GAME_SNAKE3D) return;
    const unsigned long now = millis();
    if (!snake3dStarted || snake3dPaused || snake3dGameOver) return;
    if (now - snake3dLastStepMs < snake3dCurrentStepMs()) return;
    snake3dLastStepMs = now;
    snake3dStep();
    lvglRefreshSnake3dBoard();
}
#endif

#if defined(BOARD_ESP32S3_3248S035_N16R8)
struct Snake3dVec {
    float x;
    float y;
    float z;
};

struct Snake3dCamera {
    Snake3dVec pos;
    Snake3dVec forward;
    Snake3dVec right;
    Snake3dVec up;
};

static Snake3dVec snake3dVecSub(const Snake3dVec &a, const Snake3dVec &b)
{
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}

static Snake3dVec snake3dVecAdd(const Snake3dVec &a, const Snake3dVec &b)
{
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}

static Snake3dVec snake3dVecScale(const Snake3dVec &v, float s)
{
    return {v.x * s, v.y * s, v.z * s};
}

static float snake3dVecDot(const Snake3dVec &a, const Snake3dVec &b)
{
    return (a.x * b.x) + (a.y * b.y) + (a.z * b.z);
}

static Snake3dVec snake3dVecCross(const Snake3dVec &a, const Snake3dVec &b)
{
    return {
        (a.y * b.z) - (a.z * b.y),
        (a.z * b.x) - (a.x * b.z),
        (a.x * b.y) - (a.y * b.x)
    };
}

static Snake3dVec snake3dVecNormalize(const Snake3dVec &v)
{
    const float len = sqrtf((v.x * v.x) + (v.y * v.y) + (v.z * v.z));
    if (len <= 0.0001f) return {0.0f, 0.0f, 1.0f};
    return {v.x / len, v.y / len, v.z / len};
}

static lv_color_t snake3dMixColor(lv_color_t a, lv_color_t b, uint8_t mix)
{
    return lv_color_mix(a, b, mix);
}

static void snake3dFillQuad(lv_draw_ctx_t *drawCtx, const lv_point_t pts[4], lv_color_t color, lv_opa_t opa = LV_OPA_COVER)
{
    if (!drawCtx) return;
    int minY = pts[0].y;
    int maxY = pts[0].y;
    for (int i = 1; i < 4; ++i) {
        if (pts[i].y < minY) minY = pts[i].y;
        if (pts[i].y > maxY) maxY = pts[i].y;
    }
    for (int y = minY; y <= maxY; ++y) {
        int hits[8];
        int hitCount = 0;
        for (int i = 0; i < 4; ++i) {
            const lv_point_t a = pts[i];
            const lv_point_t b = pts[(i + 1) & 3];
            if (a.y == b.y) continue;
            const int yMin = min(a.y, b.y);
            const int yMax = max(a.y, b.y);
            if (y < yMin || y >= yMax) continue;
            const float t = static_cast<float>(y - a.y) / static_cast<float>(b.y - a.y);
            hits[hitCount++] = a.x + static_cast<int>((b.x - a.x) * t);
        }
        if (hitCount < 2) continue;
        if (hits[1] < hits[0]) {
            const int tmp = hits[0];
            hits[0] = hits[1];
            hits[1] = tmp;
        }
        lvglDrawLineSeg(drawCtx, hits[0], y, hits[1], y, color, 1, opa);
    }
}

static Snake3dVec snake3dGridToWorld(float gx, float gy, float gz, float spacing)
{
    Snake3dVec v = {};
    v.x = (gx - ((SNAKE3D_COLS - 1) * 0.5f)) * spacing;
    v.y = gz * (spacing * 0.78f);
    v.z = (((SNAKE3D_ROWS - 1) * 0.5f) - gy) * spacing;
    return v;
}

static Snake3dVec snake3dForwardForDir(int8_t dir)
{
    switch (dir) {
        case 0: return {0.0f, 0.0f, 1.0f};
        case 1: return {1.0f, 0.0f, 0.0f};
        case 2: return {0.0f, 0.0f, -1.0f};
        case 3: return {-1.0f, 0.0f, 0.0f};
        default: return snake3dForwardForDir(snake3dViewDir);
    }
}

static Snake3dCamera snake3dBuildCamera(float spacing)
{
    const Snake3dVec head = snake3dGridToWorld(static_cast<float>(snake3dX[0]),
                                               static_cast<float>(snake3dY[0]),
                                               static_cast<float>(snake3dZ[0]),
                                               spacing);
    const Snake3dVec globalUp = {0.0f, 1.0f, 0.0f};
    const Snake3dVec planarForward = snake3dForwardForDir(snake3dViewDir);
    const Snake3dVec camPos = snake3dVecAdd(head,
                                            snake3dVecAdd(snake3dVecScale(planarForward, -spacing * 5.4f),
                                                          {0.0f, spacing * 3.9f, 0.0f}));
    const Snake3dVec target = snake3dVecAdd(head,
                                            snake3dVecAdd(snake3dVecScale(planarForward, spacing * 2.8f),
                                                          {0.0f, spacing * 0.15f, 0.0f}));
    Snake3dCamera cam = {};
    cam.pos = camPos;
    cam.forward = snake3dVecNormalize(snake3dVecSub(target, camPos));
    cam.right = snake3dVecNormalize(snake3dVecCross(cam.forward, globalUp));
    if (fabsf(cam.right.x) < 0.001f && fabsf(cam.right.y) < 0.001f && fabsf(cam.right.z) < 0.001f) cam.right = {1.0f, 0.0f, 0.0f};
    cam.up = snake3dVecNormalize(snake3dVecCross(cam.right, cam.forward));
    return cam;
}

static bool snake3dProjectPoint(const Snake3dVec &v, const Snake3dCamera &cam, int cx, int cy, float focal, int &sx, int &sy, float &depth)
{
    const Snake3dVec rel = snake3dVecSub(v, cam.pos);
    const float rx = snake3dVecDot(rel, cam.right);
    const float ry = snake3dVecDot(rel, cam.up);
    depth = snake3dVecDot(rel, cam.forward);
    if (depth <= 4.0f) return false;
    const float scale = focal / depth;
    sx = cx + static_cast<int>(rx * scale);
    sy = cy - static_cast<int>(ry * scale);
    return true;
}

static void snake3dDrawCube(lv_draw_ctx_t *drawCtx, int cx, int cy, float focal, const Snake3dCamera &cam, const Snake3dVec &center, float halfSize, lv_color_t color, uint8_t width)
{
    static const uint8_t faces[6][4] = {
        {0, 1, 2, 3},
        {4, 5, 6, 7},
        {0, 1, 5, 4},
        {1, 2, 6, 5},
        {2, 3, 7, 6},
        {3, 0, 4, 7}
    };
    const float offs[8][3] = {
        {-1, -1, -1}, {1, -1, -1}, {1, 1, -1}, {-1, 1, -1},
        {-1, -1, 1}, {1, -1, 1}, {1, 1, 1}, {-1, 1, 1}
    };
    Snake3dVec verts[8] = {};
    int px[8] = {};
    int py[8] = {};
    float pz[8] = {};
    bool ok[8] = {};
    for (int i = 0; i < 8; ++i) {
        verts[i] = {
            center.x + (offs[i][0] * halfSize),
            center.y + (offs[i][1] * halfSize),
            center.z + (offs[i][2] * halfSize)
        };
        ok[i] = snake3dProjectPoint(verts[i], cam, cx, cy, focal, px[i], py[i], pz[i]);
    }

    struct FaceInfo {
        int idx;
        float depth;
    };
    FaceInfo visible[6];
    int visibleCount = 0;
    for (int i = 0; i < 6; ++i) {
        const uint8_t a = faces[i][0];
        const uint8_t b = faces[i][1];
        const uint8_t c = faces[i][2];
        const uint8_t d = faces[i][3];
        if (!(ok[a] && ok[b] && ok[c] && ok[d])) continue;
        const Snake3dVec ab = snake3dVecSub(verts[b], verts[a]);
        const Snake3dVec ac = snake3dVecSub(verts[c], verts[a]);
        const Snake3dVec normal = snake3dVecCross(ab, ac);
        const Snake3dVec toCam = snake3dVecSub(cam.pos, verts[a]);
        if (snake3dVecDot(normal, toCam) <= 0.0f) continue;
        visible[visibleCount++] = {i, (pz[a] + pz[b] + pz[c] + pz[d]) * 0.25f};
    }

    for (int i = 0; i < visibleCount - 1; ++i) {
        for (int j = i + 1; j < visibleCount; ++j) {
            if (visible[j].depth > visible[i].depth) {
                const FaceInfo tmp = visible[i];
                visible[i] = visible[j];
                visible[j] = tmp;
            }
        }
    }

    for (int i = 0; i < visibleCount; ++i) {
        const uint8_t *face = faces[visible[i].idx];
        lv_point_t quad[4] = {
            {static_cast<lv_coord_t>(px[face[0]]), static_cast<lv_coord_t>(py[face[0]])},
            {static_cast<lv_coord_t>(px[face[1]]), static_cast<lv_coord_t>(py[face[1]])},
            {static_cast<lv_coord_t>(px[face[2]]), static_cast<lv_coord_t>(py[face[2]])},
            {static_cast<lv_coord_t>(px[face[3]]), static_cast<lv_coord_t>(py[face[3]])}
        };
        const uint8_t shade = static_cast<uint8_t>(50 + (i * 35));
        const lv_color_t faceColor = snake3dMixColor(color, lv_color_hex(0x081018), shade);
        snake3dFillQuad(drawCtx, quad, faceColor, LV_OPA_90);
        for (int e = 0; e < 4; ++e) {
            const int n = (e + 1) & 3;
            lvglDrawLineSeg(drawCtx, quad[e].x, quad[e].y, quad[n].x, quad[n].y, snake3dMixColor(color, lv_color_white(), 80), width, LV_OPA_COVER);
        }
    }
}

static void snake3dDrawFloorTile(lv_draw_ctx_t *drawCtx, int cx, int cy, float focal, const Snake3dCamera &cam,
                                 int gx, int gy, int planeCols, int planeRows, float tileSpacing)
{
    const float floorY = -tileSpacing * 0.11f;
    const float tileInset = tileSpacing * 0.05f;
    const float left = (static_cast<float>(gx) - (planeCols * 0.5f)) * tileSpacing + tileInset;
    const float right = left + tileSpacing - (tileInset * 2.0f);
    const float back = (((planeRows * 0.5f) - static_cast<float>(gy)) * tileSpacing) - tileInset;
    const float front = back - tileSpacing + (tileInset * 2.0f);
    Snake3dVec corners[4] = {
        {left, floorY, back},
        {right, floorY, back},
        {right, floorY, front},
        {left, floorY, front}
    };
    int px[4] = {};
    int py[4] = {};
    bool ok[4] = {};
    for (int i = 0; i < 4; ++i) {
        float depth = 0.0f;
        ok[i] = snake3dProjectPoint(corners[i], cam, cx, cy, focal, px[i], py[i], depth);
    }
    if (!(ok[0] && ok[1] && ok[2] && ok[3])) return;

    static const lv_color_t kGroundPalette[4] = {
        lv_color_hex(0x0D6E6E),
        lv_color_hex(0x1D4ED8),
        lv_color_hex(0x7C3AED),
        lv_color_hex(0x0F766E)
    };
    const uint8_t colorIdx = static_cast<uint8_t>((gx + (gy * 3)) & 0x03);
    const lv_color_t fill = kGroundPalette[colorIdx];
    const lv_color_t edge = snake3dMixColor(fill, lv_color_white(), 90);
    const lv_point_t quad[4] = {
        {static_cast<lv_coord_t>(px[0]), static_cast<lv_coord_t>(py[0])},
        {static_cast<lv_coord_t>(px[1]), static_cast<lv_coord_t>(py[1])},
        {static_cast<lv_coord_t>(px[2]), static_cast<lv_coord_t>(py[2])},
        {static_cast<lv_coord_t>(px[3]), static_cast<lv_coord_t>(py[3])}
    };
    snake3dFillQuad(drawCtx, quad, fill, LV_OPA_60);
    lvglDrawLineSeg(drawCtx, px[0], py[0], px[1], py[1], edge, 1, LV_OPA_COVER);
    lvglDrawLineSeg(drawCtx, px[1], py[1], px[2], py[2], edge, 1, LV_OPA_COVER);
    lvglDrawLineSeg(drawCtx, px[2], py[2], px[3], py[3], edge, 1, LV_OPA_COVER);
    lvglDrawLineSeg(drawCtx, px[3], py[3], px[0], py[0], edge, 1, LV_OPA_COVER);
}

void lvglSnake3dBoardDrawEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_DRAW_MAIN) return;
    lv_obj_t *obj = lv_event_get_target(e);
    lv_draw_ctx_t *drawCtx = lv_event_get_draw_ctx(e);
    if (!obj || !drawCtx) return;

    lv_area_t coords;
    lv_obj_get_coords(obj, &coords);
    const int width = static_cast<int>(coords.x2 - coords.x1 + 1);
    const int height = static_cast<int>(coords.y2 - coords.y1 + 1);
    const int centerX = static_cast<int>(coords.x1) + (width / 2);
    const int centerY = static_cast<int>(coords.y1) + (height / 2) + 30;
    const float spacing = static_cast<float>(min(width, height)) * 0.16f;
    const float groundTileSpacing = spacing * static_cast<float>(SNAKE3D_GROUND_TILE_SPAN);
    const float cubeHalf = spacing * 0.34f;
    const float focal = static_cast<float>(min(width, height)) * 1.35f;
    const Snake3dCamera cam = snake3dBuildCamera(spacing);

    for (int gy = 0; gy < SNAKE3D_GROUND_ROWS; ++gy) {
        for (int gx = 0; gx < SNAKE3D_GROUND_COLS; ++gx) {
            snake3dDrawFloorTile(drawCtx, centerX, centerY, focal, cam, gx, gy,
                                 SNAKE3D_GROUND_COLS, SNAKE3D_GROUND_ROWS, groundTileSpacing);
        }
    }

    for (int i = snake3dLen - 1; i >= 0; --i) {
        const Snake3dVec center = snake3dGridToWorld(static_cast<float>(snake3dX[i]),
                                                     static_cast<float>(snake3dY[i]),
                                                     static_cast<float>(snake3dZ[i]),
                                                     spacing);
        const lv_color_t color = (i == 0) ? lv_color_hex(0xD7FF7A) : lv_color_hex(0x58D06D);
        snake3dDrawCube(drawCtx, centerX, centerY, focal, cam, center, cubeHalf, color, (i == 0) ? 3 : 2);
    }

    const Snake3dVec foodCenter = snake3dGridToWorld(static_cast<float>(snake3dFoodX),
                                                     static_cast<float>(snake3dFoodY),
                                                     static_cast<float>(snake3dFoodZ),
                                                     spacing);
    snake3dDrawCube(drawCtx, centerX, centerY, focal, cam, foodCenter, cubeHalf * 0.8f, lv_color_hex(0xFF6C62), 2);
}

void lvglSnake3dDirEvent(lv_event_t *e)
{
    const int d = static_cast<int>(reinterpret_cast<intptr_t>(lv_event_get_user_data(e)));
    if (!snake3dStarted || snake3dPaused || snake3dGameOver) return;
    if ((d == 0 && snake3dDir != 2) || (d == 2 && snake3dDir != 0) ||
        (d == 1 && snake3dDir != 3) || (d == 3 && snake3dDir != 1) ||
        (d == 4 && snake3dDir != 5) || (d == 5 && snake3dDir != 4)) {
        snake3dNextDir = static_cast<int8_t>(d);
        if (d >= 0 && d <= 3) snake3dViewDir = static_cast<int8_t>(d);
    }
}

void lvglSnake3dPauseEvent(lv_event_t *e)
{
    (void)e;
    if (snake3dGameOver) return;
    if (!snake3dStarted) {
        snake3dStartGame();
    } else {
        snake3dPaused = !snake3dPaused;
        if (!snake3dPaused) snake3dLastStepMs = millis();
    }
    lvglRefreshSnake3dBoard();
}

void lvglSnake3dRestartEvent(lv_event_t *e)
{
    (void)e;
    snake3dStartGame();
    lvglRefreshSnake3dBoard();
}

void lvglRefreshSnake3dBoard()
{
    if (lvglSnake3dBoardObj) lv_obj_invalidate(lvglSnake3dBoardObj);
    if (lvglSnake3dScoreLabel) lv_label_set_text_fmt(lvglSnake3dScoreLabel, "Score %u", snake3dScore);
    if (lvglSnake3dBestLabel) lv_label_set_text_fmt(lvglSnake3dBestLabel, "Best %u", snake3dHighScore);
    if (lvglSnake3dPauseBtnLabel) lv_label_set_text(lvglSnake3dPauseBtnLabel, (!snake3dStarted || snake3dPaused) ? LV_SYMBOL_PLAY : LV_SYMBOL_PAUSE);

    if (!lvglSnake3dOverlay) return;
    const bool showOverlay = !snake3dStarted || snake3dPaused || snake3dGameOver;
    const bool pauseOverlay = snake3dPaused && snake3dStarted && !snake3dGameOver;
    lv_coord_t overlayW = pauseOverlay ? min<lv_coord_t>(DISPLAY_WIDTH - 64, 150) : min<lv_coord_t>(DISPLAY_WIDTH - 24, 250);
    lv_coord_t overlayH = pauseOverlay ? 84 : min<lv_coord_t>(UI_CONTENT_H - 24, 255);
    if (lvglSnake3dBoardObj) {
        lv_area_t boardCoords;
        lv_obj_get_coords(lvglSnake3dBoardObj, &boardCoords);
        const lv_coord_t boardWidth = static_cast<lv_coord_t>(boardCoords.x2 - boardCoords.x1 + 1);
        const lv_coord_t boardHeight = static_cast<lv_coord_t>(boardCoords.y2 - boardCoords.y1 + 1);
        overlayW = min<lv_coord_t>(overlayW, max<lv_coord_t>(140, boardWidth - 16));
        overlayH = min<lv_coord_t>(overlayH, max<lv_coord_t>(84, boardHeight - 16));
    }
    lv_obj_set_size(lvglSnake3dOverlay, overlayW, overlayH);
    lv_obj_center(lvglSnake3dOverlay);
    lv_obj_move_foreground(lvglSnake3dOverlay);
    if (showOverlay) lv_obj_clear_flag(lvglSnake3dOverlay, LV_OBJ_FLAG_HIDDEN);
    else lv_obj_add_flag(lvglSnake3dOverlay, LV_OBJ_FLAG_HIDDEN);

    if (lvglSnake3dOverlayTitle) {
        if (pauseOverlay) lv_label_set_text(lvglSnake3dOverlayTitle, "PAUSE");
        else lv_label_set_text(lvglSnake3dOverlayTitle, snake3dGameOver ? "Game Over" : "Snake 3D");
    }
        if (lvglSnake3dOverlaySubLabel) {
            String msg = pauseOverlay ? String("Resume to keep climbing the 3D grid")
                                      : (snake3dGameOver ? ("Score " + String(snake3dScore) + "\nBest " + String(snake3dHighScore))
                                                     : ("Chase camera 3D snake\nBest " + String(snake3dHighScore) + "  Last " + String(snake3dLastScore)));
        lv_label_set_text(lvglSnake3dOverlaySubLabel, msg.c_str());
    }
    if (lvglSnake3dOverlayBtn) {
        if (pauseOverlay) lv_obj_add_flag(lvglSnake3dOverlayBtn, LV_OBJ_FLAG_HIDDEN);
        else lv_obj_clear_flag(lvglSnake3dOverlayBtn, LV_OBJ_FLAG_HIDDEN);
    }
    if (lvglSnake3dOverlayBtnLabel && !pauseOverlay) lv_label_set_text(lvglSnake3dOverlayBtnLabel, snake3dGameOver ? "Replay" : "Start");
}
#endif

#if defined(BOARD_ESP32S3_3248S035_N16R8)
void lvglOpenSnake3dEvent(lv_event_t *e)
{
    (void)e;
    lvglEnsureScreenBuilt(UI_GAME_SNAKE3D);
    snake3dPrepareGame();
    if (screensaverActive) screensaverSetActive(false);
    if (!displayAwake) displaySetAwake(true);
    lastUserActivityMs = millis();
    lvglRefreshSnake3dBoard();
    lvglOpenScreen(UI_GAME_SNAKE3D, LV_SCR_LOAD_ANIM_MOVE_LEFT);
}
#endif
