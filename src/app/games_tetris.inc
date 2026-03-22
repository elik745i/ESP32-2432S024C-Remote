static const int8_t TETRIS_BASE[7][4][2] = {
    {{0, 1}, {1, 1}, {2, 1}, {3, 1}}, // I
    {{0, 0}, {0, 1}, {1, 1}, {2, 1}}, // J
    {{2, 0}, {0, 1}, {1, 1}, {2, 1}}, // L
    {{1, 0}, {2, 0}, {1, 1}, {2, 1}}, // O
    {{1, 0}, {2, 0}, {0, 1}, {1, 1}}, // S
    {{1, 0}, {0, 1}, {1, 1}, {2, 1}}, // T
    {{0, 0}, {1, 0}, {1, 1}, {2, 1}}  // Z
};

bool tetrisCellFor(int type, int rot, int i, int &ox, int &oy)
{
    int x = TETRIS_BASE[type][i][0];
    int y = TETRIS_BASE[type][i][1];
    rot &= 3;
    if (rot == 1) {
        int tx = x;
        x = 3 - y;
        y = tx;
    } else if (rot == 2) {
        x = 3 - x;
        y = 3 - y;
    } else if (rot == 3) {
        int tx = x;
        x = y;
        y = 3 - tx;
    }
    ox = x;
    oy = y;
    return true;
}

bool tetrisCanPlace(int px, int py, int type, int rot)
{
    for (int i = 0; i < 4; i++) {
        int ox = 0, oy = 0;
        tetrisCellFor(type, rot, i, ox, oy);
        int gx = px + ox;
        int gy = py + oy;
        if (gx < 0 || gx >= TETRIS_COLS || gy < 0 || gy >= TETRIS_ROWS) return false;
        if (tetrisGrid[gy][gx] != 0) return false;
    }
    return true;
}

void tetrisSpawnPiece()
{
    tetrisDropAnimFlags = 0;
    tetrisAnimDurationMs = 0;
    tetrisType = static_cast<int8_t>(random(0, 7));
    tetrisRot = 0;
    tetrisX = 3;
    tetrisY = 0;
    if (!tetrisCanPlace(tetrisX, tetrisY, tetrisType, tetrisRot)) {
        tetrisGameOver = true;
        tetrisStarted = false;
        tetrisLastScore = tetrisScore;
        saveGameValue("tetris_last", tetrisLastScore);
        tetrisMaybeStoreHighScore(true);
    }
}

void tetrisClearLines()
{
    for (int y = TETRIS_ROWS - 1; y >= 0; y--) {
        bool full = true;
        for (int x = 0; x < TETRIS_COLS; x++) {
            if (tetrisGrid[y][x] == 0) {
                full = false;
                break;
            }
        }
        if (!full) continue;
        for (int yy = y; yy > 0; yy--) {
            for (int x = 0; x < TETRIS_COLS; x++) tetrisGrid[yy][x] = tetrisGrid[yy - 1][x];
        }
        for (int x = 0; x < TETRIS_COLS; x++) tetrisGrid[0][x] = 0;
        tetrisScore += 100;
        tetrisMaybeStoreHighScore();
        y++;
    }
}

void tetrisLockPiece()
{
    for (int i = 0; i < 4; i++) {
        int ox = 0, oy = 0;
        tetrisCellFor(tetrisType, tetrisRot, i, ox, oy);
        int gx = tetrisX + ox;
        int gy = tetrisY + oy;
        if (gx >= 0 && gx < TETRIS_COLS && gy >= 0 && gy < TETRIS_ROWS) tetrisGrid[gy][gx] = static_cast<uint8_t>(tetrisType + 1);
    }
    tetrisClearLines();
    tetrisSpawnPiece();
}

void tetrisResetGame()
{
    for (int y = 0; y < TETRIS_ROWS; y++) {
        for (int x = 0; x < TETRIS_COLS; x++) tetrisGrid[y][x] = 0;
    }
    tetrisDropAnimFlags = 0;
    tetrisAnimDurationMs = 0;
    tetrisStarted = true;
    tetrisPaused = false;
    tetrisGameOver = false;
    tetrisScore = 0;
    tetrisSpawnPiece();
    tetrisLastStepMs = millis();
}

void tetrisPrepareGame()
{
    tetrisResetGame();
    tetrisStarted = false;
    tetrisPaused = false;
    tetrisGameOver = false;
}

void tetrisStartGame()
{
    tetrisResetGame();
    tetrisStarted = true;
    tetrisPaused = false;
    tetrisLastStepMs = millis();
}

static unsigned long tetrisCurrentStepMs()
{
    const unsigned long level = static_cast<unsigned long>(tetrisScore / 100U);
    const unsigned long reduction = level * TETRIS_STEP_MS_DELTA_PER_100_SCORE;
    if (reduction >= (TETRIS_STEP_MS_START - TETRIS_STEP_MS_MIN)) return TETRIS_STEP_MS_MIN;
    return TETRIS_STEP_MS_START - reduction;
}

void tetrisMove(int dx)
{
    if (!tetrisStarted || tetrisPaused || tetrisGameOver || (tetrisDropAnimFlags & TETRIS_DROP_ANIM_ACTIVE)) return;
    if (tetrisCanPlace(tetrisX + dx, tetrisY, tetrisType, tetrisRot)) {
        tetrisX += static_cast<int8_t>(dx);
        lvglRefreshTetrisBoard();
    }
}

void tetrisRotate()
{
    if (!tetrisStarted || tetrisPaused || tetrisGameOver || (tetrisDropAnimFlags & TETRIS_DROP_ANIM_ACTIVE)) return;
    int nr = (tetrisRot + 1) & 3;
    if (tetrisCanPlace(tetrisX, tetrisY, tetrisType, nr)) tetrisRot = static_cast<int8_t>(nr);
    else if (tetrisCanPlace(tetrisX - 1, tetrisY, tetrisType, nr)) {
        tetrisX--;
        tetrisRot = static_cast<int8_t>(nr);
    } else if (tetrisCanPlace(tetrisX + 1, tetrisY, tetrisType, nr)) {
        tetrisX++;
        tetrisRot = static_cast<int8_t>(nr);
    }
    lvglRefreshTetrisBoard();
}

void tetrisDrop()
{
    if (!tetrisStarted || tetrisPaused || tetrisGameOver || (tetrisDropAnimFlags & TETRIS_DROP_ANIM_ACTIVE)) return;
    const int startY = tetrisY;
    while (tetrisCanPlace(tetrisX, tetrisY + 1, tetrisType, tetrisRot)) tetrisY++;
    if (tetrisY == startY) {
        tetrisLockPiece();
    } else {
        tetrisAnimFromY = static_cast<int8_t>(startY);
        tetrisAnimStartTick = static_cast<uint16_t>(millis());
        const unsigned long travelMs = static_cast<unsigned long>(tetrisY - startY) * TETRIS_DROP_ANIM_MS_PER_ROW;
        tetrisAnimDurationMs = min<unsigned long>(TETRIS_DROP_ANIM_MS_MAX,
                                                  max<unsigned long>(TETRIS_DROP_ANIM_MS_MIN, travelMs));
        tetrisDropAnimFlags = TETRIS_DROP_ANIM_ACTIVE | TETRIS_DROP_LOCK_PENDING;
    }
    lvglRefreshTetrisBoard();
}

void tetrisStepDown()
{
    if (!tetrisStarted || tetrisPaused || tetrisGameOver || (tetrisDropAnimFlags & TETRIS_DROP_ANIM_ACTIVE)) return;
    if (tetrisCanPlace(tetrisX, tetrisY + 1, tetrisType, tetrisRot)) tetrisY++;
    else tetrisLockPiece();
}

void tetrisAnimationService()
{
    if (uiScreen != UI_GAME_TETRIS || (tetrisDropAnimFlags & TETRIS_DROP_ANIM_ACTIVE) == 0) return;
    const uint16_t elapsedTick = static_cast<uint16_t>(millis()) - tetrisAnimStartTick;
    if (elapsedTick >= tetrisAnimDurationMs) {
        const bool lockPending = (tetrisDropAnimFlags & TETRIS_DROP_LOCK_PENDING) != 0;
        tetrisDropAnimFlags = 0;
        tetrisAnimDurationMs = 0;
        if (lockPending) tetrisLockPiece();
        lvglRefreshTetrisBoard();
        return;
    }
    if (lvglTetrisBoardObj) lv_obj_invalidate(lvglTetrisBoardObj);
}

void tetrisTick()
{
    if (uiScreen != UI_GAME_TETRIS) return;
    if (!tetrisStarted || tetrisPaused || tetrisGameOver || (tetrisDropAnimFlags & TETRIS_DROP_ANIM_ACTIVE)) return;
    if (millis() - tetrisLastStepMs < tetrisCurrentStepMs()) return;
    tetrisLastStepMs = millis();
    tetrisStepDown();
    lvglRefreshTetrisBoard();
}

// Legacy unused tetris back callback moved to app/garbage.inc:
// lvglTetrisBackEvent(lv_event_t *)

void lvglTetrisRestartEvent(lv_event_t *e)
{
    (void)e;
    tetrisStartGame();
    lvglRefreshTetrisBoard();
}

void lvglTetrisPauseEvent(lv_event_t *e)
{
    (void)e;
    if (tetrisGameOver) return;
    if (tetrisDropAnimFlags & TETRIS_DROP_ANIM_ACTIVE) return;
    if (!tetrisStarted) {
        tetrisStartGame();
    } else {
        tetrisPaused = !tetrisPaused;
        if (!tetrisPaused) tetrisLastStepMs = millis();
    }
    lvglRefreshTetrisBoard();
}

void lvglTetrisMoveLeftEvent(lv_event_t *e) { (void)e; tetrisMove(-1); }
void lvglTetrisMoveRightEvent(lv_event_t *e) { (void)e; tetrisMove(1); }
void lvglTetrisRotateEvent(lv_event_t *e) { (void)e; tetrisRotate(); }
void lvglTetrisDropEvent(lv_event_t *e) { (void)e; tetrisDrop(); }

void lvglTetrisBoardDrawEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_DRAW_MAIN) return;
    lv_obj_t *obj = lv_event_get_target(e);
    lv_draw_ctx_t *drawCtx = lv_event_get_draw_ctx(e);
    if (!obj || !drawCtx) return;

    static const lv_color_t cols[8] = {
        lv_color_hex(0x111922), lv_color_hex(0x26C6DA), lv_color_hex(0x1565C0), lv_color_hex(0xEF6C00),
        lv_color_hex(0xFDD835), lv_color_hex(0x43A047), lv_color_hex(0x8E24AA), lv_color_hex(0xD32F2F)
    };

    lv_area_t coords;
    lv_obj_get_coords(obj, &coords);
    const int gap = 1;
    const int width = static_cast<int>(coords.x2 - coords.x1 + 1);
    const int height = static_cast<int>(coords.y2 - coords.y1 + 1);
    const int cellW = (width - ((TETRIS_COLS - 1) * gap)) / TETRIS_COLS;
    const int cellH = (height - ((TETRIS_ROWS - 1) * gap)) / TETRIS_ROWS;
    if (cellW < 2 || cellH < 2) return;

    for (int y = 0; y < TETRIS_ROWS; y++) {
        for (int x = 0; x < TETRIS_COLS; x++) {
            const int px = static_cast<int>(coords.x1) + x * (cellW + gap);
            const int py = static_cast<int>(coords.y1) + y * (cellH + gap);
            lvglDrawGridCell(drawCtx, px, py, cellW, cellH, cols[tetrisGrid[y][x] & 0x07]);
        }
    }

    const bool dropAnimating = (tetrisDropAnimFlags & TETRIS_DROP_ANIM_ACTIVE) != 0;
    const int drawPieceBaseY = dropAnimating ? static_cast<int>(tetrisAnimFromY) : static_cast<int>(tetrisY);
    int animOffsetPx = 0;
    if (dropAnimating && tetrisAnimDurationMs > 0) {
        const uint16_t elapsedTick = static_cast<uint16_t>(millis()) - tetrisAnimStartTick;
        const unsigned long elapsed = min<unsigned long>(elapsedTick, tetrisAnimDurationMs);
        const int totalRows = static_cast<int>(tetrisY - tetrisAnimFromY);
        const int stepPx = cellH + gap;
        animOffsetPx = static_cast<int>((static_cast<uint64_t>(stepPx * totalRows) * elapsed) / tetrisAnimDurationMs);
    }

    for (int i = 0; i < 4; i++) {
        int ox = 0, oy = 0;
        tetrisCellFor(tetrisType, tetrisRot, i, ox, oy);
        int gx = tetrisX + ox;
        int gy = drawPieceBaseY + oy;
        if (gx < 0 || gx >= TETRIS_COLS || gy < 0 || gy >= TETRIS_ROWS) continue;
        const int px = static_cast<int>(coords.x1) + gx * (cellW + gap);
        const int py = static_cast<int>(coords.y1) + gy * (cellH + gap) + animOffsetPx;
        lvglDrawGridCell(drawCtx, px, py, cellW, cellH, cols[(tetrisType + 1) & 0x07]);
    }
}

void lvglRefreshTetrisBoard()
{
    if (lvglTetrisBoardObj) lv_obj_invalidate(lvglTetrisBoardObj);
    if (lvglTetrisScoreLabel) {
        lv_label_set_text_fmt(lvglTetrisScoreLabel, "Score %u", tetrisScore);
    }
    if (lvglTetrisBestLabel) {
        lv_label_set_text_fmt(lvglTetrisBestLabel, "Best %u", tetrisHighScore);
    }
    if (lvglTetrisOverlay) {
        const bool showOverlay = !tetrisStarted || tetrisGameOver || tetrisPaused;
        const bool pauseOverlay = tetrisPaused && tetrisStarted && !tetrisGameOver;
        lv_coord_t overlayW = pauseOverlay ? min<lv_coord_t>(DISPLAY_WIDTH - 64, 140) : min<lv_coord_t>(DISPLAY_WIDTH - 20, 324);
        lv_coord_t overlayH = pauseOverlay ? 76 : min<lv_coord_t>(UI_CONTENT_H - 8, 467);
        if (lvglTetrisBoardObj) {
            lv_area_t boardCoords;
            lv_obj_get_coords(lvglTetrisBoardObj, &boardCoords);
            const lv_coord_t boardWidth = static_cast<lv_coord_t>(boardCoords.x2 - boardCoords.x1 + 1);
            const lv_coord_t boardHeight = static_cast<lv_coord_t>(boardCoords.y2 - boardCoords.y1 + 1);
            overlayW = min<lv_coord_t>(overlayW, max<lv_coord_t>(120, boardWidth - 16));
            overlayH = min<lv_coord_t>(overlayH, max<lv_coord_t>(137, boardHeight - 16));
        }
        lv_obj_set_size(lvglTetrisOverlay, overlayW, overlayH);
        lv_obj_center(lvglTetrisOverlay);
        lv_obj_move_foreground(lvglTetrisOverlay);
        if (showOverlay) lv_obj_clear_flag(lvglTetrisOverlay, LV_OBJ_FLAG_HIDDEN);
        else lv_obj_add_flag(lvglTetrisOverlay, LV_OBJ_FLAG_HIDDEN);
        if (lvglTetrisOverlayTitle) {
            if (pauseOverlay) lv_label_set_text(lvglTetrisOverlayTitle, "PAUSE");
            else lv_label_set_text(lvglTetrisOverlayTitle, tetrisGameOver ? "Game Over" : "Tetris");
        }
        if (lvglTetrisOverlaySubLabel) {
            String msg = pauseOverlay ? String("Tap center button to resume")
                                      : (tetrisGameOver ? ("Score " + String(tetrisScore) + "\nBest " + String(tetrisHighScore))
                                                       : ("Tap Start when ready\nBest " + String(tetrisHighScore) + "\nLast " + String(tetrisLastScore)));
            lv_label_set_text(lvglTetrisOverlaySubLabel, msg.c_str());
        }
        if (lvglTetrisOverlayBtn) {
            if (pauseOverlay) lv_obj_add_flag(lvglTetrisOverlayBtn, LV_OBJ_FLAG_HIDDEN);
            else lv_obj_clear_flag(lvglTetrisOverlayBtn, LV_OBJ_FLAG_HIDDEN);
        }
        if (lvglTetrisOverlayBtnLabel && !pauseOverlay) lv_label_set_text(lvglTetrisOverlayBtnLabel, tetrisGameOver ? "Replay" : "Start");
    }
    if (lvglTetrisPauseBtnLabel) lv_label_set_text(lvglTetrisPauseBtnLabel, (!tetrisStarted || tetrisPaused) ? LV_SYMBOL_PLAY : LV_SYMBOL_PAUSE);
}

void lvglOpenTetrisEvent(lv_event_t *e)
{
    (void)e;
    lvglEnsureScreenBuilt(UI_GAME_TETRIS);
    tetrisPrepareGame();
    if (screensaverActive) screensaverSetActive(false);
    if (!displayAwake) displaySetAwake(true);
    lastUserActivityMs = millis();
    lvglRefreshTetrisBoard();
    lvglOpenScreen(UI_GAME_TETRIS, LV_SCR_LOAD_ANIM_MOVE_LEFT);
}
