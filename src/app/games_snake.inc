bool snakeCellOccupied(int x, int y)
{
    for (int i = 0; i < snakeLen; i++) {
        if (snakeX[i] == x && snakeY[i] == y) return true;
    }
    return false;
}

void snakeSpawnFood()
{
    for (int tries = 0; tries < 300; tries++) {
        int fx = random(0, SNAKE_COLS);
        int fy = random(0, SNAKE_ROWS);
        if (!snakeCellOccupied(fx, fy)) {
            snakeFoodX = static_cast<int8_t>(fx);
            snakeFoodY = static_cast<int8_t>(fy);
            return;
        }
    }
    snakeFoodX = 0;
    snakeFoodY = 0;
}

void snakeResetGame()
{
    snakeLen = 4;
    snakeDir = 1;
    snakeNextDir = 1;
    snakeStarted = true;
    snakeGameOver = false;
    snakeScore = 0;
    const int cx = SNAKE_COLS / 2;
    const int cy = SNAKE_ROWS / 2;
    for (int i = 0; i < snakeLen; i++) {
        snakeX[i] = static_cast<int8_t>(cx - i);
        snakeY[i] = static_cast<int8_t>(cy);
    }
    snakeSpawnFood();
    snakeLastStepMs = millis();
}

void snakePrepareGame()
{
    snakeResetGame();
    snakeStarted = false;
    snakePaused = false;
    snakeGameOver = false;
}

void snakeStartGame()
{
    snakeResetGame();
    snakeStarted = true;
    snakePaused = false;
    snakeLastStepMs = millis();
}

static unsigned long snakeCurrentStepMs()
{
    const unsigned long foods = static_cast<unsigned long>(snakeScore / 10U);
    const unsigned long reduction = foods * SNAKE_STEP_MS_DELTA_PER_FOOD;
    if (reduction >= (SNAKE_STEP_MS_START - SNAKE_STEP_MS_MIN)) return SNAKE_STEP_MS_MIN;
    return SNAKE_STEP_MS_START - reduction;
}

void snakeStep()
{
    if (!snakeStarted || snakePaused || snakeGameOver) return;
    snakeDir = snakeNextDir;
    int nx = snakeX[0];
    int ny = snakeY[0];
    if (snakeDir == 0) ny--;
    else if (snakeDir == 1) nx++;
    else if (snakeDir == 2) ny++;
    else nx--;

    if (nx < 0 || nx >= SNAKE_COLS || ny < 0 || ny >= SNAKE_ROWS) {
        snakeGameOver = true;
        snakeStarted = false;
        snakeLastScore = snakeScore;
        saveGameValue("snake_last", snakeLastScore);
        snakeMaybeStoreHighScore(true);
        return;
    }
    if (snakeCellOccupied(nx, ny)) {
        snakeGameOver = true;
        snakeStarted = false;
        snakeLastScore = snakeScore;
        saveGameValue("snake_last", snakeLastScore);
        snakeMaybeStoreHighScore(true);
        return;
    }

    bool ate = (nx == snakeFoodX && ny == snakeFoodY);
    int oldLen = snakeLen;
    if (ate && snakeLen < SNAKE_MAX_CELLS) snakeLen++;
    for (int i = snakeLen - 1; i > 0; i--) {
        snakeX[i] = snakeX[i - 1];
        snakeY[i] = snakeY[i - 1];
    }
    snakeX[0] = static_cast<int8_t>(nx);
    snakeY[0] = static_cast<int8_t>(ny);

    if (ate) {
        snakeScore += 10;
        snakeMaybeStoreHighScore();
        snakeSpawnFood();
    } else if (oldLen < snakeLen) {
        snakeLen = oldLen;
    }
}

void snakeTick()
{
    if (uiScreen != UI_GAME_SNAKE) return;
    if (!snakeStarted || snakePaused || snakeGameOver) return;
    if (millis() - snakeLastStepMs < snakeCurrentStepMs()) return;
    snakeLastStepMs = millis();
    snakeStep();
    lvglRefreshSnakeBoard();
}

void lvglSnakeDirEvent(lv_event_t *e)
{
    if (!snakeStarted || snakePaused || snakeGameOver) return;
    const int d = static_cast<int>(reinterpret_cast<intptr_t>(lv_event_get_user_data(e)));
    if (d == 3 && snakeDir != 1) snakeNextDir = 3;
    else if (d == 1 && snakeDir != 3) snakeNextDir = 1;
    else if (d == 0 && snakeDir != 2) snakeNextDir = 0;
    else if (d == 2 && snakeDir != 0) snakeNextDir = 2;
}

void lvglSnakePauseEvent(lv_event_t *e)
{
    (void)e;
    if (snakeGameOver) return;
    if (!snakeStarted) {
        snakeStartGame();
    } else {
        snakePaused = !snakePaused;
        if (!snakePaused) snakeLastStepMs = millis();
    }
    lvglRefreshSnakeBoard();
}

// Legacy unused snake back callback moved to app/garbage.inc:
// lvglSnakeBackEvent(lv_event_t *)

void lvglSnakeRestartEvent(lv_event_t *e)
{
    (void)e;
    snakeStartGame();
    lvglRefreshSnakeBoard();
}

void lvglDrawGridCell(lv_draw_ctx_t *drawCtx, int x, int y, int w, int h, lv_color_t color)
{
    lv_draw_rect_dsc_t dsc;
    lv_draw_rect_dsc_init(&dsc);
    dsc.bg_opa = LV_OPA_COVER;
    dsc.bg_color = color;
    dsc.border_width = 0;
    dsc.radius = 1;
    lv_area_t a;
    a.x1 = static_cast<lv_coord_t>(x);
    a.y1 = static_cast<lv_coord_t>(y);
    a.x2 = static_cast<lv_coord_t>(x + w - 1);
    a.y2 = static_cast<lv_coord_t>(y + h - 1);
    lv_draw_rect(drawCtx, &dsc, &a);
}

void lvglSnakeBoardDrawEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_DRAW_MAIN) return;
    lv_obj_t *obj = lv_event_get_target(e);
    lv_draw_ctx_t *drawCtx = lv_event_get_draw_ctx(e);
    if (!obj || !drawCtx) return;

    lv_area_t coords;
    lv_obj_get_coords(obj, &coords);
    const int gap = 1;
    const int width = static_cast<int>(coords.x2 - coords.x1 + 1);
    const int height = static_cast<int>(coords.y2 - coords.y1 + 1);
    const int cellW = (width - ((SNAKE_COLS - 1) * gap)) / SNAKE_COLS;
    const int cellH = (height - ((SNAKE_ROWS - 1) * gap)) / SNAKE_ROWS;
    if (cellW < 2 || cellH < 2) return;

    const lv_color_t bg = lv_color_hex(0x1B2430);
    const lv_color_t food = lv_color_hex(0xC62828);
    const lv_color_t head = lv_color_hex(0xC5E84A);
    const lv_color_t body = lv_color_hex(0x4CAF50);

    for (int y = 0; y < SNAKE_ROWS; y++) {
        for (int x = 0; x < SNAKE_COLS; x++) {
            const int px = static_cast<int>(coords.x1) + x * (cellW + gap);
            const int py = static_cast<int>(coords.y1) + y * (cellH + gap);
            lvglDrawGridCell(drawCtx, px, py, cellW, cellH, bg);
        }
    }

    if (snakeFoodX >= 0 && snakeFoodX < SNAKE_COLS && snakeFoodY >= 0 && snakeFoodY < SNAKE_ROWS) {
        const int fx = static_cast<int>(coords.x1) + snakeFoodX * (cellW + gap);
        const int fy = static_cast<int>(coords.y1) + snakeFoodY * (cellH + gap);
        lvglDrawGridCell(drawCtx, fx, fy, cellW, cellH, food);
    }
    for (int i = snakeLen - 1; i >= 0; i--) {
        const int sx = snakeX[i];
        const int sy = snakeY[i];
        if (sx < 0 || sx >= SNAKE_COLS || sy < 0 || sy >= SNAKE_ROWS) continue;
        const int px = static_cast<int>(coords.x1) + sx * (cellW + gap);
        const int py = static_cast<int>(coords.y1) + sy * (cellH + gap);
        lvglDrawGridCell(drawCtx, px, py, cellW, cellH, (i == 0) ? head : body);
    }
}

void lvglRefreshSnakeBoard()
{
    if (lvglSnakeBoardObj) lv_obj_invalidate(lvglSnakeBoardObj);
    if (lvglSnakeScoreLabel) {
        lv_label_set_text_fmt(lvglSnakeScoreLabel, "Score %u", snakeScore);
    }
    if (lvglSnakeBestLabel) {
        lv_label_set_text_fmt(lvglSnakeBestLabel, "Best %u", snakeHighScore);
    }
    if (lvglSnakeOverlay) {
        const bool showOverlay = !snakeStarted || snakeGameOver || snakePaused;
        const bool pauseOverlay = snakePaused && snakeStarted && !snakeGameOver;
        lv_coord_t overlayW = pauseOverlay ? min<lv_coord_t>(DISPLAY_WIDTH - 64, 140) : min<lv_coord_t>(DISPLAY_WIDTH - 20, 324);
        lv_coord_t overlayH = pauseOverlay ? 76 : min<lv_coord_t>(UI_CONTENT_H - 8, 467);
        if (lvglSnakeBoardObj) {
            lv_area_t boardCoords;
            lv_obj_get_coords(lvglSnakeBoardObj, &boardCoords);
            const lv_coord_t boardWidth = static_cast<lv_coord_t>(boardCoords.x2 - boardCoords.x1 + 1);
            const lv_coord_t boardHeight = static_cast<lv_coord_t>(boardCoords.y2 - boardCoords.y1 + 1);
            overlayW = min<lv_coord_t>(overlayW, max<lv_coord_t>(120, boardWidth - 16));
            overlayH = min<lv_coord_t>(overlayH, max<lv_coord_t>(137, boardHeight - 16));
        }
        lv_obj_set_size(lvglSnakeOverlay, overlayW, overlayH);
        lv_obj_center(lvglSnakeOverlay);
        lv_obj_move_foreground(lvglSnakeOverlay);
        if (showOverlay) lv_obj_clear_flag(lvglSnakeOverlay, LV_OBJ_FLAG_HIDDEN);
        else lv_obj_add_flag(lvglSnakeOverlay, LV_OBJ_FLAG_HIDDEN);
        if (lvglSnakeOverlayTitle) {
            if (pauseOverlay) lv_label_set_text(lvglSnakeOverlayTitle, "PAUSE");
            else lv_label_set_text(lvglSnakeOverlayTitle, snakeGameOver ? "Game Over" : "Snake");
        }
        if (lvglSnakeOverlaySubLabel) {
            String msg = pauseOverlay ? String("Tap center button to resume")
                                      : (snakeGameOver ? ("Score " + String(snakeScore) + "\nBest " + String(snakeHighScore))
                                                       : ("Tap Start when ready\nBest " + String(snakeHighScore) + "\nLast " + String(snakeLastScore)));
            lv_label_set_text(lvglSnakeOverlaySubLabel, msg.c_str());
        }
        if (lvglSnakeOverlayBtn) {
            if (pauseOverlay) lv_obj_add_flag(lvglSnakeOverlayBtn, LV_OBJ_FLAG_HIDDEN);
            else lv_obj_clear_flag(lvglSnakeOverlayBtn, LV_OBJ_FLAG_HIDDEN);
        }
        if (lvglSnakeOverlayBtnLabel && !pauseOverlay) lv_label_set_text(lvglSnakeOverlayBtnLabel, snakeGameOver ? "Replay" : "Start");
    }
    if (lvglSnakePauseBtnLabel) lv_label_set_text(lvglSnakePauseBtnLabel, (!snakeStarted || snakePaused) ? LV_SYMBOL_PLAY : LV_SYMBOL_PAUSE);
}

void lvglOpenSnakeEvent(lv_event_t *e)
{
    (void)e;
    lvglEnsureScreenBuilt(UI_GAME_SNAKE);
    snakePrepareGame();
    if (screensaverActive) screensaverSetActive(false);
    if (!displayAwake) displaySetAwake(true);
    lastUserActivityMs = millis();
    lvglRefreshSnakeBoard();
    lvglOpenScreen(UI_GAME_SNAKE, LV_SCR_LOAD_ANIM_MOVE_LEFT);
}
