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
