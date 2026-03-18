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
