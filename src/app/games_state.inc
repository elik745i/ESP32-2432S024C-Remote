static constexpr int SNAKE_COLS = 12;
static constexpr int SNAKE_ROWS = 14;
// Legacy unused snake layout constants were moved to app/garbage.inc:
// SNAKE_CELL, SNAKE_BOARD_X, SNAKE_BOARD_Y
static constexpr int SNAKE_MAX_CELLS = SNAKE_COLS * SNAKE_ROWS;
static constexpr unsigned long SNAKE_STEP_MS_START = 840;
static constexpr unsigned long SNAKE_STEP_MS_MIN = 440;
static constexpr unsigned long SNAKE_STEP_MS_DELTA_PER_FOOD = 10;
int8_t snakeDir = 3; // 0 up,1 right,2 down,3 left
int8_t snakeNextDir = 3;
int snakeLen = 0;
int8_t snakeX[SNAKE_MAX_CELLS];
int8_t snakeY[SNAKE_MAX_CELLS];
int8_t snakeFoodX = 0;
int8_t snakeFoodY = 0;
bool snakeStarted = false;
bool snakePaused = false;
bool snakeGameOver = false;
uint16_t snakeScore = 0;
uint16_t snakeHighScore = 0;
uint16_t snakeLastScore = 0;
unsigned long snakeLastStepMs = 0;

#if defined(BOARD_ESP32S3_3248S035_N16R8)
static constexpr int SNAKE3D_COLS = 5;
static constexpr int SNAKE3D_ROWS = 5;
static constexpr int SNAKE3D_LAYERS = 6;
static constexpr int SNAKE3D_GROUND_SCALE = 4;
static constexpr int SNAKE3D_GROUND_TILE_SPAN = 2;
static constexpr int SNAKE3D_GROUND_COLS = (SNAKE3D_COLS * SNAKE3D_GROUND_SCALE) / SNAKE3D_GROUND_TILE_SPAN;
static constexpr int SNAKE3D_GROUND_ROWS = (SNAKE3D_ROWS * SNAKE3D_GROUND_SCALE) / SNAKE3D_GROUND_TILE_SPAN;
static constexpr int SNAKE3D_MAX_CELLS = SNAKE3D_COLS * SNAKE3D_ROWS * SNAKE3D_LAYERS;
static constexpr unsigned long SNAKE3D_STEP_MS_START = 720;
static constexpr unsigned long SNAKE3D_STEP_MS_MIN = 280;
static constexpr unsigned long SNAKE3D_STEP_MS_DELTA_PER_FOOD = 12;
int8_t snake3dDir = 1; // 0 up,1 right,2 down,3 left,4 climb,5 dive
int8_t snake3dNextDir = 1;
int8_t snake3dViewDir = 1;
int snake3dLen = 0;
int8_t snake3dX[SNAKE3D_MAX_CELLS];
int8_t snake3dY[SNAKE3D_MAX_CELLS];
int8_t snake3dZ[SNAKE3D_MAX_CELLS];
int8_t snake3dFoodX = 0;
int8_t snake3dFoodY = 0;
int8_t snake3dFoodZ = 0;
bool snake3dStarted = false;
bool snake3dPaused = false;
bool snake3dGameOver = false;
uint16_t snake3dScore = 0;
uint16_t snake3dHighScore = 0;
uint16_t snake3dLastScore = 0;
unsigned long snake3dLastStepMs = 0;
#endif

static constexpr int TETRIS_COLS = 10;
static constexpr int TETRIS_ROWS = 16;
// Legacy unused tetris layout constants were moved to app/garbage.inc:
// TETRIS_CELL, TETRIS_BOARD_X, TETRIS_BOARD_Y
static constexpr unsigned long TETRIS_STEP_MS_START = 900;
static constexpr unsigned long TETRIS_STEP_MS_MIN = 420;
static constexpr unsigned long TETRIS_STEP_MS_DELTA_PER_100_SCORE = 10;
static constexpr unsigned long TETRIS_DROP_ANIM_MS_MIN = 70UL;
static constexpr unsigned long TETRIS_DROP_ANIM_MS_PER_ROW = 24UL;
static constexpr unsigned long TETRIS_DROP_ANIM_MS_MAX = 260UL;
static constexpr uint8_t TETRIS_DROP_ANIM_ACTIVE = 0x01;
static constexpr uint8_t TETRIS_DROP_LOCK_PENDING = 0x02;
uint8_t tetrisGrid[TETRIS_ROWS][TETRIS_COLS];
int8_t tetrisType = 0;
int8_t tetrisRot = 0;
int8_t tetrisX = 3;
int8_t tetrisY = 0;
bool tetrisStarted = false;
bool tetrisPaused = false;
bool tetrisGameOver = false;
uint16_t tetrisScore = 0;
uint16_t tetrisHighScore = 0;
uint16_t tetrisLastScore = 0;
unsigned long tetrisLastStepMs = 0;
uint8_t tetrisDropAnimFlags = 0;
int8_t tetrisAnimFromY = 0;
uint16_t tetrisAnimStartTick = 0;
uint16_t tetrisAnimDurationMs = 0;

static constexpr int CHECKERS_MAX_BOARD_SIZE = 12;
static constexpr int CHECKERS_MAX_HINT_MOVES = 32;
static constexpr int CHECKERS_ACTION_BAR_H = 46;
static constexpr int CHECKERS_CONTROL_MSG_MARKERS = 4;
static const char *const CHECKERS_CONTROL_PREFIXES[CHECKERS_CONTROL_MSG_MARKERS] = {
    "@CHK_INVITE|",
    "@CHK_ACCEPT|",
    "@CHK_MOVE|",
    "@CHK_DECLINE|"
};

enum CheckersMode : uint8_t {
    CHECKERS_MODE_IDLE = 0,
    CHECKERS_MODE_ESP32,
    CHECKERS_MODE_TAG
};

enum CheckersVariant : uint8_t {
    CHECKERS_VARIANT_AMERICAN = 0,
    CHECKERS_VARIANT_INTERNATIONAL,
    CHECKERS_VARIANT_RUSSIAN,
    CHECKERS_VARIANT_POOL,
    CHECKERS_VARIANT_CANADIAN
};

static const char *checkersVariantName(CheckersVariant variant);

struct CheckersMove {
    int8_t fromX;
    int8_t fromY;
    int8_t toX;
    int8_t toY;
    int8_t captureX;
    int8_t captureY;
    bool capture;
};

static constexpr int CHECKERS_MAX_MOVES = 128;
int8_t checkersBoard[CHECKERS_MAX_BOARD_SIZE][CHECKERS_MAX_BOARD_SIZE] = {};
CheckersMode checkersMode = CHECKERS_MODE_IDLE;
CheckersVariant checkersVariant = CHECKERS_VARIANT_AMERICAN;
String checkersSessionId;
String checkersPeerKey;
int8_t checkersLocalSide = 0;
int8_t checkersTurn = 1;
int8_t checkersSelectedX = -1;
int8_t checkersSelectedY = -1;
int8_t checkersForcedX = -1;
int8_t checkersForcedY = -1;
bool checkersStarted = false;
bool checkersWaitingForRemote = false;
bool checkersPeerPopupOpen = false;
bool checkersVariantPopupOpen = true;
bool checkersGameOver = false;
int8_t checkersWinnerSide = 0;
uint16_t checkersLocalWins = 0;
uint16_t checkersRemoteWins = 0;
unsigned long checkersAiDueMs = 0;
