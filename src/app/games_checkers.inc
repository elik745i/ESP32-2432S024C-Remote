static bool checkersInBounds(int x, int y)
{
    const int boardSize = checkersBoardSize();
    return x >= 0 && x < boardSize && y >= 0 && y < boardSize;
}

static int8_t checkersPieceSide(int8_t piece)
{
    if (piece > 0) return 1;
    if (piece < 0) return -1;
    return 0;
}

static bool checkersIsKing(int8_t piece)
{
    return piece == 2 || piece == -2;
}

static bool checkersIsDarkSquare(int x, int y)
{
    return ((x + y) & 1) == 1;
}

static int checkersBoardSize()
{
    switch (checkersVariant) {
        case CHECKERS_VARIANT_INTERNATIONAL:
            return 10;
        case CHECKERS_VARIANT_CANADIAN:
            return 12;
        default:
            return 8;
    }
}

static int checkersStartRows()
{
    switch (checkersVariant) {
        case CHECKERS_VARIANT_INTERNATIONAL:
            return 4;
        case CHECKERS_VARIANT_CANADIAN:
            return 5;
        default:
            return 3;
    }
}

static bool checkersMenCaptureBackward()
{
    return checkersVariant != CHECKERS_VARIANT_AMERICAN;
}

static bool checkersFlyingKings()
{
    return checkersVariant != CHECKERS_VARIANT_AMERICAN;
}

static bool checkersPromotionContinuesCapture()
{
    return checkersVariant == CHECKERS_VARIANT_RUSSIAN;
}

static const char *checkersVariantName(CheckersVariant variant)
{
    switch (variant) {
        case CHECKERS_VARIANT_AMERICAN: return "American Checkers";
        case CHECKERS_VARIANT_INTERNATIONAL: return "International Draughts";
        case CHECKERS_VARIANT_RUSSIAN: return "Russian Checkers";
        case CHECKERS_VARIANT_POOL: return "Pool Checkers";
        case CHECKERS_VARIANT_CANADIAN: return "Canadian/Sri Lankan";
        default: return "Checkers";
    }
}

static void checkersClearSelection()
{
    checkersSelectedX = -1;
    checkersSelectedY = -1;
    checkersForcedX = -1;
    checkersForcedY = -1;
}

static void checkersClearSession()
{
    checkersSessionId = "";
    checkersPeerKey = "";
    checkersLocalSide = 0;
    checkersWaitingForRemote = false;
    checkersAiDueMs = 0;
}

static String checkersPeerDisplayName()
{
    if (checkersPeerKey.isEmpty()) return "peer";
    const String name = chatDisplayNameForPeerKey(checkersPeerKey);
    return name.isEmpty() ? String("peer") : name;
}

static void checkersResetBoard()
{
    const int boardSize = checkersBoardSize();
    const int startRows = checkersStartRows();
    for (int y = 0; y < CHECKERS_MAX_BOARD_SIZE; ++y) {
        for (int x = 0; x < CHECKERS_MAX_BOARD_SIZE; ++x) {
            checkersBoard[y][x] = 0;
            if (x >= boardSize || y >= boardSize) continue;
            if (!checkersIsDarkSquare(x, y)) continue;
            if (y < startRows) checkersBoard[y][x] = -1;
            else if (y >= (boardSize - startRows)) checkersBoard[y][x] = 1;
        }
    }
    checkersTurn = 1;
    checkersStarted = true;
    checkersGameOver = false;
    checkersWinnerSide = 0;
    checkersAiDueMs = 0;
    checkersVariantPopupOpen = false;
    checkersClearSelection();
}

static int checkersCollectPieceMoves(int x, int y, bool captureOnly, CheckersMove *out, int maxOut)
{
    if (!checkersInBounds(x, y)) return 0;
    const int8_t piece = checkersBoard[y][x];
    if (piece == 0) return 0;
    const int8_t side = checkersPieceSide(piece);
    const bool king = checkersIsKing(piece);
    const int dirs[4][2] = {{-1, -1}, {1, -1}, {-1, 1}, {1, 1}};
    int count = 0;

    for (int i = 0; i < 4; ++i) {
        const int dx = dirs[i][0];
        const int dy = dirs[i][1];
        if (!king && !checkersMenCaptureBackward()) {
            if (side == 1 && dy != -1) continue;
            if (side == -1 && dy != 1) continue;
        }

        if (king && checkersFlyingKings()) {
            int step = 1;
            int capX = -1;
            int capY = -1;
            while (true) {
                const int toX = x + (dx * step);
                const int toY = y + (dy * step);
                if (!checkersInBounds(toX, toY)) break;
                const int8_t target = checkersBoard[toY][toX];
                if (!captureOnly) {
                    if (target != 0) break;
                    if (out && count < maxOut) {
                        out[count] = {static_cast<int8_t>(x), static_cast<int8_t>(y), static_cast<int8_t>(toX), static_cast<int8_t>(toY), -1, -1, false};
                    }
                    count++;
                    step++;
                    continue;
                }

                if (target == 0) {
                    if (capX >= 0) {
                        if (out && count < maxOut) {
                            out[count] = {static_cast<int8_t>(x), static_cast<int8_t>(y), static_cast<int8_t>(toX), static_cast<int8_t>(toY), static_cast<int8_t>(capX), static_cast<int8_t>(capY), true};
                        }
                        count++;
                    }
                    step++;
                    continue;
                }

                if (checkersPieceSide(target) == side || capX >= 0) break;
                capX = toX;
                capY = toY;
                step++;
                continue;
            }
            continue;
        }

        if (captureOnly) {
            const int midX = x + dx;
            const int midY = y + dy;
            const int toX = x + (dx * 2);
            const int toY = y + (dy * 2);
            if (!checkersInBounds(midX, midY) || !checkersInBounds(toX, toY)) continue;
            const int8_t midPiece = checkersBoard[midY][midX];
            if (midPiece == 0 || checkersPieceSide(midPiece) == side || checkersBoard[toY][toX] != 0) continue;
            if (out && count < maxOut) {
                out[count] = {static_cast<int8_t>(x), static_cast<int8_t>(y), static_cast<int8_t>(toX), static_cast<int8_t>(toY), static_cast<int8_t>(midX), static_cast<int8_t>(midY), true};
            }
            count++;
        } else {
            if (!king) {
                if (side == 1 && dy != -1) continue;
                if (side == -1 && dy != 1) continue;
            }
            const int toX = x + dx;
            const int toY = y + dy;
            if (!checkersInBounds(toX, toY) || checkersBoard[toY][toX] != 0) continue;
            if (out && count < maxOut) {
                out[count] = {static_cast<int8_t>(x), static_cast<int8_t>(y), static_cast<int8_t>(toX), static_cast<int8_t>(toY), -1, -1, false};
            }
            count++;
        }
    }

    return count;
}

static bool checkersSideHasCapture(int8_t side)
{
    if (checkersForcedX >= 0 && checkersForcedY >= 0) {
        return checkersCollectPieceMoves(checkersForcedX, checkersForcedY, true, nullptr, 0) > 0;
    }

    const int boardSize = checkersBoardSize();
    for (int y = 0; y < boardSize; ++y) {
        for (int x = 0; x < boardSize; ++x) {
            if (checkersPieceSide(checkersBoard[y][x]) != side) continue;
            if (checkersCollectPieceMoves(x, y, true, nullptr, 0) > 0) return true;
        }
    }
    return false;
}

static int checkersCollectLegalMovesForPiece(int x, int y, CheckersMove *out, int maxOut)
{
    if (!checkersInBounds(x, y)) return 0;
    if (checkersPieceSide(checkersBoard[y][x]) != checkersTurn) return 0;
    if (checkersForcedX >= 0 && checkersForcedY >= 0 && (x != checkersForcedX || y != checkersForcedY)) return 0;
    const bool requireCapture = checkersSideHasCapture(checkersTurn);
    return checkersCollectPieceMoves(x, y, requireCapture, out, maxOut);
}

static int checkersCollectLegalMovesForTurn(CheckersMove *out, int maxOut)
{
    int count = 0;
    if (checkersForcedX >= 0 && checkersForcedY >= 0) {
        return checkersCollectLegalMovesForPiece(checkersForcedX, checkersForcedY, out, maxOut);
    }

    const bool requireCapture = checkersSideHasCapture(checkersTurn);
    const int boardSize = checkersBoardSize();
    for (int y = 0; y < boardSize; ++y) {
        for (int x = 0; x < boardSize; ++x) {
            if (checkersPieceSide(checkersBoard[y][x]) != checkersTurn) continue;
            CheckersMove local[CHECKERS_MAX_HINT_MOVES];
            const int localCount = checkersCollectPieceMoves(x, y, requireCapture, local, CHECKERS_MAX_HINT_MOVES);
            for (int i = 0; i < localCount; ++i) {
                if (out && count < maxOut) out[count] = local[i];
                count++;
            }
        }
    }
    return count;
}

static void checkersUpdateHintMoves()
{
    if (checkersSelectedX < 0 || checkersSelectedY < 0) return;
    CheckersMove moves[CHECKERS_MAX_HINT_MOVES];
    const int moveCount = checkersCollectLegalMovesForPiece(checkersSelectedX, checkersSelectedY, moves, CHECKERS_MAX_HINT_MOVES);
    if (moveCount <= 0 && checkersForcedX < 0) {
        checkersSelectedX = -1;
        checkersSelectedY = -1;
    }
}

static String checkersModeLabelText()
{
    const String variant = String(checkersVariantName(checkersVariant));
    const String stats = " W " + String(checkersLocalWins) + ":" + String(checkersRemoteWins);
    switch (checkersMode) {
        case CHECKERS_MODE_ESP32:
            return variant + " / ESP32" + stats;
        case CHECKERS_MODE_TAG:
            if (!checkersPeerKey.isEmpty()) return variant + " / Tag MP: " + checkersPeerDisplayName() + stats;
            return variant + " / Tag Multiplayer" + stats;
        default:
            return variant + stats;
    }
}

static String checkersTurnLabelText()
{
    if (checkersGameOver) {
        if (checkersMode == CHECKERS_MODE_ESP32) return (checkersWinnerSide == 1) ? "You win" : "ESP32 wins";
        if (checkersMode == CHECKERS_MODE_TAG) return (checkersWinnerSide == checkersLocalSide) ? "You win" : (checkersPeerDisplayName() + " wins");
        return (checkersWinnerSide == 1) ? "Red wins" : "Blue wins";
    }
    if (!checkersStarted) return "Pick a mode";
    if (checkersWaitingForRemote) return "Pending";
    if (checkersMode == CHECKERS_MODE_ESP32) {
        return (checkersTurn == 1) ? "Your move" : "ESP32";
    }
    if (checkersMode == CHECKERS_MODE_TAG) {
        if (checkersTurn == checkersLocalSide) return "Your move";
        return checkersPeerDisplayName();
    }
    return (checkersTurn == 1) ? "Red" : "Blue";
}

static void checkersFinishGame(int8_t winnerSide)
{
    checkersGameOver = true;
    checkersWinnerSide = winnerSide;
    checkersAiDueMs = 0;
    checkersWaitingForRemote = false;
    if ((checkersMode == CHECKERS_MODE_ESP32 || checkersMode == CHECKERS_MODE_TAG) && checkersLocalSide != 0) {
        if (winnerSide == checkersLocalSide) {
            checkersLocalWins++;
            saveGameValue("checkers_local_wins", checkersLocalWins);
        } else if (winnerSide == -checkersLocalSide) {
            checkersRemoteWins++;
            saveGameValue("checkers_remote_wins", checkersRemoteWins);
        }
    }
    checkersClearSelection();
}

static void checkersEvaluateGameOver()
{
    CheckersMove legal[CHECKERS_MAX_MOVES];
    if (checkersCollectLegalMovesForTurn(legal, CHECKERS_MAX_MOVES) > 0) return;

    if (checkersMode == CHECKERS_MODE_ESP32) {
        checkersFinishGame(static_cast<int8_t>(-checkersTurn));
    } else if (checkersMode == CHECKERS_MODE_TAG) {
        checkersFinishGame(static_cast<int8_t>(-checkersTurn));
    } else {
        checkersFinishGame(static_cast<int8_t>(-checkersTurn));
    }
}

static bool checkersApplyMove(const CheckersMove &move, bool broadcastMove)
{
    const int boardSize = checkersBoardSize();
    CheckersMove legal[CHECKERS_MAX_MOVES];
    const int legalCount = checkersCollectLegalMovesForPiece(move.fromX, move.fromY, legal, CHECKERS_MAX_MOVES);
    int match = -1;
    for (int i = 0; i < legalCount; ++i) {
        if (legal[i].toX == move.toX && legal[i].toY == move.toY) {
            match = i;
            break;
        }
    }
    if (match < 0) return false;

    const CheckersMove applied = legal[match];
    int8_t piece = checkersBoard[applied.fromY][applied.fromX];
    checkersBoard[applied.fromY][applied.fromX] = 0;
    checkersBoard[applied.toY][applied.toX] = piece;
    if (applied.capture) checkersBoard[applied.captureY][applied.captureX] = 0;
    bool promoted = false;
    if (piece == 1 && applied.toY == 0) {
        piece = 2;
        promoted = true;
    } else if (piece == -1 && applied.toY == (boardSize - 1)) {
        piece = -2;
        promoted = true;
    }
    checkersBoard[applied.toY][applied.toX] = piece;

    if (broadcastMove && checkersMode == CHECKERS_MODE_TAG && !checkersPeerKey.isEmpty() && !checkersSessionId.isEmpty()) {
        chatSendRawReliableMessage(checkersPeerKey, checkersBuildMoveText(checkersSessionId, applied), false);
    }

    const bool allowFollowUpCapture = applied.capture && (!promoted || checkersPromotionContinuesCapture());
    if (allowFollowUpCapture && checkersCollectPieceMoves(applied.toX, applied.toY, true, nullptr, 0) > 0) {
        checkersForcedX = applied.toX;
        checkersForcedY = applied.toY;
        if (checkersTurn == checkersLocalSide) {
            checkersSelectedX = applied.toX;
            checkersSelectedY = applied.toY;
        } else {
            checkersSelectedX = -1;
            checkersSelectedY = -1;
        }
    } else {
        checkersClearSelection();
        checkersTurn = static_cast<int8_t>(-checkersTurn);
        checkersEvaluateGameOver();
    }

    checkersUpdateHintMoves();
    if (!checkersGameOver && checkersMode == CHECKERS_MODE_ESP32 && checkersTurn == -1) {
        checkersAiDueMs = millis() + 360UL;
    } else if (checkersMode != CHECKERS_MODE_ESP32 || checkersTurn != -1) {
        checkersAiDueMs = 0;
    }
    return true;
}

static void checkersStartEsp32Game()
{
    checkersClearSession();
    checkersMode = CHECKERS_MODE_ESP32;
    checkersLocalSide = 1;
    checkersPeerPopupOpen = false;
    checkersResetBoard();
    checkersUpdateHintMoves();
    uiStatusLine = String(checkersVariantName(checkersVariant)) + " vs ESP32";
    if (lvglReady) lvglSyncStatusLine();
}

static void checkersStartPendingInvite(const String &peerKey)
{
    if (peerKey.isEmpty()) return;
    checkersMode = CHECKERS_MODE_TAG;
    checkersPeerKey = peerKey;
    checkersSessionId = chatGenerateMessageId();
    checkersLocalSide = 1;
    checkersWaitingForRemote = true;
    checkersPeerPopupOpen = false;
    checkersResetBoard();
    checkersUpdateHintMoves();
    chatSendRawReliableMessage(peerKey, checkersBuildInviteText(checkersSessionId, deviceShortNameValue()), true);
    uiStatusLine = String(checkersVariantName(checkersVariant)) + " invite sent to " + checkersPeerDisplayName();
    if (lvglReady) lvglSyncStatusLine();
}

static void checkersStartAcceptedInvite(const String &peerKey, const String &sessionId, CheckersVariant variant)
{
    if (peerKey.isEmpty() || sessionId.isEmpty()) return;
    checkersVariant = variant;
    checkersMode = CHECKERS_MODE_TAG;
    checkersPeerKey = peerKey;
    checkersSessionId = sessionId;
    checkersLocalSide = -1;
    checkersWaitingForRemote = false;
    checkersPeerPopupOpen = false;
    checkersResetBoard();
    checkersUpdateHintMoves();
    chatSendRawReliableMessage(peerKey, checkersBuildAcceptText(sessionId), false);
    uiStatusLine = String(checkersVariantName(checkersVariant)) + " started with " + checkersPeerDisplayName();
    if (lvglReady) lvglSyncStatusLine();
}

static void checkersOpenPeerPopup()
{
    checkersPeerPopupOpen = true;
    if (lvglReady && uiScreen == UI_GAME_CHECKERS) lvglRefreshCheckersBoard();
}

static void checkersClosePeerPopup()
{
    checkersPeerPopupOpen = false;
    if (lvglCheckersPeerPopup) lv_obj_add_flag(lvglCheckersPeerPopup, LV_OBJ_FLAG_HIDDEN);
}

static bool checkersHandleIncomingChatPayload(const String &peerKey,
                                              const String &author,
                                              const String &text,
                                              ChatTransport transport,
                                              const String &messageId)
{
    String sessionId;
    if (checkersParseInviteText(text, sessionId)) {
        if (messageId.isEmpty() || !chatHasLoggedMessageId(peerKey, messageId)) {
            chatStoreMessage(peerKey, author, text, false, transport, messageId);
        }
        uiStatusLine = "Checkers invite from " + author;
        return true;
    }

    if (checkersParseAcceptText(text, sessionId)) {
        if (gameControlMessageSeen(messageId)) return true;
        gameRememberControlMessageId(messageId);
        if (checkersMode == CHECKERS_MODE_TAG &&
            checkersWaitingForRemote &&
            checkersSessionId == sessionId &&
            checkersPeerKey == peerKey) {
            checkersWaitingForRemote = false;
            uiStatusLine = author + " joined Checkers";
            if (lvglReady && uiScreen == UI_GAME_CHECKERS) lvglRefreshCheckersBoard();
        }
        return true;
    }

    if (checkersParseDeclineText(text, sessionId)) {
        if (gameControlMessageSeen(messageId)) return true;
        gameRememberControlMessageId(messageId);
        if (checkersMode == CHECKERS_MODE_TAG &&
            checkersWaitingForRemote &&
            checkersSessionId == sessionId &&
            checkersPeerKey == peerKey) {
            checkersWaitingForRemote = false;
            checkersStarted = false;
            checkersMode = CHECKERS_MODE_IDLE;
            checkersClearSession();
            uiStatusLine = author + " declined Checkers";
            if (lvglReady && uiScreen == UI_GAME_CHECKERS) lvglRefreshCheckersBoard();
        }
        return true;
    }

    CheckersMove remoteMove = {};
    if (checkersParseMoveText(text, sessionId, remoteMove)) {
        if (gameControlMessageSeen(messageId)) return true;
        gameRememberControlMessageId(messageId);
        if (checkersMode == CHECKERS_MODE_TAG &&
            !checkersWaitingForRemote &&
            checkersSessionId == sessionId &&
            checkersPeerKey == peerKey &&
            checkersTurn != checkersLocalSide) {
            if (checkersApplyMove(remoteMove, false)) {
                uiStatusLine = "Checkers move from " + author;
                if (lvglReady && uiScreen == UI_GAME_CHECKERS) lvglRefreshCheckersBoard();
            }
        }
        return true;
    }

    return false;
}

static bool checkersLocalTurnActive()
{
    if (!checkersStarted || checkersGameOver || checkersWaitingForRemote || checkersVariantPopupOpen) return false;
    if (checkersMode == CHECKERS_MODE_ESP32) return checkersTurn == 1;
    if (checkersMode == CHECKERS_MODE_TAG) return checkersTurn == checkersLocalSide;
    return false;
}

static void checkersHandleBoardTap(int cellX, int cellY)
{
    if (!checkersLocalTurnActive()) return;
    if (!checkersInBounds(cellX, cellY) || !checkersIsDarkSquare(cellX, cellY)) return;

    if (checkersSelectedX >= 0 && checkersSelectedY >= 0) {
        CheckersMove hintMoves[CHECKERS_MAX_HINT_MOVES];
        const int hintMoveCount = checkersCollectLegalMovesForPiece(checkersSelectedX, checkersSelectedY, hintMoves, CHECKERS_MAX_HINT_MOVES);
        CheckersMove chosen = {};
        bool found = false;
        for (int i = 0; i < hintMoveCount; ++i) {
            if (hintMoves[i].toX == cellX && hintMoves[i].toY == cellY) {
                chosen = hintMoves[i];
                found = true;
                break;
            }
        }
        if (found) {
            if (checkersApplyMove(chosen, checkersMode == CHECKERS_MODE_TAG)) {
                lvglRefreshCheckersBoard();
            }
            return;
        }
    }

    if (checkersPieceSide(checkersBoard[cellY][cellX]) == checkersTurn) {
        checkersSelectedX = static_cast<int8_t>(cellX);
        checkersSelectedY = static_cast<int8_t>(cellY);
        checkersUpdateHintMoves();
        CheckersMove hintMoves[CHECKERS_MAX_HINT_MOVES];
        const int hintMoveCount = checkersCollectLegalMovesForPiece(checkersSelectedX, checkersSelectedY, hintMoves, CHECKERS_MAX_HINT_MOVES);
        if (hintMoveCount <= 0) {
            if (checkersForcedX < 0) checkersSelectedX = -1;
            if (checkersForcedY < 0) checkersSelectedY = -1;
        }
        lvglRefreshCheckersBoard();
        return;
    }

    if (checkersForcedX < 0) {
        checkersSelectedX = -1;
        checkersSelectedY = -1;
        lvglRefreshCheckersBoard();
    }
}

void lvglCheckersBoardEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    if (lvglClickSuppressed()) return;
    lv_obj_t *obj = lv_event_get_target(e);
    if (!obj) return;
    lv_indev_t *indev = lv_indev_get_act();
    if (!indev) return;
    lv_point_t pt = {0, 0};
    lv_indev_get_point(indev, &pt);
    lv_area_t coords;
    lv_obj_get_coords(obj, &coords);
    const int boardSize = checkersBoardSize();
    const int width = static_cast<int>(coords.x2 - coords.x1 + 1);
    const int height = static_cast<int>(coords.y2 - coords.y1 + 1);
    const int cellW = width / boardSize;
    const int cellH = height / boardSize;
    if (cellW <= 0 || cellH <= 0) return;
    const int localX = pt.x - coords.x1;
    const int localY = pt.y - coords.y1;
    const int cellX = localX / cellW;
    const int cellY = localY / cellH;
    checkersHandleBoardTap(cellX, cellY);
}

static void checkersDrawPiece(lv_draw_ctx_t *drawCtx, int x, int y, int w, int h, lv_color_t fill, bool king, bool selected)
{
    lv_draw_rect_dsc_t dsc;
    lv_draw_rect_dsc_init(&dsc);
    dsc.bg_opa = LV_OPA_COVER;
    dsc.bg_color = fill;
    dsc.radius = LV_RADIUS_CIRCLE;
    dsc.border_width = selected ? 3 : 2;
    dsc.border_color = selected ? lv_color_hex(0xFFF1A8) : lv_color_hex(0xD6DEE8);
    const int inset = max(2, min(w, h) / 8);
    lv_area_t area = {
        static_cast<lv_coord_t>(x + inset),
        static_cast<lv_coord_t>(y + inset),
        static_cast<lv_coord_t>(x + w - inset),
        static_cast<lv_coord_t>(y + h - inset)
    };
    lv_draw_rect(drawCtx, &dsc, &area);

    if (!king) return;
    lv_draw_rect_dsc_t crownDsc;
    lv_draw_rect_dsc_init(&crownDsc);
    crownDsc.bg_opa = LV_OPA_TRANSP;
    crownDsc.radius = LV_RADIUS_CIRCLE;
    crownDsc.border_width = 2;
    crownDsc.border_color = lv_color_hex(0xFFE08A);
    const int crownInset = inset + max(2, min(w, h) / 7);
    lv_area_t crownArea = {
        static_cast<lv_coord_t>(x + crownInset),
        static_cast<lv_coord_t>(y + crownInset),
        static_cast<lv_coord_t>(x + w - crownInset),
        static_cast<lv_coord_t>(y + h - crownInset)
    };
    lv_draw_rect(drawCtx, &crownDsc, &crownArea);
}

void lvglCheckersBoardDrawEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_DRAW_MAIN) return;
    lv_obj_t *obj = lv_event_get_target(e);
    lv_draw_ctx_t *drawCtx = lv_event_get_draw_ctx(e);
    if (!obj || !drawCtx) return;

    lv_area_t coords;
    lv_obj_get_coords(obj, &coords);
    const int boardSize = checkersBoardSize();
    const int width = static_cast<int>(coords.x2 - coords.x1 + 1);
    const int height = static_cast<int>(coords.y2 - coords.y1 + 1);
    const int cellW = width / boardSize;
    const int cellH = height / boardSize;
    if (cellW < 5 || cellH < 5) return;
    CheckersMove hintMoves[CHECKERS_MAX_HINT_MOVES];
    const int hintMoveCount = (checkersSelectedX >= 0 && checkersSelectedY >= 0)
                                  ? checkersCollectLegalMovesForPiece(checkersSelectedX, checkersSelectedY, hintMoves, CHECKERS_MAX_HINT_MOVES)
                                  : 0;

    for (int y = 0; y < boardSize; ++y) {
        for (int x = 0; x < boardSize; ++x) {
            const int px = static_cast<int>(coords.x1) + (x * cellW);
            const int py = static_cast<int>(coords.y1) + (y * cellH);
            lv_color_t square = checkersIsDarkSquare(x, y) ? lv_color_hex(0x314355) : lv_color_hex(0xD5C7AF);
            if (x == checkersSelectedX && y == checkersSelectedY) square = lv_color_hex(0x5B7244);
            for (int i = 0; i < hintMoveCount; ++i) {
                if (hintMoves[i].toX == x && hintMoves[i].toY == y) {
                    square = hintMoves[i].capture ? lv_color_hex(0x7D4A3B) : lv_color_hex(0x4B657A);
                    break;
                }
            }
            lvglDrawGridCell(drawCtx, px, py, cellW - 1, cellH - 1, square);

            const int8_t piece = checkersBoard[y][x];
            if (piece == 0) continue;
            checkersDrawPiece(drawCtx,
                              px + 1,
                              py + 1,
                              cellW - 2,
                              cellH - 2,
                              piece > 0 ? lv_color_hex(0xD94A4A) : lv_color_hex(0x2D3F8E),
                              checkersIsKing(piece),
                              x == checkersSelectedX && y == checkersSelectedY);
        }
    }
}

static void checkersRefreshPeerPopup()
{
    if (!lvglCheckersPeerPopup || !lvglCheckersPeerPopupList) return;
    if (!checkersPeerPopupOpen) {
        lv_obj_add_flag(lvglCheckersPeerPopup, LV_OBJ_FLAG_HIDDEN);
        return;
    }

    lv_obj_clear_flag(lvglCheckersPeerPopup, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_foreground(lvglCheckersPeerPopup);
    lv_obj_clean(lvglCheckersPeerPopupList);

    if (lvglCheckersPeerPopupTitle) lv_label_set_text(lvglCheckersPeerPopupTitle, "Choose Contact");

    bool havePeers = false;
    for (int i = 0; i < p2pPeerCount; ++i) {
        if (!p2pPeers[i].enabled) continue;
        havePeers = true;
        lv_obj_t *btn = lv_btn_create(lvglCheckersPeerPopupList);
        lv_obj_set_width(btn, lv_pct(100));
        lv_obj_set_height(btn, 34);
        lv_obj_set_style_radius(btn, 10, 0);
        lv_obj_set_style_border_width(btn, 0, 0);
        lv_obj_add_event_cb(btn, lvglClickFilterEvent, LV_EVENT_CLICKED, nullptr);
        lv_obj_add_event_cb(btn, lvglFilteredClickFlashEvent, LV_EVENT_CLICKED, nullptr);
        lv_obj_add_event_cb(btn, lvglCheckersPeerSelectEvent, LV_EVENT_CLICKED, reinterpret_cast<void *>(static_cast<intptr_t>(i)));
        lv_obj_t *lbl = lv_label_create(btn);
        lv_label_set_text(lbl, p2pPeers[i].name.isEmpty() ? "Peer" : p2pPeers[i].name.c_str());
        lv_obj_center(lbl);
        lvglRegisterStyledButton(btn, lv_color_hex(0x2F6D86), true);
        const String orderKey = lvglOrderTokenFromText("p", p2pPeers[i].pubKeyHex);
        lvglRegisterReorderableItem(btn, "ord_ckp", orderKey.c_str());
    }

    if (!havePeers) {
        lv_obj_t *lbl = lv_label_create(lvglCheckersPeerPopupList);
        lv_obj_set_width(lbl, lv_pct(100));
        lv_obj_set_style_text_align(lbl, LV_TEXT_ALIGN_CENTER, 0);
        lv_label_set_long_mode(lbl, LV_LABEL_LONG_WRAP);
        lv_label_set_text(lbl, "No paired contacts.\nOpen Chat Peers first and pair another device.");
        lvglRegisterReorderableItem(lbl, "ord_ckp", "empty");
    }
    lvglApplySavedOrder(lvglCheckersPeerPopupList, "ord_ckp");
}

static void checkersRefreshVariantPopup()
{
    if (!lvglCheckersVariantPopup || !lvglCheckersVariantPopupList) return;
    if (!checkersVariantPopupOpen) {
        lv_obj_add_flag(lvglCheckersVariantPopup, LV_OBJ_FLAG_HIDDEN);
        return;
    }

    lv_obj_clear_flag(lvglCheckersVariantPopup, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_foreground(lvglCheckersVariantPopup);
    lv_obj_clean(lvglCheckersVariantPopupList);

    if (lvglCheckersVariantPopupTitle) lv_label_set_text(lvglCheckersVariantPopupTitle, "Choose Rule Set");

    static const CheckersVariant variants[] = {
        CHECKERS_VARIANT_AMERICAN,
        CHECKERS_VARIANT_INTERNATIONAL,
        CHECKERS_VARIANT_RUSSIAN,
        CHECKERS_VARIANT_POOL,
        CHECKERS_VARIANT_CANADIAN
    };

    for (size_t i = 0; i < (sizeof(variants) / sizeof(variants[0])); ++i) {
        lv_obj_t *btn = lv_btn_create(lvglCheckersVariantPopupList);
        if (!btn) continue;
        lv_obj_set_width(btn, lv_pct(100));
        lv_obj_set_height(btn, 28);
        lv_obj_set_style_radius(btn, 10, 0);
        lv_obj_set_style_border_width(btn, 0, 0);
        lv_obj_add_event_cb(btn, lvglClickFilterEvent, LV_EVENT_CLICKED, nullptr);
        lv_obj_add_event_cb(btn, lvglFilteredClickFlashEvent, LV_EVENT_CLICKED, nullptr);
        lv_obj_add_event_cb(btn,
                            lvglCheckersVariantSelectEvent,
                            LV_EVENT_CLICKED,
                            reinterpret_cast<void *>(static_cast<intptr_t>(variants[i])));
        lv_obj_t *lbl = lv_label_create(btn);
        if (!lbl) continue;
        lv_label_set_text(lbl, checkersVariantName(variants[i]));
        lv_obj_center(lbl);
        lvglRegisterPersistentStateButtonColors(btn, lv_color_hex(0x2F4658), lv_color_hex(0x3A7A3A), variants[i] == checkersVariant, true);
        char orderKey[16];
        snprintf(orderKey, sizeof(orderKey), "v%d", static_cast<int>(variants[i]));
        lvglRegisterReorderableItem(btn, "ord_ckv", orderKey);
    }
    lvglApplySavedOrder(lvglCheckersVariantPopupList, "ord_ckv");
}

void lvglRefreshCheckersBoard()
{
    checkersUpdateHintMoves();
    if (lvglCheckersBoardObj) lv_obj_invalidate(lvglCheckersBoardObj);
    if (lvglCheckersModeLabel) lv_label_set_text(lvglCheckersModeLabel, checkersModeLabelText().c_str());
    if (lvglCheckersTurnLabel) lv_label_set_text(lvglCheckersTurnLabel, checkersTurnLabelText().c_str());

    if (lvglCheckersOverlay) {
        bool showOverlay = false;
        String title = checkersVariantName(checkersVariant);
        String msg = "";
        bool showBtn = false;
        String btnText = "Replay";

        if (checkersGameOver) {
            showOverlay = true;
            title = checkersTurnLabelText();
            msg = (checkersMode == CHECKERS_MODE_TAG && !checkersPeerKey.isEmpty())
                      ? ("Match finished with " + checkersPeerDisplayName())
                      : "Tap Replay to start again.";
            showBtn = true;
        } else if (checkersWaitingForRemote) {
            showOverlay = true;
            title = "Tag Multiplayer";
            msg = "Invitation sent to " + checkersPeerDisplayName() + "\nWaiting for Play...";
        } else if (!checkersStarted) {
            showOverlay = true;
            title = checkersVariantPopupOpen ? "Select Checkers Type" : String(checkersVariantName(checkersVariant));
            msg = checkersVariantPopupOpen
                      ? "Pick a rule set, then start ESP32 or Tag MP."
                      : ("Tap ESP32 for solo or Tag MP to invite a contact.\nCurrent: " + String(checkersVariantName(checkersVariant)));
        }

        if (showOverlay) lv_obj_clear_flag(lvglCheckersOverlay, LV_OBJ_FLAG_HIDDEN);
        else lv_obj_add_flag(lvglCheckersOverlay, LV_OBJ_FLAG_HIDDEN);
        if (lvglCheckersOverlayTitle) lv_label_set_text(lvglCheckersOverlayTitle, title.c_str());
        if (lvglCheckersOverlaySubLabel) lv_label_set_text(lvglCheckersOverlaySubLabel, msg.c_str());
        if (lvglCheckersOverlayBtn) {
            if (showBtn) lv_obj_clear_flag(lvglCheckersOverlayBtn, LV_OBJ_FLAG_HIDDEN);
            else lv_obj_add_flag(lvglCheckersOverlayBtn, LV_OBJ_FLAG_HIDDEN);
        }
        if (lvglCheckersOverlayBtnLabel) lv_label_set_text(lvglCheckersOverlayBtnLabel, btnText.c_str());
    }

    checkersRefreshPeerPopup();
    checkersRefreshVariantPopup();
}

void lvglCheckersEsp32Event(lv_event_t *e)
{
    (void)e;
    if (checkersVariantPopupOpen) return;
    checkersStartEsp32Game();
    lvglRefreshCheckersBoard();
}

void lvglCheckersTagEvent(lv_event_t *e)
{
    (void)e;
    if (checkersVariantPopupOpen) return;
    if (checkersPeerPopupOpen) checkersClosePeerPopup();
    else checkersOpenPeerPopup();
    lvglRefreshCheckersBoard();
}

void lvglCheckersBackEvent(lv_event_t *e)
{
    (void)e;
    checkersClosePeerPopup();
    lvglOpenScreen(UI_GAMES, LV_SCR_LOAD_ANIM_MOVE_RIGHT);
}

void lvglCheckersReplayEvent(lv_event_t *e)
{
    (void)e;
    if (checkersMode == CHECKERS_MODE_ESP32) {
        checkersStartEsp32Game();
    } else if (checkersMode == CHECKERS_MODE_TAG && !checkersPeerKey.isEmpty()) {
        checkersStartPendingInvite(checkersPeerKey);
    } else {
        checkersStarted = false;
        checkersGameOver = false;
    }
    lvglRefreshCheckersBoard();
}

void lvglCheckersPeerSelectEvent(lv_event_t *e)
{
    const int idx = static_cast<int>(reinterpret_cast<intptr_t>(lv_event_get_user_data(e)));
    if (idx < 0 || idx >= p2pPeerCount || !p2pPeers[idx].enabled) return;
    checkersStartPendingInvite(p2pPeers[idx].pubKeyHex);
    lvglRefreshCheckersBoard();
}

void lvglCheckersVariantSelectEvent(lv_event_t *e)
{
    const int idx = static_cast<int>(reinterpret_cast<intptr_t>(lv_event_get_user_data(e)));
    if (idx < static_cast<int>(CHECKERS_VARIANT_AMERICAN) || idx > static_cast<int>(CHECKERS_VARIANT_CANADIAN)) return;
    checkersVariant = static_cast<CheckersVariant>(idx);
    checkersVariantPopupOpen = false;
    checkersPeerPopupOpen = false;
    checkersStarted = false;
    checkersGameOver = false;
    checkersWinnerSide = 0;
    checkersMode = CHECKERS_MODE_IDLE;
    checkersClearSelection();
    checkersClearSession();
    lvglRefreshCheckersBoard();
}

void lvglCheckersInvitePlayEvent(lv_event_t *e)
{
    const int idx = static_cast<int>(reinterpret_cast<intptr_t>(lv_event_get_user_data(e)));
    if (idx < 0 || idx >= chatMessageCount) return;
    String sessionId;
    CheckersVariant inviteVariant = CHECKERS_VARIANT_AMERICAN;
    if (!checkersParseInviteText(chatMessages[idx].text, sessionId, &inviteVariant)) return;
    const String peerKey = currentChatPeerKey;
    if (peerKey.isEmpty()) return;
    lvglEnsureScreenBuilt(UI_GAME_CHECKERS);
    checkersStartAcceptedInvite(peerKey, sessionId, inviteVariant);
    if (screensaverActive) screensaverSetActive(false);
    if (!displayAwake) displaySetAwake(true);
    lastUserActivityMs = millis();
    lvglRefreshCheckersBoard();
    lvglOpenScreen(UI_GAME_CHECKERS, LV_SCR_LOAD_ANIM_MOVE_LEFT);
}

void lvglOpenCheckersEvent(lv_event_t *e)
{
    (void)e;
    lvglEnsureScreenBuilt(UI_GAME_CHECKERS);
    checkersMode = CHECKERS_MODE_IDLE;
    checkersPeerPopupOpen = false;
    checkersVariantPopupOpen = true;
    checkersStarted = false;
    checkersGameOver = false;
    checkersWinnerSide = 0;
    checkersClearSession();
    checkersClearSelection();
    if (screensaverActive) screensaverSetActive(false);
    if (!displayAwake) displaySetAwake(true);
    lastUserActivityMs = millis();
    lvglRefreshCheckersBoard();
    lvglOpenScreen(UI_GAME_CHECKERS, LV_SCR_LOAD_ANIM_MOVE_LEFT);
}

static bool checkersChooseAiMove(CheckersMove &bestMove)
{
    const int boardSize = checkersBoardSize();
    CheckersMove legal[CHECKERS_MAX_MOVES];
    const int legalCount = checkersCollectLegalMovesForTurn(legal, CHECKERS_MAX_MOVES);
    if (legalCount <= 0) return false;

    int bestScore = -100000;
    int bestIdx = 0;
    for (int i = 0; i < legalCount; ++i) {
        int score = legal[i].capture ? 100 : 10;
        const int8_t piece = checkersBoard[legal[i].fromY][legal[i].fromX];
        if (piece == -1 && legal[i].toY == (boardSize - 1)) score += 25;
        if (checkersIsKing(piece)) score += 12;
        score += static_cast<int>(esp_random() & 0x0F);
        if (score > bestScore) {
            bestScore = score;
            bestIdx = i;
        }
    }
    bestMove = legal[bestIdx];
    return true;
}

void checkersTick()
{
    if (uiScreen != UI_GAME_CHECKERS) return;
    if (checkersMode != CHECKERS_MODE_ESP32 || !checkersStarted || checkersGameOver || checkersWaitingForRemote) return;
    if (checkersTurn != -1) return;
    if (millis() < checkersAiDueMs) return;

    CheckersMove aiMove = {};
    if (!checkersChooseAiMove(aiMove)) {
        checkersFinishGame(1);
        lvglRefreshCheckersBoard();
        return;
    }

    checkersApplyMove(aiMove, false);
    if (!checkersGameOver && checkersTurn == -1) checkersAiDueMs = millis() + 260UL;
    lvglRefreshCheckersBoard();
}

bool mediaAllowedExt(const String &name)
{
    String n = name;
    n.toLowerCase();
    return n.endsWith(".mp3") || n.endsWith(".wav") || n.endsWith(".flac") || n.endsWith(".aac") ||
           n.endsWith(".m4a") || n.endsWith(".raw") ||
           n.endsWith(".mpga") || n.endsWith(".mpeg") || n.endsWith(".wave") || n.endsWith(".adts") ||
           n.endsWith(".m4b") || n.endsWith(".f4a");
}

const char *mediaBaseNamePtr(const char *path)
{
    if (!path) return "";
    const char *slash = strrchr(path, '/');
    if (!slash) return path;
    return slash + 1;
}

bool mediaAllowedExtC(const char *name)
{
    if (!name || !name[0]) return false;
    const char *dot = strrchr(name, '.');
    if (!dot || !dot[1]) return false;

    char ext[8] = {0};
    int i = 0;
    for (const char *p = dot; *p && i < static_cast<int>(sizeof(ext) - 1); ++p, ++i) {
        char c = *p;
        if (c >= 'A' && c <= 'Z') c = static_cast<char>(c - 'A' + 'a');
        ext[i] = c;
    }
    ext[i] = '\0';

    return strcmp(ext, ".mp3") == 0 || strcmp(ext, ".wav") == 0 || strcmp(ext, ".flac") == 0 ||
           strcmp(ext, ".aac") == 0 || strcmp(ext, ".m4a") == 0 || strcmp(ext, ".raw") == 0 ||
           strcmp(ext, ".mpga") == 0 || strcmp(ext, ".mpeg") == 0 || strcmp(ext, ".wave") == 0 ||
           strcmp(ext, ".adts") == 0 || strcmp(ext, ".m4b") == 0 || strcmp(ext, ".f4a") == 0;
}

String mediaBuildFullPath(const String &dir, const char *rawName)
{
    if (!rawName || !rawName[0]) return "";
    if (rawName[0] == '/') return String(rawName);

    String raw(rawName);
    if (dir == "/") return "/" + raw;

    String dirNoLead = dir;
    if (dirNoLead.startsWith("/")) dirNoLead.remove(0, 1);
    if (!dirNoLead.isEmpty() && (raw == dirNoLead || raw.startsWith(dirNoLead + "/"))) {
        return "/" + raw;
    }
    return dir + "/" + raw;
}

String mediaBuildListLabel(const String &name, bool isDir, size_t sizeBytes)
{
    String base = name;
    static constexpr int MEDIA_LABEL_MAX_CHARS = 34;
    if (base.length() > MEDIA_LABEL_MAX_CHARS) {
        base = base.substring(0, MEDIA_LABEL_MAX_CHARS - 3) + "...";
    }
    if (isDir) return "[DIR] " + base;

    const uint32_t kb = static_cast<uint32_t>((sizeBytes + 1023U) / 1024U);
    if (kb >= 10240U) return base + " (" + String(static_cast<unsigned long>(kb / 1024U)) + "MB)";
    return base + " (" + String(static_cast<unsigned long>(kb)) + "KB)";
}

