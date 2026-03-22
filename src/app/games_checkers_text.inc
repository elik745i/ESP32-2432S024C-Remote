static bool checkersRawTextStartsWith(const String &text, const char *prefix)
{
    return prefix && text.startsWith(prefix);
}

static bool checkersRawTextHasControlPrefix(const String &text)
{
    for (int i = 0; i < CHECKERS_CONTROL_MSG_MARKERS; ++i) {
        if (checkersRawTextStartsWith(text, CHECKERS_CONTROL_PREFIXES[i])) return true;
    }
    return false;
}

static String checkersVisibleBodyFromRawText(const String &text)
{
    if (!checkersRawTextHasControlPrefix(text)) return text;
    const int nl = text.indexOf('\n');
    if (nl < 0 || nl >= (text.length() - 1)) return "";
    return text.substring(nl + 1);
}

static bool checkersParseInviteText(const String &text, String &sessionId, CheckersVariant *variantOut = nullptr)
{
    if (!checkersRawTextStartsWith(text, "@CHK_INVITE|")) return false;
    const int lineEnd = text.indexOf('\n');
    const String marker = (lineEnd >= 0) ? text.substring(0, lineEnd) : text;
    const int firstSep = marker.indexOf('|');
    const int secondSep = (firstSep >= 0) ? marker.indexOf('|', firstSep + 1) : -1;
    if (firstSep < 0 || firstSep >= (marker.length() - 1)) return false;
    sessionId = (secondSep > firstSep) ? marker.substring(firstSep + 1, secondSep) : marker.substring(firstSep + 1);
    sessionId.trim();
    if (variantOut) {
        int rawVariant = static_cast<int>(CHECKERS_VARIANT_AMERICAN);
        if (secondSep > firstSep && secondSep < (marker.length() - 1)) rawVariant = marker.substring(secondSep + 1).toInt();
        if (rawVariant < static_cast<int>(CHECKERS_VARIANT_AMERICAN) || rawVariant > static_cast<int>(CHECKERS_VARIANT_CANADIAN)) {
            rawVariant = static_cast<int>(CHECKERS_VARIANT_AMERICAN);
        }
        *variantOut = static_cast<CheckersVariant>(rawVariant);
    }
    return !sessionId.isEmpty();
}

static bool checkersParseAcceptText(const String &text, String &sessionId)
{
    if (!checkersRawTextStartsWith(text, "@CHK_ACCEPT|")) return false;
    sessionId = text.substring(String("@CHK_ACCEPT|").length());
    sessionId.trim();
    return !sessionId.isEmpty();
}

static bool checkersParseDeclineText(const String &text, String &sessionId)
{
    if (!checkersRawTextStartsWith(text, "@CHK_DECLINE|")) return false;
    sessionId = text.substring(String("@CHK_DECLINE|").length());
    sessionId.trim();
    return !sessionId.isEmpty();
}

static bool checkersParseMoveText(const String &text, String &sessionId, CheckersMove &move)
{
    if (!checkersRawTextStartsWith(text, "@CHK_MOVE|")) return false;
    int cursor = String("@CHK_MOVE|").length();
    int nextSep = text.indexOf('|', cursor);
    if (nextSep <= cursor) return false;
    sessionId = text.substring(cursor, nextSep);
    sessionId.trim();
    if (sessionId.isEmpty()) return false;

    int values[4] = {0, 0, 0, 0};
    for (int i = 0; i < 4; ++i) {
        cursor = nextSep + 1;
        nextSep = (i == 3) ? -1 : text.indexOf('|', cursor);
        const String token = (nextSep >= 0) ? text.substring(cursor, nextSep) : text.substring(cursor);
        if (token.isEmpty()) return false;
        values[i] = token.toInt();
    }

    move.fromX = static_cast<int8_t>(values[0]);
    move.fromY = static_cast<int8_t>(values[1]);
    move.toX = static_cast<int8_t>(values[2]);
    move.toY = static_cast<int8_t>(values[3]);
    move.capture = false;
    move.captureX = -1;
    move.captureY = -1;
    return true;
}

static String checkersBuildInviteText(const String &sessionId, const String &name)
{
    return "@CHK_INVITE|" + sessionId + "|" + String(static_cast<int>(checkersVariant)) +
           "\nCheckers invite from " + name + "\n" + checkersVariantName(checkersVariant) + "\nTap Play to join.";
}

static String checkersBuildAcceptText(const String &sessionId)
{
    return "@CHK_ACCEPT|" + sessionId;
}

// Legacy unused checkers helper moved to app/garbage.inc:
// checkersBuildDeclineText(const String&)

static String checkersBuildMoveText(const String &sessionId, const CheckersMove &move)
{
    return "@CHK_MOVE|" + sessionId + "|" +
           String(move.fromX) + "|" +
           String(move.fromY) + "|" +
           String(move.toX) + "|" +
           String(move.toY);
}
