struct LvglStyledButtonEntry {
    lv_obj_t *obj;
    lv_color_t baseColor;
    uint8_t flags;
};

static constexpr uint8_t LVGL_STYLED_BUTTON_FLAG_COMPACT = 0x01;
static constexpr size_t LVGL_STYLED_BUTTON_CAPACITY = 256;
static LvglStyledButtonEntry *lvglStyledButtons = nullptr;
static constexpr size_t LVGL_REORDER_ITEM_CAPACITY = 192;
static constexpr unsigned long UI_REORDER_HOLD_MS = 2000UL;
static constexpr lv_coord_t UI_REORDER_AUTO_SCROLL_MARGIN = 26;
static constexpr lv_coord_t UI_REORDER_AUTO_SCROLL_STEP = 10;
static constexpr lv_coord_t UI_REORDER_SWAP_HYSTERESIS = 10;

struct LvglReorderItemEntry {
    lv_obj_t *obj;
    lv_obj_t *parent;
    char prefKey[16];
    char itemKey[16];
};

struct UiReorderDragState {
    lv_obj_t *obj;
    lv_obj_t *parent;
    unsigned long pressedAtMs;
    lv_point_t pressPoint;
    lv_coord_t fingerOffsetY;
    lv_coord_t originalWidth;
    lv_coord_t originalHeight;
    bool active;
    bool cancelled;
    bool parentOverflowTemporarilyEnabled;
    bool parentScrollTemporarilyDisabled;
};

static LvglReorderItemEntry *lvglReorderItems = nullptr;
static UiReorderDragState lvglReorderDrag = {};

static bool lvglEnsureUiRegistryStorage()
{
    if (!lvglStyledButtons) {
        lvglStyledButtons = static_cast<LvglStyledButtonEntry *>(allocPreferPsram(sizeof(LvglStyledButtonEntry) * LVGL_STYLED_BUTTON_CAPACITY));
        if (!lvglStyledButtons) return false;
        memset(lvglStyledButtons, 0, sizeof(LvglStyledButtonEntry) * LVGL_STYLED_BUTTON_CAPACITY);
    }
    if (!lvglReorderItems) {
        lvglReorderItems = static_cast<LvglReorderItemEntry *>(allocPreferPsram(sizeof(LvglReorderItemEntry) * LVGL_REORDER_ITEM_CAPACITY));
        if (!lvglReorderItems) return false;
        memset(lvglReorderItems, 0, sizeof(LvglReorderItemEntry) * LVGL_REORDER_ITEM_CAPACITY);
    }
    return true;
}

static void lvglApplyReorderDragVisual(lv_obj_t *obj, bool active)
{
    if (!obj || !lv_obj_is_valid(obj)) return;

    const lv_coord_t outlineWidth = active ? 2 : 0;
    const lv_coord_t shadowWidth = active ? 26 : 0;
    const lv_coord_t shadowOfs = active ? 7 : 0;
    const lv_opa_t shadowOpa = active ? LV_OPA_70 : LV_OPA_TRANSP;

    lv_obj_set_style_transform_zoom(obj, 256, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_transform_zoom(obj, 256, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_transform_width(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_transform_width(obj, 0, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_transform_height(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_transform_height(obj, 0, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_outline_width(obj, outlineWidth, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_outline_width(obj, outlineWidth, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_outline_pad(obj, active ? 1 : 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_outline_pad(obj, active ? 1 : 0, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_outline_color(obj, lv_color_hex(0xE7F2FF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_outline_color(obj, lv_color_hex(0xE7F2FF), LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_outline_opa(obj, active ? LV_OPA_60 : LV_OPA_TRANSP, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_outline_opa(obj, active ? LV_OPA_60 : LV_OPA_TRANSP, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_shadow_width(obj, shadowWidth, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(obj, shadowWidth, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_shadow_ofs_x(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_ofs_x(obj, 0, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_shadow_ofs_y(obj, shadowOfs, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_ofs_y(obj, shadowOfs, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_shadow_color(obj, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_color(obj, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_set_style_shadow_opa(obj, shadowOpa, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_opa(obj, shadowOpa, LV_PART_MAIN | LV_STATE_PRESSED);
    lv_obj_invalidate(obj);
}

static uint32_t lvglRandomStyleAccent(uint32_t avoid = 0)
{
    static const uint32_t palette[] = {
        0x3A8F4B, 0x2F6D86, 0x8A5A25, 0x925A73, 0x5A4CC7, 0x2B7D7D, 0xA35757, 0x2E6F95
    };
    const size_t count = sizeof(palette) / sizeof(palette[0]);
    uint32_t chosen = palette[random(0, static_cast<int>(count))];
    if (chosen == avoid) chosen = palette[(random(0, static_cast<int>(count - 1)) + 1U) % count];
    return chosen;
}

static uint32_t lvglFnv1a32(const char *text)
{
    uint32_t hash = 2166136261UL;
    if (!text) return hash;
    while (*text) {
        hash ^= static_cast<uint8_t>(*text++);
        hash *= 16777619UL;
    }
    return hash;
}

static String lvglOrderTokenFromText(const char *prefix, const String &text)
{
    char buf[48];
    snprintf(buf, sizeof(buf), "%s%08lx", prefix ? prefix : "", static_cast<unsigned long>(lvglFnv1a32(text.c_str())));
    return String(buf);
}

static String lvglSymbolText(const char *symbol, const char *text)
{
    if (!text || !*text) return symbol ? String(symbol) : String("");
    if (!symbol || !*symbol) return String(text);
    return String(symbol) + " " + text;
}

static String lvglSymbolText(const char *symbol, const String &text)
{
    return lvglSymbolText(symbol, text.c_str());
}

static lv_color_t lvglActiveToggleGreen(bool compact)
{
    return compact ? lv_color_hex(0x2B5A32) : lv_color_hex(0x28552F);
}

static LvglReorderItemEntry *lvglFindReorderItemEntry(lv_obj_t *obj)
{
    if (!obj || !lvglReorderItems) return nullptr;
    for (size_t i = 0; i < LVGL_REORDER_ITEM_CAPACITY; ++i) {
        if (lvglReorderItems[i].obj == obj) return &lvglReorderItems[i];
    }
    return nullptr;
}

static const LvglReorderItemEntry *lvglFindReorderItemEntryConst(const lv_obj_t *obj)
{
    if (!obj || !lvglReorderItems) return nullptr;
    for (size_t i = 0; i < LVGL_REORDER_ITEM_CAPACITY; ++i) {
        if (lvglReorderItems[i].obj == obj) return &lvglReorderItems[i];
    }
    return nullptr;
}

static void lvglPersistReorderForParent(lv_obj_t *parent, const char *prefKey)
{
    if (!parent || !prefKey || !*prefKey) return;
    String order;
    const uint32_t childCount = lv_obj_get_child_cnt(parent);
    for (uint32_t i = 0; i < childCount; ++i) {
        lv_obj_t *child = lv_obj_get_child(parent, i);
        const LvglReorderItemEntry *entry = lvglFindReorderItemEntryConst(child);
        if (!entry || entry->parent != parent || strcmp(entry->prefKey, prefKey) != 0 || entry->itemKey[0] == '\0') continue;
        if (!order.isEmpty()) order += '|';
        order += entry->itemKey;
    }
    uiPrefs.begin("ui", false);
    if (order.isEmpty()) uiPrefs.remove(prefKey);
    else uiPrefs.putString(prefKey, order);
    uiPrefs.end();
}

static void lvglApplySavedOrder(lv_obj_t *parent, const char *prefKey)
{
    if (!parent || !prefKey || !*prefKey || !lvglReorderItems) return;
    uiPrefs.begin("ui", true);
    String order = uiPrefs.getString(prefKey, "");
    uiPrefs.end();
    if (order.isEmpty()) return;

    int nextIndex = 0;
    int start = 0;
    while (start < static_cast<int>(order.length())) {
        int sep = order.indexOf('|', start);
        if (sep < 0) sep = order.length();
        const String token = order.substring(start, sep);
        start = sep + 1;
        if (token.isEmpty()) continue;

        for (size_t i = 0; i < LVGL_REORDER_ITEM_CAPACITY; ++i) {
            LvglReorderItemEntry &entry = lvglReorderItems[i];
            if (!entry.obj || !lv_obj_is_valid(entry.obj)) continue;
            if (entry.parent != parent || strcmp(entry.prefKey, prefKey) != 0) continue;
            if (strcmp(entry.itemKey, token.c_str()) != 0) continue;
            lv_obj_move_to_index(entry.obj, nextIndex++);
            break;
        }
    }
}

static void lvglEndReorderDrag(bool persistOrder)
{
    if (!lvglReorderDrag.obj || !lv_obj_is_valid(lvglReorderDrag.obj)) {
        lvglReorderDrag = {};
        lvglGestureBlocked = false;
        lvglReorderOwnsHorizontalGesture = false;
        return;
    }

    lv_obj_t *obj = lvglReorderDrag.obj;
    lv_obj_t *parent = lvglReorderDrag.parent;
    const LvglReorderItemEntry *entry = lvglFindReorderItemEntryConst(obj);
    const bool wasActive = lvglReorderDrag.active;
    if (lvglReorderDrag.originalWidth > 0 && lvglReorderDrag.originalHeight > 0) {
        lv_obj_set_size(obj, lvglReorderDrag.originalWidth, lvglReorderDrag.originalHeight);
    }
    lv_obj_set_style_translate_y(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_translate_y(obj, 0, LV_PART_MAIN | LV_STATE_PRESSED);
    lvglApplyReorderDragVisual(obj, false);
    if (lvglReorderDrag.parentOverflowTemporarilyEnabled && parent && lv_obj_is_valid(parent)) {
        lv_obj_clear_flag(parent, LV_OBJ_FLAG_OVERFLOW_VISIBLE);
    }
    if (lvglReorderDrag.parentScrollTemporarilyDisabled && parent && lv_obj_is_valid(parent)) {
        lv_obj_add_flag(parent, LV_OBJ_FLAG_SCROLLABLE);
    }

    if (persistOrder && parent && lv_obj_is_valid(parent) && entry) {
        lvglPersistReorderForParent(parent, entry->prefKey);
    }

    lvglReorderDrag = {};
    lvglGestureBlocked = false;
    lvglReorderOwnsHorizontalGesture = false;
    if (wasActive) lvglSuppressClicksAfterGesture();
    lvglRefreshAllButtonStyles();
}

static void lvglReorderItemDeleteEvent(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    if (!obj || !lvglReorderItems) return;
    for (size_t i = 0; i < LVGL_REORDER_ITEM_CAPACITY; ++i) {
        if (lvglReorderItems[i].obj == obj) {
            lvglReorderItems[i].obj = nullptr;
            lvglReorderItems[i].parent = nullptr;
            lvglReorderItems[i].prefKey[0] = '\0';
            lvglReorderItems[i].itemKey[0] = '\0';
        }
    }
    if (lvglReorderDrag.obj == obj) lvglEndReorderDrag(false);
}

static void lvglReorderItemEvent(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    LvglReorderItemEntry *entry = lvglFindReorderItemEntry(obj);
    if (!obj || !entry || !entry->parent || !lv_obj_is_valid(entry->parent)) return;

    const lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_PRESSED) {
        lv_indev_t *indev = lv_indev_get_act();
        lv_point_t pt = {0, 0};
        if (indev) lv_indev_get_point(indev, &pt);
        lvglReorderDrag.obj = obj;
        lvglReorderDrag.parent = entry->parent;
        lvglReorderDrag.pressedAtMs = millis();
        lvglReorderDrag.pressPoint = pt;
        lvglReorderDrag.fingerOffsetY = 0;
        lvglReorderDrag.originalWidth = lv_obj_get_width(obj);
        lvglReorderDrag.originalHeight = lv_obj_get_height(obj);
        lvglReorderDrag.active = false;
        lvglReorderDrag.cancelled = false;
        lvglReorderDrag.parentOverflowTemporarilyEnabled = false;
        lvglReorderDrag.parentScrollTemporarilyDisabled = false;
        return;
    }

    if (code == LV_EVENT_PRESSING) {
        if (lvglReorderDrag.obj != obj || lvglReorderDrag.parent != entry->parent) return;
        lv_indev_t *indev = lv_indev_get_act();
        if (!indev) return;
        lv_point_t pt = {0, 0};
        lv_indev_get_point(indev, &pt);

        if (!lvglReorderDrag.active) {
            if (!lvglReorderDrag.cancelled) {
                const int dx = static_cast<int>(pt.x) - static_cast<int>(lvglReorderDrag.pressPoint.x);
                const int dy = static_cast<int>(pt.y) - static_cast<int>(lvglReorderDrag.pressPoint.y);
                const int absDx = abs(dx);
                const int absDy = abs(dy);
                if ((absDx >= SWIPE_LOCK_MIN_DX && absDx > (absDy * 2)) ||
                    (absDy >= SWIPE_CANCEL_VERTICAL_DY && absDy > absDx)) {
                    lvglReorderDrag.cancelled = true;
                }
            }
            if (lvglReorderDrag.cancelled) return;
            if (static_cast<unsigned long>(millis() - lvglReorderDrag.pressedAtMs) < UI_REORDER_HOLD_MS) return;
            lv_area_t startArea;
            lv_obj_get_coords(obj, &startArea);
            lvglReorderDrag.fingerOffsetY = static_cast<lv_coord_t>(pt.y - ((startArea.y1 + startArea.y2) / 2));
            lvglReorderDrag.active = true;
            lvglGestureBlocked = true;
            lvglReorderOwnsHorizontalGesture = true;
            lvglResetGestureTracking();
            if (!lv_obj_has_flag(entry->parent, LV_OBJ_FLAG_OVERFLOW_VISIBLE)) {
                lv_obj_add_flag(entry->parent, LV_OBJ_FLAG_OVERFLOW_VISIBLE);
                lvglReorderDrag.parentOverflowTemporarilyEnabled = true;
            }
            if (lv_obj_has_flag(entry->parent, LV_OBJ_FLAG_SCROLLABLE)) {
                lv_obj_clear_flag(entry->parent, LV_OBJ_FLAG_SCROLLABLE);
                lvglReorderDrag.parentScrollTemporarilyDisabled = true;
            }
            const lv_coord_t dragWidth = static_cast<lv_coord_t>(lvglReorderDrag.originalWidth + max<lv_coord_t>(8, lvglReorderDrag.originalWidth / 8));
            const lv_coord_t dragHeight = static_cast<lv_coord_t>(lvglReorderDrag.originalHeight + max<lv_coord_t>(4, lvglReorderDrag.originalHeight / 8));
            lv_obj_set_size(obj, dragWidth, dragHeight);
            lvglApplyReorderDragVisual(obj, true);
        }

        if (lv_obj_has_flag(entry->parent, LV_OBJ_FLAG_SCROLLABLE)) {
            lv_area_t parentArea;
            lv_obj_get_coords(entry->parent, &parentArea);
            if (pt.y < (parentArea.y1 + UI_REORDER_AUTO_SCROLL_MARGIN)) {
                lv_obj_scroll_by(entry->parent, 0, -UI_REORDER_AUTO_SCROLL_STEP, LV_ANIM_OFF);
            } else if (pt.y > (parentArea.y2 - UI_REORDER_AUTO_SCROLL_MARGIN)) {
                lv_obj_scroll_by(entry->parent, 0, UI_REORDER_AUTO_SCROLL_STEP, LV_ANIM_OFF);
            }
        }

        lv_obj_set_style_translate_y(obj, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_translate_y(obj, 0, LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_update_layout(entry->parent);

        lv_area_t objArea;
        lv_obj_get_coords(obj, &objArea);
        const lv_coord_t baseCenterY = static_cast<lv_coord_t>((objArea.y1 + objArea.y2) / 2);
        const lv_coord_t desiredCenterY = static_cast<lv_coord_t>(pt.y - lvglReorderDrag.fingerOffsetY);
        const lv_coord_t deltaY = desiredCenterY - baseCenterY;

        int32_t targetIndex = lv_obj_get_index(obj);
        const uint32_t childCount = lv_obj_get_child_cnt(entry->parent);
        while (targetIndex > 0) {
            lv_obj_t *prevChild = lv_obj_get_child(entry->parent, targetIndex - 1);
            if (!prevChild || prevChild == obj) break;
            lv_area_t prevArea;
            lv_obj_get_coords(prevChild, &prevArea);
            const lv_coord_t prevCenterY = static_cast<lv_coord_t>((prevArea.y1 + prevArea.y2) / 2);
            const lv_coord_t prevHalfH = static_cast<lv_coord_t>((prevArea.y2 - prevArea.y1 + 1) / 2);
            const lv_coord_t swapThreshold = max<lv_coord_t>(UI_REORDER_SWAP_HYSTERESIS, prevHalfH / 3);
            if (desiredCenterY >= (prevCenterY - swapThreshold)) break;
            targetIndex--;
        }
        while (targetIndex < static_cast<int32_t>(childCount) - 1) {
            lv_obj_t *nextChild = lv_obj_get_child(entry->parent, targetIndex + 1);
            if (!nextChild || nextChild == obj) break;
            lv_area_t nextArea;
            lv_obj_get_coords(nextChild, &nextArea);
            const lv_coord_t nextCenterY = static_cast<lv_coord_t>((nextArea.y1 + nextArea.y2) / 2);
            const lv_coord_t nextHalfH = static_cast<lv_coord_t>((nextArea.y2 - nextArea.y1 + 1) / 2);
            const lv_coord_t swapThreshold = max<lv_coord_t>(UI_REORDER_SWAP_HYSTERESIS, nextHalfH / 3);
            if (desiredCenterY <= (nextCenterY + swapThreshold)) break;
            targetIndex++;
        }

        if (lv_obj_get_index(obj) != targetIndex) {
            lv_obj_move_to_index(obj, targetIndex);
            lv_obj_update_layout(entry->parent);
            lv_obj_get_coords(obj, &objArea);
        }

        const lv_coord_t refreshedCenterY = static_cast<lv_coord_t>((objArea.y1 + objArea.y2) / 2);
        const lv_coord_t refreshedDeltaY = desiredCenterY - refreshedCenterY;
        lv_obj_set_style_translate_y(obj, refreshedDeltaY, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_translate_y(obj, refreshedDeltaY, LV_PART_MAIN | LV_STATE_PRESSED);
        return;
    }

    if (code == LV_EVENT_RELEASED || code == LV_EVENT_PRESS_LOST) {
        if (lvglReorderDrag.obj == obj) {
            const bool persistOrder = lvglReorderDrag.active;
            if (lvglReorderDrag.active) lvglEndReorderDrag(persistOrder);
            else {
                lvglReorderDrag = {};
                lvglGestureBlocked = false;
                lvglReorderOwnsHorizontalGesture = false;
            }
        } else {
            lvglReorderOwnsHorizontalGesture = false;
        }
    }
}

static void lvglRegisterReorderableItem(lv_obj_t *obj, const char *prefKey, const char *itemKey)
{
    if (!obj || !prefKey || !*prefKey || !itemKey || !*itemKey) return;
    if (!lvglEnsureUiRegistryStorage()) return;
    lv_obj_t *parent = lv_obj_get_parent(obj);
    if (!parent) return;

    LvglReorderItemEntry *entry = lvglFindReorderItemEntry(obj);
    bool isNew = false;
    if (!entry) {
        for (size_t i = 0; i < LVGL_REORDER_ITEM_CAPACITY; ++i) {
            if (!lvglReorderItems[i].obj || !lv_obj_is_valid(lvglReorderItems[i].obj)) {
                entry = &lvglReorderItems[i];
                isNew = true;
                break;
            }
        }
    }
    if (!entry) return;

    entry->obj = obj;
    entry->parent = parent;
    snprintf(entry->prefKey, sizeof(entry->prefKey), "%s", prefKey);
    snprintf(entry->itemKey, sizeof(entry->itemKey), "%s", itemKey);

    if (!lv_obj_has_flag(obj, LV_OBJ_FLAG_CLICKABLE)) lv_obj_add_flag(obj, LV_OBJ_FLAG_CLICKABLE);

    if (isNew) {
        lv_obj_add_event_cb(obj, lvglReorderItemEvent, LV_EVENT_PRESSED, nullptr);
        lv_obj_add_event_cb(obj, lvglReorderItemEvent, LV_EVENT_PRESSING, nullptr);
        lv_obj_add_event_cb(obj, lvglReorderItemEvent, LV_EVENT_RELEASED, nullptr);
        lv_obj_add_event_cb(obj, lvglReorderItemEvent, LV_EVENT_PRESS_LOST, nullptr);
        lv_obj_add_event_cb(obj, lvglReorderItemDeleteEvent, LV_EVENT_DELETE, nullptr);
    }
}

static void lvglStyledButtonDeleteEvent(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    if (!obj || !lvglStyledButtons) return;
    for (size_t i = 0; i < LVGL_STYLED_BUTTON_CAPACITY; ++i) {
        if (lvglStyledButtons[i].obj == obj) lvglStyledButtons[i].obj = nullptr;
    }
}

static LvglStyledButtonEntry *lvglFindStyledButtonEntry(lv_obj_t *obj)
{
    if (!obj || !lvglStyledButtons) return nullptr;
    for (size_t i = 0; i < LVGL_STYLED_BUTTON_CAPACITY; ++i) {
        if (lvglStyledButtons[i].obj == obj) return &lvglStyledButtons[i];
    }
    return nullptr;
}

static const LvglStyledButtonEntry *lvglFindStyledButtonEntryConst(const lv_obj_t *obj)
{
    if (!obj || !lvglStyledButtons) return nullptr;
    for (size_t i = 0; i < LVGL_STYLED_BUTTON_CAPACITY; ++i) {
        if (lvglStyledButtons[i].obj == obj) return &lvglStyledButtons[i];
    }
    return nullptr;
}

static inline bool lvglBlackButtonThemeActive()
{
    return uiButtonStyleMode == UI_BUTTON_STYLE_BLACK;
}

static bool internetTimeValid()
{
    return time(nullptr) > 1700000000;
}

static void syncInternetTimeIfNeeded(bool force)
{
    if (airplaneModeEnabled || !wifiConnectedSafe()) return;
    const unsigned long nowMs = millis();
    const unsigned long minInterval = internetTimeValid() ? NTP_SYNC_REFRESH_MS : NTP_SYNC_RETRY_MS;
    if (!force && static_cast<unsigned long>(nowMs - topBarTimeSyncLastAttemptMs) < minInterval) return;
    configTime(0, 0, NTP_SERVER_1, NTP_SERVER_2, NTP_SERVER_3);
    topBarTimeSyncLastAttemptMs = nowMs;
}

static String buildGmtOffsetDropdownOptions()
{
    String options;
    for (int offset = TOP_BAR_GMT_MIN; offset <= TOP_BAR_GMT_MAX; ++offset) {
        if (!options.isEmpty()) options += '\n';
        options += "GMT";
        if (offset >= 0) options += '+';
        options += String(offset);
    }
    return options;
}

static const char *uiLanguageOptionName(UiLanguage lang)
{
    switch (lang) {
        case UI_LANG_RUSSIAN: return "Russian";
        case UI_LANG_CHINESE: return "Chinese";
        case UI_LANG_FRENCH: return "French";
        case UI_LANG_TURKISH: return "Turkish";
        case UI_LANG_ITALIAN: return "Italian";
        case UI_LANG_GERMAN: return "German";
        case UI_LANG_JAPANESE: return "Japanese";
        case UI_LANG_KOREAN: return "Korean";
        case UI_LANG_ENGLISH:
        default: return "English";
    }
}

static const char *vibrationIntensityLabel(VibrationIntensity intensity)
{
    switch (uiLanguage) {
        case UI_LANG_FRENCH:
            switch (intensity) {
                case VIBRATION_INTENSITY_LOW: return "Faible";
                case VIBRATION_INTENSITY_MEDIUM: return "Moyenne";
                case VIBRATION_INTENSITY_HIGH: return "Elevee";
                default: return "Moyenne";
            }
        case UI_LANG_TURKISH:
            switch (intensity) {
                case VIBRATION_INTENSITY_LOW: return "Dusuk";
                case VIBRATION_INTENSITY_MEDIUM: return "Orta";
                case VIBRATION_INTENSITY_HIGH: return "Yuksek";
                default: return "Orta";
            }
        case UI_LANG_ITALIAN:
            switch (intensity) {
                case VIBRATION_INTENSITY_LOW: return "Bassa";
                case VIBRATION_INTENSITY_MEDIUM: return "Media";
                case VIBRATION_INTENSITY_HIGH: return "Alta";
                default: return "Media";
            }
        case UI_LANG_GERMAN:
            switch (intensity) {
                case VIBRATION_INTENSITY_LOW: return "Niedrig";
                case VIBRATION_INTENSITY_MEDIUM: return "Mittel";
                case VIBRATION_INTENSITY_HIGH: return "Hoch";
                default: return "Mittel";
            }
        default:
            break;
    }
    switch (intensity) {
        case VIBRATION_INTENSITY_LOW: return "Low";
        case VIBRATION_INTENSITY_MEDIUM: return "Medium";
        case VIBRATION_INTENSITY_HIGH: return "High";
        default: return "Medium";
    }
}

static String buildLanguageDropdownOptions()
{
    String options;
    for (uint8_t i = 0; i < static_cast<uint8_t>(UI_LANG_COUNT); ++i) {
        if (!options.isEmpty()) options += '\n';
        options += uiLanguageOptionName(static_cast<UiLanguage>(i));
    }
    return options;
}

static const char *radioModuleLabel(RadioModuleType module)
{
    switch (module) {
        case RADIO_MODULE_E220: return "Ebyte E220-400T22D";
        case RADIO_MODULE_HC12:
        default: return "HC-12";
    }
}

static String buildRadioModuleDropdownOptions()
{
    String options = radioModuleLabel(RADIO_MODULE_HC12);
#if defined(BOARD_ESP32S3_3248S035_N16R8)
    options += "\n";
    options += radioModuleLabel(RADIO_MODULE_E220);
#endif
    return options;
}

static String buildVibrationIntensityDropdownOptions()
{
    String options;
    for (int i = 0; i < static_cast<int>(VIBRATION_INTENSITY_COUNT); ++i) {
        if (i > 0) options += "\n";
        options += vibrationIntensityLabel(static_cast<VibrationIntensity>(i));
    }
    return options;
}

static void saveSoundPrefs()
{
    uiPrefs.begin("ui", false);
    uiPrefs.putUChar("sys_vol", mediaVolumePercent);
    uiPrefs.putBool("vib_en", vibrationEnabled);
    uiPrefs.putUChar("vib_int", static_cast<uint8_t>(vibrationIntensity));
    uiPrefs.putBool("touch_snd", touchSoundEnabled);
    uiPrefs.putBool("boot_snd", startupSoundEnabled);
    uiPrefs.putBool("touch_vib", touchVibrationEnabled);
    uiPrefs.end();
}

static uint8_t uiSoundMode()
{
    if (mediaVolumePercent > 0) return 2;
    if (vibrationEnabled) return 1;
    return 0;
}

static const lv_img_dsc_t *uiSoundModeIcon()
{
    switch (uiSoundMode()) {
        case 0: return &img_silent_small_icon;
        case 1: return &img_vibro_small_icon;
        case 2:
        default: return &img_speaker_small_icon;
    }
}

static const lv_img_dsc_t *uiVolumeIcon()
{
    return mediaVolumePercent > 0 ? &img_speaker_small_icon : &img_silent_small_icon;
}

static const char *messageBeepToneLabel(MessageBeepTone tone)
{
    switch (tone) {
        case MESSAGE_BEEP_SINGLE: return "One Beep";
        case MESSAGE_BEEP_DOUBLE_SHORT: return "Two Short";
        case MESSAGE_BEEP_ASCEND: return "Ascend";
        case MESSAGE_BEEP_DESCEND: return "Descend";
        case MESSAGE_BEEP_DOORBELL: return "Doorbell";
        case MESSAGE_BEEP_WESTMINSTER: return "Westminster";
        case MESSAGE_BEEP_FUR_ELISE: return "Fur Elise";
        case MESSAGE_BEEP_ODE_TO_JOY: return "Ode to Joy";
        default: return "Two Short";
    }
}

static String buildMessageBeepDropdownOptions()
{
    String options;
    for (int i = 0; i < static_cast<int>(MESSAGE_BEEP_TONE_COUNT); ++i) {
        if (i > 0) options += "\n";
        options += messageBeepToneLabel(static_cast<MessageBeepTone>(i));
    }
    return options;
}

struct ChatBeepStep {
    uint16_t freq;
    uint16_t durationMs;
};

struct ChatBeepPattern {
    const ChatBeepStep *steps;
    uint8_t count;
};

static constexpr ChatBeepStep CHAT_BEEP_PATTERN_SINGLE[] = {
    {1760, 140}
};
static constexpr ChatBeepStep CHAT_BEEP_PATTERN_DOUBLE_SHORT[] = {
    {1760, 70}, {0, 55}, {1320, 95}
};
static constexpr ChatBeepStep CHAT_BEEP_PATTERN_ASCEND[] = {
    {988, 70}, {0, 35}, {1319, 70}, {0, 35}, {1760, 110}
};
static constexpr ChatBeepStep CHAT_BEEP_PATTERN_DESCEND[] = {
    {1760, 70}, {0, 35}, {1319, 70}, {0, 35}, {988, 110}
};
static constexpr ChatBeepStep CHAT_BEEP_PATTERN_DOORBELL[] = {
    {1047, 85}, {0, 40}, {1568, 150}
};
static constexpr ChatBeepStep CHAT_BEEP_PATTERN_WESTMINSTER[] = {
    {1568, 90}, {0, 30}, {1175, 90}, {0, 30}, {1319, 90}, {0, 30}, {988, 160}
};
static constexpr ChatBeepStep CHAT_BEEP_PATTERN_FUR_ELISE[] = {
    {1319, 90}, {0, 25}, {1245, 90}, {0, 25}, {1319, 90}, {0, 25}, {1245, 90}, {0, 25}, {1319, 120}
};
static constexpr ChatBeepStep CHAT_BEEP_PATTERN_ODE_TO_JOY[] = {
    {1319, 85}, {0, 25}, {1319, 85}, {0, 25}, {1480, 85}, {0, 25}, {1568, 85}, {0, 25}, {1568, 120}
};

static ChatBeepPattern chatMessageBeepPattern(MessageBeepTone tone)
{
    switch (tone) {
        case MESSAGE_BEEP_SINGLE: return {CHAT_BEEP_PATTERN_SINGLE, static_cast<uint8_t>(sizeof(CHAT_BEEP_PATTERN_SINGLE) / sizeof(CHAT_BEEP_PATTERN_SINGLE[0]))};
        case MESSAGE_BEEP_DOUBLE_SHORT: return {CHAT_BEEP_PATTERN_DOUBLE_SHORT, static_cast<uint8_t>(sizeof(CHAT_BEEP_PATTERN_DOUBLE_SHORT) / sizeof(CHAT_BEEP_PATTERN_DOUBLE_SHORT[0]))};
        case MESSAGE_BEEP_ASCEND: return {CHAT_BEEP_PATTERN_ASCEND, static_cast<uint8_t>(sizeof(CHAT_BEEP_PATTERN_ASCEND) / sizeof(CHAT_BEEP_PATTERN_ASCEND[0]))};
        case MESSAGE_BEEP_DESCEND: return {CHAT_BEEP_PATTERN_DESCEND, static_cast<uint8_t>(sizeof(CHAT_BEEP_PATTERN_DESCEND) / sizeof(CHAT_BEEP_PATTERN_DESCEND[0]))};
        case MESSAGE_BEEP_DOORBELL: return {CHAT_BEEP_PATTERN_DOORBELL, static_cast<uint8_t>(sizeof(CHAT_BEEP_PATTERN_DOORBELL) / sizeof(CHAT_BEEP_PATTERN_DOORBELL[0]))};
        case MESSAGE_BEEP_WESTMINSTER: return {CHAT_BEEP_PATTERN_WESTMINSTER, static_cast<uint8_t>(sizeof(CHAT_BEEP_PATTERN_WESTMINSTER) / sizeof(CHAT_BEEP_PATTERN_WESTMINSTER[0]))};
        case MESSAGE_BEEP_FUR_ELISE: return {CHAT_BEEP_PATTERN_FUR_ELISE, static_cast<uint8_t>(sizeof(CHAT_BEEP_PATTERN_FUR_ELISE) / sizeof(CHAT_BEEP_PATTERN_FUR_ELISE[0]))};
        case MESSAGE_BEEP_ODE_TO_JOY: return {CHAT_BEEP_PATTERN_ODE_TO_JOY, static_cast<uint8_t>(sizeof(CHAT_BEEP_PATTERN_ODE_TO_JOY) / sizeof(CHAT_BEEP_PATTERN_ODE_TO_JOY[0]))};
        default: return {CHAT_BEEP_PATTERN_DOUBLE_SHORT, static_cast<uint8_t>(sizeof(CHAT_BEEP_PATTERN_DOUBLE_SHORT) / sizeof(CHAT_BEEP_PATTERN_DOUBLE_SHORT[0]))};
    }
}

#include "app/ui_text.inc"
#include "app/radio_control.inc"
static void lvglSetMenuButtonIconMode(lv_obj_t *btn,
                                      const String &plainText,
                                      const char *symbol,
                                      const lv_img_dsc_t *customImg,
                                      lv_coord_t xOffset = 8,
                                      lv_coord_t labelShiftX = 10)
{
    if (!btn || !lv_obj_is_valid(btn)) return;
    lv_obj_t *label = lv_obj_get_child(btn, 0);
    if (!label) return;

    while (lv_obj_get_child_cnt(btn) > 1) {
        lv_obj_t *child = lv_obj_get_child(btn, lv_obj_get_child_cnt(btn) - 1);
        if (!child) break;
        lv_anim_del(child, nullptr);
        lv_obj_del(child);
    }

    const String labelText = (!menuCustomIconsEnabled && symbol && *symbol) ? lvglSymbolText(symbol, plainText) : plainText;
    lv_label_set_text(label, labelText.c_str());
    lv_obj_center(label);

    if (menuCustomIconsEnabled && customImg) {
        lvglAttachMenuButtonImage(btn, customImg, xOffset, labelShiftX);
    }
}

static void lvglRefreshPrimaryMenuButtonIcons()
{
    lvglSetMenuButtonIconMode(lvglHomeChatBtn, tr(TXT_CHAT), LV_SYMBOL_BELL, &img_chat_small_icon);
    lvglSetMenuButtonIconMode(lvglHomeRadioBtn, tr(TXT_RADIO_CONTROL), LV_SYMBOL_HOME, &img_radio_small_icon, 8, 12);
    lvglSetMenuButtonIconMode(lvglHomeMediaBtn, tr(TXT_MEDIA), LV_SYMBOL_AUDIO, &img_music_small_icon, 8, 12);
    lvglSetMenuButtonIconMode(lvglHomeInfoBtn, tr(TXT_INFO), LV_SYMBOL_WARNING, &img_info_small_icon);
    lvglSetMenuButtonIconMode(lvglHomeGamesBtn, tr(TXT_GAMES), LV_SYMBOL_PLAY, &img_games_small_icon);
    lvglSetMenuButtonIconMode(lvglHomeMqttControlsBtn, tr(TXT_MQTT_CONTROLS), LV_SYMBOL_LIST, &img_mqtt_controls_small_icon, 8, 12);
    lvglSetMenuButtonIconMode(lvglHomeConfigBtn, tr(TXT_CONFIG), LV_SYMBOL_SETTINGS, &img_config_small_icon);
    lvglSetMenuButtonIconMode(lvglHomePowerBtn, tr(TXT_POWER), LV_SYMBOL_POWER, &img_power_small_icon);
    if (menuCustomIconsEnabled) lvglSetButtonImageZoom(lvglHomePowerBtn, 205, 10, 12);

    lvglSetMenuButtonIconMode(lvglAirplaneBtn,
                              airplaneModeEnabled ? tr(TXT_AIRPLANE_ON) : tr(TXT_AIRPLANE_OFF),
                              LV_SYMBOL_CLOSE,
                              &img_airplane_mode_icon);
    lvglSetMenuButtonIconMode(lvglApModeBtn,
                              wifiSessionApMode ? tr(TXT_AP_MODE_ON) : tr(TXT_AP_MODE_OFF),
                              LV_SYMBOL_WIFI,
                              &img_ap_small_icon);
    lvglAirplaneBtnLabel = lvglAirplaneBtn ? lv_obj_get_child(lvglAirplaneBtn, 0) : nullptr;
    lvglApModeBtnLabel = lvglApModeBtn ? lv_obj_get_child(lvglApModeBtn, 0) : nullptr;

    lvglSetMenuButtonIconMode(lvglConfigWifiBtn, tr(TXT_WIFI_CONFIG), LV_SYMBOL_WIFI, &img_wifi_small_icon, 8, 12);
    lvglSetMenuButtonIconMode(lvglConfigHc12Btn, tr(TXT_HC12_CONFIG), LV_SYMBOL_SETTINGS, &img_radio_small_icon, 8, 12);
    lvglSetMenuButtonIconMode(lvglConfigRemoteBtn, tr(TXT_REMOTE_CONTROL), LV_SYMBOL_HOME, &img_radio_small_icon, 8, 12);
    lvglSetMenuButtonIconMode(lvglConfigStyleBtn, tr(TXT_SCREEN), LV_SYMBOL_SETTINGS, &img_styles_small_icon, 8, 12);
    lvglSetMenuButtonIconMode(lvglConfigMqttBtn, tr(TXT_MQTT_CONFIG), LV_SYMBOL_SETTINGS, &img_mqtt_conf_small_icon, 8, 12);
    lvglSetMenuButtonIconMode(lvglConfigScreenshotBtn, tr(TXT_SCREENSHOT), LV_SYMBOL_IMAGE, &img_screenshot_small_icon, 8, 12);
    lvglSetMenuButtonIconMode(lvglConfigLanguageBtn, tr(TXT_LANGUAGE), LV_SYMBOL_EDIT, &img_styles_small_icon, 8, 12);
    lvglSetMenuButtonIconMode(lvglConfigOtaBtn, tr(TXT_OTA_UPDATES), LV_SYMBOL_UPLOAD, &img_ota_small_icon, 8, 12);
    lvglSetMenuButtonIconMode(lvglConfigModulesBtn, "Modules", LV_SYMBOL_LIST, &img_config_small_icon, 8, 12);
    lvglSetMenuButtonIconMode(
        lvglConfigFactoryResetBtn,
        tr(TXT_FACTORY_RESET),
        LV_SYMBOL_WARNING,
        &img_power_small_icon,
        8,
        12
    );
    lvglRefreshBatteryTrainButtonIcons();
}

static void lvglSetHomeButtonVisibleState(lv_obj_t *btn, bool visible)
{
    if (!btn) return;
    if (visible) lv_obj_clear_flag(btn, LV_OBJ_FLAG_HIDDEN);
    else lv_obj_add_flag(btn, LV_OBJ_FLAG_HIDDEN);
}

static void lvglRefreshHomeButtonVisibility()
{
    lvglSetHomeButtonVisibleState(lvglHomeChatBtn, homeChatVisible && moduleInstalled[APP_MODULE_CHAT]);
    lvglSetHomeButtonVisibleState(lvglHomeRadioBtn, homeRadioVisible && moduleInstalled[APP_MODULE_RADIO]);
    lvglSetHomeButtonVisibleState(lvglHomeMediaBtn, homeMediaVisible && moduleInstalled[APP_MODULE_MEDIA]);
    lvglSetHomeButtonVisibleState(lvglHomeInfoBtn, homeInfoVisible && moduleInstalled[APP_MODULE_INFO]);
    lvglSetHomeButtonVisibleState(lvglHomeGamesBtn, homeGamesVisible && moduleInstalled[APP_MODULE_GAMES]);
    lvglSetHomeButtonVisibleState(lvglHomeMqttControlsBtn, homeMqttVisible && moduleInstalled[APP_MODULE_MQTT]);
    lvglSetHomeButtonVisibleState(lvglHomeConfigBtn, homeConfigVisible);
    lvglSetHomeButtonVisibleState(lvglHomePowerBtn, homePowerVisible);
    lvglSetHomeButtonVisibleState(lvglAirplaneBtn, homeAirplaneVisible);
    lvglSetHomeButtonVisibleState(lvglApModeBtn, homeApVisible);
}




static void lvglApplyMomentaryButtonStyle(lv_obj_t *btn, lv_obj_t *label, lv_color_t bodyCol, bool compact)
{
    if (!btn) return;

    if (lvglBlackButtonThemeActive()) {
        const lv_color_t blackBody = lv_color_hex(UI_BUTTON_BLACK_BODY_HEX);
        const lv_color_t activeGreen = lvglActiveToggleGreen(compact);
        const lv_color_t pressedCol = lv_color_mix(lv_color_black(), activeGreen, compact ? 44 : 58);
        const lv_color_t flashColor = lv_color_mix(lv_color_white(), activeGreen, compact ? 70 : 86);
        lv_obj_set_style_bg_color(btn, blackBody, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_bg_grad_color(btn, blackBody, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_bg_color(btn, activeGreen, LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_set_style_bg_grad_color(btn, activeGreen, LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_set_style_bg_color(btn, flashColor, LV_PART_MAIN | LV_STATE_CHECKED);
        lv_obj_set_style_bg_grad_color(btn, flashColor, LV_PART_MAIN | LV_STATE_CHECKED);
        lv_obj_set_style_bg_color(btn, pressedCol, LV_PART_MAIN | LV_STATE_CHECKED | LV_STATE_PRESSED);
        lv_obj_set_style_bg_grad_color(btn, pressedCol, LV_PART_MAIN | LV_STATE_CHECKED | LV_STATE_PRESSED);
        lv_obj_set_style_border_width(btn, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_border_color(btn, lv_color_hex(UI_BUTTON_BLACK_BORDER_HEX), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_border_opa(btn, static_cast<lv_opa_t>(84), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_border_width(btn, 1, LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_set_style_border_color(btn, lv_color_mix(lv_color_white(), activeGreen, 104), LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_set_style_shadow_width(btn, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_shadow_width(btn, 0, LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_set_style_translate_y(btn, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_translate_y(btn, 0, LV_PART_MAIN | LV_STATE_PRESSED);
    } else {
        const lv_color_t flashColor = lv_color_mix(lv_color_white(), bodyCol, UI_BUTTON_CLICK_FLASH_MIX);
        if (uiButtonStyleMode == UI_BUTTON_STYLE_FLAT) {
        const lv_color_t pressedCol = lv_color_mix(lv_color_black(), bodyCol, compact ? 32 : 44);
        lv_obj_set_style_bg_color(btn, bodyCol, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_bg_grad_color(btn, bodyCol, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_bg_color(btn, pressedCol, LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_set_style_bg_grad_color(btn, pressedCol, LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_set_style_bg_color(btn, flashColor, LV_PART_MAIN | LV_STATE_CHECKED);
        lv_obj_set_style_bg_grad_color(btn, flashColor, LV_PART_MAIN | LV_STATE_CHECKED);
        lv_obj_set_style_bg_color(btn, flashColor, LV_PART_MAIN | LV_STATE_CHECKED | LV_STATE_PRESSED);
        lv_obj_set_style_bg_grad_color(btn, flashColor, LV_PART_MAIN | LV_STATE_CHECKED | LV_STATE_PRESSED);
        lv_obj_set_style_border_width(btn, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_shadow_width(btn, compact ? 0 : 6, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_shadow_ofs_x(btn, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_shadow_ofs_y(btn, compact ? 0 : 2, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_shadow_opa(btn, compact ? LV_OPA_TRANSP : LV_OPA_20, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_shadow_width(btn, 0, LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_set_style_translate_y(btn, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_translate_y(btn, 0, LV_PART_MAIN | LV_STATE_PRESSED);
    } else {
        const lv_color_t topCol = lv_color_mix(lv_color_white(), bodyCol, compact ? 70 : 92);
        const lv_color_t bottomCol = lv_color_mix(lv_color_black(), bodyCol, compact ? 80 : 104);
        const lv_color_t pressedTopCol = lv_color_mix(lv_color_black(), bodyCol, compact ? 88 : 116);
        const lv_color_t pressedBottomCol = lv_color_mix(lv_color_white(), bodyCol, compact ? 56 : 76);
        const lv_color_t borderCol = lv_color_mix(lv_color_black(), bodyCol, compact ? 76 : 108);
        lv_obj_set_style_bg_color(btn, topCol, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_bg_grad_color(btn, bottomCol, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_bg_grad_dir(btn, LV_GRAD_DIR_VER, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_bg_color(btn, pressedTopCol, LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_set_style_bg_grad_color(btn, pressedBottomCol, LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_set_style_bg_grad_dir(btn, LV_GRAD_DIR_VER, LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_set_style_bg_color(btn, flashColor, LV_PART_MAIN | LV_STATE_CHECKED);
        lv_obj_set_style_bg_grad_color(btn, lv_color_mix(lv_color_white(), bottomCol, UI_BUTTON_CLICK_FLASH_MIX / 2),
                                       LV_PART_MAIN | LV_STATE_CHECKED);
        lv_obj_set_style_bg_color(btn, flashColor, LV_PART_MAIN | LV_STATE_CHECKED | LV_STATE_PRESSED);
        lv_obj_set_style_bg_grad_color(btn, lv_color_mix(lv_color_white(), bottomCol, UI_BUTTON_CLICK_FLASH_MIX / 2),
                                       LV_PART_MAIN | LV_STATE_CHECKED | LV_STATE_PRESSED);
        lv_obj_set_style_border_width(btn, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_border_color(btn, borderCol, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_border_opa(btn, LV_OPA_50, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_border_width(btn, 1, LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_set_style_border_color(btn, borderCol, LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_set_style_shadow_width(btn, compact ? 5 : 12, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_shadow_spread(btn, compact ? 0 : 1, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_shadow_ofs_x(btn, compact ? 1 : 3, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_shadow_ofs_y(btn, compact ? 2 : 4, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_shadow_color(btn, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_shadow_opa(btn, compact ? LV_OPA_30 : LV_OPA_40, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_shadow_width(btn, compact ? 1 : 3, LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_set_style_shadow_ofs_x(btn, 1, LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_set_style_shadow_ofs_y(btn, 1, LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_set_style_shadow_opa(btn, compact ? LV_OPA_10 : LV_OPA_20, LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_set_style_translate_y(btn, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_translate_y(btn, 1, LV_PART_MAIN | LV_STATE_PRESSED);
    }
    }

    if (label) {
        lv_obj_set_style_text_color(label, lv_color_hex(0xF7FBFF), 0);
        lv_obj_set_style_text_font(label, LV_FONT_DEFAULT, 0);
    }
    lv_obj_invalidate(btn);
}

static void lvglRegisterStyledButton(lv_obj_t *btn, lv_color_t baseColor, bool compact)
{
    if (!btn) return;
    if (!lvglEnsureUiRegistryStorage()) return;
    LvglStyledButtonEntry *entry = lvglFindStyledButtonEntry(btn);
    if (!entry) {
        for (size_t i = 0; i < LVGL_STYLED_BUTTON_CAPACITY; ++i) {
            if (!lvglStyledButtons[i].obj || !lv_obj_is_valid(lvglStyledButtons[i].obj)) {
                entry = &lvglStyledButtons[i];
                entry->obj = btn;
                lv_obj_add_event_cb(btn, lvglAirplaneButtonDrawEvent, LV_EVENT_DRAW_MAIN, nullptr);
                lv_obj_add_event_cb(btn, lvglStyledButtonDeleteEvent, LV_EVENT_DELETE, nullptr);
                break;
            }
        }
    }
    if (!entry) return;
    entry->baseColor = baseColor;
    entry->flags = compact ? LVGL_STYLED_BUTTON_FLAG_COMPACT : 0;
    lvglApplyMomentaryButtonStyle(btn, lv_obj_get_child(btn, 0), baseColor, compact);
}

lv_obj_t *lvglCreateMenuButton(lv_obj_t *parent, const char *txt, lv_color_t color, lv_event_cb_t cb, void *user)
{
    if (!parent) return nullptr;
    const bool compactList = (parent == lvglMediaList) || (parent == lvglWifiList) || (parent == lvglMqttCtrlList);
    lv_obj_t *btn = lv_btn_create(parent);
    if (!btn) {
        Serial.printf("[LVGL] alloc failed: menu button '%s'\n", txt ? txt : "?");
        return nullptr;
    }
    lv_obj_set_size(btn, lv_pct(100), compactList ? 38 : 44);
    lv_obj_set_style_radius(btn, 12, 0);
    lv_obj_set_style_border_width(btn, 0, 0);
    lv_obj_add_event_cb(btn, lvglClickFilterEvent, LV_EVENT_CLICKED, nullptr);
    lv_obj_add_event_cb(btn, lvglFilteredClickFlashEvent, LV_EVENT_CLICKED, nullptr);
    lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, user);
    lv_obj_t *lbl = lv_label_create(btn);
    if (lbl) {
        lv_label_set_text(lbl, txt);
        lv_obj_center(lbl);
    }
    lvglRegisterStyledButton(btn, color, compactList);
    return btn;
}

static void lvglAttachMenuButtonImage(lv_obj_t *btn, const lv_img_dsc_t *imgSrc, lv_coord_t xOffset = 8, lv_coord_t labelShiftX = 10)
{
    if (!btn || !imgSrc) return;
    lv_obj_t *img = lv_img_create(btn);
    if (!img) return;
    lv_img_set_src(img, imgSrc);
    lv_obj_clear_flag(img, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_align(img, LV_ALIGN_LEFT_MID, xOffset, 0);

    lv_obj_t *label = lv_obj_get_child(btn, 0);
    if (label) lv_obj_align(label, LV_ALIGN_CENTER, labelShiftX, 0);
}

static void lvglSetButtonImageZoom(lv_obj_t *btn, uint16_t zoom, lv_coord_t xOffset, lv_coord_t labelShiftX)
{
    if (!btn || lv_obj_get_child_cnt(btn) < 2) return;
    lv_obj_t *img = lv_obj_get_child(btn, 1);
    if (!img) return;
    lv_img_set_zoom(img, zoom);
    lv_obj_align(img, LV_ALIGN_LEFT_MID, xOffset, 0);

    lv_obj_t *label = lv_obj_get_child(btn, 0);
    if (label) lv_obj_align(label, LV_ALIGN_CENTER, labelShiftX, 0);
}

static void lvglApplyPersistentToggleButtonStyle(lv_obj_t *btn,
                                                 lv_obj_t *label,
                                                 bool enabled,
                                                 lv_color_t offBodyCol,
                                                 lv_color_t onBodyCol,
                                                 bool compact)
{
    if (!btn) return;

    const lv_color_t bodyCol = enabled ? lvglActiveToggleGreen(compact) : offBodyCol;
    const lv_color_t flashColor = lv_color_mix(lv_color_white(), bodyCol, UI_BUTTON_CLICK_FLASH_MIX);
    if (lvglBlackButtonThemeActive()) {
        const lv_color_t blackBody = lv_color_hex(UI_BUTTON_BLACK_BODY_HEX);
        const lv_color_t blackPressed = lv_color_hex(UI_BUTTON_BLACK_PRESSED_HEX);
        const lv_color_t blackFlash = lv_color_hex(UI_BUTTON_BLACK_FLASH_HEX);
        const lv_color_t activeBody = enabled ? bodyCol : blackBody;
        const lv_color_t activePressed = enabled ? lv_color_mix(lv_color_black(), bodyCol, compact ? 72 : 92) : blackPressed;
        const lv_color_t activeFlash = enabled ? flashColor : blackFlash;
        const lv_color_t borderCol = lv_color_hex(enabled ? UI_BUTTON_BLACK_BORDER_ACTIVE_HEX : UI_BUTTON_BLACK_BORDER_HEX);
        lv_obj_set_style_bg_color(btn, activeBody, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_bg_grad_color(btn, activeBody, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_bg_color(btn, activePressed, LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_set_style_bg_grad_color(btn, activePressed, LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_set_style_bg_color(btn, activeFlash, LV_PART_MAIN | LV_STATE_CHECKED);
        lv_obj_set_style_bg_grad_color(btn, activeFlash, LV_PART_MAIN | LV_STATE_CHECKED);
        lv_obj_set_style_bg_color(btn, activeFlash, LV_PART_MAIN | LV_STATE_CHECKED | LV_STATE_PRESSED);
        lv_obj_set_style_bg_grad_color(btn, activeFlash, LV_PART_MAIN | LV_STATE_CHECKED | LV_STATE_PRESSED);
        lv_obj_set_style_border_width(btn, enabled ? 2 : 1, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_border_color(btn, borderCol, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_border_opa(btn, enabled ? static_cast<lv_opa_t>(120) : static_cast<lv_opa_t>(84), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_border_width(btn, enabled ? 2 : 1, LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_set_style_border_color(btn, borderCol, LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_set_style_shadow_width(btn, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_shadow_width(btn, 0, LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_set_style_translate_y(btn, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_translate_y(btn, 0, LV_PART_MAIN | LV_STATE_PRESSED);
    } else if (uiButtonStyleMode == UI_BUTTON_STYLE_FLAT) {
        const lv_color_t pressedCol = lv_color_mix(lv_color_black(), bodyCol, compact ? 40 : 56);
        const lv_color_t borderCol = enabled
                                         ? lv_color_mix(lv_color_white(), bodyCol, compact ? 108 : 132)
                                         : lv_color_mix(lv_color_black(), bodyCol, compact ? 84 : 108);
        lv_obj_set_style_bg_color(btn, bodyCol, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_bg_grad_color(btn, bodyCol, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_bg_color(btn, pressedCol, LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_set_style_bg_grad_color(btn, pressedCol, LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_set_style_bg_color(btn, flashColor, LV_PART_MAIN | LV_STATE_CHECKED);
        lv_obj_set_style_bg_grad_color(btn, flashColor, LV_PART_MAIN | LV_STATE_CHECKED);
        lv_obj_set_style_bg_color(btn, flashColor, LV_PART_MAIN | LV_STATE_CHECKED | LV_STATE_PRESSED);
        lv_obj_set_style_bg_grad_color(btn, flashColor, LV_PART_MAIN | LV_STATE_CHECKED | LV_STATE_PRESSED);
        lv_obj_set_style_border_width(btn, enabled ? 2 : 1, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_border_color(btn, borderCol, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_border_opa(btn, enabled ? LV_OPA_90 : static_cast<lv_opa_t>(72), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_border_width(btn, enabled ? 2 : 1, LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_set_style_border_color(btn, borderCol, LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_set_style_shadow_width(btn, enabled ? 0 : (compact ? 0 : 4), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_shadow_ofs_x(btn, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_shadow_ofs_y(btn, compact ? 0 : 1, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_shadow_opa(btn, compact ? LV_OPA_TRANSP : static_cast<lv_opa_t>(18), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_shadow_width(btn, 0, LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_set_style_translate_y(btn, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_translate_y(btn, 0, LV_PART_MAIN | LV_STATE_PRESSED);
    } else {
        const lv_color_t lightCol = lv_color_mix(lv_color_white(), bodyCol, compact ? 56 : 76);
        const lv_color_t darkCol = lv_color_mix(lv_color_black(), bodyCol, compact ? 92 : 118);
        const lv_color_t topCol = enabled ? darkCol : lightCol;
        const lv_color_t bottomCol = enabled ? lightCol : darkCol;
        const lv_color_t pressedTopCol = lv_color_mix(lv_color_black(), topCol, compact ? 64 : 82);
        const lv_color_t pressedBottomCol = lv_color_mix(lv_color_black(), bottomCol, compact ? 52 : 68);
        const lv_color_t borderCol = enabled ? lv_color_mix(lv_color_black(), bodyCol, 104) : lv_color_mix(lv_color_white(), bodyCol, 88);
        const lv_color_t flashBottomCol = lv_color_mix(lv_color_white(), bottomCol, UI_BUTTON_CLICK_FLASH_MIX / 2);

        lv_obj_set_style_bg_color(btn, topCol, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_bg_grad_color(btn, bottomCol, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_bg_grad_dir(btn, LV_GRAD_DIR_VER, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_bg_main_stop(btn, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_bg_grad_stop(btn, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

        lv_obj_set_style_bg_color(btn, pressedTopCol, LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_set_style_bg_grad_color(btn, pressedBottomCol, LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_set_style_bg_grad_dir(btn, LV_GRAD_DIR_VER, LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_set_style_bg_main_stop(btn, 0, LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_set_style_bg_grad_stop(btn, 255, LV_PART_MAIN | LV_STATE_PRESSED);

        lv_obj_set_style_bg_color(btn, flashColor, LV_PART_MAIN | LV_STATE_CHECKED);
        lv_obj_set_style_bg_grad_color(btn, flashBottomCol, LV_PART_MAIN | LV_STATE_CHECKED);
        lv_obj_set_style_bg_grad_dir(btn, LV_GRAD_DIR_VER, LV_PART_MAIN | LV_STATE_CHECKED);
        lv_obj_set_style_bg_color(btn, flashColor, LV_PART_MAIN | LV_STATE_CHECKED | LV_STATE_PRESSED);
        lv_obj_set_style_bg_grad_color(btn, flashBottomCol, LV_PART_MAIN | LV_STATE_CHECKED | LV_STATE_PRESSED);
        lv_obj_set_style_bg_grad_dir(btn, LV_GRAD_DIR_VER, LV_PART_MAIN | LV_STATE_CHECKED | LV_STATE_PRESSED);

        lv_obj_set_style_border_width(btn, enabled ? 3 : 1, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_border_color(btn, borderCol, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_border_opa(btn, enabled ? static_cast<lv_opa_t>(76) : static_cast<lv_opa_t>(72), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_border_width(btn, enabled ? 2 : 1, LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_set_style_border_color(btn, borderCol, LV_PART_MAIN | LV_STATE_PRESSED);

        lv_obj_set_style_shadow_width(btn, compact ? (enabled ? 1 : 6) : (enabled ? 1 : 12), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_shadow_spread(btn, enabled ? 0 : 1, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_shadow_ofs_x(btn, compact ? (enabled ? 0 : 2) : (enabled ? 0 : 4), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_shadow_ofs_y(btn, compact ? (enabled ? 0 : 2) : (enabled ? 0 : 4), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_shadow_color(btn, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_shadow_opa(btn,
                                    compact ? (enabled ? static_cast<lv_opa_t>(6) : static_cast<lv_opa_t>(28))
                                            : (enabled ? static_cast<lv_opa_t>(8) : LV_OPA_40),
                                    LV_PART_MAIN | LV_STATE_DEFAULT);

        lv_obj_set_style_shadow_width(btn, compact ? 1 : 2, LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_set_style_shadow_ofs_x(btn, enabled ? 0 : 1, LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_set_style_shadow_ofs_y(btn, enabled ? 0 : 1, LV_PART_MAIN | LV_STATE_PRESSED);
        lv_obj_set_style_shadow_opa(btn, compact ? static_cast<lv_opa_t>(6) : static_cast<lv_opa_t>(10), LV_PART_MAIN | LV_STATE_PRESSED);

        lv_obj_set_style_translate_y(btn, enabled ? 2 : 0, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_translate_y(btn, enabled ? 3 : 1, LV_PART_MAIN | LV_STATE_PRESSED);
    }

    if (label) {
        lv_obj_set_style_text_color(label, lv_color_hex(0xF7FBFF), 0);
        lv_obj_set_style_text_font(label, LV_FONT_DEFAULT, 0);
    }

    lv_obj_invalidate(btn);
}

static void lvglAirplaneButtonDrawEvent(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_DRAW_MAIN) return;
    if (uiButtonStyleMode != UI_BUTTON_STYLE_3D) return;
    lv_obj_t *obj = lv_event_get_target(e);
    lv_draw_ctx_t *drawCtx = lv_event_get_draw_ctx(e);
    if (!obj || !drawCtx) return;

    lv_color_t bodyCol = lv_color_hex(0x4E5D6C);
    bool sunken = lv_obj_has_state(obj, LV_STATE_PRESSED);
    bool compact = false;
    if (obj == lvglAirplaneBtn) {
        bodyCol = airplaneModeEnabled ? lvglActiveToggleGreen(false) : lv_color_hex(0x98632E);
        sunken = airplaneModeEnabled || sunken;
    } else if (obj == lvglApModeBtn) {
        bodyCol = wifiSessionApMode ? lvglActiveToggleGreen(false) : lv_color_hex(0xA66A2A);
        sunken = wifiSessionApMode || sunken;
    } else if (obj == lvglChatDiscoveryBtn) {
        bodyCol = p2pDiscoveryEnabled ? lvglActiveToggleGreen(true) : lv_color_hex(0x4E5D6C);
        sunken = p2pDiscoveryEnabled || sunken;
        compact = true;
    } else if (obj == lvglWifiWebServerBtn) {
        bodyCol = webServerEnabled ? lvglActiveToggleGreen(true) : lv_color_hex(0x6B3A3A);
        sunken = webServerEnabled || sunken;
        compact = true;
    } else if (obj == lvglStyleButtonFlatBtn) {
        bodyCol = (uiButtonStyleMode == UI_BUTTON_STYLE_FLAT) ? lvglActiveToggleGreen(true) : lv_color_hex(uiButtonStyleFlatSelectorColor);
        sunken = (uiButtonStyleMode == UI_BUTTON_STYLE_FLAT) || sunken;
        compact = true;
    } else if (obj == lvglStyleButton3dBtn) {
        bodyCol = (uiButtonStyleMode == UI_BUTTON_STYLE_3D) ? lvglActiveToggleGreen(true) : lv_color_hex(uiButtonStyle3dSelectorColor);
        sunken = (uiButtonStyleMode == UI_BUTTON_STYLE_3D) || sunken;
        compact = true;
    } else {
        const LvglStyledButtonEntry *entry = lvglFindStyledButtonEntryConst(obj);
        if (!entry) return;
        bodyCol = entry->baseColor;
        compact = (entry->flags & LVGL_STYLED_BUTTON_FLAG_COMPACT) != 0;
    }

    lv_area_t a;
    lv_obj_get_coords(obj, &a);
    const int inset = compact ? 3 : 4;
    const int x1 = static_cast<int>(a.x1) + inset;
    const int y1 = static_cast<int>(a.y1) + inset;
    const int x2 = static_cast<int>(a.x2) - inset;
    const int y2 = static_cast<int>(a.y2) - inset;
    if (x2 <= x1 || y2 <= y1) return;

    const lv_color_t lightCol = lv_color_mix(lv_color_white(), bodyCol, compact ? 56 : 76);
    const lv_color_t darkCol = lv_color_mix(lv_color_black(), bodyCol, compact ? 92 : 118);
    const lv_opa_t strongOpa = sunken ? (compact ? static_cast<lv_opa_t>(56) : static_cast<lv_opa_t>(72))
                                      : (compact ? static_cast<lv_opa_t>(72) : LV_OPA_90);
    const lv_opa_t softOpa = sunken ? (compact ? static_cast<lv_opa_t>(34) : static_cast<lv_opa_t>(46))
                                    : (compact ? static_cast<lv_opa_t>(44) : LV_OPA_60);

    const lv_color_t topCol = sunken ? darkCol : lightCol;
    const lv_color_t leftCol = sunken ? darkCol : lightCol;
    const lv_color_t bottomCol = sunken ? lightCol : darkCol;
    const lv_color_t rightCol = sunken ? lightCol : darkCol;

    lvglDrawLineSeg(drawCtx, x1 + 3, y1, x2 - 3, y1, topCol, 1, strongOpa);
    lvglDrawLineSeg(drawCtx, x1, y1 + 3, x1, y2 - 3, leftCol, 1, strongOpa);
    lvglDrawLineSeg(drawCtx, x1 + 3, y2, x2 - 3, y2, bottomCol, 1, strongOpa);
    lvglDrawLineSeg(drawCtx, x2, y1 + 3, x2, y2 - 3, rightCol, 1, strongOpa);

    lvglDrawLineSeg(drawCtx, x1 + 4, y1 + 1, x2 - 4, y1 + 1, topCol, 1, softOpa);
    lvglDrawLineSeg(drawCtx, x1 + 1, y1 + 4, x1 + 1, y2 - 4, leftCol, 1, softOpa);
    lvglDrawLineSeg(drawCtx, x1 + 4, y2 - 1, x2 - 4, y2 - 1, bottomCol, 1, softOpa);
    lvglDrawLineSeg(drawCtx, x2 - 1, y1 + 4, x2 - 1, y2 - 4, rightCol, 1, softOpa);
}

void lvglApplyAirplaneButtonStyle()
{
    lvglApplyPersistentToggleButtonStyle(
        lvglAirplaneBtn, lvglAirplaneBtnLabel, airplaneModeEnabled, lv_color_hex(0x98632E), lv_color_hex(0x3B78B6), false);
}

void lvglApplyApModeButtonStyle()
{
    lvglApplyPersistentToggleButtonStyle(
        lvglApModeBtn, lvglApModeBtnLabel, wifiSessionApMode, lv_color_hex(0xA66A2A), lv_color_hex(0x3A7A3A), false);
}

void lvglApplyChatDiscoveryButtonStyle()
{
    lv_obj_t *label = lvglChatDiscoveryBtn ? lv_obj_get_child(lvglChatDiscoveryBtn, 0) : nullptr;
    lvglApplyPersistentToggleButtonStyle(
        lvglChatDiscoveryBtn, label, p2pDiscoveryEnabled, lv_color_hex(0x4E5D6C), lv_color_hex(0x3A7A3A), true);
}

void lvglApplyWifiWebServerButtonStyle()
{
    lv_obj_t *label = lvglWifiWebServerBtn ? lv_obj_get_child(lvglWifiWebServerBtn, 0) : nullptr;
    lvglApplyPersistentToggleButtonStyle(
        lvglWifiWebServerBtn, label, webServerEnabled, lv_color_hex(0x6B3A3A), lv_color_hex(0x357A38), true);
}

void lvglRefreshAllButtonStyles()
{
    if (!lvglStyledButtons) return;
    for (size_t i = 0; i < LVGL_STYLED_BUTTON_CAPACITY; ++i) {
        LvglStyledButtonEntry &entry = lvglStyledButtons[i];
        if (!entry.obj || !lv_obj_is_valid(entry.obj)) {
            entry.obj = nullptr;
            continue;
        }
        const bool compact = (entry.flags & LVGL_STYLED_BUTTON_FLAG_COMPACT) != 0;
        lvglApplyMomentaryButtonStyle(entry.obj, lv_obj_get_child(entry.obj, 0), entry.baseColor, compact);
    }
    lvglApplyAirplaneButtonStyle();
    lvglApplyApModeButtonStyle();
    lvglApplyChatDiscoveryButtonStyle();
    lvglApplyWifiWebServerButtonStyle();
    if (lvglStyleButtonFlatBtn) {
        lvglApplyPersistentToggleButtonStyle(lvglStyleButtonFlatBtn,
                                             lvglStyleButtonFlatBtnLabel,
                                             uiButtonStyleMode == UI_BUTTON_STYLE_FLAT,
                                             lv_color_hex(uiButtonStyleFlatSelectorColor),
                                             lv_color_hex(uiButtonStyleFlatSelectorColor),
                                             true);
    }
    if (lvglStyleButton3dBtn) {
        lvglApplyPersistentToggleButtonStyle(lvglStyleButton3dBtn,
                                             lvglStyleButton3dBtnLabel,
                                             uiButtonStyleMode == UI_BUTTON_STYLE_3D,
                                             lv_color_hex(uiButtonStyle3dSelectorColor),
                                             lv_color_hex(uiButtonStyle3dSelectorColor),
                                             true);
    }
    if (lvglStyleButtonBlackBtn) {
        lvglApplyPersistentToggleButtonStyle(lvglStyleButtonBlackBtn,
                                             lvglStyleButtonBlackBtnLabel,
                                             uiButtonStyleMode == UI_BUTTON_STYLE_BLACK,
                                             lv_color_hex(UI_BUTTON_BLACK_BODY_HEX),
                                             lv_color_hex(UI_BUTTON_BLACK_BODY_HEX),
                                             true);
    }
    if (lvglStyleTimezoneDd) {
        const uint16_t selected = static_cast<uint16_t>(topBarTimezoneGmtOffset - TOP_BAR_GMT_MIN);
        if (lv_dropdown_get_selected(lvglStyleTimezoneDd) != selected) {
            lv_dropdown_set_selected(lvglStyleTimezoneDd, selected);
        }
    }
    if (lvglStyleTopCenterNameBtn) {
        lvglApplyPersistentToggleButtonStyle(lvglStyleTopCenterNameBtn,
                                             lvglStyleTopCenterNameBtnLabel,
                                             topBarCenterMode == TOP_BAR_CENTER_NAME,
                                             lv_color_hex(0x3F4A57),
                                             lv_color_hex(0x3F4A57),
                                             true);
    }
    if (lvglStyleTopCenterTimeBtn) {
        lvglApplyPersistentToggleButtonStyle(lvglStyleTopCenterTimeBtn,
                                             lvglStyleTopCenterTimeBtnLabel,
                                             topBarCenterMode == TOP_BAR_CENTER_TIME,
                                             lv_color_hex(0x3F4A57),
                                             lv_color_hex(0x3F4A57),
                                             true);
    }
}

void lvglSetButtonStyleMode(UiButtonStyleMode mode, bool persist)
{
    if (mode != UI_BUTTON_STYLE_FLAT && mode != UI_BUTTON_STYLE_3D && mode != UI_BUTTON_STYLE_BLACK) {
        mode = UI_BUTTON_STYLE_3D;
    }
    uiButtonStyleMode = mode;
    if (persist) {
        uiPrefs.begin("ui", false);
        uiPrefs.putUChar("btn_style", static_cast<uint8_t>(uiButtonStyleMode));
        uiPrefs.end();
    }
    lvglRefreshAllButtonStyles();
    lvglRefreshStyleUi();
}
