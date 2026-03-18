static bool otaFetchLatestReleaseInfo(String &tagNameOut, String &binUrlOut, String &errorOut)
{
    return otaFetchReleaseCatalog(tagNameOut, binUrlOut, errorOut);
}

static bool otaFetchReleaseCatalog(String &latestTagOut, String &latestUrlOut, String &errorOut)
{
    latestTagOut = "";
    latestUrlOut = "";
    errorOut = "";
    const String prevSelectedTag =
        (otaSelectedReleaseIndex >= 0 && otaSelectedReleaseIndex < static_cast<int8_t>(OTA_RELEASE_LIST_MAX))
            ? otaReleaseEntries[otaSelectedReleaseIndex].tag
            : String();
    for (uint8_t i = 0; i < OTA_RELEASE_LIST_MAX; ++i) {
        otaReleaseEntries[i].tag = "";
        otaReleaseEntries[i].binUrl = "";
    }
    otaReleaseEntryCount = 0;
    otaSelectedReleaseIndex = -1;

    int ghStatus = -1;
    const String body = httpsGetText("https://api.github.com/repos/elik745i/ESP32-2432S024C-Remote/releases?per_page=10", &ghStatus);
    if (body.isEmpty()) {
        errorOut = "github_fetch_failed";
        return false;
    }

    JsonDocument filter;
    JsonObject rel = filter[0].to<JsonObject>();
    rel["tag_name"] = true;
    rel["draft"] = true;
    rel["prerelease"] = true;
    rel["assets"][0]["name"] = true;
    rel["assets"][0]["browser_download_url"] = true;

    JsonDocument parsed;
    const DeserializationError err = deserializeJson(parsed, body, DeserializationOption::Filter(filter));
    if (err || !parsed.is<JsonArray>()) {
        errorOut = "json_parse_failed";
        return false;
    }

    const String currentTag = String("v") + FW_VERSION;
    uint8_t releaseCount = 0;
    bool latestFound = false;

    for (JsonObjectConst releaseObj : parsed.as<JsonArrayConst>()) {
        const bool draft = releaseObj["draft"] | false;
        const bool prerelease = releaseObj["prerelease"] | false;
        if (draft || prerelease) continue;

        const String tag = String(static_cast<const char *>(releaseObj["tag_name"] | ""));
        const String url = chooseLatestFirmwareBinUrl(releaseObj["assets"]);
        if (tag.isEmpty() || url.isEmpty()) continue;

        if (!latestFound) {
            latestTagOut = tag;
            latestUrlOut = url;
            latestFound = true;
            continue;
        }

        if (tag.equalsIgnoreCase(currentTag)) continue;
        if (releaseCount >= OTA_RELEASE_LIST_MAX) continue;
        otaReleaseEntries[releaseCount].tag = tag;
        otaReleaseEntries[releaseCount].binUrl = url;
        if (!prevSelectedTag.isEmpty() && prevSelectedTag.equalsIgnoreCase(tag)) {
            otaSelectedReleaseIndex = static_cast<int8_t>(releaseCount);
        }
        releaseCount++;
    }

    otaReleaseEntryCount = releaseCount;
    if (otaSelectedReleaseIndex >= static_cast<int8_t>(otaReleaseEntryCount)) otaSelectedReleaseIndex = -1;
    if (!latestFound || latestTagOut.isEmpty() || latestUrlOut.isEmpty()) {
        errorOut = "release_asset_not_found";
        otaReleaseEntryCount = 0;
        otaSelectedReleaseIndex = -1;
        return false;
    }
    return true;
}

static bool otaFetchReleaseInfoForTag(const String &tagName, String &binUrlOut, String &errorOut)
{
    binUrlOut = "";
    errorOut = "";
    if (tagName.isEmpty()) {
        errorOut = "missing_tag";
        return false;
    }

    int ghStatus = -1;
    const String url = String("https://api.github.com/repos/elik745i/ESP32-2432S024C-Remote/releases/tags/") + tagName;
    const String body = httpsGetText(url, &ghStatus);
    if (body.isEmpty()) {
        errorOut = "github_fetch_failed";
        return false;
    }

    JsonDocument filter;
    filter["assets"][0]["name"] = true;
    filter["assets"][0]["browser_download_url"] = true;

    JsonDocument parsed;
    const DeserializationError err = deserializeJson(parsed, body, DeserializationOption::Filter(filter));
    if (err) {
        errorOut = "json_parse_failed";
        return false;
    }

    binUrlOut = chooseLatestFirmwareBinUrl(parsed["assets"]);
    if (binUrlOut.isEmpty()) {
        errorOut = "release_asset_not_found";
        return false;
    }
    return true;
}

static bool otaStartUpdateTask(const String &binUrl, const String &targetVersion, const String &statusText)
{
    if (otaUpdateTaskHandle || otaUiState == OTA_UI_DOWNLOADING || otaUiState == OTA_UI_FINALIZING) return false;
    if (!OTA_FIRMWARE_FLASH_SUPPORTED) {
        otaUiState = OTA_UI_ERROR;
        otaSetStatus("This board build does not have an OTA flash slot");
        if (lvglReady) lvglRefreshOtaUi();
        uiStatusLine = "OTA unsupported on current flash layout";
        lvglSyncStatusLine();
        return false;
    }
    if (!wifiConnectedSafe()) {
        otaUiState = OTA_UI_ERROR;
        otaSetStatus(apModeActive ? "Not connected to update server (AP mode only)" : "Not connected to update server");
        if (lvglReady) lvglRefreshOtaUi();
        uiStatusLine = apModeActive ? "OTA needs internet WiFi, not AP-only mode" : "Connect WiFi before OTA";
        lvglSyncStatusLine();
        return false;
    }
    if (binUrl.isEmpty()) {
        otaUiState = OTA_UI_ERROR;
        otaSetStatus("Firmware URL missing");
        if (lvglReady) lvglRefreshOtaUi();
        return false;
    }

    otaLatestBinUrl = binUrl;
    copyTextToBuf(otaLatestVersion, sizeof(otaLatestVersion), otaNormalizedVersion(targetVersion));
    otaUiState = OTA_UI_DOWNLOADING;
    otaProgressPercent = 0;
    otaUpdatePromptPending = false;
    otaUpdateAvailable = false;
    otaPersistAvailabilityState();
    otaSetStatus(statusText.isEmpty() ? String("Starting update...") : statusText);
    if (xTaskCreatePinnedToCore(otaUpdateTask, "ota_update", 12288, nullptr, 1, &otaUpdateTaskHandle, ARDUINO_RUNNING_CORE) != pdPASS) {
        otaUpdateTaskHandle = nullptr;
        otaUiState = OTA_UI_ERROR;
        otaSetStatus("Update start failed");
        if (lvglReady) lvglRefreshOtaUi();
        return false;
    }
    if (lvglReady) lvglRefreshOtaUi();
    return true;
}

static void otaSetStatus(const String &text)
{
    otaStatusText = text;
}

static void otaFinalizePendingBootImage()
{
#if defined(ESP_ARDUINO_VERSION_MAJOR)
    const esp_partition_t *running = esp_ota_get_running_partition();
    if (!running) return;

    esp_ota_img_states_t state = ESP_OTA_IMG_UNDEFINED;
    if (esp_ota_get_state_partition(running, &state) != ESP_OK) return;

    if (state == ESP_OTA_IMG_PENDING_VERIFY) {
        const esp_err_t markErr = esp_ota_mark_app_valid_cancel_rollback();
        if (markErr == ESP_OK) {
            Serial.printf("[OTA] marked running image valid: %s @ 0x%08lx\n",
                          running->label,
                          static_cast<unsigned long>(running->address));
        } else {
            Serial.printf("[OTA] mark app valid failed: %s\n", esp_err_to_name(markErr));
        }
    }
#endif
}

bool otaDownloadAndApplyFromUrl(const String &url, String &errorOut)
{
    errorOut = "";
    if (!OTA_FIRMWARE_FLASH_SUPPORTED) {
        errorOut = "ota_partition_unsupported";
        return false;
    }
    if (!wifiConnectedSafe()) {
        errorOut = "no_wifi";
        return false;
    }
    if (!url.length()) {
        errorOut = "missing_url";
        return false;
    }

    WiFiClientSecure client;
    client.setInsecure();

    HTTPClient http;
    http.setConnectTimeout(7000);
    http.setTimeout(12000);
    http.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);
    if (!http.begin(client, url)) {
        errorOut = "http_begin_failed";
        return false;
    }

    const int code = http.GET();
    if (code < 200 || code >= 300) {
        errorOut = "http_" + String(code);
        http.end();
        return false;
    }

    const int totalLen = http.getSize();
    WiFiClient *stream = http.getStreamPtr();
    if (!stream) {
        errorOut = "no_stream";
        http.end();
        return false;
    }

    if (!Update.begin(totalLen > 0 ? static_cast<size_t>(totalLen) : UPDATE_SIZE_UNKNOWN)) {
        errorOut = "update_begin_failed";
        http.end();
        return false;
    }

    uint8_t *buf = reinterpret_cast<uint8_t *>(allocPreferPsram(OTA_DOWNLOAD_BUF_SIZE));
    if (!buf) {
        errorOut = "oom_buffer";
        Update.abort();
        http.end();
        return false;
    }
    int remaining = totalLen;
    size_t writtenTotal = 0;
    otaUiState = OTA_UI_DOWNLOADING;
    otaProgressPercent = 0;
    otaSetStatus(totalLen > 0 ? "Downloading update..." : "Downloading update...");
    while (http.connected() && (remaining > 0 || remaining == -1)) {
        const size_t avail = static_cast<size_t>(stream->available());
        if (!avail) {
            delay(1);
            continue;
        }
        const size_t toRead = avail > OTA_DOWNLOAD_BUF_SIZE ? OTA_DOWNLOAD_BUF_SIZE : avail;
        const int n = stream->readBytes(buf, toRead);
        if (n <= 0) {
            delay(1);
            continue;
        }
        if (Update.write(buf, static_cast<size_t>(n)) != static_cast<size_t>(n)) {
            errorOut = "update_write_failed";
            Update.abort();
            http.end();
            free(buf);
            return false;
        }
        writtenTotal += static_cast<size_t>(n);
        if (remaining > 0) remaining -= n;
        if (totalLen > 0) {
            const size_t pct = min<size_t>(100U, (writtenTotal * 100U) / static_cast<size_t>(totalLen));
            otaProgressPercent = static_cast<uint8_t>(pct);
        }
        delay(0);
    }
    http.end();
    free(buf);

    otaUiState = OTA_UI_FINALIZING;
    otaProgressPercent = 100;
    otaSetStatus("Finalizing update...");
    if (!Update.end(true)) {
        errorOut = "update_end_failed";
        return false;
    }
    return true;
}

