const char *authName(wifi_auth_mode_t auth)
{
    return auth == WIFI_AUTH_OPEN ? "Open" : "Secured";
}

static bool wifiScanShouldUseSyncMode()
{
#if defined(BOARD_JC4880P443C_I_W)
    return true;
#else
    return false;
#endif
}

static void wifiPrepareScanRuntime()
{
#if defined(BOARD_JC4880P443C_I_W)
    stopWebServerRuntime();
    apModeActive = false;
    wifiStaGotIpPending = false;
    wifiStaDisconnectedPending = false;
    wifiForgetPendingUi = false;
    bootStaConnectInProgress = false;
    WiFi.scanDelete();
    WiFi.mode(WIFI_STA);
    WiFi.disconnect(false, false);
    delay(100);
#else
    wifiPrepareStaRuntime("scan", true);
#endif
}

static constexpr unsigned long WIFI_SCAN_TIMEOUT_MS = 12000UL;
static constexpr unsigned long WIFI_SCAN_STOP_WAIT_MS = 600UL;

static void wifiScanApplyResults(int scanResult, const String *statusOverride = nullptr, bool refreshUi = true)
{
    wifiScanInProgress = false;
    wifiScanStartedMs = 0;
    wifiCount = 0;

    if (scanResult > 0) {
        for (int i = 0; i < scanResult && wifiCount < MAX_WIFI_RESULTS; ++i) {
            wifiEntries[wifiCount].ssid = WiFi.SSID(i);
            wifiEntries[wifiCount].rssi = WiFi.RSSI(i);
            wifiEntries[wifiCount].auth = WiFi.encryptionType(i);
            wifiCount++;
        }
        if (statusOverride) uiStatusLine = *statusOverride;
        else uiStatusLine = "Scan complete: " + String(wifiCount);
    } else if (scanResult == 0) {
        if (statusOverride) uiStatusLine = *statusOverride;
        else uiStatusLine = "No access points found";
    } else {
        if (statusOverride) uiStatusLine = *statusOverride;
        else uiStatusLine = wifiScanFallbackUsed ? "WiFi scan timed out" : "WiFi scan failed";
    }

    WiFi.scanDelete();
    wifiScanFallbackUsed = false;
    if (refreshUi && lvglReady) {
        if (uiScreen == UI_WIFI_LIST) lvglRefreshWifiList();
        lvglSyncStatusLine();
    } else if (lvglReady) {
        lvglSyncStatusLine();
    }
}

void refreshWifiScan()
{
    if (!WIFI_RUNTIME_SUPPORTED) {
        wifiScanInProgress = false;
        wifiScanStartedMs = 0;
        wifiCount = 0;
        uiStatusLine = "WiFi on this board is handled by the onboard ESP32-C6 and is not enabled in this build";
        if (lvglReady) lvglSyncStatusLine();
        return;
    }
    wifiPrepareScanRuntime();
        if (wifiScanShouldUseSyncMode()) {
            wifiCount = 0;
            wifiScanInProgress = true;
            wifiScanFallbackUsed = false;
            wifiScanStartedMs = millis();
            wifiScanAnimLastMs = wifiScanStartedMs;
            wifiScanAnimPhase = 0;
            uiStatusLine = "Searching...";
            if (lvglReady) lvglSyncStatusLine();
            delay(50);
            const int syncResult = WiFi.scanNetworks(false, false);
            wifiScanApplyResults(syncResult);
            return;
        }
    const int scanState = WiFi.scanComplete();
    if (scanState == WIFI_SCAN_RUNNING) {
        const bool staleScan = !wifiScanInProgress || static_cast<unsigned long>(millis() - wifiScanStartedMs) >= WIFI_SCAN_TIMEOUT_MS;
        if (!staleScan) {
            uiStatusLine = "Searching...";
            if (lvglReady) lvglSyncStatusLine();
            return;
        }
        WiFi.scanDelete();
        wifiScanInProgress = false;
        wifiScanStartedMs = 0;
        wifiScanFallbackUsed = false;
        Serial.println("[WIFI] stale async scan aborted before restart");
    }

    WiFi.scanDelete();
    wifiCount = 0;
    wifiScanInProgress = true;
    wifiScanFallbackUsed = false;
    wifiScanStartedMs = millis();
    wifiScanAnimLastMs = wifiScanStartedMs;
    wifiScanAnimPhase = 0;
    delay(50);
    const int scanStart = WiFi.scanNetworks(true, false);
    if (scanStart == WIFI_SCAN_FAILED) {
        wifiScanInProgress = false;
        wifiScanStartedMs = 0;
        wifiScanFallbackUsed = true;
        Serial.println("[WIFI] async scan start failed, retrying sync scan");
        const int syncResult = WiFi.scanNetworks(false, false);
        wifiScanApplyResults(syncResult);
        return;
    }

    uiStatusLine = "Searching...";
    if (lvglReady) lvglSyncStatusLine();
}

void stopWifiScan(bool refreshUi)
{
    if (!WIFI_RUNTIME_SUPPORTED) {
        (void)refreshUi;
        wifiScanInProgress = false;
        wifiScanStartedMs = 0;
        wifiCount = 0;
        return;
    }
    if (!wifiScanInProgress) return;

    int scanResult = WiFi.scanComplete();
    if (scanResult == WIFI_SCAN_RUNNING) {
        esp_wifi_scan_stop();
        const unsigned long waitStartedMs = millis();
        do {
            delay(10);
            scanResult = WiFi.scanComplete();
        } while (scanResult == WIFI_SCAN_RUNNING &&
                 static_cast<unsigned long>(millis() - waitStartedMs) < WIFI_SCAN_STOP_WAIT_MS);
    }

    if (scanResult == WIFI_SCAN_RUNNING) {
        wifiScanInProgress = false;
        wifiScanStartedMs = 0;
        wifiScanFallbackUsed = false;
        wifiCount = 0;
        WiFi.scanDelete();
        uiStatusLine = "WiFi scan stopped";
        if (lvglReady) {
            if (refreshUi && uiScreen == UI_WIFI_LIST) lvglRefreshWifiList();
            lvglSyncStatusLine();
        }
        return;
    }

    String stoppedStatus;
    if (scanResult > 0) stoppedStatus = "Scan stopped: " + String(scanResult) + " found";
    else if (scanResult == 0) stoppedStatus = "Scan stopped: no access points found";
    else stoppedStatus = "WiFi scan stopped";
    wifiScanApplyResults(scanResult, &stoppedStatus, refreshUi);
}

void wifiScanService()
{
    if (!WIFI_RUNTIME_SUPPORTED) return;
    if (!wifiScanInProgress) return;
    int n = WiFi.scanComplete();
    if (n == WIFI_SCAN_RUNNING) {
        if (static_cast<unsigned long>(millis() - wifiScanStartedMs) < WIFI_SCAN_TIMEOUT_MS) return;

        // On some board/runtime combinations async scans can stick in WIFI_SCAN_RUNNING indefinitely.
        WiFi.scanDelete();
        wifiScanFallbackUsed = true;
        Serial.println("[WIFI] async scan timeout, retrying sync scan");
        n = WiFi.scanNetworks(false, false);
    }

    wifiScanApplyResults(n);
}

void clearWifiScanResults()
{
    if (wifiScanInProgress) {
        stopWifiScan(false);
    }
    wifiCount = 0;
    if (!WIFI_RUNTIME_SUPPORTED) return;
    WiFi.scanDelete();
}
