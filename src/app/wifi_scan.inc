const char *authName(wifi_auth_mode_t auth)
{
    return auth == WIFI_AUTH_OPEN ? "Open" : "Secured";
}

void refreshWifiScan()
{
    const wifi_mode_t scanMode = apModeActive ? WIFI_AP_STA : WIFI_STA;
    wifiEnsureRuntimeEnabled("scan", scanMode, apModeActive);
    const int scanState = WiFi.scanComplete();
    if (scanState == WIFI_SCAN_RUNNING) {
        uiStatusLine = "Searching for access points";
        if (lvglReady) lvglSyncStatusLine();
        return;
    }
    WiFi.scanDelete();
    wifiCount = 0;
    wifiScanInProgress = true;
    wifiScanAnimLastMs = millis();
    wifiScanAnimPhase = 0;
    // Add small delay to ensure WiFi radio is fully initialized before first scan
    delay(50);
    const int scanStart = WiFi.scanNetworks(true, true);
    if (scanStart == WIFI_SCAN_FAILED) {
        wifiScanInProgress = false;
        uiStatusLine = "WiFi scan start failed";
        if (lvglReady) lvglSyncStatusLine();
        return;
    }
    uiStatusLine = "Searching for access points";
    if (lvglReady) lvglSyncStatusLine();
}

void wifiScanService()
{
    if (!wifiScanInProgress) return;
    const int n = WiFi.scanComplete();
    if (n == WIFI_SCAN_RUNNING) return;

    wifiScanInProgress = false;
    wifiCount = 0;
    if (n > 0) {
        for (int i = 0; i < n && wifiCount < MAX_WIFI_RESULTS; i++) {
            wifiEntries[wifiCount].ssid = WiFi.SSID(i);
            wifiEntries[wifiCount].rssi = WiFi.RSSI(i);
            wifiEntries[wifiCount].auth = WiFi.encryptionType(i);
            wifiCount++;
        }
        uiStatusLine = "Scan complete: " + String(wifiCount);
    } else if (n == 0) {
        uiStatusLine = "No access points found";
    } else {
        uiStatusLine = "WiFi scan failed";
    }
    WiFi.scanDelete();
    if (lvglReady) {
        if (uiScreen == UI_WIFI_LIST) lvglRefreshWifiList();
        lvglSyncStatusLine();
    }
}

void clearWifiScanResults()
{
    wifiScanInProgress = false;
    wifiCount = 0;
    WiFi.scanDelete();
}
