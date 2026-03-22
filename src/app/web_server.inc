static void beginDnsForAp(IPAddress ip)
{
    if (ip[0] == 0) return;
    if (!webServerEnabled) return;
    dnsServer.stop();
    dnsServer.start(53, "*", ip);
    dnsRunning = true;
}

static void stopDnsForAp()
{
    if (!dnsRunning) return;
    dnsServer.stop();
    dnsRunning = false;
}

static void stopWebServerRuntime()
{
    if (mdnsStarted) {
        MDNS.end();
        mdnsStarted = false;
    }
    stopDnsForAp();
    if (webServerRunning) {
        wsCarInput.cleanupClients();
        server.end();
        webServerRunning = false;
    }
}

static void ensureWebServerRuntime()
{
    if (!webServerEnabled || airplaneModeEnabled || networkSuspendedForAudio) return;
    if (!webRoutesRegistered) {
        setupWebRoutes();
        webRoutesRegistered = true;
    }
    if (!webServerRunning) {
        server.begin();
        webServerRunning = true;
    }
    if (apModeActive) beginDnsForAp(WiFi.softAPIP());
}

static void persistWebServerEnabled()
{
    uiPrefs.begin("ui", false);
    uiPrefs.putBool("web_srv", webServerEnabled);
    uiPrefs.end();
}

static void wifiEnsureRuntimeEnabled(const char *reason, wifi_mode_t mode, bool startWebServer)
{
    if (airplaneModeEnabled) return;
    wifiRuntimeManaged = true;
    registerWifiEvents();
    WiFi.persistent(false);
    WiFi.setAutoReconnect(false);
    WiFi.setSleep(false);
    if (WiFi.getMode() != mode) WiFi.mode(mode);
    if (startWebServer && webServerEnabled) ensureWebServerRuntime();
    Serial.printf("[WIFI] runtime enabled reason=%s mode=%d web=%d\n",
                  reason ? reason : "-",
                  static_cast<int>(mode),
                  (startWebServer && webServerEnabled) ? 1 : 0);
}

void ensureApOnline(const char *reason)
{
    if (networkSuspendedForAudio || airplaneModeEnabled) return;
    if (apModeActive) return;
    normalizeSavedApCreds();
    wifiEnsureRuntimeEnabled(reason ? reason : "ensure_ap", WIFI_AP_STA, true);
    const char *apSsid = savedApSsid.length() ? savedApSsid.c_str() : AP_SSID;
    const char *apPass = savedApPass.length() ? savedApPass.c_str() : AP_PASS;
    if (!WiFi.softAP(apSsid, apPass)) {
        Serial.printf("[WIFI] AP start failed reason=%s\n", reason ? reason : "-");
        return;
    }
    apModeActive = true;
    const IPAddress ip = WiFi.softAPIP();
    beginDnsForAp(ip);
    uiStatusLine = "AP ready: " + ip.toString();
    if (lvglReady) {
        lvglRefreshConfigUi();
        lvglRefreshAllButtonStyles();
    }
    Serial.printf("[WIFI] AP enabled reason=%s ip=%s\n",
                  reason ? reason : "-",
                  ip.toString().c_str());
}

void forceSessionApMode(const char *reason)
{
    wifiSessionApMode = true;
    saveApModePref(true);
    wifiForgetPendingUi = false;
    pendingSaveCreds = false;
    pendingSaveSsid = "";
    pendingSavePass = "";
    bootStaConnectInProgress = false;
    bootStaConnectStartedMs = 0;
    staLastConnectAttemptMs = 0;
    WiFi.disconnect(false, false);
    ensureApOnline(reason ? reason : "session_ap_mode");
}

void disableApWhenStaConnected(const char *reason)
{
    if (!apModeActive && WiFi.getMode() == WIFI_STA) return;
    stopDnsForAp();
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_STA);
    apModeActive = false;
    if (lvglReady) {
        lvglRefreshConfigUi();
        lvglRefreshAllButtonStyles();
    }
    Serial.printf("[WIFI] AP disabled reason=%s\n", reason ? reason : "-");
}

static void beginStaConnectAttempt(const char *reason)
{
    if (networkSuspendedForAudio || airplaneModeEnabled || wifiSessionApMode || !wifiHasStaTarget()) return;
    const String ssid = wifiDesiredStaSsid();
    const String pass = wifiDesiredStaPass();
    const wifi_mode_t mode = apModeActive ? WIFI_AP_STA : WIFI_STA;
    wifiEnsureRuntimeEnabled(reason ? reason : "sta_connect", mode, apModeActive);
    WiFi.begin(ssid.c_str(), pass.c_str());
    bootStaConnectInProgress = true;
    bootStaConnectStartedMs = millis();
    staLastConnectAttemptMs = bootStaConnectStartedMs;
    uiStatusLine = "Connecting to " + ssid;
    Serial.printf("[WIFI] STA connect start reason=%s ssid=%s ap=%d\n",
                  reason ? reason : "-",
                  ssid.c_str(),
                  apModeActive ? 1 : 0);
}

void tryBootStaReconnect()
{
    if (airplaneModeEnabled) return;
    if (!wifiHasStaTarget()) {
        ensureApOnline("no_saved_sta");
        return;
    }
    apModeActive = false;
    stopDnsForAp();
    WiFi.mode(WIFI_STA);
    beginStaConnectAttempt("boot");
}

void startWifiConnect(const String &ssid, const String &pass)
{
    if (airplaneModeEnabled) {
        uiStatusLine = "Disable airplane mode first";
        lvglSyncStatusLine();
        return;
    }
    wifiSessionApMode = false;
    clearWifiScanResults();
    pendingSaveSsid = ssid;
    pendingSavePass = pass;
    pendingSaveCreds = true;
    ensureApOnline("manual_connect");
    beginStaConnectAttempt("manual_connect");
    lvglSyncStatusLine();
}

String urlDecode(const String &input)
{
    String s;
    char a = 0, b = 0;
    for (size_t i = 0; i < input.length(); i++) {
        if ((input[i] == '%') && (i + 2 < input.length()) && ((a = input[i + 1]) && (b = input[i + 2])) && isxdigit(a) && isxdigit(b)) {
            if (a >= 'a') a -= 'a' - 'A';
            if (a >= 'A') a = a - 'A' + 10;
            else a -= '0';
            if (b >= 'a') b -= 'a' - 'A';
            if (b >= 'A') b = b - 'A' + 10;
            else b -= '0';
            s += char(16 * a + b);
            i += 2;
        } else if (input[i] == '+') {
            s += ' ';
        } else {
            s += input[i];
        }
    }
    return s;
}

bool pathUnderWeb(const String &path)
{
    String p = path;
    if (!p.startsWith("/")) p = "/" + p;
    return (p == "/web") || p.startsWith("/web/");
}

void setupWifiAndServer()
{
    if (airplaneModeEnabled) {
        wifiRuntimeManaged = false;
        stopWebServerRuntime();
        WiFi.mode(WIFI_OFF);
        apModeActive = false;
        return;
    }

    wifiRuntimeManaged = true;
    registerWifiEvents();
    WiFi.persistent(false);
    WiFi.setAutoReconnect(false);
    WiFi.setSleep(false);
    stopDnsForAp();

    if (wifiSessionApMode) {
        Serial.println("Booting in saved AP mode");
        ensureApOnline("boot_pref");
        return;
    }

    WiFi.mode(WIFI_STA);
    apModeActive = false;
    ensureWebServerRuntime();

    loadSavedStaCreds();
    tryBootStaReconnect();
}
