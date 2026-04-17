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
    if (!WIFI_RUNTIME_SUPPORTED) return;
    if (!moduleInstalled[APP_MODULE_WEBSERVER] || !webServerEnabled || airplaneModeEnabled || networkSuspendedForAudio) return;
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

static bool wifiAutoApFallbackEnabled()
{
#if defined(BOARD_JC4880P443C_I_W)
    return false;
#else
    return true;
#endif
}

static bool wifiApRuntimeSupported()
{
#if defined(BOARD_JC4880P443C_I_W)
    return false;
#else
    return true;
#endif
}

static wifi_mode_t wifiStaScanOrConnectMode()
{
    return (wifiApRuntimeSupported() && apModeActive) ? WIFI_AP_STA : WIFI_STA;
}

static bool configureHostedWifiPinsIfNeeded()
{
#if defined(BOARD_JC4880P443C_I_W) && defined(BOARD_HAS_SDIO_ESP_HOSTED)
    static bool attempted = false;
    if (!attempted) {
        attempted = true;
        Serial.printf("[WIFI] using board hosted pins clk=%d cmd=%d d0=%d d1=%d d2=%d d3=%d rst=%d\n",
                      BOARD_SDIO_ESP_HOSTED_CLK,
                      BOARD_SDIO_ESP_HOSTED_CMD,
                      BOARD_SDIO_ESP_HOSTED_D0,
                      BOARD_SDIO_ESP_HOSTED_D1,
                      BOARD_SDIO_ESP_HOSTED_D2,
                      BOARD_SDIO_ESP_HOSTED_D3,
                      BOARD_SDIO_ESP_HOSTED_RESET);
    }
    return true;
#elif defined(BOARD_JC4880P443C_I_W) && defined(CONFIG_ESP_WIFI_REMOTE_ENABLED) && defined(CONFIG_ESP_SDIO_PIN_CLK) && defined(CONFIG_ESP_SDIO_PIN_CMD) && defined(CONFIG_ESP_SDIO_PIN_D0) && defined(CONFIG_ESP_SDIO_PIN_D1) && defined(CONFIG_ESP_SDIO_PIN_D2) && defined(CONFIG_ESP_SDIO_PIN_D3) && defined(CONFIG_ESP_SDIO_GPIO_RESET_SLAVE)
    static bool attempted = false;
    static bool configured = false;
    if (!attempted) {
        attempted = true;
        configured = WiFi.setPins(
            CONFIG_ESP_SDIO_PIN_CLK,
            CONFIG_ESP_SDIO_PIN_CMD,
            CONFIG_ESP_SDIO_PIN_D0,
            CONFIG_ESP_SDIO_PIN_D1,
            CONFIG_ESP_SDIO_PIN_D2,
            CONFIG_ESP_SDIO_PIN_D3,
            CONFIG_ESP_SDIO_GPIO_RESET_SLAVE
        );
        Serial.printf("[WIFI] hosted pin config %s clk=%d cmd=%d d0=%d d1=%d d2=%d d3=%d rst=%d\n",
                      configured ? "ok" : "failed",
                      CONFIG_ESP_SDIO_PIN_CLK,
                      CONFIG_ESP_SDIO_PIN_CMD,
                      CONFIG_ESP_SDIO_PIN_D0,
                      CONFIG_ESP_SDIO_PIN_D1,
                      CONFIG_ESP_SDIO_PIN_D2,
                      CONFIG_ESP_SDIO_PIN_D3,
                      CONFIG_ESP_SDIO_GPIO_RESET_SLAVE);
    }
    return configured;
#else
    return true;
#endif
}

static void wifiEnsureRuntimeEnabled(const char *reason, wifi_mode_t mode, bool startWebServer)
{
    if (!WIFI_RUNTIME_SUPPORTED) {
        (void)reason;
        (void)mode;
        (void)startWebServer;
        return;
    }
    if (airplaneModeEnabled) return;
    if (!configureHostedWifiPinsIfNeeded()) return;
    wifiRuntimeManaged = true;
    registerWifiEvents();
    WiFi.persistent(false);
    WiFi.setAutoReconnect(false);
    WiFi.setSleep(false);
    if (WiFi.getMode() != mode) WiFi.mode(mode);
    if (startWebServer && moduleInstalled[APP_MODULE_WEBSERVER] && webServerEnabled) ensureWebServerRuntime();
    Serial.printf("[WIFI] runtime enabled reason=%s mode=%d web=%d\n",
                  reason ? reason : "-",
                  static_cast<int>(mode),
                  (startWebServer && moduleInstalled[APP_MODULE_WEBSERVER] && webServerEnabled) ? 1 : 0);
}

static void wifiPrepareStaRuntime(const char *reason, bool disconnectFirst)
{
    if (!WIFI_RUNTIME_SUPPORTED) {
        (void)reason;
        (void)disconnectFirst;
        return;
    }
    wifiEnsureRuntimeEnabled(reason ? reason : "sta_prepare", wifiStaScanOrConnectMode(), apModeActive);
#if defined(BOARD_JC4880P443C_I_W)
    if (disconnectFirst) {
        wifiStaGotIpPending = false;
        wifiStaDisconnectedPending = false;
        WiFi.disconnect(false, false);
        delay(100);
    }
#else
    (void)disconnectFirst;
#endif
}

static wifi_mode_t wifiDesiredApMode()
{
    return (wifiApRuntimeSupported() && (wifiHasStaTarget() || pendingSaveCreds || bootStaConnectInProgress)) ? WIFI_AP_STA : WIFI_AP;
}

void ensureApOnline(const char *reason)
{
    if (!WIFI_RUNTIME_SUPPORTED) return;
    if (networkSuspendedForAudio || airplaneModeEnabled) return;
    if (apModeActive) return;
    if (!wifiApRuntimeSupported()) {
        wifiSessionApMode = false;
        saveApModePref(false);
        apModeActive = false;
        uiStatusLine = "AP mode unavailable on this board";
        if (lvglReady) {
            lvglSyncStatusLine();
            lvglRefreshConfigUi();
            lvglRefreshAllButtonStyles();
        }
        Serial.printf("[WIFI] AP request ignored reason=%s board_sta_only=1\n", reason ? reason : "-");
        return;
    }
    normalizeSavedApCreds();
    wifiEnsureRuntimeEnabled(reason ? reason : "ensure_ap", wifiDesiredApMode(), true);
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
    if (!WIFI_RUNTIME_SUPPORTED) {
        (void)reason;
        wifiSessionApMode = false;
        saveApModePref(false);
        uiStatusLine = "WiFi uses onboard ESP32-C6; hosted support is not enabled in this build";
        if (lvglReady) lvglSyncStatusLine();
        return;
    }
    if (!wifiApRuntimeSupported()) {
        wifiSessionApMode = false;
        saveApModePref(false);
        uiStatusLine = "AP mode unavailable on this board";
        if (lvglReady) {
            lvglSyncStatusLine();
            lvglRefreshConfigUi();
            lvglRefreshAllButtonStyles();
        }
        Serial.printf("[WIFI] session AP ignored reason=%s board_sta_only=1\n", reason ? reason : "-");
        return;
    }
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
    if (!WIFI_RUNTIME_SUPPORTED) {
        (void)reason;
        apModeActive = false;
        return;
    }
    if (!apModeActive && WiFi.getMode() == WIFI_STA) return;
    stopDnsForAp();
    WiFi.softAPdisconnect(true);
    if (!configureHostedWifiPinsIfNeeded()) return;
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
    if (!WIFI_RUNTIME_SUPPORTED) {
        (void)reason;
        return;
    }
    if (networkSuspendedForAudio || airplaneModeEnabled || wifiSessionApMode || !wifiHasStaTarget()) return;
    const String ssid = wifiDesiredStaSsid();
    const String pass = wifiDesiredStaPass();
    wifiPrepareStaRuntime(reason ? reason : "sta_connect", true);
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
    if (!WIFI_RUNTIME_SUPPORTED) return;
    if (airplaneModeEnabled) return;
    if (!wifiHasStaTarget()) {
        if (wifiAutoApFallbackEnabled()) {
            ensureApOnline("no_saved_sta");
        } else {
            uiStatusLine = "WiFi disconnected";
            Serial.println("[WIFI] no saved STA; staying in STA-only idle mode");
        }
        return;
    }
    apModeActive = false;
    stopDnsForAp();
    beginStaConnectAttempt("boot");
}

void startWifiConnect(const String &ssid, const String &pass)
{
    if (!WIFI_RUNTIME_SUPPORTED) {
        (void)ssid;
        (void)pass;
        uiStatusLine = "WiFi uses onboard ESP32-C6; hosted support is not enabled in this build";
        lvglSyncStatusLine();
        return;
    }
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
    if (!WIFI_RUNTIME_SUPPORTED) {
        wifiRuntimeManaged = false;
        bootStaConnectInProgress = false;
        apModeActive = false;
        stopWebServerRuntime();
        return;
    }
    if (airplaneModeEnabled) {
        wifiRuntimeManaged = false;
        stopWebServerRuntime();
        WiFi.mode(WIFI_OFF);
        apModeActive = false;
        return;
    }

    wifiRuntimeManaged = true;
    if (!configureHostedWifiPinsIfNeeded()) return;
    registerWifiEvents();
    WiFi.persistent(false);
    WiFi.setAutoReconnect(false);
    WiFi.setSleep(false);
    stopDnsForAp();
    loadSavedStaCreds();

    if (wifiSessionApMode) {
        if (wifiAutoApFallbackEnabled()) {
            Serial.println("Booting in saved AP mode");
            ensureApOnline("boot_pref");
            return;
        }
        wifiSessionApMode = false;
        saveApModePref(false);
        Serial.println("[WIFI] ignoring saved AP boot preference on this board");
    }

    apModeActive = false;
    tryBootStaReconnect();
}
