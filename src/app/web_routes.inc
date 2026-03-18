String mimeFor(const String &path)
{
    if (path.endsWith(".html")) return "text/html";
    if (path.endsWith(".css")) return "text/css";
    if (path.endsWith(".js")) return "application/javascript";
    if (path.endsWith(".json")) return "application/json";
    if (path.endsWith(".png")) return "image/png";
    if (path.endsWith(".jpg") || path.endsWith(".jpeg")) return "image/jpeg";
    if (path.endsWith(".svg")) return "image/svg+xml";
    if (path.endsWith(".ico")) return "image/x-icon";
    if (path.endsWith(".txt")) return "text/plain";
    if (path.endsWith(".wav")) return "audio/wav";
    if (path.endsWith(".bin")) return "application/octet-stream";
    return "application/octet-stream";
}

void sendUploadFallbackPage(AsyncWebServerRequest *request)
{
    String html = FPSTR(UPLOAD_FALLBACK_HTML);
    html.replace("%FIRMWARE_VERSION%", FW_VERSION);
    request->send(200, "text/html", html);
}

void sendWifiSetupPage(AsyncWebServerRequest *request)
{
    String html = FPSTR(WIFI_SETUP_HTML);
    html.replace("%FIRMWARE_VERSION%", FW_VERSION);
    html.replace("%AP_SSID%", AP_SSID);
    html.replace("%AP_IP%", WiFi.softAPIP().toString());
    request->send(200, "text/html", html);
}

String apLandingContinueUrl()
{
    if (sdEnsureMounted() && hasPrimaryWebUi()) return "/?skip_ap_portal=1";
    return "/recovery";
}

void sendApWifiLandingPage(AsyncWebServerRequest *request)
{
    const String continueUrl = apLandingContinueUrl();
    const bool hasFrontend = (continueUrl == "/?skip_ap_portal=1");
    String html = FPSTR(AP_WIFI_LANDING_HTML);
    html.replace("%FIRMWARE_VERSION%", FW_VERSION);
    html.replace("%AP_SSID%", savedApSsid.length() ? savedApSsid : String(AP_SSID));
    html.replace("%AP_IP%", WiFi.softAPIP().toString());
    html.replace("%CONTINUE_URL%", continueUrl);
    html.replace("%CONTINUE_LABEL%", hasFrontend ? "the main web UI" : "the recovery browser");
    html.replace("%CONTINUE_HINT%", hasFrontend ? "Close to open the SD web frontend." : "Close to open the embedded recovery browser.");
    request->send(200, "text/html", html);
}

void sendRecoveryBrowserPage(AsyncWebServerRequest *request)
{
    AsyncWebServerResponse *res = request->beginResponse(
        200,
        "text/html",
        reinterpret_cast<const uint8_t *>(RECOVERY_BROWSER_HTML_GZ),
        RECOVERY_BROWSER_HTML_GZ_LEN
    );
    res->addHeader("Content-Encoding", "gzip");
    res->addHeader("Cache-Control", "public, max-age=300");
    request->send(res);
}

uint8_t wifiQualityPercentFromRssi(int32_t rssi)
{
    if (rssi <= -100) return 0;
    if (rssi >= -50) return 100;
    return static_cast<uint8_t>((rssi + 100) * 2);
}

uint32_t heapFragPercent(uint32_t freeBytes, uint32_t largestFreeBlock)
{
    if (freeBytes == 0 || largestFreeBlock >= freeBytes) return 0;
    return 100U - static_cast<uint32_t>((static_cast<uint64_t>(largestFreeBlock) * 100ULL) / freeBytes);
}

float readChipTemperatureC()
{
    const float tempC = temperatureRead();
    return isfinite(tempC) ? tempC : 0.0f;
}

void sendCarInputTelemetrySnapshot(AsyncWebSocketClient *client)
{
    sampleTopIndicators();

    if (ESP.getFreeHeap() < WS_TELEMETRY_MIN_FREE_HEAP) return;

    const bool staConnected = wifiConnectedSafe();
    const int wifiQuality = staConnected ? static_cast<int>(wifiQualityPercentFromRssi(wifiRssiSafe())) : 0;
    const unsigned long uptimeSecs = millis() / 1000UL;
    const float chipTempC = readChipTemperatureC();
    const uint32_t heapTotal = static_cast<uint32_t>(ESP.getHeapSize());
    const uint32_t heapFree = static_cast<uint32_t>(ESP.getFreeHeap());
    const uint32_t psramTotal = static_cast<uint32_t>(heap_caps_get_total_size(MALLOC_CAP_SPIRAM));
    const uint32_t psramFree = static_cast<uint32_t>(heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

    char msg[96];
    auto sendText = [&](const char *text) {
        if (client) client->text(text);
        else wsCarInput.textAll(text);
    };

    snprintf(msg, sizeof(msg), "BATT,%u,%.2f,%d", batteryPercent, batteryVoltage, wifiQuality);
    sendText(msg);

    snprintf(msg, sizeof(msg), "AIRPLANE,%s", airplaneModeEnabled ? "ON" : "OFF");
    sendText(msg);

    snprintf(msg, sizeof(msg), "CHARGE,%s", batteryCharging ? "YES" : "NO");
    sendText(msg);

    snprintf(msg, sizeof(msg), "STATS,%lu,%.1f", uptimeSecs, chipTempC);
    sendText(msg);

    snprintf(msg, sizeof(msg), "HEAP,%lu,%lu",
             static_cast<unsigned long>(heapTotal),
             static_cast<unsigned long>(heapFree));
    sendText(msg);

    snprintf(msg, sizeof(msg), "PSRAM,%lu,%lu",
             static_cast<unsigned long>(psramTotal),
             static_cast<unsigned long>(psramFree));
    sendText(msg);
}

void handleCarInputSocketEvent(AsyncWebSocket *serverRef,
                               AsyncWebSocketClient *client,
                               AwsEventType type,
                               void *arg,
                               uint8_t *data,
                               size_t len)
{
    (void)serverRef;
    if (!client) return;

    if (type == WS_EVT_CONNECT) {
        sendCarInputTelemetrySnapshot(client);
        return;
    }

    if (type != WS_EVT_DATA || !arg || !data || len == 0) return;

    AwsFrameInfo *info = reinterpret_cast<AwsFrameInfo *>(arg);
    if (!info || info->opcode != WS_TEXT || !info->final || info->index != 0 || info->len != len) return;

    char msg[80];
    const size_t copyLen = min(len, sizeof(msg) - 1U);
    memcpy(msg, data, copyLen);
    msg[copyLen] = '\0';

    if (strcmp(msg, "PING") == 0 || strcmp(msg, "ping") == 0) {
        client->text("PONG");
        return;
    }

    if (handleUiSettingMessage(msg)) {
        return;
    }
}

void setupCarInputSocket()
{
    wsCarInput.onEvent(handleCarInputSocketEvent);
    server.addHandler(&wsCarInput);
}

void serviceCarInputTelemetry()
{
    if (!webServerRunning) return;
    if (static_cast<unsigned long>(millis() - lastCarInputTelemetryMs) < CAR_INPUT_TELEMETRY_PERIOD_MS) return;
    lastCarInputTelemetryMs = millis();
    wsCarInput.cleanupClients();
    sendCarInputTelemetrySnapshot(nullptr);
}

bool captureScreenToJpeg(String &savedPathOut, String &errorOut)
{
    savedPathOut = "";
    errorOut = "";
    if (!sdEnsureMounted()) { errorOut = "no_sd"; return false; }
    if (mediaIsPlaying || mediaPaused) { errorOut = "media_busy"; return false; }
    if (!sdLock()) { errorOut = "sd_lock"; return false; }

    if (!SD.exists("/Screenshots") && !SD.mkdir("/Screenshots")) {
        sdUnlock();
        errorOut = "mkdir_failed";
        return false;
    }

    const time_t now = time(nullptr);
    const bool timeValid = now > 1700000000;
    struct tm tmNow = {};
    if (timeValid) localtime_r(&now, &tmNow);
    else {
        unsigned long sec = millis() / 1000UL;
        tmNow.tm_year = 70;
        tmNow.tm_mon = 0;
        tmNow.tm_mday = 1;
        tmNow.tm_hour = static_cast<int>((sec / 3600UL) % 24UL);
        tmNow.tm_min = static_cast<int>((sec / 60UL) % 60UL);
        tmNow.tm_sec = static_cast<int>(sec % 60UL);
    }

    char name[80];
    snprintf(
        name,
        sizeof(name),
        "/Screenshots/screenshot_%04d%02d%02d_%02d%02d%02d.jpg",
        tmNow.tm_year + 1900,
        tmNow.tm_mon + 1,
        tmNow.tm_mday,
        tmNow.tm_hour,
        tmNow.tm_min,
        tmNow.tm_sec
    );
    String path(name);
    if (SD.exists(path)) {
        for (int i = 1; i <= 999; i++) {
            snprintf(
                name,
                sizeof(name),
                "/Screenshots/screenshot_%04d%02d%02d_%02d%02d%02d_%03d.jpg",
                tmNow.tm_year + 1900,
                tmNow.tm_mon + 1,
                tmNow.tm_mday,
                tmNow.tm_hour,
                tmNow.tm_min,
                tmNow.tm_sec,
                i
            );
            if (!SD.exists(name)) {
                path = String(name);
                break;
            }
        }
    }
    if (path.isEmpty()) {
        sdUnlock();
        errorOut = "name_exhausted";
        return false;
    }

    auto jpegOpen = [](const char *filename) -> void * {
        (void)filename;
        screenshotJpegFile = SD.open(screenshotJpegPath, FILE_WRITE);
        if (!screenshotJpegFile) return nullptr;
        return static_cast<void *>(&screenshotJpegFile);
    };
    auto jpegClose = [](JPEGE_FILE *pFile) {
        File *f = reinterpret_cast<File *>(pFile ? pFile->fHandle : nullptr);
        if (!f || !(*f)) return;
        f->flush();
        f->close();
    };
    auto jpegRead = [](JPEGE_FILE *pFile, uint8_t *pBuf, int32_t len) -> int32_t {
        File *f = reinterpret_cast<File *>(pFile ? pFile->fHandle : nullptr);
        if (!f || !(*f) || !pBuf || len <= 0) return 0;
        return static_cast<int32_t>(f->read(pBuf, static_cast<size_t>(len)));
    };
    auto jpegWrite = [](JPEGE_FILE *pFile, uint8_t *pBuf, int32_t len) -> int32_t {
        File *f = reinterpret_cast<File *>(pFile ? pFile->fHandle : nullptr);
        if (!f || !(*f) || !pBuf || len <= 0) return 0;
        return static_cast<int32_t>(f->write(pBuf, static_cast<size_t>(len)));
    };
    auto jpegSeek = [](JPEGE_FILE *pFile, int32_t pos) -> int32_t {
        File *f = reinterpret_cast<File *>(pFile ? pFile->fHandle : nullptr);
        if (!f || !(*f) || pos < 0) return -1;
        return f->seek(static_cast<uint32_t>(pos)) ? static_cast<int32_t>(f->position()) : -1;
    };

    screenshotJpegPath = path;
    JPEGENC encoder;
    JPEGENCODE state = {};
    int rc = encoder.open(path.c_str(), jpegOpen, jpegClose, jpegRead, jpegWrite, jpegSeek);
    if (rc != JPEGE_SUCCESS) {
        sdMarkFault("captureScreenToJpeg/open");
        sdUnlock();
        errorOut = "jpeg_open";
        return false;
    }

    const int w = tft.width();
    const int h = tft.height();
    rc = encoder.encodeBegin(&state, w, h, JPEGE_PIXEL_RGB565, JPEGE_SUBSAMPLE_420, JPEGE_Q_HIGH);
    if (rc != JPEGE_SUCCESS || state.cx <= 0 || state.cy <= 0) {
        encoder.close();
        if (SD.exists(path)) SD.remove(path);
        sdUnlock();
        errorOut = "jpeg_begin";
        return false;
    }

    const int mcuW = state.cx;
    const int mcuH = state.cy;
    const size_t mcuBytes = static_cast<size_t>(mcuW) * static_cast<size_t>(mcuH) * 2U;
    uint8_t *mcuBuf = reinterpret_cast<uint8_t *>(allocPreferPsram(mcuBytes));
    if (!mcuBuf) {
        encoder.close();
        if (SD.exists(path)) SD.remove(path);
        sdUnlock();
        errorOut = "oom_mcu";
        return false;
    }
    Serial.printf("[JPEG] MCU buffer bytes=%u psram=%d\n",
                  static_cast<unsigned int>(mcuBytes),
                  boardHasUsablePsram() && esp_ptr_external_ram(mcuBuf) ? 1 : 0);

    bool ok = true;
    const unsigned long startedMs = millis();
    for (int y = 0; y < h && ok; y += mcuH) {
        for (int x = 0; x < w && ok; x += mcuW) {
            const int validW = min(mcuW, w - x);
            const int validH = min(mcuH, h - y);
            for (int ry = 0; ry < validH; ry++) {
                uint16_t *dst = reinterpret_cast<uint16_t *>(mcuBuf + static_cast<size_t>(ry) * static_cast<size_t>(mcuW) * 2U);
                tft.readRect(x, y + ry, validW, 1, dst);
                if (validW < mcuW) {
                    const uint16_t edge = dst[validW - 1];
                    for (int xx = validW; xx < mcuW; xx++) dst[xx] = edge;
                }
            }
            if (validH < mcuH) {
                uint16_t *lastRow = reinterpret_cast<uint16_t *>(mcuBuf + static_cast<size_t>(validH - 1) * static_cast<size_t>(mcuW) * 2U);
                for (int ry = validH; ry < mcuH; ry++) {
                    uint16_t *dst = reinterpret_cast<uint16_t *>(mcuBuf + static_cast<size_t>(ry) * static_cast<size_t>(mcuW) * 2U);
                    memcpy(dst, lastRow, static_cast<size_t>(mcuW) * sizeof(uint16_t));
                }
            }
            rc = encoder.addMCU(&state, mcuBuf, mcuW * 2);
            if (rc != JPEGE_SUCCESS) {
                ok = false;
                errorOut = "jpeg_add_mcu";
                break;
            }
            if ((millis() - startedMs) > 6000UL) {
                ok = false;
                errorOut = "timeout";
                break;
            }
        }
        delay(0);
    }

    free(mcuBuf);
    const int outSize = encoder.close();
    screenshotJpegPath = "";
    sdUnlock();

    if (!ok || outSize < 128 || !SD.exists(path)) {
        sdMarkFault("captureScreenToJpeg/write");
        if (SD.exists(path)) SD.remove(path);
        if (errorOut.isEmpty()) errorOut = "encode_failed";
        return false;
    }

    File verify = SD.open(path, FILE_READ);
    const uint32_t sizeOnDisk = verify ? static_cast<uint32_t>(verify.size()) : 0U;
    if (verify) verify.close();
    if (sizeOnDisk < 128U) {
        SD.remove(path);
        errorOut = "verify_failed";
        return false;
    }

    savedPathOut = path;
    return true;
}

bool sendMaybeGz(AsyncWebServerRequest *request, const String &path)
{
    if (!sdEnsureMounted()) return false;
    String gzPath = path + ".gz";
    if (SD.exists(gzPath)) {
        AsyncWebServerResponse *res = request->beginResponse(SD, gzPath, mimeFor(path), false);
        res->addHeader("Content-Encoding", "gzip");
        request->send(res);
        return true;
    }
    if (SD.exists(path)) {
        request->send(SD, path, mimeFor(path), false);
        return true;
    }
    return false;
}

bool sdHasFileOrGz(const String &path)
{
    if (!sdEnsureMounted()) return false;
    return SD.exists(path) || SD.exists(path + ".gz");
}

bool hasPrimaryWebUi()
{
    // Accept fully gzipped or plain web bundles; only an index is required as entrypoint.
    return sdHasFileOrGz("/web/index.html") || sdHasFileOrGz("/web/index.htm");
}

String webIndexPath()
{
    if (sdHasFileOrGz("/web/index.html")) return "/web/index.html";
    if (sdHasFileOrGz("/web/index.htm")) return "/web/index.htm";
    return "";
}

void sendFileFromWebOr404(AsyncWebServerRequest *request, const String &urlPath)
{
    String path = urlPath;
    if (!path.startsWith("/web/") && path != "/web") path = "/web" + path;
    if (sendMaybeGz(request, path)) return;
    request->send(404, "text/plain", "Not found");
}

void setupWebRoutes()
{
    setupCarInputSocket();

    server.on("/version", HTTP_GET, [](AsyncWebServerRequest *request) {
        JsonDocument doc;
        doc["current"] = FW_VERSION;
        String payload;
        serializeJson(doc, payload);
        request->send(200, "application/json", payload);
    });

    server.on("/set_ws_reboot_watchdog", HTTP_GET, [](AsyncWebServerRequest *request) {
        const bool enabled = request->hasParam("value") && request->getParam("value")->value().toInt() != 0;
        wsRebootOnDisconnectEnabled = enabled;
        uiPrefs.begin("ui", false);
        uiPrefs.putBool("ws_reboot", enabled);
        uiPrefs.end();
        JsonDocument doc;
        doc["ok"] = true;
        doc["value"] = enabled ? 1 : 0;
        String payload;
        serializeJson(doc, payload);
        request->send(200, "application/json", payload);
    });

    server.on("/serial/logs", HTTP_GET, [](AsyncWebServerRequest *request) {
        JsonDocument doc;
        JsonArray lines = doc["lines"].to<JsonArray>();
        size_t count = 0;
        if (SERIAL_TERMINAL_TRANSFER_ENABLED) {
            markSerialTerminalStreamingActive();
            const size_t cap = serialLogActiveCapacity();
            count = min(serialLogCount, cap);
            const size_t start = (serialLogHead + cap - count) % cap;
            for (size_t i = 0; i < count; ++i) {
                const size_t idx = (start + i) % cap;
                if (serialLogRing) lines.add(serialLogSlot(idx));
            }
        }
        doc["count"] = static_cast<uint32_t>(count);
        doc["ok"] = true;
        doc["enabled"] = SERIAL_TERMINAL_TRANSFER_ENABLED ? 1 : 0;
        String payload;
        serializeJson(doc, payload);
        request->send(200, "application/json", payload);
    });

    server.on("/serial/command", HTTP_ANY, [](AsyncWebServerRequest *request) {
        String cmd;
        if (request->hasParam("cmd", true)) cmd = request->getParam("cmd", true)->value();
        else if (request->hasParam("cmd")) cmd = request->getParam("cmd")->value();

        JsonDocument doc;
        if (!cmd.length()) {
            doc["ok"] = false;
            doc["error"] = "missing_cmd";
            String payload;
            serializeJson(doc, payload);
            request->send(400, "application/json", payload);
            return;
        }

        if (SERIAL_TERMINAL_TRANSFER_ENABLED) {
            markSerialTerminalStreamingActive();
            serialLogPushLine((String("> ") + cmd).c_str(), true);
            executeSerialCommand(cmd);
        }
        doc["ok"] = true;
        doc["enabled"] = SERIAL_TERMINAL_TRANSFER_ENABLED ? 1 : 0;
        String payload;
        serializeJson(doc, payload);
        request->send(200, "application/json", payload);
    });

    server.on("/ota/upload", HTTP_POST,
              [](AsyncWebServerRequest *request) {
                  OtaUploadCtx *ctx = reinterpret_cast<OtaUploadCtx *>(request->_tempObject);
                  int code = 200;
                  String body = "OK";
                  if (!ctx) {
                      code = 500;
                      body = "OTA context missing";
                  } else if (ctx->error.length()) {
                      code = 500;
                      body = ctx->error;
                  } else if (!ctx->finished) {
                      code = 500;
                      body = "OTA incomplete";
                  } else {
                      body = "Update complete. Rebooting...";
                      rebootRequested = true;
                      rebootRequestedAtMs = millis();
                  }
                  request->send(code, "text/plain", body);
                  if (ctx) {
                      delete ctx;
                      request->_tempObject = nullptr;
                  }
              },
              [](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
                  (void)filename;
                  OtaUploadCtx *ctx = reinterpret_cast<OtaUploadCtx *>(request->_tempObject);
                  if (!ctx) {
                      ctx = new OtaUploadCtx();
                      request->_tempObject = ctx;
                  }

                  if (index == 0) {
                      if (len < 1 || data[0] != 0xE9) {
                          ctx->error = "Invalid firmware format";
                          return;
                      }
                      ctx->valid = true;
                      ctx->started = true;
                      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
                          ctx->error = "OTA begin failed";
                          return;
                      }
                  }

                  if (!ctx->valid || ctx->error.length()) return;

                  if (len && Update.write(data, len) != len) {
                      ctx->error = "OTA write failed";
                      Update.abort();
                      return;
                  }
                  ctx->bytesWritten += len;

                  if (final) {
                      if (Update.end(true)) {
                          ctx->finished = true;
                      } else {
                          ctx->error = "OTA end failed";
                      }
                  }
              });

    server.on("/ota/latest", HTTP_GET, [](AsyncWebServerRequest *request) {
        JsonDocument doc;
        doc["ok"] = false;
        doc["repo"] = "elik745i/ESP32-2432S024C-Remote";
        String tagName;
        String binUrl;
        String error;
        if (!otaFetchLatestReleaseInfo(tagName, binUrl, error)) {
            doc["error"] = error;
            String payload;
            serializeJson(doc, payload);
            request->send(500, "application/json", payload);
            return;
        }

        doc["ok"] = true;
        doc["tag_name"] = tagName;
        doc["bin_url"] = binUrl;
        doc["source"] = "device";
        String payload;
        serializeJson(doc, payload);
        request->send(200, "application/json", payload);
    });

    server.on("/ota/update_from_url", HTTP_ANY, [](AsyncWebServerRequest *request) {
        String url;
        if (request->hasParam("url", true)) url = request->getParam("url", true)->value();
        else if (request->hasParam("url", false)) url = request->getParam("url", false)->value();

        JsonDocument doc;
        if (url.isEmpty()) {
            doc["ok"] = false;
            doc["error"] = "missing_url";
            String payload;
            serializeJson(doc, payload);
            request->send(400, "application/json", payload);
            return;
        }

        String otaErr;
        const bool ok = otaDownloadAndApplyFromUrl(url, otaErr);
        doc["ok"] = ok;
        if (!ok) {
            doc["error"] = otaErr;
            String payload;
            serializeJson(doc, payload);
            request->send(500, "application/json", payload);
            return;
        }

        doc["rebooting"] = true;
        rebootRequested = true;
        rebootRequestedAtMs = millis();
        String payload;
        serializeJson(doc, payload);
        request->send(200, "application/json", payload);
    });

    // Captive portal helper routes.
    server.on("/generate_204", HTTP_GET, [](AsyncWebServerRequest *request) { request->redirect("/"); });
    server.on("/fwlink", HTTP_GET, [](AsyncWebServerRequest *request) { request->redirect("/"); });
    server.on("/hotspot-detect.html", HTTP_GET, [](AsyncWebServerRequest *request) { request->redirect("/"); });
    server.on("/redirect", HTTP_GET, [](AsyncWebServerRequest *request) { request->redirect("/"); });
    server.on("/connecttest.txt", HTTP_GET, [](AsyncWebServerRequest *request) { request->send(200, "text/plain", "OK"); });
    server.on("/ncsi.txt", HTTP_GET, [](AsyncWebServerRequest *request) { request->send(200, "text/plain", "Microsoft NCSI"); });

    server.on("/wifi", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (apModeActive) {
            sendApWifiLandingPage(request);
            return;
        }
        sendWifiSetupPage(request);
    });

    server.on("/listwifi", HTTP_GET, [](AsyncWebServerRequest *request) {
        (void)request;
        JsonDocument doc;
        JsonArray arr = doc.to<JsonArray>();
        WiFi.scanDelete();
        int n = WiFi.scanNetworks(false, true);
        for (int i = 0; i < n; i++) {
            JsonObject it = arr.add<JsonObject>();
            it["ssid"] = WiFi.SSID(i);
            it["rssi"] = WiFi.RSSI(i);
            it["auth"] = authName(WiFi.encryptionType(i));
        }
        String payload;
        serializeJson(doc, payload);
        request->send(200, "application/json", payload);
    });

    server.on("/savewifi", HTTP_POST, [](AsyncWebServerRequest *request) {
        String ssid;
        String pass;
        if (request->hasParam("ssid", true)) ssid = urlDecode(request->getParam("ssid", true)->value());
        else if (request->hasParam("ssid", false)) ssid = urlDecode(request->getParam("ssid", false)->value());
        if (request->hasParam("password", true)) pass = urlDecode(request->getParam("password", true)->value());
        else if (request->hasParam("password", false)) pass = urlDecode(request->getParam("password", false)->value());
        if (ssid.isEmpty()) {
            request->send(400, "text/plain", "missing_ssid");
            return;
        }
        startWifiConnect(ssid, pass);
        request->send(200, "application/json", "{\"ok\":true,\"status\":\"connecting\"}");
    });

    server.on("/list_saved_wifi", HTTP_GET, [](AsyncWebServerRequest *request) {
        (void)request;
        JsonDocument doc;
        JsonArray arr = doc.to<JsonArray>();
        const String ssid = wifiDesiredStaSsid();
        if (ssid.length()) {
            JsonObject it = arr.add<JsonObject>();
            it["ssid"] = ssid;
            it["preferred"] = true;
            it["retry"] = 3;
            it["autoReconnect"] = true;
        }
        String payload;
        serializeJson(doc, payload);
        request->send(200, "application/json", payload);
    });

    server.on("/delete_saved_wifi", HTTP_POST, [](AsyncWebServerRequest *request) {
        String ssid;
        if (request->hasParam("ssid", true)) ssid = urlDecode(request->getParam("ssid", true)->value());
        else if (request->hasParam("ssid", false)) ssid = urlDecode(request->getParam("ssid", false)->value());

        const String current = wifiDesiredStaSsid();
        if (current.isEmpty() || (ssid.length() && !ssid.equalsIgnoreCase(current))) {
            request->send(404, "application/json", "{\"ok\":false,\"error\":\"not_found\"}");
            return;
        }

        pendingSaveCreds = false;
        pendingSaveSsid = "";
        pendingSavePass = "";
        clearStaCreds();
        wifiForgetPendingUi = false;
        WiFi.disconnect(false, false);
        ensureApOnline("web_delete_saved_wifi");
        request->send(200, "application/json", "{\"ok\":true,\"removed\":true}");
    });

    server.on("/connect_saved_wifi", HTTP_ANY, [](AsyncWebServerRequest *request) {
        String ssid;
        if (request->hasParam("ssid", true)) ssid = urlDecode(request->getParam("ssid", true)->value());
        else if (request->hasParam("ssid", false)) ssid = urlDecode(request->getParam("ssid", false)->value());
        if (ssid.isEmpty()) ssid = wifiDesiredStaSsid();
        if (ssid.isEmpty() || !ssid.equalsIgnoreCase(wifiDesiredStaSsid())) {
            request->send(404, "text/plain", "saved_wifi_not_found");
            return;
        }
        startWifiConnect(wifiDesiredStaSsid(), wifiDesiredStaPass());
        request->send(200, "text/plain", "connecting");
    });

    server.on("/wifi_try_connect", HTTP_POST, [](AsyncWebServerRequest *request) {
        String ssid;
        if (request->hasParam("ssid", true)) ssid = urlDecode(request->getParam("ssid", true)->value());
        else if (request->hasParam("ssid", false)) ssid = urlDecode(request->getParam("ssid", false)->value());
        if (ssid.isEmpty() || !ssid.equalsIgnoreCase(wifiDesiredStaSsid())) {
            request->send(404, "text/plain", "saved_wifi_not_found");
            return;
        }
        startWifiConnect(wifiDesiredStaSsid(), wifiDesiredStaPass());
        request->send(200, "text/plain", "connecting");
    });

    server.on("/update_wifi_password", HTTP_POST, [](AsyncWebServerRequest *request) {
        String ssid;
        String pass;
        if (request->hasParam("ssid", true)) ssid = urlDecode(request->getParam("ssid", true)->value());
        else if (request->hasParam("ssid", false)) ssid = urlDecode(request->getParam("ssid", false)->value());
        if (request->hasParam("password", true)) pass = urlDecode(request->getParam("password", true)->value());
        else if (request->hasParam("password", false)) pass = urlDecode(request->getParam("password", false)->value());
        const String current = wifiDesiredStaSsid();
        if (current.isEmpty() || !ssid.equalsIgnoreCase(current)) {
            request->send(404, "text/plain", "saved_wifi_not_found");
            return;
        }
        saveStaCreds(current, pass);
        pendingSaveCreds = false;
        pendingSaveSsid = "";
        pendingSavePass = "";
        request->send(200, "text/plain", "ok");
    });

    server.on("/wifi_set_autoreconnect", HTTP_POST, [](AsyncWebServerRequest *request) {
        (void)request;
        request->send(200, "text/plain", "ok");
    });

    server.on("/update_retry_count", HTTP_GET, [](AsyncWebServerRequest *request) {
        (void)request;
        request->send(200, "text/plain", "ok");
    });

    server.on("/wifi_set_priority", HTTP_POST, [](AsyncWebServerRequest *request) {
        String ssid;
        if (request->hasParam("ssid", true)) ssid = urlDecode(request->getParam("ssid", true)->value());
        else if (request->hasParam("ssid", false)) ssid = urlDecode(request->getParam("ssid", false)->value());
        JsonDocument doc;
        doc["ok"] = ssid.length() && ssid.equalsIgnoreCase(wifiDesiredStaSsid());
        String payload;
        serializeJson(doc, payload);
        request->send(doc["ok"] ? 200 : 404, "application/json", payload);
    });

    server.on("/set_ap_password", HTTP_POST, [](AsyncWebServerRequest *request) {
        String pass;
        if (request->hasParam("ap_pass", true)) pass = urlDecode(request->getParam("ap_pass", true)->value());
        else if (request->hasParam("ap_pass", false)) pass = urlDecode(request->getParam("ap_pass", false)->value());
        pass.trim();

        JsonDocument doc;
        if (pass.isEmpty() || pass.length() < 8 || pass.length() > 63) {
            doc["ok"] = false;
            doc["err"] = "len";
            String payload;
            serializeJson(doc, payload);
            request->send(400, "application/json", payload);
            return;
        }

        saveApCreds(savedApSsid.length() ? savedApSsid : String(AP_SSID), pass);
        doc["ok"] = true;
        String payload;
        serializeJson(doc, payload);
        request->send(200, "application/json", payload);
    });

    server.on("/get_ap_password", HTTP_GET, [](AsyncWebServerRequest *request) {
        JsonDocument doc;
        doc["ok"] = true;
        doc["open"] = savedApPass.isEmpty();
        String payload;
        serializeJson(doc, payload);
        request->send(200, "application/json", payload);
    });

    server.on("/api/wifi/scan", HTTP_GET, [](AsyncWebServerRequest *request) {
        (void)request;
        JsonDocument doc;
        JsonArray arr = doc["networks"].to<JsonArray>();
        WiFi.scanDelete();
        int n = WiFi.scanNetworks(false, true);
        for (int i = 0; i < n; i++) {
            JsonObject it = arr.add<JsonObject>();
            it["ssid"] = WiFi.SSID(i);
            it["rssi"] = WiFi.RSSI(i);
            it["auth"] = authName(WiFi.encryptionType(i));
        }
        String payload;
        serializeJson(doc, payload);
        request->send(200, "application/json", payload);
    });

    server.on("/api/wifi/connect", HTTP_POST, [](AsyncWebServerRequest *request) {
        String ssid;
        String pass;
        if (request->hasParam("ssid", true)) ssid = urlDecode(request->getParam("ssid", true)->value());
        else if (request->hasParam("ssid", false)) ssid = urlDecode(request->getParam("ssid", false)->value());
        if (request->hasParam("pass", true)) pass = urlDecode(request->getParam("pass", true)->value());
        else if (request->hasParam("pass", false)) pass = urlDecode(request->getParam("pass", false)->value());
        if (ssid.isEmpty()) {
            request->send(400, "text/plain", "missing_ssid");
            return;
        }
        startWifiConnect(ssid, pass);
        request->send(200, "text/plain", "connecting");
    });

    server.on("/api/telemetry", HTTP_GET, [](AsyncWebServerRequest *request) {
        sampleTopIndicators();
        JsonDocument doc;
        doc["battery_percent"] = batteryPercent;
        doc["battery_voltage"] = String(batteryVoltage, 2);
        doc["battery_charging"] = batteryCharging;
        doc["light_percent"] = lightPercent;
        bool connected = (WiFi.status() == WL_CONNECTED);
        doc["wifi_connected"] = connected;
        doc["wifi_ssid"] = connected ? WiFi.SSID() : "";
        doc["wifi_rssi"] = connected ? WiFi.RSSI() : 0;
        String payload;
        serializeJson(doc, payload);
        request->send(200, "application/json", payload);
    });

    server.on("/api/chat/messages", HTTP_GET, [](AsyncWebServerRequest *request) {
        JsonDocument doc;
        JsonArray arr = doc["messages"].to<JsonArray>();
        for (int i = 0; i < chatMessageCount; ++i) {
            JsonObject msg = arr.add<JsonObject>();
            msg["author"] = chatDisplayAuthorForMessage(chatMessages[i]);
            msg["text"] = chatMessages[i].text;
            msg["outgoing"] = chatMessages[i].outgoing;
            msg["ts_ms"] = static_cast<uint64_t>(chatMessages[i].tsMs);
        }
        String payload;
        serializeJson(doc, payload);
        request->send(200, "application/json", payload);
    });

    server.on("/api/chat/identity", HTTP_GET, [](AsyncWebServerRequest *request) {
        JsonDocument doc;
        doc["device"] = deviceShortNameValue();
        doc["port"] = P2P_UDP_PORT;
        doc["public_key"] = p2pPublicKeyHex();
        String payload;
        serializeJson(doc, payload);
        request->send(200, "application/json", payload);
    });

    server.on("/api/chat/peers", HTTP_GET, [](AsyncWebServerRequest *request) {
        JsonDocument doc;
        JsonArray arr = doc["peers"].to<JsonArray>();
        for (int i = 0; i < p2pPeerCount; ++i) {
            JsonObject peer = arr.add<JsonObject>();
            peer["name"] = p2pPeers[i].name;
            peer["public_key"] = p2pPeers[i].pubKeyHex;
            peer["ip"] = p2pPeers[i].ip.toString();
            peer["port"] = p2pPeers[i].port;
            peer["enabled"] = p2pPeers[i].enabled;
            peer["last_seen_ms"] = static_cast<uint64_t>(p2pPeers[i].lastSeenMs);
        }
        String payload;
        serializeJson(doc, payload);
        request->send(200, "application/json", payload);
    });

    server.on("/api/chat/discovery", HTTP_GET, [](AsyncWebServerRequest *request) {
        JsonDocument doc;
        JsonArray arr = doc["discovered"].to<JsonArray>();
        for (int i = 0; i < p2pDiscoveredCount; ++i) {
            JsonObject peer = arr.add<JsonObject>();
            peer["name"] = p2pDiscoveredPeers[i].name;
            peer["public_key"] = p2pDiscoveredPeers[i].pubKeyHex;
            peer["ip"] = p2pDiscoveredPeers[i].ip.toString();
            peer["port"] = p2pDiscoveredPeers[i].port;
            peer["trusted"] = p2pDiscoveredPeers[i].trusted;
            peer["last_seen_ms"] = static_cast<uint64_t>(p2pDiscoveredPeers[i].lastSeenMs);
        }
        JsonArray radioArr = doc["radio_discovered"].to<JsonArray>();
        for (int i = 0; i < hc12DiscoveredCount; ++i) {
            JsonObject peer = radioArr.add<JsonObject>();
            peer["name"] = hc12DiscoveredPeers[i].name;
            peer["public_key"] = hc12DiscoveredPeers[i].pubKeyHex;
            peer["last_seen_ms"] = static_cast<uint64_t>(hc12DiscoveredPeers[i].lastSeenMs);
        }
        String payload;
        serializeJson(doc, payload);
        request->send(200, "application/json", payload);
    });

    server.on("/api/chat/discovery/pair", HTTP_POST, [](AsyncWebServerRequest *request) {
        String pubKey;
        if (request->hasParam("public_key", true)) pubKey = urlDecode(request->getParam("public_key", true)->value());
        else if (request->hasParam("public_key", false)) pubKey = urlDecode(request->getParam("public_key", false)->value());
        pubKey.trim();

        JsonDocument doc;
        const int idx = p2pFindDiscoveredByPubKeyHex(pubKey);
        if (idx < 0) {
            doc["ok"] = false;
            doc["error"] = "peer_not_discovered";
            String payload;
            serializeJson(doc, payload);
            request->send(404, "application/json", payload);
            return;
        }

        const bool ok = p2pAddOrUpdateTrustedPeer(
            p2pDiscoveredPeers[idx].name.isEmpty() ? String("Peer") : p2pDiscoveredPeers[idx].name,
            p2pDiscoveredPeers[idx].pubKeyHex,
            p2pDiscoveredPeers[idx].ip,
            p2pDiscoveredPeers[idx].port);
        p2pDiscoveredPeers[idx].trusted = ok;
        doc["ok"] = ok;
        if (!ok) doc["error"] = "pair_failed";
        String payload;
        serializeJson(doc, payload);
        request->send(ok ? 200 : 409, "application/json", payload);
    });

    server.on("/api/chat/peers/add", HTTP_POST, [](AsyncWebServerRequest *request) {
        String name;
        String pubKey;
        String ipText;
        uint16_t port = P2P_UDP_PORT;
        if (request->hasParam("name", true)) name = urlDecode(request->getParam("name", true)->value());
        else if (request->hasParam("name", false)) name = urlDecode(request->getParam("name", false)->value());
        if (request->hasParam("public_key", true)) pubKey = urlDecode(request->getParam("public_key", true)->value());
        else if (request->hasParam("public_key", false)) pubKey = urlDecode(request->getParam("public_key", false)->value());
        if (request->hasParam("ip", true)) ipText = urlDecode(request->getParam("ip", true)->value());
        else if (request->hasParam("ip", false)) ipText = urlDecode(request->getParam("ip", false)->value());
        if (request->hasParam("port", true)) port = static_cast<uint16_t>(request->getParam("port", true)->value().toInt());
        else if (request->hasParam("port", false)) port = static_cast<uint16_t>(request->getParam("port", false)->value().toInt());
        name.trim();
        pubKey.trim();

        JsonDocument doc;
        unsigned char testPk[P2P_PUBLIC_KEY_BYTES] = {0};
        IPAddress ip;
        if (name.isEmpty() || !p2pHexToBytes(pubKey, testPk, sizeof(testPk)) || !ip.fromString(ipText)) {
            doc["ok"] = false;
            doc["error"] = "invalid_peer";
            String payload;
            serializeJson(doc, payload);
            request->send(400, "application/json", payload);
            return;
        }

        const bool ok = p2pAddOrUpdateTrustedPeer(name, pubKey, ip, port);
        doc["ok"] = ok;
        if (!ok) doc["error"] = "peer_list_full";
        String payload;
        serializeJson(doc, payload);
        request->send(ok ? 200 : 409, "application/json", payload);
    });

    server.on("/api/chat/peers/toggle", HTTP_POST, [](AsyncWebServerRequest *request) {
        String pubKey;
        bool enabled = true;
        if (request->hasParam("public_key", true)) pubKey = urlDecode(request->getParam("public_key", true)->value());
        else if (request->hasParam("public_key", false)) pubKey = urlDecode(request->getParam("public_key", false)->value());
        if (request->hasParam("enabled", true)) enabled = request->getParam("enabled", true)->value().toInt() != 0;
        else if (request->hasParam("enabled", false)) enabled = request->getParam("enabled", false)->value().toInt() != 0;
        pubKey.trim();

        JsonDocument doc;
        const int idx = p2pFindPeerByPubKeyHex(pubKey);
        if (idx < 0) {
            doc["ok"] = false;
            doc["error"] = "peer_not_found";
            String payload;
            serializeJson(doc, payload);
            request->send(404, "application/json", payload);
            return;
        }
        p2pPeers[idx].enabled = enabled;
        saveP2pConfig();
        const int didx = p2pFindDiscoveredByPubKeyHex(pubKey);
        if (didx >= 0) p2pDiscoveredPeers[didx].trusted = enabled;
        doc["ok"] = true;
        doc["enabled"] = enabled;
        String payload;
        serializeJson(doc, payload);
        request->send(200, "application/json", payload);
    });

    server.on("/api/chat/peers/remove", HTTP_POST, [](AsyncWebServerRequest *request) {
        String pubKey;
        if (request->hasParam("public_key", true)) pubKey = urlDecode(request->getParam("public_key", true)->value());
        else if (request->hasParam("public_key", false)) pubKey = urlDecode(request->getParam("public_key", false)->value());
        pubKey.trim();

        JsonDocument doc;
        const int idx = p2pFindPeerByPubKeyHex(pubKey);
        if (idx < 0) {
            doc["ok"] = false;
            doc["error"] = "peer_not_found";
            String payload;
            serializeJson(doc, payload);
            request->send(404, "application/json", payload);
            return;
        }
        for (int i = idx + 1; i < p2pPeerCount; ++i) p2pPeers[i - 1] = p2pPeers[i];
        p2pPeerCount--;
        saveP2pConfig();
        const int didx = p2pFindDiscoveredByPubKeyHex(pubKey);
        if (didx >= 0) p2pDiscoveredPeers[didx].trusted = false;
        doc["ok"] = true;
        String payload;
        serializeJson(doc, payload);
        request->send(200, "application/json", payload);
    });

    server.on("/api/chat/send", HTTP_POST, [](AsyncWebServerRequest *request) {
        String text;
        if (request->hasParam("text", true)) text = urlDecode(request->getParam("text", true)->value());
        else if (request->hasParam("text", false)) text = urlDecode(request->getParam("text", false)->value());
        text.trim();

        JsonDocument doc;
        if (text.isEmpty()) {
            doc["ok"] = false;
            doc["error"] = "missing_text";
            String payload;
            serializeJson(doc, payload);
            request->send(400, "application/json", payload);
            return;
        }

        if (currentChatPeerKey.isEmpty()) {
            doc["ok"] = false;
            doc["error"] = "missing_peer";
            String payload;
            serializeJson(doc, payload);
            request->send(400, "application/json", payload);
            return;
        }

        const bool ok = chatSendAndStoreMessage(currentChatPeerKey, text);
        doc["ok"] = ok;
        doc["transport"] = "auto";
        String payload;
        serializeJson(doc, payload);
        request->send(ok ? 200 : 409, "application/json", payload);
    });

    server.on("/list_sd_files", HTTP_GET, [](AsyncWebServerRequest *request) {
        String path = "/";
        if (request->hasParam("path")) path = urlDecode(request->getParam("path")->value());
        path = normalizeSdRoutePath(path, true);
        const bool showSystem = request->hasParam("showSystem") && request->getParam("showSystem")->value().toInt() != 0;
        int start = request->hasParam("start") ? request->getParam("start")->value().toInt() : 0;
        int count = request->hasParam("count") ? request->getParam("count")->value().toInt() : 20;
        if (start < 0) start = 0;
        if (count < 1) count = 20;
        if (count > 128) count = 128;

        JsonDocument doc;
        JsonArray arr = doc.to<JsonArray>();

        if (!sdEnsureMounted() || !sdLock()) {
            String payload;
            serializeJson(arr, payload);
            request->send(200, "application/json", payload);
            return;
        }

        if (!SD.exists(path)) {
            sdUnlock();
            String payload;
            serializeJson(arr, payload);
            request->send(200, "application/json", payload);
            return;
        }

        File dir = SD.open(path);
        if (!dir || !dir.isDirectory()) {
            if (dir) dir.close();
            sdUnlock();
            String payload;
            serializeJson(arr, payload);
            request->send(200, "application/json", payload);
            return;
        }

        int visibleIndex = 0;
        File entry = dir.openNextFile();
        while (entry) {
            String child = entry.name();
            bool isDir = entry.isDirectory();
            uint32_t size = isDir ? 0U : static_cast<uint32_t>(entry.size());
            if (!child.startsWith("/")) child = (path == "/") ? ("/" + child) : (path + "/" + child);
            String name = child;
            int slash = name.lastIndexOf('/');
            if (slash >= 0) name = name.substring(slash + 1);

            const bool hidden = !showSystem && sdShouldHideSystemEntry(name);
            if (!hidden) {
                if (visibleIndex >= start && arr.size() < static_cast<size_t>(count)) {
                    JsonObject obj = arr.add<JsonObject>();
                    obj["name"] = name;
                    obj["isFolder"] = isDir;
                    obj["size"] = size;
                    if (isDir) obj["type"] = "folder";
                    else if (name.endsWith(".js")) obj["type"] = "js";
                    else if (name.endsWith(".html") || name.endsWith(".htm")) obj["type"] = "html";
                    else if (name.endsWith(".txt")) obj["type"] = "txt";
                    else obj["type"] = "default";
                    time_t lastWrite = entry.getLastWrite();
                    if (lastWrite > 0) obj["date"] = static_cast<uint64_t>(lastWrite) * 1000ULL;
                }
                visibleIndex++;
                if (arr.size() >= static_cast<size_t>(count)) {
                    entry.close();
                    break;
                }
            }

            entry.close();
            entry = dir.openNextFile();
        }
        dir.close();
        sdUnlock();

        String payload;
        serializeJson(arr, payload);
        request->send(200, "application/json", payload);
    });

    server.on("/sd_reindex", HTTP_POST, [](AsyncWebServerRequest *request) {
        String path = "/";
        if (request->hasParam("path", true)) path = urlDecode(request->getParam("path", true)->value());
        else if (request->hasParam("path", false)) path = urlDecode(request->getParam("path", false)->value());
        path = normalizeSdRoutePath(path, true);

        JsonDocument doc;
        doc["ok"] = true;
        doc["status"] = "started";
        doc["path"] = path;

        String payload;
        serializeJson(doc, payload);
        request->send(202, "application/json", payload);
    });

    server.on("/sd_reindex_status", HTTP_GET, [](AsyncWebServerRequest *request) {
        (void)request;
        JsonDocument doc;
        doc["pending"] = false;
        doc["path"] = "";
        doc["count"] = 0;
        doc["total"] = 0;
        doc["counting"] = false;

        String payload;
        serializeJson(doc, payload);
        request->send(200, "application/json", payload);
    });

    server.on("/reboot", HTTP_POST, [](AsyncWebServerRequest *request) {
        rebootRequested = true;
        rebootRequestedAtMs = millis();
        request->send(200, "application/json", "{\"ok\":true,\"status\":\"restarting\"}");
    });

    server.on("/download_sd", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (!request->hasParam("path")) {
            request->send(400, "application/json", "{\"ok\":false,\"error\":\"missing_path\"}");
            return;
        }
        String path = normalizeSdRoutePath(urlDecode(request->getParam("path")->value()), false);
        if (!path.length() || !pathAllowedInRecovery(path)) {
            request->send(400, "application/json", "{\"ok\":false,\"error\":\"path_not_allowed\"}");
            return;
        }
        if (!sdEnsureMounted()) {
            request->send(503, "application/json", "{\"ok\":false,\"error\":\"no_sd\"}");
            return;
        }
        if (!SD.exists(path)) {
            request->send(404, "application/json", "{\"ok\":false,\"error\":\"not_found\"}");
            return;
        }
        request->send(SD, path, mimeFor(path), true);
    });

    server.on("/delete_sd", HTTP_POST, [](AsyncWebServerRequest *request) {
        String path;
        if (request->hasParam("path", true)) path = urlDecode(request->getParam("path", true)->value());
        else if (request->hasParam("path", false)) path = urlDecode(request->getParam("path", false)->value());
        path = normalizeSdRoutePath(path, false);

        JsonDocument doc;
        int code = 200;

        if (!path.length()) {
            doc["ok"] = false;
            doc["error"] = "missing_path";
            code = 400;
        } else if (!pathAllowedInRecovery(path) || path == "/recycle") {
            doc["ok"] = false;
            doc["error"] = "path_not_allowed";
            code = 400;
        } else if (mediaIsPlaying || mediaPaused) {
            doc["ok"] = false;
            doc["error"] = "media_busy";
            code = 409;
        } else if (fsWriteBusy()) {
            doc["ok"] = false;
            doc["error"] = "fs_busy";
            code = 409;
        } else if (!sdEnsureMounted()) {
            doc["ok"] = false;
            doc["error"] = "no_sd";
            code = 503;
        } else {
            fsWriteBegin();
            if (!sdLock()) {
                doc["ok"] = false;
                doc["error"] = "sd_lock_failed";
                code = 500;
            } else if (!SD.exists(path)) {
                doc["ok"] = false;
                doc["error"] = "not_found";
                code = 404;
                sdUnlock();
            } else {
                if (!SD.exists("/recycle")) SD.mkdir("/recycle");
                String filename = path.substring(path.lastIndexOf('/') + 1);
                String recyclePath = makeUniquePath("/recycle/" + filename);
                String metaPath = recycleMetaPathFor(recyclePath);
                File meta = SD.open(metaPath, FILE_WRITE);
                if (meta) {
                    meta.print(path);
                    meta.close();
                }
                const bool ok = SD.rename(path, recyclePath);
                if (!ok && SD.exists(metaPath)) SD.remove(metaPath);
                sdUnlock();

                doc["ok"] = ok;
                doc["path"] = path;
                if (ok) doc["recyclePath"] = recyclePath;
                else {
                    doc["error"] = "move_failed";
                    code = 500;
                }
            }
            fsWriteEnd();
        }

        String payload;
        serializeJson(doc, payload);
        request->send(code, "application/json", payload);
    });

    server.on("/permadelete_sd", HTTP_POST, [](AsyncWebServerRequest *request) {
        String path;
        if (request->hasParam("path", true)) path = urlDecode(request->getParam("path", true)->value());
        else if (request->hasParam("path", false)) path = urlDecode(request->getParam("path", false)->value());
        path = normalizeSdRoutePath(path, false);

        JsonDocument doc;
        int code = 200;

        if (!path.length()) {
            doc["ok"] = false;
            doc["error"] = "missing_path";
            code = 400;
        } else if (!pathAllowedInRecovery(path)) {
            doc["ok"] = false;
            doc["error"] = "path_not_allowed";
            code = 400;
        } else if (mediaIsPlaying || mediaPaused) {
            doc["ok"] = false;
            doc["error"] = "media_busy";
            code = 409;
        } else if (fsWriteBusy()) {
            doc["ok"] = false;
            doc["error"] = "fs_busy";
            code = 409;
        } else if (!sdEnsureMounted()) {
            doc["ok"] = false;
            doc["error"] = "no_sd";
            code = 503;
        } else {
            fsWriteBegin();
            if (!sdLock()) {
                doc["ok"] = false;
                doc["error"] = "sd_lock_failed";
                code = 500;
            } else if (!SD.exists(path)) {
                doc["ok"] = false;
                doc["error"] = "not_found";
                code = 404;
                sdUnlock();
            } else {
                const bool ok = deletePathRecursive(path, false);
                if (path.startsWith("/recycle/")) {
                    String metaPath = recycleMetaPathFor(path);
                    if (SD.exists(metaPath)) SD.remove(metaPath);
                }
                sdUnlock();
                doc["ok"] = ok;
                doc["path"] = path;
                if (!ok) {
                    doc["error"] = "delete_failed";
                    code = 500;
                }
            }
            fsWriteEnd();
        }

        String payload;
        serializeJson(doc, payload);
        request->send(code, "application/json", payload);
    });

    server.on("/recover_sd", HTTP_GET, [](AsyncWebServerRequest *request) {
        String name = request->hasParam("name") ? urlDecode(request->getParam("name")->value()) : String("");
        JsonDocument doc;
        int code = 200;

        if (!name.length()) {
            doc["ok"] = false;
            doc["error"] = "missing_name";
            code = 400;
        } else if (!sdEnsureMounted()) {
            doc["ok"] = false;
            doc["error"] = "no_sd";
            code = 503;
        } else if (mediaIsPlaying || mediaPaused) {
            doc["ok"] = false;
            doc["error"] = "media_busy";
            code = 409;
        } else if (fsWriteBusy()) {
            doc["ok"] = false;
            doc["error"] = "fs_busy";
            code = 409;
        } else {
            fsWriteBegin();
            if (!sdLock()) {
                doc["ok"] = false;
                doc["error"] = "sd_lock_failed";
                code = 500;
            } else {
                String recyclePath = "/recycle/" + name;
                String metaPath = recycleMetaPathFor(recyclePath);
                String dst = "/" + name;

                if (!SD.exists(recyclePath)) {
                    doc["ok"] = false;
                    doc["error"] = "not_found";
                    code = 404;
                } else {
                    if (SD.exists(metaPath)) {
                        File meta = SD.open(metaPath, FILE_READ);
                        if (meta) {
                            String orig = meta.readString();
                            meta.close();
                            orig.trim();
                            if (orig.startsWith("/")) dst = orig;
                        }
                    }
                    ensureFolderExists(dst);
                    if (SD.exists(dst)) {
                        doc["ok"] = false;
                        doc["error"] = "dest_exists";
                        doc["dest"] = dst;
                        code = 409;
                    } else {
                        const bool ok = SD.rename(recyclePath, dst);
                        if (ok && SD.exists(metaPath)) SD.remove(metaPath);
                        doc["ok"] = ok;
                        doc["path"] = dst;
                        if (!ok) {
                            doc["error"] = "recover_failed";
                            code = 500;
                        }
                    }
                }
                sdUnlock();
            }
            fsWriteEnd();
        }

        String payload;
        serializeJson(doc, payload);
        request->send(code, "application/json", payload);
    });

    server.on("/create_file", HTTP_POST, [](AsyncWebServerRequest *request) {
        String path;
        if (request->hasParam("path", true)) path = urlDecode(request->getParam("path", true)->value());
        else if (request->hasParam("path", false)) path = urlDecode(request->getParam("path", false)->value());
        path = normalizeSdRoutePath(path, false);

        JsonDocument doc;
        int code = 200;

        if (!path.length()) {
            doc["ok"] = false;
            doc["error"] = "missing_path";
            code = 400;
        } else if (!pathAllowedInRecovery(path)) {
            doc["ok"] = false;
            doc["error"] = "path_not_allowed";
            code = 400;
        } else if (mediaIsPlaying || mediaPaused) {
            doc["ok"] = false;
            doc["error"] = "media_busy";
            code = 409;
        } else if (fsWriteBusy()) {
            doc["ok"] = false;
            doc["error"] = "fs_busy";
            code = 409;
        } else if (!sdEnsureMounted()) {
            doc["ok"] = false;
            doc["error"] = "no_sd";
            code = 503;
        } else {
            fsWriteBegin();
            if (!sdLock()) {
                doc["ok"] = false;
                doc["error"] = "sd_lock_failed";
                code = 500;
            } else {
                String finalPath = makeUniquePath(path);
                ensureFolderExists(finalPath);
                File file = SD.open(finalPath, FILE_WRITE);
                const bool ok = static_cast<bool>(file);
                if (file) file.close();
                sdUnlock();
                doc["ok"] = ok;
                doc["path"] = finalPath;
                if (!ok) {
                    doc["error"] = "create_failed";
                    code = 500;
                }
            }
            fsWriteEnd();
        }

        String payload;
        serializeJson(doc, payload);
        request->send(code, "application/json", payload);
    });

    server.on("/create_folder", HTTP_POST, [](AsyncWebServerRequest *request) {
        String path;
        if (request->hasParam("path", true)) path = urlDecode(request->getParam("path", true)->value());
        else if (request->hasParam("path", false)) path = urlDecode(request->getParam("path", false)->value());
        path = normalizeSdRoutePath(path, false);

        JsonDocument doc;
        int code = 200;

        if (!path.length()) {
            doc["ok"] = false;
            doc["error"] = "missing_path";
            code = 400;
        } else if (!pathAllowedInRecovery(path)) {
            doc["ok"] = false;
            doc["error"] = "path_not_allowed";
            code = 400;
        } else if (mediaIsPlaying || mediaPaused) {
            doc["ok"] = false;
            doc["error"] = "media_busy";
            code = 409;
        } else if (fsWriteBusy()) {
            doc["ok"] = false;
            doc["error"] = "fs_busy";
            code = 409;
        } else if (!sdEnsureMounted()) {
            doc["ok"] = false;
            doc["error"] = "no_sd";
            code = 503;
        } else {
            fsWriteBegin();
            if (!sdLock()) {
                doc["ok"] = false;
                doc["error"] = "sd_lock_failed";
                code = 500;
            } else {
                String finalPath = makeUniquePath(path);
                const bool ok = SD.mkdir(finalPath);
                sdUnlock();
                doc["ok"] = ok;
                doc["path"] = finalPath;
                if (!ok) {
                    doc["error"] = "mkdir_failed";
                    code = 500;
                }
            }
            fsWriteEnd();
        }

        String payload;
        serializeJson(doc, payload);
        request->send(code, "application/json", payload);
    });

    server.on("/upload_sd", HTTP_POST,
              [](AsyncWebServerRequest *request) {
                  FsUploadCtx *ctx = reinterpret_cast<FsUploadCtx *>(request->_tempObject);
                  JsonDocument doc;
                  int code = 200;
                  if (ctx) {
                      if (ctx->file) ctx->file.close();
                      if (ctx->started) {
                          if (!ctx->tmpPath.isEmpty() && sdEnsureMounted() && sdLock()) {
                              if (ctx->error.length()) {
                                  if (SD.exists(ctx->tmpPath)) SD.remove(ctx->tmpPath);
                              } else {
                                  if (SD.exists(ctx->destPath)) {
                                      if (!SD.exists("/recycle")) SD.mkdir("/recycle");
                                      String recyclePath = makeUniquePath("/recycle/" + ctx->destPath.substring(ctx->destPath.lastIndexOf('/') + 1));
                                      String metaPath = recycleMetaPathFor(recyclePath);
                                      File meta = SD.open(metaPath, FILE_WRITE);
                                      if (meta) {
                                          meta.print(ctx->destPath);
                                          meta.close();
                                      }
                                      if (!SD.rename(ctx->destPath, recyclePath) && SD.exists(metaPath)) SD.remove(metaPath);
                                  }
                                  if (!SD.rename(ctx->tmpPath, ctx->destPath)) {
                                      if (SD.exists(ctx->tmpPath)) SD.remove(ctx->tmpPath);
                                      ctx->error = "rename_failed";
                                  }
                              }
                              sdUnlock();
                          } else if (ctx->tmpPath.length()) {
                              ctx->error = "sd_lock_failed";
                          }
                          fsWriteEnd();
                      }
                      if (ctx->error.length()) {
                          code = 400;
                          doc["ok"] = false;
                          doc["error"] = ctx->error;
                      } else {
                          doc["ok"] = true;
                          doc["path"] = ctx->destPath;
                          doc["bytes"] = static_cast<uint32_t>(ctx->bytesWritten);
                      }
                      delete ctx;
                      request->_tempObject = nullptr;
                  } else {
                      code = 500;
                      doc["ok"] = false;
                      doc["error"] = "unknown";
                  }
                  String payload;
                  serializeJson(doc, payload);
                  request->send(code, "application/json", payload);
              },
              [](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
                  FsUploadCtx *ctx = reinterpret_cast<FsUploadCtx *>(request->_tempObject);
                  if (!ctx) {
                      ctx = new FsUploadCtx();
                      request->_tempObject = ctx;
                  }

                  if (index == 0) {
                      if (mediaIsPlaying || mediaPaused) {
                          ctx->error = "media_busy";
                          return;
                      }
                      if (!sdEnsureMounted()) {
                          ctx->error = "no_sd";
                          return;
                      }
                      if (fsWriteBusy()) {
                          ctx->error = "fs_busy";
                          return;
                      }
                      fsWriteBegin();
                      ctx->started = true;

                      String destPath;
                      if (request->hasParam("path", true)) destPath = urlDecode(request->getParam("path", true)->value());
                      else if (request->hasParam("path", false)) destPath = urlDecode(request->getParam("path", false)->value());
                      else destPath = "/" + filename;
                      destPath = normalizeSdRoutePath(destPath, false);
                      ctx->destPath = destPath;

                      if (!ctx->destPath.length() || !pathAllowedInRecovery(ctx->destPath)) {
                          ctx->error = "path_not_allowed";
                          return;
                      }

                      ctx->tmpPath = ctx->destPath + ".upload_tmp";
                      ensureFolderExists(ctx->destPath);
                      if (!sdLock()) {
                          ctx->error = "sd_lock_failed";
                          return;
                      }
                      if (SD.exists(ctx->tmpPath)) SD.remove(ctx->tmpPath);
                      ctx->file = SD.open(ctx->tmpPath, FILE_WRITE);
                      sdUnlock();
                      if (!ctx->file) {
                          ctx->error = "open_failed";
                          return;
                      }
                  }

                  if (!ctx->error.length() && ctx->file && len) {
                      const size_t written = ctx->file.write(data, len);
                      if (written != len) {
                          ctx->error = "write_failed";
                      } else {
                          ctx->bytesWritten += written;
                      }
                  }

                  if (final && ctx->file) {
                      ctx->file.flush();
                      ctx->file.close();
                  }
              });

    server.on("/getsettings", HTTP_GET, [](AsyncWebServerRequest *request) {
        JsonDocument doc;
        doc["darkMode"] = 0;
        doc["horizontalScreen"] = 0;
        doc["holdBucket"] = 0;
        doc["holdAux"] = 0;
        doc["BluepadEnabled"] = 0;
        doc["GamepadEnabled"] = 0;
        doc["ModelRotX"] = 0;
        doc["ModelRotY"] = 0;
        doc["ModelRotZ"] = 0;
        doc["ModelDirX"] = 1;
        doc["ModelDirY"] = 1;
        doc["ModelDirZ"] = 1;
        doc["ModelAxisX"] = "x";
        doc["ModelAxisY"] = "y";
        doc["ModelAxisZ"] = "z";
        appendUiSettings(doc);

        String payload;
        serializeJson(doc, payload);
        request->send(200, "application/json", payload);
    });

    server.on("/get_mqtt", HTTP_GET, [](AsyncWebServerRequest *request) {
        loadMqttConfig();
        JsonDocument doc;
        doc["enable"] = mqttCfg.enabled;
        doc["host"] = mqttCfg.broker;
        doc["port"] = mqttCfg.port;
        doc["user"] = mqttCfg.username;
        doc["pass"] = mqttCfg.password;
        doc["topic_prefix"] = mqttCfg.discoveryPrefix;
        String payload;
        serializeJson(doc, payload);
        request->send(200, "application/json", payload);
    });

    server.on("/set_mqtt", HTTP_POST,
              [](AsyncWebServerRequest *request) {
                  String *body = reinterpret_cast<String *>(request->_tempObject);
                  if (!body) {
                      request->send(400, "text/plain", "Invalid JSON");
                      return;
                  }

                  JsonDocument doc;
                  const DeserializationError err = deserializeJson(doc, *body);
                  delete body;
                  request->_tempObject = nullptr;

                  if (err || !doc.is<JsonObject>()) {
                      request->send(400, "text/plain", "Invalid JSON");
                      return;
                  }

                  JsonObject obj = doc.as<JsonObject>();
                  mqttCfg.enabled = obj["enable"] | false;
                  mqttCfg.broker = String(static_cast<const char *>(obj["host"] | ""));
                  mqttCfg.port = static_cast<uint16_t>(obj["port"] | 1883);
                  mqttCfg.username = String(static_cast<const char *>(obj["user"] | ""));
                  mqttCfg.password = String(static_cast<const char *>(obj["pass"] | ""));
                  mqttCfg.discoveryPrefix = String(static_cast<const char *>(obj["topic_prefix"] | "homeassistant"));
                  if (mqttCfg.port == 0) mqttCfg.port = 1883;
                  if (mqttCfg.discoveryPrefix.isEmpty()) mqttCfg.discoveryPrefix = "homeassistant";
                  mqttChatInboxTopic = "esp32/remote/chat/" + String(MQTT_CHAT_NAMESPACE) + "/inbox/" + p2pPublicKeyHex();
                  saveMqttConfig();
                  mqttDiscoveryPublished = false;
                  mqttLastReconnectMs = 0;
                  if (mqttClient.connected()) mqttClient.disconnect();
                  if (mqttCfg.enabled && wifiConnectedSafe() && !networkSuspendedForAudio) mqttConnectNow();
                  else mqttStatusLine = mqttCfg.enabled ? "Enabled" : "Disabled";
                  request->send(200, "text/plain", "OK");
              },
              nullptr,
              [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
                  String *body = reinterpret_cast<String *>(request->_tempObject);
                  if (index == 0) {
                      body = new String();
                      body->reserve(total + 1);
                      request->_tempObject = body;
                  }
                  if (body && len) body->concat(reinterpret_cast<const char *>(data), len);
              });

    server.on("/mqtt_test", HTTP_POST, [](AsyncWebServerRequest *request) {
        loadMqttConfig();
        if (mqttCfg.broker.isEmpty() || mqttCfg.port == 0) {
            request->send(200, "text/plain", "Invalid host/port");
            return;
        }
        if (!wifiConnectedSafe()) {
            request->send(200, "text/plain", "No WiFi");
            return;
        }

        WiFiClient testNetClient;
        PubSubClient testClient(testNetClient);
        testNetClient.setTimeout(2000);
        testClient.setServer(mqttCfg.broker.c_str(), mqttCfg.port);
        testClient.setSocketTimeout(2);
        String clientId = "MiniExcoTest-" + String(static_cast<uint32_t>(millis()));

        bool connected = false;
        yield();
        if (mqttCfg.username.length()) {
            connected = testClient.connect(clientId.c_str(), mqttCfg.username.c_str(), mqttCfg.password.c_str());
        } else {
            connected = testClient.connect(clientId.c_str());
        }
        yield();
        if (connected) testClient.disconnect();

        request->send(200, "text/plain", connected ? "MQTT connection OK" : "MQTT connection failed");
    });

    server.on("/mqtt_status", HTTP_GET, [](AsyncWebServerRequest *request) {
        JsonDocument doc;
        const bool connected = mqttClient.connected();
        doc["connected"] = connected;
        if (!connected && !mqttStatusLine.isEmpty() && mqttStatusLine != "Connected") {
            doc["last_error"] = mqttStatusLine;
        }
        String payload;
        serializeJson(doc, payload);
        request->send(200, "application/json", payload);
    });

    server.on("/mqtt_discovery", HTTP_POST, [](AsyncWebServerRequest *request) {
        if (mqttClient.connected()) {
            mqttPublishDiscovery();
            request->send(200, "text/plain", "Discovery published");
        } else {
            request->send(200, "text/plain", "MQTT not connected");
        }
    });

    server.on("/get_keymap", HTTP_GET, [](AsyncWebServerRequest *request) {
        JsonDocument doc;
        doc["forward"] = "w";
        doc["backward"] = "s";
        doc["left"] = "a";
        doc["right"] = "d";
        doc["stop"] = " ";
        doc["arm_up"] = " ";
        doc["arm_down"] = " ";
        doc["bucket_up"] = "e";
        doc["bucket_down"] = "q";
        doc["aux_up"] = "r";
        doc["aux_down"] = "f";
        doc["light_toggle"] = "l";
        doc["beacon"] = "b";
        doc["emergency"] = "x";
        doc["horn"] = "h";

        String payload;
        serializeJson(doc, payload);
        request->send(200, "application/json", payload);
    });

    server.on("/camera_status", HTTP_GET, [](AsyncWebServerRequest *request) {
        JsonDocument doc;
        doc["enabled"] = false;
        doc["initialized"] = false;
        doc["boot_restore_pending"] = false;

        String payload;
        serializeJson(doc, payload);
        request->send(200, "application/json", payload);
    });

    server.on("/heap_report", HTTP_GET, [](AsyncWebServerRequest *request) {
        JsonDocument doc;

        const uint32_t heapFree = static_cast<uint32_t>(ESP.getFreeHeap());
        const uint32_t heapTotal = static_cast<uint32_t>(ESP.getHeapSize());
        const uint32_t heapLargest = static_cast<uint32_t>(heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
        const uint32_t heapMinFree = static_cast<uint32_t>(ESP.getMinFreeHeap());

        const uint32_t internalFree = static_cast<uint32_t>(heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
        const uint32_t internalTotal = static_cast<uint32_t>(heap_caps_get_total_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
        const uint32_t internalLargest = static_cast<uint32_t>(heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
        const uint32_t internalMinFree = static_cast<uint32_t>(heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));

        const uint32_t psramFree = static_cast<uint32_t>(heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
        const uint32_t psramTotal = static_cast<uint32_t>(heap_caps_get_total_size(MALLOC_CAP_SPIRAM));
        const uint32_t psramLargest = static_cast<uint32_t>(heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM));
        const uint32_t psramMinFree = static_cast<uint32_t>(heap_caps_get_minimum_free_size(MALLOC_CAP_SPIRAM));

        JsonObject heap = doc["heap"].to<JsonObject>();
        heap["free"] = heapFree;
        heap["total"] = heapTotal;
        heap["largest_free_block"] = heapLargest;
        heap["min_free"] = heapMinFree;
        heap["frag_pct"] = heapFragPercent(heapFree, heapLargest);

        JsonObject internalHeap = doc["internal_heap"].to<JsonObject>();
        internalHeap["free"] = internalFree;
        internalHeap["total"] = internalTotal;
        internalHeap["largest_free_block"] = internalLargest;
        internalHeap["min_free"] = internalMinFree;
        internalHeap["frag_pct"] = heapFragPercent(internalFree, internalLargest);

        JsonObject psram = doc["psram"].to<JsonObject>();
        psram["free"] = psramFree;
        psram["total"] = psramTotal;
        psram["largest_free_block"] = psramLargest;
        psram["min_free"] = psramMinFree;
        psram["frag_pct"] = heapFragPercent(psramFree, psramLargest);

        doc["tasks"].to<JsonArray>();

        String payload;
        serializeJson(doc, payload);
        request->send(200, "application/json", payload);
    });

    server.on("/upload", HTTP_GET, [](AsyncWebServerRequest *request) {
        sendUploadFallbackPage(request);
    });

    server.on("/recovery", HTTP_GET, [](AsyncWebServerRequest *request) {
        sendRecoveryBrowserPage(request);
    });

    server.on("/fs/list", HTTP_GET, [](AsyncWebServerRequest *request) {
        JsonDocument doc;
        JsonObject root = doc.to<JsonObject>();

        String dirPath;
        if (request->hasParam("dir")) dirPath = urlDecode(request->getParam("dir")->value());
        if (!sdEnsureMounted()) {
            root["ok"] = false;
            root["error"] = "no_sd";
        } else if (!sdLock()) {
            root["ok"] = false;
            root["error"] = "sd_lock_failed";
        } else {
            if (dirPath.length()) {
                if (!dirPath.startsWith("/")) dirPath = "/" + dirPath;
                while (dirPath.endsWith("/") && dirPath.length() > 1) dirPath.remove(dirPath.length() - 1);
                if (dirPath != "/" && !pathAllowedInRecovery(dirPath)) {
                    root["ok"] = false;
                    root["error"] = "path_not_allowed";
                } else {
                    JsonArray entries = root["entries"].to<JsonArray>();
                    listSdDirEntries(dirPath, entries);
                    root["ok"] = true;
                    root["dir"] = dirPath;
                }
            } else {
                JsonArray files = root["files"].to<JsonArray>();
                if (SD.exists("/web")) listSdFilesRecursive("/web", files);
                root["ok"] = true;
            }
            sdUnlock();
        }

        String payload;
        serializeJson(doc, payload);
        request->send(root["ok"] == false ? 400 : 200, "application/json", payload);
    });

    server.on("/fs/mkdir", HTTP_POST, [](AsyncWebServerRequest *request) {
        if (mediaIsPlaying || mediaPaused) {
            request->send(409, "application/json", "{\"ok\":false,\"error\":\"media_busy\"}");
            return;
        }
        if (fsWriteBusy()) {
            request->send(409, "application/json", "{\"ok\":false,\"error\":\"fs_busy\"}");
            return;
        }
        fsWriteBegin();
        String path = "/web/new_folder";
        if (request->hasParam("path", true)) path = urlDecode(request->getParam("path", true)->value());
        else if (request->hasParam("path", false)) path = urlDecode(request->getParam("path", false)->value());
        if (!path.startsWith("/")) path = "/" + path;
        while (path.endsWith("/") && path.length() > 1) path.remove(path.length() - 1);

        JsonDocument doc;
        JsonObject root = doc.to<JsonObject>();
        int code = 200;

        if (!sdEnsureMounted()) {
            root["ok"] = false;
            root["error"] = "no_sd";
            code = 503;
        } else if (!pathAllowedInRecovery(path)) {
            root["ok"] = false;
            root["error"] = "path_not_allowed";
            code = 400;
        } else if (!sdLock()) {
            root["ok"] = false;
            root["error"] = "sd_lock_failed";
            code = 500;
        } else {
            bool ok = SD.exists(path) || SD.mkdir(path);
            if (!ok) sdMarkFault("fs_mkdir");
            root["ok"] = ok;
            root["path"] = path;
            if (!ok) {
                root["error"] = "mkdir_failed";
                code = 500;
            }
            sdUnlock();
        }

        String payload;
        serializeJson(doc, payload);
        request->send(code, "application/json", payload);
        fsWriteEnd();
    });

    server.on("/fs/download", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (!request->hasParam("path")) {
            request->send(400, "text/plain", "missing_path");
            return;
        }
        String path = urlDecode(request->getParam("path")->value());
        if (!path.startsWith("/")) path = "/" + path;
        if (!pathAllowedInRecovery(path)) {
            request->send(400, "text/plain", "path_not_allowed");
            return;
        }
        if (!sdEnsureMounted()) {
            request->send(503, "text/plain", "no_sd");
            return;
        }
        if (fsWriteBusy()) {
            request->send(409, "text/plain", "fs_busy");
            return;
        }
        if (!SD.exists(path)) {
            request->send(404, "text/plain", "not_found");
            return;
        }
        request->send(SD, path, mimeFor(path), true);
    });

    server.on("/fs/delete", HTTP_POST, [](AsyncWebServerRequest *request) {
        if (mediaIsPlaying || mediaPaused) {
            request->send(409, "application/json", "{\"ok\":false,\"error\":\"media_busy\"}");
            return;
        }
        if (fsWriteBusy()) {
            request->send(409, "application/json", "{\"ok\":false,\"error\":\"fs_busy\"}");
            return;
        }
        fsWriteBegin();
        String path = "/web";
        if (request->hasParam("path", true)) path = urlDecode(request->getParam("path", true)->value());
        else if (request->hasParam("path", false)) path = urlDecode(request->getParam("path", false)->value());
        if (!path.startsWith("/")) path = "/" + path;
        while (path.endsWith("/") && path.length() > 1) path.remove(path.length() - 1);

        JsonDocument doc;
        JsonObject root = doc.to<JsonObject>();
        int code = 200;

        if (!sdEnsureMounted()) {
            root["ok"] = false;
            root["error"] = "no_sd";
            code = 503;
        } else if (!pathAllowedInRecovery(path)) {
            root["ok"] = false;
            root["error"] = "path_not_allowed";
            code = 400;
        } else if (!sdLock()) {
            root["ok"] = false;
            root["error"] = "sd_lock_failed";
            code = 500;
        } else {
            bool exists = SD.exists(path);
            String lowPath = path;
            lowPath.toLowerCase();
            bool keepRoot = (path == "/web") || (lowPath == "/screenshots");
            bool ok = exists ? deletePathRecursive(path, keepRoot) : true;
            if (path == "/web" && !SD.exists("/web")) SD.mkdir("/web");
            if (pathUnderScreenshots(path) && !SD.exists("/Screenshots")) SD.mkdir("/Screenshots");
            if (!ok) sdMarkFault("fs_delete");
            root["ok"] = ok;
            if (!ok) {
                root["error"] = "delete_failed";
                code = 500;
            }
            sdUnlock();
        }

        String payload;
        serializeJson(doc, payload);
        request->send(code, "application/json", payload);
        fsWriteEnd();
    });

    server.on("/fs/upload", HTTP_POST,
              [](AsyncWebServerRequest *request) {
                  FsUploadCtx *ctx = reinterpret_cast<FsUploadCtx *>(request->_tempObject);
                  int code = 200;
                  String body = "ok";
                  if (ctx) {
                      if (ctx->file) ctx->file.close();
                      if (ctx->started) {
                          if (!ctx->tmpPath.isEmpty() && sdEnsureMounted() && sdLock()) {
                              if (ctx->error.length()) {
                                  if (SD.exists(ctx->tmpPath)) SD.remove(ctx->tmpPath);
                              } else {
                                  if (SD.exists(ctx->destPath)) SD.remove(ctx->destPath);
                                  if (!SD.rename(ctx->tmpPath, ctx->destPath)) {
                                      sdMarkFault("fs_upload_rename");
                                      body = "rename_failed";
                                      code = 500;
                                      if (SD.exists(ctx->tmpPath)) SD.remove(ctx->tmpPath);
                                  }
                              }
                              sdUnlock();
                          } else if (ctx->tmpPath.length()) {
                              body = "sd_lock_failed";
                              code = 500;
                          }
                          fsWriteEnd();
                      }
                      if (ctx->error.length()) {
                          code = 400;
                          body = ctx->error;
                      }
                      delete ctx;
                      request->_tempObject = nullptr;
                  }
                  request->send(code, "text/plain", body);
              },
              [](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
                  FsUploadCtx *ctx = reinterpret_cast<FsUploadCtx *>(request->_tempObject);
                  if (!ctx) {
                      ctx = new FsUploadCtx();
                      request->_tempObject = ctx;
                  }

                  if (index == 0) {
                      if (mediaIsPlaying || mediaPaused) {
                          ctx->error = "media_busy";
                          return;
                      }
                      if (!sdEnsureMounted()) {
                          ctx->error = "no_sd";
                          return;
                      }
                      if (fsWriteBusy()) {
                          ctx->error = "fs_busy";
                          return;
                      }
                      fsWriteBegin();
                      ctx->started = true;
                      String basePath = "/web/";
                      if (request->hasParam("path", true)) basePath = urlDecode(request->getParam("path", true)->value());
                      else if (request->hasParam("path", false)) basePath = urlDecode(request->getParam("path", false)->value());
                      if (!basePath.startsWith("/")) basePath = "/" + basePath;
                      if (!basePath.endsWith("/")) basePath += "/";
                      ctx->destPath = basePath + filename;
                      ctx->tmpPath = ctx->destPath + ".upload_tmp";

                      if (!pathAllowedInRecovery(ctx->destPath)) {
                          ctx->error = "path_not_allowed";
                          return;
                      }

                      ensureFolderExists(ctx->destPath);
                      if (!sdLock()) {
                          ctx->error = "sd_lock_failed";
                          return;
                      }
                      if (SD.exists(ctx->tmpPath)) SD.remove(ctx->tmpPath);
                      ctx->file = SD.open(ctx->tmpPath, FILE_WRITE);
                      sdUnlock();
                      if (!ctx->file) {
                          sdMarkFault("fs_upload_open");
                          ctx->error = "open_failed";
                          return;
                      }
                  }

                  if (!ctx->error.length() && ctx->file && len) {
                      if (ctx->file.write(data, len) != len) {
                          sdMarkFault("fs_upload_write");
                          ctx->error = "write_failed";
                      }
                  }

                  if (final && ctx->file) {
                      ctx->file.flush();
                      ctx->file.close();
                  }
              });

    server.on("/sd_info", HTTP_GET, [](AsyncWebServerRequest *request) {
        JsonDocument doc;
        JsonObject root = doc.to<JsonObject>();
        SdStats s = sdStatsSnapshot();
        root["sd_faults"] = s.faultCount;
        root["sd_remount_attempts"] = s.remountAttempts;
        root["sd_remount_ok"] = s.remountSuccess;
        root["sd_remount_fail"] = s.remountFailure;
        root["sd_try_8mhz"] = s.remountTryFast;
        root["sd_try_4mhz"] = s.remountTryRecovery;
        root["sd_try_1mhz"] = s.remountTrySafe;
        root["sd_force_remounts"] = s.forceRemountCalls;
        root["sd_auto_retries"] = s.autoRetryCalls;
        root["sd_last_fault_ms"] = s.lastFaultMs;
        root["sd_last_remount_ok_ms"] = s.lastRemountOkMs;
        root["sd_last_remount_fail_ms"] = s.lastRemountFailMs;
        root["sd_last_fault_at"] = s.lastFaultWhere[0] ? String(s.lastFaultWhere) : String("");
        if (!sdEnsureMounted()) {
            root["ok"] = false;
            root["error"] = "no_sd";
            String payload;
            serializeJson(doc, payload);
            request->send(503, "application/json", payload);
            return;
        }
        uint64_t total = SD.totalBytes();
        uint64_t used = SD.usedBytes();
        root["ok"] = true;
        root["total"] = total;
        root["used"] = used;
        root["free"] = (total >= used) ? (total - used) : 0;
        String payload;
        serializeJson(doc, payload);
        request->send(200, "application/json", payload);
    });

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        const bool staConnected = (WiFi.status() == WL_CONNECTED);
        const bool bypassApPortal = request->hasParam("skip_ap_portal");
        if (apModeActive && !bypassApPortal) {
            sendApWifiLandingPage(request);
            return;
        }
        if (sdEnsureMounted() && hasPrimaryWebUi()) {
            String idx = webIndexPath();
            if (idx.length() && sendMaybeGz(request, idx)) return;
            // If core UI is expected but index still failed, keep behavior deterministic.
            request->send(500, "text/plain", "Web UI gate passed but index missing");
            return;
        }
        // Connected: go to recovery file manager when SD UI is missing.
        if (staConnected || (apModeActive && bypassApPortal)) {
            request->redirect("/recovery");
            return;
        }
        // Not connected: WiFi setup captive page.
        sendWifiSetupPage(request);
    });

    server.onNotFound([](AsyncWebServerRequest *request) {
        const bool staConnected = (WiFi.status() == WL_CONNECTED);
        if (request->url().startsWith("/api/")) {
            request->send(404, "application/json", "{\"ok\":false,\"error\":\"not_found\"}");
            return;
        }
        if (sdEnsureMounted() && hasPrimaryWebUi()) {
            sendFileFromWebOr404(request, request->url());
            return;
        }
        if (staConnected) {
            // Connected without SD UI -> fallback manager flow.
            if (request->url() == "/upload" || request->url() == "/recovery") return;
            request->redirect("/upload");
            return;
        }
        request->redirect("/wifi");
    });
}

void wifiConnectionService()
{
    if (networkSuspendedForAudio || airplaneModeEnabled) return;
#if defined(BOARD_ESP32S3_3248S035_N16R8)
    if (!wifiRuntimeManaged) return;
    if (!bootWifiInitPending && WiFi.getMode() == WIFI_OFF) return;
#endif

    if (wifiStaGotIpPending) {
        wifiStaGotIpPending = false;
        bootStaConnectInProgress = false;
        const String ssid = wifiSsidSafe();
        const String ip = wifiIpSafe();
        clearWifiScanResults();
        wifiForgetPendingUi = false;
        disableApWhenStaConnected("sta_got_ip");
        if (pendingSaveCreds && ssid == pendingSaveSsid) {
            saveStaCreds(pendingSaveSsid, pendingSavePass);
            pendingSaveCreds = false;
            pendingSaveSsid = "";
            pendingSavePass = "";
        }
        uiStatusLine = "Connected: " + ssid;
        syncInternetTimeIfNeeded(true);
        lvglTopIndicatorStateValid = false;
        if (lvglReady) lvglRefreshTopIndicators();
        if (lvglWifiPwdConnectPending && ssid == lvglWifiPendingSsid) {
            lvglWifiPwdConnectPending = false;
            if (lvglWifiPwdStatusLabel) lv_label_set_text(lvglWifiPwdStatusLabel, "Connected");
            lvglWifiPwdCancelEvent(nullptr);
            lvglOpenScreen(UI_HOME, LV_SCR_LOAD_ANIM_MOVE_RIGHT);
        }
        Serial.printf("[WIFI] STA connected ssid=%s ip=%s rssi=%d\n",
                      ssid.c_str(),
                      ip.c_str(),
                      static_cast<int>(wifiRssiSafe()));
    }

    if (wifiStaDisconnectedPending) {
        const uint8_t reason = wifiStaDisconnectReasonPending;
        wifiStaDisconnectedPending = false;
        bootStaConnectInProgress = false;
        wifiForgetPendingUi = false;
        ensureApOnline("sta_disconnected");
        uiStatusLine = wifiSessionApMode ? "AP mode enabled" : "WiFi disconnected";
        if (lvglWifiPwdConnectPending) {
            lvglWifiPwdConnectPending = false;
            if (lvglWifiPwdStatusLabel) {
                lv_label_set_text_fmt(lvglWifiPwdStatusLabel, "Connection failed: %s",
                                      WiFi.disconnectReasonName(static_cast<wifi_err_reason_t>(reason)));
            }
            if (lvglWifiPwdTa) {
                lv_textarea_set_text(lvglWifiPwdTa, "");
                lv_event_send(lvglWifiPwdTa, LV_EVENT_FOCUSED, nullptr);
            }
        }
        Serial.printf("[WIFI] STA disconnected reason=%u (%s)\n",
                      static_cast<unsigned int>(reason),
                      WiFi.disconnectReasonName(static_cast<wifi_err_reason_t>(reason)));
    }

    if (wifiConnectedSafe()) return;
    if (wifiSessionApMode) {
        if (!apModeActive) ensureApOnline("session_ap_mode");
        return;
    }
    if (!wifiHasStaTarget()) {
        if (!apModeActive) ensureApOnline("no_sta_target");
        return;
    }

    const unsigned long now = millis();
    if (bootStaConnectInProgress) {
        if (static_cast<unsigned long>(now - bootStaConnectStartedMs) >= BOOT_STA_TIMEOUT_MS) {
            bootStaConnectInProgress = false;
            WiFi.disconnect(false, false);
            ensureApOnline("sta_timeout");
            uiStatusLine = "Saved STA reconnect failed";
            if (lvglWifiPwdConnectPending) {
                lvglWifiPwdConnectPending = false;
                if (lvglWifiPwdStatusLabel) lv_label_set_text(lvglWifiPwdStatusLabel, "Connection timed out");
                if (lvglWifiPwdTa) {
                    lv_textarea_set_text(lvglWifiPwdTa, "");
                    lv_event_send(lvglWifiPwdTa, LV_EVENT_FOCUSED, nullptr);
                }
            }
            Serial.printf("[WIFI] STA connect timeout ssid=%s\n", wifiDesiredStaSsid().c_str());
            staLastConnectAttemptMs = now;
        }
        return;
    }

    ensureApOnline("sta_retry_pending");
    if (staLastConnectAttemptMs == 0 || static_cast<unsigned long>(now - staLastConnectAttemptMs) >= STA_RETRY_INTERVAL_MS) {
        beginStaConnectAttempt("retry");
    }
}

void refreshMdnsState()
{
    if (airplaneModeEnabled || networkSuspendedForAudio || !webServerEnabled) {
        if (mdnsStarted) {
            MDNS.end();
            mdnsStarted = false;
        }
        return;
    }
    if (wifiConnectedSafe()) {
        if (!mdnsStarted) {
            if (MDNS.begin(MDNS_HOST)) {
                MDNS.addService("http", "tcp", 80);
                mdnsStarted = true;
                uiStatusLine = "mDNS active: " + String(MDNS_HOST) + ".local";
            }
        }
    } else if (mdnsStarted) {
        MDNS.end();
        mdnsStarted = false;
    }
}

void networkSuspendForAudio()
{
    if (networkSuspendedForAudio) return;

    mqttClient.disconnect();
    stopWebServerRuntime();

    WiFi.softAPdisconnect(true);
    apModeActive = false;
    stopDnsForAp();
    WiFi.disconnect(true, false);
    stopWebServerRuntime();
    WiFi.mode(WIFI_OFF);
    delay(40);
    networkSuspendedForAudio = true;
    bootStaConnectInProgress = false;
    wifiStaGotIpPending = false;
    wifiStaDisconnectedPending = false;
    networkResumeLastAttemptMs = 0;
    Serial.printf("[AUDIO] network suspended for playback, free_heap=%lu largest=%lu\n",
                  static_cast<unsigned long>(ESP.getFreeHeap()),
                  static_cast<unsigned long>(heap_caps_get_largest_free_block(MALLOC_CAP_8BIT)));
}

bool networkResumeAfterAudio()
{
    if (!networkSuspendedForAudio) return true;
    if (mediaIsPlaying || mediaPaused) return false;
    if (uiScreen == UI_MEDIA) return false;
    if (airplaneModeEnabled) return false;

    stopWebServerRuntime();
    WiFi.mode(WIFI_OFF);
    delay(30);
    WiFi.mode(WIFI_STA);
    if (wifiHasStaTarget()) {
        beginStaConnectAttempt("audio_resume");
    } else {
        ensureApOnline("audio_resume_no_sta");
    }
    if (wifiHasStaTarget() || apModeActive) {
        networkSuspendedForAudio = false;
        Serial.printf("[AUDIO] network resume requested free_heap=%lu largest=%lu\n",
                      static_cast<unsigned long>(ESP.getFreeHeap()),
                      static_cast<unsigned long>(heap_caps_get_largest_free_block(MALLOC_CAP_8BIT)));
        return true;
    }
    Serial.printf("[AUDIO] network resume deferred free_heap=%lu largest=%lu\n",
                  static_cast<unsigned long>(ESP.getFreeHeap()),
                  static_cast<unsigned long>(heap_caps_get_largest_free_block(MALLOC_CAP_8BIT)));
    return false;
}

