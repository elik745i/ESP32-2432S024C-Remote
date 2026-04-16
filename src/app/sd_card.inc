bool sdLock(TickType_t timeout = pdMS_TO_TICKS(3000))
{
    if (!sdMutex) return true;
    return xSemaphoreTake(sdMutex, timeout) == pdTRUE;
}

void sdUnlock()
{
    if (sdMutex) xSemaphoreGive(sdMutex);
}

void fsWriteBegin()
{
    portENTER_CRITICAL(&fsWriteMux);
    fsWriteOps++;
    portEXIT_CRITICAL(&fsWriteMux);
}

void fsWriteEnd()
{
    portENTER_CRITICAL(&fsWriteMux);
    if (fsWriteOps > 0) fsWriteOps--;
    portEXIT_CRITICAL(&fsWriteMux);
}

bool fsWriteBusy()
{
    portENTER_CRITICAL(&fsWriteMux);
    uint32_t n = fsWriteOps;
    portEXIT_CRITICAL(&fsWriteMux);
    return n != 0;
}

SdStats sdStatsSnapshot()
{
    SdStats snap;
    portENTER_CRITICAL(&sdStatsMux);
    snap = sdStats;
    portEXIT_CRITICAL(&sdStatsMux);
    return snap;
}

void sdStatsLogSnapshot(const SdStats &s, const char *reason)
{
    const char *tag = (reason && reason[0]) ? reason : "periodic";
    Serial.printf(
        "[SD][stats] reason=%s mounted=%d faults=%lu remount_attempts=%lu remount_ok=%lu remount_fail=%lu try_8m=%lu try_4m=%lu try_1m=%lu force=%lu auto=%lu last_fault_ms=%lu last_ok_ms=%lu last_fail_ms=%lu last_fault_at=%s\n",
        tag,
        sdMounted ? 1 : 0,
        static_cast<unsigned long>(s.faultCount),
        static_cast<unsigned long>(s.remountAttempts),
        static_cast<unsigned long>(s.remountSuccess),
        static_cast<unsigned long>(s.remountFailure),
        static_cast<unsigned long>(s.remountTryFast),
        static_cast<unsigned long>(s.remountTryRecovery),
        static_cast<unsigned long>(s.remountTrySafe),
        static_cast<unsigned long>(s.forceRemountCalls),
        static_cast<unsigned long>(s.autoRetryCalls),
        s.lastFaultMs,
        s.lastRemountOkMs,
        s.lastRemountFailMs,
        s.lastFaultWhere[0] ? s.lastFaultWhere : "-"
    );
}

void sdStatsService()
{
    if (static_cast<unsigned long>(millis() - sdStatsLastLogMs) < SD_STATS_LOG_PERIOD_MS) return;
    sdStatsLastLogMs = millis();
    SdStats snap = sdStatsSnapshot();
    bool changed = (snap.faultCount != sdStatsLastLoggedFaultCount) ||
                   (snap.remountAttempts != sdStatsLastLoggedRemountAttempts) ||
                   (snap.remountSuccess != sdStatsLastLoggedRemountSuccess) ||
                   (snap.remountFailure != sdStatsLastLoggedRemountFailure);
    if (!changed) return;
    sdStatsLastLoggedFaultCount = snap.faultCount;
    sdStatsLastLoggedRemountAttempts = snap.remountAttempts;
    sdStatsLastLoggedRemountSuccess = snap.remountSuccess;
    sdStatsLastLoggedRemountFailure = snap.remountFailure;
    sdStatsLogSnapshot(snap, "periodic");
}

bool sdMountAttemptAtFreq(uint32_t freqHz)
{
    if (SD_CS < 0 || SD_SCK < 0 || SD_MOSI < 0 || SD_MISO < 0) return false;
    if (!sdSpiReady) {
        sdSpi.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
        sdSpiReady = true;
    }
    SD.end();
    delay(2);
    if (!SD.begin(SD_CS, sdSpi, freqHz)) return false;
    if (!SD.exists("/web")) SD.mkdir("/web");
    if (!SD.exists(CHAT_LOG_DIR)) SD.mkdir(CHAT_LOG_DIR);
    return true;
}

bool sdEnsureMountedLocked(bool forceRemount = false)
{
    if (sdMounted && !forceRemount) return true;

    const unsigned long now = millis();
    static unsigned long lastRecoveryAttemptMs = 0;
    if (!forceRemount && static_cast<unsigned long>(now - lastRecoveryAttemptMs) < SD_RECOVERY_MIN_INTERVAL_MS) {
        return sdMounted;
    }
    lastRecoveryAttemptMs = now;

    portENTER_CRITICAL(&sdStatsMux);
    sdStats.remountAttempts++;
    if (forceRemount) sdStats.forceRemountCalls++;
    portEXIT_CRITICAL(&sdStatsMux);

    static const uint32_t freqs[] = {SD_SPI_FREQ_HZ, SD_SPI_FREQ_RECOVERY_HZ, SD_SPI_FREQ_SAFE_HZ};
    for (size_t i = 0; i < (sizeof(freqs) / sizeof(freqs[0])); i++) {
        portENTER_CRITICAL(&sdStatsMux);
        if (freqs[i] == SD_SPI_FREQ_HZ) sdStats.remountTryFast++;
        else if (freqs[i] == SD_SPI_FREQ_RECOVERY_HZ) sdStats.remountTryRecovery++;
        else sdStats.remountTrySafe++;
        portEXIT_CRITICAL(&sdStatsMux);

        if (sdMountAttemptAtFreq(freqs[i])) {
            sdMounted = true;
            const unsigned long okMs = millis();
            portENTER_CRITICAL(&sdStatsMux);
            sdStats.remountSuccess++;
            sdStats.lastRemountOkMs = okMs;
            portEXIT_CRITICAL(&sdStatsMux);
            Serial.printf("[SD] remount ok @ %lu Hz\n", static_cast<unsigned long>(freqs[i]));
            return true;
        }
        delay(SD_RECOVERY_RETRY_DELAY_MS);
    }

    sdMounted = false;
    const unsigned long failMs = millis();
    portENTER_CRITICAL(&sdStatsMux);
    sdStats.remountFailure++;
    sdStats.lastRemountFailMs = failMs;
    portEXIT_CRITICAL(&sdStatsMux);
    Serial.println("[SD] remount failed at all SPI speeds");
    return false;
}

bool sdEnsureMounted(bool forceRemount)
{
    if (sdMounted && !forceRemount) return true;
    if (!sdMounted && !forceRemount) return false;
    if (fsWriteBusy()) return false;
    if (!sdLock()) return false;
    bool ok = sdEnsureMountedLocked(forceRemount);
    sdUnlock();
    return ok;
}

void sdMarkFault(const char *where)
{
    sdMounted = false;
    const unsigned long now = millis();
    portENTER_CRITICAL(&sdStatsMux);
    sdStats.faultCount++;
    sdStats.lastFaultMs = now;
    if (where && where[0]) {
        snprintf(sdStats.lastFaultWhere, sizeof(sdStats.lastFaultWhere), "%s", where);
    } else {
        sdStats.lastFaultWhere[0] = '\0';
    }
    portEXIT_CRITICAL(&sdStatsMux);
    if (where && where[0]) {
        Serial.printf("[SD] I/O failure at %s, scheduling remount\n", where);
    }
}

bool sdMounted = false;
bool sdSpiReady = false;
unsigned long sdLastAutoRetryMs = 0;
bool bootSdInitPending = true;

// Legacy unused SD route helper moved to app/garbage.inc:
// pathUnderConversations(const String&)

bool pathAllowedInRecovery(const String &path)
{
    String p = path;
    if (!p.length()) return false;
    if (!p.startsWith("/")) p = "/" + p;
    return p.startsWith("/");
}

void ensureFolderExists(const String &fullPath)
{
    int lastSlash = fullPath.lastIndexOf('/');
    if (lastSlash <= 0) return;
    String folder = fullPath.substring(0, lastSlash);
    if (!SD.exists(folder)) SD.mkdir(folder);
}

void listSdFilesRecursive(const String &path, JsonArray out)
{
    File dir = SD.open(path);
    if (!dir) return;
    File entry = dir.openNextFile();
    while (entry) {
        String child = entry.name();
        bool isDir = entry.isDirectory();
        uint32_t size = isDir ? 0 : entry.size();
        if (!child.startsWith("/")) child = (path == "/") ? ("/" + child) : (path + "/" + child);

        JsonObject it = out.add<JsonObject>();
        it["path"] = child;
        it["size"] = size;

        entry.close();
        if (isDir) listSdFilesRecursive(child, out);
        entry = dir.openNextFile();
    }
    dir.close();
}

void listSdDirEntries(const String &path, JsonArray out)
{
    File dir = SD.open(path);
    if (!dir || !dir.isDirectory()) {
        if (dir) dir.close();
        return;
    }

    File entry = dir.openNextFile();
    while (entry) {
        String child = entry.name();
        bool isDir = entry.isDirectory();
        uint32_t size = isDir ? 0 : entry.size();
        if (!child.startsWith("/")) child = (path == "/") ? ("/" + child) : (path + "/" + child);
        String name = child;
        int slash = name.lastIndexOf('/');
        if (slash >= 0) name = name.substring(slash + 1);

        JsonObject it = out.add<JsonObject>();
        it["name"] = name;
        it["path"] = child;
        it["isDir"] = isDir;
        it["size"] = size;

        entry.close();
        entry = dir.openNextFile();
    }
    dir.close();
}

String normalizeSdRoutePath(const String &rawPath, bool allowEmptyRoot)
{
    String path = rawPath;
    if (path.isEmpty()) path = allowEmptyRoot ? String("/") : String("");
    if (!path.length()) return path;
    if (!path.startsWith("/")) path = "/" + path;
    while (path.indexOf("//") >= 0) path.replace("//", "/");
    while (path.endsWith("/") && path.length() > 1) path.remove(path.length() - 1);
    return path;
}

bool sdShouldHideSystemEntry(const String &name)
{
    return name.startsWith(".") ||
           name.endsWith(".index") ||
           name.endsWith(".path") ||
           name.endsWith(".bak") ||
           name.endsWith(".meta") ||
           name.equalsIgnoreCase("System Volume Information") ||
           name.startsWith("FOUND.") ||
           name == "Thumbs.db";
}

String makeUniquePath(const String &path)
{
    if (!sdEnsureMounted()) return path;
    if (!SD.exists(path)) return path;

    int slash = path.lastIndexOf('/');
    String dir = (slash > 0) ? path.substring(0, slash) : String("");
    String name = (slash >= 0) ? path.substring(slash + 1) : path;
    int dot = name.lastIndexOf('.');
    String base = (dot > 0) ? name.substring(0, dot) : name;
    String ext = (dot > 0) ? name.substring(dot) : String("");

    for (int n = 1; n < 1000; n++) {
        String candidate = dir + "/" + base + "_" + String(n) + ext;
        if (!SD.exists(candidate)) return candidate;
    }
    return path;
}

String recycleMetaPathFor(const String &recyclePath)
{
    return recyclePath + ".path";
}

void setupSd()
{
    if (SD_CS < 0 || SD_SCK < 0 || SD_MOSI < 0 || SD_MISO < 0) {
        sdMounted = false;
        return;
    }
    if (!sdLock(pdMS_TO_TICKS(40))) {
        Serial.println("SD boot probe skipped: lock timeout");
        return;
    }
    const bool ok = sdMountAttemptAtFreq(SD_BOOT_PROBE_FREQ_HZ);
    sdMounted = ok;
    sdUnlock();
    if (ok) Serial.printf("SD mounted (boot probe @ %lu Hz)\n", static_cast<unsigned long>(SD_BOOT_PROBE_FREQ_HZ));
    else Serial.printf("SD mount deferred (boot probe failed @ %lu Hz)\n", static_cast<unsigned long>(SD_BOOT_PROBE_FREQ_HZ));
    if (ok) chatLoadPendingOutbox();
}
