static void batteryCalibrationLoad()
{
    batteryPrefs.begin("battery", true);
    batteryCalState.observedFullRawV = batteryPrefs.getFloat("cal_full", 0.0f);
    batteryCalState.observedEmptyRawV = batteryPrefs.getFloat("cal_empty", 0.0f);
    batteryPrefs.end();
    batteryCalState.hasFullAnchor = batteryCalState.observedFullRawV > 0.0f;
    batteryCalState.hasEmptyAnchor = batteryCalState.observedEmptyRawV > 0.0f;
}

static void batteryCalibrationSave()
{
    batteryPrefs.begin("battery", false);
    if (batteryCalState.hasFullAnchor) batteryPrefs.putFloat("cal_full", batteryCalState.observedFullRawV);
    else batteryPrefs.remove("cal_full");
    if (batteryCalState.hasEmptyAnchor) batteryPrefs.putFloat("cal_empty", batteryCalState.observedEmptyRawV);
    else batteryPrefs.remove("cal_empty");
    batteryPrefs.end();
}

static void batteryCalibrationReset()
{
    batteryCalState.hasFullAnchor = false;
    batteryCalState.hasEmptyAnchor = false;
    batteryCalState.observedFullRawV = 0.0f;
    batteryCalState.observedEmptyRawV = 0.0f;
    batteryCalibrationSave();
}

static void batteryTrainingLoad()
{
    batteryPrefs.begin("battery", true);
    batteryTrainingState.active = batteryPrefs.getBool("train_on", false);
    batteryTrainingState.powerOverrideActive = batteryPrefs.getBool("train_ovr", false);
    batteryTrainingState.autoCalibrationEnabled = batteryPrefs.getBool("auto_en", false);
    batteryTrainingState.phase = static_cast<BatteryTrainingPhase>(batteryPrefs.getUChar("train_ph", static_cast<uint8_t>(BATTERY_TRAIN_IDLE)));
    if (batteryTrainingState.phase > BATTERY_TRAIN_DISCHARGE) batteryTrainingState.phase = BATTERY_TRAIN_IDLE;
    batteryTrainingState.savedPowerOffMs = batteryPrefs.getULong("train_pwr", 0UL);
    batteryPrefs.end();
    if (!batteryTrainingState.active) batteryTrainingState.phase = BATTERY_TRAIN_IDLE;
    if (batteryTrainingState.powerOverrideActive) powerOffIdleTimeoutMs = 0UL;
}

static void batteryTrainingSave()
{
    batteryPrefs.begin("battery", false);
    batteryPrefs.putBool("train_on", batteryTrainingState.active);
    batteryPrefs.putBool("train_ovr", batteryTrainingState.powerOverrideActive);
    batteryPrefs.putBool("auto_en", batteryTrainingState.autoCalibrationEnabled);
    batteryPrefs.putUChar("train_ph", static_cast<uint8_t>(batteryTrainingState.phase));
    batteryPrefs.putULong("train_pwr", batteryTrainingState.savedPowerOffMs);
    batteryPrefs.end();
}

static const char *batteryTrainingPhaseLabel()
{
    if (!batteryTrainingState.active) {
        return batteryTrainingState.autoCalibrationEnabled ? "Auto Calibration" : "Manual Calibration";
    }
    switch (batteryTrainingState.phase) {
        case BATTERY_TRAIN_CHARGE: return "Charge Training";
        case BATTERY_TRAIN_DISCHARGE: return "Discharge Training";
        default: return "Manual Calibration";
    }
}

static void batteryTrainingRestorePowerOff()
{
    if (!batteryTrainingState.powerOverrideActive) return;
    powerOffIdleTimeoutMs = POWER_OFF_IDLE_TIMEOUT_OPTIONS_MS[powerOffTimeoutOptionIndex(batteryTrainingState.savedPowerOffMs)];
    batteryTrainingState.powerOverrideActive = false;
    uiPrefs.begin("ui", false);
    uiPrefs.putULong("pwr_idle", powerOffIdleTimeoutMs);
    uiPrefs.end();
    applyDisplayIdleTimeoutPowerOffCap(true);
    batteryTrainingSave();
    if (lvglReady) lvglRefreshStyleUi();
}

static void batteryTrainingStartCharge()
{
    if (!batteryTrainingState.powerOverrideActive) batteryTrainingState.savedPowerOffMs = powerOffIdleTimeoutMs;
    powerOffIdleTimeoutMs = 0UL;
    batteryTrainingState.active = true;
    batteryTrainingState.autoCalibrationEnabled = false;
    batteryTrainingState.phase = BATTERY_TRAIN_CHARGE;
    batteryTrainingState.powerOverrideActive = true;
    uiPrefs.begin("ui", false);
    uiPrefs.putULong("pwr_idle", powerOffIdleTimeoutMs);
    uiPrefs.end();
    batteryTrainingSave();
    if (lvglReady) {
        lvglRefreshStyleUi();
        lvglRefreshBatteryTrainUi();
    }
}

static void batteryTrainingStartDischarge()
{
    if (!batteryTrainingState.powerOverrideActive) batteryTrainingState.savedPowerOffMs = powerOffIdleTimeoutMs;
    powerOffIdleTimeoutMs = 0UL;
    batteryTrainingState.active = true;
    batteryTrainingState.autoCalibrationEnabled = false;
    batteryTrainingState.phase = BATTERY_TRAIN_DISCHARGE;
    batteryTrainingState.powerOverrideActive = true;
    uiPrefs.begin("ui", false);
    uiPrefs.putULong("pwr_idle", powerOffIdleTimeoutMs);
    uiPrefs.end();
    batteryTrainingSave();
    if (lvglReady) {
        lvglRefreshStyleUi();
        lvglRefreshBatteryTrainUi();
    }
}

static void batteryTrainingStopManual()
{
    batteryTrainingRestorePowerOff();
    batteryTrainingState.active = false;
    batteryTrainingState.phase = BATTERY_TRAIN_IDLE;
    batteryTrainingSave();
    if (lvglReady) lvglRefreshBatteryTrainUi();
}

static float batteryBlendObserved(float current, float observed)
{
    if (current <= 0.0f) return observed;
    return current + BATTERY_CAL_BLEND_ALPHA * (observed - current);
}

static void batteryCalibrationLearnFull(float rawV)
{
    if (rawV <= 0.0f) return;
    const float next = batteryBlendObserved(batteryCalState.observedFullRawV, rawV);
    if (batteryCalState.hasFullAnchor && fabsf(next - batteryCalState.observedFullRawV) < 0.003f) return;
    batteryCalState.observedFullRawV = next;
    batteryCalState.hasFullAnchor = true;
    batteryCalibrationSave();
}

static void batteryCalibrationSetFull(float rawV)
{
    if (rawV <= 0.0f) return;
    batteryCalState.observedFullRawV = rawV;
    batteryCalState.hasFullAnchor = true;
    batteryCalibrationSave();
}

static void batteryCalibrationForceFull(float rawV)
{
    if (rawV <= 0.0f) return;
    batteryCalState.observedFullRawV = rawV;
    batteryCalState.hasFullAnchor = true;

    // Rebuild the empty anchor onto the same scale so manual FULL maps the
    // current reading to 4.20V immediately instead of being diluted by older data.
    const float scale = BATTERY_FULL_V / rawV;
    if (scale > 0.0f && batteryCalState.hasEmptyAnchor) {
        batteryCalState.observedEmptyRawV = BATTERY_EMPTY_V / scale;
    }
    batteryCalibrationSave();
}

static void batteryCalibrationLearnEmpty(float rawV)
{
    if (rawV <= 0.0f) return;
    const float next = batteryBlendObserved(batteryCalState.observedEmptyRawV, rawV);
    if (batteryCalState.hasEmptyAnchor && fabsf(next - batteryCalState.observedEmptyRawV) < 0.003f) return;
    batteryCalState.observedEmptyRawV = next;
    batteryCalState.hasEmptyAnchor = true;
    batteryCalibrationSave();
}

static float batteryCalibratedVoltageFromRaw(float rawV)
{
    if (rawV <= 0.0f) return 0.0f;

    if (batteryCalState.hasFullAnchor && batteryCalState.hasEmptyAnchor) {
        const float observedSpan = batteryCalState.observedFullRawV - batteryCalState.observedEmptyRawV;
        if (observedSpan >= BATTERY_CAL_MIN_SPAN_V) {
            const float ratio = (rawV - batteryCalState.observedEmptyRawV) / observedSpan;
            return BATTERY_EMPTY_V + ratio * (BATTERY_FULL_V - BATTERY_EMPTY_V);
        }
    }

    if (batteryCalState.hasFullAnchor && batteryCalState.observedFullRawV > 0.0f) {
        return rawV * (BATTERY_FULL_V / batteryCalState.observedFullRawV);
    }

    if (batteryCalState.hasEmptyAnchor && batteryCalState.observedEmptyRawV > 0.0f) {
        return rawV * (BATTERY_EMPTY_V / batteryCalState.observedEmptyRawV);
    }

    return rawV * BATTERY_CAL_FACTOR;
}

static float batteryFilterSample(float calibratedV)
{
    if (calibratedV <= 0.0f) return calibratedV;

    batteryVoltageMedianWindow[batteryVoltageMedianIndex] = calibratedV;
    batteryVoltageMedianIndex = static_cast<uint8_t>((batteryVoltageMedianIndex + 1U) % BATTERY_MEDIAN_WINDOW);
    if (batteryVoltageMedianCount < BATTERY_MEDIAN_WINDOW) batteryVoltageMedianCount++;

    float sorted[BATTERY_MEDIAN_WINDOW];
    for (uint8_t i = 0; i < batteryVoltageMedianCount; ++i) sorted[i] = batteryVoltageMedianWindow[i];
    for (uint8_t i = 1; i < batteryVoltageMedianCount; ++i) {
        const float key = sorted[i];
        int8_t j = static_cast<int8_t>(i) - 1;
        while (j >= 0 && sorted[j] > key) {
            sorted[j + 1] = sorted[j];
            --j;
        }
        sorted[j + 1] = key;
    }
    const float median = sorted[batteryVoltageMedianCount / 2U];

    if (!batteryFilterInitialized) {
        batteryVoltage = median;
        batteryFilterInitialized = true;
        return batteryVoltage;
    }

    const float delta = median - batteryVoltage;
    float alpha = delta >= 0.0f ? BATTERY_FILTER_ALPHA_RISE : BATTERY_FILTER_ALPHA_FALL;
    if (fabsf(delta) >= BATTERY_FILTER_FAST_DELTA_V) alpha = BATTERY_FILTER_FAST_ALPHA;
    batteryVoltage += alpha * delta;
    return batteryVoltage;
}

float batteryCalibrationFactor()
{
    if (batteryRawVoltage > 0.0f) {
        const float calibrated = batteryCalibratedVoltageFromRaw(batteryRawVoltage);
        if (calibrated > 0.0f) return calibrated / batteryRawVoltage;
    }
    if (batteryCalState.hasFullAnchor && batteryCalState.observedFullRawV > 0.0f) {
        return BATTERY_FULL_V / batteryCalState.observedFullRawV;
    }
    if (batteryCalState.hasEmptyAnchor && batteryCalState.observedEmptyRawV > 0.0f) {
        return BATTERY_EMPTY_V / batteryCalState.observedEmptyRawV;
    }
    return BATTERY_CAL_FACTOR;
}

float readBatteryVoltage()
{
    if (BATTERY_ADC_PIN < 0) return 0.0f;
    uint32_t mvSum = 0;
    uint32_t rawSum = 0;
    for (int i = 0; i < BATTERY_ADC_SAMPLES; i++) {
        for (int settle = 0; settle < BATTERY_ADC_SETTLE_READS; ++settle) {
            (void)analogReadMilliVolts(BATTERY_ADC_PIN);
            delayMicroseconds(BATTERY_ADC_SETTLE_US);
        }
        mvSum += analogReadMilliVolts(BATTERY_ADC_PIN);
        rawSum += static_cast<uint32_t>(analogRead(BATTERY_ADC_PIN));
        delayMicroseconds(BATTERY_ADC_SETTLE_US);
    }
    const uint32_t mv = mvSum / BATTERY_ADC_SAMPLES;
    const uint32_t raw = rawSum / BATTERY_ADC_SAMPLES;
    float pinV = static_cast<float>(mv) / 1000.0f;
    if (pinV <= 0.001f && raw > 0U) {
        pinV = (static_cast<float>(raw) / 4095.0f) * BATTERY_ADC_FALLBACK_REF_V;
    }
    const float ratio = (BATTERY_DIVIDER_R_TOP + BATTERY_DIVIDER_R_BOTTOM) / BATTERY_DIVIDER_R_BOTTOM;
    return pinV * ratio;
}

uint8_t batteryPercentFromVoltage(float vbat)
{
    float p = (vbat - BATTERY_EMPTY_V) * 100.0f / (BATTERY_FULL_V - BATTERY_EMPTY_V);
    if (p < 0.0f) p = 0.0f;
    if (p > 100.0f) p = 100.0f;
    return static_cast<uint8_t>(p + 0.5f);
}

void cpuLoadService(uint32_t loopStartUs)
{
    if (cpuLoadPrevLoopStartUs != 0U) {
        const uint32_t loopTotalUs = loopStartUs - cpuLoadPrevLoopStartUs;
        const uint32_t loopActiveUs = (cpuLoadPrevActiveUs > loopTotalUs) ? loopTotalUs : cpuLoadPrevActiveUs;
        cpuLoadAccumActiveUs += loopActiveUs;
        cpuLoadAccumTotalUs += loopTotalUs;
    }
    cpuLoadPrevLoopStartUs = loopStartUs;

    const unsigned long nowMs = millis();
    if (cpuLoadWindowStartMs == 0UL) cpuLoadWindowStartMs = nowMs;
    if ((nowMs - cpuLoadWindowStartMs) < 1000UL) return;

    uint8_t nextLoad = cpuLoadPercent;
    if (cpuLoadAccumTotalUs > 0U) {
        const uint32_t pct = (cpuLoadAccumActiveUs * 100U) / cpuLoadAccumTotalUs;
        nextLoad = static_cast<uint8_t>(pct > 100U ? 100U : pct);
    }
    cpuLoadPercent = nextLoad;
    cpuLoadAccumActiveUs = 0U;
    cpuLoadAccumTotalUs = 0U;
    cpuLoadWindowStartMs = nowMs;
}

uint16_t readLightRaw()
{
    if (LIGHT_ADC_PIN < 0) return 0;
    uint32_t sum = 0;
    for (int i = 0; i < LIGHT_ADC_SAMPLES; i++) {
        sum += analogRead(LIGHT_ADC_PIN);
    }
    uint32_t raw = sum / LIGHT_ADC_SAMPLES;
    if (raw > 4095) raw = 4095;
    return static_cast<uint16_t>(raw);
}

uint8_t lightPercentFromRaw(uint16_t raw)
{
    // Track observed raw range to auto-calibrate high-impedance light dividers.
    if (raw < lightMinObserved) lightMinObserved = raw;
    if (raw > lightMaxObserved) lightMaxObserved = raw;

    int span = static_cast<int>(lightMaxObserved) - static_cast<int>(lightMinObserved);
    float p = 0.0f;
    if (span >= LIGHT_MIN_SPAN_RAW) {
        p = (static_cast<float>(raw - lightMinObserved) * 100.0f) / static_cast<float>(span);
    } else {
        // Fallback for very small-range/high-impedance circuits.
        int calSpan = static_cast<int>(LIGHT_RAW_CAL_MAX) - static_cast<int>(LIGHT_RAW_CAL_MIN);
        if (calSpan < 1) calSpan = 1;
        int shifted = static_cast<int>(raw) - static_cast<int>(LIGHT_RAW_CAL_MIN);
        if (shifted < 0) shifted = 0;
        if (shifted > calSpan) shifted = calSpan;
        p = (static_cast<float>(shifted) * 100.0f) / static_cast<float>(calSpan);
    }

    if (LIGHT_INVERT) p = 100.0f - p;
    if (p < 0.0f) p = 0.0f;
    if (p > 100.0f) p = 100.0f;
    return static_cast<uint8_t>(p + 0.5f);
}

void sampleTopIndicators()
{
    const unsigned long now = millis();
    if (BATTERY_ADC_PIN >= 0) {
        batteryRawVoltage = readBatteryVoltage();
        const float rawVoltage = batteryCalibratedVoltageFromRaw(batteryRawVoltage);
        batteryVoltage = batteryFilterSample(rawVoltage);
        batteryPercent = batteryPercentFromVoltage(batteryVoltage);
    } else {
        batteryRawVoltage = 0.0f;
        batteryVoltage = 0.0f;
        batteryPercent = 0;
        batteryCharging = false;
        chargeFilterInitialized = false;
        chargeSessionActive = false;
    }

    if (BATTERY_ADC_PIN < 0) {
        if (LIGHT_ADC_PIN >= 0) {
            lightRawAdc = readLightRaw();
            const uint8_t rawLightPercent = lightPercentFromRaw(lightRawAdc);
            if (!lightFilterInitialized) {
                lightPercentFiltered = rawLightPercent;
                lightFilterInitialized = true;
            } else {
                lightPercentFiltered += LIGHT_FILTER_ALPHA * (static_cast<float>(rawLightPercent) - lightPercentFiltered);
            }
            lightPercent = static_cast<uint8_t>(lightPercentFiltered + 0.5f);
        } else {
            lightRawAdc = 0;
            lightPercent = 0;
            lightPercentFiltered = 0.0f;
            lightFilterInitialized = true;
        }
        return;
    }

    if (!chargeFilterInitialized) {
        chargeFilteredVoltage = batteryVoltage;
        chargeFilterInitialized = true;
    } else {
        chargeFilteredVoltage += CHARGE_FILTER_ALPHA * (batteryVoltage - chargeFilteredVoltage);
    }

    if (lastChargeEvalMs == 0) {
        lastChargeEvalMs = now;
        chargePrevVoltage = chargeFilteredVoltage;
    } else if (now - lastChargeEvalMs >= CHARGE_DETECT_INTERVAL_MS) {
        const float dv = chargeFilteredVoltage - chargePrevVoltage;
        chargePrevVoltage = chargeFilteredVoltage;
        lastChargeEvalMs = now;

        if (dv > CHARGE_RISE_THRESHOLD_V) {
            if (chargeTrendScore < CHARGE_SCORE_MAX) chargeTrendScore++;
        } else if (dv < -CHARGE_RISE_THRESHOLD_V) {
            if (chargeTrendScore > -CHARGE_SCORE_MAX) chargeTrendScore--;
        } else {
            if (chargeTrendScore > 0) chargeTrendScore--;
            else if (chargeTrendScore < 0) chargeTrendScore++;
        }
        if (chargeTrendScore >= CHARGE_SCORE_ON) lastChargeSeenMs = now;
        batteryCharging = chargeTrendScore >= CHARGE_SCORE_ON ||
                          (lastChargeSeenMs != 0 && (now - lastChargeSeenMs) <= CHARGE_HOLD_MS);

        if (CHARGE_LOG_TO_SERIAL) {
            Serial.printf("[CHG] raw=%.3f V=%.3f Vf=%.3f dv=%.4f score=%d charging=%d cal=%.4f\n",
                          batteryRawVoltage, batteryVoltage, chargeFilteredVoltage, dv, chargeTrendScore, batteryCharging ? 1 : 0,
                          batteryCalibrationFactor());
        }
    }

    if (batteryCharging) {
        if (!chargeSessionActive) {
            chargeSessionActive = true;
            chargeSessionStartMs = now;
            chargePlateauStartMs = 0;
            chargeSessionStartRawVoltage = batteryRawVoltage;
            chargeSessionMaxRawVoltage = batteryRawVoltage;
        } else {
            if (batteryRawVoltage > chargeSessionMaxRawVoltage) chargeSessionMaxRawVoltage = batteryRawVoltage;
        }

        if ((chargeSessionMaxRawVoltage - batteryRawVoltage) <= CHARGE_CAL_PLATEAU_BAND_V) {
            if (chargePlateauStartMs == 0) chargePlateauStartMs = now;
        } else {
            chargePlateauStartMs = 0;
        }

        const bool sessionLongEnough = (now - chargeSessionStartMs) >= CHARGE_CAL_SESSION_MIN_MS;
        const bool sessionRoseEnough = (chargeSessionMaxRawVoltage - chargeSessionStartRawVoltage) >= CHARGE_CAL_SESSION_MIN_RISE_V;
        const bool plateauLongEnough = chargePlateauStartMs != 0 && (now - chargePlateauStartMs) >= CHARGE_CAL_PLATEAU_HOLD_MS;
        if (batteryTrainingState.autoCalibrationEnabled &&
            sessionLongEnough && sessionRoseEnough && plateauLongEnough &&
            chargeSessionMaxRawVoltage >= CHARGE_CAL_FULL_MIN_RAW_V) {
            batteryCalibrationLearnFull(chargeSessionMaxRawVoltage);
        }
    } else if (chargeSessionActive) {
        chargeSessionActive = false;
        chargeSessionStartMs = 0;
        chargePlateauStartMs = 0;
        chargeSessionStartRawVoltage = 0.0f;
        chargeSessionMaxRawVoltage = 0.0f;
    }

    if (LIGHT_ADC_PIN >= 0) {
        lightRawAdc = readLightRaw();
        const uint8_t rawLightPercent = lightPercentFromRaw(lightRawAdc);
        if (!lightFilterInitialized) {
            lightPercentFiltered = rawLightPercent;
            lightFilterInitialized = true;
        } else {
            lightPercentFiltered += LIGHT_FILTER_ALPHA * (static_cast<float>(rawLightPercent) - lightPercentFiltered);
        }
        lightPercent = static_cast<uint8_t>(lightPercentFiltered + 0.5f);

        if (LIGHT_LOG_RAW_TO_SERIAL && millis() - lastLightLogMs >= 2000) {
            lastLightLogMs = millis();
            Serial.printf("[LIGHT] raw=%u min=%u max=%u pct=%u\n",
                          lightRawAdc, lightMinObserved, lightMaxObserved, lightPercent);
        }
    } else {
        lightRawAdc = 0;
        lightPercent = 0;
        lightPercentFiltered = 0.0f;
        lightFilterInitialized = true;
    }
}
