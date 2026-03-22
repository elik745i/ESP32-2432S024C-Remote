void p2pEnsureUdp()
{
    if (!p2pReady) return;
    if (p2pUdpStarted) return;
    if (WiFi.getMode() == WIFI_OFF) return;
    if (!wifiLanAvailableSafe()) return;
    if (p2pUdp.begin(P2P_UDP_PORT) == 1) {
        p2pUdpStarted = true;
#if defined(BOARD_ESP32S3_3248S035_N16R8)
        p2pUdpLastParseError = 0;
        p2pUdpLastParseErrorMs = 0;
#endif
        Serial.printf("[P2P] UDP listening on %u\n", static_cast<unsigned int>(P2P_UDP_PORT));
    }
}

static bool p2pSendPacketToPeer(const P2PPeer &peer, uint8_t type, const uint8_t *plain, size_t plainLen)
{
    if (!peer.enabled || plainLen == 0 || plainLen > (P2P_MAX_PACKET - 64)) return false;
    unsigned char peerPk[P2P_PUBLIC_KEY_BYTES] = {0};
    if (!p2pHexToBytes(peer.pubKeyHex, peerPk, sizeof(peerPk))) return false;

    unsigned char nonce[P2P_NONCE_BYTES] = {0};
    randombytes_buf(nonce, sizeof(nonce));
    const size_t cipherLen = plainLen + P2P_MAC_BYTES;
    unsigned char cipher[P2P_MAX_PACKET] = {0};
    if (crypto_box_curve25519xchacha20poly1305_easy(cipher, plain, plainLen, nonce, peerPk, p2pSecretKey) != 0) return false;

    uint8_t packet[P2P_MAX_PACKET] = {0};
    size_t pos = 0;
    packet[pos++] = P2P_PKT_MAGIC_0;
    packet[pos++] = P2P_PKT_MAGIC_1;
    packet[pos++] = P2P_PKT_VERSION;
    packet[pos++] = type;
    memcpy(packet + pos, p2pPublicKey, sizeof(p2pPublicKey));
    pos += sizeof(p2pPublicKey);
    memcpy(packet + pos, nonce, sizeof(nonce));
    pos += sizeof(nonce);
    packet[pos++] = static_cast<uint8_t>((cipherLen >> 8) & 0xFFU);
    packet[pos++] = static_cast<uint8_t>(cipherLen & 0xFFU);
    memcpy(packet + pos, cipher, cipherLen);
    pos += cipherLen;

    p2pEnsureUdp();
    if (!p2pUdpStarted) return false;
    if (!p2pUdp.beginPacket(peer.ip, peer.port ? peer.port : P2P_UDP_PORT)) return false;
    p2pUdp.write(packet, pos);
    return p2pUdp.endPacket() == 1;
}

bool p2pSendChatMessageWithId(const String &peerKey, const String &text, const String &messageId)
{
    if (!p2pReady) return false;
    if (peerKey.isEmpty() || text.isEmpty() || messageId.isEmpty()) return false;
    const int peerIdx = p2pFindPeerByPubKeyHex(peerKey);
    if (peerIdx < 0 || !p2pPeers[peerIdx].enabled || p2pPeers[peerIdx].ip == IPAddress((uint32_t)0)) return false;

    JsonDocument doc;
    doc["kind"] = "chat";
    doc["id"] = messageId;
    doc["author"] = deviceShortNameValue();
    doc["text"] = text.substring(0, P2P_MAX_CHAT_TEXT);
    char plain[P2P_MAX_PACKET] = {0};
    const size_t plainLen = serializeJson(doc, plain, sizeof(plain));
    if (plainLen == 0) return false;

    return p2pSendPacketToPeer(p2pPeers[peerIdx], P2P_PKT_TYPE_CHAT, reinterpret_cast<const uint8_t *>(plain), plainLen);
}

bool p2pSendChatAck(const String &peerKey, const String &messageId)
{
    if (!p2pReady || peerKey.isEmpty() || messageId.isEmpty()) return false;
    const int peerIdx = p2pFindPeerByPubKeyHex(peerKey);
    if (peerIdx < 0 || !p2pPeers[peerIdx].enabled || p2pPeers[peerIdx].ip == IPAddress((uint32_t)0)) return false;

    JsonDocument doc;
    doc["kind"] = "ack";
    doc["ack_id"] = messageId;
    char plain[P2P_MAX_PACKET] = {0};
    const size_t plainLen = serializeJson(doc, plain, sizeof(plain));
    if (plainLen == 0) return false;

    return p2pSendPacketToPeer(p2pPeers[peerIdx], P2P_PKT_TYPE_CHAT, reinterpret_cast<const uint8_t *>(plain), plainLen);
}

bool p2pSendMessageDelete(const String &peerKey, const String &messageId)
{
    if (!p2pReady || peerKey.isEmpty() || messageId.isEmpty()) return false;
    const int peerIdx = p2pFindPeerByPubKeyHex(peerKey);
    if (peerIdx < 0 || !p2pPeers[peerIdx].enabled || p2pPeers[peerIdx].ip == IPAddress((uint32_t)0)) return false;

    JsonDocument doc;
    doc["kind"] = "delete_message";
    doc["id"] = messageId;
    doc["author"] = deviceShortNameValue();
    char plain[P2P_MAX_PACKET] = {0};
    const size_t plainLen = serializeJson(doc, plain, sizeof(plain));
    if (plainLen == 0) return false;

    return p2pSendPacketToPeer(p2pPeers[peerIdx], P2P_PKT_TYPE_CHAT, reinterpret_cast<const uint8_t *>(plain), plainLen);
}

bool p2pSendConversationDelete()
{
    if (!p2pReady || currentChatPeerKey.isEmpty()) return false;
    const int peerIdx = p2pFindPeerByPubKeyHex(currentChatPeerKey);
    if (peerIdx < 0 || !p2pPeers[peerIdx].enabled || p2pPeers[peerIdx].ip == IPAddress((uint32_t)0)) return false;

    JsonDocument doc;
    doc["kind"] = "delete_conversation";
    doc["author"] = deviceShortNameValue();
    char plain[P2P_MAX_PACKET] = {0};
    const size_t plainLen = serializeJson(doc, plain, sizeof(plain));
    if (plainLen == 0) return false;

    return p2pSendPacketToPeer(p2pPeers[peerIdx], P2P_PKT_TYPE_CHAT, reinterpret_cast<const uint8_t *>(plain), plainLen);
}

void p2pBroadcastDiscover()
{
    if (!p2pReady || !wifiLanAvailableSafe()) return;
    p2pEnsureUdp();
    if (!p2pUdpStarted) return;

    JsonDocument doc;
    doc["kind"] = "discover";
    doc["device"] = deviceShortNameValue();
    doc["public_key"] = p2pPublicKeyHex();
    doc["port"] = P2P_UDP_PORT;
    char payload[256] = {0};
    const size_t len = serializeJson(doc, payload, sizeof(payload));
    if (len == 0) return;

    IPAddress bcast = wifiLanBroadcastIpSafe();
    if (bcast[0] == 0) return;
    if (p2pUdp.beginPacket(bcast, P2P_UDP_PORT)) {
        p2pUdp.write(reinterpret_cast<const uint8_t *>("\x50\x32\x01\x02"), 4);
        p2pUdp.write(reinterpret_cast<const uint8_t *>(payload), len);
        p2pUdp.endPacket();
    }
}

void p2pSendDiscoveryProbe()
{
    if (!p2pReady || !wifiLanAvailableSafe()) return;
    p2pEnsureUdp();
    if (!p2pUdpStarted) return;

    JsonDocument doc;
    doc["kind"] = "discover_probe";
    doc["device"] = deviceShortNameValue();
    doc["public_key"] = p2pPublicKeyHex();
    doc["port"] = P2P_UDP_PORT;
    char payload[256] = {0};
    const size_t len = serializeJson(doc, payload, sizeof(payload));
    if (len == 0) return;

    IPAddress bcast = wifiLanBroadcastIpSafe();
    if (bcast[0] == 0) return;
    if (p2pUdp.beginPacket(bcast, P2P_UDP_PORT)) {
        p2pUdp.write(reinterpret_cast<const uint8_t *>("\x50\x32\x01\x02"), 4);
        p2pUdp.write(reinterpret_cast<const uint8_t *>(payload), len);
        p2pUdp.endPacket();
    }
}

void p2pSendDiscoveryAnnounceTo(const IPAddress &ip, uint16_t port)
{
    if (!p2pReady || !wifiLanAvailableSafe() || !p2pDiscoveryEnabled) return;
    p2pEnsureUdp();
    if (!p2pUdpStarted) return;

    JsonDocument doc;
    doc["kind"] = "discover";
    doc["device"] = deviceShortNameValue();
    doc["public_key"] = p2pPublicKeyHex();
    doc["port"] = P2P_UDP_PORT;
    char payload[256] = {0};
    const size_t len = serializeJson(doc, payload, sizeof(payload));
    if (len == 0) return;

    if (p2pUdp.beginPacket(ip, port ? port : P2P_UDP_PORT)) {
        p2pUdp.write(reinterpret_cast<const uint8_t *>("\x50\x32\x01\x02"), 4);
        p2pUdp.write(reinterpret_cast<const uint8_t *>(payload), len);
        p2pUdp.endPacket();
    }
}

bool p2pSendPairRequest(const String &name, const String &pubKeyHex, const IPAddress &ip, uint16_t port)
{
    if (!p2pReady || !wifiLanAvailableSafe() || pubKeyHex.isEmpty()) return false;
    p2pEnsureUdp();
    if (!p2pUdpStarted) return false;

    JsonDocument doc;
    doc["kind"] = "pair_request";
    doc["device"] = deviceShortNameValue();
    doc["public_key"] = p2pPublicKeyHex();
    doc["port"] = P2P_UDP_PORT;
    char payload[256] = {0};
    const size_t len = serializeJson(doc, payload, sizeof(payload));
    if (len == 0) return false;

    p2pTouchDiscoveredSeen(name, pubKeyHex, ip, port);
    if (!p2pUdp.beginPacket(ip, port ? port : P2P_UDP_PORT)) return false;
    p2pUdp.write(reinterpret_cast<const uint8_t *>("\x50\x32\x01\x02"), 4);
    p2pUdp.write(reinterpret_cast<const uint8_t *>(payload), len);
    return p2pUdp.endPacket() == 1;
}

void p2pSendPairResponse(const String &name, const String &pubKeyHex, const IPAddress &ip, uint16_t port, bool accepted)
{
    if (!p2pReady || !wifiLanAvailableSafe() || pubKeyHex.isEmpty()) return;
    p2pEnsureUdp();
    if (!p2pUdpStarted) return;

    JsonDocument doc;
    doc["kind"] = accepted ? "pair_accept" : "pair_reject";
    doc["device"] = deviceShortNameValue();
    doc["public_key"] = p2pPublicKeyHex();
    doc["port"] = P2P_UDP_PORT;
    char payload[256] = {0};
    const size_t len = serializeJson(doc, payload, sizeof(payload));
    if (len == 0) return;

    p2pTouchDiscoveredSeen(name, pubKeyHex, ip, port);
    if (p2pUdp.beginPacket(ip, port ? port : P2P_UDP_PORT)) {
        p2pUdp.write(reinterpret_cast<const uint8_t *>("\x50\x32\x01\x02"), 4);
        p2pUdp.write(reinterpret_cast<const uint8_t *>(payload), len);
        p2pUdp.endPacket();
    }
}

bool p2pAddOrUpdateTrustedPeer(const String &name, const String &pubKeyHex, const IPAddress &ip, uint16_t port)
{
    unsigned char testPk[P2P_PUBLIC_KEY_BYTES] = {0};
    if (name.isEmpty() || !p2pHexToBytes(pubKeyHex, testPk, sizeof(testPk))) return false;
    if (!p2pEnsurePeerStorage()) return false;
    int idx = p2pFindPeerByPubKeyHex(pubKeyHex);
    if (idx < 0) {
        int sameNameIdx = -1;
        int sameNameCount = 0;
        const bool haveIp = ip != IPAddress((uint32_t)0);
        for (int i = 0; i < p2pPeerCount; ++i) {
            if (!p2pPeers[i].name.equalsIgnoreCase(name)) continue;
            if (haveIp && p2pPeers[i].ip != ip) continue;
            sameNameIdx = i;
            sameNameCount++;
            if (haveIp) break;
        }
        if (sameNameCount == 1 && sameNameIdx >= 0) idx = sameNameIdx;
    }
    if (idx < 0) {
        if (p2pPeerCount >= MAX_P2P_PEERS) return false;
        idx = p2pPeerCount++;
    }
    const String oldPeerKey = p2pPeers[idx].pubKeyHex;
    p2pPeers[idx].name = name;
    p2pPeers[idx].pubKeyHex = pubKeyHex;
    p2pPeers[idx].ip = ip;
    p2pPeers[idx].port = (port == 0) ? P2P_UDP_PORT : port;
    p2pPeers[idx].enabled = true;
    p2pPeers[idx].unread = false;
    p2pPeers[idx].lastSeenMs = millis();
    if (!oldPeerKey.isEmpty() && !oldPeerKey.equalsIgnoreCase(pubKeyHex) && currentChatPeerKey == oldPeerKey) {
        currentChatPeerKey = pubKeyHex;
    }

    p2pDeduplicateTrustedPeers();
    saveP2pConfig();
    p2pTouchDiscoveredSeen(name, pubKeyHex, ip, port);
    return true;
}

void p2pService()
{
    if (!p2pReady) return;
    if (!wifiLanAvailableSafe()) {
        if (p2pUdpStarted) {
            p2pUdp.stop();
            p2pUdpStarted = false;
        }
        return;
    }

    p2pEnsureUdp();
    if (!p2pUdpStarted) return;

    const unsigned long now = millis();
    if (p2pDiscoveryEnabled && wifiLanAvailableSafe() && (now - p2pLastDiscoverAnnounceMs) >= 5000UL) {
        p2pLastDiscoverAnnounceMs = now;
        p2pBroadcastDiscover();
    }

    int packetLen = p2pUdp.parsePacket();
    if (packetLen < 0) {
        const unsigned long nowMs = millis();
#if defined(BOARD_ESP32S3_3248S035_N16R8)
        if (p2pUdpLastParseError != packetLen ||
            static_cast<unsigned long>(nowMs - p2pUdpLastParseErrorMs) >= 3000UL) {
            Serial.printf("[P2P] UDP parsePacket error=%d mode=%d sta=%d ap=%d\n",
                          packetLen,
                          static_cast<int>(WiFi.getMode()),
                          wifiConnectedSafe() ? 1 : 0,
                          apModeActive ? 1 : 0);
            p2pUdpLastParseError = packetLen;
            p2pUdpLastParseErrorMs = nowMs;
        }
#else
        (void)nowMs;
#endif
        p2pUdp.stop();
        p2pUdpStarted = false;
        delay(1);
        return;
    }
    while (packetLen > 0) {
        if (packetLen >= 4 && packetLen <= static_cast<int>(P2P_MAX_PACKET)) {
            uint8_t packet[P2P_MAX_PACKET] = {0};
            const int got = p2pUdp.read(packet, sizeof(packet));
            if (got >= 0) {
                size_t pos = 0;
                const bool headerOk = packet[pos++] == P2P_PKT_MAGIC_0 &&
                                      packet[pos++] == P2P_PKT_MAGIC_1 &&
                                      packet[pos++] == P2P_PKT_VERSION;
                const uint8_t type = packet[pos++];
                if (headerOk && type == P2P_PKT_TYPE_DISCOVER) {
                    JsonDocument doc;
                    if (deserializeJson(doc, packet + pos, got - static_cast<int>(pos)) == DeserializationError::Ok) {
                        const String kind = String(static_cast<const char *>(doc["kind"] | ""));
                        const String device = String(static_cast<const char *>(doc["device"] | ""));
                        const String pubKeyHex = String(static_cast<const char *>(doc["public_key"] | ""));
                        const uint16_t port = static_cast<uint16_t>(doc["port"] | P2P_UDP_PORT);
                        if (kind == "discover" && !pubKeyHex.equalsIgnoreCase(p2pPublicKeyHex())) {
                            p2pTouchDiscoveredSeen(device, pubKeyHex, p2pUdp.remoteIP(), port);
                        } else if (kind == "discover_probe" && !pubKeyHex.equalsIgnoreCase(p2pPublicKeyHex())) {
                            if (p2pDiscoveryEnabled) p2pSendDiscoveryAnnounceTo(p2pUdp.remoteIP(), port);
                        } else if (kind == "pair_request" && !pubKeyHex.equalsIgnoreCase(p2pPublicKeyHex())) {
                            p2pTouchDiscoveredSeen(device, pubKeyHex, p2pUdp.remoteIP(), port);
                            if (p2pDiscoveryEnabled) {
                                const int requestIdx = p2pFindDiscoveredByPubKeyHex(pubKeyHex);
                                p2pPairRequestPeerKey = pubKeyHex;
                                p2pPairRequestPeerName = device.isEmpty() ? String("Peer") : device;
                                p2pPairRequestPeerIp = p2pUdp.remoteIP();
                                p2pPairRequestPeerPort = port;
                                p2pPairRequestPending = true;
                                p2pPairPromptVisible = false;
                                p2pPairRequestDiscoveredIdx = requestIdx;
                                wakeDisplayForIncomingNotification();
                            }
                        } else if (kind == "pair_accept" && !pubKeyHex.equalsIgnoreCase(p2pPublicKeyHex())) {
                            const bool ok = p2pAddOrUpdateTrustedPeer(device.isEmpty() ? String("Peer") : device,
                                                                      pubKeyHex,
                                                                      p2pUdp.remoteIP(),
                                                                      port);
                            if (ok && currentChatPeerKey.isEmpty()) currentChatPeerKey = pubKeyHex;
                            uiStatusLine = ok ? "Peer paired" : "Pair accept failed";
                            if (lvglReady) {
                                lvglSyncStatusLine();
                                lvglRefreshChatPeerUi();
                                lvglRefreshChatContactsUi();
                            }
                        } else if (kind == "pair_reject" && !pubKeyHex.equalsIgnoreCase(p2pPublicKeyHex())) {
                            uiStatusLine = (device.isEmpty() ? String("Peer") : device) + " rejected pairing";
                            if (lvglReady) lvglSyncStatusLine();
                        }
                    }
                } else if (headerOk && type == P2P_PKT_TYPE_CHAT &&
                           got >= static_cast<int>(4 + P2P_PUBLIC_KEY_BYTES + P2P_NONCE_BYTES + 2)) {
                    unsigned char senderPk[P2P_PUBLIC_KEY_BYTES] = {0};
                    unsigned char nonce[P2P_NONCE_BYTES] = {0};
                    memcpy(senderPk, packet + pos, sizeof(senderPk));
                    pos += sizeof(senderPk);
                    memcpy(nonce, packet + pos, sizeof(nonce));
                    pos += sizeof(nonce);
                    const size_t cipherLen = (static_cast<size_t>(packet[pos]) << 8) | static_cast<size_t>(packet[pos + 1]);
                    pos += 2;
                    if ((pos + cipherLen) <= static_cast<size_t>(got) && cipherLen >= P2P_MAC_BYTES) {
                        const String senderHex = p2pBytesToHex(senderPk, sizeof(senderPk));
                        const int peerIdx = p2pFindPeerByPubKeyHex(senderHex);
                        if (peerIdx >= 0 && p2pPeers[peerIdx].enabled) {
                            unsigned char plain[P2P_MAX_PACKET] = {0};
                            if (crypto_box_curve25519xchacha20poly1305_open_easy(
                                    plain,
                                    packet + pos,
                                    cipherLen,
                                    nonce,
                                    senderPk,
                                    p2pSecretKey) == 0) {
                                JsonDocument doc;
                                if (deserializeJson(doc, plain, cipherLen - P2P_MAC_BYTES) == DeserializationError::Ok) {
                                    const String kind = String(static_cast<const char *>(doc["kind"] | ""));
                                    const String author = String(static_cast<const char *>(doc["author"] | p2pPeers[peerIdx].name.c_str()));
                                    if (kind == "chat") {
                                        const String messageId = String(static_cast<const char *>(doc["id"] | ""));
                                        const String text = String(static_cast<const char *>(doc["text"] | ""));
                                        if (!text.isEmpty()) {
                                            wakeDisplayForIncomingNotification();
                                            p2pRefreshTrustedPeerIdentity(p2pPeers[peerIdx].pubKeyHex, author, p2pUdp.remoteIP(), p2pUdp.remotePort());
                                            p2pTouchPeerSeen(peerIdx, p2pUdp.remoteIP(), p2pUdp.remotePort());

                                            const String &senderPeerKey = p2pPeers[peerIdx].pubKeyHex;
                                            const bool isDuplicate = !messageId.isEmpty() && chatHasLoggedMessageId(senderPeerKey, messageId);

                                            // ACK as early as possible so sender is released faster.
                                            // Also ACK duplicates again, in case sender missed the first ACK.
                                            if (!messageId.isEmpty()) {
                                                p2pSendChatAck(senderPeerKey, messageId);
                                            }

                                            if (checkersHandleIncomingChatPayload(senderPeerKey,
                                                                                author,
                                                                                text,
                                                                                CHAT_TRANSPORT_WIFI,
                                                                                messageId)) {
                                                if (lvglReady) {
                                                    lvglSyncStatusLine();
                                                    if (uiScreen == UI_CHAT) lvglRefreshChatUi();
                                                }
                                            } else if (!isDuplicate) {
                                                chatStoreMessage(senderPeerKey, author, text, false, CHAT_TRANSPORT_WIFI, messageId);
                                                uiStatusLine = "Chat received from " + author;
                                                if (lvglReady) {
                                                    lvglSyncStatusLine();
                                                    if (uiScreen == UI_CHAT) lvglRefreshChatUi();
                                                }
                                            }
                                        }
                                    } else if (kind == "ack") {
                                        const String ackId = String(static_cast<const char *>(doc["ack_id"] | ""));
                                        if (!ackId.isEmpty()) chatAckOutgoingMessage(p2pPeers[peerIdx].pubKeyHex, ackId);
                                    } else if (kind == "delete_message") {
                                        const String messageId = String(static_cast<const char *>(doc["id"] | ""));
                                        if (!messageId.isEmpty()) {
                                            p2pRefreshTrustedPeerIdentity(p2pPeers[peerIdx].pubKeyHex, author, p2pUdp.remoteIP(), p2pUdp.remotePort());
                                            p2pTouchPeerSeen(peerIdx, p2pUdp.remoteIP(), p2pUdp.remotePort());
                                            chatDeleteMessageById(p2pPeers[peerIdx].pubKeyHex, messageId, "Message deleted by " + author);
                                        }
                                    } else if (kind == "delete_conversation") {
                                        p2pRefreshTrustedPeerIdentity(p2pPeers[peerIdx].pubKeyHex, author, p2pUdp.remoteIP(), p2pUdp.remotePort());
                                        p2pTouchPeerSeen(peerIdx, p2pUdp.remoteIP(), p2pUdp.remotePort());
                                        chatApplyConversationDeletion(p2pPeers[peerIdx].pubKeyHex, "Conversation deleted by " + author);
                                    }
                                }
                            }
                        }
                    }
                }
            }
        } else {
            while (p2pUdp.available()) p2pUdp.read();
        }
        packetLen = p2pUdp.parsePacket();
        if (packetLen < 0) {
            p2pUdp.stop();
            p2pUdpStarted = false;
            delay(1);
            return;
        }
        yield();
    }
}

void lvglRefreshInfoPanel(bool refreshIndicators)
{
    if (!lvglInfoList) return;
    if (uiScreen != UI_INFO) return;
    if (refreshIndicators) {
        sampleTopIndicators();
        lvglRefreshTopIndicators();
    }
    const uint32_t largest8 = static_cast<uint32_t>(heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
    const uint32_t freeHeap = static_cast<uint32_t>(ESP.getFreeHeap());
    const uint32_t psramFree = boardHasUsablePsram()
                                   ? static_cast<uint32_t>(heap_caps_get_free_size(MALLOC_CAP_SPIRAM))
                                   : 0U;
    const uint32_t psramTotal = boardHasUsablePsram()
                                    ? static_cast<uint32_t>(heap_caps_get_total_size(MALLOC_CAP_SPIRAM))
                                    : 0U;
    char batteryBuf[40];
    const int batteryCentivolts = static_cast<int>(batteryVoltage * 100.0f + 0.5f);
    snprintf(batteryBuf, sizeof(batteryBuf), "%d.%02dV  |  %s",
             batteryCentivolts / 100,
             abs(batteryCentivolts % 100),
             batteryCharging ? "Charging" : "On battery");
    if (lvglInfoBatteryValueLabel) lvglLabelSetTextIfChanged(lvglInfoBatteryValueLabel, String(batteryPercent) + "%");
    if (lvglInfoBatterySubLabel) lvglLabelSetTextIfChanged(lvglInfoBatterySubLabel, batteryBuf);
    if (lv_obj_t *batteryBar = lvglInfoCardBar(lvglInfoBatteryCard)) {
        lvglBarSetValueIfChanged(batteryBar, static_cast<int32_t>(batteryPercent), LV_ANIM_ON);
        lv_color_t batteryColor = lv_color_hex(0x52B788);
        if (batteryPercent <= 20) batteryColor = lv_color_hex(0xD95C5C);
        else if (batteryPercent <= 45) batteryColor = lv_color_hex(0xF2C35E);
        lvglSetInfoBarColor(batteryBar, batteryColor);
    }

    if (lvglInfoWifiValueLabel) lvglLabelSetTextIfChanged(lvglInfoWifiValueLabel, infoWifiValueText);
    if (lvglInfoWifiSubLabel) lvglLabelSetTextIfChanged(lvglInfoWifiSubLabel, infoWifiSubText);
    if (lv_obj_t *wifiBar = lvglInfoCardBar(lvglInfoWifiCard)) {
        lvglBarSetValueIfChanged(wifiBar, static_cast<int32_t>(infoWifiQualityPercent), LV_ANIM_ON);
        lv_color_t wifiColor = lv_color_hex(0x4A5563);
        if (infoWifiQualityPercent > 0U) {
            wifiColor = lv_color_hex(0x4FC3F7);
            if (infoWifiQualityPercent <= 25) wifiColor = lv_color_hex(0xD95C5C);
            else if (infoWifiQualityPercent <= 55) wifiColor = lv_color_hex(0xF2C35E);
        }
        lvglSetInfoBarColor(wifiBar, wifiColor);
    }

    applyRadioInfoLabels();

    if (lvglInfoLightValueLabel) lvglLabelSetTextIfChanged(lvglInfoLightValueLabel, infoLightValueText);
    if (lvglInfoLightSubLabel) lvglLabelSetTextIfChanged(lvglInfoLightSubLabel, infoLightSubText);
    if (lv_obj_t *lightBar = lvglInfoCardBar(lvglInfoLightCard)) {
        lvglBarSetValueIfChanged(lightBar, static_cast<int32_t>(infoLightPercentValue), LV_ANIM_ON);
        lv_color_t lightColor = lv_color_hex(0x4A5563);
        if (infoLightPercentValue >= 75) lightColor = lv_color_hex(0xF4B942);
        else if (infoLightPercentValue >= 35) lightColor = lv_color_hex(0xF2C35E);
        else lightColor = lv_color_hex(0x6AAEE6);
        lvglSetInfoBarColor(lightBar, lightColor);
    }

    if (lv_obj_t *sdValue = lvglInfoCardValueLabel(lvglInfoSdCard)) {
        lvglLabelSetTextIfChanged(sdValue, infoSdValueText);
    }
    if (lv_obj_t *sdSub = lvglInfoCardSubLabel(lvglInfoSdCard)) {
        lvglLabelSetTextIfChanged(sdSub, infoSdSubText);
    }
    if (lv_obj_t *sdBar = lvglInfoCardBar(lvglInfoSdCard)) {
        lvglBarSetValueIfChanged(sdBar, static_cast<int32_t>(infoSdPercentValue), LV_ANIM_ON);
        lv_color_t sdColor = lv_color_hex(0xE59F45);
        if (infoSdPercentValue >= 90) sdColor = lv_color_hex(0xD95C5C);
        else if (infoSdPercentValue >= 75) sdColor = lv_color_hex(0xF2C35E);
        lvglSetInfoBarColor(sdBar, infoSdPercentValue > 0U ? sdColor : lv_color_hex(0x3A4150));
    }

    if (lv_obj_t *sramValue = lvglInfoCardValueLabel(lvglInfoSramCard)) {
        lvglLabelSetTextIfChanged(sramValue, infoSramValueText);
    }
    if (lv_obj_t *sramSub = lvglInfoCardSubLabel(lvglInfoSramCard)) {
        lvglLabelSetTextIfChanged(sramSub, infoSramSubText);
    }
    if (lv_obj_t *sramBar = lvglInfoCardBar(lvglInfoSramCard)) {
        lvglBarSetValueIfChanged(sramBar, static_cast<int32_t>(infoSramPercentValue), LV_ANIM_ON);
        lv_color_t sramColor = lv_color_hex(0x4FC3F7);
        if (infoSramPercentValue >= 85) sramColor = lv_color_hex(0xD95C5C);
        else if (infoSramPercentValue >= 65) sramColor = lv_color_hex(0xF2C35E);
        lvglSetInfoBarColor(sramBar, sramColor);
    }

    if (lv_obj_t *psramValue = lvglInfoCardValueLabel(lvglInfoPsramCard)) {
        lvglLabelSetTextIfChanged(psramValue, infoPsramValueText);
    }
    if (lv_obj_t *psramSub = lvglInfoCardSubLabel(lvglInfoPsramCard)) {
        lvglLabelSetTextIfChanged(psramSub, infoPsramSubText);
    }
    if (lv_obj_t *psramBar = lvglInfoCardBar(lvglInfoPsramCard)) {
        lvglBarSetValueIfChanged(psramBar, static_cast<int32_t>(infoPsramPercentValue), LV_ANIM_ON);
        lv_color_t psramColor = lv_color_hex(0x9B7CF2);
        if (infoPsramPercentValue >= 85) psramColor = lv_color_hex(0xD95C5C);
        else if (infoPsramPercentValue >= 65) psramColor = lv_color_hex(0xF2C35E);
        lvglSetInfoBarColor(psramBar, psramTotal > 0 ? psramColor : lv_color_hex(0x3A4150));
    }

    if (lv_obj_t *cpuValue = lvglInfoCardValueLabel(lvglInfoCpuCard)) {
        lvglLabelSetTextIfChanged(cpuValue, String(cpuLoadPercent) + "%");
    }
    if (lv_obj_t *cpuSub = lvglInfoCardSubLabel(lvglInfoCpuCard)) {
        if (psramTotal > 0) {
            lvglLabelSetTextIfChanged(cpuSub,
                                      String("Approx. main-loop load  |  SRAM ") +
                                      String(static_cast<unsigned long>(freeHeap / 1024U)) +
                                      " KB  |  PSRAM " +
                                      String(static_cast<unsigned long>(psramFree / 1024U)) +
                                      " KB");
        } else {
            lvglLabelSetTextIfChanged(cpuSub,
                                      String("Approx. main-loop load  |  SRAM ") +
                                      String(static_cast<unsigned long>(freeHeap / 1024U)) +
                                      " KB");
        }
    }
    if (lv_obj_t *cpuBar = lvglInfoCardBar(lvglInfoCpuCard)) {
        lvglBarSetValueIfChanged(cpuBar, static_cast<int32_t>(cpuLoadPercent), LV_ANIM_OFF);
        lv_color_t cpuColor = lv_color_hex(0x52B788);
        if (cpuLoadPercent >= 85) cpuColor = lv_color_hex(0xD95C5C);
        else if (cpuLoadPercent >= 65) cpuColor = lv_color_hex(0xF2C35E);
        lvglSetInfoBarColor(cpuBar, cpuColor);
    }

    if (lv_obj_t *tempValue = lvglInfoCardValueLabel(lvglInfoTempCard)) {
        lvglLabelSetTextIfChanged(tempValue, infoTempValueText);
    }
    if (lv_obj_t *tempSub = lvglInfoCardSubLabel(lvglInfoTempCard)) lvglLabelSetTextIfChanged(tempSub, infoTempSubText);
    if (lv_obj_t *tempBar = lvglInfoCardBar(lvglInfoTempCard)) {
        lvglBarSetValueIfChanged(tempBar, static_cast<int32_t>(infoTempBarValue), LV_ANIM_OFF);
        lv_color_t tempColor = lv_color_hex(0x4FC3F7);
        if (infoTempBarValue >= INFO_TEMP_HOT_C) tempColor = lv_color_hex(0xD95C5C);
        else if (infoTempBarValue >= INFO_TEMP_WARN_C) tempColor = lv_color_hex(0xF2C35E);
        lvglSetInfoBarColor(tempBar, tempColor);
    }

    if (lvglInfoSystemLabel) {
        if (psramTotal > 0) {
            lv_label_set_text_fmt(
                lvglInfoSystemLabel,
                "Model: %s\nDevice name: %s\nFirmware: %s\nSoftAP: %s\nLargest 8-bit block: %lu KB\nPSRAM: %lu / %lu KB free\nSD: %s\nMedia: %s",
                DEVICE_MODEL,
                deviceShortNameValue().c_str(),
                FW_VERSION,
                AP_SSID,
                static_cast<unsigned long>(largest8 / 1024U),
                static_cast<unsigned long>(psramFree / 1024U),
                static_cast<unsigned long>(psramTotal / 1024U),
                sdMounted ? "mounted" : "offline",
                mediaIsPlaying ? mediaNowPlaying.c_str() : "stopped");
        } else {
            lv_label_set_text_fmt(
                lvglInfoSystemLabel,
                "Model: %s\nDevice name: %s\nFirmware: %s\nSoftAP: %s\nLargest 8-bit block: %lu KB\nSD: %s\nMedia: %s",
                DEVICE_MODEL,
                deviceShortNameValue().c_str(),
                FW_VERSION,
                AP_SSID,
                static_cast<unsigned long>(largest8 / 1024U),
                sdMounted ? "mounted" : "offline",
                mediaIsPlaying ? mediaNowPlaying.c_str() : "stopped");
        }
    }
}

void lvglRefreshWifiList()
{
    if (!lvglWifiList) return;
    lv_obj_clean(lvglWifiList);
    lvglWifiScanLabel = nullptr;
    lvglWifiApSsidTa = nullptr;
    lvglWifiApPassTa = nullptr;
    lvglWifiApPassShowBtnLabel = nullptr;
    lvglWifiApPassShowBtn = nullptr;
    lvglWifiWebServerBtn = nullptr;

    auto makeCard = [&](const char *title) -> lv_obj_t * {
        lv_obj_t *card = lv_obj_create(lvglWifiList);
        if (!card) return nullptr;
        lv_obj_set_width(card, lv_pct(100));
        lv_obj_set_height(card, LV_SIZE_CONTENT);
        lv_obj_set_style_bg_color(card, lv_color_hex(0x16212C), 0);
        lv_obj_set_style_border_width(card, 0, 0);
        lv_obj_set_style_radius(card, 12, 0);
        lv_obj_set_style_pad_all(card, 8, 0);
        lv_obj_set_style_pad_row(card, 6, 0);
        lv_obj_set_flex_flow(card, LV_FLEX_FLOW_COLUMN);
        lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_t *lbl = lv_label_create(card);
        if (lbl) {
            lv_label_set_text(lbl, title);
            lv_obj_set_style_text_color(lbl, lv_color_hex(0xE5ECF3), 0);
        }
        return card;
    };

    auto makeBtn = [](lv_obj_t *parent, const char *txt, int32_t w, int32_t h, lv_color_t col, lv_event_cb_t cb, void *ud = nullptr) -> lv_obj_t * {
        lv_obj_t *btn = lv_btn_create(parent);
        if (!btn) return nullptr;
        lv_obj_set_size(btn, w, h);
        lv_obj_set_style_radius(btn, 8, 0);
        lv_obj_set_style_border_width(btn, 0, 0);
        lv_obj_add_event_cb(btn, lvglClickFilterEvent, LV_EVENT_CLICKED, nullptr);
        lv_obj_add_event_cb(btn, lvglFilteredClickFlashEvent, LV_EVENT_CLICKED, nullptr);
        lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, ud);
        lv_obj_t *lbl = lv_label_create(btn);
        if (lbl) {
            lv_label_set_text(lbl, txt);
            lv_obj_center(lbl);
        }
        lvglRegisterStyledButton(btn, col, true);
        return btn;
    };

    lv_obj_t *staCard = makeCard("WiFi Config");
    if (staCard) {
        lvglRegisterReorderableItem(staCard, "ord_wifi", "sta");
        lv_obj_t *info = lv_label_create(staCard);
        lv_obj_set_width(info, lv_pct(100));
        lv_label_set_long_mode(info, LV_LABEL_LONG_WRAP);
        const String desiredStaSsid = wifiDesiredStaSsid();
        String staLine = desiredStaSsid.isEmpty() ? String("Saved network: none") : ("Saved network: " + desiredStaSsid);
        if (wifiForgetPendingUi) staLine += "\nStatus: forgetting saved WiFi...";
        else if (wifiConnectedSafe()) staLine += "\nConnected: " + wifiSsidSafe() + "  |  " + wifiIpSafe();
        else if (bootStaConnectInProgress) staLine += "\nStatus: connecting...";
        else staLine += "\nStatus: disconnected";
        lv_label_set_text(info, staLine.c_str());
        lv_obj_set_style_text_color(info, lv_color_hex(0xB7C4D1), 0);

        lv_obj_t *row = lv_obj_create(staCard);
        lv_obj_set_size(row, lv_pct(100), 34);
        lv_obj_set_style_bg_opa(row, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(row, 0, 0);
        lv_obj_set_style_pad_all(row, 0, 0);
        lv_obj_set_style_pad_column(row, 6, 0);
        lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
        lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);
        if (wifiConnectedSafe() || bootStaConnectInProgress || wifiForgetPendingUi) {
            makeBtn(row, lvglSymbolText(LV_SYMBOL_CLOSE, "Disconnect").c_str(), 102, 28, lv_color_hex(0x2F6D86), lvglWifiDisconnectEvent);
        }
        if (!desiredStaSsid.isEmpty() || pendingSaveCreds || wifiForgetPendingUi) {
            makeBtn(row, lvglSymbolText(LV_SYMBOL_TRASH, "Forget").c_str(), 82, 28, lv_color_hex(0x8A3A3A), lvglWifiForgetEvent);
        }

        lvglWifiWebServerBtn = makeBtn(
            staCard,
            lvglSymbolText(LV_SYMBOL_DIRECTORY, webServerEnabled ? "Web Server: ON" : "Web Server: OFF").c_str(),
            164,
            30,
            webServerEnabled ? lv_color_hex(0x357A38) : lv_color_hex(0x6B3A3A),
            lvglWifiWebServerToggleEvent
        );
        if (lvglWifiWebServerBtn) {
            lvglApplyWifiWebServerButtonStyle();
        }
    }

    lv_obj_t *scanBtn = makeBtn(lvglWifiList, lvglSymbolText(LV_SYMBOL_REFRESH, "Scan").c_str(), 108, 30, lv_color_hex(0x2F6D86), lvglWifiRescanEvent);
    if (scanBtn) lvglRegisterReorderableItem(scanBtn, "ord_wifi", "scan");

    if (wifiScanInProgress) {
        lvglWifiScanLabel = lv_label_create(lvglWifiList);
        if (lvglWifiScanLabel) {
            lv_label_set_text(lvglWifiScanLabel, "Searching for access points");
            lv_obj_set_width(lvglWifiScanLabel, lv_pct(100));
            lv_obj_set_style_text_align(lvglWifiScanLabel, LV_TEXT_ALIGN_CENTER, 0);
            lv_obj_set_style_text_color(lvglWifiScanLabel, lv_color_hex(0xC8CED6), 0);
            lvglRegisterReorderableItem(lvglWifiScanLabel, "ord_wifi", "scaning");
        }
    } else if (wifiCount > 0) {
        lv_obj_t *sec = lv_label_create(lvglWifiList);
        lv_label_set_text(sec, "Discovered Networks");
        lv_obj_set_style_text_color(sec, lv_color_hex(0xE5ECF3), 0);
        lvglRegisterReorderableItem(sec, "ord_wifi", "hdr");
        for (int i = 0; i < wifiCount; i++) {
            char line[96];
            snprintf(line, sizeof(line), "%s (%ddBm) [%s]", wifiEntries[i].ssid.c_str(), static_cast<int>(wifiEntries[i].rssi), authName(wifiEntries[i].auth));
            lv_obj_t *networkBtn = lvglCreateMenuButton(
                lvglWifiList,
                lvglSymbolText(LV_SYMBOL_WIFI, line).c_str(),
                (wifiEntries[i].auth == WIFI_AUTH_OPEN) ? lv_color_hex(0x357A38) : lv_color_hex(0x375A7A),
                lvglWifiEntryEvent,
                reinterpret_cast<void *>(static_cast<intptr_t>(i))
            );
            if (networkBtn) {
                const String orderKey = lvglOrderTokenFromText("n", wifiEntries[i].ssid + "|" + String(static_cast<int>(wifiEntries[i].auth)));
                lvglRegisterReorderableItem(networkBtn, "ord_wifi", orderKey.c_str());
            }
        }
    } else {
        lv_obj_t *lbl = lv_label_create(lvglWifiList);
        lv_label_set_text(lbl, "No scan started yet. Tap Scan.");
        lv_obj_set_style_text_color(lbl, lv_color_hex(0xC8CED6), 0);
        lvglRegisterReorderableItem(lbl, "ord_wifi", "empty");
    }

    lv_obj_t *apCard = makeCard("AP Config");
    if (apCard) {
        lvglRegisterReorderableItem(apCard, "ord_wifi", "ap");
        lv_obj_t *hint = lv_label_create(apCard);
        lv_obj_set_width(hint, lv_pct(100));
        lv_label_set_long_mode(hint, LV_LABEL_LONG_WRAP);
        lv_label_set_text_fmt(hint, "Current AP: %s", savedApSsid.c_str());
        lv_obj_set_style_text_color(hint, lv_color_hex(0xB7C4D1), 0);

        auto addTa = [&](lv_obj_t *parent, const char *ph, const char *value, bool pass) -> lv_obj_t * {
            lv_obj_t *ta = lv_textarea_create(parent ? parent : apCard);
            lv_obj_set_width(ta, lv_pct(100));
            lv_textarea_set_one_line(ta, true);
            lv_textarea_set_placeholder_text(ta, ph);
            if (pass) lvglConfigureSingleLinePasswordTa(ta);
            lv_textarea_set_text(ta, value ? value : "");
            lv_obj_add_event_cb(ta, lvglTextAreaFocusEvent, LV_EVENT_FOCUSED, nullptr);
            return ta;
        };

        lvglWifiApSsidTa = addTa(apCard, "AP name", savedApSsid.c_str(), false);

        lv_obj_t *pwdRow = lv_obj_create(apCard);
        lv_obj_set_size(pwdRow, lv_pct(100), LV_SIZE_CONTENT);
        lv_obj_set_style_bg_opa(pwdRow, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(pwdRow, 0, 0);
        lv_obj_set_style_pad_all(pwdRow, 0, 0);
        lv_obj_set_style_pad_column(pwdRow, 6, 0);
        lv_obj_set_flex_flow(pwdRow, LV_FLEX_FLOW_ROW);
        lv_obj_clear_flag(pwdRow, LV_OBJ_FLAG_SCROLLABLE);

        lvglWifiApPassTa = addTa(pwdRow, "AP password", savedApPass.c_str(), true);
        lv_obj_set_width(lvglWifiApPassTa, lv_pct(100));
        lv_obj_set_flex_grow(lvglWifiApPassTa, 1);
        lv_obj_set_height(lvglWifiApPassTa, 38);

        lvglWifiApPassShowBtn = lv_btn_create(pwdRow);
        lv_obj_set_size(lvglWifiApPassShowBtn, 34, 34);
        lv_obj_set_style_radius(lvglWifiApPassShowBtn, 8, 0);
        lv_obj_set_style_border_width(lvglWifiApPassShowBtn, 0, 0);
        lv_obj_add_event_cb(lvglWifiApPassShowBtn, lvglClickFilterEvent, LV_EVENT_CLICKED, nullptr);
        lv_obj_add_event_cb(lvglWifiApPassShowBtn, lvglFilteredClickFlashEvent, LV_EVENT_CLICKED, nullptr);
        lv_obj_add_event_cb(lvglWifiApPassShowBtn, lvglWifiApShowToggleEvent, LV_EVENT_CLICKED, nullptr);
        lvglWifiApPassShowBtnLabel = lv_label_create(lvglWifiApPassShowBtn);
        lv_label_set_text(lvglWifiApPassShowBtnLabel, LV_SYMBOL_EYE_CLOSE);
        lv_obj_center(lvglWifiApPassShowBtnLabel);
        lvglRegisterStyledButton(lvglWifiApPassShowBtn, lv_color_hex(0x3F4A57), true);

        makeBtn(apCard, lvglSymbolText(LV_SYMBOL_SAVE, "Save AP Config").c_str(), 144, 30, lv_color_hex(0x3A7A3A), lvglWifiApSaveEvent);
    }
    lvglApplySavedOrder(lvglWifiList, "ord_wifi");
}

void lvglQueueMediaRefresh()
{
    lvglMediaRefreshPending = true;
}

void lvglRefreshMediaLayout()
{
    if (!lvglMediaList || !lvglMediaPlayerPanel) return;
    if (lvglMediaPlayerVisible) {
        lv_obj_clear_flag(lvglMediaPlayerPanel, LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_size(lvglMediaPlayerPanel, lv_pct(100), MEDIA_PLAYER_PANEL_H);
        lv_obj_align(lvglMediaPlayerPanel, LV_ALIGN_TOP_MID, 0, UI_CONTENT_TOP_Y);
        lv_obj_set_size(lvglMediaList, lv_pct(100), UI_CONTENT_H - MEDIA_PLAYER_PANEL_H);
        lv_obj_align(lvglMediaList, LV_ALIGN_TOP_MID, 0, UI_CONTENT_TOP_Y + MEDIA_PLAYER_PANEL_H);
    } else {
        lv_obj_add_flag(lvglMediaPlayerPanel, LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_size(lvglMediaList, lv_pct(100), UI_CONTENT_H);
        lv_obj_align(lvglMediaList, LV_ALIGN_TOP_MID, 0, UI_CONTENT_TOP_Y);
    }
}

void lvglSetMediaPlayerVisible(bool visible)
{
    if (lvglMediaPlayerVisible == visible) return;
    lvglMediaPlayerVisible = visible;
    lvglRefreshMediaLayout();
}

void lvglRefreshMediaList()
{
    if (!lvglMediaList) return;
    lv_obj_clean(lvglMediaList);
    mediaEnsureStorageReadyForUi();
    loadMediaEntries();

    if (mediaOffset > 0) {
        lv_obj_t *btn = lvglCreateMenuButton(lvglMediaList, lvglSymbolText(LV_SYMBOL_PREV, "Prev page").c_str(), lv_color_hex(0x3F4A57), lvglMediaEntryEvent, reinterpret_cast<void *>(static_cast<intptr_t>(MEDIA_ENTRY_PREV_PAGE)));
        if (btn) lvglRegisterReorderableItem(btn, "ord_media", "pg_prev");
    }
    if (mediaCurrentDir != "/") {
        lv_obj_t *btn = lvglCreateMenuButton(lvglMediaList, lvglSymbolText(LV_SYMBOL_DIRECTORY, "Parent").c_str(), lv_color_hex(0x4E5D6C), lvglMediaEntryEvent, reinterpret_cast<void *>(static_cast<intptr_t>(MEDIA_ENTRY_PARENT)));
        if (btn) lvglRegisterReorderableItem(btn, "ord_media", "parent");
    }
    for (int i = 0; i < mediaCount; i++) {
        String label = lvglSymbolText(mediaEntries[i].isDir ? LV_SYMBOL_DIRECTORY : LV_SYMBOL_FILE,
                                      mediaBuildListLabel(mediaEntries[i].name, mediaEntries[i].isDir, mediaEntries[i].size));
        lv_obj_t *entryBtn = lvglCreateMenuButton(
            lvglMediaList,
            label.c_str(),
            mediaEntries[i].isDir ? lv_color_hex(0x2E7D9A) : lv_color_hex(0x355C3D),
            lvglMediaEntryEvent,
            reinterpret_cast<void *>(static_cast<intptr_t>(i))
        );
        if (entryBtn) {
            const String orderKey = lvglOrderTokenFromText("m", mediaEntries[i].path);
            lvglRegisterReorderableItem(entryBtn, "ord_media", orderKey.c_str());
        }
    }
    if (mediaHasMore) {
        lv_obj_t *btn = lvglCreateMenuButton(lvglMediaList, lvglSymbolText(LV_SYMBOL_NEXT, "Next page").c_str(), lv_color_hex(0x3F4A57), lvglMediaEntryEvent, reinterpret_cast<void *>(static_cast<intptr_t>(MEDIA_ENTRY_NEXT_PAGE)));
        if (btn) lvglRegisterReorderableItem(btn, "ord_media", "pg_next");
    }
    if (mediaCount == 0) {
        lv_obj_t *lbl = lv_label_create(lvglMediaList);
        lv_label_set_text(lbl, "No media entries");
        lv_obj_set_style_text_color(lbl, lv_color_hex(0xC8CED6), 0);
        lvglRegisterReorderableItem(lbl, "ord_media", "empty");
    }
    lvglApplySavedOrder(lvglMediaList, "ord_media");
}

void lvglStatusPush(const String &line)
{
    uiStatusLine = line;
    lvglSyncStatusLine();
    if (lvglMqttStatusLabel) lv_label_set_text_fmt(lvglMqttStatusLabel, "MQTT: %s", mqttStatusLine.c_str());
}

void lvglHideKeyboard()
{
    if (lvglKb) {
        lv_keyboard_set_textarea(lvglKb, nullptr);
        lv_obj_add_flag(lvglKb, LV_OBJ_FLAG_HIDDEN);
    }
    lvglKeyboardFocusedTa = nullptr;
    lvglKeyboardUseLocalized = false;
    lvglKeyboardSpaceSwipeActive = false;
    lvglKeyboardSpaceSwipeTriggered = false;
    lvglKeyboardShiftOneShot = false;
    lvglKeyboardShiftLocked = false;
    lvglKeyboardShiftLastPressMs = 0;
    lvglKeyboardReleasePaddedContainer();
    lvglSetChatKeyboardVisible(false);
    lvglSetConfigKeyboardVisible(false);
    lvglRefreshWifiPasswordDialogLayout();
}

void lvglTextAreaFocusEvent(lv_event_t *e)
{
    lv_obj_t *ta = lv_event_get_target(e);
    if (!ta) return;
    if (!lvglKb) {
        lvglKb = lv_keyboard_create(lv_layer_top());
        lv_obj_set_size(lvglKb, lv_pct(100), 120);
        lv_obj_align(lvglKb, LV_ALIGN_BOTTOM_MID, 0, 0);
        lv_obj_add_flag(lvglKb, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_event_cb(
            lvglKb,
            [](lv_event_t *ke) {
                lv_event_code_t code = lv_event_get_code(ke);
                if (code == LV_EVENT_PRESSED && lvglKb) {
                    lvglKeyboardSpaceSwipeActive = false;
                    lvglKeyboardSpaceSwipeTriggered = false;
                    lv_point_t pt = {0, 0};
                    lv_indev_t *indev = lv_indev_get_act();
                    if (indev) lv_indev_get_point(indev, &pt);
                    lvglKeyboardSpaceSwipeStart = pt;
                    return;
                }
                if (code == LV_EVENT_PRESSING && lvglKb) {
                    const uint16_t btn = lv_btnmatrix_get_selected_btn(lvglKb);
                    const char *txt = lv_btnmatrix_get_btn_text(lvglKb, btn);
                    if (!txt || strcmp(txt, " ") != 0 || !lvglKeyboardLocalizedLayoutAvailable(uiLanguage)) return;

                    lv_point_t pt = {0, 0};
                    lv_indev_t *indev = lv_indev_get_act();
                    if (indev) lv_indev_get_point(indev, &pt);
                    const int dx = static_cast<int>(pt.x) - static_cast<int>(lvglKeyboardSpaceSwipeStart.x);
                    const int dy = static_cast<int>(pt.y) - static_cast<int>(lvglKeyboardSpaceSwipeStart.y);

                    if (!lvglKeyboardSpaceSwipeTriggered && abs(dx) >= 18 && abs(dx) > (abs(dy) * 2)) {
                        lvglKeyboardSpaceSwipeTriggered = true;
                        lvglKeyboardSpaceSwipeActive = true;
                        lvglKeyboardUseLocalized = !lvglKeyboardUseLocalized;
                        lvglKeyboardApplyTextMode();
                    }
                    return;
                }
                if ((code == LV_EVENT_RELEASED || code == LV_EVENT_PRESS_LOST) && lvglKb) {
                    lvglKeyboardSpaceSwipeActive = false;
                    return;
                }
                if (code == LV_EVENT_VALUE_CHANGED && lvglKb) {
                    const uint16_t btn = lv_btnmatrix_get_selected_btn(lvglKb);
                    const char *txt = lv_btnmatrix_get_btn_text(lvglKb, btn);
                    const bool isShiftKey = txt &&
                                            ((strcmp(txt, "ABC") == 0) ||
                                             (strcmp(txt, "Abc") == 0) ||
                                             (strcmp(txt, "abc") == 0) ||
                                             (strcmp(txt, LV_SYMBOL_UP) == 0));
                    if (isShiftKey) {
                        if (lvglKeyboardShiftLocked) {
                            lvglKeyboardShiftLocked = false;
                            lvglKeyboardShiftOneShot = false;
                            lvglKeyboardShiftLastPressMs = 0;
                            lvglKeyboardApplyTextMode();
                            return;
                        }
                        if (lvglKeyboardShiftOneShot) {
                            lvglKeyboardShiftLocked = true;
                            lvglKeyboardShiftOneShot = false;
                            lvglKeyboardShiftLastPressMs = 0;
                            lvglKeyboardApplyTextMode();
                            return;
                        }
                        lvglKeyboardShiftLocked = false;
                        lvglKeyboardShiftOneShot = true;
                        lvglKeyboardShiftLastPressMs = millis();
                        lvglKeyboardApplyTextMode();
                        return;
                    }
                    if (txt && ((strcmp(txt, "abc") == 0) || (strcmp(txt, "ABC") == 0))) {
                        lvglKeyboardApplyTextMode();
                        return;
                    }
                    if (txt && strcmp(txt, " ") == 0 && lvglKeyboardSpaceSwipeTriggered) {
                        if (lvglKeyboardFocusedTa && lv_obj_is_valid(lvglKeyboardFocusedTa)) {
                            uint32_t pos = lv_textarea_get_cursor_pos(lvglKeyboardFocusedTa);
                            if (pos > 0) {
                                lv_textarea_del_char(lvglKeyboardFocusedTa);
                            }
                        }
                        lvglKeyboardSpaceSwipeTriggered = false;
                        return;
                    }
                    if (lvglKeyboardShiftOneShot && lvglKeyboardIsLetterKey(txt)) {
                        lvglKeyboardShiftOneShot = false;
                        lvglKeyboardShiftLocked = false;
                        lvglKeyboardShiftLastPressMs = 0;
                        lvglKeyboardApplyTextMode();
                    }
                    if (!(lvglKeyboardFocusedTa &&
                          lv_obj_is_valid(lvglKeyboardFocusedTa) &&
                          lv_textarea_get_password_mode(lvglKeyboardFocusedTa))) {
                        lvglKeyboardEnsureFocusedVisible(false);
                    }
                    return;
                }
                if (code != LV_EVENT_READY && code != LV_EVENT_CANCEL) return;
                if (code == LV_EVENT_READY && lvglScreenLockSetupPinTa && lv_keyboard_get_textarea(lvglKb) == lvglScreenLockSetupPinTa) {
                    lvglScreenLockUnlockEvent(nullptr);
                } else if (code == LV_EVENT_READY && lvglScreenLockPinTa && lv_keyboard_get_textarea(lvglKb) == lvglScreenLockPinTa) {
                    lvglScreenLockUnlockEvent(nullptr);
                } else if (code == LV_EVENT_READY && lvglWifiPwdModal && !lv_obj_has_flag(lvglWifiPwdModal, LV_OBJ_FLAG_HIDDEN) &&
                    lvglWifiPwdTa && lv_keyboard_get_textarea(lvglKb) == lvglWifiPwdTa) {
                    lvglWifiPwdConnectEvent(nullptr);
                } else if (code == LV_EVENT_READY && lvglConfigDeviceNameTa && lv_keyboard_get_textarea(lvglKb) == lvglConfigDeviceNameTa) {
                    lvglSaveDeviceNameEvent(nullptr);
                } else if (code == LV_EVENT_READY && hc12CmdTaObj() && lv_keyboard_get_textarea(lvglKb) == hc12CmdTaObj()) {
                    lvglHc12SendEvent(nullptr);
                } else if (code == LV_EVENT_READY && lvglChatInputTa && lv_keyboard_get_textarea(lvglKb) == lvglChatInputTa) {
                    if (lvglChatPromptIfAirplaneBlocked()) return;
                    const char *raw = lv_textarea_get_text(lvglChatInputTa);
                    String text = raw ? String(raw) : String("");
                    text.trim();
                    if (!text.isEmpty()) {
                        if (currentChatPeerKey.isEmpty()) {
                            uiStatusLine = "Select a contact first";
                            lvglSyncStatusLine();
                            return;
                        }
                        chatSendAndStoreMessage(currentChatPeerKey, text);
                        lv_textarea_set_text(lvglChatInputTa, "");
                        lvglSetChatKeyboardVisible(false);
                    }
                }
                lvglHideKeyboard();
            },
            LV_EVENT_ALL,
            nullptr
        );
    }
    lvglKeyboardFocusedTa = ta;
    const bool numericMode = screenLockNumericTarget(ta);
    lvglKeyboardUseLocalized = numericMode ? false : lvglKeyboardLocalizedLayoutAvailable(uiLanguage);
    lvglKeyboardSpaceSwipeActive = false;
    lvglKeyboardSpaceSwipeTriggered = false;
    lvglKeyboardShiftOneShot = false;
    lvglKeyboardShiftLocked = false;
    lvglKeyboardShiftLastPressMs = 0;
    lv_keyboard_set_textarea(lvglKb, ta);
    if (numericMode) {
        lv_keyboard_set_mode(lvglKb, LV_KEYBOARD_MODE_NUMBER);
    } else {
        lvglKeyboardApplyTextMode();
    }
    lv_obj_clear_flag(lvglKb, LV_OBJ_FLAG_HIDDEN);
    lvglKeyboardRefreshLayout();
}

static unsigned long e220RuntimeBaud()
{
    const int maxIndex = static_cast<int>(sizeof(E220_UART_BAUD_VALUES) / sizeof(E220_UART_BAUD_VALUES[0])) - 1;
    const int idx = constrain(e220CurrentBaudIndex, 0, maxIndex);
    return E220_UART_BAUD_VALUES[idx];
}

static unsigned long hc12RuntimeBaud()
{
    const int maxIndex = static_cast<int>(sizeof(HC12_SUPPORTED_BAUDS) / sizeof(HC12_SUPPORTED_BAUDS[0])) - 1;
    const int idx = constrain(hc12CurrentBaudIndex, 0, maxIndex);
    return HC12_SUPPORTED_BAUDS[idx];
}

static void hc12SerialReopen(unsigned long baud)
{
    Serial1.flush();
    Serial1.end();
    delay(10);

    if (radioModuleType == RADIO_MODULE_HC12) {
        Serial1.begin(baud, SERIAL_8N1, hc12ActiveRxPin(), hc12ActiveTxPin());
    } else {
        Serial1.begin(baud, SERIAL_8N1, e220ActiveRxPin(), e220ActiveTxPin());
    }

    delay(10);
}

static void hc12InitIfNeeded()
{
    if (radioModuleType == RADIO_MODULE_E220 &&
        (e220ActiveRxPin() < 0 || e220ActiveTxPin() < 0 || e220ActiveM0Pin() < 0 || e220ActiveM1Pin() < 0)) {
        hc12ConfigStatusText = "E220 is only wired on the ESP32-S3 build";
        return;
    }

    if (hc12TerminalLog) return;

    if (radioModuleType == RADIO_MODULE_HC12) {
        pinMode(hc12ActiveSetPin(), OUTPUT);
        digitalWrite(hc12ActiveSetPin(), HIGH);
        hc12SerialReopen(hc12RuntimeBaud());
    } else {
        pinMode(e220ActiveM0Pin(), OUTPUT);
        pinMode(e220ActiveM1Pin(), OUTPUT);
        digitalWrite(e220ActiveM0Pin(), LOW);
        digitalWrite(e220ActiveM1Pin(), LOW);
        hc12SerialReopen(e220RuntimeBaud());
    }

    String banner = String("[RADIO] Serial1 ready on ESP RX") +
                    (radioModuleType == RADIO_MODULE_HC12 ? hc12ActiveRxPin() : e220ActiveRxPin()) +
                    " TX" +
                    (radioModuleType == RADIO_MODULE_HC12 ? hc12ActiveTxPin() : e220ActiveTxPin()) + "\n";

    if (radioModuleType == RADIO_MODULE_HC12) {
        banner += String("[HC12] HC12 RXD->GPIO") + hc12ActiveTxPin() +
                  " TXD->GPIO" + hc12ActiveRxPin() +
                  " SET->GPIO" + hc12ActiveSetPin() + "\n";
    } else {
        banner += String("[E220] RX->GPIO") + e220ActiveTxPin() +
                  " TX->GPIO" + e220ActiveRxPin() +
                  " M0->GPIO" + e220ActiveM0Pin() +
                  " M1->GPIO" + e220ActiveM1Pin() +
                  " runtime baud " + e220RuntimeBaud() +
                  " / cfg baud " + E220_AT_BAUD + "\n";
    }

    hc12AppendTerminal(banner.c_str());
}
