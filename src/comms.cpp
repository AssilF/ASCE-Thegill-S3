#include "comms.h"
#include <cstring>
#include <ctype.h>

namespace Comms {
static bool g_paired = false;
static ThegillCommand lastCmd = {};
static uint8_t controllerMac[6] = {0};
const uint8_t BroadcastMac[6] = {0xff,0xff,0xff,0xff,0xff,0xff};

static bool identityContains(const IdentityMessage &msg, const char *needle)
{
    const size_t maxLen = sizeof(msg.identity);
    const size_t needleLen = strlen(needle);
    if (needleLen == 0 || needleLen > maxLen)
        return false;

    for (size_t start = 0; start + needleLen <= maxLen; ++start)
    {
        if (msg.identity[start] == '\0')
            break;

        bool match = true;
        for (size_t i = 0; i < needleLen; ++i)
        {
            size_t idx = start + i;
            if (idx >= maxLen)
            {
                match = false;
                break;
            }
            char actual = msg.identity[idx];
            if (actual == '\0')
            {
                match = false;
                break;
            }
            char expected = needle[i];
            actual = toupper(static_cast<unsigned char>(actual));
            expected = toupper(static_cast<unsigned char>(expected));
            if (actual != expected)
            {
                match = false;
                break;
            }
        }
        if (match)
            return true;
    }
    return false;
}

static bool isEliteControllerIdentity(const IdentityMessage &msg)
{
    if (msg.type == ILITE_IDENTITY || msg.type == ELITE_IDENTITY)
        return true;
    if (identityContains(msg, "ELITE"))
        return true;
    if (identityContains(msg, "ILITE"))
        return true;
    return false;
}

static void onDataRecv(const uint8_t* mac, const uint8_t* incomingData, int len) {
    if (len == sizeof(IdentityMessage) && !g_paired) {
        const IdentityMessage* msg = reinterpret_cast<const IdentityMessage*>(incomingData);
        if (msg->type == SCAN_REQUEST) {
            IdentityMessage resp{};
            resp.type = DRONE_IDENTITY;
            strncpy(resp.identity, "THEGILL", sizeof(resp.identity));
            WiFi.macAddress(resp.mac);
            esp_now_send(mac, reinterpret_cast<const uint8_t*>(&resp), sizeof(resp));
            return;
        } else if (isEliteControllerIdentity(*msg)) {
            memcpy(controllerMac, mac, 6);
            if (!esp_now_is_peer_exist(mac)) {
                esp_now_peer_info_t peerInfo{};
                memcpy(peerInfo.peer_addr, mac, 6);
                peerInfo.channel = 0;
                peerInfo.encrypt = false;
                esp_now_add_peer(&peerInfo);
            }
            IdentityMessage ack{};
            ack.type = DRONE_ACK;
            esp_now_send(mac, reinterpret_cast<const uint8_t*>(&ack), sizeof(ack));
            g_paired = true;
            return;
        }
    }
    if (len == sizeof(ThegillCommand)) {
        const ThegillCommand* cmd = reinterpret_cast<const ThegillCommand*>(incomingData);
        lastCmd = *cmd;
        return;
    }
}

static bool initInternal(const char *ssid, const char *password, int tcpPort, esp_now_recv_cb_t recvCallback) {
    (void)tcpPort;
    // Run in AP+STA mode so ESP-NOW remains operational alongside SoftAP
    WiFi.mode(WIFI_AP_STA);
    WiFi.setTxPower(WIFI_POWER_19_5dBm);
    WiFi.setSleep(false);
    WiFi.softAP(ssid, password);

    if (esp_now_init() != ESP_OK) {
        return false;
    }

    esp_now_peer_info_t peerInfo{};
    memcpy(peerInfo.peer_addr, BroadcastMac, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    if (!esp_now_is_peer_exist(BroadcastMac)) {
        esp_now_add_peer(&peerInfo);
    }

    esp_now_register_recv_cb(recvCallback ? recvCallback : onDataRecv);

    g_paired = false;
    memset(controllerMac, 0, sizeof(controllerMac));
    memset(&lastCmd, 0, sizeof(lastCmd));
    return true;
}

bool init(const char *ssid, const char *password, int tcpPort) {
    return initInternal(ssid, password, tcpPort, nullptr);
}

bool init(const char *ssid, const char *password, int tcpPort, esp_now_recv_cb_t recvCallback) {
    return initInternal(ssid, password, tcpPort, recvCallback);
}

bool receiveCommand(ThegillCommand &cmd) {
    cmd = lastCmd;
    return g_paired;
}

bool paired() {
    return g_paired;
}
}

