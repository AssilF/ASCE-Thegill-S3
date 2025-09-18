#include "comms.h"
#include <cstring>

namespace Comms {
static bool g_paired = false;
static DriveCommand lastCmd = {0};
static uint8_t controllerMac[6] = {0};
const uint8_t BroadcastMac[6] = {0xff,0xff,0xff,0xff,0xff,0xff};


static void onDataRecv(const uint8_t* mac, const uint8_t* incomingData, int len) {
    if (len == sizeof(IdentityMessage) && !g_paired) {
        const IdentityMessage* msg = reinterpret_cast<const IdentityMessage*>(incomingData);
        if (msg->type == SCAN_REQUEST) {
            IdentityMessage resp{};
            resp.type = DRONE_IDENTITY;
            strncpy(resp.identity, "DRONEGAZE", sizeof(resp.identity));
            WiFi.macAddress(resp.mac);
            esp_now_send(mac, reinterpret_cast<const uint8_t*>(&resp), sizeof(resp));
            return;
        } else if (msg->type == ILITE_IDENTITY) {
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
    if (len == sizeof(ThrustCommand)) {
        const DriveCommand* cmd = reinterpret_cast<const DriveCommand*>(incomingData);
        lastCmd = *cmd;
        return;
    }
}

void init(const char *ssid, const char *password, int tcpPort) {
    // Run in AP+STA mode so ESP-NOW remains operational alongside SoftAP
    WiFi.mode(WIFI_AP_STA);
    WiFi.setTxPower(WIFI_POWER_8_5dBm);
    WiFi.setSleep(false);
    WiFi.softAP(ssid, password);

    esp_now_init();

    esp_now_peer_info_t peerInfo{};
    memcpy(peerInfo.peer_addr, BroadcastMac, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    esp_now_add_peer(&peerInfo);

    g_paired = false;
}

bool receiveCommand(ThrustCommand &cmd) {
    cmd = lastCmd;
    return g_paired;
}

bool paired() {
    return g_paired;
}
}

