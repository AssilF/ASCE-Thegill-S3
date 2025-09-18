#pragma once
#include <Arduino.h>

namespace Commands {
void sendLine(const String &line);
void sendHeartbeat();
void handleCommand(const String &cmd);
}
