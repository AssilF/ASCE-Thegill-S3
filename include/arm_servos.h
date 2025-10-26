#pragma once

#include <cstdint>

namespace ArmServos {

enum class ServoId : uint8_t {
    Shoulder = 0,
    Elbow,
    Pitch,
    Roll,
    Yaw,
    Count
};

struct ServoStatus {
    ServoId id;
    float targetDegrees;
    float minDegrees;
    float maxDegrees;
    bool enabled;
    bool attached;
};

void init();
void setEnabled(bool enabled);
bool enabled();

bool setTargetDegrees(ServoId id, float degrees);
bool setTargetNormalized(ServoId id, float normalized);

float targetDegrees(ServoId id);
float normalizedTarget(ServoId id);

ServoStatus status(ServoId id);
const char *name(ServoId id);

} // namespace ArmServos
