#include "thegill.h"
#include <math.h>

float applyEasingCurve(GillEasing mode, float t)
{
    if (t <= 0.f) return 0.f;
    if (t >= 1.f) return 1.f;
    switch (mode) {
        case GillEasing::Linear:
            return t;
        case GillEasing::EaseIn:
            return t * t;
        case GillEasing::EaseOut:
            return 1.f - powf(1.f - t, 2.f);
        case GillEasing::EaseInOut:
            if (t < 0.5f) {
                return 2.f * t * t;
            }
            return 1.f - powf(-2.f * t + 2.f, 2.f) / 2.f;
        case GillEasing::Sine:
        default:
            return sinf((t * PI) / 2.f);
    }
}

