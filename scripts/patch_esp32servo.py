import os
import pathlib

from SCons.Script import DefaultEnvironment

env = DefaultEnvironment()


def _patch_esp32servo():
    libdir = pathlib.Path(env.subst("$PROJECT_LIBDEPS_DIR")) / env.get("PIOENV", "")
    header = libdir / "ESP32Servo" / "src" / "ESP32PWM.h"
    if not header.exists():
        print("[patch_esp32servo] ESP32Servo not present; skipping patch")
        return

    text = header.read_text()
    marker = "SOC_LEDC_CHANNEL_NUM"
    if marker in text:
        print("[patch_esp32servo] Patch already applied")
        return

    needle = "#define NUM_PWM 16"
    replacement = "\n".join(
        [
            "#ifdef SOC_LEDC_CHANNEL_NUM",
            "#define NUM_PWM SOC_LEDC_CHANNEL_NUM",
            "#else",
            "#define NUM_PWM 16",
            "#endif",
        ]
    )

    if needle not in text:
        print("[patch_esp32servo] NUM_PWM define not found; skipping")
        return

    header.write_text(text.replace(needle, replacement, 1))
    print("[patch_esp32servo] Patched ESP32Servo to use SOC_LEDC_CHANNEL_NUM")


_patch_esp32servo()
