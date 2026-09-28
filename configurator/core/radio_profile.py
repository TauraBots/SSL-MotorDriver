"""Radio-link target profiles.

The rates in this module are scheduler targets, not claims about AirPort RF
throughput.  NORMAL intentionally retains the legacy 20/5 Hz behaviour.
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class RadioProfile:
    name: str
    display_name: str
    command_hz: float
    telemetry_fast_hz: float
    telemetry_full_hz: float
    split_telemetry: bool
    recommended_baud: int
    telemetry_reply_guard: bool
    response_timeout_ms: int
    telemetry_phase_fraction: float = 0.0


NORMAL = RadioProfile("NORMAL", "NORMAL", 20, 5, 5, False, 9600, True, 250,
                      telemetry_phase_fraction=0.5)
TEST_50_20 = RadioProfile("TEST_50_20", "TEST 50/20", 50, 20, 10, True, 921600, False, 150)
TEST_100_50 = RadioProfile("TEST_100_50", "TEST 100/50", 100, 50, 10, True, 921600, False, 100)
TEST_120_60 = RadioProfile("TEST_120_60", "TEST 120/60", 120, 60, 10, True, 921600, False, 100)
VALIDATION_120 = RadioProfile(
    "VALIDATION_120", "VALIDATION 120 (EXPERIMENTAL)", 120, 120, 10,
    True, 921600, False, 100, telemetry_phase_fraction=0.5,
)

RADIO_PROFILES = (NORMAL, TEST_50_20, TEST_100_50, TEST_120_60, VALIDATION_120)
RADIO_PROFILES_BY_NAME = {profile.name: profile for profile in RADIO_PROFILES}


def get_radio_profile(value):
    """Resolve a profile object, stable name, or UI display name."""
    if isinstance(value, RadioProfile):
        return value
    for profile in RADIO_PROFILES:
        if value in (profile.name, profile.display_name):
            return profile
    raise ValueError(f"Unknown radio profile: {value}")
