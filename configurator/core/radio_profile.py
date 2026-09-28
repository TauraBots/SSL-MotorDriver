"""Radio-link target profiles.

The rates in this module are scheduler targets, not claims about AirPort RF
throughput.  NORMAL intentionally retains the legacy 20/5 Hz behaviour.
"""

from dataclasses import dataclass

from .protocol import Protocol


@dataclass(frozen=True)
class RadioProfile:
    name: str
    display_name: str
    command_hz: float
    telemetry_fast_hz: float
    telemetry_full_hz: float
    split_telemetry: bool
    default_airport_baud: int
    telemetry_reply_guard: bool
    response_timeout_ms: int
    telemetry_phase_fraction: float = 0.0


NORMAL = RadioProfile("NORMAL", "NORMAL", 20, 5, 5, False, 9600, True, 250,
                      telemetry_phase_fraction=0.5)
TEST_50_20 = RadioProfile("TEST_50_20", "TEST 50/20", 50, 20, 10, True, 9600, False, 150)
TEST_100_50 = RadioProfile("TEST_100_50", "TEST 100/50", 100, 50, 10, True, 9600, False, 100)
TEST_120_60 = RadioProfile("TEST_120_60", "TEST 120/60", 120, 60, 10, True, 9600, False, 100)
VALIDATION_120 = RadioProfile(
    "VALIDATION_120", "VALIDATION 120 (EXPERIMENTAL)", 120, 120, 10,
    True, 9600, False, 100, telemetry_phase_fraction=0.5,
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


@dataclass(frozen=True)
class ProtocolTrafficEstimate:
    """Useful serial bytes only; excludes UART and ExpressLRS RF overhead."""

    estimated_protocol_tx_bytes_per_s: float
    estimated_protocol_rx_bytes_per_s: float
    command_frame_bytes: int
    telemetry_request_frame_bytes: int
    telemetry_fast_response_bytes: int
    telemetry_full_response_bytes: int


def estimate_protocol_traffic(profile):
    """Estimate one-robot protocol traffic for a target profile."""
    profile = get_radio_profile(profile)
    command_bytes = len(Protocol.encode_velocity("A", 0, 0.0, 0.0, 0.0))
    request_bytes = len(Protocol.encode_telemetry_request("A", 0))
    fast_bytes = (Protocol.TELEMETRY_RESPONSE_BASE_SIZE + 7 + 16)
    full_bytes = fast_bytes + 4 + 13
    tx_bytes_per_s = (profile.command_hz * command_bytes +
                      profile.telemetry_fast_hz * request_bytes)
    if profile.split_telemetry:
        full_hz = min(profile.telemetry_fast_hz, profile.telemetry_full_hz)
        fast_only_hz = max(0.0, profile.telemetry_fast_hz - full_hz)
        rx_bytes_per_s = fast_only_hz * fast_bytes + full_hz * full_bytes
    else:
        rx_bytes_per_s = profile.telemetry_fast_hz * full_bytes
    return ProtocolTrafficEstimate(
        tx_bytes_per_s, rx_bytes_per_s, command_bytes, request_bytes,
        fast_bytes, full_bytes,
    )


def airport_ota_capacity_message(profile, ota_capacity_bytes_per_s=None):
    """Return a non-blocking capacity note without inferring the ELRS RF mode."""
    if ota_capacity_bytes_per_s is None:
        return "AirPort OTA capacity unknown — validate experimentally."
    estimate = estimate_protocol_traffic(profile)
    if (estimate.estimated_protocol_tx_bytes_per_s > ota_capacity_bytes_per_s or
            estimate.estimated_protocol_rx_bytes_per_s > ota_capacity_bytes_per_s):
        return ("Target protocol traffic exceeds the configured/estimated AirPort OTA "
                "throughput. Measured rates may be substantially lower.")
    return ("Target traffic is below the supplied OTA estimate; physical validation "
            "is still required.")
