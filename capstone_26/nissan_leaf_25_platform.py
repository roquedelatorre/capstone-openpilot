# =============================================================================
# NISSAN_LEAF_2025 platform entry
# Add this to opendbc/car/nissan/values.py inside `class CAR(Platforms):`
#
# Confirmed inherited from NISSAN_LEAF (no change needed):
#   - bus topology / parser map (pt=0, cam=1, adas=2)   [relay-open probe]
#   - safety config: nissan, ALT_EPS_BUS = false         [interface.py default]
#   - control: angle, 0x169 LKAS, CRC8/J1850 checksum
#   - all 16 required messages present at correct lengths [fingerprint]
#
# Needs car-specific verification (marked TODO):
#   - wheelbase / steerRatio: the 2025 is a new generation; the 2.705 m / 17.0
#     values are the 2018-23 ZE1 figures. These affect LATERAL TUNING only, not
#     safety (safety is angle/rate based). Verify against the 2025 spec before
#     tuning, but they're fine to start replay with.
#   - FW versions: see note below. Not needed for offline replay / forced
#     fingerprint, needed for robust auto-fingerprinting.
# =============================================================================

NISSAN_LEAF_2025 = NissanPlatformConfig(
    [NissanCarDocs("Nissan Leaf 2025")],
    NissanCarSpecs(mass=1610, wheelbase=2.705),          # TODO: verify for 2025 gen
    {Bus.pt: 'nissan_leaf_2018_generated'},   # start from stock DBC; override only confirmed-different signals
  )


# =============================================================================
# FW versions
# -----------------------------------------------------------------------------
# Current opendbc nissan/fingerprints.py is FW-only (no message fingerprint
# dict). To make NISSAN_LEAF_2025 auto-fingerprint, capture the 2025's ECU FW
# versions and add a FW_VERSIONS[CAR.NISSAN_LEAF_2025] block mirroring the
# NISSAN_LEAF structure (abs 0x740, eps 0x742, fwdCamera 0x707, gateway
# 0x18dad0f1).
#
# Until then, to run OFFLINE REPLAY / CarState validation now, force the
# fingerprint instead of relying on FW matching — set the car explicitly so
# openpilot skips fingerprinting. (Exact mechanism depends on your openpilot
# version; the dev path is the FINGERPRINT / "SKIP_FW_QUERY" style override, or
# replaying with --car. Ask and we'll nail the exact command for your tree.)
# =============================================================================