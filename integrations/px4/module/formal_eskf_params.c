#include <px4_platform_common/param.h>

/**
 * Enable the formal-eskf SITL proof of concept
 *
 * Selects the single-instance INS adapter instead of EKF2 at boot.
 * Requires SENS_IMU_MODE=1. Restart required; SITL only.
 *
 * @boolean
 * @reboot_required true
 * @group Formal ESKF
 */
PARAM_DEFINE_INT32(FESKF_EN, 1);

/**
 * Magnetic declination sources
 *
 * Bit 0 uses geographic declination when available. Bit 1 permits the
 * configured FESKF_MAG_DECL fallback; otherwise magnetic north is used.
 * WMM remains an independent field-health reference in every mode.
 * Settings are read at module start. No learned-declination autosave.
 *
 * @bit 0 Geographic heading
 * @bit 1 Stored declination fallback
 * @min 0
 * @max 3
 * @reboot_required true
 * @group Formal ESKF
 */
PARAM_DEFINE_INT32(FESKF_DECL_TYPE, 3);

/**
 * Stored magnetic declination
 *
 * East-positive declination used when allowed by FESKF_DECL_TYPE and no
 * selected geographic heading is available. Zero selects magnetic north.
 * Settings are read at module start; the estimator does not overwrite this value.
 *
 * @unit deg
 * @min -180
 * @max 180
 * @decimal 1
 * @reboot_required true
 * @group Formal ESKF
 */
PARAM_DEFINE_FLOAT(FESKF_MAG_DECL, 0.0f);

/**
 * Enable gravity vector aiding
 *
 * Uses the existing unnormalized gravity observation only when tilt is aligned,
 * horizontal aiding is inactive and gravity-band/rest conditions permit it.
 * Clipped intervals are rejected. Read at module start.
 *
 * @boolean
 * @reboot_required true
 * @group Formal ESKF
 */
PARAM_DEFINE_INT32(FESKF_GRAV_EN, 1);

/**
 * Main filter interval
 *
 * Only the 250 Hz one-interval mode (4000 us) is implemented.
 * Unsupported combinations are rejected at startup. Read once; restart required.
 *
 * @unit us
 * @min 0
 * @reboot_required true
 * @group Formal ESKF
 */
PARAM_DEFINE_INT32(FESKF_DT_US, 4000);

/**
 * Minimum fusion delay
 *
 * The fixed history length is derived from this lag. Initial profile: 153 to 239 ms with zero source delays.
 * Unsupported combinations are rejected at startup. Read once; restart required.
 *
 * @unit ms
 * @min 0
 * @reboot_required true
 * @group Formal ESKF
 */
PARAM_DEFINE_INT32(FESKF_DELAY_MS, 200);

/**
 * GNSS residual delay
 *
 * Subtracted from the GNSS sample timestamp in addition to half a filter period.
 * Unsupported combinations are rejected at startup. Read once; restart required.
 *
 * @unit ms
 * @min 0
 * @reboot_required true
 * @group Formal ESKF
 */
PARAM_DEFINE_INT32(FESKF_GPS_DLY, 0);

/**
 * Magnetometer residual delay
 *
 * Subtracted from the magnetometer sample timestamp in addition to half a filter period.
 * Unsupported combinations are rejected at startup. Read once; restart required.
 *
 * @unit ms
 * @min 0
 * @reboot_required true
 * @group Formal ESKF
 */
PARAM_DEFINE_INT32(FESKF_MAG_DLY, 0);

/**
 * Barometer residual delay
 *
 * Subtracted from the barometer sample timestamp in addition to half a filter period.
 * Unsupported combinations are rejected at startup. Read once; restart required.
 *
 * @unit ms
 * @min 0
 * @reboot_required true
 * @group Formal ESKF
 */
PARAM_DEFINE_INT32(FESKF_BARO_DLY, 0);
