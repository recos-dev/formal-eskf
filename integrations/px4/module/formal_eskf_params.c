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
