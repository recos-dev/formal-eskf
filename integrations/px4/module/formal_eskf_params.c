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
