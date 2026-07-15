// For optitrack multi-object position logging by Chenyao

#include "modules/mission/moving_setup_logger_optitrack.h"

#include "generated/airframe.h"
#include "pprzlink/dl_protocol.h"
#include "mcu_periph/sys_time.h"

struct MovingSetupState moving_setup;


void moving_setup_logger_init(void)
{
  moving_setup.target_id = 0;
  moving_setup.target_timestamp = 0;
  moving_setup.last_rx_time = 0;
  moving_setup.valid = false;

  moving_setup.enu_x = 0.0f;
  moving_setup.enu_y = 0.0f;
  moving_setup.enu_z = 0.0f;

  moving_setup.enu_xd = 0.0f;
  moving_setup.enu_yd = 0.0f;
  moving_setup.enu_zd = 0.0f;
}


void moving_setup_logger_parse_MOVING_SETUP_POS(uint8_t *buf)
{
  const uint8_t msg_ac_id =
      DL_MOVING_SETUP_POS_ac_id(buf);

  if (msg_ac_id != AC_ID) {
    return;
  }

  moving_setup.target_id =
      DL_MOVING_SETUP_POS_target_id(buf);

  moving_setup.target_timestamp =
      DL_MOVING_SETUP_POS_timestamp(buf);

  moving_setup.last_rx_time =
      get_sys_time_msec();

  moving_setup.valid = true;

  moving_setup.enu_x =
      DL_MOVING_SETUP_POS_enu_x(buf);

  moving_setup.enu_y =
      DL_MOVING_SETUP_POS_enu_y(buf);

  moving_setup.enu_z =
      DL_MOVING_SETUP_POS_enu_z(buf);

  moving_setup.enu_xd =
      DL_MOVING_SETUP_POS_enu_xd(buf);

  moving_setup.enu_yd =
      DL_MOVING_SETUP_POS_enu_yd(buf);

  moving_setup.enu_zd =
      DL_MOVING_SETUP_POS_enu_zd(buf);
}