// For optitrack multi-object position logging by Chenyao


#ifndef MOVING_SETUP_LOGGER_OPTITRACK_H
#define MOVING_SETUP_LOGGER_OPTITRACK_H

#include <stdint.h>
#include <stdbool.h>

struct MovingSetupState {
  uint16_t target_id;

  /*
   * Timestamp from natnet2ivy / OptiTrack side, in ms.
   */
  uint32_t target_timestamp;

  /*
   * Onboard time when the Bebop received the latest setup message.
   */
  uint32_t last_rx_time;

  bool valid;

  float enu_x;
  float enu_y;
  float enu_z;

  float enu_xd;
  float enu_yd;
  float enu_zd;
};

extern struct MovingSetupState moving_setup;

void moving_setup_logger_init(void);

/*
 * Datalink parser callback.
 * This function receives MOVING_SETUP_POS from natnet2ivy.
 */
void moving_setup_logger_parse_MOVING_SETUP_POS(uint8_t *buf);

#endif /* MOVING_SETUP_LOGGER_OPTITRACK_H */