/*
 * Copyright 2024 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "board/board.h"
#include "console/prompt.h"
#include "drivers/ambient_light.h"
#include "drivers/i2c.h"
#include "drivers/periph_config.h"
#include "kernel/util/sleep.h"
#include "mfg/mfg_info.h"
#include "system/logging.h"
#include "system/passert.h"

#include <inttypes.h>

static uint32_t s_sensor_light_dark_threshold;
static bool s_initialized = false;

static bool prv_read_register(uint8_t register_address, uint16_t *result) {
  uint8_t buf[2];
  i2c_use(I2C_OPT3001);
  bool rv = i2c_read_register_block(I2C_OPT3001, register_address, 2, buf);
  *result = buf[0] << 8 | buf[1];
  i2c_release(I2C_OPT3001);
  return rv;
}

static bool prv_write_register(uint8_t register_address, uint16_t datum) {
  i2c_use(I2C_OPT3001);
  uint8_t block[3] = { register_address, datum >> 8, datum & 0xFF };
  bool rv = i2c_write_block(I2C_OPT3001, 3, block);
  i2c_release(I2C_OPT3001);
  return rv;
}

static uint32_t prv_get_default_ambient_light_dark_threshold(void) {
  PBL_ASSERTN(BOARD_CONFIG.ambient_light_dark_threshold != 0);
  return BOARD_CONFIG.ambient_light_dark_threshold;
}

void ambient_light_init(void) {
  s_sensor_light_dark_threshold = prv_get_default_ambient_light_dark_threshold();

  uint16_t mf, id;
  bool found = prv_read_register(0x7E, &mf) && prv_read_register(0x7F, &id);
  if (found) {
    PBL_LOG(LOG_LEVEL_INFO, "found OPT3001 with manuf %04x, id %04x", mf, id);
  } else {
    PBL_LOG(LOG_LEVEL_ERROR, "failed to read OPT3001 ID registers");
  }

  if (BOARD_CONFIG.als_always_on) {
    prv_write_register(0x01 /* CONFIG */, 0xC610 /* autorange, continuous */);
  }

  s_initialized = true;
}

uint32_t ambient_light_get_light_level(void) {
  if (!s_initialized) {
    return BOARD_CONFIG.ambient_light_dark_threshold;
  }
  
  if (!BOARD_CONFIG.als_always_on) {
    prv_write_register(0x01 /* CONFIG */, 0xC210 /* autorange, singleshot */);
    /* we will get a result delayed by one conversion */
  }

  uint16_t result;
  prv_read_register(0x00, &result);
  uint32_t exp = result >> 12;
  uint32_t mant = result & 0xFFF;

  uint32_t level = mant << exp;

  return level;
}

void command_als_read(void) {
  char buffer[16];
  prompt_send_response_fmt(buffer, sizeof(buffer), "%"PRIu32"", ambient_light_get_light_level());
}

uint32_t ambient_light_get_dark_threshold(void) {
  return s_sensor_light_dark_threshold;
}

void ambient_light_set_dark_threshold(uint32_t new_threshold) {
  PBL_ASSERTN(new_threshold <= AMBIENT_LIGHT_LEVEL_MAX);
  s_sensor_light_dark_threshold = new_threshold;
}

bool ambient_light_is_light(void) {
  // if the sensor is not enabled, always return that it is dark
  return s_initialized && ambient_light_get_light_level() > s_sensor_light_dark_threshold;
}

AmbientLightLevel ambient_light_level_to_enum(uint32_t light_level) {
  if (!s_initialized) {
    // if the sensor is not enabled, always return that it is very dark
    return AmbientLightLevelUnknown;
  }

  const uint32_t k_delta_threshold = BOARD_CONFIG.ambient_k_delta_threshold;
  if (light_level < (s_sensor_light_dark_threshold - k_delta_threshold)) {
    return AmbientLightLevelVeryDark;
  } else if (light_level < s_sensor_light_dark_threshold) {
    return AmbientLightLevelDark;
  } else if (light_level < (s_sensor_light_dark_threshold + k_delta_threshold)) {
    return AmbientLightLevelLight;
  } else {
    return AmbientLightLevelVeryLight;
  }
}
