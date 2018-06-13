#ifndef __SYS_DATA_H__
#define __SYS_DATA_H__
// sys
typedef struct SYS_DATA{
  float heating_panel_temp = 0.0;
  float inside_temp = 0.0;
  float inside_humi = 0.0;
  
  int ax = 0;
  int ay = 0;
  int az = 0;
  int gx = 0;
  int gy = 0;
  int gz = 0;
  int mx = 0;
  int my = 0;
  int mz = 0;
  float pitch = 0.0;
  float roll = 0.0;
  float yaw = 0.0;
  float mag_heading = 0.0;
  float vertical_accel = 0.0;
  
  uint8_t gps_satellites = 0;
  float gps_HDOP = 0.0;
  float gps_latitude = 0.0;
  float gps_longitude = 0.0;
  float gps_gndspeed = 0.0;
  uint32_t gps_altitude = 0;
  uint16_t gps_distance = 0;
  
  double baro_pressure = 0.0;
  uint32_t baro_altitude = 0;
  double baro_base_pressure = 0.0;

  uint8_t uv_index = 0;
  uint16_t lux = 0;

  bool solar_panel_status = 0;
  int16_t  left_solar_rod = 0;
  int16_t right_solar_rod = 0;
  float solar_panel_voltage = 0.0;

  bool antena_status = 0;

  float fm_transmit_frq = 0.0;
}DATA_TYPE;

// sys_cmd
typedef struct SYS_CMD{
  volatile char system_flag = false;
  volatile uint8_t heart_beat = 0;
  volatile char function_reset = false;
  volatile char system_reboot = false;

  volatile char ctrl_flag = false;
  volatile int ctrl_solar_angle = 0;
  volatile char ctrl_solar_reset = false;
  volatile char ctrl_ant = false;

  volatile char set_fm_flag = false;
  volatile char set_fm_pwr = false;
  volatile float set_fm_frq = 83.5;
  volatile uint8_t set_fm_vol = 15;

  volatile char set_uhf_flag = false;
  volatile uint8_t set_uhf_bud = 115;
  volatile uint8_t set_uhf_ch = 1;

  volatile char set_mled_flag = false;
  volatile uint8_t set_mled_num = 0;
  volatile uint16_t set_mled_rgb = 0x00FF;
  volatile uint8_t set_mled[8] = {0x00};
  volatile char set_mled_mode = false; // 0(normal) / 1(user)
}CMD_TYPE;

extern DATA_TYPE sys;
extern CMD_TYPE sys_cmd;

#endif

