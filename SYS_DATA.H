#ifndef __SYS_DATA_H__
#define __SYS_DATA_H__
// sys
typedef struct SYS_DATA{
  volatile uint8_t heart_beat = 0;
  
  volatile float heating_panel_temp = 0.0;
  volatile float inside_temp = 0.0;
  volatile float inside_humi = 0.0;
  
  volatile double ax = 0;
  volatile double ay = 0;
  volatile double az = 0;
  volatile double gx = 0;
  volatile double gy = 0;
  volatile double gz = 0;
  volatile double mx = 0;
  volatile double my = 0;
  volatile double mz = 0;
  volatile float pitch = 0.0;
  volatile float roll = 0.0;
  volatile float yaw = 0.0;
  volatile float mag_heading = 0.0;
  volatile float vertical_accel = 0.0;
  
  volatile uint8_t gps_satellites = 0;
  volatile float gps_HDOP = 0.0;
  volatile float gps_latitude = 0.0;
  volatile float gps_longitude = 0.0;
  volatile float gps_gndspeed = 0.0;
  volatile uint32_t gps_altitude = 0;
  volatile uint16_t gps_distance = 0;
  
  volatile uint32_t baro_pressure = 0.0;
  volatile uint32_t baro_altitude = 0;
  volatile uint32_t baro_base_pressure = 0.0;

  volatile uint8_t uv_index = 0;
  volatile uint16_t lux = 0;

  volatile bool solar_panel_status = 0;
  volatile int16_t  left_solar_rod = 0;
  volatile int16_t right_solar_rod = 0;
  volatile float solar_panel_voltage = 0.0;

  volatile bool antena_status = 0;

  volatile int fm_transmit_frq = 0;
  volatile int fm_transmit_vol = 0;
}DATA_TYPE;

// sys_cmd
typedef struct SYS_CMD{
  volatile char factory_flag = false;
  volatile char factory_mode = 0; // 0:user;  1:servo;  2:UHF;  3:GPS;  4:BT;
  
  volatile char system_flag = false;
  volatile uint8_t heart_beat = 0;
  volatile char function_reset = false;
  volatile char system_reboot = false;

  volatile char ctrl_solar_flag = false;
  volatile int ctrl_solar_angle = 0;
  volatile char ctrl_solar_reset = false;

  volatile char ctrl_ant_flag = false;
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

  volatile char ctrl_cam_flag = false;
  volatile char ctrl_cam_cap = false; // 0(null) / 1(capture)

  volatile char set_rtc_flag = false;
  volatile int set_adjust_time_year = 0;
  volatile int set_adjust_time_month = 0;
  volatile int set_adjust_time_day = 0;
  volatile int set_adjust_time_hour = 0;
  volatile int set_adjust_time_minute = 0;
  
}CMD_TYPE;

extern DATA_TYPE sys;
extern CMD_TYPE sys_cmd;

#endif

