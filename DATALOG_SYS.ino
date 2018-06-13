#include <SPI.h>
#include <SD.h>
#include <stdio.h>

void SDlogger_handle(void) {
char logBuf[1000];
char heating_panel_temp_sBuf[20];
char inside_temp_sBuf[20];
char inside_humi_sBuf[20];
char pitch_sBuf[20];
char roll_sBuf[20];
char yaw_sBuf[20];
char mag_heading_sBuf[20];
char gps_latitude_sBuf[20];
char gps_longitude_sBuf[20];
char gps_altitude_sBuf[20];
char baro_altitude_sBuf[20];
char baro_pressure_sBuf[20];
char gps_gndspeed_sBuf[20];
char vertical_accel_sBuf[20]; 
  int j = 0;
  // open the file. note that only one file can be open at a time.
  File dataFile = SD.open("datalog.txt", FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    dtostrf(sys.heating_panel_temp, 1, 1, heating_panel_temp_sBuf);
    dtostrf(sys.inside_temp, 1, 1, inside_temp_sBuf);
    dtostrf(sys.inside_humi, 1, 1, inside_humi_sBuf);
    dtostrf(sys.pitch, 1, 1, pitch_sBuf);
    dtostrf(sys.roll, 1, 1, roll_sBuf);
    dtostrf(sys.yaw, 1, 1, yaw_sBuf);
    dtostrf(sys.mag_heading, 1, 1, mag_heading_sBuf);
    dtostrf(sys.gps_latitude, 1, 4, gps_latitude_sBuf);
    dtostrf(sys.gps_longitude,1, 4, gps_longitude_sBuf);
    dtostrf(sys.gps_altitude, 1, 1, gps_altitude_sBuf);
    dtostrf(sys.baro_altitude, 1, 1, baro_altitude_sBuf);
    dtostrf(sys.baro_pressure, 1, 3, baro_pressure_sBuf);
    dtostrf(sys.gps_gndspeed, 1, 2, gps_gndspeed_sBuf);
    dtostrf(sys.vertical_accel, 1, 2, vertical_accel_sBuf);
    j = sprintf(logBuf, "{");
    j += sprintf(logBuf+j, "\"OUT_TP\":{\"v\":%s},", heating_panel_temp_sBuf);
    j += sprintf(logBuf+j, "\"IN_TP\":{\"v\":%s},", inside_temp_sBuf);
    j += sprintf(logBuf+j, "\"IN_HM\":{\"v\":%s},", inside_humi_sBuf);
    j += sprintf(logBuf+j, "\"ROL\":{\"v\":%s},", roll_sBuf);
    j += sprintf(logBuf+j, "\"PIT\":{\"v\":%s},", pitch_sBuf);
    j += sprintf(logBuf+j, "\"YAW\":{\"v\":%s},", yaw_sBuf);
    j += sprintf(logBuf+j, "\"HEAD\":{\"v\":%s},", mag_heading_sBuf);
    j += sprintf(logBuf+j, "\"G_SAT\":{\"v\":%d},", sys.gps_satellites);
    j += sprintf(logBuf+j, "\"LAT\":{\"v\":%s},", gps_latitude_sBuf);
    j += sprintf(logBuf+j, "\"LON\":{\"v\":%s},", gps_longitude_sBuf);
    j += sprintf(logBuf+j, "\"G_ALT\":{\"v\":%s},", gps_altitude_sBuf);
    j += sprintf(logBuf+j, "\"B_ALT\":{\"v\":%s},", baro_altitude_sBuf);
    j += sprintf(logBuf+j, "\"BARO\":{\"v\":%s},", baro_pressure_sBuf);
    j += sprintf(logBuf+j, "\"G_SPD\":{\"v\":%s},", gps_gndspeed_sBuf);
    j += sprintf(logBuf+j, "\"ACC_G\":{\"v\":%s},", vertical_accel_sBuf);
    j += sprintf(logBuf+j, "\"G_DIS\":{\"v\":%d},", sys.gps_distance);
    j += sprintf(logBuf+j, "}");
    dataFile.println(logBuf);
    dataFile.close();
    Serial.println(logBuf);
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
  }
}
