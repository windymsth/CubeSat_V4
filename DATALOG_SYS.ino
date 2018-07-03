#include <SPI.h>
#include <SD.h>
#include <stdio.h>

const int logchipSelect = 5;

File dataFile;

static struct pt pt_datalog_driver;
static struct pt pt_datalog_record_task;

//  Called by "CubeSat_V4"->"setup()"->first call be uesd to init this system.
void DataLog_System_Init() {
  SDlogger_init();

  PT_INIT(&pt_datalog_driver);
}

void TASK_DataLog_Handle() {
  
  thread_datalog_driver(&pt_datalog_driver);
  
  thread_datalog_record_task(&pt_datalog_record_task);
}

static int thread_datalog_driver(struct pt *pt) {
  PT_BEGIN(pt);
  
  PT_INIT(&pt_datalog_record_task);

  PT_WAIT_THREAD(pt, 
         thread_datalog_record_task(&pt_datalog_record_task));
  PT_END(pt);
}

void SDlogger_init(void) {
  Serial.println("Initializing SD card...");
  // see if the card is present and can be initialized:
  if (!SD.begin(logchipSelect)) {
    Serial.println("Logsystem Card failed, or not present");
    return;
  }
  Serial.println("card initialized.\r\n");
  Serial.println("DNL checks for datalog.txt files...");
  if (SD.exists("datalog.txt")) {
    Serial.println("datalog.txt exists.");
    // delete the file:
    Serial.println("Removing datalog.txt...");
    SD.remove("datalog.txt");
  } else {
    Serial.println("datalog.txt doesn't exist.");
  }
  // creat a new file and immediately close it:
  Serial.println("\nCreating new datalog.txt...\r\n");
  dataFile = SD.open("datalog.txt", FILE_WRITE);
  dataFile.close();
}

// Waste lots of ram, *** need optimize
static int thread_datalog_record_task(struct pt *pt) {
  
  static char logBuf[200];
  
  PT_BEGIN(pt);
  while(1) {
    static int j = 0;
    // open the file. note that only one file can be open at a time.
    dataFile = SD.open("datalog.txt", FILE_WRITE);
  
    // if the file is available, write to it:
    if (dataFile) {
//      dtostrf(sys.heating_panel_temp, 1, 1, heating_panel_temp_sBuf);
//      dtostrf(sys.inside_temp, 1, 1, inside_temp_sBuf);
//      dtostrf(sys.inside_humi, 1, 1, inside_humi_sBuf);
//      dtostrf(sys.pitch, 1, 1, pitch_sBuf);
//      dtostrf(sys.roll, 1, 1, roll_sBuf);
//      dtostrf(sys.yaw, 1, 1, yaw_sBuf);
//      dtostrf(sys.mag_heading, 1, 1, mag_heading_sBuf);
//      dtostrf(sys.gps_latitude, 1, 4, gps_latitude_sBuf);
//      dtostrf(sys.gps_longitude,1, 4, gps_longitude_sBuf);
//      dtostrf(sys.gps_altitude, 1, 1, gps_altitude_sBuf);
//      dtostrf(sys.baro_altitude, 1, 1, baro_altitude_sBuf);
//      dtostrf(sys.baro_pressure, 1, 3, baro_pressure_sBuf);
//      dtostrf(sys.gps_gndspeed, 1, 2, gps_gndspeed_sBuf);
//      dtostrf(sys.vertical_accel, 1, 2, vertical_accel_sBuf);
      j = sprintf(logBuf, "{");
      j += sprintf(logBuf+j, "\"OUT_TP\":{\"v\":%d.%1d},", (int)sys.heating_panel_temp, int( (sys.heating_panel_temp-(int)sys.heating_panel_temp) *10 ));
      j += sprintf(logBuf+j, "\"IN_TP\":{\"v\":%d.%1d},", (int)sys.inside_temp, int( (sys.inside_temp-(int)sys.inside_temp) *10 ));
      j += sprintf(logBuf+j, "\"IN_HM\":{\"v\":%d.%1d},", (int)sys.inside_humi, int( (sys.inside_humi-(int)sys.inside_humi) *10 ));
      j += sprintf(logBuf+j, "\"ROL\":{\"v\":%d.%1d},", (int)sys.roll, int( (sys.roll-(int)sys.roll) *10 ));
      j += sprintf(logBuf+j, "\"PIT\":{\"v\":%d.%1d},", (int)sys.pitch, int( (sys.pitch-(int)sys.pitch) *10 ));
      j += sprintf(logBuf+j, "\"YAW\":{\"v\":%d.%1d},", (int)sys.yaw, int( (sys.yaw-(int)sys.yaw) *10 ));
      j += sprintf(logBuf+j, "\"HEAD\":{\"v\":%d.%1d},", (int)sys.mag_heading, int( (sys.mag_heading-(int)sys.mag_heading) *10 ));
      j += sprintf(logBuf+j, "\"G_SAT\":{\"v\":%d},", sys.gps_satellites);
      j += sprintf(logBuf+j, "\"LAT\":{\"v\":%d.%4d},", (long)sys.gps_latitude, long( (sys.gps_latitude-(long)sys.gps_latitude) *10000 ));
      j += sprintf(logBuf+j, "\"LON\":{\"v\":%d.%4d},", (long)sys.gps_longitude, long( (sys.gps_longitude-(long)sys.gps_longitude) *10000 ));
      j += sprintf(logBuf+j, "\"G_ALT\":{\"v\":%d.%1d},", (int)sys.gps_altitude, int( (sys.gps_altitude-(int)sys.gps_altitude) *10 ));
      j += sprintf(logBuf+j, "\"B_ALT\":{\"v\":%d.%1d},", (int)sys.baro_altitude, int( (sys.baro_altitude-(int)sys.baro_altitude) *10 ));
      j += sprintf(logBuf+j, "\"BARO\":{\"v\":%d.%3d},", (int)sys.baro_pressure, int( (sys.baro_pressure-(int)sys.baro_pressure) *1000 ));
      j += sprintf(logBuf+j, "\"G_SPD\":{\"v\":%d.%2d},", (int)sys.gps_gndspeed, int( (sys.gps_gndspeed-(int)sys.gps_gndspeed) *100 ));
      j += sprintf(logBuf+j, "\"ACC_G\":{\"v\":%d.%2d},", (int)sys.vertical_accel, int( (sys.vertical_accel-(int)sys.vertical_accel) *100 ));
      j += sprintf(logBuf+j, "\"G_DIS\":{\"v\":%d},", sys.gps_distance);
      j += sprintf(logBuf+j, "}");
      dataFile.println(logBuf);
      dataFile.close();
//      Serial.println(logBuf);
    }
    // if the file isn't open, pop up an error:
    else {
      Serial.println("error opening datalog.txt");
    }

    PT_TIMER_DELAY(pt, 5000);
  }// end while(1)
  PT_END(pt);
}
