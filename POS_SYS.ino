#include <Adafruit_GPS.h>
#include "ublox_NEMA_lib.h"

Adafruit_GPS GPS(&Serial1);

#define GPSECHO false

commIMU imu;

//#define debugBMP180
//#define debugGPS

static struct pt pt_pos_driver;
static struct pt pt_acq_9dof_task, pt_acq_baro_task, pt_acq_gps_task;
static struct pt pt_clc_pos_task, pt_clc_alt_task, pt_clc_nema_task;

//  Called by "CubeSat_V4"->"setup()"->first call be uesd to init this system.
void Postion_System_Init() {
  Wire.begin();
  imuInit();
  pressureInit();
  GPS_init();
  PT_INIT(&pt_pos_driver);
}

void TASK_Postion_Handle() {
  
  thread_pos_driver(&pt_pos_driver);
  
  thread_acq_9dof_task(&pt_acq_9dof_task);
  thread_acq_baro_task(&pt_acq_baro_task);
  thread_acq_gps_task(&pt_acq_gps_task);
  thread_clc_pos_task(&pt_clc_pos_task);
  thread_clc_alt_task(&pt_clc_alt_task);
  thread_clc_nema_task(&pt_clc_nema_task);
}

static int thread_pos_driver(struct pt *pt) {
  PT_BEGIN(pt);
  PT_INIT(&pt_acq_9dof_task);
  PT_INIT(&pt_acq_baro_task);
  PT_INIT(&pt_acq_gps_task);
  PT_INIT(&pt_clc_pos_task);
  PT_INIT(&pt_clc_alt_task);
  PT_INIT(&pt_clc_nema_task);

  PT_WAIT_THREAD(pt, 
         thread_acq_9dof_task(&pt_acq_9dof_task) &
         thread_acq_baro_task(&pt_acq_baro_task) &
         thread_acq_gps_task(&pt_acq_gps_task) &
         thread_clc_pos_task(&pt_clc_pos_task) &
         thread_clc_alt_task(&pt_clc_alt_task) &
         thread_clc_nema_task(&pt_clc_nema_task));
  PT_END(pt);
}

static bool Acq_Pos_Ok_Flag = false;
static int thread_acq_9dof_task(struct pt *pt) {
  PT_BEGIN(pt);
  while(1){
    imu.getData();
    sys.ax = imu.AX; sys.ay = imu.AY; sys.az = imu.AZ;
    sys.gx = imu.GX; sys.gy = imu.GY; sys.gz = imu.GZ;
    sys.mx = imu.MX; sys.my = imu.MY; sys.mz = imu.MZ;
//    Serial.println(imu.AX);
    Acq_Pos_Ok_Flag = true;
    PT_TIMER_DELAY(pt, 50);
  }
  PT_END(pt);
}

static int thread_clc_pos_task(struct pt *pt) {
  PT_BEGIN(pt);
  while(1){
    PT_WAIT_UNTIL(pt, Acq_Pos_Ok_Flag == true);
    Acq_Pos_Ok_Flag = false;
  }
  PT_END(pt);
}

static bool Acq_Baro_Ok_Flag = false;
static double temperature;
static int thread_acq_baro_task(struct pt *pt) {
  PT_BEGIN(pt);
  while(1){
//    temperature = tempVal();
//    sys.baro_pressure = pressureVal();
    Acq_Baro_Ok_Flag = true;
    PT_TIMER_DELAY(pt, 500);
  }
  PT_END(pt);
}

static int thread_clc_alt_task(struct pt *pt) {
  PT_BEGIN(pt);
  while(1){
    PT_WAIT_UNTIL(pt, Acq_Baro_Ok_Flag == true);
    Acq_Baro_Ok_Flag = false;
//    sys.baro_altitude = altitudeVal();
#ifdef debugBMP180
    Serial.print(sys.baro_altitude);Serial.print("\t"); 
    Serial.print(sys.baro_pressure);Serial.print("\t");
    Serial.print(temperature);Serial.println("\t");
#endif
  }
  PT_END(pt);
}


static bool Acq_GPS_Ok_Flag = false;
static int thread_acq_gps_task(struct pt *pt) {
  PT_BEGIN(pt);
  while(1){

    Acq_GPS_Ok_Flag = true;
    PT_TIMER_DELAY(pt, 2000);
  }
  PT_END(pt);
}

static int thread_clc_nema_task(struct pt *pt) {
  PT_BEGIN(pt);
  while(1){
    PT_WAIT_UNTIL(pt, Acq_GPS_Ok_Flag == true);
    Acq_GPS_Ok_Flag = false;
    sys.gps_satellites = (unsigned char)GPS.satellites;
    if( GPS.fix == true && GPS.fixquality == true ) {
      sys.gps_gndspeed = GPS.speed;
      sys.gps_latitude = GPS.latitude;
      sys.gps_longitude = GPS.longitude;
      sys.gps_altitude = GPS.altitude;
      sys.gps_HDOP = GPS.HDOP;
    }
  }
  PT_END(pt);
}

void GPS_init() {
  Serial1.begin(9600);
  Serial.println("GPS Serial Start...\r\n");
  Serial1.write(disgsa, 14); //DIS GSA
  Serial1.write(disgsv, 14); //DIS GSV
  Serial1.write(disgll, 14); //DIS GLL
  Serial1.write(disvtg, 14); //DIS VTG
  Serial1.write(enrmc, 14);  // EN RMC
  Serial1.write(engga, 14);  // EN GGA
  Serial1.write(setbud960, 28);  // SET buad: 9600
  delay(500);
  Serial1.begin(9600);
  // Ask for firmware version
  Serial1.println(PMTK_Q_RELEASE);
}

void serialEvent1() {
  char c = GPS.read();
  if (GPSECHO)
    if (c) Serial.print(c);
  if (GPS.newNMEAreceived()) {
    #ifdef debugGPS
    Serial.println(GPS.lastNMEA());
    #endif
    if (!GPS.parse(GPS.lastNMEA()))
      return; // we can fail to parse a sentence in which case we should just wait for another
  }
}

const double EARTH_RADIUS = 6378.137;
static double rad(double d) {
   return d * PI / 180.0;
}

static double Calculate_GPS_Distance(double lat1, double lng1, double lat2, double lng2) {
   double radLat1 = rad(lat1);
   double radLat2 = rad(lat2);
   double a = radLat1 - radLat2;
   double b = rad(lng1) - rad(lng2);
   double s = 2 * asin(sqrt(pow(sin(a/2),2) +
    cos(radLat1)*cos(radLat2)*pow(sin(b/2),2)));
   s = s * EARTH_RADIUS;
   s = round(s * 10000) / 10000;
   return s;
}

