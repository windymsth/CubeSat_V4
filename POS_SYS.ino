#include <Adafruit_GPS.h>
#include <Kalman.h> // Source: https://github.com/TKJElectronics/KalmanFilter
#include <SimpleKalmanFilter.h>
#include "ublox_NEMA_lib.h"

Adafruit_GPS GPS(&Serial1);

#define GPSECHO false

commIMU imu;

//#define debugBMP180
//#define debugGPS

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

static uint32_t timer;

static struct pt pt_pos_driver;
static struct pt pt_acq_9dof_task, pt_acq_baro_task, pt_acq_gps_task;
static struct pt pt_clc_pos_task, pt_clc_alt_task, pt_clc_nema_task;

//  Called by "CubeSat_V4"->"setup()"->first call be uesd to init this system.
void Postion_System_Init() {
  Serial.println("\r\nPostion System Init:\r\n");
  
  Wire.begin();
  Serial.println("Setp:0");
  imuInit();
  delay(100);
  imu.getData();
  Serial.println("Setp:1");
  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  double roll  = atan2(imu.AY, imu.AZ) * RAD_TO_DEG;
  double pitch = atan(-imu.AX / sqrt(imu.AY * imu.AY + imu.AZ * imu.AZ)) * RAD_TO_DEG;

  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);

  timer = micros();
  Serial.println("Setp:2");
  pressureInit();
  Serial.println("Setp:3");
  GPS_init();
  
  Serial.println("\r\nPostion System Init Finished...\r\n");
  
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

//#define magbias_x 470
//#define magbias_y 120
//#define magbias_z 125
#define magbias_x 0
#define magbias_y 0
#define magbias_z 0
static bool Acq_Pos_Ok_Flag = false;
static int thread_acq_9dof_task(struct pt *pt) {
  PT_BEGIN(pt);
  while(1){
    imu.getData();

    sys.ax = (imu.AX -525) /16384; sys.ay = (imu.AY +105) /16384; sys.az = (imu.AZ +336) /16384;
    sys.gx = imu.GX *250/32768; sys.gy = imu.GY *250/32768; sys.gz = imu.GZ *250/32768;
    sys.mx = imu.MX; sys.my = imu.MY; sys.mz = imu.MZ;
// 1 Gas = 100 uT   1 mGas = 0.1 uT   10 mGas = 1uT
// AK8963 is 49.12 Ga.
    //  mRes : uT convert to mGas
    static float mRes = 10.*4912./32760.0;
//    sys.mx = sys.mx*mRes*1.13 - magbias_x;
//    sys.my = sys.my*mRes*1.13 - magbias_y;
//    sys.mz = sys.mz*mRes*1.11 - magbias_z;
    sys.mx = sys.mx*mRes - magbias_x;
    sys.my = sys.my*mRes - magbias_y;
    sys.mz = sys.mz*mRes - magbias_z;

//  Serial.print(imu.AX); Serial.print("\t");Serial.print(imu.AY); Serial.print("\t");Serial.print(imu.AZ); Serial.print("\t");
//  Serial.print(imu.GX); Serial.print("\t");Serial.print(imu.GY); Serial.print("\t");Serial.print(imu.GZ); Serial.print("\t");
//  Serial.print(imu.MX); Serial.print("\t");Serial.print(imu.MY); Serial.print("\t");Serial.print(imu.MZ); Serial.print("\t");
//  Serial.println();
#if 0
    Serial.print(sys.ax); Serial.print("\t");
    Serial.print(sys.ay); Serial.print("\t");
    Serial.print(sys.az); Serial.print("\t");
    Serial.print(sys.gx); Serial.print("\t");
    Serial.print(sys.gy); Serial.print("\t");
    Serial.print(sys.gz); Serial.print("\t");
    Serial.print(sys.mx); Serial.print('\t');
    Serial.print(sys.my); Serial.print('\t');
    Serial.print(sys.mz); Serial.print('\t');
    Serial.println();
#endif    
    Acq_Pos_Ok_Flag = true; // ***need modify to sem
    PT_TIMER_DELAY(pt, 50); // 20 Hz
  }
  PT_END(pt);
}

// Correct angle
float correctAngle(float heading) {
  if (heading < 0) { heading += 2 * PI; }
  if (heading > 2 * PI) { heading -= 2 * PI; }
  return heading;
}

#define FILTER_N 20
float filter_buf[FILTER_N + 1];
float shiftwinFilter(float val) {
  int i;
  float filter_sum = 0;
  filter_buf[FILTER_N] = val;
  for(i = 0; i < FILTER_N; i++) {
  filter_buf[i] = filter_buf[i + 1]; // 所有数据左移，低位仍掉
  filter_sum += filter_buf[i];
  }
  return (float)(filter_sum / FILTER_N);
}

#define CONSTANTS_ONE_G 9.8015f    /* m/s^2    */
static double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter
static int thread_clc_pos_task(struct pt *pt) {
  PT_BEGIN(pt);
  while(1){
    PT_WAIT_UNTIL(pt, Acq_Pos_Ok_Flag == true);
    Acq_Pos_Ok_Flag = false;

    double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
    timer = micros();
    
    double roll  = atan2(imu.AY, imu.AZ) * RAD_TO_DEG;
    double pitch = atan(-imu.AX / sqrt(imu.AY * imu.AY + imu.AZ * imu.AZ)) * RAD_TO_DEG;

    double gyroXrate = imu.GX / 131.0; // Convert to deg/s
    double gyroYrate = imu.GY / 131.0; // Convert to deg/s
  
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    kalAngleX = roll;
  } else
  sys.roll = kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  sys.pitch = kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);

//  double XH = sys.mx*cos(kalAngleY) + sys.my*sin(kalAngleX)*sin(kalAngleY) - sys.mz*cos(kalAngleX)*sin(kalAngleY);
//  double YH = sys.my*cos(kalAngleX) + sys.mz*sin(kalAngleX);
//  double yaw = atan( YH / XH );
  double yaw = atan2( sys.my, sys.mx );
//  double yaw = atan2( imu.MY, imu.MX );
  float declinationAngle = (4.0 + (26.0 / 60.0)) / (180 / M_PI);
  yaw += declinationAngle;
  yaw = correctAngle(yaw);
  sys.mag_heading = sys.yaw = yaw = yaw * 180/M_PI;

  sys.vertical_accel = sqrt( sys.ax * sys.ax + sys.ay * sys.ay + sys.az * sys.az);
  sys.vertical_accel = shiftwinFilter(sys.vertical_accel) * CONSTANTS_ONE_G;
//  Serial.print(sys.vertical_accel); Serial.print("\t");
//  Serial.print(yaw); Serial.print("\t");
//  Serial.println();
  /* Print Data */
#if 0 // Set to 1 to activate
  Serial.print(imu.AX); Serial.print("\t");Serial.print(imu.AY); Serial.print("\t");Serial.print(imu.AZ); Serial.print("\t");
  Serial.print(imu.GX); Serial.print("\t");Serial.print(imu.GY); Serial.print("\t");Serial.print(imu.GZ); Serial.print("\t");
  Serial.print("\r\n");
#endif

#if 0
  Serial.print(roll); Serial.print("\t");Serial.print(kalAngleX); Serial.print("\t");
  Serial.print(pitch); Serial.print("\t");Serial.print(kalAngleY); Serial.print("\t");
  Serial.print(yaw); Serial.print("\t");
  Serial.print("\r\n");
#endif
  
  }
  PT_END(pt);
}

static bool Acq_Baro_Ok_Flag = false;
static float temperature;
static int thread_acq_baro_task(struct pt *pt) {
  PT_BEGIN(pt);
  while(1){
    orderTV();
    PT_TIMER_DELAY(pt, 5);
    temperature = tempVal();
    
    orderPV();
    PT_TIMER_DELAY(pt, 50);
    sys.baro_pressure = pressureVal();
    
    Acq_Baro_Ok_Flag = true;
    PT_TIMER_DELAY(pt, 500);
  }
  PT_END(pt);
}

//SimpleKalmanFilter pressureKalmanFilter(1, 1, 0.01);
static int thread_clc_alt_task(struct pt *pt) {
  PT_BEGIN(pt);
  while(1){
    PT_WAIT_UNTIL(pt, Acq_Baro_Ok_Flag == true);
    Acq_Baro_Ok_Flag = false;
//    sys.baro_altitude = altitudeVal(sys.baro_pressure, 101325);
//    sys.baro_pressure = pressureKalmanFilter.updateEstimate( sys.baro_pressure );
    sys.baro_altitude = altitudeVal( sys.baro_pressure, 101325 );
#if 0
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
      sys.gps_gndspeed = GPS.speed * 1.852;  // 1 knot = 1.852 Km/h
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

