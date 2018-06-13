DATA_TYPE sys;
CMD_TYPE sys_cmd;

const int setBT_Line  = 30;
const int setFM_Line  = 42;

#define Debug20Hz
#define Debug10Hz
#define Debug5Hz
#define Debug2Hz
#define Debug1Hz
#define Debug0_5Hz
#define Debug0_25Hz

enum strBuf{
  inside_temp,inside_humi,
  pitch,roll,yaw,mag_heading,vertical_accel,
  gps_HDOP,gps_latitude,gps_longitude,gps_gndspeed,
  baro_pressure,baro_base_pressure,solar_panel_voltage,
  fm_transmit_frq
};

char strBuf[15][20] = {0};
char prtBuf[100];
unsigned char rfcrc8 = 0xFE;

static struct pt pt_rf_driver;
static struct pt pt_transmit_20Hz_task, pt_transmit_10Hz_task;
static struct pt pt_transmit_5Hz_task, pt_transmit_2Hz_task;
static struct pt pt_transmit_1Hz_task, pt_transmit_0_5Hz_task;
static struct pt pt_transmit_0_25Hz_task, pt_transmit_0_2Hz_task;

//  Called by "CubeSat_V4"->"setup()"->first call be uesd to init this system.
void RF_System_Init() {
//  pinMod(setBT_Line, OUTPUT);
//  pinMod(setFM_Line, OUTPUT);
//  Serial2.begin(115200);
  PT_INIT(&pt_rf_driver);
}

void TASK_RF_Handle(void) {
  thread_rf_driver(&pt_rf_driver);
  
  thread_transmit_20Hz_task(&pt_transmit_20Hz_task);
  thread_transmit_10Hz_task(&pt_transmit_10Hz_task);
  thread_transmit_5Hz_task(&pt_transmit_5Hz_task);
  thread_transmit_2Hz_task(&pt_transmit_2Hz_task);
  thread_transmit_1Hz_task(&pt_transmit_1Hz_task);
  thread_transmit_0_5Hz_task(&pt_transmit_0_5Hz_task);
  thread_transmit_0_25Hz_task(&pt_transmit_0_25Hz_task);
  thread_transmit_0_2Hz_task(&pt_transmit_0_2Hz_task);
}

static int thread_rf_driver(struct pt *pt) {
  PT_BEGIN(pt);
  PT_INIT(&pt_transmit_20Hz_task);
  PT_INIT(&pt_transmit_10Hz_task);
  PT_INIT(&pt_transmit_5Hz_task);
  PT_INIT(&pt_transmit_2Hz_task);
  PT_INIT(&pt_transmit_1Hz_task);
  PT_INIT(&pt_transmit_0_5Hz_task);
  PT_INIT(&pt_transmit_0_25Hz_task);
  PT_INIT(&pt_transmit_0_2Hz_task);

  PT_WAIT_THREAD(pt, 
         thread_transmit_20Hz_task(&pt_transmit_20Hz_task) &
         thread_transmit_10Hz_task(&pt_transmit_10Hz_task) &
         thread_transmit_5Hz_task(&pt_transmit_5Hz_task) &
         thread_transmit_2Hz_task(&pt_transmit_2Hz_task) &
         thread_transmit_1Hz_task(&pt_transmit_1Hz_task) &
         thread_transmit_0_5Hz_task(&pt_transmit_0_5Hz_task) &
         thread_transmit_0_25Hz_task(&pt_transmit_0_25Hz_task) &
         thread_transmit_0_2Hz_task(&pt_transmit_0_2Hz_task));
         
  PT_END(pt);
}

static bool recevie_ok_flag = false;
static char readbuf[100] = {0};
void serialEvent() {
  static int cnt = 0;
  static bool start_flag = false;
      if ( Serial.available() > 0) {
//        Serial.println();
          if ( start_flag == false ) {
            if ( Serial.peek() == '{' ) {
              cnt = 0;
              start_flag = true;
            } else {
              cnt = 0;
              start_flag = false;
              recevie_ok_flag = false;
            }
          }
          
          if ( start_flag == true ) {
            while ( Serial.available() > 0 ){
              readbuf[cnt] = Serial.read();
              if( readbuf[cnt] == '}' ) {
                recevie_ok_flag = true;
              } else {
                cnt++;
              } 
            }
          }

          if ( recevie_ok_flag == true ) {
              if ( strchr(readbuf, '}') != NULL ) {
                  Serial.println(readbuf);
                  RF_Recevie_Parsing();
                  memset(readbuf, 0x00, sizeof(readbuf));
                  recevie_ok_flag = false;
                  start_flag = false;
              } else {
                  Serial.println("Receive Failure!\r\n");
                  memset(readbuf, 0x00, sizeof(readbuf));
                  recevie_ok_flag = false;
                  start_flag = false;
              }
          }
      }
}

volatile bool RF_Recevie_Parsing() {
  String str1 = String(readbuf);
  String Key1;
  volatile int first_symbol = 0;
  volatile int second_symbol = 0;
  volatile int next_symbol = 0;
  first_symbol = str1.indexOf('"');
  first_symbol = first_symbol + 1;
  second_symbol = str1.indexOf('"', first_symbol);
  next_symbol = second_symbol + 1;
  Key1 = str1.substring(first_symbol, second_symbol);
//  Serial.println(Key1);
  if( Key1 == "HB") {
    sys_cmd.system_flag = true;
    Serial.println("HB");
    String Value = str1.substring( str1.indexOf(':', next_symbol) +1, str1.indexOf(',', next_symbol));
    sys_cmd.heart_beat = Value.toInt();
  }
  if( Key1 == "FUNR") {
    sys_cmd.system_flag = true;
    Serial.println("FUNR");
    String Value = str1.substring( str1.indexOf(':', next_symbol) +1, str1.indexOf(',', next_symbol));
    sys_cmd.function_reset = Value.toInt();
  }
  if( Key1 == "SYSR") {
    sys_cmd.system_flag = true;
    Serial.println("SYSR");
    String Value = str1.substring( str1.indexOf(':', next_symbol) +1, str1.indexOf(',', next_symbol));
    sys_cmd.system_reboot = Value.toInt();
  }
  if( Key1 == "LSLR") {
    sys_cmd.ctrl_flag = true;
    Serial.println("LSLR");
    String Value = str1.substring( str1.indexOf(':', next_symbol) +1, str1.indexOf(',', next_symbol));
    sys_cmd.ctrl_solar_angle = Value.toInt();
  }
  if( Key1 == "SANT") {
    sys_cmd.ctrl_flag = true;
    Serial.println("SANT");
    String Value = str1.substring( str1.indexOf(':', next_symbol) +1, str1.indexOf(',', next_symbol));
    sys_cmd.ctrl_ant = Value.toInt();
  }
  if( Key1 == "FMPW") {
    sys_cmd.set_fm_flag = true;
    Serial.println("FMPW");
    String Value = str1.substring( str1.indexOf(':', next_symbol) +1, str1.indexOf(',', next_symbol));
    sys_cmd.set_fm_pwr = Value.toInt();
  }
  if( Key1 == "FMFQ") {
    sys_cmd.set_fm_flag = true;
    Serial.println("FMFQ");
    String Value = str1.substring( str1.indexOf(':', next_symbol) +1, str1.indexOf(',', next_symbol));
    char fmFrqBuf[10];
    Value.toCharArray(fmFrqBuf, 10);
    sys_cmd.set_fm_frq = atof(fmFrqBuf);
    Serial.println(sys_cmd.set_fm_frq);
    
    int s = str1.indexOf("\"FMVL\":") + sizeof("\"FMVL\":") - 1;
    Value = str1.substring( s, str1.indexOf(',', s));
    sys_cmd.set_fm_vol = Value.toInt();
    Serial.println(sys_cmd.set_fm_vol);
  }
  if( Key1 == "UBUD") {
    sys_cmd.set_uhf_flag = true;
    Serial.println("UBUD");
    String Value = str1.substring( str1.indexOf(':', next_symbol) +1, str1.indexOf(',', next_symbol));
    sys_cmd.set_uhf_bud = Value.toInt();
    Serial.println(sys_cmd.set_uhf_bud);
    
    int s = str1.indexOf("\"UCHL\":") + sizeof("\"UCHL\":") - 1;
    Value = str1.substring( s, str1.indexOf(',', s));
    sys_cmd.set_uhf_ch = Value.toInt();
    Serial.println(sys_cmd.set_uhf_ch);
  }
  if( Key1 == "MN") {
    sys_cmd.set_mled_flag = true;
    Serial.println("MN");
    String Value = str1.substring( str1.indexOf(':', next_symbol) +1, str1.indexOf(',', next_symbol));
    sys_cmd.set_mled_num = Value.toInt();
//    Serial.println(sys_cmd.set_mled_num);
    
    int s = str1.indexOf("\"CF\":") + sizeof("\"CF\":");
    Value = str1.substring( s, str1.indexOf(',', s)-1 );
    char rgbBuf[5];
    Value.toCharArray(rgbBuf, 5);
    sscanf(rgbBuf, "%4x", &sys_cmd.set_mled_rgb);
//    Serial.println(sys_cmd.set_mled_rgb, HEX);

    s = str1.indexOf("\"MD\":") + sizeof("\"MD\":");
    Value = str1.substring( s, str1.indexOf(',', s)-1 );
    for(char i=0; i<8; i++) { 
      char mDataBuf[4];    
      Value.substring(i*2, i*2+2).toCharArray(mDataBuf, 3);
      sscanf(mDataBuf, "%x", &sys_cmd.set_mled[i]);
//      Serial.println(sys_cmd.set_mled[i], HEX);
    }
  }
  if( Key1 == "MLMD") {
    sys_cmd.set_mled_flag = true;
    Serial.println("MLMD");
    String Value = str1.substring( str1.indexOf(':', next_symbol) +1, str1.indexOf(',', next_symbol));
    sys_cmd.set_mled_mode = Value.toInt();
    HMI_DISPLAY_MODE = (bool)Value.toInt();
//    Serial.println(HMI_DISPLAY_MODE, DEC);
  }
  if( Key1 == "RSLR") {
    sys_cmd.ctrl_flag = true;
    Serial.println("RSLR");
    String Value = str1.substring( str1.indexOf(':', next_symbol) +1, str1.indexOf(',', next_symbol));
    sys_cmd.ctrl_solar_reset = Value.toInt();
  }
  return false;
}

/*
//static int thread_receive_task(struct pt *pt) {
//  static char readbuf[100] = {0};
//  static int cnt = 0;
//  PT_BEGIN(pt);
////  Serial.println("into rec step:0\r\n");
//  while(1) {
//      PT_WAIT_UNTIL(pt, (Serial.available() > 0) );
//      PT_TIMER_DELAY(pt, 500);
//          if( Serial.peek() != '{' ) {
//            while(Serial.read() >= 0);
//          } else {
//              for(int i=0; Serial.available()>0 && i<100; i++) {
//                readbuf[i] = Serial.read();
//                cnt = i;
//                if( readbuf[i] == '\r' ) break;
//              }
//              
//              if( cnt > 0 && strchr(readbuf, '}') != NULL ) {
//                RF_Recevie_Parsing(readbuf);
////                Serial.println(readbuf);
//                memset(readbuf, 0x00, sizeof(readbuf));
//              } else {
//                Serial.println("Receive Failure!\r\n");
//                memset(readbuf, 0x00, sizeof(readbuf));
//              }
//          }
//  } // end by "while(1)"
//  PT_END(pt);
//}
*/

static int thread_transmit_20Hz_task(struct pt *pt) {
  PT_BEGIN(pt);
  while(1) {
    #ifdef  Debug20Hz
    sprintf(prtBuf, "{\"N\":14,\"AX\":%d,\"AY\":%d,\"AZ\":%d,\"CK\":\"%02X\"}", sys.ax, sys.ay, sys.az, rfcrc8);Serial.println(prtBuf);
    sprintf(prtBuf, "{\"N\":15,\"GX\":%d,\"GY\":%d,\"GZ\":%d,\"CK\":\"%02X\"}", sys.gx, sys.gy, sys.gz, rfcrc8);Serial.println(prtBuf);
    sprintf(prtBuf, "{\"N\":16,\"MX\":%d,\"MY\":%d,\"MZ\":%d,\"CK\":\"%02X\"}", sys.mx, sys.my, sys.mz, rfcrc8);Serial.println(prtBuf);
    #endif
    PT_TIMER_DELAY(pt, 50);
  }
  PT_END(pt);
}

static int thread_transmit_10Hz_task(struct pt *pt) {
  PT_BEGIN(pt);
  while(1) {
    #ifdef  Debug10Hz
    sprintf(prtBuf, "{\"N\":3,\"PIT\":%s,\"CK\":\"%02X\"}", strBuf[pitch], rfcrc8);Serial.println(prtBuf);
    sprintf(prtBuf, "{\"N\":4,\"ROL\":%s,\"CK\":\"%02X\"}", strBuf[roll], rfcrc8);Serial.println(prtBuf);
    sprintf(prtBuf, "{\"N\":5,\"YAW\":%s,\"CK\":\"%02X\"}", strBuf[yaw], rfcrc8);Serial.println(prtBuf);
    sprintf(prtBuf, "{\"N\":6,\"HEAD\":%s,\"CK\":\"%02X\"}", strBuf[mag_heading], rfcrc8);Serial.println(prtBuf);
    #endif
    PT_TIMER_DELAY(pt, 100);
  }
  PT_END(pt);
}

static int thread_transmit_5Hz_task(struct pt *pt) {
  PT_BEGIN(pt);
  while(1) {
    #ifdef  Debug5Hz
    sprintf(prtBuf, "{\"N\":9,\"PRS\":%s,\"CK\":\"%02X\"}", strBuf[baro_pressure], rfcrc8);Serial.println(prtBuf);
    sprintf(prtBuf, "{\"N\":11,\"VG\":%s,\"CK\":\"%02X\"}", strBuf[vertical_accel], rfcrc8);Serial.println(prtBuf);
    #endif
    PT_TIMER_DELAY(pt, 200);
  }
  PT_END(pt);
}

static int thread_transmit_2Hz_task(struct pt *pt) {
  PT_BEGIN(pt);
  while(1) {
    #ifdef  Debug2Hz
//    //sprintf(prtBuf, "{\"OUT_TP\":{\"v\":%s}", strBuf[0]);Serial.println(prtBuf);
    sprintf(prtBuf, "{\"N\":1,\"INTP\":%s,\"CK\":\"%02X\"}", strBuf[inside_temp], rfcrc8);Serial.println(prtBuf);
    sprintf(prtBuf, "{\"N\":2,\"INHM\":%s,\"CK\":\"%02X\"}", strBuf[inside_humi], rfcrc8);Serial.println(prtBuf);
    sprintf(prtBuf, "{\"N\":18,\"BALT\":%d,\"CK\":\"%02X\"}", sys.baro_altitude, rfcrc8);Serial.println(prtBuf);
    #endif
    PT_TIMER_DELAY(pt, 500);
  }
  PT_END(pt);
}

static int thread_transmit_1Hz_task(struct pt *pt) {
  PT_BEGIN(pt);
  while(1) {
    #ifdef  Debug1Hz
    sprintf(prtBuf, "{\"N\":12,\"UVE\":%d,\"CK\":\"%02X\"}", sys.uv_index, rfcrc8);Serial.println(prtBuf);
    sprintf(prtBuf, "{\"N\":13,\"LUXE\":%d,\"CK\":\"%02X\"}", sys.lux, rfcrc8);Serial.println(prtBuf);
//    sprintf(prtBuf, "{\"N\":24,\"LSRD\":%d,\"CK\":\"%02X\"}", sys.left_solar_rod, rfcrc8);Serial.println(prtBuf);
//    sprintf(prtBuf, "{\"N\":25,\"RSLD\":%d,\"CK\":\"%02X\"}", sys.right_solar_rod, rfcrc8);Serial.println(prtBuf);
    sprintf(prtBuf, "{\"N\":27,\"VSLR\":%s,\"CK\":\"%02X\"}", strBuf[solar_panel_voltage], rfcrc8);Serial.println(prtBuf);
    #endif
    PT_TIMER_DELAY(pt, 1000);
  }
  PT_END(pt);
}

static int thread_transmit_0_5Hz_task(struct pt *pt) {
  PT_BEGIN(pt);
  while(1) {
    #ifdef  Debug0_5Hz
    sprintf(prtBuf, "{\"N\":7,\"LAT\":%s,\"CK\":\"%02X\"}", strBuf[gps_latitude], rfcrc8);Serial.println(prtBuf);
    sprintf(prtBuf, "{\"N\":8,\"LON\":%s,\"CK\":\"%02X\"}", strBuf[gps_longitude], rfcrc8);Serial.println(prtBuf);
    sprintf(prtBuf, "{\"N\":22,\"SANT\":%d,\"CK\":\"%02X\"}", sys.antena_status, rfcrc8);Serial.println(prtBuf);
    sprintf(prtBuf, "{\"N\":23,\"SSLR\":%d,\"CK\":\"%02X\"}", sys.solar_panel_status, rfcrc8);Serial.println(prtBuf);
    #endif
    PT_TIMER_DELAY(pt, 2000);
  }
  PT_END(pt);
}

static int thread_transmit_0_25Hz_task(struct pt *pt) {
  PT_BEGIN(pt);
  while(1) {
    #ifdef  Debug0_25Hz
    sprintf(prtBuf, "{\"N\":19,\"NSAT\":%d,\"CK\":\"%02X\"}", sys.gps_satellites, rfcrc8);Serial.println(prtBuf);
    sprintf(prtBuf, "{\"N\":10,\"GSPD\":%s,\"CK\":\"%02X\"}", strBuf[gps_gndspeed], rfcrc8);Serial.println(prtBuf);
    sprintf(prtBuf, "{\"N\":21,\"GALT\":%d,\"CK\":\"%02X\"}", sys.gps_altitude, rfcrc8);Serial.println(prtBuf);
//    sprintf(prtBuf, "{\"N\":26,\"FMTF\":%s,\"CK\":\"%02X\"}", strBuf[fm_transmit_frq], rfcrc8);Serial.println(prtBuf);
    #endif
    PT_TIMER_DELAY(pt, 4000);
  }
  PT_END(pt);
}

static int thread_transmit_0_2Hz_task(struct pt *pt) {
  PT_BEGIN(pt);
  while(1) {
    #ifdef  Debug0_5Hz
//    sprintf(prtBuf, "{\"N\":17,\"BSP\":%s,\"CK\":\"%02X\"}", strBuf[baro_base_pressure], rfcrc8);Serial.println(prtBuf);
    sprintf(prtBuf, "{\"N\":20,\"HDOP\":%s,\"CK\":\"%02X\"}", strBuf[gps_HDOP], rfcrc8);Serial.println(prtBuf);
    #endif
    PT_TIMER_DELAY(pt, 5000);
  }
  PT_END(pt);
}

void Data_Str_stack() {
  dtostrf(sys.inside_temp, 1, 1, strBuf[inside_temp]);
  dtostrf(sys.inside_humi, 1, 1, strBuf[inside_humi]);
  dtostrf(sys.pitch, 1, 1, strBuf[pitch]);
  dtostrf(sys.roll, 1, 1, strBuf[roll]);
  dtostrf(sys.yaw, 1, 1, strBuf[yaw]);
  dtostrf(sys.mag_heading, 1, 1, strBuf[mag_heading]);
  dtostrf(sys.vertical_accel, 1, 2, strBuf[vertical_accel]);
  dtostrf(sys.gps_HDOP, 1, 1, strBuf[gps_HDOP]);
  dtostrf(sys.gps_latitude, 1, 4, strBuf[gps_latitude]);
  dtostrf(sys.gps_longitude,1, 4, strBuf[gps_longitude]);
  dtostrf(sys.gps_gndspeed,1, 4, strBuf[gps_gndspeed]);
  dtostrf(sys.baro_pressure, 1, 3, strBuf[baro_pressure]);
  dtostrf(sys.baro_base_pressure, 1, 3, strBuf[baro_base_pressure]);
  dtostrf(sys.solar_panel_voltage, 1, 1, strBuf[solar_panel_voltage]);
  dtostrf(sys.fm_transmit_frq, 1, 3, strBuf[fm_transmit_frq]);
  
}


