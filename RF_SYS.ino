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

static struct pt pt_rf_driver;
static struct pt pt_receive_ctrl_task;
static struct pt pt_transmit_20Hz_task, pt_transmit_10Hz_task;
static struct pt pt_transmit_5Hz_task, pt_transmit_2Hz_task;
static struct pt pt_transmit_1Hz_task, pt_transmit_0_5Hz_task;
static struct pt pt_transmit_0_25Hz_task, pt_transmit_0_2Hz_task;

//  Called by "CubeSat_V4"->"setup()"->first call be uesd to init this system.
void RF_System_Init() {
  Serial.println("\r\nRF System Init:\r\n");
  
//  pinMod(setBT_Line, OUTPUT);
//  pinMod(setFM_Line, OUTPUT);
  Serial3.begin(115200);

  Serial.println("\r\nRF System Init Finished...\r\n");
  
  PT_INIT(&pt_rf_driver);
}

void TASK_RF_Handle(void) {
  thread_rf_driver(&pt_rf_driver);

  thread_receive_ctrl_task(&pt_receive_ctrl_task);
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
  PT_INIT(&pt_receive_ctrl_task);
  PT_INIT(&pt_transmit_20Hz_task);
  PT_INIT(&pt_transmit_10Hz_task);
  PT_INIT(&pt_transmit_5Hz_task);
  PT_INIT(&pt_transmit_2Hz_task);
  PT_INIT(&pt_transmit_1Hz_task);
  PT_INIT(&pt_transmit_0_5Hz_task);
  PT_INIT(&pt_transmit_0_25Hz_task);
  PT_INIT(&pt_transmit_0_2Hz_task);

  PT_WAIT_THREAD(pt, 
         thread_receive_ctrl_task(&pt_receive_ctrl_task) &
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
void serialEvent3() {
  static int cnt = 0;
  static bool start_flag = false;
      if ( Serial3.available() > 0) {
//        Serial.println();
          if ( start_flag == false ) {
            if ( Serial3.peek() == '{' ) {
              cnt = 0;
              start_flag = true;
            } else {
              cnt = 0;
              start_flag = false;
              recevie_ok_flag = false;
            }
          }
          
          if ( start_flag == true ) {
            while ( Serial3.available() > 0 ){
              readbuf[cnt] = Serial3.read();
              if( readbuf[cnt] == '}' ) {
                recevie_ok_flag = true;
              } else {
                cnt++;
              } 
            }
          }

          if ( recevie_ok_flag == true ) {
              if ( strchr(readbuf, '}') != NULL ) {
//                  Serial.println(readbuf);

                  RF_Recevie_Parsing(); //  *** nead optimus
                  
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
  if( Key1 == "FACTORY") {
    sys_cmd.factory_flag = true;
    Serial.println(Key1);
    String Value = str1.substring( str1.indexOf(':', next_symbol) +1, str1.indexOf(',', next_symbol));
    sys_cmd.factory_mode = Value.toInt();
  }
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
    sys_cmd.ctrl_solar_flag = true;
    Serial.println("LSLR");
    String Value = str1.substring( str1.indexOf(':', next_symbol) +1, str1.indexOf(',', next_symbol));
    sys_cmd.ctrl_solar_angle = Value.toInt();
    Serial.println( sys_cmd.ctrl_solar_angle );
  }
  if( Key1 == "SANT") {
    sys_cmd.ctrl_ant_flag = true;
    Serial.println("SANT");
    String Value = str1.substring( str1.indexOf(':', next_symbol) +1, str1.indexOf(',', next_symbol));
    sys_cmd.ctrl_ant = Value.toInt();
    
    CTRL_DEPLOY_ANT = (bool)Value.toInt();
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
    sys_cmd.ctrl_solar_flag = true;
    Serial.println("RSLR");
    String Value = str1.substring( str1.indexOf(':', next_symbol) +1, str1.indexOf(',', next_symbol));
    sys_cmd.ctrl_solar_reset = Value.toInt();
  }
  if( Key1 == "CCAP") {
    sys_cmd.ctrl_cam_flag = true;
    Serial.println("CCAP");
    String Value = str1.substring( str1.indexOf(':', next_symbol) +1, str1.indexOf(',', next_symbol));
    sys_cmd.ctrl_cam_cap = Value.toInt();
  }
  if( Key1 == "STIM") {
    sys_cmd.set_rtc_flag = true;
    Serial.println("STIM");
    String Value = str1.substring( str1.indexOf(':', next_symbol) +1, str1.indexOf(',', next_symbol));
    sys_cmd.set_adjust_time_year = Value.substring(0, 4).toInt();
    sys_cmd.set_adjust_time_month = Value.substring(4, 6).toInt();
    sys_cmd.set_adjust_time_day = Value.substring(6, 8).toInt();
    sys_cmd.set_adjust_time_hour = Value.substring(8, 10).toInt();
    sys_cmd.set_adjust_time_minute = Value.substring(10, 12).toInt();
    Serial.print(sys_cmd.set_adjust_time_year);Serial.print('\t');
    Serial.print(sys_cmd.set_adjust_time_month);Serial.print('\t');
    Serial.print(sys_cmd.set_adjust_time_day);Serial.print('\t');
    Serial.print(sys_cmd.set_adjust_time_hour);Serial.print('\t');
    Serial.print(sys_cmd.set_adjust_time_minute);Serial.print('\n');
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

const int UHF_SET_PIN = 22;
static int thread_receive_ctrl_task(struct pt *pt) {
  PT_BEGIN(pt);
  while(1) {
    PT_WAIT_UNTIL(pt, sys_cmd.set_uhf_flag == true);
//    delay(50);
//    enAT( UHF_SET_PIN );
//    delay(100);
//    
//    testAt();
//    delay(100);
//    
//    setCh( constrain(sys_cmd.set_uhf_ch, 1, 99) );
//    delay(500);
//
//    outAT( UHF_SET_PIN );
//    delay(500);
  }
  PT_END(pt);
}

//char strBuf[15][20] = {0};
static char prtBuf[100];
static uint8_t rfcrc8 = 0xFE;
static int thread_transmit_20Hz_task(struct pt *pt) {
  PT_BEGIN(pt);
  while(1) {
    #ifdef  Debug20Hz
    char strBuf[9][20];
    dtostrf(sys.ax, 1, 2, strBuf[0]); dtostrf(sys.ay, 1, 2, strBuf[1]); dtostrf(sys.az, 1, 2, strBuf[2]);
    dtostrf(sys.gx, 1, 2, strBuf[3]); dtostrf(sys.gy, 1, 2, strBuf[4]); dtostrf(sys.gz, 1, 2, strBuf[5]);
    dtostrf(sys.mx, 1, 2, strBuf[6]); dtostrf(sys.my, 1, 2, strBuf[7]); dtostrf(sys.mz, 1, 2, strBuf[8]);
    sprintf(prtBuf, "{\"N\":14,\"AX\":%s,\"AY\":%s,\"AZ\":%s,\"CK\":\"%X\"}", strBuf[0], strBuf[1], strBuf[2], rfcrc8);Serial3.println(prtBuf);
    sprintf(prtBuf, "{\"N\":15,\"GX\":%s,\"GY\":%s,\"GZ\":%s,\"CK\":\"%X\"}", strBuf[3], strBuf[4], strBuf[5], rfcrc8);Serial3.println(prtBuf);
    sprintf(prtBuf, "{\"N\":16,\"MX\":%s,\"MY\":%s,\"MZ\":%s,\"CK\":\"%X\"}", strBuf[6], strBuf[7], strBuf[8], rfcrc8);Serial3.println(prtBuf);
    #endif
    PT_TIMER_DELAY(pt, 50);
  }
  PT_END(pt);
}

static int thread_transmit_10Hz_task(struct pt *pt) {
  PT_BEGIN(pt);
  while(1) {
    #ifdef  Debug10Hz
    sprintf(prtBuf, "{\"N\":3,\"PIT\":%d,\"CK\":\"%X\"}", int(sys.pitch), rfcrc8);Serial3.println(prtBuf);
    sprintf(prtBuf, "{\"N\":4,\"ROL\":%d,\"CK\":\"%X\"}", int(sys.roll), rfcrc8);Serial3.println(prtBuf);
    sprintf(prtBuf, "{\"N\":5,\"YAW\":%d,\"CK\":\"%X\"}", int(sys.yaw), rfcrc8);Serial3.println(prtBuf);
    sprintf(prtBuf, "{\"N\":6,\"HEAD\":%d,\"CK\":\"%X\"}", int(sys.mag_heading)+8, rfcrc8);Serial3.println(prtBuf);
    #endif
    PT_TIMER_DELAY(pt, 100);
  }
  PT_END(pt);
}

static int thread_transmit_5Hz_task(struct pt *pt) {
  PT_BEGIN(pt);
  while(1) {
    #ifdef  Debug5Hz
    char strBuf[20];
//    Serial.println( sys.vertical_accel );
    dtostrf(constrain(sys.vertical_accel, 0.0, 100.0), 1, 2, strBuf);
    sprintf(prtBuf, "{\"N\":9,\"PRS\":%ld,\"CK\":\"%X\"}", sys.baro_pressure, rfcrc8);Serial3.println(prtBuf);
    sprintf(prtBuf, "{\"N\":11,\"VG\":%s,\"CK\":\"%X\"}", strBuf, rfcrc8);Serial3.println(prtBuf);
    #endif
    PT_TIMER_DELAY(pt, 200);
  }
  PT_END(pt);
}

static int thread_transmit_2Hz_task(struct pt *pt) {
  PT_BEGIN(pt);
  while(1) {
    #ifdef  Debug2Hz
    //sprintf(prtBuf, "{\"OUT_TP\":{\"v\":%s}", strBuf[0]);Serial2.println(prtBuf);
    sprintf(prtBuf, "{\"N\":1,\"INTP\":%d,\"CK\":\"%X\"}", int(sys.inside_temp*10), rfcrc8);Serial3.println(prtBuf);
    sprintf(prtBuf, "{\"N\":2,\"INHM\":%d,\"CK\":\"%X\"}", int(sys.inside_humi*10), rfcrc8);Serial3.println(prtBuf);
    sprintf(prtBuf, "{\"N\":18,\"BALT\":%ld,\"CK\":\"%X\"}", sys.baro_altitude, rfcrc8);Serial3.println(prtBuf);
    #endif
    PT_TIMER_DELAY(pt, 500);
  }
  PT_END(pt);
}

static int thread_transmit_1Hz_task(struct pt *pt) {
  PT_BEGIN(pt);
  while(1) {
    #ifdef  Debug1Hz
    sprintf(prtBuf, "{\"N\":0,\"HB\":%d,\"CK\":\"%X\"}", sys.heart_beat++, rfcrc8);Serial3.println(prtBuf);
    sprintf(prtBuf, "{\"N\":12,\"UVE\":%d,\"CK\":\"%X\"}", sys.uv_index, rfcrc8);Serial3.println(prtBuf);
    sprintf(prtBuf, "{\"N\":13,\"LUXE\":%d,\"CK\":\"%X\"}", sys.lux, rfcrc8);Serial3.println(prtBuf);
//    sprintf(prtBuf, "{\"N\":24,\"LSRD\":%d,\"CK\":\"%X\"}", sys.left_solar_rod, rfcrc8);Serial3.println(prtBuf);
//    sprintf(prtBuf, "{\"N\":25,\"RSLD\":%d,\"CK\":\"%X\"}", sys.right_solar_rod, rfcrc8);Serial3.println(prtBuf);
    sprintf(prtBuf, "{\"N\":27,\"VSLR\":%d,\"CK\":\"%X\"}", int(sys.solar_panel_voltage*10), rfcrc8);Serial3.println(prtBuf);
    #endif
    PT_TIMER_DELAY(pt, 1000);
  }
  PT_END(pt);
}

static int thread_transmit_0_5Hz_task(struct pt *pt) {
  PT_BEGIN(pt);
  while(1) {
    #ifdef  Debug0_5Hz
    char strBuf[2][20];
    dtostrf(sys.gps_latitude, 1, 4, strBuf[0]);  dtostrf(sys.gps_longitude,1, 4, strBuf[1]);
    sprintf(prtBuf, "{\"N\":7,\"LAT\":%s,\"CK\":\"%X\"}", strBuf[0], rfcrc8);Serial3.println(prtBuf);
    sprintf(prtBuf, "{\"N\":8,\"LON\":%s,\"CK\":\"%X\"}", strBuf[1], rfcrc8);Serial3.println(prtBuf);
    sprintf(prtBuf, "{\"N\":22,\"SANT\":%d,\"CK\":\"%X\"}", sys.antena_status = deployAntStatus, rfcrc8);Serial3.println(prtBuf);
    sprintf(prtBuf, "{\"N\":23,\"SSLR\":%d,\"CK\":\"%X\"}", sys.solar_panel_status = solarPanelStatus, rfcrc8);Serial3.println(prtBuf);
    #endif
    PT_TIMER_DELAY(pt, 2000);
  }
  PT_END(pt);
}

static int thread_transmit_0_25Hz_task(struct pt *pt) {
  PT_BEGIN(pt);
  while(1) {
    #ifdef  Debug0_25Hz
    sprintf(prtBuf, "{\"N\":19,\"NSAT\":%d,\"CK\":\"%X\"}", sys.gps_satellites, rfcrc8);Serial3.println(prtBuf);
    sprintf(prtBuf, "{\"N\":10,\"GSPD\":%d,\"CK\":\"%X\"}", int(sys.gps_gndspeed*100), rfcrc8);Serial3.println(prtBuf);
    sprintf(prtBuf, "{\"N\":21,\"GALT\":%ld,\"CK\":\"%X\"}", sys.gps_altitude, rfcrc8);Serial3.println(prtBuf);
    sprintf(prtBuf, "{\"N\":26,\"FMTF\":%d,\"CK\":\"%X\"}", sys.fm_transmit_frq, rfcrc8);Serial3.println(prtBuf);
    #endif
    PT_TIMER_DELAY(pt, 4000);
  }
  PT_END(pt);
}

static int thread_transmit_0_2Hz_task(struct pt *pt) {
  PT_BEGIN(pt);
  while(1) {
    #ifdef  Debug0_5Hz
    char strBuf[20];
//    sprintf(prtBuf, "{\"N\":17,\"BSP\":%s,\"CK\":\"%X\"}", strBuf[baro_base_pressure], rfcrc8);Serial3.println(prtBuf);
    dtostrf(sys.gps_HDOP, 1, 1, strBuf);
    sprintf(prtBuf, "{\"N\":20,\"HDOP\":%s,\"CK\":\"%X\"}", strBuf, rfcrc8);Serial3.println(prtBuf);
    #endif
    PT_TIMER_DELAY(pt, 5000);
  }
  PT_END(pt);
}

