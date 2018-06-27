static struct pt pt_hmi_driver;
static struct pt pt_hmi_change_mode_task, pt_hmi_display_right_task, pt_hmi_display_left_task;
static struct pt pt_hmi_ctrl_fmradio_task;

//  Called by "CubeSat_V4"->"setup()"->first call be uesd to init this system.
void HMI_System_Init() {
  Serial.println("\r\nHMI System Init:\r\n");
  
  FMinit();
  delay(10);

  Serial.print("Get FMradio Frq and Vol:");
  
  uint8_t err_cnt = 5;
//  while( (sys.fm_transmit_frq < 870 || sys.fm_transmit_frq > 1080) && err_cnt-- ) {
//    
//    orderState();
//    delay(100);
//  
//    getState( sys.fm_transmit_vol, sys.fm_transmit_frq );
//    Serial.print(" . ");
//  }
  
  if( err_cnt  == 0 ) Serial.println("\r\nERROR:Get FMradio Failure!\r\n");
  
  Serial.print("FMradio:[ ");
  Serial.print(sys.fm_transmit_frq);
  Serial.print(" , ");
  Serial.print(sys.fm_transmit_vol);
  Serial.println(" ]");
  
  wsInit();

  Serial.println("\r\nHMI System Init Finished...\r\n");
  
  PT_INIT(&pt_hmi_driver);
}

void TASK_HMI_Handle() {
  
  thread_hmi_driver(&pt_hmi_driver);
  
  thread_hmi_change_mode_task(&pt_hmi_change_mode_task);
  thread_hmi_display_right_task(&pt_hmi_display_right_task);
  thread_hmi_display_left_task(&pt_hmi_display_left_task);
  thread_hmi_ctrl_fmradio_task(&pt_hmi_ctrl_fmradio_task);
}

static int thread_hmi_driver(struct pt *pt) {
  PT_BEGIN(pt);
  
  PT_INIT(&pt_hmi_change_mode_task);
  PT_INIT(&pt_hmi_display_right_task);
  PT_INIT(&pt_hmi_display_left_task);
  PT_INIT(&pt_hmi_ctrl_fmradio_task);

  PT_WAIT_THREAD(pt, 
         thread_hmi_change_mode_task(&pt_hmi_change_mode_task) &
         thread_hmi_display_right_task(&pt_hmi_display_right_task) &
         thread_hmi_display_left_task(&pt_hmi_display_left_task) &
         thread_hmi_ctrl_fmradio_task(&pt_hmi_ctrl_fmradio_task));
  PT_END(pt);
}

#define  Normal 0
#define  User   1
static uint8_t R = 0, G = 0, B = 150;
static char HMI_DISPLAY_MODE = 0;
static int thread_hmi_change_mode_task(struct pt *pt) {
  PT_BEGIN(pt);
  while(1){
    PT_WAIT_UNTIL(pt, sys_cmd.set_mled_flag == true);
    sys_cmd.set_mled_flag = false;
    
    R = (uint8_t)map( uint8_t(sys_cmd.set_mled_rgb >> 11) & 0b00011111, 0, 32, 0, 255);
    G = (uint8_t)map( uint8_t(sys_cmd.set_mled_rgb >>  5) & 0b00111111, 0, 64, 0, 255);
    B = (uint8_t)map( uint8_t(sys_cmd.set_mled_rgb >>  0) & 0b00011111, 0, 32, 0, 255);
    Serial.println(R, BIN);
    Serial.println(G, BIN);
    Serial.println(B, BIN);
  }
  PT_END(pt);
}

static int thread_hmi_display_right_task(struct pt *pt) {
  PT_BEGIN(pt);
  while(1){
    if( HMI_DISPLAY_MODE == Normal ) {
      char hmibuf[20];
      float temp;
      temp = sys.inside_temp;
      sprintf(hmibuf, "%d.%2d", (int)temp, int( (temp-(int)temp)*100 ) );
      static String str;
      str = "TEMP:";
      str += hmibuf;
      clearPixels("right");
      while( showStrRight(str, R, G, B) == 0 ) {      
//        if( HMI_DISPLAY_MODE != Normal ) PT_YIELD(pt);
        PT_TIMER_DELAY(pt, 50);
      }
    }

    if( HMI_DISPLAY_MODE == User ){
      clearPixels("right");
      for(char i=0; i<8; i++) {
        setPixels("right", i, sys_cmd.set_mled[i], R, G, B);//0行，B00000001，R:0，G:0，B:150
      }
      showPixels("right");
      PT_TIMER_DELAY(pt, 500);
    }
  }
  PT_END(pt);
}

static int thread_hmi_display_left_task(struct pt *pt) {
  PT_BEGIN(pt);
  while(1){
    if( HMI_DISPLAY_MODE == Normal ) {
      char hmibuf[20];
      float temp;
      temp = sys.inside_temp;
      sprintf(hmibuf, "%d.%2d", (int)temp, int( (temp-(int)temp)*100 ) );
      static String str;
      str = "TEMP:";
      str += hmibuf;
      clearPixels("left");
      while( showStrLeft(str, R, G, B) == 0 ) {
//        if( HMI_DISPLAY_MODE != Normal ) PT_YIELD(pt);
        PT_TIMER_DELAY(pt, 50);
      }
    }
    
    if( HMI_DISPLAY_MODE == User ){
      clearPixels("left");
      for(char i=0; i<8; i++) {
        setPixels("left", i, sys_cmd.set_mled[i], R, G, B);//0行，B00000001，R:0，G:0，B:150
      }
      showPixels("left");
      PT_TIMER_DELAY(pt, 500);
    }
  }
  PT_END(pt);
}

static int thread_hmi_ctrl_fmradio_task(struct pt *pt) {
  PT_BEGIN(pt);
  while(1) {
    PT_WAIT_UNTIL(pt, sys_cmd.set_fm_flag == true);
      sys_cmd.set_fm_flag = false;      
      setFre( int(sys_cmd.set_fm_frq *10) );
      PT_TIMER_DELAY(pt, 200);
      setVol( int(sys_cmd.set_fm_vol) );//  setup vol failure
      PT_TIMER_DELAY(pt, 200);
      
      Serial.print("FM:");
      Serial.print(int(sys_cmd.set_fm_frq *10));
      Serial.print(" , ");
      Serial.println(int(sys_cmd.set_fm_vol));
  }
  PT_END(pt);
}


