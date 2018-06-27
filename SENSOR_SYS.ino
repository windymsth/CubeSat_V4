//#define debugRTC

//年月日时分秒
static int y,m,d,h,f,s;

static struct pt pt_sensor_driver;
static struct pt pt_sensor_acq_task;
static struct pt pt_adjust_rtc_task;
static struct pt pt_camera_capture_task;

//  Called by "CubeSat_V4"->"setup()"->first call be uesd to init this system.
void Sensor_System_Init() {
  Serial.println("\r\nSensor System Init:\r\n");
  
  Wire.begin();
  RTCinit();
  shtInit(0x44);//add：0x44   初始化温湿度传感器sht31
  TSL2561Init();

  camInit();
  delay(200);
  clearInput();

  Serial.println("\r\nSensor System Init Finished...\r\n");
  
  PT_INIT(&pt_sensor_driver);
}

void TASK_Sensor_Handle(void) {
  
  thread_sensor_driver(&pt_sensor_driver);

  thread_sensor_acq_task(&pt_sensor_acq_task);
  thread_adjust_rtc_task(&pt_adjust_rtc_task);
  thread_camera_capture_task(&pt_camera_capture_task);
}

static int thread_sensor_driver(struct pt *pt) {
  PT_BEGIN(pt);
  
  PT_INIT(&pt_sensor_acq_task);
  PT_INIT(&pt_adjust_rtc_task);
  PT_INIT(&pt_camera_capture_task);

  PT_WAIT_THREAD(pt, 
         thread_sensor_acq_task(&pt_sensor_acq_task) &
         thread_adjust_rtc_task(&pt_adjust_rtc_task) &
         thread_camera_capture_task(&pt_camera_capture_task) );
         
  PT_END(pt);
}

static int thread_sensor_acq_task(struct pt *pt) {
  
  PT_BEGIN(pt);
  while(1) {
    getTime(&y,&m,&d,&h,&f,&s); 
    
    orderTH();
    PT_TIMER_DELAY(pt, 200);
    getTH( &sys.inside_temp, &sys.inside_humi);
    
    sys.uv_index = uvLevel(A0);
    sys.solar_panel_voltage = dcVal(A1);
    sys.lux = lightVal();

#ifdef debugRTC
    Serial.print(y);Serial.print("/"); Serial.print(m);Serial.print("/"); Serial.print(d);Serial.print("/"); 
    Serial.print(h);Serial.print(":"); Serial.print(f);Serial.print(":"); Serial.println(s);
#endif

    PT_TIMER_DELAY(pt, 700);
  }
  PT_END(pt);
}

static int thread_adjust_rtc_task(struct pt *pt) {
  
  PT_BEGIN(pt);
  while(1) {
    PT_WAIT_UNTIL(pt, sys_cmd.set_rtc_flag == true);
    sys_cmd.set_rtc_flag = false;

    adjTime(sys_cmd.set_adjust_time_year, sys_cmd.set_adjust_time_month, sys_cmd.set_adjust_time_day, 
            sys_cmd.set_adjust_time_hour, sys_cmd.set_adjust_time_minute, 0 );

    PT_TIMER_DELAY(pt, 200);
    
    getTime(&y,&m,&d,&h,&f,&s);

    Serial.print("Now RTC Time: ");
    Serial.print(y);Serial.print("/"); Serial.print(m);Serial.print("/"); Serial.print(d);Serial.print("/"); 
    Serial.print(h);Serial.print(":"); Serial.print(f);Serial.print(":"); Serial.println(s);
  }
  PT_END(pt);
}

static int thread_camera_capture_task(struct pt *pt) {
  static long len = 0;
  static long time;
  static String name = "";
  PT_BEGIN(pt);
  while(1) {
    PT_WAIT_UNTIL(pt, sys_cmd.ctrl_cam_flag == true);
    sys_cmd.ctrl_cam_flag = false;
    
    if( sys_cmd.ctrl_cam_cap == true ) {
      //step 1: 停止刷新图像
      do{
        orderStop();
        PT_TIMER_DELAY(pt, 100);
      }while(!getStop());//check error

      //step 2: 获取数据长度到len
      do{
        orderLenth();
        PT_TIMER_DELAY(pt, 100);
      }while(getLenth(&len));

      //step 3:存入内存卡中
  
      //3.0 创建并开启文件
      preFile(&name);
      Serial.print("Begin write ");
      Serial.println(name);
      Serial.print("total lenth :");
      Serial.println(len);
    
      //3.1写入文件 
      while(orderPic()){//如果还能order（返回 1）
        //等数据
        PT_TIMER_DELAY(pt, 10);  
        //写数据
        getPic();       //则持续获取图片并写入文件  
        Serial.print(".");
      }
      Serial.println();
      //3.2完成后关闭文件
      closeFile(); 
    
    
      //step 4:恢复刷新屏幕
      refreshImg();
      Serial.println(" is complated.");
      PT_TIMER_DELAY(pt, 1000);
    }
  }
  PT_END(pt);
}

