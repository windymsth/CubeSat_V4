//#include "commINC.h"

//#define debugRTC

//年月日时分秒
static int y,m,d,h,f,s;

static struct pt pt_sensor_driver;
static struct pt pt_sensor_acq_task;

//  Called by "CubeSat_V4"->"setup()"->first call be uesd to init this system.
void Sensor_System_Init() {
  Wire.begin();
  RTCinit();
  shtInit(0x44);//add：0x44   初始化温湿度传感器sht31
  TSL2561Init();
  
  PT_INIT(&pt_sensor_driver);
}

void TASK_Sensor_Handle(void) {
  
  thread_sensor_driver(&pt_sensor_driver);

  thread_sensor_acq_task(&pt_sensor_acq_task);
}

static int thread_sensor_driver(struct pt *pt) {
  PT_BEGIN(pt);
  
  PT_INIT(&pt_sensor_acq_task);

  PT_WAIT_THREAD(pt, 
         thread_sensor_acq_task(&pt_sensor_acq_task));
         
  PT_END(pt);
}

static int thread_sensor_acq_task(struct pt *pt) {
  
  PT_BEGIN(pt);
  while(1) {
    getTime(&y,&m,&d,&h,&f,&s); 
     
    sht31dVal(&sys.inside_temp, &sys.inside_humi);
    sys.uv_index = uvLevel(A0);
    sys.solar_panel_voltage = dcVal(A1);
    sys.lux = lightVal();

#ifdef debugRTC
    Serial.print(y);Serial.print("/"); Serial.print(m);Serial.print("/"); Serial.print(d);Serial.print("/"); 
    Serial.print(h);Serial.print(":"); Serial.print(f);Serial.print(":"); Serial.println(s);
#endif

    PT_TIMER_DELAY(pt, 1000);
  }
  PT_END(pt);
}



