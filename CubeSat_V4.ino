#define PT_USE_TIMER
#define PT_USE_SEM
#include "pt.h"
#include "commINC.h"
#include "SYS_DATA.H"

static struct pt pt_test_task, pt_factory_task;
//static struct pt_sem sem_LED;

//#define SERIAL_RX_BUFFER_SIZE 256

static uint32_t boottime = 0;

//  2018/06/26

void setup() {
  //Set Hardware
  pinMode(13,OUTPUT);
  Serial.begin(115200);

  DataLog_System_Init();
  Electrl_System_Init();
  HMI_System_Init();
  Postion_System_Init();
  RF_System_Init();
  Sensor_System_Init();

  Setup_Watchdog(6); //  6=1 sec
  
  //Initialize Semaphore
//  PT_SEM_INIT(&sem_LED,1);
  
  //Initialize the Threads
  PT_INIT(&pt_test_task);
//  PT_INIT(&pt_factory_task);
  Serial.println("step:boot\r\n");
  boottime = millis();
}

static int thread_factory_task(struct pt *pt)
{
  PT_BEGIN(pt);
  while (1) {
    if( millis() - boottime <= 10000 ) {
      Serial.println(sys_cmd.factory_mode);
      if( sys_cmd.factory_mode == 1 ){
        load( 0x00 );
        load( 0x01 );
          moveToAngle(0x00, 120, 2000);
          moveToAngle(0x01, 120, 2000);
        while(1);
      }
      PT_TIMER_DELAY(pt, 500);
    }
  }
  PT_END(pt);
}

static int thread_test_task(struct pt *pt)
{
  static long nowtime = 0;
  PT_BEGIN(pt);
  while (1) {
//    Data_Str_stack(); // Waste lots of time, *** need optimize
    Serial.print("Ms:");Serial.print(millis() - nowtime);Serial.println('\t');
    nowtime = millis();
    PT_TIMER_DELAY(pt, 1000);    
  }
  PT_END(pt);
}

void loop() {
  //Check each thread by priority
  TASK_DataLog_Handle();
  
  TASK_Electrl_Handle();
  
  TASK_HMI_Handle();

  TASK_Postion_Handle();

  TASK_RF_Handle();

  TASK_Sensor_Handle();

//  thread_factory_task(&pt_factory_task);
  thread_test_task(&pt_test_task);
}

