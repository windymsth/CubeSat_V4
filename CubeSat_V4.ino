#define PT_USE_TIMER
#define PT_USE_SEM
#include "pt.h"
#include "commINC.h"
#include "SYS_DATA.H"

static struct pt pt_test_task;
static struct pt_sem sem_LED;

#define SERIAL_RX_BUFFER_SIZE 256

void setup() {
  //Set Hardware
  pinMode(13,OUTPUT);
  Serial.begin(115200);

  Electrl_System_Init();
  HMI_System_Init();
  Postion_System_Init();
  RF_System_Init();
  Sensor_System_Init();
  
  //Initialize Semaphore
  PT_SEM_INIT(&sem_LED,1);
  
  //Initialize the Threads
  PT_INIT(&pt_test_task);
  Serial.println("step:boot\r\n");
}

static int thread_test_task(struct pt *pt)
{
  static long nowtime = 0;
  PT_BEGIN(pt);
  while (1) {
    Data_Str_stack();
    Serial.print("Data_Str_stack: ");
    Serial.println(millis() - nowtime);
    nowtime = millis();
    PT_TIMER_DELAY(pt, 2000);    
  }
  PT_END(pt);
}

void loop() {
  //Check each thread by priority
  TASK_Electrl_Handle();
  
  TASK_HMI_Handle();

  TASK_Postion_Handle();

  TASK_RF_Handle();

  TASK_Sensor_Handle();

  thread_test_task(&pt_test_task);
  
}

