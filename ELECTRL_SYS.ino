static struct pt pt_electrl_driver;
static struct pt pt_electrl_manage_task, pt_electrl_execute_task;

const int solarPanelCW  = 37;
const int solarPanelCCW = 38;

const int deployAntCW   = 39;
const int deployAntCCW  = 40;

const int solarPanelCurrent = A5;
const int deployAntCurrent  = A6;

//  Called by "CubeSat_V4"->"setup()"->first call be uesd to init this system.
void Electrl_System_Init() {
  pinMode(solarPanelCW, OUTPUT);
  pinMode(solarPanelCCW, OUTPUT);
  pinMode(deployAntCW, OUTPUT);
  pinMode(deployAntCCW, OUTPUT);

  digitalWrite(solarPanelCW, LOW);
  digitalWrite(solarPanelCCW, LOW);
  digitalWrite(solarPanelCW, LOW);
  digitalWrite(solarPanelCCW, LOW);

  PT_INIT(&pt_electrl_driver);
}

void TASK_Electrl_Handle() {
  
  thread_electrl_driver(&pt_electrl_driver);
  
  thread_electrl_manage_task(&pt_electrl_manage_task);
  thread_electrl_execute_task(&pt_electrl_execute_task);
}

static int thread_electrl_driver(struct pt *pt) {
  PT_BEGIN(pt);
  
  PT_INIT(&pt_electrl_manage_task);
  PT_INIT(&pt_electrl_execute_task);

  PT_WAIT_THREAD(pt, 
         thread_electrl_manage_task(&pt_electrl_manage_task) &
         thread_electrl_execute_task(&pt_electrl_execute_task));
  PT_END(pt);
}

static int thread_electrl_manage_task(struct pt *pt) {
  PT_BEGIN(pt);
  while(1){
    PT_WAIT_UNTIL(pt, sys_cmd.ctrl_flag == true);
      sys_cmd.ctrl_flag = false;
  }
  PT_END(pt);
}

static int thread_electrl_execute_task(struct pt *pt) {
  PT_BEGIN(pt);
  while(1){

    PT_TIMER_DELAY(pt, 500);
  }
  PT_END(pt);
}


