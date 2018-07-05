#define solarPanel 0x00
#define deployAnt 0x01

#define close false
#define open  true
static struct pt pt_electrl_driver;
static struct pt pt_electrl_manage_task, pt_electrl_execute_task;

// PIN
static int solarPanelStatus  = false;
const int solarPanel_Left_LimitClose  = 5;
const int solarPanel_Right_LimitClose  = 38;

static int deployAntStatus   = false;
const int deployAntLimitClose   = 30;

// Var
static int CTRL_DEPLOY_ANT = close;
static int solarPanel_execute_flag = false;
static int solarPanel_readOK_flag = false;
static int deployAnt_readOK_flag = false;
static int deployAnt_execute_flag = false;
static int solarPanel_offset_zero = 0;
static int deployAnt_offset_zero = 0;

int overload = 10;
int readSevroStatus(int i) {
  NOP;
}

//  Called by "CubeSat_V4"->"setup()"->first call be uesd to init this system.
void Electrl_System_Init() {
  Serial.println("\r\nElectrl System Init:\r\n");
  
  pinMode(solarPanel_Left_LimitClose, INPUT);
  pinMode(solarPanel_Right_LimitClose, INPUT);  
//  pinMode(deployAntLimitClose, INPUT);

  digitalWrite(solarPanel_Left_LimitClose, HIGH);
  digitalWrite(solarPanel_Right_LimitClose, HIGH);
//  digitalWrite(deployAntLimitClose, HIGH);

  solarPanelStatus  = close;
  deployAntStatus   = close;

  Serial.print("\r\nServo Init:\r\n");
  ssInit();
  int err_cnt;
  for( err_cnt=0; err_cnt<5; err_cnt++ ) {
    solarPanel_offset_zero = readAngle( solarPanel );
    deployAnt_offset_zero = readAngle( deployAnt );
    Serial.print(solarPanel_offset_zero);
    Serial.print(',');
    Serial.print(deployAnt_offset_zero);
    Serial.print('\t');
    if( solarPanel_offset_zero != -491 && solarPanel_offset_zero != -491 ) {
      Serial.print("\r\nServo ReadAngle OK:[");
      Serial.print(solarPanel_offset_zero);
      Serial.print(" , ");
      Serial.print(solarPanel_offset_zero);
      Serial.print("]\r\n");
      break;
    }
  }
  if( err_cnt >= 5 ) {
    Serial.print("\r\nERROR:Servo Init Failure!\r\n");
  }
//      moveToAngle(0,120,500);moveToAngle(1,120,500);
//      delay(1000);
//      Serial.println(readAngle(0));Serial.println(readAngle(1));
//      
//      moveToAngle(0,30,500);moveToAngle(1,30,500);
//      delay(1000);
//      Serial.println(readAngle(0));Serial.println(readAngle(1));
//      
//      moveToAngle(0,120,500);moveToAngle(1,120,500);
//      delay(2000);
//      Serial.println(readAngle(0));Serial.println(readAngle(1)); 

    Serial.print("\r\nLimitKey:[");
    Serial.print( digitalRead(solarPanel_Left_LimitClose), DEC );
    Serial.print(" , ");
    Serial.print( digitalRead(solarPanel_Right_LimitClose), DEC );
    Serial.print("]\r\n");
      
  //  solarPanel Servo zeroPosition offset
  if( digitalRead(solarPanel_Left_LimitClose) == LOW &&
      digitalRead(solarPanel_Right_LimitClose) == LOW ) {
    delay(20);  //  Key anti shake 
    
    if( digitalRead(solarPanel_Left_LimitClose) == LOW &&
        digitalRead(solarPanel_Right_LimitClose) == LOW ) {
          
//      solarPanel_offset_zero = readAngle( solarPanel );
      Serial.print("\r\nSolarPanel Offset Zero:[");
      Serial.print(solarPanel_offset_zero);
      Serial.print("]\r\n");
      
      if( solarPanel_offset_zero >= 110 && solarPanel_offset_zero <= 120 )
        solarPanel_readOK_flag = true;
      else
        Serial.println("ERROR:Servo Angle read abnormal!\r\n"); 
    }
  }

//  deployAnt Servo zeroPosition offset
Init_deployAnt:
  if( digitalRead(deployAntLimitClose) == LOW ) {
    delay(20);  //  Key anti shake
    
    if( digitalRead(deployAntLimitClose) == LOW ) {

      //  Servo angle read normal should is 120
      deployAnt_offset_zero = readAngle( deployAnt );
      Serial.println(deployAnt_offset_zero);
      
      if( deployAnt_offset_zero >= 0 && deployAnt_offset_zero <= 240 )
        deployAnt_readOK_flag = true;
      else  
        Serial.println("ERROR:Servo Angle read abnormal!\r\n");
        
    } else {
      //  deployANT is not closed status, when boot.
      //  Servo angle read normal should is 120
      for(static uint8_t bkCnt = 0; bkCnt < 2; bkCnt++) {
        
        deployAnt_offset_zero = readAngle( deployAnt );
        if( deployAnt_offset_zero < 10 && deployAnt_offset_zero > 230 ) {
          Serial.println("\r\nWarning : deployAnt Servo setup status problems!\r\n");
        } else {
          if( deployAnt_offset_zero < 120 ) deployAnt_offset_zero = 130;
          if( deployAnt_offset_zero > 120 ) deployAnt_offset_zero = 110;
          
          load( deployAnt );
          moveToAngle(deployAnt, deployAnt_offset_zero, 2500);
          
          long nowtimes = millis();
          while( (millis() - nowtimes) > 2600 ) {
            nowtimes = millis();
            if( digitalRead(deployAntLimitClose) == LOW ) {
              unLoad( deployAnt );
              goto Init_deployAnt;  //  ***goto
            }
            if( readSevroStatus( deployAnt ) == overload ){
              unLoad( deployAnt );
              break;
            }
            delay(1);
          } //  end of while

          Serial.println("\r\nWarning : deployAnt Servo Reset Failure!\r\n");
          Serial.println("\r\nWarning : deployAnt Servo setup status problems!\r\n");
        } //  end of else
      } //  end of for
      Serial.println("\r\nWarning : deployAnt Servo Reinit Failure!\r\n");
      Serial.println("\r\nWarning : deployAnt Servo setup status problems!\r\n");
    } //  end of else
  }
  
  unLoad(solarPanel);
  unLoad(deployAnt);
  
  Serial.println("\r\nElectrl System Init Finished...\r\n");
  
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
    PT_WAIT_UNTIL(pt, sys_cmd.ctrl_solar_flag == true || sys_cmd.ctrl_ant_flag == true );

      if( solarPanelStatus == close && sys_cmd.ctrl_solar_flag == true) {
        sys_cmd.ctrl_solar_flag = false;

        //  limit pin is LOW trig, HIGH mean's panel is open
        if( digitalRead(solarPanel_Left_LimitClose) == HIGH &&
            digitalRead(solarPanel_Right_LimitClose) == HIGH &&
            solarPanel_readOK_flag == true ) {
          
          solarPanelStatus = open;

          solarPanel_execute_flag = true;
        }
      }

      if( solarPanelStatus == open && sys_cmd.ctrl_solar_flag == true ) {
        sys_cmd.ctrl_solar_flag = false;

        // if solar panel is open status
        if( digitalRead(solarPanel_Left_LimitClose) == HIGH && 
            digitalRead(solarPanel_Right_LimitClose) == HIGH ) {
              
          solarPanel_execute_flag = true;
        }
      }

      if( deployAntStatus == close && sys_cmd.ctrl_ant_flag == true ) {
        sys_cmd.ctrl_ant_flag = false;
        
        //  limit pin is LOW trig, LOW mean's panel is closed
        if( digitalRead(deployAntLimitClose) == LOW && solarPanel_readOK_flag == true ) {
          if( sys_cmd.ctrl_ant == open ) {
            
            deployAnt_execute_flag = true;
          }
        }
      }

      if( deployAntStatus == open  && sys_cmd.ctrl_ant_flag == true ) {
        sys_cmd.ctrl_ant_flag = false;
        
         //  limit pin is LOW trig, LOW mean's panel is closed
        if( digitalRead(deployAntLimitClose) == HIGH ) {
          
          if( sys_cmd.ctrl_ant == close ) {

            deployAnt_execute_flag = true;
          }
        }
      }
  }
  PT_END(pt);
}

static int thread_electrl_execute_task(struct pt *pt) {
  PT_BEGIN(pt);
  while(1){
//    PT_WAIT_UNTIL(pt, solarPanel_execute_flag == true);

    if( solarPanelStatus == open && solarPanel_execute_flag == true ) {
      load( solarPanel );
//      moveToAngle(solarPanel, sys.ctrl_solar_angle + solarPanel_offset_zero + 90, 2500);
      moveToAngle(solarPanel, sys_cmd.ctrl_solar_angle + solarPanel_offset_zero, 2500);
      PT_TIMER_DELAY(pt, 2500);

      solarPanel_execute_flag = false;
    }

    if( solarPanelStatus == open && sys_cmd.ctrl_solar_reset == true ) {
      load( solarPanel );
      moveToAngle(solarPanel, 0 + solarPanel_offset_zero, 2500);
      PT_TIMER_DELAY(pt, 2500);
      unLoad( solarPanel );
      
      solarPanelStatus = close;
      solarPanel_execute_flag = false;
    }

    //  deploy ANT
    if( CTRL_DEPLOY_ANT == open && deployAnt_execute_flag == true ) {
      load( deployAnt );
      moveToAngle(deployAnt, sys_cmd.ctrl_ant + deployAnt_offset_zero, 3000);
      
      PT_TIMER_DELAY(pt, 3000);
      unLoad( deployAnt );
      
      deployAntStatus = open;

      deployAnt_execute_flag = false;
    }

    //  close ANT
    if( CTRL_DEPLOY_ANT == close && deployAnt_execute_flag == true ) {
      load( deployAnt );
      moveToAngle(deployAnt, 0 + deployAnt_offset_zero, 3000);
      
      PT_TIMER_DELAY(pt, 3000);
      unLoad( deployAnt );

      deployAntStatus = close;
      
      deployAnt_execute_flag = false;
    }

//    if( digitalRead(deployAntLimitOpen) == LOW ) {
//        unLoad( deployAnt );
//    }
    
  PT_TIMER_DELAY(pt, 20);
  }
  PT_END(pt);
}


void deployAnt_Servo_Init() {
//  deployAnt Servo zeroPosition offset
Init_deployAnt:
  if( digitalRead(deployAntLimitClose) == LOW ) {
    delay(20);  //  Key anti shake
    
    if( digitalRead(deployAntLimitClose) == LOW ) {

      //  Servo angle read normal should is 120
      deployAnt_offset_zero = readAngle( deployAnt );
      Serial.println(deployAnt_offset_zero);
      
      if( deployAnt_offset_zero >= 0 && deployAnt_offset_zero <= 240 )
        deployAnt_readOK_flag = true;
      else  
        Serial.println("ERROR:Servo Angle read abnormal!\r\n");
        
    } else {
      //  deployANT is not closed status, when boot.
      //  Servo angle read normal should is 120
      for(static uint8_t bkCnt = 0; bkCnt < 2; bkCnt++) {
        
        deployAnt_offset_zero = readAngle( deployAnt );
        if( deployAnt_offset_zero < 10 && deployAnt_offset_zero > 230 ) {
          Serial.println("\r\nWarning : deployAnt Servo setup status problems!\r\n");
        } else {
          if( deployAnt_offset_zero < 120 ) deployAnt_offset_zero = 130;
          if( deployAnt_offset_zero > 120 ) deployAnt_offset_zero = 110;
          
          load( deployAnt );
          moveToAngle(deployAnt, deployAnt_offset_zero, 2500);
          
          long nowtimes = millis();
          while( (millis() - nowtimes) > 2600 ) {
            nowtimes = millis();
            if( digitalRead(deployAntLimitClose) == LOW ) {
              unLoad( deployAnt );
              goto Init_deployAnt;  //  ***goto
            }
            if( readSevroStatus( deployAnt ) == overload ){
              unLoad( deployAnt );
              break;
            }
            delay(1);
          } //  end of while

          Serial.println("\r\nWarning : deployAnt Servo Reset Failure!\r\n");
          Serial.println("\r\nWarning : deployAnt Servo setup status problems!\r\n");
        } //  end of else
      } //  end of for
      Serial.println("\r\nWarning : deployAnt Servo Reinit Failure!\r\n");
      Serial.println("\r\nWarning : deployAnt Servo setup status problems!\r\n");
    } //  end of else
  }
  
}

