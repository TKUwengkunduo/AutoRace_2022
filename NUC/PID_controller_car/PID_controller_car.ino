#include <DynamixelWorkbench.h>
//#include <random.h>

#define DXL_BUS_SERIAL3 "3"            //Dynamixel on Serial3(USART3)  <-OpenCM 485EXP
#define BAUDRATE  1000000
#define R_Motor_ID    1
#define L_Motor_ID    2
//#define 
//#define KP             0.8
#define KI             0
#define SERIAL Serial2
float KP[6]={1.8, 2.8, 2.8, 1.7, 1.7, 1.5};
int target_speed[6] = {130, 125, 125, 130 ,100, 35};
int error[50]={0};
int  R_speed=0;
int L_speed=0;
int temp=0;
int jetson_sign,old_jetson_sign;
int sign = 48;
int flag = 0;
unsigned long newTime = 0;
unsigned long oldTime = 0;

DynamixelWorkbench R_Motor;
DynamixelWorkbench L_Motor;



void setup()
{
  Serial.begin(9600);
  SERIAL.begin(9600);
  Serial.setTimeout(100);
  SERIAL.setTimeout(100);

  R_Motor.begin(DXL_BUS_SERIAL3, BAUDRATE);
  L_Motor.begin(DXL_BUS_SERIAL3, BAUDRATE);

  R_Motor.ping(R_Motor_ID);
  L_Motor.ping(L_Motor_ID);

  R_Motor.wheelMode(R_Motor_ID);
  L_Motor.wheelMode(L_Motor_ID);

  
}






/*
 * Arduino傳送'8'代表欲接收Jetson訊號
 * 
 * Jetson回傳訊號:
 * 0 = 無看到標示
 * 1 = 左轉
 * 2 = 右轉
 * 3 = 避障
 * 4 = 停車
 * 5 = 柵欄
 * 6 = 隧道
 * 
 * 
 * 傳送NUC訊號:
 * 0 = 雙循線
 * 1 = 左循
 * 2 = 右循
 * 3 = 避障
 * 4 = 雙黃
 * 5 = 
 * 6 = 
 * 8 = 結束本輪任務(模式更換)
 * 9 = 停車
 */

void loop()
{
  
  SERIAL.print('0');
  delay(500);
  while(1){
    SERIAL.print('0');
    jetson_sign = SERIAL.read();
    jetson_sign = jetson_sign - sign ;
    if(jetson_sign==9){
      R_Motor.goalSpeed(R_Motor_ID, -80);
      L_Motor.goalSpeed(L_Motor_ID, -80);
      delay(10000);
      R_Motor.goalSpeed(R_Motor_ID, 0);
      L_Motor.goalSpeed(L_Motor_ID, 0);
      break;
    }
    delay(300);
  }

  oldTime = millis();
  while(1){
    newTime = millis();
    SERIAL.print('8');
    jetson_sign = SERIAL.read();
    jetson_sign = jetson_sign - sign ;
    
  //  Serial.print("jetson_sign= ");
  //  Serial.println(jetson_sign);
  
    if(old_jetson_sign!=jetson_sign){
      Serial.print('8');
      Serial.print('8');
      old_jetson_sign = jetson_sign;
    }
    // 無看到標誌
    if(jetson_sign==0){
      double_line();
    // 左轉
    }else if(jetson_sign==1){
      Serial.print('1');
      for(int i=0;i<=30;i++){
        stop_car();
      }
      R_Motor.goalSpeed(R_Motor_ID, -30);
      delay(1000);
      for(int i=0;i<=140;i++){
        left_line();
      }
      Serial.print('8');
      Serial.print('0');
      Serial.print('0');
      for(int i=0;i<=5;i++){
        stop_car();
      }
  
    // 右轉
    }else if(jetson_sign==2){
      for(int i=0;i<=30;i++){ 
        stop_car();
      }
      L_Motor.goalSpeed(L_Motor_ID, -30);
      delay(1000);
      for(int i=0;i<=140;i++){
        right_line();
      }
      Serial.print('8');
      Serial.print('0');
      Serial.print('0');
      for(int i=0;i<=5;i++){
        stop_car();
      }
  
    // 避障
    }else if(jetson_sign==3){
      Serial.print('3');
      for(int i=0;i<=70;i++){
        stop_car();
      }
      oldTime = millis();
      for(int i=0;i<=275;i++){
        newTime = millis();
        Avoidance();
      }
      Serial.print('8');
      Serial.print('0');
      Serial.print('0');
  
    // 停車
    }else if(jetson_sign==4){
      flag = 1;
      double_yellow_line();

      for(int i=0;i<=25;i++){
        stop_car();
      }
      R_Motor.goalSpeed(R_Motor_ID, -30);
      delay(1000);
      for(int i=0;i<=120;i++){
        left_line();
      }
      Serial.print('8');
      Serial.print('0');
      Serial.print('0');
      for(int i=0;i<=5;i++){
        stop_car();
      }

      
//      stop_car();
//      delay(1000);
//      for(int i=0;i<=15;i++){
//        left_line();
//      }
//      stop_car();
//      delay(1000);
//      for(int i=0;i<=45;i++){
//        error[0] = Serial.readString().toInt();
//        double_yellow_line();
//      }
//      for(int i=0;i<=50;i++){
//        left_line();
//      }
  
    // 柵欄
    }else if(jetson_sign==5){
      Serial.print('8');
      Serial.print('0');
      for(int i=0;i<=40;i++){
        Serial.print('0');
        train_double_line();
        SERIAL.read();
      }
      stop_car();
      delay(800);
      SERIAL.read();
      Serial.print('7');
      while(SERIAL.read()==5){
        stop_car();
        delay(1000);
        Serial.print('7');
      }
  
    // 隧道
    }else if(jetson_sign==6 && flag == 1){
      for(int i=0;i<=18;i++){
        Serial.print('0');
        double_line();
        SERIAL.read();
      }
      for(int i=0;i<=4;i++){
        go_straight();
      }
      turn_left();
      go_straight();
      turn_right();
      go_straight();
      turn_left();
      go_straight_3();
      turn_right();
      go_straight();
      turn_left();
      for(int i=0;i<=15;i++){
        go_straight_3();
      }
    // 例外狀況
    }else{
      double_line();
    }
  }
}





/*
 * PID計算
 */
int PID_conrtoller(char RL, int mode){
  float wheel_speed=0;
  float P_pid=0, I_pid=0;
  int error_sum=0;
  int i;

  if(mode==2&&abs(error[0])>60){
    P_pid = (error[0])*KP[mode]*1.0;
  }else{
    P_pid = (error[0])*KP[mode];
  }

  if(RL=='R'){
    wheel_speed = int(target_speed[mode]+P_pid);
  }else{
    wheel_speed = int(target_speed[mode]-P_pid);
  }

  if(target_speed[mode]<wheel_speed){
    wheel_speed = wheel_speed/abs(wheel_speed/target_speed[mode]);
  }
  
  return wheel_speed;
}


void go_straight(){
  R_speed = 100;
  L_speed = 100;
  R_Motor.goalSpeed(R_Motor_ID, -R_speed);
  L_Motor.goalSpeed(L_Motor_ID, -L_speed);
  delay(3500);
}
void go_straight_2(){
  R_speed = 100;
  L_speed = 100;
  R_Motor.goalSpeed(R_Motor_ID, -R_speed);
  L_Motor.goalSpeed(L_Motor_ID, -L_speed);
  delay(3100);
}
void go_straight_3(){
  R_speed = 200;
  L_speed = 200;
  R_Motor.goalSpeed(R_Motor_ID, -R_speed);
  L_Motor.goalSpeed(L_Motor_ID, -L_speed);
  delay(3500);
}
void turn_left(){
  R_speed = 80;
  L_speed = -80;
  R_Motor.goalSpeed(R_Motor_ID, -R_speed);
  L_Motor.goalSpeed(L_Motor_ID, -L_speed);
  delay(2000);
}

void turn_around(){
  R_speed = 80;
  L_speed = -80;
  R_Motor.goalSpeed(R_Motor_ID, -R_speed);
  L_Motor.goalSpeed(L_Motor_ID, -L_speed);
  delay(4000);
}

void turn_right(){
  R_speed = -80;
  L_speed = 80;
  R_Motor.goalSpeed(R_Motor_ID, -R_speed);
  L_Motor.goalSpeed(L_Motor_ID, -L_speed);
  delay(2000);
}



/*
 * 雙循線
 */
void double_line(){
  if(newTime-oldTime>180){
    Serial.print('0');
    oldTime = newTime;
  }
  
  error[0] = Serial.readString().toInt();
  if(error[0]==0){
    R_speed = R_speed;
    L_speed = R_speed;
    R_Motor.goalSpeed(R_Motor_ID, -R_speed);
    L_Motor.goalSpeed(L_Motor_ID, -L_speed);
  }else if(error[0]==-999){
    R_Motor.goalSpeed(R_Motor_ID, 0);
    L_Motor.goalSpeed(L_Motor_ID, 0);
  }else{
    R_speed = PID_conrtoller('R', 0);
    L_speed = PID_conrtoller('L', 0);
    R_Motor.goalSpeed(R_Motor_ID, -R_speed);
    L_Motor.goalSpeed(L_Motor_ID, -L_speed);
  }
}
void train_double_line(){
  if(newTime-oldTime>180){
    Serial.print('0');
    oldTime = newTime;
  }
  
  error[0] = Serial.readString().toInt();
  if(error[0]==0){
    R_speed = R_speed;
    L_speed = R_speed;
    R_Motor.goalSpeed(R_Motor_ID, -R_speed);
    L_Motor.goalSpeed(L_Motor_ID, -L_speed);
  }else if(error[0]==-999){
    R_Motor.goalSpeed(R_Motor_ID, 5);
    L_Motor.goalSpeed(L_Motor_ID, 5);
  }else{
    R_speed = PID_conrtoller('R', 5);
    L_speed = PID_conrtoller('L', 5);
    R_Motor.goalSpeed(R_Motor_ID, -R_speed);
    L_Motor.goalSpeed(L_Motor_ID, -L_speed);
  }
}

/*
 * 左循線
 */
void left_line(){
  Serial.print('1');


  error[0] = Serial.readString().toInt()+10;

  if(error[0]==0){
    R_speed = R_speed;
    L_speed = R_speed;
    R_Motor.goalSpeed(R_Motor_ID, -R_speed);
    L_Motor.goalSpeed(L_Motor_ID, -L_speed);
  }else if(error[0]==-999){
    R_Motor.goalSpeed(R_Motor_ID, 0);
    L_Motor.goalSpeed(L_Motor_ID, 0);
  }else{
    R_speed = PID_conrtoller('R', 1);
    L_speed = PID_conrtoller('L', 1);
    R_Motor.goalSpeed(R_Motor_ID, -R_speed);
    L_Motor.goalSpeed(L_Motor_ID, -L_speed);
  }
}


/*
 * 右循線
 */
void right_line(){
  Serial.print('2');

  error[0] = Serial.readString().toInt()-15;

  if(error[0]==0){
    R_speed = R_speed;
    L_speed = R_speed;
    R_Motor.goalSpeed(R_Motor_ID, -R_speed);
    L_Motor.goalSpeed(L_Motor_ID, -L_speed);
  }else if(error[0]==-999){
    R_Motor.goalSpeed(R_Motor_ID, 0);
    L_Motor.goalSpeed(L_Motor_ID, 0);
  }else{
    R_speed = PID_conrtoller('R', 2);
    L_speed = PID_conrtoller('L', 2);
    R_Motor.goalSpeed(R_Motor_ID, -R_speed);
    L_Motor.goalSpeed(L_Motor_ID, -L_speed);
  }
  
}


/*
 * 雙黃循線
 */
void double_yellow_line(){
  Serial.print('4');
  Serial.print('4');
  Serial.print('4');
  Serial.print('4');
  Serial.print('4');
  while(1){
    
    error[0] = Serial.readString().toInt();

    if(error[0]==-777){
      stop_car();
      break;
    }
    
  
    if(error[0]==0){
      R_speed = R_speed;
      L_speed = R_speed;
      R_Motor.goalSpeed(R_Motor_ID, -R_speed);
      L_Motor.goalSpeed(L_Motor_ID, -L_speed);
    }else if(error[0]==-999){
      R_Motor.goalSpeed(R_Motor_ID, 0);
      L_Motor.goalSpeed(L_Motor_ID, 0);
    }else{
      R_speed = PID_conrtoller('R', 4);
      L_speed = PID_conrtoller('L', 4);
      R_Motor.goalSpeed(R_Motor_ID, -R_speed);
      L_Motor.goalSpeed(L_Motor_ID, -L_speed);
    }
  }
  for(int i=0;i<=100;i++){
        stop_car();
  }

  Serial.print('9');
  delay(2000);
  while(1){
    error[0] = Serial.readString().toInt();
    if(error[0]==-99){
      go_straight();
      turn_right();
      go_straight_2();
      turn_around();
      go_straight();
      turn_left();
      go_straight();
      delay(700);
      break;
    }
    else if(error[0]==-88){
      go_straight();
      turn_left();
      go_straight_2();
      turn_around();
      go_straight();
      turn_right();
      go_straight();
      delay(700);
      break;
    }
    else{
      delay(20);
      Serial.print('9');
    }
  }

  
//  Serial.print('4');
//  error[0] = Serial.readString().toInt()+10;
//  
//  if(error[0]==0){
//    R_speed = R_speed;
//    L_speed = R_speed;
//    R_Motor.goalSpeed(R_Motor_ID, -R_speed);
//    L_Motor.goalSpeed(L_Motor_ID, -L_speed);
//  }
//
// // 雙黃線出現分岔
//  if(error[0]==-777){
//    stop_car();
//    for(int i=0;i<=5;i++){
//      Serial.print('9');
//      delay(200);
//      error[0] = Serial.readString().toInt();
//    }
//    do{
//      error[0] = Serial.readString().toInt();
//    }while(error[0]!=-77 && error[0]!=-7);
//    
//    if(error[0]==-77){
//      go_straight();
//      turn_right();
//      go_straight();
//      turn_around();
//      go_straight();
//      turn_left();
//      go_straight();
//      delay(700);
//    }
//    else{
//      go_straight();
//      turn_left();
//      go_straight();
//      turn_around();
//      go_straight();
//      turn_right();
//      go_straight();
//      delay(700);
//    }
//  }
//
//  //  (不明?)
//  else if(error[0]==-999){
//    stop_car();
//
//  // 正常狀況下，正常雙黃循線
//  }else{
//    R_speed = PID_conrtoller('R', 4);
//    L_speed = PID_conrtoller('L', 4);
//    R_Motor.goalSpeed(R_Motor_ID, -R_speed);
//    L_Motor.goalSpeed(L_Motor_ID, -L_speed);
//  }
}



/*
 * 避障
 */
void Avoidance(){
  if(newTime-oldTime>2000){
    Serial.print('3');
    oldTime = newTime;
  }
  
  error[0] = Serial.readString().toInt();

  if(error[0]==0){
    R_speed = R_speed;
    L_speed = R_speed;
    R_Motor.goalSpeed(R_Motor_ID, -R_speed);
    L_Motor.goalSpeed(L_Motor_ID, -L_speed);
  }else if(error[0]==-999){
    R_Motor.goalSpeed(R_Motor_ID, 0);
    L_Motor.goalSpeed(L_Motor_ID, 0);
  }else{
    R_speed = PID_conrtoller('R', 3);
    L_speed = PID_conrtoller('L', 3);
    R_Motor.goalSpeed(R_Motor_ID, -R_speed);
    L_Motor.goalSpeed(L_Motor_ID, -L_speed);

  }
}


/*
 * 柵欄
 */


void stop_car(){
  R_speed = 0;
  L_speed = 0;
  R_Motor.goalSpeed(R_Motor_ID, -R_speed);
  L_Motor.goalSpeed(L_Motor_ID, -L_speed);
}

void fffff(int d){
  Serial.print(char(d));
}
