#include <Servo.h>
// #include <AFMotor.h>
#include <Wire.h>
#include <Adafruit_TCS34725.h>
#include <stdio.h>
#include <string.h>


// #include<RF24.h>
// #include<SPI.h>
// #include <nRF24L01.h>
// #include<LiquidCrystal_I2C.h>


{/*
Miso 12
Mosi 11
SCK 13

GFSK Modulation ISN frequency


*/}



// RC radio(8, 9);
// LC_I2C lcd(ads, 16, 2)

// const byte addresses[][3] = {"a2c", "c2a"};
// #define instration;

// const byte custome_char[8][8] = [
//   {
// 0b01110,
// 0b10001,
// 0b10001,
// 0b11111,
// 0b11011,
// 0b11011,
// 0b11111,
// 0b00000
// },
// ]


// void setup() {
//   // put your setup code here, to run once:
//   Serial.begin(9600);
//   lcd.init();
//   lcd.clear();         
//   lcd.backlight(); 
//   radio.begin();
//   radio.openWritingPipe(addresses[0]);
//   radio.openReadingPipe(0, addresses[1]);
//   radio.setPLALevel(RF24_PA_MAX);
//   lcd.createChar(0, Heart);
//   lcd.createChar(1, Bell);
//   lcd.createChar(2, Alien);
//   lcd.createChar(3, Check);
//   lcd.createChar(4, Speaker);
//   lcd.createChar(5, Sound);
//   lcd.createChar(6, Skull);
//   lcd.createChar(7, Lock);
// }

// void loop() {
//   // put your main code here, to run repeatedly:
//   sender(instration)
// }


// void sender(instration){
//   radio.stopListening();
//   radio.write(&instration, sizeof(instration))
// }

// char reciever(){
//   radio.startListening();
//   char instration_received[10] = "";
//   if (radio.available()) {
//     radio.read(&instration_received, sizeof(instration_received));
//   }
//   return instration_received;
// }


// void screenWrite(c_pos, txt){
//   lcd.setCursor(c_pos);
//   lcd.print(txt)
//   lcd.write(char)
// }



// #include <NewPing.h>  // https://bitbucket.org/teckel12/arduino-new-ping/wiki/Home
// #include "dht.h"      // https://github.com/RobTillaart/DHTlib

// #define TRIGGER_PIN  9
// #define ECHO_PIN     10
// #define MAX_DISTANCE 400
// #define dht22 5 // DHT22 temperature and humidity sensor

// NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
// dht DHT; // Creats a DHT object

// int readDHT, temp, hum;
// float speedOfSound, distance, duration;

// void setup() {
//   Serial.begin(9600);
// }

// void loop() {
//   delay(100);

//   // Read temperature and humidity from DHT22 sensor
//   readDHT = DHT.read22(dht22); // Reads the data from the sensor
//   temp = DHT.temperature; // Gets the values of the temperature
//   hum = DHT.humidity; // Gets the values of the humidity


//   speedOfSound = 331.4 + (0.6 * temp) + (0.0124 * hum); // Calculate speed of sound in m/s

//   duration = sonar.ping_median(10); // 10 interations - returns duration in microseconds
//   duration = duration/1000000; // Convert mircroseconds to seconds
//   distance = (speedOfSound * duration)/2;
//   distance = distance * 100; // meters to centimeters

//   Serial.print("Distance: ");
//   Serial.print(distance);
//   Serial.println("cm");

// }


int m1_dir1 = 2;
int m1_dir2 = 3;

int m2_dir1 = 4;
int m2_dir2 = 6;

int m1_s = 9;
int m2_s = 10;



Adafruit_TCS34725 tcs;
Servo s_motor;
// AF_DCMotor m_1_fl(4);
// AF_DCMotor m_2_fr(3);
// AF_DCMotor m_3_bl(1);
// AF_DCMotor m_4_br(2);


const int trigger = 4;
const int echo = 5;
const int servo = 7;
const int color_1 = 0;
const int color_2 = 1;
const int s_ir1_l = 7;
const int s_ir2_cl = 8;
const int s_ir3_c = 11;
const int s_ir4_cr = 12;
const int s_ir5_r = 13;
// const int s_ir4_cr = A4;
// const int s_ir5_r = A5;

float ki ;
float kp ;
float kd ;
int error;
float speed_regulator;
float P;
float D;
float I;
float prev_err;
int desire;


long distance, duration;
int angle;
char path[] = {};
int m=0;
int j;
char opt_path[] = {};
char opt_paths[] = {};
int runs = 0;
int irs[4];
int regular_speed = 150;
int time_added = 0;
int time_added_rotaion = 0;
int opt_path_length = 1000;
int opt_run;
bool final_run = false;
bool algo_left = true;

void setup(){
  Serial.begin(9600);
  pinMode(trigger, OUTPUT);
  pinMode(echo, INPUT);
  s_motor.attach(servo);
  pinMode(color_1, INPUT);
  pinMode(color_2, INPUT);
  pinMode(s_ir1_l, INPUT);
  pinMode(s_ir2_cl, INPUT);
  pinMode(s_ir3_c, INPUT);
  pinMode(s_ir4_cr, INPUT);
  pinMode(s_ir5_r, INPUT);
  pinMode(m1_dir1, OUTPUT);
  pinMode(m1_dir2. OUTPUT);
  pinMode(m2_dir1, OUTPUT);
  pinMode(m2_dir2. OUTPUT);
  pinMode(m1_s, OUTPUT);
  pinMode(m2_s. OUTPUT);
  
  if(tcs.begin()){
    Serial.print("tcs is alright.");
  } else{
    Serial.print("Not connected.");
  };
}

void loop(){
  //servo_rotation();
  /*color_detection(wall_scanner());*/
  //colorDetection();
  
  testIR(s_ir1_l, s_ir2_cl, s_ir3_c , s_ir4_cr, s_ir5_r);
  //motor_test();
  // line_scanner();
  // moveStraight();
  moveStraight();

}


void servo_rotation(){
  for(angle; angle<=180; angle+=90){
    delay(200);
    s_motor.write(angle);
    wall_scanner_writter(wall_scanner());
    delay(200);
  }
  for(angle; angle>0; angle-=90){
    s_motor.write(angle);
    wall_scanner_writter(wall_scanner());
    delay(200);
  }
}

int wall_scanner(){
  digitalWrite(trigger, LOW);
  delayMicroseconds(2);
  digitalWrite(trigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigger, LOW);
  duration = pulseIn(echo, HIGH);
  distance = (duration*0.034/2)+1;
  return distance;  
}

void wall_scanner_writter(int distance){
  if (distance==1 or distance==0) {
    Serial.println("No echo response.");
  } else {
    Serial.println(distance);
  }
}


void color_detection(int wavelenght){
  if(wavelenght > 10) {
    analogWrite(color_1, 255);
    analogWrite(color_2, 0);
  } else {
    analogWrite(color_2, 255);
    analogWrite(color_1, 0);
  }
}


int testIR(int ir1, int ir2, int ir3, int ir4, int ir5){
  irs[1] = digitalRead(ir1);
  irs[2] = digitalRead(ir2);
  irs[3] = digitalRead(ir3);
  irs[4] = digitalRead(ir4);
  irs[5] = digitalRead(ir5);
  for(int i=1; i<=5; i++){
    Serial.print(irs[i]);
  }
  Serial.print("\n");
  delay(1000);
  return irs;
}


void colorDetection(){
  uint16_t r, g, b, c;
  tcs.getRawData(&r, &g, &b, &c);
  
  Serial.print("Red: ");
  Serial.println(r);
  Serial.print("Green: ");
  Serial.println(g);
  Serial.print("Blue: ");
  Serial.println(b);
  Serial.print("Clear: ");
  Serial.println(c);
  Serial.println();
  
  delay(1000);
}


/*void motor_test(){
  uint8_t i;
  m_1.run(BACKWARD);
  m_2.run(FORWARD);
  m_3.run(BACKWARD);
  m_4.run(FORWARD);
	
	// Accelerate from zero to maximum speed
	for (i=0; i<255; i++) 
	{
		m_4.setSpeed(i);  
		delay(10);
	}
	
	// Decelerate from maximum speed to zero
	for (i=255; i!=0; i--) 
	{
		m_4.setSpeed(i);  
		delay(10);
	}

	// Now change m_4 direction
	m_4.run(BACKWARD);
	
	// Accelerate from zero to maximum speed
	for (i=0; i<255; i++) 
	{
		m_4.setSpeed(i);  
		delay(10);
	}

	// Decelerate from maximum speed to zero
	for (i=255; i!=0; i--) 
	{
		m_4.setSpeed(i);  
		delay(10);
	}
}*/

float PID_calc(){
  for(int i=0; i<4; i++){
    error += error + irs[i] * i;
  };
  P = (error-desire) * kp;
  I += (I+error) * ki;
  D = -(prev_err - error) * kd;
  speed_regulator = P + I + D;
  prev_err = error;
  return speed_regulator;
}

void movement_regulator(int PID){
  m_1_fl.run(FORWARD);
  m_1_fl.setSpeed(regular_speed - PID);
  m_4_br.run(FORWARD);
  m_4_br.setSpeed(regular_speed + PID);
  m_3_bl.run(FORWARD);
  m_3_bl.setSpeed(regular_speed - PID);
  m_2_fr.run(FORWARD);
  m_2_fr.setSpeed(regular_speed + PID);
} 

void line_scanner(){
  if(digitalRead(s_ir1_l)==1 && digitalRead(s_ir2_cl)==1 && digitalRead(s_ir3_c)==0 && digitalRead(s_ir4_cr)==0 && digitalRead(s_ir5_r)==1)
      if (algo_left) {
        endVerify();
        if (digitalRead(s_ir3_c)==0) {
          moveStraight();
        } else if(digitalRead(s_ir5_r)==0){
          turnRight();
      } else {
        turnRight();
      }
      }
  else if(digitalRead(s_ir1_l)==1 && digitalRead(s_ir2_cl)==0 && digitalRead(s_ir3_c)==0 && digitalRead(s_ir4_cr)==1 && digitalRead(s_ir5_r)==1)
     { if(algo_left){ turnLeft();
      } else {
        endVerify();
        if (digitalRead(s_ir3_c)==0) {
          moveStraight();
        } else if(digitalRead(s_ir5_r)==0){
          turnLeft();
      }
     }
     }
  else if(digitalRead(s_ir5_r)==1 && digitalRead(s_ir4_cr)==0 && digitalRead(s_ir2_cl)==0 && digitalRead(s_ir1_l)==0)
     { endVerify();
       if (digitalRead(s_ir5_r)==1 && !digitalRead(s_ir4_cr)==0 && digitalRead(s_ir2_cl)==1 && digitalRead(s_ir1_l)==1){
       turnRight();
       }
       else{
       moveStraight();
       }
     }

  else if(digitalRead(s_ir4_cr)==1 && digitalRead(s_ir2_cl)==1 && digitalRead(s_ir3_c)==0)
     { moveStraight();
     }
  else if(digitalRead(s_ir4_cr)==1 && digitalRead(s_ir2_cl)==1 && digitalRead(s_ir3_c)==1)
     { moveBack_FULL_ROTAION();
     }
  else if(digitalRead(s_ir4_cr)==0 && digitalRead(s_ir3_c)==0 && digitalRead(s_ir2_cl)==0){   endVerify();
       if (digitalRead(s_ir5_r)==0 && digitalRead(s_ir3_c)==0 && digitalRead(s_ir4_cr)==0 && digitalRead(s_ir1_l)==0){
        stop();
        calOpt_paths();
       }
       else if(digitalRead(s_ir5_r)==0 && digitalRead(s_ir3_c)==0 && digitalRead(s_ir1_l)==0){
         if (algo_left){
          turnLeft();
         } else {
           turnRight();
         }
       
       }
  } else {
    endVerify();
  }
}


void endVerify(){
  moveStraight();
  delay(time_added);
  stop();
}

void turnRight(){
  //general mouvement
  // m_3_bl.run(FORWARD);
  // m_3_bl.setSpeed(regular_speed);
  // m_2_fr.run(BACKWARD);
  // m_2_fr.setSpeed(regular_speed);
  // m_1_fl.run(RELEASE);
  // m_4_br.run(RELEASE);

  // delay(time_added_rotaion);
  // stop();
  // path[m]="R";
  // m++;
  digitalWrite(m1_dir1,  LOW);
  digitalWrite(m1_dir2,  LOW);

  digitalWrite(m2_dir1, HIGH);
  digitalWrite(m2_dir2,  LOW);
}

void moveBack_FULL_ROTAION(){
  //rotation
  m_1_fl.run(BACKWARD);
  m_1_fl.setSpeed(regular_speed);
  m_4_br.run(FORWARD);
  m_4_br.setSpeed(regular_speed);
  m_2_fr.run(RELEASE);
  m_3_bl.run(RELEASE);
  delay(2*time_added_rotaion);
  stop();
  path[m]="B";
  m++;
}

void moveStraight(){
  // m_1_fl.run(FORWARD);
  // m_1_fl.setSpeed(regular_speed);
  // m_4_br.run(FORWARD);
  // m_4_br.setSpeed(regular_speed);
  // m_3_bl.run(FORWARD);
  // m_3_bl.setSpeed(regular_speed);
  // m_2_fr.run(FORWARD);
  // m_2_fr.setSpeed(regular_speed);
  // path[m]="S";
  // m++;
  digitalWrite(m1_dir1,  HIGH);
  digitalWrite(m1_dir2,  LOW);

  digitalWrite(m2_dir1, HIGH);
  digitalWrite(m2_dir2,  LOW);
}

void turnLeft(){
  // m_1_fl.run(BACKWARD);
  // m_1_fl.setSpeed(regular_speed);
  // m_4_br.run(FORWARD);
  // m_4_br.setSpeed(regular_speed);
  // m_2_fr.run(RELEASE);
  // m_3_bl.run(RELEASE);
  // delay(time_added_rotaion);
  // stop();
  // path[m]="L";
  // m++;
  digitalWrite(m1_dir1,  HIGH);
  digitalWrite(m1_dir2,  HIGH);

  digitalWrite(m2_dir2, LOW);
  digitalWrite(m2_dir1,  LOW);
}

void stop(){
  // m_2_fr.run(RELEASE);
  // m_3_bl.run(RELEASE);
  // m_1_fl.run(RELEASE);
  // m_4_br.run(RELEASE);
  digitalWrite(m1_dir1,  LOW);
  digitalWrite(m1_dir2,  LOW);

  digitalWrite(m2_dir2, LOW);
  digitalWrite(m2_dir1,  LOW);
}


// void calOpt_paths(){
//   int i = 0;
//   int j = 0;
//   while(i<=(strlen(path))){
//     if (i <= (strlen(path) - 2)){
//       if((path[i]=="L" && path[i+1]=="B" && path[i+2]=="R") || (path[i]=="R" && path[i+1]=="B" && path[i+2]=="L") || (path[i]=="S" && path[i+1]=="B" && path[i+2]=="S")){
//         opt_path[j] = "B";
//         j++;
//         i += 3;
//       } else if ((path[i]=="S" && path[i+1]=="B" && path[i+2]=="L") || (path[i]=="L" && path[i+1]=="B" && path[i+2]=="S")){
//         opt_path[j] = "R";
//         j++;
//         i += 3;
//     }
//       else if (path[i]=="L" && path[i+1]=="B" && path[i+2]=="L"){
//         opt_path[j] = "S";
//         j++;
//         i += 3;
//       };
//       if(i == strlen(path)){
//         memcpy(path, opt_path, sizeof(opt_path));
//         calOpt_paths();
//       };
//     } 
//     else {
//       opt_path[j] = path[i];
//       if(i == strlen(path)){
//         if (opt_path != path){
//           memcpy(path, opt_path, sizeof(opt_path));
//           calOpt_paths();
//         } else {
//           opt_paths[runs] = opt_path;
//           runs += 1;
//           break;
//         }
//       };
//     }
//   }
// }

// void finalOptPath(){
//   for(int i=0; i<runs; i++){
//     if(strlen(opt_paths[i]) < opt_path_length) {
//       opt_path_length = i;
//       memcpy(opt_path, opt_paths[i], sizeof(opt_paths[i]));
//     } else if(strlen(opt_paths[i]) == opt_path_length){
//       1;
//     }
//   }
// }



// //edison science corner

// #include <QTRSensors.h>
// QTRSensors qtr;
// const uint8_t SensorCount = 8;
// uint16_t sensorValues[SensorCount];
// float Kp = .2; 
// float Ki = 0;
// float Kd =.1;
// int P;
// int I;
// int D;
// int lastError = 0;
// boolean onoff = false;

// const uint8_t maxspeeda = 250;
// const uint8_t maxspeedb = 250;
// const uint8_t basespeeda = 100;
// const uint8_t basespeedb = 100;


// int mode = 8;
// int aphase = 9;
// int aenbl = 6;
// int bphase = 5;
// int benbl = 3;


// int buttoncalibrate = 17;//pin A3
// int buttonstart = 2;

// void setup() {
//   Serial.begin(9600);
//   qtr.setTypeRC();
 
//   qtr.setSensorPins((const uint8_t[]){10, 11, 12, 14, 15, 16, 18, 19}, SensorCount);
//   qtr.setEmitterPin(7);//LEDON PIN
//    pinMode(mode, OUTPUT);
//   pinMode(aphase, OUTPUT);
//   pinMode(aenbl, OUTPUT);
//   pinMode(bphase, OUTPUT);
//   pinMode(benbl, OUTPUT);
//   digitalWrite(mode, HIGH);

//   delay(500);
//   pinMode(LED_BUILTIN, OUTPUT);

//   boolean Ok = false;
//   while (Ok == false) { 
//     if(digitalRead(buttoncalibrate) == HIGH) {
//       calibration(); 
//       Ok = true;
//     }
//   }
//   forward_brake(0, 0);
// }

// void calibration() {
//   digitalWrite(4, HIGH);
//   for (uint16_t i = 0; i < 400; i++)
//   {
//     qtr.calibrate();
//   }
//   digitalWrite(4, LOW);
// }

// void loop() {
//   if(digitalRead(buttonstart) == HIGH) {
//     onoff =! onoff;
//     if(onoff = true) {
//       delay(1000);
//     }
//     else {
//       delay(50);
//     }
//   }
//   if (onoff == true) {
//     PID_control();
//   }
//   else {
//     forward_brake(0,0);
//   }
// }
// void forward_brake(int posa, int posb) {
//   digitalWrite(aphase, LOW);
//   digitalWrite(bphase, HIGH);
//   analogWrite(aenbl, posa);
//   analogWrite(benbl, posb);
// }
// void PID_control() {
//   uint16_t position = qtr.readLineBlack(sensorValues);
//   int error = 3500 - position;

//   P = error;
//   I = I + error;
//   D = error - lastError;
//   lastError = error;
//   int motorspeed = P*Kp + I*Ki + D*Kd;
  
//   int motorspeeda = basespeeda + motorspeed;
//   int motorspeedb = basespeedb - motorspeed;
  
//   if (motorspeeda > maxspeeda) {
//     motorspeeda = maxspeeda;
//   }
//   if (motorspeedb > maxspeedb) {
//     motorspeedb = maxspeedb;
//   }
//   if (motorspeeda < 0) {
//     motorspeeda = 0;
//   }
//   if (motorspeedb < 0) {
//     motorspeedb = 0;
//   } 
//   forward_brake(motorspeeda, motorspeedb);
// }


/*
 Sample Line Following Code for the Robojunkies LF-2 robot
*/

// #include <SparkFun_TB6612.h>

// #define AIN1 4
// #define BIN1 6
// #define AIN2 3
// #define BIN2 7
// #define PWMA 9
// #define PWMB 10
// #define STBY 5

// // these constants are used to allow you to make your motor configuration
// // line up with function names like forward.  Value can be 1 or -1
// const int offsetA = 1;
// const int offsetB = 1;

// // Initializing motors.  The library will allow you to initialize as many
// // motors as you have memory for.  If you are using functions like forward
// // that take 2 motors as arguements you can either write new functions or
// // call the function more than once.

// Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
// Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);


// int P, D, I, previousError, PIDvalue, error;
// int lsp, rsp;
// int lfspeed = 200;

// float Kp = 0;
// float Kd = 0;
// float Ki = 0 ;


// int minValues[6], maxValues[6], threshold[6];

// void setup()
// {
//   Serial.begin(9600);
//   pinMode(11, INPUT_PULLUP);
//   pinMode(12, INPUT_PULLUP);
// }


// void loop()
// {
//   while (digitalRead(11)) {}
//   delay(1000);
//   calibrate();
//   while (digitalRead(12)) {}
//   delay(1000);

//   while (1)
//   {
//     if (analogRead(1) > threshold[1] && analogRead(5) < threshold[5] )
//     {
//       lsp = 0; rsp = lfspeed;
//       motor1.drive(0);
//       motor2.drive(lfspeed);
//     }

//     else if (analogRead(5) > threshold[5] && analogRead(1) < threshold[1])
//     { lsp = lfspeed; rsp = 0;
//       motor1.drive(lfspeed);
//       motor2.drive(0);
//     }
//     else if (analogRead(3) > threshold[3])
//     {
//       Kp = 0.0006 * (1000 - analogRead(3));
//       Kd = 10 * Kp;
//       //Ki = 0.0001;
//       linefollow();
//     }
//   }
// }

// void linefollow()
// {
//   int error = (analogRead(2) - analogRead(4));

//   P = error;
//   I = I + error;
//   D = error - previousError;

//   PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
//   previousError = error;

//   lsp = lfspeed - PIDvalue;
//   rsp = lfspeed + PIDvalue;

//   if (lsp > 255) {
//     lsp = 255;
//   }
//   if (lsp < 0) {
//     lsp = 0;
//   }
//   if (rsp > 255) {
//     rsp = 255;
//   }
//   if (rsp < 0) {
//     rsp = 0;
//   }
//   motor1.drive(lsp);
//   motor2.drive(rsp);

// }

// void calibrate()
// {
//   for ( int i = 1; i < 6; i++)
//   {
//     minValues[i] = analogRead(i);
//     maxValues[i] = analogRead(i);
//   }
  
//   for (int i = 0; i < 3000; i++)
//   {
//     motor1.drive(50);
//     motor2.drive(-50);

//     for ( int i = 1; i < 6; i++)
//     {
//       if (analogRead(i) < minValues[i])
//       {
//         minValues[i] = analogRead(i);
//       }
//       if (analogRead(i) > maxValues[i])
//       {
//         maxValues[i] = analogRead(i);
//       }
//     }
//   }

//   for ( int i = 1; i < 6; i++)
//   {
//     threshold[i] = (minValues[i] + maxValues[i]) / 2;
//     Serial.print(threshold[i]);
//     Serial.print("   ");
//   }
//   Serial.println();
  
//   motor1.drive(0);
//   motor2.drive(0);
// }