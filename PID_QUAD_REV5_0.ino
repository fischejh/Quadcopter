#include <Wire.h> // I2C 
// PID control
const float Kp = 7.5;
const float Ti = 0.1;//0.10
const float Td = 120; // 120. stable

const float Kp_yaw = 1.5;
const float Ti_yaw = 0;
const float Td_yaw = 0;

const int PID_MAX_ROLL = 400;
const int PID_MAX_PITCH = 400;
const int PID_MAX_YAW = 400;

bool safetySwitch = false;
float PID_I_err_roll;
float PID_I_err_pitch;
float PID_I_err_yaw;

float PID_D_err_roll;
float PID_D_err_pitch;
float PID_D_err_yaw;

float PID_output_roll = 0;
float PID_output_pitch = 0;
float PID_output_yaw = 0;
 
float PID_setpoint_roll;
float PID_setpoint_pitch;
float PID_setpoint_yaw;

float roll_error;
float pitch_error;
float yaw_error;

float PID_P_output_roll;
float PID_P_output_pitch;
float PID_P_output_yaw;

float PID_I_output_roll;
float PID_I_output_pitch;
float PID_I_output_yaw;
float PID_D_output_roll;
float PID_D_output_pitch;
float PID_D_output_yaw;
float last_error_roll = 0;
float last_error_pitch = 0;
float last_error_yaw = 0;

// config receriver
float current;
int signal_chanel1;
int signal_chanel2;
int signal_chanel3;
int signal_chanel4;
float duration_chanel1 = 0; // roll
float duration_chanel2 = 0; // throttle
float duration_chanel3 = 0; // pitch
float duration_chanel4 = 0; // jaw
float timer_chanel1;
float timer_chanel2;
float timer_chanel3;
float timer_chanel4;

//config mpu
const int MPU_addr = 0x68; //hexa number 
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
float averageGyroX;
float averageGyroY;
float averageGyroZ;
float pitchGyro;
float rollGyro;
float yawGyro;
boolean gyro_start = false;
float acc_total;
float acc_roll;
float acc_pitch;
float acc_yaw;
float cal_acc_total;
float cal_acc_pitch;
float cal_acc_roll;
float cal_acc_yaw;

//config esc
float esc_pulse_timer;
float esc_current_timer;
int pin_esc1 = 8; //bottom right
int pin_esc2 = 9;  //top right 
int pin_esc3 = 10; //top left
int pin_esc4 = 11;  //bottom left
int esc1_speed;
int esc2_speed;
int esc3_speed;
int esc4_speed;
/**
float ave_mid_roll = 1500;
float ave_low_roll = 1000;
float ave_high_roll = 2000;
float ave_low_pitch = 1000;
float ave_mid_pitch = 1500;
float ave_high_pitch = 2000;
float ave_low_yaw = 1000;
float ave_mid_yaw = 1500;
float ave_high_yaw = 2000;
float average_low = 1000;
float average_high = 2000;
float average_center = 1500;
*/
//end config

void setup() {
  // receiver setup=========================================================
  PCICR |= (1 << PCIE1);
  PCMSK1 |= (1 << PCINT8); // A0
  PCMSK1 |= (1 << PCINT9); // A1
  PCMSK1 |= (1 << PCINT10); // A2
  PCMSK1 |= (1 << PCINT11); // A3

  
  interruptPin_Initial();
  
  //=======================================================================
  Serial.begin(115200);
  Serial.println("Starting MPU setup...");
  //=======================================================================
  imuSetup(); 
  Serial.println("Finish");
  //=======================================================================
  Serial.println("Starting receiver calibration");
  //recCalibration();
  Serial.println("Finish");
  //=======================================================================
  Serial.println("Starting ESC setup: ");
  escSetup();
  Serial.println("Finish");
  //=======================================================================
  delay(100);
}
/**
 * Update:
 * Re-arrange ESC output pulse width into the code
 * New Loop tiem: 3200 microseconds(old: 4500)
 * Update speed increase: 40.6%
 */
void loop() {
  float innerClock = micros();
  
  writeMPU(); // approximate delay time  500 microseconds
  // starting level gyro calibration
GyX = GyX - averageGyroX;
GyY = GyY - averageGyroY;
GyZ = GyZ - averageGyroZ;

// convert raw value to pitch,roll, jaw angle 
    // 65.5 is 1dps, 303.03 hz refresh rate
pitchGyro += GyX*0.0000687;
rollGyro += GyY*0.0000687;
yawGyro -= GyZ*0.0000687;

if(yawGyro > 180){yawGyro = yawGyro - 360;}
if(yawGyro < -180){yawGyro = yawGyro + 360;}

pitchGyro += rollGyro * sin(GyZ * 0.0000012);
rollGyro -= pitchGyro * sin(GyZ * 0.0000012);

acc_total = sqrt(pow(AcX,2) + pow(AcY,2) + pow(AcZ,2));
acc_pitch = asin((float)AcY/acc_total) * 57.296;

esc_pulse_timer = micros(); // set the timer for esc output
PORTB = B00001111; // set pin 8-11 HIGH

acc_roll = asin((float)AcX/acc_total)* -57.296;
acc_pitch -= cal_acc_pitch;
acc_roll -= cal_acc_roll;

// synchronize gyro and acc, due to acc's vibration, in testing, using 0.5% of acc's reading causes drifting, thus a value lower than 0.005 is suggested
if(gyro_start)
{
    pitchGyro = (pitchGyro * 0.999) + (acc_pitch * 0.001);     //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
    rollGyro = (rollGyro * 0.999) + (acc_roll * 0.001);
  }
else
{
  pitchGyro = acc_pitch;
  rollGyro = acc_roll;

  gyro_start = true;
  }

//Serial.println(micros() - innerClock);
  
  //Serial.println(micros() - innerClock);
  //==========================PID Control================================================================
  //=============================delay time: 400 microseconds============================================
  
  PID_setpoint_roll = 0; 
  PID_setpoint_pitch = 0;
  PID_setpoint_yaw = 0;

  //set the PID_setpoint to the target angle

  if(duration_chanel1 >= 1486 && duration_chanel1 <= 1516) { duration_chanel1 = 1500;}
  PID_setpoint_roll = map(duration_chanel1,1000,2000,-30.0,30.0); // map the stick position to angle, max 30 degrees
  if(duration_chanel3 >= 1486 && duration_chanel3 <= 1516) { duration_chanel3 = 1500;}
  PID_setpoint_pitch = (-1) * map(duration_chanel3,1000,2000,-30,30);
  if(duration_chanel4 >= 1486 && duration_chanel4 <= 1516) { duration_chanel4 = 1500;}
  PID_setpoint_yaw = map(duration_chanel4,1000,2000,-180,180);
  
  
   //===============Desired rotation speed generated, Below calculate the PID output =====================
    roll_error = PID_setpoint_roll - rollGyro;
    pitch_error = PID_setpoint_pitch - pitchGyro;

    // if joystick is to the right, and yaw angle overshoots the angle(180), inverse the stick angle 
    if(PID_setpoint_yaw > 150 && yawGyro < 0){yaw_error = PID_setpoint_yaw - (360+yawGyro);}
    else if(PID_setpoint_yaw < -150 && yawGyro > 0){yaw_error = PID_setpoint_yaw - (yawGyro - 360);}
    else
    {
      yaw_error = PID_setpoint_yaw - yawGyro;
    }
    if(duration_chanel2 != 0)
    {
    // =======P controller======
      PID_P_output_roll = roll_error * Kp;
      PID_P_output_pitch = pitch_error * Kp;
      PID_P_output_yaw = yaw_error * Kp_yaw;
    
    // =======I controller======

      if(duration_chanel2 < 1300)
      {
        PID_I_err_roll = 0;
        PID_I_err_pitch = 0;
        PID_I_err_yaw = 0;
      }
      else
      {
        PID_I_err_roll += roll_error;
        PID_I_err_pitch += pitch_error;
        PID_I_err_yaw += yaw_error;
      }
      
      PID_I_output_roll = PID_I_err_roll * Ti;
      PID_I_output_pitch = PID_I_err_pitch * Ti;
      PID_I_output_yaw = PID_I_err_yaw * Ti_yaw;
    
    // =======D controller======
      PID_D_err_roll = roll_error - last_error_roll;
      PID_D_err_pitch = pitch_error - last_error_pitch;
      PID_D_err_yaw = yaw_error - last_error_yaw;
    
      PID_D_output_roll = PID_D_err_roll*Td;
      PID_D_output_pitch = PID_D_err_pitch*Td;
      PID_D_output_yaw = PID_D_err_yaw*Td_yaw;
    
      last_error_roll = roll_error;
      last_error_pitch = pitch_error;
      last_error_yaw = yaw_error;
    //==========adding all controller output together=====================
      PID_output_roll = PID_P_output_roll + PID_I_output_roll + PID_D_output_roll;
      PID_output_pitch = PID_P_output_pitch + PID_I_output_pitch + PID_D_output_pitch;
      PID_output_yaw = PID_P_output_yaw + PID_I_output_yaw + PID_D_output_yaw;
    }
    
    if(PID_output_roll > PID_MAX_ROLL){
        PID_output_roll = PID_MAX_ROLL;
        }
    else if((-1) * PID_output_roll > PID_MAX_ROLL){
      PID_output_roll = -1 * PID_MAX_ROLL;
      }
    if(PID_output_pitch > PID_MAX_PITCH){
        PID_output_pitch = PID_MAX_PITCH;
        }
    else if((-1) * PID_output_pitch > PID_MAX_PITCH){
      PID_output_pitch = -1 * PID_MAX_PITCH;
      }
    
   // ============NOTATION: P affects the speed of correction, I controlls the steady state error and D affects the smoothness of the quad movement===========
   // =====================Apply to motor throttle, yaw degree not applied================

  // stop the quad when throttle is at the lowest position
 esc1_speed = duration_chanel2 - PID_output_roll - PID_output_pitch - PID_output_yaw;
 esc2_speed = duration_chanel2 - PID_output_roll + PID_output_pitch + PID_output_yaw;
 esc3_speed = duration_chanel2 + PID_output_roll + PID_output_pitch - PID_output_yaw;
 esc4_speed = duration_chanel2 + PID_output_roll - PID_output_pitch + PID_output_yaw;


 // Only for testing
  if(rollGyro > 60 || rollGyro < -60 || pitchGyro > 60 || pitchGyro <-60)
  {
    safetySwitch = true;
    }

if(duration_chanel2 >= 1800) {duration_chanel2 = 1800;}
//diasable motors whever angles > set point, throttle < 1030
if(esc1_speed < 1000 || duration_chanel2 < 1030 || safetySwitch){esc1_speed = 1000;}
if(esc2_speed < 1000 || duration_chanel2 < 1030 || safetySwitch){esc2_speed = 1000;}
if(esc3_speed < 1000 || duration_chanel2 < 1030 || safetySwitch){esc3_speed = 1000;}
if(esc4_speed < 1000 || duration_chanel2 < 1030 || safetySwitch){esc4_speed = 1000;}
//Serial.println(micros() - innerClock);
//==============================gemerate esc output==============================


  //esc_pulse_timer = micros(); // set the timer for esc output
  //PORTB = B00001111; // set pin 8-11 HIGH
  while(PORTB > 0) // wait until all esc are set LOW
  {
    esc_current_timer = micros();
    if(esc_current_timer >= esc_pulse_timer + esc1_speed) {PORTB &= B11111110;}
    if(esc_current_timer >= esc_pulse_timer + esc2_speed) {PORTB &= B11111101;}
    if(esc_current_timer >= esc_pulse_timer + esc3_speed) {PORTB &= B11111011;}
    if(esc_current_timer >= esc_pulse_timer + esc4_speed) {PORTB &= B11110111;}
  }

//Serial.println(String(esc1_speed) + " " + String(esc2_speed) + " " + String(esc3_speed) + " " + String(esc4_speed));
Serial.println(String(pitchGyro) + "  " + String(rollGyro));

//Serial.println(micros() - innerClock);
//if(micros()-innerClock > 3300){while(true);}
//float sum = esc1_speed + esc2_speed + esc3_speed + esc4_speed;
Serial.println(String(PID_output_roll) + " " + String(duration_chanel2));
  while(micros() - innerClock < 4500);
}

void escSetup()
{ 
  // start esc setup

  DDRB = B00001111; // set pin 8-11 as OUTPUT 

  PORTB = B00001111; // set pin 8-11 HIGH
  delayMicroseconds(1000);
  PORTB = B00000000;
  delayMicroseconds(3000);
  }

void writeMPU()
{
Wire.beginTransmission(MPU_addr);
Wire.write(0x3B);
Wire.endTransmission(false);
Wire.requestFrom(MPU_addr,14,true);
// recieving 14 bytes data
AcX = Wire.read() << 8| Wire.read();
AcY = Wire.read() << 8| Wire.read();
AcZ = Wire.read() << 8| Wire.read();
Tmp = Wire.read() << 8| Wire.read();
GyX = Wire.read() << 8| Wire.read();
GyY = Wire.read() << 8| Wire.read();
GyZ = Wire.read() << 8| Wire.read();
}
  Serial.print(String(Gyx));
  Serial.println(String(AcX));
void imuSetup()
{
  Wire.begin();

  TWBR = 12;
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B); // wake up MPU
  Wire.write(0);
  Wire.endTransmission(true);

  // set the acc to +/- 8g

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();

  // set the gyro to 500dps

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1B);
  Wire.write(0x08);

  Wire.endTransmission(); 
  calibration();
  }

  void calibration()
{
  //lcd.setCursor(0,0);
  Serial.println("Calibrating...");
  float gyroXSum = 0;
  float gyroYSum = 0;
  float gyroZSum = 0;
  float accXSum = 0;
  float accYSum = 0;
  float accZSum = 0;
  for(int i = 0; i < 2000; i++)
  {
    
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr,14,true);
    AcX = Wire.read() << 8| Wire.read();
    AcY = Wire.read() << 8| Wire.read();
    AcZ = Wire.read() << 8| Wire.read();
    Tmp = Wire.read() << 8| Wire.read();
    GyX = Wire.read() << 8| Wire.read();
    GyY = Wire.read() << 8| Wire.read();
    GyZ = Wire.read() << 8| Wire.read();
    
    gyroXSum += GyX;gyroYSum += GyY;gyroZSum += GyZ;
    
    cal_acc_total = sqrt(pow(AcX,2) + pow(AcY,2) + pow(AcZ,2)); // find magnitude of acc values
    cal_acc_pitch += asin((float)AcY/cal_acc_total)* 57.296; 
    cal_acc_roll += asin((float)AcX/cal_acc_total)* -57.296;
    delay(2);
  }
  cal_acc_pitch /= 2000; cal_acc_roll /= 2000;
  averageGyroX = gyroXSum/2000; averageGyroY = gyroYSum/2000; averageGyroZ = gyroZSum/2000;
  
  Serial.println("MPU Calibration Finished");
  //lcd.setCursor(0,0);
  //lcd.print("Finish");
}
ISR(PCINT1_vect)
{
  current = micros();
  // PINB & B00000001 same as digitalRead() with less loop time. here signal changes from 0 to 1
  // chanel 1
  if(PINC & B00000001) { if(signal_chanel1 == 0){signal_chanel1 = 1; timer_chanel1 = current;}}
    else if(signal_chanel1 == 1) {signal_chanel1 = 0; duration_chanel1 = micros() - timer_chanel1;}
  // chanel 2
  if(PINC & B00000010) { if(signal_chanel2 == 0){signal_chanel2 = 1; timer_chanel2 = current;}}
    else if(signal_chanel2 == 1) {signal_chanel2 = 0; duration_chanel2 = micros() - timer_chanel2;}
  // chanel 3
  if(PINC & B00000100) { if(signal_chanel3 == 0){signal_chanel3 = 1; timer_chanel3 = current;}}
    else if(signal_chanel3 == 1) {signal_chanel3 = 0; duration_chanel3 = micros() - timer_chanel3;}
  // chanel 4
  if(PINC & B00001000) { if(signal_chanel4 == 0){signal_chanel4 = 1; timer_chanel4 = current;}}
    else if(signal_chanel4 == 1) { signal_chanel4 = 0; duration_chanel4 = micros() - timer_chanel4;}
}

void interruptPin_Initial()
{ 
  if(analogRead(A0) == 0) {signal_chanel1 = 0;}
    else {signal_chanel1 = 1;}
  if(analogRead(A1) == 0) {signal_chanel2 = 0;}
    else {signal_chanel2 = 1;}
  if(analogRead(A2) == 0) {signal_chanel3 = 0;}
    else {signal_chanel3 = 1;}
  if(analogRead(A3) == 0) {signal_chanel4 = 0;}
    else {signal_chanel4 = 1;}
  }
  
