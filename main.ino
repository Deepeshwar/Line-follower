#include<avr/sfr_defs.h>

int SENSOR_SIGN[9] = {1,1,1,-1,-1,-1,1,1,1}; //Correct directions x,y,z - gyro, accelerometer, magnetometer
//int SENSOR_SIGN[9] = {1,-1,-1,-1,1,1,1,-1,-1}; //Correct directions x,y,z - gyro, accelerometer, magnetometer

#include <Wire.h>
#include <math.h>
#define GRAVITY 256  //this equivalent to 1G in the raw data coming from the accelerometer 

#define ToRad(x) ((x)*0.01745329252)  // *pi/180
#define ToDeg(x) ((x)*57.2957795131)  // *180/pi

#define Gyro_Gain_X 0.07 //X axis Gyro gain
#define Gyro_Gain_Y 0.07 //Y axis Gyro gain
#define Gyro_Gain_Z 0.07 //Z axis Gyro gain
#define Gyro_Scaled_X(x) ((x)*ToRad(Gyro_Gain_X)) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Y(x) ((x)*ToRad(Gyro_Gain_Y)) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Z(x) ((x)*ToRad(Gyro_Gain_Z)) //Return the scaled ADC raw data of the gyro in radians for second

#define M_X_MIN -421
#define M_Y_MIN -639
#define M_Z_MIN -238
#define M_X_MAX 424
#define M_Y_MAX 295
#define M_Z_MAX 472

#define Kp_ROLLPITCH 0.02
#define Ki_ROLLPITCH 0.00002
#define Kp_YAW 1.2
#define Ki_YAW 0.00002

/*For debugging purposes*/
//OUTPUTMODE=1 will print the corrected data, 
//OUTPUTMODE=0 will print uncorrected data of the gyros (with drift)
#define OUTPUTMODE 1

#define PRINT_EULER 1   //Will print the Euler angles Roll, Pitch and Yaw

float G_Dt=0.02;    // Integration time (DCM algorithm)  We will run the integration loop at 50Hz if possible

long timer=0;   //general purpuse timer
long timer_old;
long timer24=0; //Second timer used to print values 
int AN[6]; //array that stores the gyro and accelerometer data
int AN_OFFSET[6]={0,0,0,0,0,0}; //Array that stores the Offset of the sensors

int gyro_x;
int gyro_y;
int gyro_z;
int accel_x;
int accel_y;
int accel_z;
int magnetom_x;
int magnetom_y;
int magnetom_z;
float c_magnetom_x;
float c_magnetom_y;
float c_magnetom_z;
float MAG_Heading;

float Accel_Vector[3]= {0,0,0}; //Store the acceleration in a vector
float Gyro_Vector[3]= {0,0,0};//Store the gyros turn rate in a vector
float Omega_Vector[3]= {0,0,0}; //Corrected Gyro_Vector data
float Omega_P[3]= {0,0,0};//Omega Proportional correction
float Omega_I[3]= {0,0,0};//Omega Integrator
float Omega[3]= {0,0,0};

// Euler angles
float roll;
float pitch;
float yaw;

float errorRollPitch[3]= {0,0,0}; 
float errorYaw[3]= {0,0,0};

unsigned int counter=0;
byte gyro_sat=0;

float DCM_Matrix[3][3]= {
  {
    1,0,0  }
  ,{
    0,1,0  }
  ,{
    0,0,1  }
}; 
float Update_Matrix[3][3]={{0,1,2},{3,4,5},{6,7,8}}; //Gyros here


float Temporary_Matrix[3][3]={
  {
    0,0,0  }
  ,{
    0,0,0  }
  ,{
    0,0,0  }
};


/*PID constants */
float KP_main = 15.0;
float KI_main = 0.0;
float KD_main = 0.0;

/*Sensor macros*/
#define NUMBER_OF_SENSOR  8
#define STATUS_LED 13

/*Encoder macros*/
#define CHANNELA      2
#define J_INT      3
#define TICKS_PER_ROUND   1000.0
#define CIRCUMFERENCE   23.5
#define DISTANCE_PER_TICK 0.0235
  
/*Servo macros*/
#define PWMPIN 10
#define OFFSET 105.0
#define HITECSERVO_TIME_PERIOD 40000.0
#define SERVO_ZERO_PULSE 1100.0
#define SERVO_MAX_PULSE 4600.0
const float angle2pwm_factor = (SERVO_MAX_PULSE-SERVO_ZERO_PULSE)/180;

/*Sample macros */
#define DISTANCE_SAMPLE 0.24
#define DISTANCE_FROM_CENTER 24.0
float samplingDistance = DISTANCE_SAMPLE;

/*Line-follow variables*/
const uint8_t sensorPin[8] = { 4, 5, 6, 7, 8, 9, 11, 12};
volatile uint8_t sensorData[NUMBER_OF_SENSOR] = {0,0,0,0,0,0,0,0};
volatile float e_old = 0, E = 0;
volatile float prev_error = 0;
volatile float required = 3.5; 
volatile bool jFlag = false;
volatile uint16_t jCount = 0; 

/*Theta Mapper variables*/
uint8_t mapCounter = 0;
float mapAngle[3] = {30.0,-5.0,45.0};

/*Spline interpolation variables*/
volatile float slope = 0, y_intercept = 0;
volatile float curAngle = 0;
volatile float nextAngle = 0;
volatile float curDist = 0;
volatile float lastDist = curDist;

volatile float thetaHead = 0.0, thetaOffset = 0.0;
bool thetaMapMode = false;
bool IMUInitDone = false;

void getDistance();
void jInterrupt();
void setupEcobot(void) {
  /*Sensor input setup*/
  for(int i = 0; i < 8; i++) {
    pinMode(sensorPin[i], INPUT);
    digitalWrite(i, LOW);
  }
  pinMode(STATUS_LED,OUTPUT);
  /*Encoder setup*/
  pinMode(CHANNELA,INPUT);
  digitalWrite(CHANNELA,LOW);
  attachInterrupt(digitalPinToInterrupt(CHANNELA),getDistance,RISING);

  /*Junction interrupt*/
  
  pinMode(J_INT,INPUT);
  digitalWrite(J_INT,LOW);
  attachInterrupt(digitalPinToInterrupt(J_INT),jInterrupt,RISING);
  
  /*Servo control setup*/
  DDRB  |= 1<<PB2;
  TCCR1A  = 1<<COM1B1 | 1<<WGM11;
  TCCR1B  = 1<<WGM12 | 1<<WGM13 | 1<<CS11;
  ICR1 = HITECSERVO_TIME_PERIOD;
}

float readSensor(void) {
  uint8_t count = 0;
  float error = 0;
  for(int i = 0; i < 8; i++) {
    if(digitalRead(sensorPin[i])) {
        sensorData[i] = 1;
        count++;  
      } else {
      sensorData[i] = 0;
    }
    error += i*sensorData[i];
  }
  if(count!=0) {
    error /= count;
    error = required - error;
    prev_error = error;
    return error;
  }
  else {
    if(prev_error<0) {
      return prev_error;
    }
    else {
      return -prev_error;
    }
  }
}

void getDistance(void) {
  curDist += DISTANCE_PER_TICK;
}

void setHitecServoAngle(volatile float angle)
{
  float resultant_angle = (OFFSET - angle);
  if(resultant_angle <= 0) {
    resultant_angle = 0; 
  } else if(resultant_angle>=180) {
    resultant_angle = 180;
  }
  volatile uint32_t angle_ = (resultant_angle * angle2pwm_factor) +SERVO_ZERO_PULSE;
  OCR1B = angle_;
}

void jInterrupt(void) {
  jFlag = true;
  jCount++;
}

/************************************************************************/
/* Spline interpolation functions                                        */
/************************************************************************/

void spline_interpolation(float curDist, float lastDist, float nextAngle, float curAngle) {
  if(curDist!=lastDist) {
    slope = (nextAngle - curAngle)/DISTANCE_FROM_CENTER;
    y_intercept = curAngle - (slope*curDist);
  }
  else {
    slope = nextAngle/DISTANCE_FROM_CENTER;
    y_intercept = 0;
  }
}

float curve_fit(float sample_instance) {
  if(sample_instance>=0) {
    return (slope*sample_instance)+y_intercept;
  }
  else {
    return 0;
  }
}

/*
PID control loop
*/
float PID(float e)
{
  float u;
  E += e;
  u = KP_main*e + KI_main*E + KD_main*(e-e_old);
  e_old = e;
  return u;
}

void setup() {
  digitalWrite(STATUS_LED,LOW);
  Serial.begin(38400);
  setupEcobot();
  setHitecServoAngle(0);
  curDist = 0;
  curAngle = 0;
  nextAngle = curAngle + PID(-readSensor());
  spline_interpolation(curDist,lastDist,nextAngle, curAngle);
  lastDist = curDist;
  thetaMapMode = false;
  minIMU_Init();
  digitalWrite(STATUS_LED,HIGH);
  required = 5.5;
}

void loop() {
 float tempHead;
 if(!thetaMapMode) {
  if((curDist-lastDist)<DISTANCE_SAMPLE) {
    curAngle = curve_fit(curDist);
    if(curAngle < -74) {
      curAngle = -75;
    }
    else if(curAngle > 104) {
      curAngle = 105;
    }
    setHitecServoAngle(curAngle);
  } else {
    curAngle = curve_fit(curDist);
    if(curAngle < -74) {
      curAngle = -75;
    }
    else if(curAngle > 104) {
      curAngle = 105;
    }
    nextAngle = curAngle + PID(-readSensor());
    spline_interpolation(curDist,lastDist,nextAngle,curAngle);
    lastDist = curDist;
  } 
 } else {
    if((curDist-lastDist)<DISTANCE_SAMPLE) {
      curAngle = curve_fit(curDist);
      if(curAngle < -74) {
        curAngle = -75;
      }
      else if(curAngle > 104) {
        curAngle = 105;
      }
      setHitecServoAngle(curAngle);
      } else {
      curAngle = curve_fit(curDist);
      if(curAngle < -74) {
        curAngle = -75;
      }
      else if(curAngle > 104) {
        curAngle = 105;
      }
      nextAngle = curAngle - (mapAngle[mapCounter] - thetaHead);
      spline_interpolation(curDist,lastDist,nextAngle,curAngle);
      lastDist = curDist;
    }
 }
  if(thetaMapMode == true) {
    if(curDist > 50.0 && curDist < 60.0) {
      mapCounter = 1; 
    } else if(curDist > 170.0 && curDist < 220.0) {
      mapCounter = 2;
    } else if(curDist > 220.0) {
      thetaMapMode = false;
      required = 3.5;
      curDist = 0;
      lastDist = -0.1;
      nextAngle = -curAngle;
      spline_interpolation(curDist,lastDist,nextAngle, curAngle);
    }
  }
  if(jFlag) {
    jFlag = false;
    if(jCount == 1) {
      thetaMapMode = true;
        if(thetaMapMode) {
        prev_error = 0;
        curDist = 0;
        lastDist = -0.1;
        nextAngle = curAngle - PID(-readSensor());
        spline_interpolation(curDist,lastDist,nextAngle, curAngle);
      }
    }
  }
  tempHead = getHeading();
  if(tempHead != 255.0) {
    thetaHead = tempHead;
  }
  
  /*if(jFlag) {
    jFlag = false;
    if(curDist > 230.0 && curDist < 270.0) {
      float initialDist = curDist;      
      thetaMapMode = true;
      while((curDist - initialDist) < DISTANCE_FROM_CENTER);
      setHitecServoAngle(curAngle + 90.0 - thetaHead);
      if(thetaMapMode) {
        nextAngle = mapAngle[mapCounter];
      }
    }
  }*/  
  Serial.println(curDist);
}
