#ifndef MinIMU9AHRS
#define MinIMU9AHRS

void minIMU_Init()
{ 
  I2C_Init();

  Serial.println("Pololu MinIMU-9 + Arduino AHRS");

  delay(1500);
 
  Accel_Init();
  Compass_Init();
  Gyro_Init();
  
  delay(20);
  
  for(int i=0;i<32;i++)    // We take some readings...
    {
    Read_Gyro();
    Read_Accel();
    for(int y=0; y<6; y++)   // Cumulate values
      AN_OFFSET[y] += AN[y];
    delay(20);
    }
    
  for(int y=0; y<6; y++)
    AN_OFFSET[y] = AN_OFFSET[y]/32;
    
  AN_OFFSET[5]-=GRAVITY*SENSOR_SIGN[5];
    
  delay(2000);
    
  timer=millis();
  delay(20);
  counter=0;
  unsigned int counter__ = 0;
  float theta;
  while(counter__ < 200) {
    do {
      theta = getHeading();
    } while(theta==255);
    counter__++;
  }
  counter__ = 0;
  float tempHead;
  while(counter__ < 25) {
    tempHead =  getHeading();
    if(tempHead != 255.0 && counter__ < 25) {
      thetaOffset += tempHead;
      counter__++;
    }  
  }
  thetaOffset /= 25;
  IMUInitDone = true;
}

float getHeading() //Main Loop
{
  if((millis()-timer)>=20)  // Main loop runs at 50Hz
  {
    counter++;
    timer_old = timer;
    timer=millis();
    if (timer>timer_old)
      G_Dt = (timer-timer_old)/1000.0;    // Real time of loop run. We use this on the DCM algorithm (gyro integration time)
    else
      G_Dt = 0;
    
    // *** DCM algorithm
    // Data adquisition
    Read_Gyro();   // This read gyro data
    Read_Accel();     // Read I2C accelerometer
    
    if (counter > 5)  // Read compass data at 10Hz... (5 loop runs)
      {
      counter=0;
      Read_Compass();    // Read I2C magnetometer
      Compass_Heading(); // Calculate magnetic heading  
      }
    
    // Calculations...
    Matrix_update(); 
    Normalize();
    Drift_correction();
    Euler_angles();
    // ***
   if(!IMUInitDone) {
      return printdata();  
   } else {
      float theta__ = printdata() - thetaOffset;
      if(theta__>180) {
        theta__ = -(360 - theta__);
      } else if(theta__ < -180) {
        theta__ = 360 + theta__;
      }
      return (theta__);
   }
  } else {
     return 255;
    }
}

#endif
