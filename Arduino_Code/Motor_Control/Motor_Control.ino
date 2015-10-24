// 22/10/15 added a while loop to read safety IR sensors
// 22/10/15 added buffer clearing and waiting which is necessary after the IR beams are broken.  Not clearing means the sensors and position are out of step.  

void setup() {
  Serial.begin(9600);
}

// declare global variables
int inPin_Safe = 2;     // pin 2 is safety input
int val_inPin_Safe = 0;     // 0 is not safe (sensors read obstacles), 1 is safe (no obstacles)
int right_speed = 0;
int left_speed = 0;
int sensor;
int setpoint = 128; // set the target sensor value
int error;
int max_speed = 100;
// these must be in global scope so that they are retained between function calls
int proportional;
int integral;
int derivative;
int last_proportional;

void flushReceive(){ //function to flush the serial buffer
while(Serial.available())Serial.read();
}

int read_IR_sensors() {
  byte byteRead;  // define variable

  flushReceive(); //flush the incoming buffer to eliminate timing issues casued by IR sensor pausing

  while(Serial.available()==0) { // Wait for data after clearing the buffer
  }

  if (Serial.available()) {  // read data from the buffer
    /* read the most recent byte */
    byteRead = Serial.read();
    /*ECHO the value that was read, back to the serial port. */
//    Serial.print("IR Sensor Value = ");
//    Serial.println(byteRead);
  }

  return byteRead; // return the sensor reading so that it can be used in the PID function
}

int pid_calc(int setpoint, int position)
{
  int error_value;
  float Kp = 3.0; // for SID 6.0 works
  float Ki = 0.0; //0.1; // for SID 0.0 works
  float Kd = 0.5; // for SID 0.0 works // try 0.5 to 1.0

  proportional = position - setpoint; // Replace setpoint by your set point
  integral = integral + proportional;
  derivative = proportional - last_proportional;
  last_proportional = proportional;
  error_value = (proportional * Kp + integral * Ki + derivative * Kd);
  //  Serial.print(" Proportional = ");
  //  Serial.print(proportional * Kp);
  //  Serial.print(" Integral = ");
  //  Serial.print(integral * Ki);
  //  Serial.print(" Derivative = ");
  //  Serial.print(derivative * Kd);
  return error_value;
}

void calc_turn(int max_speed, int error)
{
  //Restricting the error value between +256.
  if (error < -256)
  {
    error = -256;
  }
  if (error > 256)
  {
    error = 256;
  }
  // If error_value is less than zero calculate right turn speed values
  if (error < 0)
  {
    right_speed = max_speed + error;
    if (right_speed < 0)
    {
      right_speed = 0;
    }
    left_speed = max_speed;
  }
  // Iferror_value is greater than zero calculate left turn values
  else
  {
    right_speed = max_speed;
    left_speed = max_speed - error;
    if (left_speed < 0)
    {
      left_speed = 0;
    }

  }
  //  Serial.print(" Leftmotor = ");
  //  Serial.print(left_speed);
  //  Serial.print(" Rightmotor = ");
  //  Serial.print(right_speed);


}


void motor_drive(int left, int right)
{
  // declare motor pins (must be PWM pins)
  int RightMotorPWMPin = 3;
  int LeftMotorPWMPin = 5;

  // Drive motors according to the calculated values for a turn
  analogWrite(RightMotorPWMPin, right);
  analogWrite(LeftMotorPWMPin, left);
}

void loop() {
  pinMode(inPin_Safe, INPUT);
  val_inPin_Safe = !digitalRead(inPin_Safe);   // read the safety input pin
  // light the LED if safety sensors detect obstacle
    if (val_inPin_Safe == 0)
    {
      digitalWrite(13, HIGH);
      motor_drive(0,0); //stop the motors
    }
    else
    {
      digitalWrite(13, LOW);
      sensor = read_IR_sensors();
      error = pid_calc(setpoint, sensor); //get the error reading from the PID calculation.  Setpoint is defined inside this function
      calc_turn(max_speed, error);
      motor_drive(left_speed, right_speed);
    }
      delay(50);
}

