/*
 * This code is for controlling a DC motor with 95 rpm speed using Arduino Mega and PID controller
 * and an incoder with 612 pulse for a full rotation
 * 
 * Note:
 * I found that the maximum speed of the motor is 75 rpm although it is supposed to be 95 rpm!!!
*/


#define channelA 2 //pin 1 of motor incoder
#define channelB 24 //pin 2 of motor incoder

#define PWM 3 //PWM pin of the motor
#define INR0 30 //IN1 pin of the motor driver L298
#define INR1 32 //IN2 pin of the motor driver L298

#define pot A0 //Analog pin for tunning the gains

//PID gains
volatile double Kp_0 = 2;
volatile double Ki_0 = 6; 
volatile double Kd_0 = 0;

//variables for the PID controller
volatile double sampleTime = 0.1; //sample time for PID controller = 100 ms
volatile double kpPart_0, kiPart_0, kdPart_0; 
volatile double curError_0, preError_0, errorSum_0;
volatile int16_t PIDOUT, motorPower = 127;

//variables to calculate the motor speed and angle of rotation
volatile double targetSpeed = 50; //set the target speed
volatile bool bState; //to check the direction of the motor rotation
volatile double angleDeg;
const double stepPerPul = 0.0103; // 2 * PI / number of pulses per revolution = 2 * PI / 612 = 0.01026664265
volatile double  curSpeed = 0;
volatile double curPos = 0;
volatile unsigned long pulsesCnt = 0;
volatile bool motorDir = 1;
const uint16_t pulsesPerRot = 612; //number of incoder pulses for a full rotation

//variables for reading the pot value
int16_t potRead, potShift = 0;
double potAns;

//Initialize timer 1
void INIT_TIMER(void)
{
  cli();          // disable global interrupts
  TCCR1A = 0;     // set entire TCCR1A register to 0
  TCCR1B = 0;     // same for TCCR1B    
  // set compare match register to set sample time 100 ms
  OCR1A = 24999;    
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS11 and CS10 bits for prescaling by 64
  TCCR1B |= (1 << CS11) | (1 << CS10); 
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei();          // enable global interrupts
}


void setup() {
  //Initialize external interrupt for the incoder pin
  attachInterrupt(digitalPinToInterrupt(channelA), POSITION_CONTROL, RISING);  

  //Initialize serial moniter and output pins
  Serial.begin(38400);
  pinMode(PWM, OUTPUT);
  pinMode(INR0, OUTPUT);
  pinMode(INR1, OUTPUT);
  pinMode(22, OUTPUT);
  digitalWrite(22, LOW);

  //Call the timer initialization code
  INIT_TIMER();
}

void loop() 
{
  //calculate the angle of rotation in degrees
  angleDeg = (curPos * 180) / PI;

  
  
  //Print the current speed and target speed in the arduino plotter to see the 
    PLOTTER_SHOW();

  //You can comment the PLOTTER_SHOW function to see the SERIAL_MONITER_SHOW function output
  
  //Print the current error and PID controller parts, output, and the current motor speed
//  SERIAL_MONITER_SHOW();

}

void PLOTTER_SHOW(void)
{
  Serial.print(curSpeed);
  Serial.print(" ");
  Serial.print(targetSpeed);
  Serial.print(" ");
  Serial.print(0);
  Serial.print(" ");
  Serial.println(100);
}

void SERIAL_MONITER_SHOW(void)
{
  Serial.print(curError_0);
  Serial.print('\t');
  Serial.print(kpPart_0);
  Serial.print('\t');
  Serial.print(kiPart_0);
  Serial.print('\t');
  Serial.print(kdPart_0);
  Serial.print('\t');  
  Serial.print(PIDOUT);
  Serial.print('\t');  
  Serial.println(curSpeed);
}

double READ_POT(void)
{
  potRead = analogRead(pot);
  potRead = map(potRead, 0, 1023, 0, 75);
  potAns = (double)potRead / 1;
  potAns = potAns + potShift;
  return potAns;
}

void POSITION_CONTROL()
{
  //read the other incoder pin to know the direction of rotation
  //bState = HIGH => clockwise rotation
  //bState = LOW => counter-clockwise rotation
  bState = digitalRead(channelB);
  pulsesCnt++;
  if(bState == HIGH)
  {
    curPos = curPos + stepPerPul;
  }
  else
  {
    curPos = curPos - stepPerPul;
  }

  //set current angle to zero when finishing a full rotation
//  if(curPos > 2 * PI)
//    curPos = 0;
//  else if(curPos < (-2 * PI))
//    curPos = 0;
}


ISR(TIMER1_COMPA_vect)
{
//  Calculate the speed of the motor  
    curSpeed = (double)pulsesCnt * (1 / sampleTime); //number of pulses per second
    curSpeed = curSpeed / pulsesPerRot; // rps = (counted pulses until now / total number of pulses per revolution)
    curSpeed = curSpeed * 60;
    pulsesCnt = 0;

//  Calculate the error in speed  
    curError_0 = targetSpeed - curSpeed;

//  Calculate the error sum
    errorSum_0 = errorSum_0 + curError_0;  

//  Calculate KP_part, KI_part, KD_part
    kpPart_0 = Kp_0*curError_0;
    kiPart_0 = Ki_0*errorSum_0*sampleTime;
    kdPart_0 = (Kd_0*(curError_0 - preError_0)) / sampleTime;

//  Calculate the PID output 
    motorPower = kpPart_0 + kiPart_0 + kdPart_0;

//  Save the previous value of the error
    preError_0 = curError_0;

    PIDOUT = (int16_t) motorPower;

//  Determine the max limits and min limits of the PID controller output
    if(PIDOUT > 255)
      PIDOUT = 255;
    else if(PIDOUT < 0)
      PIDOUT = 0;

//  
    if(motorDir)
    {
      digitalWrite(INR0, HIGH);
      digitalWrite(INR1, LOW);
    }
    else
    {
      digitalWrite(INR0, LOW);
      digitalWrite(INR1, HIGH);
    }
    analogWrite(PWM,PIDOUT);
}
