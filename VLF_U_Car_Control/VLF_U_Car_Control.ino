#include <NewPing.h>

int flag =1;

  int count2 = 0;


#define AOpin  A0     // Analog output - yellow
#define SIpin  22     // Start Integration - orange
#define CLKpin 23     // Clock - red
// Vcc - brown
// GND - black

#define NPIXELS 128  // No. of pixels in array

byte Pixel[NPIXELS]; // Field for measured values <0-255>

int LineSensor_Data[NPIXELS];           // line sensor data(original)
int LineSensor_Data_Adaption[NPIXELS];  // line sensor data(modified)
int MAX_LineSensor_Data[NPIXELS];       // Max value of sensor
int MIN_LineSensor_Data[NPIXELS];       // Min value of sensor
int flag_line_adapation;          // flag to check line sensor adpation
int line_data = 0;

#define FASTADC 1
// defines for setting and clearing register bits
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

/////////////////////  Ultrasonic Sensor ///////////////////

#define MOTOR_DIR 4
#define MOTOR_PWM 5

#define F_Sonar_TRIG 28
#define F_Sonar_ECHO 29
#define R_Sonar_TRIG 30
#define R_Sonar_ECHO 31
#define L_Sonar_TRIG 32
#define L_Sonar_ECHO 33
#define MaxDistance 350

  NewPing R_sensor(R_Sonar_TRIG,R_Sonar_ECHO,MaxDistance);
  float R_Sonar_Error = 0.0;
  float R_Sonar_distance = 0.0;
  float R_Sonar_distance_old = 0.0;

  NewPing L_sensor(L_Sonar_TRIG,L_Sonar_ECHO,MaxDistance);
  float L_Sonar_Error = 0.0;
  float L_Sonar_distance = 0.0;
  float L_Sonar_distance_old = 0.0;

  NewPing F_sensor(F_Sonar_TRIG,F_Sonar_ECHO,MaxDistance);
  float F_Sonar_Error = 0.0;
  float F_Sonar_distance = 0.0;
  float F_Sonar_distance_old = 0.0;

  void read_sonar_sensor(void)   //초음파센서 측정
  {
    R_Sonar_distance = R_sensor.ping_cm()*10.0;
    L_Sonar_distance = L_sensor.ping_cm()*10.0;
    F_Sonar_distance = F_sensor.ping_cm()*10.0;
    if(R_Sonar_distance == 0){R_Sonar_distance = MaxDistance * 10.0;}
    if(L_Sonar_distance == 0){L_Sonar_distance = MaxDistance * 10.0;}
    if(F_Sonar_distance == 0){F_Sonar_distance = MaxDistance * 10.0;}
  }

  void update_sonar_old(void)    //초음파 센서의 옛날값 저장
  {
    R_Sonar_distance_old = R_Sonar_distance;
    L_Sonar_distance_old = L_Sonar_distance;
    F_Sonar_distance_old = F_Sonar_distance;
  }

  void update_sonar_error(void)   //초음파 센서의 옛날값과 현재값의 차이 저장
  {
    R_Sonar_Error = R_Sonar_distance - R_Sonar_distance_old;
    L_Sonar_Error = L_Sonar_distance - L_Sonar_distance_old;
    F_Sonar_Error = F_Sonar_distance - F_Sonar_distance_old;
  }

/////////////////////  Steering Servo Control ///////////////////
#include <Servo.h>
#define RC_SERVO_PIN 8
#define NEURAL_ANGLE 88
#define LEFT_STEER_ANLGE -30
#define RIGHT_STEER_ANLGE 50

Servo Steeringservo;

int Steering_Angle = NEURAL_ANGLE;

void setup() {
  // put your setup code here, to run once:
  int i;
 
  for (i = 0; i < NPIXELS; i++)
  {
    LineSensor_Data[i] = 0;
    LineSensor_Data_Adaption[i] = 0;
    MAX_LineSensor_Data[i] = 1023; //0;
    MIN_LineSensor_Data[i] = 0; //1023;
  }

  pinMode(SIpin, OUTPUT);
  pinMode(CLKpin, OUTPUT);

  digitalWrite(SIpin, LOW);   // IDLE state
  digitalWrite(CLKpin, LOW);  // IDLE state

#if FASTADC
  // set prescale to 16
  sbi(ADCSRA, ADPS2);
  cbi(ADCSRA, ADPS1);
  cbi(ADCSRA, ADPS0);
#endif

  flag_line_adapation = 0;

  Steeringservo.attach(RC_SERVO_PIN);
  Steeringservo.write(NEURAL_ANGLE);
  pinMode(MOTOR_DIR, OUTPUT);
  pinMode(MOTOR_PWM, OUTPUT);

  Serial.begin(115200);

}
void line_adaptation(void)
{
  int i;
  for (i = 0; i < NPIXELS; i++)
  {
    if (LineSensor_Data[i] >= MAX_LineSensor_Data[i])  MAX_LineSensor_Data[i] = LineSensor_Data[i];
    if (LineSensor_Data[i] <= MIN_LineSensor_Data[i])  MIN_LineSensor_Data[i] = LineSensor_Data[i];
  }

  /*for (i = 0; i < NPIXELS; i++)
    {
    Serial.print("[");
    Serial.print(i);
    Serial.print("]");
    Serial.print("   : ");
    Serial.print(MAX_LineSensor_Data[i]);
    Serial.print(" | ");
    Serial.print(MIN_LineSensor_Data[i]);
    Serial.println(" ");
    }*/
}
void read_line_sensor(void)
{
  int i;

  delayMicroseconds (1);  /* Integration time in microseconds */
  delay(10);              /* Integration time in miliseconds  */

  digitalWrite (CLKpin, LOW);
  digitalWrite (SIpin, HIGH);
  digitalWrite (CLKpin, HIGH);
  digitalWrite (SIpin, LOW);
  
  delayMicroseconds (1);

  for (i = 0; i < NPIXELS; i++) {
    Pixel[i] = analogRead (AOpin) / 4 ; // 8-bit is enough
    digitalWrite (CLKpin, LOW);
    delayMicroseconds (1);
    digitalWrite (CLKpin, HIGH);
  }

  for (i = 0; i < NPIXELS; i++)
  {
    LineSensor_Data_Adaption[i] = map(Pixel[i], MIN_LineSensor_Data[i], MAX_LineSensor_Data[i], 0, 256);
  }

}

#define threshold_value 60

int i;

void threshold(void)
{
  for (i = 0; i < NPIXELS; i++)
  {
    if((byte)Pixel[i] >= threshold_value)  LineSensor_Data_Adaption[i] = 255;
    else LineSensor_Data_Adaption[i] = 0;
  }
}

#define camera_pixel_offset 0
void steering_by_camera(void)
{
  int i;
  long sum= 0;
  long x_sum= 0;
  int steer_data_l =0;
  for (i = 0; i < NPIXELS; i++)
  {
    sum +=LineSensor_Data_Adaption[i];
    x_sum += LineSensor_Data_Adaption[i] * i;
  }
  steer_data_l = x_sum/sum - NPIXELS/2 + camera_pixel_offset;
  if(steer_data_l>=-60 && steer_data_l <=60){
    steering_control_l(steer_data_l*1.4);        //마음대로 부호가 반대면 음수 적용
  }
  //Serial.println(steer_data);
}


/////////////////////////   DC_Motor_Control    //////////////////////////
#define MOTOR_DIR 4
#define MOTOR_PWM 5

int Motor_Speed =0;
#define NORMAL_SPEED 100
#define SLOW_SPEED    70


void motor_control(int direction, int speed)
{
  digitalWrite(MOTOR_DIR, 1-direction);
  analogWrite(MOTOR_PWM, speed);

}
/////////////////////////   DC_Motor_Control     //////////////////////////


///////////////////////   Steering Servo Control   ///////////////////////
#include <Servo.h>
#define RC_SERVO_PIN 8
#define NEUTRAL_ANGLE 88
#define NEUTRAL_ANGLE_L 88
#define LEFT_STEER_ANGLE -37
#define RIGHT_STEER_ANGLE 37
#define LEFT_STEER_ANGLE_L -37
#define RIGHT_STEER_ANGLE_L 37

//Servo Steeringservo; 
//int Steering_Angle = NEUTRAL_ANGLE;

void steering_control(int steer_angle)
{
  //Steeringservo.write(NEUTRAL_ANGLE + steer_angle);

  if(steer_angle >= RIGHT_STEER_ANGLE) steer_angle = RIGHT_STEER_ANGLE;
  if(steer_angle <= LEFT_STEER_ANGLE) steer_angle = LEFT_STEER_ANGLE;
  Steeringservo.write(NEUTRAL_ANGLE + steer_angle);
}


void steering_control_l(int steer_angle_l)
{
  //Steeringservo.write(NEUTRAL_ANGLE + steer_angle);

  if(steer_angle_l >= RIGHT_STEER_ANGLE_L) steer_angle_l = RIGHT_STEER_ANGLE_L;
  if(steer_angle_l <= LEFT_STEER_ANGLE_L) steer_angle_l = LEFT_STEER_ANGLE_L;
  Steeringservo.write(NEUTRAL_ANGLE_L + steer_angle_l);
}
///////////////////////   Steering Servo Control   ///////////////////////


///////////////////////     Two Wall Following     ///////////////////////


void two_wall_following()                                                         //두 개의 벽을 이용하여 주행할 때(두 개의 벽을 검출했을 때)
{
  read_sonar_sensor();
  motor_control(1, 110);
  int A=1120, B=100;
  
  if(R_Sonar_distance + L_Sonar_distance <= A && R_Sonar_distance+B >= L_Sonar_distance && R_Sonar_distance-B <= L_Sonar_distance)
  {
    steering_control(0);
    delay(100);
    }
    
  else if(R_Sonar_distance > L_Sonar_distance)                                    //R_D가 L_D보다 길 때 -> 차체가 왼쪽으로 기울었을 때
  {
    steering_control(12);
    delay(100);
  }
  else if(R_Sonar_distance < L_Sonar_distance)                                    //L_D가 R_D보다 길 때 -> 차체가 오른쪽으로 기울었을 때
  {
    steering_control(-12);
    delay(100);
  }
  else                                                                            //차체가 기울어지지 않았을 때
  {
    steering_control(0);
    delay(100);
  }
 
}


///////////////////////     Two Wall Following     ///////////////////////


//////////////////////         TURN CONRNER         //////////////////////
int a= 0,b= 0;


void turn_corner()
{
  read_sonar_sensor();
  if(count2=0)   a=0, b= 0;
  else a= 50, b= 0;
  
  if(R_Sonar_distance+L_Sonar_distance <= 1120)
  {
    two_wall_following();
    delay(10);
  }

  else if(R_Sonar_distance > 800)
  {
    motor_control(1,130);
    steering_control(0);
    if(F_Sonar_distance <=800+a)
    {
      steering_control(40);
      delay(100);
      while(1)
      {
        read_sonar_sensor();
        steering_control(40);
        
        if(L_Sonar_distance <= 750-b)
        {Serial.print(count2);
          count2++;
          Serial.print(count2);
          break;
          delay(100);
          
        }
      }
    }
    
  }
      //delay(200);
}
      //delay(10);
    
//////////////////////     LINE DETECTION CHECK     //////////////////////


int line_detect_check()
{
  int Line_detect=0;
  for(i=0; i <NPIXELS; i++)
  {
    if(LineSensor_Data_Adaption[i] == 255)  Line_detect++;
  }
  return Line_detect;
}

int Line_de =0;

  
void loop()
{
  //////////////////////////////////////////////////////////////
  /////////////////////        FLAG        /////////////////////
  
  if(flag == 1)
  {
    read_sonar_sensor();
    line_adaptation();
    read_line_sensor();
    threshold();
    motor_control(1,120);
    steering_by_camera();
    Line_de = line_detect_check();
    Serial.print("Line_detect ");
    Serial.println(Line_de);
    if(R_Sonar_distance+L_Sonar_distance <=1120) 
    {
     motor_control(1,0);
     delay(100);
     flag = 3;
    }
  }

  if(flag == 3)
  {
    read_sonar_sensor();
    steering_control(0);
    motor_control(1,110);
    delay(50);
    if(F_Sonar_distance <= 800)
    {
      flag = 4;
    }
  }
  
  if(flag == 4)
  {
    read_sonar_sensor();
    steering_control(40);
    motor_control(1,150);
    delay(150);
    if(L_Sonar_distance <= 600)
    {
      flag = 5;
    }
  }

  if(flag == 5)
  {
    read_sonar_sensor();
    steering_control(0);
    motor_control(1,110);
    if(L_Sonar_distance <= 750)
    {
      flag = 6;
    }
  }

  if(flag == 6)
  {
    read_sonar_sensor();
    two_wall_following();
    motor_control(1,110);
    if(R_Sonar_distance >= 800)
    {
      flag = 7;
    }
  }

  if(flag ==7)
  {
    read_sonar_sensor();
    steering_control(0);
    motor_control(1,110);
    if(F_Sonar_distance <= 680)
    {
      flag = 8;
    }
  }

  if(flag == 8)
  {
    read_sonar_sensor();
    steering_control(40);
    motor_control(1,165);
    delay(10);
    if(L_Sonar_distance >= 675)
    {
      flag = 9;
    }
  }

  if(flag == 9)
  {
    read_sonar_sensor();
    steering_control(0);
    motor_control(1,110);
    if(L_Sonar_distance+R_Sonar_distance >=1120)
    {
      flag = 10;
    }
    
  }

  if(flag == 10)
  {
    read_sonar_sensor();
    two_wall_following();
    motor_control(1,110);
    line_adaptation();
    read_line_sensor();
    threshold();
    steering_control(0);
    //motor_control(1,110);
    Line_de = line_detect_check();
  //Serial.print("Line_detect ");
  //Serial.println(Line_de);
    if(R_Sonar_distance+L_Sonar_distance >= 1120) 
    {
     motor_control(1,0);
     delay(10);
     flag = 11;
    }
  }

  if(flag == 11)
  {
    read_sonar_sensor();
    line_adaptation();
    read_line_sensor();
    threshold();
    motor_control(1,120);
    steering_by_camera();
    Line_de = line_detect_check();
    if(F_Sonar_distance <= 400)
    {
      steering_control(0);
      motor_control(1, 0);
    }
  //Serial.print("Line_detect ");
  //Serial.println(Line_de)
  }
  
  /////////////////////        FLAG        /////////////////////
  //////////////////////////////////////////////////////////////
  

  /*
  int i;
  int Line_de =0;

  read_sonar_sensor();
    line_adaptation();
  read_line_sensor();
  threshold();
  Line_de = line_detect_check();
  
  
  if (digitalRead(CLKpin) == HIGH)
  {
    //line_adaptation
    flag_line_adapation = 1;
  }
   //line_detect_check();
   //threshold();
   //steering_by_camera();
  for (i = 0; i < NPIXELS; i++)
  {
   // if (digitalRead(CLKpin) == LOW)      Serial.print(LineSensor_Data_Adaption[i]); // Serial.print(LineSensor_Data[i] );
    //else                                 Serial.print ((byte)Pixel[i] + 1);
    //Serial.print ((byte)Pixel[i] + 1);
    // read_sonar_sensor();
  

    line_data = LineSensor_Data_Adaption[i];
    Serial.print(Line_de);
    Serial.print(" ");
    if(i == NPIXELS)  Serial.println(" ");
  }
  
  Serial.println("  ");
  delay(100);
  */
}
