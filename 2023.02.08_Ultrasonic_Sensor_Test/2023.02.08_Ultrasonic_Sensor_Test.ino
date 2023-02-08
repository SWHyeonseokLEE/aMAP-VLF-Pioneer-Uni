#include <NewPing.h>

  #define FL_Sonar 9
  #define FR_Sonar 10
  #define R_Sonar 11          
  #define MaxDistance  350
  
  NewPing FL_sensor(FL_Sonar,FL_Sonar,MaxDistance);
  float FL_Sonar_distance;
  NewPing FR_sensor(FR_Sonar,FR_Sonar,MaxDistance);
  float FR_Sonar_distance;
  NewPing R_sensor(R_Sonar,R_Sonar,MaxDistance);
  float R_Sonar_distance;
  
  void read_sonar_sensor(void)   //초음파센서 측정
  {
    FL_Sonar_distance = FL_sensor.ping_cm()*10.0;
    FR_Sonar_distance = FR_sensor.ping_cm()*10.0;
    R_Sonar_distance = R_sensor.ping_cm()*10.0;
    if(FL_Sonar_distance == 0){FL_Sonar_distance = MaxDistance * 10.0;}
    if(FR_Sonar_distance == 0){FR_Sonar_distance = MaxDistance * 10.0;}
    if(R_Sonar_distance == 0){R_Sonar_distance = MaxDistance * 10.0;}
  }

void setup()
{
  Serial.begin(115200);
}

void loop()
{
  read_sonar_sensor();
  Serial.print(FL_Sonar_distance);
  Serial.print("    ");
  Serial.print(FR_Sonar_distance);
  Serial.print("    ");
  Serial.println(R_Sonar_distance);
}
