#include "sparki-ik.h"

#define CONTROLLER_FOLLOW_LINE 1
#define CONTROLLER_GOTO_POSITION_PART2 2
#define CONTROLLER_GOTO_POSITION_PART3 3

// Line following configuration variables
const int threshold = 700;
int line_left = 1000;
int line_center = 1000;
int line_right = 1000;

// Controller and dTheta update rule settings
int current_state = CONTROLLER_GOTO_POSITION_PART3;

Sparki sprk(0.1, 0.05, 0);

float to_radians(double deg) {
  return  deg * 3.1415/180.;
}

float to_degrees(double rad) {
  return  rad * 180 / 3.1415;
}

void setup() {
  Serial.begin(9600);
  
}

void readSensors() {
  line_left = sparki.lineLeft();
  line_right = sparki.lineRight();
  line_center = sparki.lineCenter();
}

void displayOdometry() {
  sparki.print("X: ");
  sparki.print(sprk.x);
  sparki.print(" Xg: ");
  sparki.println(sprk.target_x);
  sparki.print("Y: ");
  sparki.print(sprk.y);
  sparki.print(" Yg: ");
  sparki.println(sprk.target_y);
  sparki.print("T: ");
  sparki.print(to_degrees(sprk.theta));
  sparki.print(" Tg: ");
  sparki.println(to_degrees(sprk.target_theta));

  sparki.print("phl: "); sparki.print(sprk.l_speed); sparki.print(" phr: "); sparki.println(sprk.r_speed);
  sparki.print("p: "); sparki.print(sprk.distance); sparki.print(" a: "); sparki.println(to_degrees(sprk.bearing));
  sparki.print("h: "); sparki.println(to_degrees(sprk.heading));
}

void print_deets()
{
  Serial.print("x ");
  Serial.print(sprk.x);
  Serial.print("\ty ");
  Serial.print(sprk.y);
  Serial.print("\ttheta ");
  Serial.println(sprk.theta);
  Serial.print("dist ");
  Serial.print(sprk.distance);
  Serial.print("\tbearing ");
  Serial.print(sprk.bearing);
  Serial.print("\theading ");
  Serial.println(sprk.heading);
  Serial.println();
}

void loop() {
  unsigned long begin_time = millis();
  unsigned long end_time = 0;
  unsigned long delay_time = 0;
  
  switch (current_state) {
 
    case CONTROLLER_GOTO_POSITION_PART3:
      // TODO: Implement solution using motorRotate and proportional feedback controller.
      // sparki.motorRotate function calls for reference:
      //      sparki.motorRotate(MOTOR_LEFT, left_dir, int(left_speed_pct*100.));
      //      sparki.motorRotate(MOTOR_RIGHT, right_dir, int(right_speed_pct*100.));

        float phi_l_pct = 0;
        float phi_r_pct = 0;
         
        sprk.step(&phi_l_pct, &phi_r_pct);
        print_deets();
        if(fabs(sprk.l_speed) <  DIST_ERR && fabs(sprk.r_speed) < DIST_ERR)
        {
          sparki.motorStop(MOTOR_LEFT);
          sparki.motorStop(MOTOR_RIGHT);
          phi_l_pct = 0;
          phi_r_pct = 0;
        }
        else
        {
          if(phi_l_pct < 0)
          {
            sparki.motorRotate(MOTOR_LEFT, DIR_CW, fabs(phi_l_pct) * 100);
          }
          else
          {
            sparki.motorRotate(MOTOR_LEFT, DIR_CCW, phi_l_pct * 100);
          }
    
          if(phi_r_pct < 0)
          {
            sparki.motorRotate(MOTOR_RIGHT, DIR_CCW, fabs(phi_r_pct) * 100);
          }
          else
          {
            sparki.motorRotate(MOTOR_RIGHT, DIR_CW, phi_r_pct * 100);
          }
        }
      
      break;
  }

  sparki.clearLCD();
  displayOdometry();
  sparki.updateLCD();
  end_time = millis();
  delay_time = end_time - begin_time;
  if (delay_time < 1000*CYCLE_TIME)
    delay(1000*CYCLE_TIME - delay_time); // each loop takes CYCLE_TIME ms
  else
    delay(10);
}
