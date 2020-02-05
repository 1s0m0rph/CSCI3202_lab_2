#include <Sparki.h>

#define CYCLE_TIME .100// seconds

// Program States
#define CONTROLLER_FOLLOW_LINE 1
#define CONTROLLER_DISTANCE_MEASURE 2

#define FORWARD 0
#define TURNING_LEFT 1
#define TURNING_RIGHT 2

//#define ROBOT_SPEED 0.03  //m/s adjusted to provide accurate positional readings
#define ROBOT_AXEL_RADIUS 0.042 // using the turn radius removes redundant division by 2

int current_state = CONTROLLER_FOLLOW_LINE; // Change this variable to determine which controller to run
const int threshold = 700;
int line_left = 1000;
int line_center = 1000;
int line_right = 1000;

int movement_direction = 0;

unsigned long timer_begin = 0;

float pose_x = 0., pose_y = 0., pose_theta = 0.;

void setup()
{
  pose_x = 0.;
  pose_y = 0.;
  pose_theta = 0.;
  delay(1000);
  timer_begin = millis();
}

void readSensors()
{
  line_left = sparki.lineLeft();
  line_right = sparki.lineRight();
  line_center = sparki.lineCenter();
  // distance = sparki.ping();
}

void measure_30cm_speed()
{
  sparki.moveForward();
  if(line_left < threshold && line_right < threshold)
  {
    sparki.moveStop();
    unsigned long timer_end = millis();//probably won't work

    double duration_sec = (timer_end - timer_begin) / 1000.;
    double speed = (.3) / duration_sec;
    sparki.print("Speed (m/s): ");
    sparki.println(speed);
    sparki.updateLCD();
    //current_state = CONTROLLER_FOLLOW_LINE;
    //measured to be 0.03 m/s
  }
}


void updateOdometry()
{
  //double elapsed_time_s = (millis() - timer_begin) / 1000.;
  //double elapsed_time_s = .01;
  double dist = .0003;//this will always be the dist traveled
  if(movement_direction == TURNING_LEFT)
    pose_theta += ((2 * dist) / ROBOT_AXEL_RADIUS);
  else if(movement_direction == TURNING_RIGHT)
    pose_theta -= ((2 * dist) / ROBOT_AXEL_RADIUS);
  //else we're moving forward, so no angle update
  else
  {
    pose_x += (dist) * cos(pose_theta);
    pose_y += (dist) * sin(pose_theta);
  }
}

void displayOdometry()
{
  sparki.print("(");
  sparki.print(pose_x);
  sparki.print(",");
  sparki.print(pose_y);
  sparki.print(",");
  sparki.print(pose_theta);
  sparki.println(")");
  sparki.updateLCD();
}

void close_loop()
{
  pose_x = 0;
  pose_y = 0;
  pose_theta = 0;
}

void loop()
{
  unsigned long loop_begin = millis();
  
  readSensors();
  
  switch (current_state) 
  {
    case CONTROLLER_FOLLOW_LINE:
      displayOdometry();
      updateOdometry();
      if( (line_center < threshold) && (line_left < threshold) && (line_right < threshold))
      {
        close_loop();
         
      }
      else if ( line_left < threshold ) // if line is below left line sensor
      {
        sparki.moveLeft(); // turn left
        movement_direction = TURNING_LEFT;
      }
       
      else if ( line_right < threshold ) // if line is below right line sensor
      {
        sparki.moveRight(); // turn right
        movement_direction = TURNING_RIGHT;
      }
       
      // if the center line sensor is the only one reading a line
      if ( line_center < threshold)
      {
        sparki.moveForward(); // move forward
        movement_direction = FORWARD;
      }
      break;
    case CONTROLLER_DISTANCE_MEASURE:
      measure_30cm_speed();
      break;
  }
  unsigned long loop_end = millis();
  double elapsed_time_s = (loop_end - loop_begin) / 1000.;
  double remaining_time = (elapsed_time_s >= CYCLE_TIME) ? 0 : CYCLE_TIME - elapsed_time_s;
  delay(remaining_time);
}
