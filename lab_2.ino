#include <sparki.h>

#define CYCLE_TIME .100// seconds

// Program States
#define CONTROLLER_FOLLOW_LINE 1
#define CONTROLLER_DISTANCE_MEASURE 2


int current_state = CONTROLLER_FOLLOW_LINE; // Change this variable to determine which controller to run
const int threshold = 700;
int line_left = 1000;
int line_center = 1000;
int line_right = 1000;

float pose_x = 0., pose_y = 0., pose_theta = 0.;

void setup()
{
	pose_x = 0.;
	pose_y = 0.;
	pose_theta = 0.;
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
	unsigned long timer_begin = millis();
	sparki.moveForward(30);//move forward 30cm
	unsigned long timer_end = millis();//probably won't work
	
	double duration_sec = (timer_end - timer_begin) / 1000.;
	double speed = (30. * (1. / 100.)) / duration_sec;
	sparki.print("Speed (m/s): ");
	sparki.println(speed);
}


void updateOdometry()
{
	// TODO
}

void displayOdometry()
{
	// TODO
}

void loop()
{
	// TODO: Insert loop timing/initialization code here
	unsigned long loop_begin = millis();
	
	readSensors();

	switch (current_state) {
		case CONTROLLER_FOLLOW_LINE:
			if ( lineLeft < threshold ) // if line is below left line sensor
			{
				sparki.moveLeft(); // turn left
			}
			 
			if ( lineRight < threshold ) // if line is below right line sensor
			{
				sparki.moveRight(); // turn right
			}
			 
			// if the center line sensor is the only one reading a line
			if ( (lineCenter < threshold) && (lineLeft > threshold) && (lineRight > threshold) )
			{
				sparki.moveForward(); // move forward
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