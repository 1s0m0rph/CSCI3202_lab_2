#include <Sparki.h>

#define CYCLE_TIME .100// seconds

// Program States
#define CONTROLLER_FOLLOW_LINE 1
#define CONTROLLER_DISTANCE_MEASURE 2


int current_state = CONTROLLER_DISTANCE_MEASURE; // Change this variable to determine which controller to run
const int threshold = 700;
int line_left = 1000;
int line_center = 1000;
int line_right = 1000;

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
	if(line_center > threshold)
	{
		sparki.moveStop();
		unsigned long timer_end = millis();//probably won't work

		double duration_sec = (timer_end - timer_begin) / 1000.;
		double speed = (30. * (1. / 100.)) / duration_sec;
		sparki.print("Speed (m/s): ");
		sparki.println(speed);
		sparki.updateLCD();
		current_state = CONTROLLER_FOLLOW_LINE;
		//measured to be 0.03 m/s
	}
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
			if ( line_left < threshold ) // if line is below left line sensor
			{
				sparki.moveLeft(); // turn left
			}
			 
			if ( line_right < threshold ) // if line is below right line sensor
			{
				sparki.moveRight(); // turn right
			}
			 
			// if the center line sensor is the only one reading a line
			if ( (line_center < threshold) && (line_left > threshold) && (line_right > threshold) )
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
