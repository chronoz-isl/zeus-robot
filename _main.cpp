/** zeus-robot :: _main.cpp
  * Project for 2110453 Intro to Robotics (Lab 1) 
  * Team members: Ben, Dino, Kris
  */

#include <stdio.h>
#include <iostream>
#include <chrono>

#include "RobotConnector.h"

#include "cv.h"
#include "highgui.h"

using namespace std;
using namespace std::chrono;

#define Create_Comport "COM3"

// Tuning variables //

#define BUMPER_THRESHOLD 12
#define INIT_DIR_CTR_THRESHOLD 3
#define TURN_BACK_DURATION 850

#define MAX_N 100
#define DRIFT_CONST_A 0.55
#define DRIFT_CONST_B (-0.001)

// Constants //

#define CCW       0
#define CW        1
#define UNCHECKED 2

bool isRecord = false;

/* |-- black --|-- grey --|-- white --| */

const int32_t white_thresholds[4] = { 700, 700, 1050, 1270 };
const int32_t black_thresholds[4] = { 500, 700, 1050, 1230 };

// Global Variables //

int n = 0;

// Time Functions //

long long now_ms() {
	milliseconds ms = duration_cast< milliseconds >(
		system_clock::now().time_since_epoch());
	return ms.count();
}

void sleep(int duration_ms) {
	long long target = now_ms() + duration_ms;
	while (now_ms() < target);
}

// Main //

int main() {
	CreateData robotData;
	RobotConnector robot;
	ofstream record;
	record.open("../data/robot.txt");

	if (!robot.Connect(Create_Comport)) {
		cout << "Error : Can't connect to robot @" << Create_Comport << endl;
		return -1;
	}

	robot.DriveDirect(0, 0);
	cvNamedWindow("Robot");
	
	// Loop Variables //
	
	char floor_colors[4] = { 'w', 'w', 'w', 'w' };
	int robot_direction = UNCHECKED;
	int cliff_history[4] = { -1, -1, -1, -1 };
	int ccw_counter = 0;
	int cw_counter = 0;
	int ma[4];
	int bumper_count = 0;
	
	// Main Loop //

	while (true) {
            
        // local loop variables //
        
		int velL, velR;
		
		//char c = cvWaitKey(30);
		//if (c == 27) break;

		//double vx, vz;
		//vx = vz = 0.0;

		//switch (c)
		//{
		//case 'w': vx = +1; break;
		//case 's': vx = -1; break;
		//case ' ': vx = vz = 0; break;
		//case 'c': robot.Connect(Create_Comport); break;
		//}
		//vx = 1;
		
		// Read Data //

		if (!robot.ReadData(robotData))
			cout << "ReadData Fail" << endl;

		/* Update Filter 
		 * The filter is a 'moving average' filter.
         * If the buffer/history is not filled, the denominator is changed to match buffer/history size.
         */
		
		for (int i = 0; i <= 3; i++) {
			if (cliff_history[i] == -1) {
				ma[i] = robotData.cliffSignal[i];
			}
			else {
				ma[i] = (cliff_history[i] + robotData.cliffSignal[i]) / 2;
			}
			cliff_history[i] = robotData.cliffSignal[i];
		}
		
		/* Process Cliff Signal
		 * Cliff Signal is processed into floor_color according to configured color thresholds.
		 * If max_black and min_white thresholds are the same, color will be black.
		 */

		for (int i = 0; i <= 3; i++) {
			if (ma[i] <= black_thresholds[i]) {
				floor_colors[i] = 'b';
			}
			else if (ma[i] >= white_thresholds[i]) {
				floor_colors[i] = 'w';
			}
			else {
				floor_colors[i] = 'g';
			}
		}

		/* Initial Check direction
		 * Check direction once when start. This code will run repeatedly until INIT_DIR_CTR_THRESHOLD is reached.
		 * The direction is set to the most frequent direction measured.
		 */
		
		if (robot_direction == UNCHECKED) {
			if ((ccw_counter + cw_counter) < INIT_DIR_CTR_THRESHOLD) {
				if (floor_colors[1] == 'w' && floor_colors[2] == 'b') {
					ccw_counter++;
				}
				else if (floor_colors[1] == 'b' && floor_colors[2] == 'w') {
					cw_counter++;
				}
				continue;
			}
			else {  // finished checking
                if (ccw_counter > cw_counter) {
                    robot_direction = CCW;
                }
                else {
                    robot_direction = CW;
                }
			}
		}
		
		// Bumper Filtering

		if (robotData.bumper[0] || robotData.bumper[1]) {
			bumper_count++;
		}
		else {
			bumper_count = 0;
		}
		
		/* Handle Bumping
		 * If bumper_count is more than BUMPER_THRESHOLD, then bumping procedure is called.
		 * The bumping procedure consists of (1) moving back, (2) turning around 180 degrees and 
		 * (3) reversing the current direction.
		 */

		if (bumper_count >= BUMPER_THRESHOLD) {
			bumper_count = 0;
			cout << "bumped!" << endl;
			if (robot_direction != UNCHECKED) {
				// move back
				velR = (int) (-0.5 * Create_MaxVel);
				velL = (int) (-0.5 * Create_MaxVel);
				if (!robot.DriveDirect(velL, velR))
					cout << "SetControl Fail" << endl;
				sleep(100);

				// turn around 180 degrees
				velR = (int) (-1.0 * Create_MaxVel);
				velL = (int)  (1.0 * Create_MaxVel);
				if (!robot.DriveDirect(velL, velR))
					cout << "SetControl Fail" << endl;
				sleep(TURN_BACK_DURATION);

				// Change Direction
				if (robot_direction == CW) {
					robot_direction = CCW;
				}
				else if (robot_direction == CCW) {
					robot_direction = CW;
				}
			}
			continue;
		}


		/* Drifting (Keeping in line)
		 * If 'out of lane' is detected then one of the wheels runs at reduced_velocity. 
		 * Else, both runs at max velocity. Reduced Velocity is calculated by a linear equation 
		 * 'RVel = A + nB'. n is the number of loop cycles where 'out of lane' is detected. 
		 * B is normally negative to account for 'straight vs turn' conditions. 
		 * n is constrained to MAX_N and is reseted to 0 when robot is in lane.
		 */

		double vl, vr;
		vl = vr = 1.0;
		int16_t temp_color;
		if ((temp_color = floor_colors[1]) == floor_colors[2]) {
            double reduced_velocity = DRIFT_CONST_A + (n * DRIFT_CONST_B);
			if (temp_color == 'w') {
				if (robot_direction == CCW) {
					vr = reduced_velocity;  // turn right
				}
				else if (robot_direction == CW) {
					vl = reduced_velocity;  // turn left
				}
				if (n < MAX_N) {
					n++;
				}
			}
			else if (temp_color == 'b') {
				if (robot_direction == CCW) {
					vl = reduced_velocity;  // turn left
				}
				else if (robot_direction == CW) {
					vr = reduced_velocity;  // turn right
				}
				if (n < MAX_N) {
					n++;
				}
			}
		}
		else {
			n = 0;
		}

        // Actuate Wheels //

		velL = (int)(vl * Create_MaxVel);
		velR = (int)(vr * Create_MaxVel);
		if (!robot.DriveDirect(velL, velR))
			cout << "SetControl Fail" << endl;

        // Set LED //

		int led_color = (abs(velL) + abs(velR)) / 4;
		led_color = (led_color < 0) ? 0 : (led_color > 255) ? 255 : led_color;

		int inten = (robotData.cliffSignal[1] + robotData.cliffSignal[2]) / 8 - 63;
		inten = (inten < 0) ? 0 : (inten > 255) ? 255 : inten;
		
		robot.LEDs(velL > 0, velR > 0, led_color, inten);
		
		// Logging/Debuging //

		//cout << led_color << " " << inten << " " << robotData.cliffSignal[1] << " " << robotData.cliffSignal[2] << endl;
		//cout << robotData.cliffSignal[0] << "\t" << robotData.cliffSignal[1] << "\t" << robotData.cliffSignal[2] << "\t" << robotData.cliffSignal[3] << endl;
		//cout << floor_colors[0] << "\t" << floor_colors[1] << "\t"
		//		<< floor_colors[2] << "\t" << floor_colors[3] << endl;
		//cout << robotData.bumper[0] << "\t" << robotData.bumper[1] << endl;
		//cout << "Robot " << robotData.infrared << endl;
		//cout << "NOW: " << now_ms() << std::endl;
	}

	robot.Disconnect();

	return 0;
}
