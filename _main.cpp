#include <stdio.h>
#include <iostream>
#include <chrono>

#include "RobotConnector.h"

#include "cv.h"
#include "highgui.h"

using namespace std;
using namespace std::chrono;

#define Create_Comport "COM3"

#define CCW 0
#define CW  1

bool isRecord = false;

/* |-- black --|-- grey --|-- white --| */

const int32_t white_thresholds[4] = { 700, 700, 1050, 1270 };
const int32_t black_thresholds[4] = { 500, 700, 1050, 1230 };
int n = 0;


long long now_ms() {
	milliseconds ms = duration_cast< milliseconds >(
		system_clock::now().time_since_epoch());
	return ms.count();
}

void sleep(int duration_ms) {
	long long start = now_ms();
	while (now_ms() - start < duration_ms);
}

int main() {
	CreateData robotData;
	RobotConnector robot;
	char floor_colors[4] = { 'w', 'w', 'w', 'w' };
	int robot_direction = CCW;
	bool direction_checked = false;

	ofstream record;
	record.open("../data/robot.txt");

	if (!robot.Connect(Create_Comport)) {
		cout << "Error : Can't connect to robot @" << Create_Comport << endl;
		return -1;
	}

	robot.DriveDirect(0, 0);
	cvNamedWindow("Robot");

	int cliff_history[4] = { 0, 0, 0, 0 };
	int direction_checked_counter = 0;
	int ccw_counter = 0;
	int cw_counter = 0;
	int ma[4];
	int bumper_count = 0;
#define BUMPER_THRESHOLD 12

	while (true) {
		//char c = cvWaitKey(30);
		//if (c == 27) break;

		double vx, vz;
		vx = vz = 0.0;

		/*switch (c)
		{
		case 'w': vx = +1; break;
		case 's': vx = -1; break;
		case ' ': vx = vz = 0; break;
		case 'c': robot.Connect(Create_Comport); break;
		}
		*/
		vx = 1;

		if (!robot.ReadData(robotData))
			cout << "ReadData Fail" << endl;

		// update Filter
		for (int i = 0; i <= 3; i++) {
			if (cliff_history[i] == 0) {
				ma[i] = robotData.cliffSignal[i];
			}
			else {
				ma[i] = (cliff_history[i] + robotData.cliffSignal[i]) >> 1;
			}
			cliff_history[i] = robotData.cliffSignal[i];
		}
		// Process From cliffSignal => floor color

		for (int i = 0; i <= 3; i++) {
			if (ma[i] < black_thresholds[i]) {
				floor_colors[i] = 'b';
			}
			else if (ma[i] > white_thresholds[i]) {
				floor_colors[i] = 'w';
			}
			else {
				floor_colors[i] = 'g';
			}
		}

		int velL, velR;
		// handle bumping

		if (robotData.bumper[0] || robotData.bumper[1]) {
			bumper_count++;
		}
		else {
			bumper_count = 0;
		}

		if (bumper_count >= BUMPER_THRESHOLD) {
			bumper_count = 0;
			cout << "bumped!" << endl;
			if (direction_checked) {
				// move back
				velR = (int)-0.5 * Create_MaxVel;
				velL = (int)-0.5 * Create_MaxVel;
				if (!robot.DriveDirect(velL, velR))
					cout << "SetControl Fail" << endl;
				sleep(100);

				// turn around 180 degrees
				velR = (int)-1.0 * Create_MaxVel;
				velL = (int)  1.0 * Create_MaxVel;
				if (!robot.DriveDirect(velL, velR))
					cout << "SetControl Fail" << endl;
				sleep(850);

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


		// Check direction
		if (!direction_checked) {
			if (direction_checked_counter < 3) {
				if (floor_colors[1] == 'w' && floor_colors[2] == 'b') {
					ccw_counter++;
				}
				else if (floor_colors[1] == 'b' && floor_colors[2] == 'w') {
					cw_counter++;
				}
				else {
					continue;
				}
				direction_checked_counter++;
				continue;
			}
			else {
				direction_checked = true;
			}
			if (ccw_counter > cw_counter) {
				robot_direction = CCW;
			}
			else {
				robot_direction = CW;
			}

		}




		// Condition of turning left/right
#define MAX_N 100
		double vl, vr;
		vl = vr = vx;
		int16_t temp_color;
		if ((temp_color = floor_colors[1]) == floor_colors[2]) {
			if (temp_color == 'w') {
				if (robot_direction == CCW) {
					vr = 0.55 - n * 0.001;  // turn right
				}
				else if (robot_direction == CW) {
					vl = 0.55 - n * 0.001;  // turn left
				}
				if (n < MAX_N) {
					n += 1;
				}
			}
			else if (temp_color == 'b') {
				if (robot_direction == CCW) {
					vl = 0.45 - n * 0.001;  // turn left
				}
				else if (robot_direction == CW) {
					vr = 0.45 - n * 0.001;  // turn right
				}
				if (n < MAX_N) {
					n += 1;
				}
			}
		}
		else {
			n = 0;
		}

		cout << "N: " << n << " ";

		velL = (int)(vl * Create_MaxVel);
		velR = (int)(vr * Create_MaxVel);
		int led_color = (abs(velL) + abs(velR)) / 4;
		led_color = (led_color < 0) ? 0 : (led_color > 255) ? 255 : led_color;

		int inten = (robotData.cliffSignal[1] + robotData.cliffSignal[2]) / 8 - 63;
		inten = (inten < 0) ? 0 : (inten > 255) ? 255 : inten;

		//cout << led_color << " " << inten << " " << robotData.cliffSignal[1] << " " << robotData.cliffSignal[2] << endl;

		robot.LEDs(velL > 0, velR > 0, led_color, inten);

		if (!robot.DriveDirect(velL, velR))
			cout << "SetControl Fail" << endl;

		//cout << robotData.cliffSignal[0] << "\t" << robotData.cliffSignal[1] << "\t" << robotData.cliffSignal[2] << "\t" << robotData.cliffSignal[3] << endl;
		//cout << floor_colors[0] << "\t" << floor_colors[1] << "\t"
		//		<< floor_colors[2] << "\t" << floor_colors[3] << endl;
		cout << robotData.bumper[0] << "\t" << robotData.bumper[1] << endl;
		//cout << "Robot " << robotData.infrared << endl;
		cout << "NOW: " << now_ms() << std::endl;
	}

	robot.Disconnect();

	return 0;
}
