
#include <stdio.h>
#include <iostream>

#include "RobotConnector.h"

#include "cv.h"
#include "highgui.h"

using namespace std;

#define Create_Comport "COM3"

bool isRecord = false;

/* |-- black --|-- grey --|-- white --| */

const int32_t white_thresholds[4] = { 700, 700, 1050, 1270 };
const int32_t black_thresholds[4] = { 500, 700, 1050, 1230 };

int main()
{
	CreateData	robotData;
	RobotConnector	robot;
	char floor_colors[4] = { 'w', 'w', 'w', 'w' };

	ofstream	record;
	record.open("../data/robot.txt");

	if (!robot.Connect(Create_Comport))
	{
		cout << "Error : Can't connect to robot @" << Create_Comport << endl;
		return -1;
	}

	robot.DriveDirect(0, 0);
	cvNamedWindow("Robot");

	int previous0 = 0;
	int previous1 = 0;
	int previous2 = 0;
	int previous3 = 0;
	int ma0;
	int ma1;
	int ma2;
	int ma3;

	while (true)
	{
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

		//Filter
		if (previous0 == 0) {
			ma0 = robotData.cliffSignal[0];
		}
		else {
			ma0 = (previous0 + robotData.cliffSignal[0]) / 2;
		}
		if (previous1 == 0) {
			ma1 = robotData.cliffSignal[1];
		}
		else {
			ma1 = (previous1 + robotData.cliffSignal[1]) / 2;
		}
		if (previous2 == 0) {
			ma2 = robotData.cliffSignal[2];
		}
		else {
			ma2 = (previous2 + robotData.cliffSignal[2]) / 2;
		}
		if (previous3 == 0) {
			ma3 = robotData.cliffSignal[3];
		}
		else {
			ma3 = (previous0 + robotData.cliffSignal[3]) / 2;
		}




		// Process From cliffSignal => floor color
	
		if (ma0 < black_thresholds[0]) {
			floor_colors[0] = 'b';
		}
		else if (ma0 > white_thresholds[0]) {
			floor_colors[0] = 'w';
		}
		else {
			floor_colors[0] = 'g';
		}
		if (ma1 < black_thresholds[1]) {
			floor_colors[1] = 'b';
		}
		else if (ma1 > white_thresholds[1]) {
			floor_colors[1] = 'w';
		}
		else {
			floor_colors[1] = 'g';
		}
		if (ma2 < black_thresholds[2]) {
			floor_colors[2] = 'b';
		}
		else if (ma2 > white_thresholds[2]) {
			floor_colors[2] = 'w';
		}
		else {
			floor_colors[2] = 'g';
		}
		if (ma3 < black_thresholds[3]) {
			floor_colors[3] = 'b';
		}
		else if (ma3 > white_thresholds[3]) {
			floor_colors[3] = 'w';
		}
		else {
			floor_colors[3] = 'g';
		}
		

		// Condition of turning left/right

		double vl, vr;
		vl = vr = vx;
		int16_t temp_color;
		if ((temp_color = floor_colors[1]) == floor_colors[2]) {
			if (temp_color == 'w') {
				vr = 0;  // turn right
			}
			else if (temp_color == 'b') {
				vl = 0;  // turn left
			}
		}

		int velL = (int)(vl*Create_MaxVel);
		int velR = (int)(vr*Create_MaxVel);

		int color = (abs(velL) + abs(velR)) / 4;
		color = (color < 0) ? 0 : (color > 255) ? 255 : color;

		int inten = (robotData.cliffSignal[1] + robotData.cliffSignal[2]) / 8 - 63;
		inten = (inten < 0) ? 0 : (inten > 255) ? 255 : inten;

		//cout << color << " " << inten << " " << robotData.cliffSignal[1] << " " << robotData.cliffSignal[2] << endl;

		robot.LEDs(velL > 0, velR > 0, color, inten);

		if (!robot.DriveDirect(velL, velR))
			cout << "SetControl Fail" << endl;

		//cout << robotData.cliffSignal[0] << "\t" << robotData.cliffSignal[1] << "\t" << robotData.cliffSignal[2] << "\t" << robotData.cliffSignal[3] << endl;
		cout << floor_colors[0] << "\t" << floor_colors[1] << "\t" << floor_colors[2] << "\t" << floor_colors[3] << endl;

		//cout << "Robot " << robotData.infrared << endl;
	}

	robot.Disconnect();

	return 0;
}
