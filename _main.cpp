
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
int n = 0;

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

	int previous[4] = {0, 0, 0, 0};
	int ma[4];

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
		for (int i = 0; i <= 3; i++) {
            if (previous[i] == 0) {
                ma[i] = robotData.cliffSignal[i];
            }
            else {
                ma[i] = (previous[i] + robotData.cliffSignal[i]) >> 1;
            }
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


		// Condition of turning left/right
        #define MAX_N 100
		double vl, vr;
		vl = vr = vx;
		int16_t temp_color;
		if ((temp_color = floor_colors[1]) == floor_colors[2]) {
			if (temp_color == 'w') {
				vr = 0.55 - n * 0.001;  // turn right
				if (n < MAX_N){
                    n += 1;
				}
			}
			else if (temp_color == 'b') {
				vl = 0.55 - n * 0.001;  // turn left
				if (n < MAX_N) {
                    n += 1;
				}
			}
		} else {
            n = 0;
		}
		cout << "N: " << n << " ";

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
