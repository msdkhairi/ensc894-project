// ProgrammingDemo.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <stdio.h>
#include <conio.h>
//#include "ensc-488.h"
#include <iostream>
#include <string>
#include "helper.h"
using namespace std;

int main(int argc, char* argv[])
{
	JOINT q1 = {0, 0, -100, 0};
	JOINT q1_d = { 0, 0, -200, 0 };
	JOINT q2 = {90, 90, -200, 45};
	JOINT q3 = { 0, 0, -200, 0 };
	printf("Keep this window in focus, and...\n");

	

	char ch;
	int c;

	const int ESC = 27;
	
	printf("Press any key to continue \n");
	printf("Press ESC to exit \n");

	c = _getch() ;

	// initialize the robot
	ROBSIM::ROBOT robot;
	// set the robot q to the current configuration
	auto current_q(robot.current_config());
	robot.setQ(current_q);

	system("cls");

	while (1)
	{	
		printf("=======================================================\n");
		printf("Press 'c': clear, 'r': reset, 'g': grasp, 'u': ungrasp\n");
		printf("Press '1' to move to a joint configuration \n");
		printf("Press '2' to move to a pose \n");
		printf("Press '3' to pick and place an object \n");
		printf("Press ESC to exit \n");
		printf("-------------------------------------------------------\n");
		std::cout << "input: " << std::flush;
		ch = _getch();
		std::cout << "\n" << std::flush;
		if (ch != ESC)
		{
			//printf("Press '1' or '2' \n");
			//ch = _getch();

			if (ch == '1')
			{
				current_q = robot.current_config();
				robot.setQ(current_q);

				bool sol = false;
				std::cout << "Please enter the joint values \ntheta1 theta2 d3 theta4\n" << std::flush;
				JOINT q_input{ 100, 0, -100, 0 };
				for (int i = 0; i < 4; ++i) {
					std::cin >> q_input[i];
				}
				printf("You entered the following joint values\n");
				ROBSIM::q_vec q_vec_input(q_input);
				q_vec_input.display();

				ROBSIM::frame trels;
				auto trels_vec = robot.WHERE(q_vec_input, trels);

				if (robot.is_robot_q_valid()) {
					printf("trels: \n");
					trels.matrix.display();

					printf("trels_vec (pose): ");
					trels_vec.display();

					/*ROBSIM::frame trels_current;
					ROBSIM::q_vec near, far;
					bool sol;
					robot.SOLVE(trels_vec, current_q, near, far, sol);*/


					printf("Would you like the robot to move to this configuration\n");
					printf("Press 'y' for yes and 'n' for no\n");
					ch = _getch();
					if (ch == 'y') {
						std::cout << "Robot Moving to the configuration ...\n";
						MoveToConfiguration(q_input);
						std::cout << "Robot has moved to the configuration\n";
					}
					else {
						printf("The robot will not move to the configuration\n");
					}
				}
				else {
					printf("Try again\n");
				}
			}
			else if (ch == '2')
			{
				//MoveToConfiguration(q1);
				//DisplayConfiguration(q1);
				bool sol = false;
				ROBSIM::q_vec near, far;

				current_q = robot.current_config();
				robot.setQ(current_q);

				std::cout << "Please enter the pose values\nx\ty\tz\tphi\n" << std::flush;
				JOINT pose{ 100, 0, -100, 0 };
				for (int i = 0; i < 4; ++i) {
					std::cin >> pose[i];
				}
				printf("You entered the following pose values\n");
				ROBSIM::vec4 pose_input(pose[0], pose[1], pose[2], pose[3]);
				pose_input.display();

				robot.SOLVE(pose_input, current_q, near, far, sol);

				if (sol) {
					printf("Would you like the robot to move to this configuration\n");
					printf("Press 'y' for yes and 'n' for no\n");
					ch = _getch();
					if (ch == 'y') {
						std::cout << "Robot Moving to the configuration ...\n";
						JOINT near_joint;
						near.toJOINT(near_joint);
						MoveToConfiguration(near_joint);
						std::cout << "Robot has moved to the configuration\n";
					}
					else {
						printf("The robot will not move to the configuration\n");
					}
				}
				else {
					printf("Try again\n");
				}
			}

			else if (ch == '3')
			{
				//MoveToConfiguration(q1);
				//DisplayConfiguration(q1);
				bool sol = false;
				ROBSIM::q_vec near, far;

				current_q = robot.current_config();
				robot.setQ(current_q);

				std::cout << "Please enter the pose of the object or desired location of object\nx\ty\tz\tphi\n" << std::flush;
				JOINT pose{ 100, 0, -100, 0 };
				for (int i = 0; i < 4; ++i) {
					std::cin >> pose[i];
				}
				printf("You entered the following pose values\n");
				ROBSIM::vec4 pose_input(pose[0], pose[1], pose[2], pose[3]);
				pose_input.display();

				robot.SOLVE(pose_input, current_q, near, far, sol);

				if (sol) {
					printf("Would you like the robot to move to this configuration above the object\n");
					printf("Press 'y' for yes and 'n' for no\n");
					ch = _getch();
					if (ch == 'y') {
						std::cout << "Robot Moving to the configuration above the object ...\n" << std::flush;
						
						JOINT near_joint;
						near.toJOINT(near_joint);
						
						JOINT near_joint_up = { near_joint[0], near_joint[1], -199, near_joint[3] };

						MoveToConfiguration(near_joint_up);
						
						printf("Would you like the robot to move to the desired location of object\n");
						std::cout << "Press 'y' for yes and 'n' for no\n" << std::flush;
						ch = _getch();
						if (ch == 'y') {
							MoveToConfiguration(near_joint);
						}
						else {
							printf("The robot will not move to the configuration\n");
						}
					}
					else {
						printf("The robot will not move to the configuration\n");
					}
				}
				else {
					printf("Try again\n");
					continue;
				}
			}
			else if (ch == 'c') { //clear
				system("cls");
			}
			else if (ch == 'g') { //grasp
				Grasp(true);
			}
			else if (ch == 'u') { //ungrasp
				Grasp(false);
			}
			else if (ch == 'r') { //reset robot
				StopRobot();
				ResetRobot();
				printf("Robot has been reset\n");
			}
			else if (ch == 'd') { //display configuration
				std::cout << "Robot is at the following configuration\n";
				robot.getQ().display();
			}
			else
			{
				system("cls");
				printf("Invalid input; Choose from options below \n");
			}
			//printf("Press any key to continue \n");
			//printf("Press q to exit \n");
			//c = _getch();
		}
		else
			break;
	}
	return 0;
}
