// ProgrammingDemo.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <stdio.h>
#include <conio.h>
#include <iostream>
#include <string>
#include <ctime>

#include "helper.h"
//#include "ensc-488.h"

using namespace std;
using namespace ROBSIM;


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

	q_vec last_q_in_stage;
	vec4 last_pose_in_stage;

	system("cls");

	while (1)
	{	
		printf("=======================================================\n");
		printf("Press 'c': clear, 'r': reset, 'g': grasp, 'u': ungrasp\n");
		printf("Press '1' to move to a joint configuration \n");
		printf("Press '2' to move to a pose \n");
		//printf("Press '3' to pick and place an object \n");
		printf("Press '3' to use trajectory planner \n");
		printf("Press ESC to exit \n");
		printf("-------------------------------------------------------\n");
		std::cout << "user input: " << std::flush;
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
					//printf("trels: \n");
					//trels.matrix.display();

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
				last_pose_in_stage = pose_input;

				robot.SOLVE(pose_input, current_q, near, far, sol);
				last_q_in_stage = near;

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
			else if (ch == '3') {
				current_q = robot.current_config();
				robot.setQ(current_q);

				//vec4 p0 = { 230, 220, 120, 10 };
				////auto q0 = robot.current_config();
				////robot.setQ(q0);
				//q_vec q0 = { 59.99, -38.88, -190, 11.11 };
				//JOINT q0_joint;
				//q0.toJOINT(q0_joint);
				//MoveToConfiguration(q0_joint);
				//std::this_thread::sleep_for(std::chrono::seconds(8));

				std::cout << "Please enter the via points and the goal pose" << std::endl << std::flush;
				JOINT pose[4];
				for (int i = 0; i < 4; ++i) {
					if (i == 3)
						std::cout << "Goal Point:\tx   y   z   phi\n\t\t" << std::flush;
					else
						std::cout << "Via Point " << i + 1 << ":\tx   y   z   phi\n\t\t" << std::flush;
					for (int j = 0; j < 4; ++j) {
						std::cin >> pose[i][j];
					}
				}

				ROBSIMDouble total_duration;
				std::cout << "Please enter the total duration for the trajectory: " << std::flush;
				std::cin >> total_duration;

				ROBSIMDouble t0 = 0;
				ROBSIMDouble tf = total_duration;
				ROBSIMDouble num_samples = 1000;

				std::vector<vec4> pose_vec;
				pose_vec.push_back(last_pose_in_stage);

				printf("You entered the following pose values:\n");
				for (int i = 0; i < 4; ++i) {
					vec4 pose_input(pose[i][0], pose[i][1], pose[i][2], pose[i][3]);
					pose_vec.push_back(pose_input);
					pose_input.display();
				}
				std::cout << "Total duration: " << total_duration << std::endl;

				current_q = robot.current_config();
				robot.setQ(current_q);


				//vec4 p0 = { 230, 220, 120, 10 };
				//vec4 p1 = { 230, 100, 100, 10 };
				//vec4 p2 = { 230, 0, 80, 10 };
				//vec4 p3 = { 230, -100, 60, 10 };
				//vec4 pf = { 230, -220, 30, 10 };
				q_vec q1, q2, q3, qf;
				//auto sol_init = robot.SOLVE(p0, p1, p2, p3, pf, q0, q1, q2, q3, qf);
				auto sol_init = robot.SOLVE(pose_vec[0], pose_vec[1], pose_vec[2], pose_vec[3], pose_vec[4], current_q, q1, q2, q3, qf);

				if (sol_init) {
					printf("Solutions found for via points\n");
					printf("Would you like the robot to move through these configurations\n");
					printf("[y/n] Press 'y' for yes and 'n' for no\n");
					ch = _getch();
					if (ch == 'y') {
						robot.setQ(current_q);
						robot.create_trajectory_planner(current_q, q1, q2, q3, qf, t0, tf, num_samples, true);
					}
					else {
						printf("The robot will not move through the configuration\n");
						printf("Trajectory planner will be created but not executed\n");
						robot.setQ(current_q);
						robot.create_trajectory_planner(current_q, q1, q2, q3, qf, t0, tf, num_samples, false);
					}
				}
				else {
					printf("No solution found for via points\n");
				}
			}
			
			else if (ch == '4') { // used for testing
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

				ROBSIM::q_vec near, far;

				robot.SOLVE(trels_vec, current_q, near, far, sol);
			}
			else if (ch == '9')
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
			else if (ch == '5') {
				/*q_vec q0 = { 32.00, 30.00, -120, 63.00 };
				q_vec q1 = { -6.57, 81.76, -120, 75.20 };
				q_vec q2 = { -37.96, 95.48, -120, 57.52};
				q_vec q3 = { -59.01, 81.67, -120, 21.95 };
				q_vec qf = { -57.88, 30.71, -120, -27.69 };*/

				q_vec q0 = { 32.00, 30.00, -120, 63.00 };
				q_vec q1 = { -6.57, 81.76, -120, 75.20 };
				q_vec q2 = { -37.96, 95.48, -120, 57.52 };
				q_vec q3 = { -59.01, 81.67, -120, 21.95 };
				q_vec qf = { -57.88, 30.71, -120, -27.69 };

				robot.setQ(q0);

				//time_t t0 = time(0);
				//auto tf = t0 + 20;
				ROBSIMDouble t0 = 0;
				ROBSIMDouble tf = 10;
				ROBSIMDouble num_samples = 1000;

				printf("Creating trajectory planner\n");
				robot.create_trajectory_planner(q0, q1, q2, q3, qf, t0, tf, num_samples);
			}
			else if (ch == '6') {
				vec4 p0 = { 230, 220, 120, 10 };
				vec4 p1 = { 230, 100, 100, 10 };
				vec4 p2 = { 230, 0, 80, 10 };
				vec4 p3 = { 230, -100, 60, 10 };
				vec4 pf = { 230, -220, 30, 10 };

				q_vec q0 = { 59.99, -38.88, -190, 11.11 };
				q_vec q1, q2, q3, qf;

				robot.setQ(q0);

				robot.SOLVE(p0, p1, p2, p3, pf, q0, q1, q2, q3, qf);

				//time_t t0 = time(0);
				//auto tf = t0 + 20;
				ROBSIMDouble t0 = 0;
				ROBSIMDouble tf = 45;
				ROBSIMDouble num_samples = 1000;

				robot.create_trajectory_planner(q0, q1, q2, q3, qf, t0, tf, num_samples);
			}
			else if (ch == '7') {
				vec4 p0 = { 230, 220, 120, 10 };
				//auto q0 = robot.current_config();
				//robot.setQ(q0);
				q_vec q0 = { 59.99, -38.88, -190, 11.11 };
				JOINT q0_joint;
				q0.toJOINT(q0_joint);
				MoveToConfiguration(q0_joint);
				std::this_thread::sleep_for(std::chrono::seconds(8));

				vec4 p1 = { 230, 100, 100, 10 };
				vec4 p2 = { 230, 0, 80, 10 };
				vec4 p3 = { 230, -100, 60, 10 };
				vec4 pf = { 230, -220, 30, 10 };
				q_vec q1, q2, q3, qf;

				auto sol_init = robot.SOLVE(p0, p1, p2, p3, pf, q0, q1, q2, q3, qf);

				if (sol_init) {
					robot.setQ(q0);

					ROBSIMDouble t0 = 0;
					ROBSIMDouble tf = 30;
					ROBSIMDouble num_samples = 1000;

					robot.create_trajectory_planner(q0, q1, q2, q3, qf, t0, tf, num_samples, true);
				}
				else {
					//printf("No solution found\n");
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
				frame trels;
				auto q_cur = robot.current_config();
				q_cur.display();
				robot.WHERE(q_cur, trels).display();
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
