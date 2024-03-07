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

	while (1)
	{
		ROBSIM::ROBOT robot;
		if (c != ESC)
		{
			printf("Press '1' or '2' \n");
			ch = _getch();

			if (ch == '1')
			{
				/*T_01 = Transformation('r', 'z', q1[0]);*/
				MoveToConfiguration(q1);
				//DisplayConfiguration(q1);
			}
			if (ch == 'd')
			{
				/*T_01 = Transformation('r', 'z', q1[0]);*/
				MoveToConfiguration(q1_d);
				//DisplayConfiguration(q1_d);
			}
			else if (ch == 'k')
			{
				JOINT q1 = { 0, 0, -100, 0 };
				JOINT q2 = { 0, 0, -200, 0 };
				JOINT q3 = { 90, 90, -200, 45 };
				ROBSIM::q_vec q1_vec(q1), q2_vec(q2), q3_vec(q3);
				auto dist = q1_vec.getDistance(q3_vec);
				std::cout << "dist: " << dist << std::endl;
				auto dist2 = ROBSIM::getDistance(q1_vec, q3_vec);
				std::cout << "dist2: " << dist2 << std::endl;

			}
			else if (ch == '2')
			{
				bool sol = false;

				printf("Please enter the joint values \ntheta1 theta2 d3 theta4\n");
				JOINT q_input{100, -100, -100, 90};
				for (int i = 0; i < 4; ++i) {
					std::cin >> q_input[i];
				}



				ROBSIM::q_vec q_vec(q_input);
				ROBSIM::q_vec near, far;
				auto current_q(robot.current_config());
				printf("current_q: \n");
				current_q.display();
				printf("q: \n");
				q_vec.display();
				
				ROBSIM::frame trels;
				auto trels_vec = robot.WHERE(q_vec, trels);
				printf("trels: \n");
				trels.matrix.display();

				printf("trels_vec: \n");
				trels_vec.display();
				
				//robot.SOLVE(trels, current_q, near, far, sol);
				//robot.SOLVE(trels_vec, current_q, near, far, sol);

				//printf("near: \n");
				//near.display();
				//printf("far: \n");
				//far.display();
				if (sol) {
					JOINT near_q;
					near.toJOINT(near_q);
					printf("near_q: ");
					for (int i = 0; i < 4; ++i) {
						std::cout << near_q[i] << " ";
					}
					MoveToConfiguration(near_q);
				}
			}
			else if (ch == '3')
			{
				ROBSIM::vec4 vect1(108, 193, 75, 0);
				ROBSIM::q_vec near, far;
				auto current_q(robot.current_config());

				robot.setQ(current_q);

				bool sol = false;
				robot.SOLVE(vect1, current_q, near, far, sol);
				
				Grasp(false); // Open gripper
				printf("near: \n");
				near.display();
				if (sol) {
					JOINT near_q, near_q_up;
					near.toJOINT(near_q);

					near_q_up[0] = near_q[0];
					near_q_up[1] = near_q[1];
					near_q_up[2] = -200;
					near_q_up[3] = near_q[3];

					MoveToConfiguration(near_q_up);
					MoveToConfiguration(near_q);
					Grasp(true); // Close gripper

				}
			}
			else if (ch == '4')
			{
				vector<ROBSIMDouble> v1 = { 1, 4, -2, 0 };
				ROBSIM::vec4 vect1(v1);
				ROBSIM::frame brela;
				ROBSIM::UTOI(vect1, brela);

				vector<ROBSIMDouble> v2 = { -1, -4, 2, 0 };
				ROBSIM::vec4 vect2(v2);
				ROBSIM::frame crelb;
				ROBSIM::UTOI(vect2, crelb);

				ROBSIM::frame crela;
				ROBSIM::TMULT(brela, crelb, crela);
				crela.matrix.display();
				brela.matrix.display();
				crelb.matrix.display();
			}

			else if (ch == '5')
			{
				JOINT q8 = { 90, 250, -100, 0 };
				auto robot = ROBSIM::ROBOT();
				ROBSIM::q_vec q(q8);
				ROBSIM::frame f;
				//auto q_isOK = robot.check_q_limits(q);
				//std::cout << "q_isOK: " << q_isOK << std::endl;
				robot.setQ(q);
				
				//ROBSIM::KIN(q, f);
				//f.matrix.display();
			}
			else if (ch == '6')
			{
				//JOINT q8 = { 75, 60, -150, 15 };
				JOINT q8 = { -98, 99, -120, -150 };
				auto robot = ROBSIM::ROBOT();
				ROBSIM::q_vec q(q8);
				ROBSIM::q_vec q2, q3;
				bool sol;
				ROBSIM::frame f;
				//auto q_isOK = robot.check_q_limits(q);
				//std::cout << "q_isOK: " << q_isOK << std::endl;
				robot.setQ(q);

				auto current_q = robot.getQ();

				robot.KIN(q, f);
				
				robot.INVKIN(f, current_q, q2, q3, sol);
				//f.matrix.display();
			}
			else if (ch == '7')
			{
				ROBSIM::ROBOT robot;
				JOINT current_config;
				GetConfiguration(current_config);
				ROBSIM::q_vec q1(current_config);
				q1.display();
				std::cout << "robot init" << std::endl;
				robot.current_config().display();

				StopRobot();
				ResetRobot();

				std::cout << "robot reset" << std::endl;
				robot.current_config().display();
			}
			else if (ch == '8')
			{
				bool sol = false;

				printf("Please enter the joint values \ntheta1 theta2 d3 theta4\n");
				JOINT q_input;
				for (int i = 0; i < 4; ++i) {
					std::cin >> q_input[i];
				}

				ROBSIM::q_vec q_vec(q_input);
				ROBSIM::q_vec near, far;
				auto current_q(robot.current_config());
				printf("current_q: \n");
				current_q.display();
				printf("q: \n");
				q_vec.display();

				ROBSIM::frame trels;
				auto trels_vec = robot.WHERE(q_vec, trels);
				printf("trels: \n");
				trels.matrix.display();

				printf("trels_vec: \n");
				trels_vec.display();

				//robot.SOLVE(trels, current_q, near, far, sol);
				robot.SOLVE(trels_vec, current_q, near, far, sol);

				//printf("near: \n");
				//near.display();
				//printf("far: \n");
				//far.display();
				if (sol) {
					JOINT near_q;
					near.toJOINT(near_q);
					printf("near_q: ");
					for (int i = 0; i < 4; ++i) {
						std::cout << near_q[i] << " ";
					}
					MoveToConfiguration(near_q);
				}
				}
			else
			{
				printf("Invalid input \n");
			}
				

			printf("Press any key to continue \n");
			printf("Press q to exit \n");
			c = _getch();
		}
		else
			break;
			
		
	}
	

	return 0;
}
