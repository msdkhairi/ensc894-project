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
	JOINT q2 = {90, 90, -200, 45};
	JOINT q3 = { 0, 0, -200, 0 };
	printf("Keep this window in focus, and...\n");

	

	char ch;
	int c;

	const int ESC = 27;
	
	printf("1Press any key to continue \n");
	printf("2Press ESC to exit \n");

	c = _getch() ;

	while (1)
	{
		
		if (c != ESC)
		{
			printf("Press '1' or '2' \n");
			ch = _getch();

			if (ch == '1')
			{
				/*T_01 = Transformation('r', 'z', q1[0]);*/
				MoveToConfiguration(q1);
				DisplayConfiguration(q1);
			}
			else if (ch == '2')
			{
				MoveToConfiguration(q2);
				//DisplayConfiguration(q2);
			}
			else if (ch == '3')
			{
				vector<ROBSIMDouble> v = { 1, 4, -2, 90 };
				ROBSIM::vec4 v1(v);

				ROBSIM::frame f;
				ROBSIM::UTOI(v1, f);
				f.matrix.display();
				ROBSIM::vec4 v2;
				ROBSIM::ITOU(f, v2);
				v2.display();
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
				ROBSIM::q_vec q(q2);
				ROBSIM::frame f;
				ROBSIM::KIN(q, f);
				f.matrix.display();
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
