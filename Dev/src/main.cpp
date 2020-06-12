// pioneer_p3dx.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <string>
#include <chrono>
#include <thread>
#include "pioneer_p3dx.h"
#include "INIReader.h"

enum string_code {
	TFL,
	DTFL,
	Option_Invalid,
};


string_code hashit(std::string const& inString) {
	if (inString == "TFL") return TFL;
	if (inString == "DTFL") return DTFL;
	return Option_Invalid;
}


int main()
{
	INIReader reader("conf.ini");
    	if (reader.ParseError() != 0) {
		std::cout << "Can't load conf.ini'\n";
		return 1;
    	}
	std::cout << "Hello World!\n";
	float* control;
	float* states;
	float u1, u2, v1, v2;   //initial controls
	time_t  simTime;
	float b;
	float k1 = reader.GetInteger("user", "k1", -1), k2 = reader.GetInteger("user", "k2", -1);
	float l = reader.GetReal("pioneer", "length", -1), r = reader.GetReal("pioneer", "radius", -1) ;
	float e1[2];
	std::string controller;
	std::cout << "Please insert simulation duration time in seconds\n";
	std::cin >> simTime;
	std::cout << "Controller type: <FL/TFL/DTFL>\n";
	std::getline(std::cin, controller);
	if (!std::getline(std::cin, controller)) {
		std::cout << "[WARNING] Default feedback linearization controller will be used" << std::endl;
	}
	pioneer_p3dx myPioneer;
	if (myPioneer.configure()) {
		std::cout << "Connection established with vrep" << std::endl;
		std::cout << "Simulation Started" << std::endl;
		time_t start = time(0);  // Make the start time absolute and outside the loop
		int    timeLeft = simTime;       // timeLeft is a relative value that can be negative => `int`
		simxSynchronousTrigger(myPioneer.m_clientID);

		switch (hashit(controller)) {
		case TFL: {
			std::cout << "Simulating TFL over a circle" << std::endl;
			while (timeLeft > 0)
			{
				simxSynchronousTrigger(myPioneer.m_clientID);
				time_t end = time(0);
				time_t timeTaken = end - start;
				myPioneer.update_tfl();
				u1 = 0.2;
				u2 = -k2 * myPioneer.m_alpha[0] - k1 * myPioneer.m_alpha[1];
				control = myPioneer.controlTx_tfl(u1, u2, l, r);
				myPioneer.move(*(control), *(control + 1));
				timeLeft = simTime - timeTaken;
			} break;
		}

		case DTFL: {
			std::cout << "Simulating DTFL over a circle" << std::endl;
			Eigen::Vector2f b; // performing a least square solution to find inverse of D
			Eigen::Vector2f cnt;
			while (timeLeft > 0)
			{
				simxSynchronousTrigger(myPioneer.m_clientID);
				time_t end = time(0);
				time_t timeTaken = end - start;
				myPioneer.update_dtfl();
				v1 = - k2 * myPioneer.m_alpha[0] - k1 * myPioneer.m_alpha[1];
				v2 = - k1 * (myPioneer.m_pi[1] - myPioneer.m_pi_des[1]);
				b << v1, v2;
				myPioneer.m_Di = myPioneer.m_D.completeOrthogonalDecomposition().pseudoInverse();
				cnt = myPioneer.m_Di * (b - myPioneer.m_lf2);
				u1 = cnt(0);
				u2 = cnt(1);
				control = myPioneer.controlTx_dtfl(u1, u2, l ,r);
				myPioneer.move(*(control), *(control + 1));
				timeLeft = simTime - timeTaken;
			}   break;
		}

		default: {
			std::cout << "[DEFAULT] Please insert b [suggestion 0.2]" << std::endl;
			std::cin >> b;
			while (timeLeft > 0)
			{
				simxSynchronousTrigger(myPioneer.m_clientID);
				time_t end = time(0);
				time_t timeTaken = end - start;
				myPioneer.update_fblinearization(b);
				e1[0] = myPioneer.m_robotPosition[0] - myPioneer.m_pathPosition[0];
				e1[1] = myPioneer.m_robotPosition[1] - myPioneer.m_pathPosition[1];
				u1 = k1 * (myPioneer.m_pathlinVelocity[0] - myPioneer.m_robotlinVelocity[0]) - k1 * e1[0];
				u2 = k2 * (myPioneer.m_pathlinVelocity[1] - myPioneer.m_robotlinVelocity[1]) - k2 * e1[1];
				control = myPioneer.controlTx(u1, u2, b, l, r);
				myPioneer.move(*(control), *(control + 1));
				timeLeft = simTime - timeTaken;
			} break;
		}
		}
		myPioneer.close();
	}
	return 0;
}
