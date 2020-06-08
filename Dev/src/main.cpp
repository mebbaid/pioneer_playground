// pioneer_p3dx.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <chrono>
#include <thread>
#include "pioneer_p3dx.h"


int main()
{
    //std::chrono::seconds dura(5);
    std::cout << "Hello World!\n";
    float * control;
    float* states;
    float u1, u2;   //initial controls
    time_t  simTime = 200;
    float b = 0.2 ,  k1 = 2, k2 = 3 ;
    float e1[2] ;

    pioneer_p3dx myPioneer;
    if (myPioneer.configure()) {
        std::cout << "Connection established with vrep" << std::endl;
        std::cout << "Simulation Started" << std::endl;
        time_t start = time(0);  // Make the start time absolute and outside the loop
        int    timeLeft = simTime;       // timeLeft is a relative value that can be negative => `int`
        simxSynchronousTrigger(myPioneer.m_clientID);
        while (timeLeft > 0 )
        {
            time_t end = time(0);
            time_t timeTaken = end - start;
            myPioneer.update_fblinearization(b);
            e1[0] =  myPioneer.m_robotPosition[0] - myPioneer.m_pathPosition[0] ;
            e1[1] =  myPioneer.m_robotPosition[1] - myPioneer.m_pathPosition[1] ;
            u1 = k1*(myPioneer.m_pathlinVelocity[0] - myPioneer.m_robotlinVelocity[0]) - k1 * e1[0];
            u2 = k2*(myPioneer.m_pathlinVelocity[1] - myPioneer.m_robotlinVelocity[1])  - k2 * e1[1];
            control = myPioneer.controlTx(u1, u2, b);
            myPioneer.move(*(control), *(control + 1));
            //myPioneer.move(u2, u1);
            timeLeft = simTime - timeTaken;
            simxSynchronousTrigger(myPioneer.m_clientID);
        }
        myPioneer.close();
    }
    return 0;
}
