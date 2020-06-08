// pioneer_p3dx.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <chrono>
#include <thread>
#include "pioneer_p3dx.h"

enum string_code {
	    TFL,
	    DTFL,
};


string_code hashit (std::string const& inString) {
	    if (inString == "TFL") return TFL;
	    if (inString == "DTFL") return DTFL;
}

int main()
{
    //std::chrono::seconds dura(5);
    std::cout << "Hello World!\n";
    float * control;
    float* states;
    float u1, u2, v1, v2;   //initial controls
    time_t  simTime;
    float b,  k1 = 2, k2 = 3 ;
    float e1[2] ; 
    std::string controller;
    std::cout << "Please insert simulation duration time in seconds\n"; 
    std::cin >> simTime ;
    std::cout << "Controller type: <TFL/DTFL>\n";
    std::getline(std::cin, controller) ;
    if (!std::getline(std::cin, controller)) { 
	std::cout << "WARNING: Default feedback linearization controller will be used" << std::endl;
	std::cout << "Please insert b [suggestion 0.2]" << std::endl;
	std::cin >> b ;
    }
    std::cout << "Controller is: " << controller << "\n";

    pioneer_p3dx myPioneer;
    if (myPioneer.configure()) {
        std::cout << "Connection established with vrep" << std::endl;
        std::cout << "Simulation Started" << std::endl;
        time_t start = time(0);  // Make the start time absolute and outside the loop
        int    timeLeft = simTime;       // timeLeft is a relative value that can be negative => `int`
        simxSynchronousTrigger(myPioneer.m_clientID);
	
	    switch (hashit(controller)) {
			case TFL : { 
			 std::cout << "Simulating TFL over a circle" << std::endl; 
			 while (timeLeft > 0 )
				   {
				    time_t end = time(0);
				    time_t timeTaken = end - start;
				    myPioneer.update_tfl();
				    u1 = 0.2;
				    u2 = -k2 * myPioneer.m_alpha[0] - k1 * myPioneer.m_alpha[1];
				    control = myPioneer.controlTx_tfl(u1,u2);
				    myPioneer.move(*(control), *(control+1));
				    timeLeft = simTime - timeTaken ;
				    simxSynchronousTrigger(myPioneer.m_clientID);	
			 	   }
			  	myPioneer.close();
			  } 
			
			break ;

			case DTFL: { 
			  std::cout << "Simulating DTFL over a circle" << std::endl;	
			  while (timeLeft > 0 )
				   {
				    time_t end = time(0);
				    time_t timeTaken = end - start;
				    myPioneer.update_dtfl();
				    v1 = -k2 * myPioneer.m_alpha[0] - k1 * myPioneer.m_alpha[1];
				    v2 = -k1 * (myPioneer.m_pi[1]-0.2); // assign a velocity on the path
				    u1 = myPioneer.m_Di[0][0]*(v1- myPioneer.m_lf2[0]) + myPioneer.m_Di[0][1]	* (v2 - myPioneer.m_lf2[1]) ; 
				    u1 = myPioneer.m_Di[1][0]*(v1- myPioneer.m_lf2[0]) + myPioneer.m_Di[1][1]	* (v2 - myPioneer.m_lf2[1]) ;  
				    control = myPioneer.controlTx_dtfl(u1,u2);
				    myPioneer.move(*(control), *(control+1));
				    timeLeft = simTime - timeTaken ;
				    simxSynchronousTrigger(myPioneer.m_clientID);	
			 	   }
			  	myPioneer.close();	    
			} 
			break ;

			default : {
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
	    }
    }	      
    return 0;
}
