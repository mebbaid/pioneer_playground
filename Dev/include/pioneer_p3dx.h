#include<extApi.h>
#include<math.h>
class pioneer_p3dx
{
public:
	simxInt m_clientID;
	const simxChar* m_connAddress = "127.0.0.1";
	const simxInt m_connPort = 20000;
	const simxInt m_conntimeout = 5000;
	const simxInt m_connThreadcycle = 5;
	const simxFloat m_simPeriod = 5; // vrep sim period ms
	simxInt m_robotHandle;
	simxInt m_pathHandle;
	simxInt m_leftMotorHandle;
	simxInt m_rightMotorHandle;
	const simxFloat m_leftMotorinitVelocity = 0;
	const simxFloat m_rightMotorinitVelocity = 0;
	const simxFloat m_robotinitPosition[3] = { +1.3000e+00,-7.0000e-01,+1.3879e-01};  // TODO: initial z coordinates can be retrieved as well
	const simxFloat m_robotinitOrientation[3] = {0,0,-8.6057e+01 };
	simxFloat m_robotPosition[3];
	simxFloat m_pathPosition[3];
	simxFloat m_pathlinVelocity[3];
	simxFloat m_pathAngVelocity[3];
	simxFloat m_robotOrientation[3];
	simxFloat m_robotlinVelocity[3];
	simxFloat m_robotAngVelocity[3];
	simxFloat m_control[2];
	simxFloat m_desiredleftvelocity;
	simxFloat m_desiredrightvelocity;
	simxFloat m_alpha[2];  // dummy output corresponding to path
	simxFloat m_pi[2];    // dummy output on the path
	simxFloat m_lf2[2];   // 2nd lie derivative of h to be compensated by the dtfl feedback 
	simxFloat m_zeta;  // new state for dyn. extension
	simxFloat m_Di [2][2] ; //inverse of decoupling matrix


	bool configure();	// establish the connection and retrieve handles
	bool update_fblinearization(const float b);		// main function updating robot state
        bool update_tfl();
	bool update_dtfl();
	simxFloat* controlTx(float u1, float u2, const float b);  // To convert (u,w) -> (u1,u2)
	simxFloat* controlTx_tfl(float u1, float u2);  
	simxFloat* controlTx_dtfl(float u1, float u2); 
	bool move(simxFloat m_desiredrightvelocity, simxFloat m_desiredleftvelocity);
	//float* statesTx(float b, float x, float y, float theta);
	bool close();		// close connection and simulation
};

