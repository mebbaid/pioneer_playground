#include<extApi.h>
#include<math.h>
#include<Eigen/Dense>



class pioneer_p3dx
{
public:
	simxInt m_clientID;
	const simxChar* m_connAddress = "127.0.0.1";
	const simxInt m_connPort = 20000;
	const simxInt m_conntimeout = 5000;
	const simxInt m_connThreadcycle = 5;
	const simxFloat m_simPeriod = 0.005; // vrep sim period 
	//const simxFloat m_intPeriod = 1e-04;
	simxInt m_robotHandle;
	simxInt m_pathHandle;
	simxInt m_leftMotorHandle;
	simxInt m_rightMotorHandle;
	const simxFloat m_leftMotorinitVelocity = 0;
	const simxFloat m_rightMotorinitVelocity = 0;
	const simxFloat m_robotinitPosition[3] = { +1.5500e+00,+6.5000e-01,+1.3879e-01 };  // TODO: initial z coordinates can be retrieved as well
	const simxFloat m_robotinitOrientation[3] = { 0,0,-8.6057e+01 };
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
	simxFloat m_pi_des[2];
	simxFloat m_zeta ;  // new state for dyn. extension
	Eigen::Matrix<float, 2, 2> m_D; //decoupling matrix and it's inverse
	Eigen::Matrix<float, 2, 2> m_Di;
	Eigen::Vector2f m_lf2;
	//Eigen::VectorXf m_zetad;

	bool configure();	// establish the connection and retrieve handles
	bool update_fblinearization(const float b);		// main function updating robot state
	bool update_tfl();
	bool update_dtfl();
	simxFloat* controlTx(float u1, float u2, float b, float l, float r) ;  // To convert (u,w) -> (u1,u2)
	simxFloat* controlTx_tfl(float u1, float u2, float l, float r) ;
	simxFloat* controlTx_dtfl(float u1, float u2, float l, float r) ;
	bool move(simxFloat m_desiredrightvelocity, simxFloat m_desiredleftvelocity);
	//float* statesTx(float b, float x, float y, float theta);
	bool close();		// close connection and simulation
};
