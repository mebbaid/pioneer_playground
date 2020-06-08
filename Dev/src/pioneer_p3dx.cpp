#include <iostream>
#include "pioneer_p3dx.h"

bool pioneer_p3dx::configure() {
	simxFinish(-1);
	this ->m_clientID = simxStart(m_connAddress, m_connPort, true, true, m_conntimeout,m_connThreadcycle);
	if (this->m_clientID == -1) {
		std::cout << "[CONFIG] Connection Error" << std::endl;
		return false;
	}
	if (simxGetObjectHandle(m_clientID, "Pioneer_p3dx", &m_robotHandle, simx_opmode_blocking) != simx_return_ok) {
		std::cout << "[CONFIG] Error retrieving robot handle" << std::endl;
		return false;
	}
	if (simxGetObjectHandle(m_clientID, "target", &m_pathHandle, simx_opmode_blocking) != simx_return_ok) {
		std::cout << "[CONFIG] Error retrieving path handle" << std::endl;
		return false;
	}
	if (simxGetObjectHandle(m_clientID, "Pioneer_p3dx_leftMotor", &m_leftMotorHandle, simx_opmode_blocking) != simx_return_ok) {
		std::cout << "[CONFIG] Error retrieving left motor handle" << std::endl;
		return false;
	}
	if (simxGetObjectHandle(m_clientID, "Pioneer_p3dx_rightMotor", &m_rightMotorHandle, simx_opmode_blocking) != simx_return_ok) {
		std::cout << "[CONFIG] Error retrieving right motor handle" << std::endl;
		return false;
	}
	if (simxSetObjectPosition(m_clientID, m_robotHandle, -1, m_robotinitPosition, simx_opmode_oneshot_wait) != simx_return_ok) {
		std::cout << "[CONFIG] Failed to set robot initial positon" << std::endl;
		return false;
	}
	if (simxSetObjectOrientation(m_clientID, m_robotHandle, -1, m_robotinitOrientation, simx_opmode_oneshot_wait) != simx_return_ok) {
		std::cout << "[CONFIG] Failed to set robot initial orientation" << std::endl;
		return false;
	}
	if (simxSetJointTargetVelocity(m_clientID, m_leftMotorHandle, m_leftMotorinitVelocity, simx_opmode_oneshot_wait) != simx_return_ok) {
		std::cout << "[CONFIG] Failed to set left motor initial velocity" << std::endl;
		return false;
	}
	if (simxSetJointTargetVelocity(m_clientID, m_rightMotorHandle, m_rightMotorinitVelocity, simx_opmode_oneshot_wait) != simx_return_ok) {
		std::cout << "[CONFIG] Failed to set right motor initial velocity" << std::endl;
		return false;
	}
	if (simxGetObjectPosition(m_clientID, m_robotHandle, -1, m_robotPosition, simx_opmode_streaming) != simx_return_ok) {
		std::cout << "[CONFIG] Error retrieving robot position" << std::endl;
		//return false;
	}
	if (simxGetObjectPosition(m_clientID, m_pathHandle, -1, m_pathPosition, simx_opmode_streaming) != simx_return_ok) {
		std::cout << "[CONFIG] Error retrieving path position" << std::endl;
		//return false;
	}
	if (simxGetObjectOrientation(m_clientID, m_robotHandle, -1, m_robotOrientation, simx_opmode_streaming) != simx_return_ok) {
		std::cout << "[CONFIG] Error retrieving robot orientation" << std::endl;
		//return false;
	}	
	if (simxSynchronous(m_clientID, 1) != simx_return_ok) {
		std::cout << "[CONFIG] unable to start synchornous mode " << std::endl;
		return false;
	 }
	if (simxStartSimulation(m_clientID, simx_opmode_oneshot_wait) != simx_return_ok) {
		std::cout << "[CONFIG] Failed to start simulation" << std::endl;
		return false;
	}
	return true;
}
// TODO update module
bool pioneer_p3dx::update_fblinearization(const float b) {

	if (simxGetObjectPosition(m_clientID, m_robotHandle, -1, m_robotPosition, simx_opmode_buffer) != simx_return_ok) {
		std::cout << "[UPDATE] robot position not retireved" << std::endl;
		return false;
	}
	if (simxGetObjectOrientation(m_clientID, m_robotHandle, -1, m_robotOrientation, simx_opmode_buffer) != simx_return_ok) {
		std::cout << "[UPDATE] robot orientation not retireved" << std::endl;
		return false;
	}
	if (simxGetObjectPosition(m_clientID, m_pathHandle, -1, m_pathPosition, simx_opmode_buffer) != simx_return_ok) {
		std::cout << "[UPDATE] path position not retireved" << std::endl;
		return false;
	}
	simxGetObjectVelocity(m_clientID, m_robotHandle, m_robotlinVelocity, m_robotAngVelocity, simx_opmode_oneshot_wait);
	simxGetObjectVelocity(m_clientID, m_pathHandle, m_pathlinVelocity , m_pathAngVelocity, simx_opmode_oneshot_wait);
	m_robotPosition[0] = m_robotPosition[0] + b * cos(m_robotOrientation[2]);  // for fb_linearization
	m_robotPosition[1] = m_robotPosition[1] + b * sin(m_robotOrientation[2]); 
	//std::cout << "[UPDATE] robot position " << m_robotPosition[0] << "," << m_robotPosition[1] << std::endl;
	//std::cout << "[UPDATE] target position " << m_pathPosition[0] << "," << m_pathPosition[1] << std::endl;
	return true;
}

float* pioneer_p3dx::controlTx(float u1, float u2, const float b) {
	float m_theta = m_robotOrientation[2];
	std::cout << "[controlTX] robot orientation " << m_robotOrientation[2]<< std::endl;
	float v = cos(m_theta) * u1 + sin(m_theta) * u2;
	float w = -(u1 / b )*sin(m_theta)  + (u2 / b)*cos(m_theta) ;
	//float dw = (w - m_robotAngVelocity[2]) / m_simPeriod;
	const float l = 0.35, r = 0.1;
	m_control[0] = (v + 0.5*l*w)/r; // l = 0.35 is the distance between the wheels, r is radius
	m_control[1] = (v - 0.5*l*w)/r;

	return m_control;
}

bool pioneer_p3dx::move(simxFloat  m_desiredrightvelocity, simxFloat m_desiredleftvelocity) {
	if (simxSetJointTargetVelocity(m_clientID, m_leftMotorHandle, m_desiredleftvelocity, simx_opmode_oneshot_wait) != simx_return_ok) {
		std::cout << "[MOVE] unable to move the left motor" << std::endl;
		return false;
	}
	if (simxSetJointTargetVelocity(m_clientID, m_rightMotorHandle, m_desiredrightvelocity, simx_opmode_oneshot_wait) != simx_return_ok) {
		std::cout << "[MOVE] unable to move the right motor" << std::endl;
		return false;
	}
	return true;
}

bool pioneer_p3dx::close() {
	if (simxStopSimulation(m_clientID, simx_opmode_blocking) != simx_return_ok) {
		std::cout << "[CLOSE] unable to stop simulation" << std::endl;
		return false;
	}
	simxFinish(m_clientID);
	return true;
}
