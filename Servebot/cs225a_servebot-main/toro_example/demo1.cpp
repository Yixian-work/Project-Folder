/**
 * @file demo1.cpp
 * @brief Toro controller 
 * 
 */

#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "Sai2Primitives.h"

#include <unistd.h>
#include <iostream>
#include <fstream>
#include <string>

#include <signal.h>

// Demo State Machine
# define USER_SELECT_LOCATION 0
# define USER_SELECT_POSE 1
# define PREPARE 2
# define SERVE_FIRST 3
# define SERVE_SECOND 4
# define RETURN 5

// Camera state Machine
# define ROBOT_BACK_VIEW 10 // (-14,-3.25,1.0),(0,0,2),(-6.5,-1.75,0.0)
# define SELECTION_VIEW 11 // (-2.10637, 0,0, 5.80117),(0, 0, 1), (-1.47045, 0.0, 5.02942)
# define OPPONENT_VIEW 12 // (11.7499, 3.44443, 0.679269),(0,0,1),(10.7815,3.20929,0.59578)
# define ROBOT_SIDE_VIEW 13 // (-10.9529,-5.49318,0.0740845),(0.0,1),(-10.9679,-4.49507,0.0145338)
# define ROBOT_FRONT_VIEW 14 // (-7.41958,-2.07966,0.194219),(0,0,1),(-8.41482,-2.05996,0.0987788)
# define COURT_VIEW 15 // (0.0,-13.8237,11.364),(0,0,1),(0.0,-13.0598,10.7186)

bool runloop = true;
void sighandler(int sig)
{ runloop = false; }

using namespace std;
using namespace Eigen;

const string robot_file = "./resources/toro.urdf";	

// enum Control
// {
// 	JOINT_CONTROLLER = 0, 
// 	POSORI_CONTROLLER
// };

#include "redis_keys.h"

unsigned long long controller_counter = 0;

int main() {
	// BOOLION of serving gesture (true: upper serve, false: under serve)
	bool pose = true;

	// int state = JOINT_CONTROLLER;
	int state = USER_SELECT_LOCATION;
	string controller_status = "1";

	// Camera State Machine
	int camera_state = ROBOT_BACK_VIEW;

	// start redis client
	auto redis_client = RedisClient();
	redis_client.connect();

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load robots
	auto robot = new Sai2Model::Sai2Model(robot_file, false);
	robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
	robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);
	robot->updateModel();	

	// prepare controller
	int dof = robot->dof();
	VectorXd command_torques = VectorXd::Zero(dof);
	MatrixXd N_prec = MatrixXd::Identity(dof, dof);

	// Define these useful parameters
	Vector3d x_pos_RL, x_pos_LL, x_pos_RA, x_pos_LA, desired_landing_pos;
	Matrix3d x_ori_RL, x_ori_LL, x_ori_RA, x_ori_LA;

	// Initialize desired_landing_pos
	desired_landing_pos << 6.0, -1.5, -1.15;

	// Define turning angle
	double x_prime = 6.5;
	double y_prime = 2.0;
	double turn_rad = 0.20;

	// pose task for right foot (fixed)
	string control_link_RL = "RL_foot";
	Vector3d control_point_RL = Vector3d(0, 0, 0);
	auto posori_task_right_foot = new Sai2Primitives::PosOriTask(robot, control_link_RL, control_point_RL);

	posori_task_right_foot->_use_interpolation_flag = false;
	posori_task_right_foot->_use_velocity_saturation_flag = false;
	
	VectorXd posori_task_torques_right_foot = VectorXd::Zero(dof);
	posori_task_right_foot->_kp_pos = 400.0;
	posori_task_right_foot->_kv_pos = 20.0;
	posori_task_right_foot->_kp_ori = 400.0;
	posori_task_right_foot->_kv_ori = 20.0;

	// We tend to fix the right foot all the time	
	robot->positionInWorld(x_pos_RL, control_link_RL, control_point_RL);
	robot->rotationInWorld(x_ori_RL, control_link_RL);
	posori_task_right_foot->_desired_position = x_pos_RL;
	posori_task_right_foot->_desired_orientation = x_ori_RL; 

	// pose task for left foot (fixed)
	string control_link_LL = "LL_foot";
	Vector3d control_point_LL = Vector3d(0, 0, 0);
	auto posori_task_left_foot = new Sai2Primitives::PosOriTask(robot, control_link_LL, control_point_LL);

	posori_task_left_foot->_use_interpolation_flag = false;
	posori_task_left_foot->_use_velocity_saturation_flag = false;

	VectorXd posori_task_torques_left_foot = VectorXd::Zero(dof);
	posori_task_left_foot->_kp_pos = 400.0;
	posori_task_left_foot->_kv_pos = 20.0;
	posori_task_left_foot->_kp_ori = 400.0;
	posori_task_left_foot->_kv_ori = 20.0;

	// We tend to fix the left foot all the time	
	robot->positionInWorld(x_pos_LL, control_link_LL, control_point_LL);
	robot->rotationInWorld(x_ori_LL, control_link_LL);
	posori_task_left_foot->_desired_position = x_pos_LL;
	posori_task_left_foot->_desired_orientation = x_ori_LL; 

	// pose task for right hand 
	string control_link_RA = "ra_link7";
	Vector3d control_point_RA = Vector3d(0, 0, 0.48855);
	auto posori_task_right_hand = new Sai2Primitives::PosOriTask(robot, control_link_RA, control_point_RA);

	posori_task_right_hand->_use_interpolation_flag = false;
	posori_task_right_hand->_use_velocity_saturation_flag = false;
	
	VectorXd posori_task_torques_right_hand = VectorXd::Zero(dof);
	posori_task_right_hand->_kp_pos = 200.0;
	posori_task_right_hand->_kv_pos = 20.0;
	posori_task_right_hand->_kp_ori = 200.0;
	posori_task_right_hand->_kv_ori = 20.0;

	// pose task for left hand
	string control_link_LA = "la_link7";
	Vector3d control_point_LA = Vector3d(0, -0.1, 0);
	auto posori_task_left_hand = new Sai2Primitives::PosOriTask(robot, control_link_LA, control_point_LA);
	posori_task_left_hand->setDynamicDecouplingFull();

	posori_task_left_hand->_use_interpolation_flag = false;
	posori_task_left_hand->_use_velocity_saturation_flag = false;
	
	VectorXd posori_task_torques_left_hand = VectorXd::Zero(dof);
	posori_task_left_hand->_kp_pos = 200.0;
	posori_task_left_hand->_kv_pos = 20.0;
	posori_task_left_hand->_kp_ori = 200.0;
	posori_task_left_hand->_kv_ori = 20.0;

	// print flag
	bool print_flag = false;

	// joint task
	auto joint_task = new Sai2Primitives::JointTask(robot);

	joint_task->_use_interpolation_flag = false;
	joint_task->_use_velocity_saturation_flag = false;

	VectorXd joint_task_torques = VectorXd::Zero(dof);
	joint_task->_kp = 100.0;
	joint_task->_kv = 20.0;

	// initialize desired joint posture to be the initial robot configuration
	VectorXd q_init_desired = robot->_q;  // Robot starting position
	joint_task->_desired_position = q_init_desired;

	// setup redis callback
	redis_client.createReadCallback(0);
	redis_client.createWriteCallback(0);

	// add to read callback
	redis_client.addEigenToReadCallback(0, JOINT_ANGLES_KEY, robot->_q);
	redis_client.addEigenToReadCallback(0, JOINT_VELOCITIES_KEY, robot->_dq);

	// add to write callback
	redis_client.addStringToWriteCallback(0, CONTROLLER_RUNNING_KEY, controller_status);
	redis_client.addEigenToWriteCallback(0, JOINT_TORQUES_COMMANDED_KEY, command_torques);

	// button names and densities
	string button_name, prev_button_name;
	double density, prev_density;
	prev_button_name = "A_button";
	prev_density = 1.0;

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(200); 
	double start_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;
	
	// Initial message 
	cout << endl;
	cout << "Embrace the servebot! You can switch cameras by pressing X!" << endl;
	cout << endl;
	cout << "Select your landing position! (Play it around with left joystick, Press A to proceed)" << endl;
	while (runloop) {
		// wait for next scheduled loop
		timer.waitForNextLoop();
		double time = timer.elapsedTime() - start_time;

		// execute redis read callback
		redis_client.executeReadCallback(0);

		// update model
		robot->updateModel();
		
		// state machine 1: selection process
		if (state == USER_SELECT_LOCATION) {
				// This keeps the robot in place while we select the location
				joint_task->_desired_position = q_init_desired;
				N_prec.setIdentity();
				joint_task->updateTaskModel(N_prec);
				joint_task->computeTorques(joint_task_torques);
				command_torques = joint_task_torques;

				// Let's create a interface to retrieve the desired position from redis
				button_name = redis_client.get("Xbox_input_button");
				density = stod(redis_client.get("Xbox_input_density"));
				// Change desired_landing based on our joystick output
				// if (button_name == "L_Stick_Horizontal") {
				// 		// fix drift
				// 		if (abs(density) >= 0.2) {
				// 				desired_landing_pos(1) -= 0.1 * density;
				// 		}
				// }
				// if (button_name == "L_Stick_Vertical") {
				// 		if (abs(density) >= 0.2) {
				// 			desired_landing_pos(0) += 0.1 * density;
				// 		}
				// }
				desired_landing_pos = redis_client.getEigenMatrixJSON("red_target_box_position");
				// Write that to a key that shown in simulation
				redis_client.setEigenMatrixJSON("updated_pos", desired_landing_pos);

				// Camera state to redis (selection phase)
				redis_client.setEigenMatrixJSON("camera_pos", Vector3d(-2.10637, 0.0, 5.80117));
				redis_client.setEigenMatrixJSON("camera_lookat", Vector3d(-1.47045, 0.0, 5.02942));

				// Will later be changed into redis key writeout
				// cout << "Your selected position: " << desired_landing_pos(0) << " " << desired_landing_pos(1) << endl;
				// Change state if "A button" is pressed
				if (button_name == "A_button") {
						if ((prev_button_name != "A_button") || ((prev_density == 0.0) && (density == 1.0))) {
								state = USER_SELECT_POSE;
								cout << "Select your sereving pose! (B button to switch, A button to proceed)" << endl;
								// Change camera state to robot back view
								camera_state == ROBOT_BACK_VIEW;
								redis_client.setEigenMatrixJSON("camera_pos", Vector3d(-14,-3.25,1.0));
								redis_client.setEigenMatrixJSON("camera_lookat", Vector3d(-6.5,-1.75,0.0));
						}
				} 
		}

		// state machine 2: pose process
		else if (state == USER_SELECT_POSE) {
				// This keeps the robot in place while we select the location
				joint_task->_desired_position = q_init_desired;
				N_prec.setIdentity();
				joint_task->updateTaskModel(N_prec);
				joint_task->computeTorques(joint_task_torques);
				command_torques = joint_task_torques;

				// Let's create a interface to retrieve the button info from redis
				button_name = redis_client.get("Xbox_input_button");
				density = stod(redis_client.get("Xbox_input_density"));

				// Toggle pose based on our B_button pressing
				if (button_name == "B_button") {
						if (prev_button_name != "B_button" || (prev_density == 0.0 && density == 1.0)) {
								pose = !pose;
								cout << "Your serve pose is changed to " << (pose ? "UPPER_SERVE" : "UNDER_SERVE") << endl;
						}
				} 
				
				// Will later be changed into redis key writeout
				// string pose_name = pose ? "UPPER_SERVE" : "UNDER_SERVE";
				// cout << "Your selected pose: " <<  pose_name << endl;
				if (button_name == "A_button") {
						if ((prev_button_name != "A_button") || (prev_density == 0.0 && density == 1.0)) {
								cout << "Transitioning to prepare stage, please wait..." << endl;
								state = PREPARE;
								print_flag = true;
								// Record values here
								if (pose) {
										x_prime = desired_landing_pos(0) - 0.63195 * (desired_landing_pos(1) + 2.15) / sqrt(pow(desired_landing_pos(0) + 11.0, 2) + pow(desired_landing_pos(1) + 2.15, 2));
										y_prime = desired_landing_pos(1) + 0.63195 * (desired_landing_pos(0) + 11.0) / sqrt(pow(desired_landing_pos(0) + 11.0, 2) + pow(desired_landing_pos(1) + 2.15, 2));
										turn_rad = atan((y_prime + 2.15) / (x_prime + 11.0));
								} else {
										// turn_rad = atan((desired_landing_pos(1) + 2.15 + 0.32)/(desired_landing_pos(0) + 11.0 - 0.5)); // Approximation
										x_prime = desired_landing_pos(0) + 11.0 - 0.7724 * sin(51.436 + turn_rad);
										y_prime = desired_landing_pos(1) + 2.15 + 0.7724 * cos(51.436 + turn_rad);
										turn_rad = atan((desired_landing_pos(1) + 2.15 + 0.24)/(desired_landing_pos(0) + 11.0 - 0.72)); // Approximation

										cout << "x_prime: " << x_prime << endl;
										cout << "y_prime: " << y_prime << endl;
								}
						}
				} 
		}

		// state machine 3: prepare stage (joint space control)
		else if (state == PREPARE) {
				// This moves the robot according to joint control scheme
				VectorXd q_new_desired = q_init_desired;

				double conversion_factor = M_PI / 180.0;
    			//   q_init_desired(8) = M_PI / 6.0;
    			//   q_init_desired(13) = M_PI / 6.0;    // Between orange and thigh (left)
    			//   q_init_desired(18) = -45 * conversion_factor;     // Between hip and trunk
        		
				
				q_new_desired(18) = turn_rad;
				
				if (pose) {
						q_new_desired(19) = 45 * conversion_factor;    // Between trunk and white (right)
						q_new_desired(20) = 120 * conversion_factor;    // Between white and shoulder (right)
						q_new_desired(21) = 90 * conversion_factor;    // Between shoulder and upper-arm (right)
						q_new_desired(22) = 60 * conversion_factor;    // Between upper-arm and elbow (right)
						q_new_desired(29) = 135 * conversion_factor;    // Between upper-arm and elbow (left)
						q_new_desired(33) = -30 * conversion_factor;    // Neck horizontal rotation
						q_new_desired(34) = -30 * conversion_factor;    // Neck vertical rotation
						q_new_desired(25) = -20 * conversion_factor;    // Right wrist
				} else {
						q_new_desired(20) = 20 * conversion_factor;    // Between white and shoulder (right)
						q_new_desired(21) = 180 * conversion_factor;    // Between shoulder and upper-arm (right)
						q_new_desired(24) = 20 * conversion_factor;
						q_new_desired(33) = -45 * conversion_factor;    // Neck horizontal rotation
						q_new_desired(34) = 45 * conversion_factor;    // Neck vertical rotation
						q_new_desired(25) = -90 * conversion_factor;    // Right wrist
				}	
				joint_task->_desired_position = q_new_desired;

				N_prec.setIdentity();
				joint_task->updateTaskModel(N_prec);
				joint_task->computeTorques(joint_task_torques);
				command_torques = joint_task_torques;

				// Let's create a interface to retrieve the button info from redis
				button_name = redis_client.get("Xbox_input_button");
				density = stod(redis_client.get("Xbox_input_density"));

				if ((button_name == "A_button") && ((q_new_desired - robot->_q).norm() <= 0.01)) {
						if (print_flag) {
							cout << "Spawn ball and wait for the user to serve (A to proceed)" << endl;
							print_flag = !print_flag;
						}
						if ((prev_button_name != "A_button") || (prev_density == 0.0 && density == 1.0)) {
								redis_client.set("updated_ball_flag", "1");
								Vector3d updated_ball_pos;
								if (pose) {
										updated_ball_pos << -11.0 + 0.63195 * sin(turn_rad) + 0.0435 * cos(turn_rad), -2.15 - 0.63195 * cos(turn_rad) + 0.0435 * sin(turn_rad), 1.355;
								} else {
										// updated_ball_pos << -11.0 + 0.60965 * sin(37.834*conversion_factor + turn_rad) + 0.0435 * cos(37.834*conversion_factor + turn_rad) * sqrt(3) / 2.0, 
										// 					-2.15 - 0.60965 * cos(37.834*conversion_factor + turn_rad) + 0.0435 * sin(37.834*conversion_factor + turn_rad) * sqrt(3)/2.0, 
										// 					-0.616874 + 1.0 / 2.0 * 0.0435;
										updated_ball_pos << -11.0 + (0.7724) * sin(51.436*conversion_factor + turn_rad) + 0.0435 * cos(51.436*conversion_factor + turn_rad) * sqrt(1) / 2.0, 
										 					-2.15 - (0.7724) * cos(51.436*conversion_factor + turn_rad) + 0.0435 * sin(51.436*conversion_factor + turn_rad) * sqrt(1) / 2.0, 
										 					-0.416873 + sqrt(3) / 2.0 * 0.0435;
								}
								redis_client.setEigenMatrixJSON("updated_ball_pos", updated_ball_pos); // Change this value based on calculation
								sleep(1.0); // Pause the program to allow the simviz captures the spawned ball, also used to adjust the exact hitting time.
								state = SERVE_FIRST;
								// Define left hand desired in next phase
								robot->positionInWorld(x_pos_LA, control_link_LA, control_point_LA);
								robot->rotationInWorld(x_ori_LA, control_link_LA);
								posori_task_left_hand->_desired_position = x_pos_LA;
								posori_task_left_hand->_desired_orientation = x_ori_LA; 
								// Define right hand desired in next phase
								robot->positionInWorld(x_pos_RA, control_link_RA, control_point_RA);
								robot->rotationInWorld(x_ori_RA, control_link_RA);
								// posori_task_right_hand->_desired_position = x_pos_RA + Vector3d(0.5, 0.0, 0.0);
								if (pose) {
										posori_task_right_hand->_desired_position = x_pos_RA + Vector3d(0.485*cos(turn_rad), 0.485*sin(turn_rad), 0.16);
										posori_task_right_hand->_desired_orientation = AngleAxisd(180 * conversion_factor + turn_rad, Vector3d::UnitZ()).toRotationMatrix();
										posori_task_right_hand->_desired_velocity = Vector3d((x_prime + 11.0)/(0.7203*6.15), (y_prime + 2.15)/(0.7203*6.15), 0.0);
								} else {
										// posori_task_right_hand->_desired_position = x_pos_RA + Vector3d(0.32*cos(turn_rad), 0.32*sin(turn_rad), 0.15);
										posori_task_right_hand->_desired_position = x_pos_RA + Vector3d(0.55*cos(turn_rad), 0.55*sin(turn_rad), 0.35);
										Matrix3d new_ori;
										new_ori << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;
										// posori_task_right_hand->_desired_orientation = AngleAxisd(180 * conversion_factor + turn_rad, Vector3d::UnitZ()) * AngleAxisd(180 * conversion_factor, Vector3d::UnitX()) * AngleAxisd(-30 * conversion_factor, Vector3d::UnitY()) * new_ori;
										posori_task_right_hand->_desired_orientation = AngleAxisd(180 * conversion_factor + turn_rad, Vector3d::UnitZ()) * AngleAxisd(180 * conversion_factor, Vector3d::UnitX()) * AngleAxisd(-60 * conversion_factor, Vector3d::UnitY()) * new_ori;
										// posori_task_right_hand->_desired_velocity = Vector3d(sqrt(3)/2.0 * cos(turn_rad), sqrt(3)/2.0 * sin(turn_rad), 0.5) * 8.0;
										double temp = pow(x_prime,2) + pow(y_prime,2);
										double h = -0.416873 + sqrt(3) / 2.0 * 0.0435 + 1.15;
										double vz = sqrt((3.0/2.0*9.81*temp)/(sqrt(3*temp)-h));
										cout << "vz: " << vz << endl;
										double dist = sqrt(temp);
										double denom_val = -0.710176834031674*dist + 19.2626589020666;

										cout << "denom_val: " << denom_val << endl;
										denom_val = 2.0;
										posori_task_right_hand->_desired_velocity = Vector3d(1.0/sqrt(3) * cos(turn_rad), 1.0/sqrt(3) * sin(turn_rad), 0.0) * pow(vz, 1) / pow(denom_val, 1);
										// posori_task_right_hand->_desired_velocity = Vector3d(0.0, 0.0, 1.0) * pow(vz, 1) / pow(denom_val, 1);
										// 13.703
								}
								// posori_task_right_hand->_desired_velocity = Vector3d((x_prime + 11.0), (y_prime + 2.15), 0.0);
								// cout << posori_task_right_hand->_desired_velocity << endl;
								print_flag = true;
						}
				}
		}

		// state machine 4: first serving stage (operational space control)
		else if (state == SERVE_FIRST) {
				redis_client.set("updated_ball_flag", "0");
				// This moves the robot according to joint control scheme
				VectorXd q_new_desired = q_init_desired;

				double conversion_factor = M_PI / 180.0;
    			//   q_init_desired(8) = M_PI / 6.0;
    			//   q_init_desired(13) = M_PI / 6.0;    // Between orange and thigh (left)
    			//   q_init_desired(18) = -45 * conversion_factor;     // Between hip and trunk
        		
				q_new_desired(18) = turn_rad;
				
				if (pose) {
						q_new_desired(19) = 45 * conversion_factor;    // Between trunk and white (right)
						q_new_desired(20) = 120 * conversion_factor;    // Between white and shoulder (right)
						q_new_desired(21) = 90 * conversion_factor;    // Between shoulder and upper-arm (right)
						q_new_desired(22) = 60 * conversion_factor;    // Between upper-arm and elbow (right)
						q_new_desired(29) = 135 * conversion_factor;    // Between upper-arm and elbow (left)
						q_new_desired(33) = -30 * conversion_factor;    // Neck horizontal rotation
						q_new_desired(34) = -30 * conversion_factor;    // Neck vertical rotation
						q_new_desired(25) = -20 * conversion_factor;    // Right wrist
				} else {
						q_new_desired(20) = 20 * conversion_factor;    // Between white and shoulder (right)
						q_new_desired(21) = 180 * conversion_factor;    // Between shoulder and upper-arm (right)
						q_new_desired(24) = 20 * conversion_factor;
						q_new_desired(33) = -45 * conversion_factor;    // Neck horizontal rotation
						q_new_desired(34) = 45 * conversion_factor;    // Neck vertical rotation
						q_new_desired(25) = -90 * conversion_factor;    // Right wrist
				}

        		joint_task->_desired_position = q_new_desired;  // This isn't super important

				// calculate torques to move right hand 
				N_prec.setIdentity();
				posori_task_right_hand->updateTaskModel(N_prec);
				posori_task_right_hand->computeTorques(posori_task_torques_right_hand);

				// fix right foot
				N_prec = posori_task_right_hand->_N;
				posori_task_right_foot->updateTaskModel(N_prec);
				posori_task_right_foot->computeTorques(posori_task_torques_right_foot);

				// fix left foot
				N_prec = posori_task_right_foot->_N;
				posori_task_left_foot->updateTaskModel(N_prec);
				posori_task_left_foot->computeTorques(posori_task_torques_left_foot);

				// calculate torques to move left hand
				N_prec = posori_task_left_foot->_N;
				posori_task_left_hand->updateTaskModel(N_prec);
				posori_task_left_hand->computeTorques(posori_task_torques_left_hand);

				// calculate torques to maintain joint posture
				N_prec = posori_task_left_hand->_N;
				joint_task->updateTaskModel(N_prec);
				joint_task->computeTorques(joint_task_torques);

				command_torques = posori_task_torques_right_foot + posori_task_torques_left_foot + posori_task_torques_right_hand + posori_task_torques_left_hand + joint_task_torques;

				// Let's create a interface to retrieve the button info from redis
				button_name = redis_client.get("Xbox_input_button");
				density = stod(redis_client.get("Xbox_input_density"));

				robot->positionInWorld(x_pos_RA, control_link_RA, control_point_RA);
				if (button_name == "A_button") {
						if (print_flag) {
							cout << "The ball is hit! Serve another ball? (A to proceed)" << endl;
							print_flag = !print_flag;
						}
						if ((prev_button_name != "A_button") || (prev_density == 0.0 && density == 1.0)) {
							//state = SERVE_SECOND;
							state = RETURN;
							// // Define left hand desired in next phase
							// robot->positionInWorld(x_pos_LA, control_link_LA, control_point_LA);
							// robot->rotationInWorld(x_ori_LA, control_link_LA);
							// posori_task_left_hand->_desired_position = x_pos_LA;
							// posori_task_left_hand->_desired_orientation = x_ori_LA; 
							// // Define right hand desired in next phase
							// robot->positionInWorld(x_pos_RA, control_link_RA, control_point_RA);
							// robot->rotationInWorld(x_ori_RA, control_link_RA);
							// posori_task_right_hand->_desired_position = x_pos_RA + Vector3d(0.45, 0.0, -0.3);
							// posori_task_right_hand->_desired_orientation = AngleAxisd(52.5 * conversion_factor, Vector3d::UnitY()).toRotationMatrix() * AngleAxisd(0 * conversion_factor, Vector3d::UnitZ()).toRotationMatrix() * x_ori_RA;
							// posori_task_right_hand->_desired_velocity = Vector3d(0.0, 0.0, 0.0);
							// Print transition message
							// cout << "Transitioning to phase 2, operational space control" << endl;
							cout << "Returning..." << endl;
						}
				}
		}

		// // state machine 5: second serving stage (operational space control)
		// else if (state == SERVE_SECOND) {
		// 		// This moves the robot according to joint control scheme
		// 		VectorXd q_new_desired = q_init_desired;

		// 		double conversion_factor = M_PI / 180.0;
    	// 		//   q_init_desired(8) = M_PI / 6.0;
    	// 		//   q_init_desired(13) = M_PI / 6.0;    // Between orange and thigh (left)
    	// 		//   q_init_desired(18) = -45 * conversion_factor;     // Between hip and trunk
        // 		q_new_desired(19) = 60 * conversion_factor;    // Between trunk and white (right)
        // 		q_new_desired(20) = 60 * conversion_factor;    // Between white and shoulder (right)
        // 		q_new_desired(21) = 60 * conversion_factor;    // Between shoulder and upper-arm (right)
        // 		q_new_desired(22) = 135 * conversion_factor;    // Between upper-arm and elbow (right)
       	// 		q_new_desired(29) = 150 * conversion_factor;    // Between upper-arm and elbow (left)
        // 		q_new_desired(33) = -30 * conversion_factor;    // Neck horizontal rotation
        // 		q_new_desired(34) = -30 * conversion_factor;    // Neck vertical rotation

		// 		joint_task->_desired_position = q_new_desired;

		// 		// calculate torques to move right hand 
		// 		N_prec.setIdentity();
		// 		posori_task_right_hand->updateTaskModel(N_prec);
		// 		posori_task_right_hand->computeTorques(posori_task_torques_right_hand);

		// 		// fix right foot
		// 		N_prec = posori_task_right_hand->_N;
		// 		posori_task_right_foot->updateTaskModel(N_prec);
		// 		posori_task_right_foot->computeTorques(posori_task_torques_right_foot);

		// 		// fix left foot
		// 		N_prec = posori_task_right_foot->_N;
		// 		posori_task_left_foot->updateTaskModel(N_prec);
		// 		posori_task_left_foot->computeTorques(posori_task_torques_left_foot);

		// 		// calculate torques to move left hand
		// 		N_prec = posori_task_left_foot->_N;
		// 		posori_task_left_hand->updateTaskModel(N_prec);
		// 		posori_task_left_hand->computeTorques(posori_task_torques_left_hand);

		// 		// calculate torques to maintain joint posture
		// 		N_prec = posori_task_left_hand->_N;
		// 		joint_task->updateTaskModel(N_prec);
		// 		joint_task->computeTorques(joint_task_torques);

		// 		command_torques = posori_task_torques_right_foot + posori_task_torques_left_foot + posori_task_torques_right_hand + posori_task_torques_left_hand + joint_task_torques;

		// 		// Let's create a interface to retrieve the button info from redis
		// 		button_name = redis_client.get("Xbox_input_button");
		// 		density = stod(redis_client.get("Xbox_input_density"));

		// 		robot->positionInWorld(x_pos_RA, control_link_RA, control_point_RA);
		// 		if ((button_name == "A_button") && ((x_pos_RA - posori_task_right_hand->_desired_position).norm() <= 0.01)) {
		// 				cout << "serve another ball? (Press A to continue, X to quit)" << endl;
		// 				if ((prev_button_name != "A_button") || (prev_density == 0.0 && density == 1.0)) {
		// 						state = RETURN;
		// 				}
		// 		}
		// }

		// state machine 6: return stage (joint space control)
		else if (state == RETURN) {
				// This moves the robot according to joint control scheme
				VectorXd q_new_desired = q_init_desired;
				double conversion_factor = M_PI / 180.0;
				if (!pose) {
						q_new_desired(18) = turn_rad;
						q_new_desired(20) = 20 * conversion_factor;    // Between white and shoulder (right)
						q_new_desired(21) = 180 * conversion_factor;    // Between shoulder and upper-arm (right)
						q_new_desired(24) = 20 * conversion_factor;
						q_new_desired(33) = -45 * conversion_factor;    // Neck horizontal rotation
						q_new_desired(34) = 45 * conversion_factor;    // Neck vertical rotation
						q_new_desired(25) = -90 * conversion_factor;    // Right wrist
				}
				joint_task->_desired_position = q_new_desired;
				N_prec.setIdentity();
				joint_task->updateTaskModel(N_prec);
				joint_task->computeTorques(joint_task_torques);
				command_torques = joint_task_torques;

				// Let's create a interface to retrieve the button info from redis
				button_name = redis_client.get("Xbox_input_button");
				density = stod(redis_client.get("Xbox_input_density"));

				if ((q_new_desired - robot->_q).norm() <= 0.001) {
						state = USER_SELECT_LOCATION;
						// print message
						cout << endl;
						cout << "Embrace the servebot! You can switch cameras by pressing X!" << endl;
						cout << endl;
						cout << "select your new location (left joysticks to select, A to proceed)" << endl;
				}
		}

		// Add a camera button (X), tell simviz to change to desired camera location and orientation
		// button_name = redis_client.get("Xbox_input_button");
		// density = stod(redis_client.get("Xbox_input_density"));
		if (button_name == "X_button") {
				if (prev_button_name != "X_button" || (prev_density == 0.0 && density == 1.0)) {
						// cout << "correct" << endl;
						if (camera_state == ROBOT_BACK_VIEW) {
								camera_state = ROBOT_SIDE_VIEW;
								redis_client.setEigenMatrixJSON("camera_pos", Vector3d(-10.9529,-5.49318,0.0740845));
								redis_client.setEigenMatrixJSON("camera_lookat", Vector3d(-10.9679,-4.49507,0.0145338));
						}
						else if (camera_state == ROBOT_SIDE_VIEW) {
								camera_state = ROBOT_FRONT_VIEW;
								redis_client.setEigenMatrixJSON("camera_pos", Vector3d(-7.41958,-2.07966,0.194219));
								redis_client.setEigenMatrixJSON("camera_lookat", Vector3d(-8.41482,-2.05996,0.0987788));
						}
						else if (camera_state == ROBOT_FRONT_VIEW) {
								camera_state = COURT_VIEW;
								redis_client.setEigenMatrixJSON("camera_pos", Vector3d(0.0,-13.8237,11.364));
								redis_client.setEigenMatrixJSON("camera_lookat", Vector3d(0.0,-13.0598,10.7186));
						}
						else if (camera_state == COURT_VIEW) {
								camera_state = OPPONENT_VIEW;
								redis_client.setEigenMatrixJSON("camera_pos", Vector3d(11.7499, 3.44443, 0.679269));
								redis_client.setEigenMatrixJSON("camera_lookat", Vector3d(10.7815,3.20929,0.59578));
						}
						else if (camera_state == OPPONENT_VIEW) {
								camera_state = ROBOT_BACK_VIEW;
								redis_client.setEigenMatrixJSON("camera_pos", Vector3d(-14,-3.25,1.0));
								redis_client.setEigenMatrixJSON("camera_lookat", Vector3d(-6.5,-1.75,0.0));
						}
						// cout << camera_state << endl;
				}
		}

		// execute redis write callback
		redis_client.executeWriteCallback(0);	

		prev_button_name = button_name;
		prev_density = density;
		controller_counter++;
	}

	double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
    std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";

	redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, 0 * command_torques);

	return 0;
}