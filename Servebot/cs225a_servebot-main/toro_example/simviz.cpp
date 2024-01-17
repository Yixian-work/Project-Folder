/**
 * @file simviz.cpp
 * @brief Simulation + visualization.
 * 
 */

#include <GL/glew.h>
#include "Sai2Model.h"
#include "Sai2Graphics.h"
#include "Sai2Simulation.h"
#include <dynamics3d.h>
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"

#include <GLFW/glfw3.h> // must be loaded after loading opengl/glew

#include "uiforce/UIForceWidget.h"

#include <iostream>
#include <string>

#include <signal.h>
bool fSimulationRunning = false;
void sighandler(int){fSimulationRunning = false;}

#include <chrono>

using namespace std;
using namespace Eigen;

const string world_file = "./resources/world_toro.urdf";
const string robot_file = "./resources/toro.urdf";
const string robot_name = "toro";
const string camera_name = "camera_fixed";

#include "redis_keys.h"
#include "../include/object.h"

RedisClient redis_client;
RedisClient redis_client_xbox;

// Dynamic objects information
const string object_names = "ball1";
Vector3d object_pos;
Vector3d object_lin_vel;
Quaterniond object_ori;
Vector3d object_ang_vel;
// const int n_objects = object_names.size();

// Static object information
// const string static_name = "red_square";
// Vector3d static_pos;
// Quaterniond static_ori;

// Ball was hit determined variable
bool ball_was_hit = true;

// Camera change variable
// bool switch_camera = false;

// simulation function prototype
void simulation(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim, UIForceWidget *ui_force_widget);

// callback to print glfw errors
void glfwError(int error, const char* description);

// callback to print glew errors
bool glewInitialize();

// callback when a key is pressed
void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods);

// callback when a mouse button is pressed
void mouseClick(GLFWwindow* window, int button, int action, int mods);

// flags for scene camera movement
bool fTransXp = false;
bool fTransXn = false;
bool fTransYp = false;
bool fTransYn = false;
bool fTransZp = false;
bool fTransZn = false;
bool fRotPanTilt = false;
bool fRobotLinkSelect = false;

int main() {
	cout << "Loading URDF world model file: " << world_file << endl;

	// start redis client
	redis_client = RedisClient();
	redis_client.connect();

	redis_client_xbox = RedisClient();
	redis_client_xbox.connect();

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load graphics scene
	auto graphics = new Sai2Graphics::Sai2Graphics(world_file, true);
	Eigen::Vector3d camera_pos, camera_lookat, camera_vertical;
	graphics->getCameraPose(camera_name, camera_pos, camera_vertical, camera_lookat);
	cout << camera_pos << endl;
	graphics->_world->setBackgroundColor(66.0/255, 135.0/255, 245.0/255);  // set blue background 
	graphics->getCamera(camera_name)->setClippingPlanes(0.1, 50); // Camera vision clipping range

	// load second graphics scene
	// auto graphics2 = new Sai2Graphics::Sai2Graphics(world_file, true);
	// Eigen::Vector3d camera_pos2, camera_lookat2, camera_vertical2;
	// graphics2->getCameraPose(camera_name2, camera_pos2, camera_vertical2, camera_lookat2);
	// graphics2->_world->setBackgroundColor(66.0/255, 135.0/255, 245.0/255);  // set blue background 
	// graphics2->getCamera(camera_name2)->setClippingPlanes(0.1, 50); // Camera vision clipping range

	// load robots
	auto robot = new Sai2Model::Sai2Model(robot_file, false);
	// add initial robot configuration here if needed 

	// load simulation world
	auto sim = new Simulation::Sai2Simulation(world_file, false);
	sim->setCollisionRestitution(0.75);
	sim->setCoeffFrictionStatic(0.0);
	sim->setCoeffFrictionDynamic(0.0);

	// read joint positions, velocities, update model
	sim->getJointPositions(robot_name, robot->_q);
	sim->getJointVelocities(robot_name, robot->_dq);
	robot->updateModel();

    sim->getObjectPosition(object_names, object_pos, object_ori);
    sim->getObjectVelocity(object_names, object_lin_vel, object_ang_vel);

	// Fill in static object information
	// sim->getObjectPosition(static_name, static_pos, static_ori);

	/*------- Set up visualization -------*/
	// set up error callback
	glfwSetErrorCallback(glfwError);

	// initialize GLFW
	glfwInit();

	// retrieve resolution of computer display and position window accordingly
	GLFWmonitor* primary = glfwGetPrimaryMonitor();
	const GLFWvidmode* mode = glfwGetVideoMode(primary);

	// information about computer screen and GLUT display window
	int screenW = mode->width;
	int screenH = mode->height;
	int windowW = 0.8 * screenH;
	int windowH = 0.5 * screenH;
	int windowPosY = (screenH - windowH) / 2;
	int windowPosX = windowPosY;

	// create window and make it current
	glfwWindowHint(GLFW_VISIBLE, 0);
	GLFWwindow* window = glfwCreateWindow(windowW, windowH, "SAI2.0 - ToroApplications", NULL, NULL);
	glfwSetWindowPos(window, windowPosX, windowPosY);
	glfwShowWindow(window);
	glfwMakeContextCurrent(window);
	glfwSwapInterval(1);

	// create window and make it current
	// glfwWindowHint(GLFW_VISIBLE, 0);
	// GLFWwindow* window2 = glfwCreateWindow(windowW, windowH, "SAI2.0 - ToroApplications2", NULL, NULL);
	// glfwSetWindowPos(window2, windowPosX, windowPosY);
	// glfwShowWindow(window2);
	// glfwMakeContextCurrent(window2);
	// glfwSwapInterval(1);

	// set callbacks
	glfwSetKeyCallback(window, keySelect);
	glfwSetMouseButtonCallback(window, mouseClick);

	// set callbacks
	// glfwSetKeyCallback(window2, keySelect);
	// glfwSetMouseButtonCallback(window2, mouseClick);

	// init click force widget 
	auto ui_force_widget = new UIForceWidget(robot_name, robot, graphics);
	ui_force_widget->setEnable(false);

	// cache variables
	double last_cursorx, last_cursory;

	// initialize glew
	glewInitialize();

	fSimulationRunning = true;
	thread sim_thread(simulation, robot, sim, ui_force_widget);

	// Add a red_target_box
	// string button_name, prev_button_name;
	// double density, prev_density;
	Vector3d target_box_pos = Vector3d(3.0, 1.5, -1.15);
	Vector3d target_box_size = Vector3d(1.0, 1.0, 0.05);
	addBox(graphics, "red_target_box", target_box_pos, Quaterniond(1, 0, 0, 0), target_box_size, Vector4d(1, 0, 0, 1));
	
	// while window is open:
	while (!glfwWindowShouldClose(window) && fSimulationRunning)
	{
		// update graphics. this automatically waits for the correct amount of time
		int width, height;
		glfwGetFramebufferSize(window, &width, &height);
		graphics->updateGraphics(robot_name, robot);
		// graphics2->updateGraphics(robot_name, robot);

        graphics->updateObjectGraphics(object_names, object_pos, object_ori);
		// graphics2->updateObjectGraphics(object_names[i], object_pos[i], object_ori[i]);
		
		// update static object graphic
		// graphics->updateObjectGraphics(static_name, static_pos, static_ori);

		graphics->render(camera_name, width, height);
		// graphics2->render(camera_name2, width, height);

		// swap buffers
		glfwSwapBuffers(window);

		// swap buffers
		// glfwSwapBuffers(window2);

		// wait until all GL commands are completed
		glFinish();

		// check for any OpenGL errors
		GLenum err;
		err = glGetError();
		assert(err == GL_NO_ERROR);

		// poll for events
		glfwPollEvents();

		// New camera position
		//if (switch_camera) {
		//		camera_pos = redis_client.getEigenMatrixJSON("camera_pos");
		//		camera_lookat = redis_client.getEigenMatrixJSON("camera_lookat");
		//}

		// move scene camera as required
		graphics->getCameraPose(camera_name, camera_pos, camera_vertical, camera_lookat);
		Eigen::Vector3d cam_depth_axis;
		cam_depth_axis = camera_lookat - camera_pos;
		cam_depth_axis.normalize();
		Eigen::Vector3d cam_up_axis;
		// cam_up_axis = camera_vertical;
		// cam_up_axis.normalize();
		cam_up_axis << 0.0, 0.0, 1.0; //TODO: there might be a better way to do this
		Eigen::Vector3d cam_roll_axis = (camera_lookat - camera_pos).cross(cam_up_axis);
		cam_roll_axis.normalize();
		Eigen::Vector3d cam_lookat_axis = camera_lookat;
		cam_lookat_axis.normalize();
		if (fTransXp) {
			camera_pos = camera_pos + 0.05*cam_roll_axis;
			camera_lookat = camera_lookat + 0.05*cam_roll_axis;
		}
		if (fTransXn) {
			camera_pos = camera_pos - 0.05*cam_roll_axis;
			camera_lookat = camera_lookat - 0.05*cam_roll_axis;
		}
		if (fTransYp) {
			// camera_pos = camera_pos + 0.05*cam_lookat_axis;
			camera_pos = camera_pos + 0.05*cam_up_axis;
			camera_lookat = camera_lookat + 0.05*cam_up_axis;
		}
		if (fTransYn) {
			// camera_pos = camera_pos - 0.05*cam_lookat_axis;
			camera_pos = camera_pos - 0.05*cam_up_axis;
			camera_lookat = camera_lookat - 0.05*cam_up_axis;
		}
		if (fTransZp) {
			camera_pos = camera_pos + 0.1*cam_depth_axis;
			camera_lookat = camera_lookat + 0.1*cam_depth_axis;
		}	    
		if (fTransZn) {
			camera_pos = camera_pos - 0.1*cam_depth_axis;
			camera_lookat = camera_lookat - 0.1*cam_depth_axis;
		}
		if (fRotPanTilt) {
			// get current cursor position
			double cursorx, cursory;
			glfwGetCursorPos(window, &cursorx, &cursory);
			//TODO: might need to re-scale from screen units to physical units
			double compass = 0.006*(cursorx - last_cursorx);
			double azimuth = 0.006*(cursory - last_cursory);
			double radius = (camera_pos - camera_lookat).norm();
			Eigen::Matrix3d m_tilt; m_tilt = Eigen::AngleAxisd(azimuth, -cam_roll_axis);
			camera_pos = camera_lookat + m_tilt*(camera_pos - camera_lookat);
			Eigen::Matrix3d m_pan; m_pan = Eigen::AngleAxisd(compass, -cam_up_axis);
			camera_pos = camera_lookat + m_pan*(camera_pos - camera_lookat);
		}

		// Print out information about current camera
		// cout << camera_pos << endl;
		// cout << cam_up_axis << endl;
		// cout << camera_lookat << endl;

		camera_pos = redis_client_xbox.getEigenMatrixJSON("camera_pos");
		camera_lookat = redis_client_xbox.getEigenMatrixJSON("camera_lookat");

		graphics->setCameraPose(camera_name, camera_pos, cam_up_axis, camera_lookat);

		glfwGetCursorPos(window, &last_cursorx, &last_cursory);

/*		ui_force_widget->setEnable(fRobotLinkSelect);
		if (fRobotLinkSelect)
		{
			double cursorx, cursory;
			int wwidth_scr, wheight_scr;
			int wwidth_pix, wheight_pix;
			std::string ret_link_name;
			Eigen::Vector3d ret_pos;

			// get current cursor position
			glfwGetCursorPos(window, &cursorx, &cursory);

			glfwGetWindowSize(window, &wwidth_scr, &wheight_scr);
			glfwGetFramebufferSize(window, &wwidth_pix, &wheight_pix);

			int viewx = floor(cursorx / wwidth_scr * wwidth_pix);
			int viewy = floor(cursory / wheight_scr * wheight_pix);

			if (cursorx > 0 && cursory > 0)
			{
				ui_force_widget->setInteractionParams(camera_name, viewx, wheight_pix - viewy, wwidth_pix, wheight_pix);
				//TODO: this behavior might be wrong. this will allow the user to click elsewhere in the screen
				// then drag the mouse over a link to start applying a force to it.
			}
		} */

        // Visualize the desired landing position (via the red box, controlled by the Xbox joystick)
		double Xbox_motion_hor = stod(redis_client_xbox.get("Xbox_motion_hor")) / 10.0;
		double Xbox_motion_ver = stod(redis_client_xbox.get("Xbox_motion_ver")) / 10.0;
		target_box_pos -= Vector3d(Xbox_motion_ver, 0.0, 0.0);
		target_box_pos -= Vector3d(0.0, Xbox_motion_hor, 0.0);

		cout << "Xbox_motion_hor: " << Xbox_motion_hor << "\tXbox_motion_ver: " << Xbox_motion_ver << endl;
		graphics->updateObjectGraphics("red_target_box", target_box_pos, Quaterniond(1, 0, 0, 0));

		redis_client_xbox.setEigenMatrixJSON("red_target_box_position", target_box_pos);

	}

	// stop simulation
	fSimulationRunning = false;
	sim_thread.join();

	// destroy context
	glfwSetWindowShouldClose(window,GL_TRUE);
	glfwDestroyWindow(window);

	// glfwSetWindowShouldClose(window2,GL_TRUE);
	// glfwDestroyWindow(window2);

	// terminate
	glfwTerminate();

	return 0;
}

bool isObjectInContact(const Simulation::Sai2Simulation& sim, const string object_name) {
	bool object_exists = false;
	list<cDynamicBase*>::iterator i;
	for (i = sim._world->m_dynamicObjects.begin(); i != sim._world->m_dynamicObjects.end(); ++i) {
		cDynamicBase* object = *i;
		if (object->m_name == object_name) {
			object_exists = true;
			int num_contacts = object->m_dynamicContacts->getNumContacts();
			if (num_contacts > 0) {
				return true;
			}
		}
	}
	if (!object_exists) {
		throw std::runtime_error("object does not exists in is ObjectInContact()");
	}
	return false;
}

//------------------------------------------------------------------------------
void simulation(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim, UIForceWidget *ui_force_widget) {

	int dof = robot->dof();
	VectorXd command_torques = VectorXd::Zero(dof);
	redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);
	string controller_status = "0";
	redis_client.set(CONTROLLER_RUNNING_KEY, controller_status);

	// Change restitution
	// double vel_res = stod(redis_client.get("velocity_res"));
	// sim->setCollisionRestitution(vel_res);

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(200); 
	double last_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;

	// init variables
	VectorXd g(dof);

	Eigen::Vector3d ui_force;
	ui_force.setZero();

	Eigen::VectorXd ui_force_command_torques;
	ui_force_command_torques.setZero();

	// setup redis callback
	redis_client.createReadCallback(0);
	redis_client.createWriteCallback(0);

	// add to read callback
	redis_client.addStringToReadCallback(0, CONTROLLER_RUNNING_KEY, controller_status);
	redis_client.addEigenToReadCallback(0, JOINT_TORQUES_COMMANDED_KEY, command_torques);

	// add to write callback
	redis_client.addEigenToWriteCallback(0, JOINT_ANGLES_KEY, robot->_q);
	redis_client.addEigenToWriteCallback(0, JOINT_VELOCITIES_KEY, robot->_dq);

	while (fSimulationRunning) {
		fTimerDidSleep = timer.waitForNextLoop();

		// execute redis read callback
		redis_client.executeReadCallback(0);

		// get gravity torques
		robot->gravityVector(g);
		
		// get ui force 
	//	ui_force_widget->getUIForce(ui_force);
	//	ui_force_widget->getUIJointTorques(ui_force_command_torques);

	//	if (fRobotLinkSelect)
	//		sim->setJointTorques(robot_name, command_torques + ui_force_command_torques + g);
	//	else
		sim->setJointTorques(robot_name, command_torques + g);

		// sim->setJointTorque(object_names, 2, 0.056*9.81);
		// Determine whether we need to freeze the ball in space based on state machine
		if (!ball_was_hit) {
			sim->setJointTorque(object_names, 2, 0.056*9.81);
		}
		if (isObjectInContact(*sim, object_names)) {
			sim->setJointTorque(object_names, 2, 0.0);
			sim->setCollisionRestitution(0.75);
			ball_was_hit = true;
		}

		// integrate forward
		double curr_time = timer.elapsedTime();
		double loop_dt = 0.001; 
		sim->integrate(loop_dt);

		// read joint positions, velocities, update model
		sim->getJointPositions(robot_name, robot->_q);
		sim->getJointVelocities(robot_name, robot->_dq);
		robot->updateModel();
		
		// STATE MACHINE RELATED: Spawn the ball after we are ready to serve
		if (stod(redis_client.get("updated_ball_flag")) == 1) {
			sim->setObjectPosition(object_names, redis_client.getEigenMatrixJSON("updated_ball_pos"), object_ori);
			ball_was_hit = false;
			sim->setCollisionRestitution(2.0);
			sim->setJointVelocity(object_names, 0, 0.0);
			sim->setJointVelocity(object_names, 1, 0.0);
			sim->setJointVelocity(object_names, 2, 0.0);
		}
		// get dynamic object positions
        sim->getObjectPosition(object_names, object_pos, object_ori);
        sim->getObjectVelocity(object_names, object_lin_vel, object_ang_vel);
	
		// Update static object location
		// sim->setObjectPosition(static_name, redis_client.getEigenMatrixJSON("updated_pos"), static_ori);

		// visualizer
		// robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
		// robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);
		// robot->updateModel();

		// execute redis write callback
		redis_client.executeWriteCallback(0);	

		// update last time
		last_time = curr_time;
		
	}

	double end_time = timer.elapsedTime();
	std::cout << "\n";
	std::cout << "Simulation Loop run time  : " << end_time << " seconds\n";
	std::cout << "Simulation Loop updates   : " << timer.elapsedCycles() << "\n";
	std::cout << "Simulation Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";
}

//------------------------------------------------------------------------------

void glfwError(int error, const char* description) {
	cerr << "GLFW Error: " << description << endl;
	exit(1);
}

//------------------------------------------------------------------------------

bool glewInitialize() {
	bool ret = false;
	#ifdef GLEW_VERSION
	if (glewInit() != GLEW_OK) {
		cout << "Failed to initialize GLEW library" << endl;
		cout << glewGetErrorString(ret) << endl;
		glfwTerminate();
	} else {
		ret = true;
	}
	#endif
	return ret;
}

//------------------------------------------------------------------------------

void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	bool set = (action != GLFW_RELEASE);
	switch(key) {
		case GLFW_KEY_ESCAPE:
			// exit application
			fSimulationRunning = false;
			glfwSetWindowShouldClose(window, GL_TRUE);
			break;
		case GLFW_KEY_RIGHT:
			fTransXp = set;
			break;
		case GLFW_KEY_LEFT:
			fTransXn = set;
			break;
		case GLFW_KEY_UP:
			fTransYp = set;
			break;
		case GLFW_KEY_DOWN:
			fTransYn = set;
			break;
		case GLFW_KEY_A:
			fTransZp = set;
			break;
		case GLFW_KEY_Z:
			fTransZn = set;
			break;
		default:
			break;
	}
}

//------------------------------------------------------------------------------

void mouseClick(GLFWwindow* window, int button, int action, int mods) {
	bool set = (action != GLFW_RELEASE);
	//TODO: mouse interaction with robot
	switch (button) {
		// left click pans and tilts
		case GLFW_MOUSE_BUTTON_LEFT:
			fRotPanTilt = set;
			// NOTE: the code below is recommended but doesn't work well
			// if (fRotPanTilt) {
			// 	// lock cursor
			// 	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
			// } else {
			// 	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
			// }
			break;
		// if right click: don't handle. this is for menu selection
		case GLFW_MOUSE_BUTTON_RIGHT:
			fRobotLinkSelect = set;
			break;
		// if middle click: don't handle. doesn't work well on laptops
		case GLFW_MOUSE_BUTTON_MIDDLE:
			break;
		default:
			break;
	}
}
