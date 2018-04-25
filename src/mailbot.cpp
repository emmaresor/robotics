#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include <actionlib/server/simple_action_server.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "std_msgs/String.h"
#include "std_srvs/Empty.h"

#include <yaml-cpp/yaml.h>
#include <sound_play/sound_play.h>
#include <sound_play/SoundRequest.h>

#include <iostream>

using namespace std;

string response;
bool heard_data = false;

void outputCB(const std_msgs::String::ConstPtr& s)
{
	response = s->data;
	heard_data = true;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "mailbot");
	ros::NodeHandle n;
	
	/********************************************************************************************
	  Initialize sound publisher and enumerate messages
	 ********************************************************************************************/
	ros::Publisher sound_pub;
	sound_play::SoundRequest S;
	S.sound = -3; // =SAY
	S.command = 1; // =PLAY_ONCE
	string messages[10] = {"Hi Megan! Are you delivering to a room or a professor?", "Oops! Incorrect input. Try again!",
			      "Please enter the room number.", "Please enter the professor's last name.", 
			      "I don't know where to go for the location you specified.", "Okay, I know where to go!",
			      "I have mail for you!","Did you pick up your mail?", "I delivered the mail!", 
			      "I didn't deliver the mail, sorry."};
	
	/********************************************************************************************
	  Initialize YAML nodes from loaded file of office locations
	 ********************************************************************************************/

	YAML::Node config = YAML::LoadFile(argv[1]);
	const YAML::Node& profs = config["Professors"];
	const YAML::Node& rooms = config["Rooms"];
	const YAML::Node& office = config["Office"];
	double officex = office[0].as<double>();
	double officey = office[1].as<double>();
	double officez = office[2].as<double>();

	/********************************************************************************************
	  Prompt user to indicate room or professor, determine which YAML node to use
	 ********************************************************************************************/

	S.arg = messages[1];
	sound_pub.publish(S);

	char location_type;
	cout << "Type R for room and P for professor: ";
	cin >> location_type;
	while (location_type != 'R' && location_type != 'P') {
		S.arg = messages[2];
		sound_pub.publish(S);
		cout << "Type R for room and P for professor: ";
		cin >> location_type;
	}

	string location;
	if (location_type == 'R') {
		S.arg = messages[3];
		sound_pub.publish(S);
		cout << "Room #: ";
		cin >> location;
	} else {
		S.arg = messages[4];
		sound_pub.publish(S);
		cout << "Professor's last name: ";
		cin >> location;
	}

	/********************************************************************************************
	  Get x,y,z positions
	 ********************************************************************************************/

	double xpos = -1.0;
	double ypos, zpos;
	if (location_type == 'R') {
		for (size_t i = 0; i < rooms.size(); i++) {
			const YAML::Node& destination = rooms[i];
			if (destination["room"].as<string>() == location) {
				xpos = destination["x"].as<double>();
				ypos = destination["y"].as<double>();
				zpos = destination["z"].as<double>();
				break;
			}
		}
	} else {
		for (size_t i = 0; i < profs.size(); i++) {
			const YAML::Node& destination = profs[i];
			if (destination["name"].as<string>() == location) {
				xpos = destination["x"].as<double>();
				ypos = destination["y"].as<double>();
				zpos = destination["z"].as<double>();
				break;
			}
		}
	}
	// Check if name not found
	if (xpos == -1.0) {
		S.arg = messages[5];
		sound_pub.publish(S);
		// Do something...
	} else {
		S.arg = messages[6];
		sound_pub.publish(S);
	}

	//travel TO mail delivery location based on x,y,z positions
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base",true);
	ac.waitForServer();
	move_base_msgs::MoveBaseGoal goal;

	goal.target_pose.header.stamp = ros::Time::now();
	goal.target_pose.header.frame_id = "/map";

	double yaw = 0.0;
	goal.target_pose.pose.position.x = xpos;
	goal.target_pose.pose.position.y = ypos;
	goal.target_pose.pose.position.z = zpos;
	goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

	ac.sendGoal(goal);
    
	ac.waitForResult();
	
	//Check if mail received
	S.arg = messages[6];
	sound_pub.publish(S);
	
	//wait for a sec
	
	S.arg = messages[7];
	sound_pub.publish(S);
	
	//pocketsphynx to hear a "yes" or "no" response
	ros::ServiceClient listenclient = n.serviceClient<std_srvs::Empty>("~start");
	ros::ServiceClient stopclient = n.serviceClient<std_srvs::Empty>("~stop");
	ros::Subscriber output_sub = n.subscribe("~output", 1, outputCB);
	
	double now = ros::Time::now().toSec();    
    	double timer;
    	bool stillwaiting = false;

	std_srvs::Empty srv;

	listenclient.call(srv);
    	while (!stillwaiting) {
		ros::spin();
		timer = ros::Time::now().toSec() - now;
		if (timer >= 20 || heard_data) {
	   		stillwaiting = false;
			stopclient.call(srv);
			break;
		}
	}
	
	//travel BACK to main office

	//set the header
	goal.target_pose.header.stamp = ros::Time::now();
	goal.target_pose.header.frame_id = "/map";
    
	//set relative x, y, and angle
	goal.target_pose.pose.position.x = officex;
	goal.target_pose.pose.position.y = officey;
	goal.target_pose.pose.position.z = officez;
	goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

	//send the goal
	ac.sendGoal(goal);
    
	//block until the action is completed
	ac.waitForResult();
	
	//Tell Megan the result of delivery
	if (heard_data) {
		S.arg = messages[8];
		sound_pub.publish(S);
	} else {
		S.arg = messages[9];
		sound_pub.publish(S);	
	}
	
	return 0;
}


