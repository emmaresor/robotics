#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include <actionlib/server/simple_action_server.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"

#include <yaml-cpp/yaml.h>
#include <sound_play/sound_play.h>
#include <sound_play/SoundRequest.h>

#include <iostream>

using namespace std;

int main(int argc, char** argv)
{
	/********************************************************************************************
	  Initialize sound publisher and enumerate messages
	 ********************************************************************************************/

	ros::Publisher sound_pub;
	sound_play::SoundRequest S;
	S.sound = -3; // =SAY
	S.command = 1; // =PLAY_ONCE
	string messages[8] = {"Hi Megan! Are you delivering to a room or a professor?", "Oops! Incorrect input. Try again!",
		"Please enter the room number.", "Please enter the professor's last name.", 
		"I don't know where to go for the location you specified.", "Okay, I know where to go!",
		"I have mail for you!","Did you pick up your mail?"};
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

	YAML::Node& profs = confg["Professors"];
	YAML::Node& rooms = confg["Rooms"];
	YAML::Node& office = confg["Office"];
	double officex = office[0];
	double officey = office[1];
	double officez = office[2];

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
}

//call nav_goal function on x,y,z positions
