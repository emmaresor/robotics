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

struct Location {
	double x_coord, y_coord, z_coord;
};

void outputCB(const std_msgs::String& s)
{
	response = s.data;
	cerr << "response: " << response << endl;
	//what does it convert the response to exactly??
	heard_data = true;
}
void pause(int t, ros::NodeHandle& n)
{
	if (n.ok())
		sleep(t);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "mailbot");
	ros::NodeHandle n;
	
	/********************************************************************************************
	  Initialize sound publisher and enumerate messages
	 ********************************************************************************************/
	ros::Publisher sound_pub = n.advertise<sound_play::SoundRequest>("/robotsound", 1);
	sound_play::SoundClient S;
	pause(1, n);
	string messages[12] = {"Hi Megan! Do you have mail to deliver?", "Are you delivering to a room or a professor?", "Oops! Incorrect input. Try again!",
			      "Please enter the room number.", "Please enter the professor's last name.", 
			      "I don't know where to go for the location you specified.", "Okay, I know where to go!",
			      "I have mail for you!","Did you pick up your mail?", "I delivered the mail!", 
			      "I was not able to deliver the mail, sorry.", "Do you have more mail?"};


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
	 	pocketsphinx to get yes/no response regarding having mail to deliver.
	 ********************************************************************************************/
	//"Hi Megan! Do you have mail to deliver?"
	S.say(messages[0]);
	pause(1, n);
	
	/*
	//pocketsphynx to hear a "yes" or "no" response
	ros::ServiceClient listenclient = n.serviceClient<std_srvs::Empty>("/recognizer/start");
	ros::ServiceClient stopclient = n.serviceClient<std_srvs::Empty>("/recognizer/stop");
	ros::Subscriber output_sub = n.subscribe("/recognizer/output", 1, outputCB);
	
	double now = ros::Time::now().toSec();    
    	double timer;
    	bool stillwaiting = false;

	std_srvs::Empty srv;

	listenclient.call(srv);
    	while (!stillwaiting) {
		ros::spin();
		timer = ros::Time::now().toSec() - now;
		if (timer >= 10 || heard_data) {
	   		stillwaiting = false;
			stopclient.call(srv);
			break;
		}
	}
	*/
	//if no, exit program 
	//if yes, enter while loop and prompt for room/prof. set bool to yes/true
	bool yes;
	string response;
	cin >> response;
	if (response == "yes" /* && heard_data*/){
		yes = true;
	}else
		return 0;
	
	
	/********************************************************************************************
		  Prompt user to indicate room or professor, determine which YAML node to use
	 ********************************************************************************************/
	Location deliveries[30];
	int delivery_num = 0;
	
	while (yes){
		//"Are you delivering to a room or a professor?"
		S.say(messages[1]);
		pause(1, n);

		char location_type;
		cout << "Type R for room and P for professor: ";
		cin >> location_type;
		while (location_type != 'R' && location_type != 'P') {
			//"Oops! Incorrect input. Try again!",
			S.say(messages[2]);
			pause(1, n);
			cout << "Type R for room and P for professor: ";
			cin >> location_type;
		}

		string location;
		if (location_type == 'R') {
			//"Please enter the room number."
			S.say(messages[3]);
			pause(1, n);
			cout << "Room #: ";
			cin >> location;
		} else {
			//"Please enter the professor's last name."
			S.say(messages[4]);
			pause(1, n);
			cout << "Professor's last name: ";
			cin >> location;
		}

		/********************************************************************************************
		  Get x,y,z positions
		 ********************************************************************************************/
		deliveries[delivery_num].x_coord = -1.0;
		if (location_type == 'R') {
			for (size_t i = 0; i < rooms.size(); i++) {
				const YAML::Node& destination = rooms[i];
				if (destination["room"].as<string>() == location) {
					deliveries[delivery_num].x_coord = destination["x"].as<double>();
					deliveries[delivery_num].y_coord = destination["y"].as<double>();
					deliveries[delivery_num].z_coord = destination["z"].as<double>();
					delivery_num++;
					break;
				}
			}
		} else {
			for (size_t i = 0; i < profs.size(); i++) {
				const YAML::Node& destination = profs[i];
				if (destination["name"].as<string>() == location) {
					deliveries[delivery_num].x_coord = destination["x"].as<double>();
					deliveries[delivery_num].y_coord = destination["y"].as<double>();
					deliveries[delivery_num].z_coord = destination["z"].as<double>();
					delivery_num++;
					break;
				}
			}
		}
		// Check if name not found
		if (deliveries[delivery_num].x_coord == -1.0) {
			//"I don't know where to go for the location you specified."
			S.say(messages[5]);
			pause(1, n);
		} else {
			//"Okay, I know where to go!"
			S.say(messages[6]);
			pause(3, n);
		}
		//"Do you have more mail?"
		S.say(messages[11]);
		pause(1, n);
		/*
		now = ros::Time::now().toSec();    
    		stillwaiting = false;
		
		listenclient.call(srv);
    		while (!stillwaiting) {
			ros::spin();
			timer = ros::Time::now().toSec() - now;
			if (timer >= 10 || heard_data) {
				stillwaiting = false;
				stopclient.call(srv);
				break;
			}
		}
		*/
		cin >> response;
		if (response == "no" /*&& heard_data*/)
			yes = false;
	}
	//travel TO mail delivery location based on x,y,z positions
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base",true);
	ac.waitForServer();
	move_base_msgs::MoveBaseGoal goal;

	goal.target_pose.header.stamp = ros::Time::now();
	goal.target_pose.header.frame_id = "/map";

	double yaw = 0.0;
	for (int i = 0; i < delivery_num; i++){
		goal.target_pose.pose.position.x = deliveries[i].x_coord;
		goal.target_pose.pose.position.y = deliveries[i].y_coord;
		goal.target_pose.pose.position.z = deliveries[i].z_coord;
		goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

		ac.sendGoal(goal);
    
		ac.waitForResult();
	
		//Check if mail received

		// "I have mail for you!"
		S.say(messages[7]);
		pause(1, n);

		//wait for a sec

		//"Did you pick up your mail?"
		S.say(messages[8]);
		pause(1, n);
		bool heard_data;
		string response;
		cin >> response;
		if (response == "yes")
		{	
			heard_data = true;
		}
			
/*
		//pocketsphynx to hear a "yes" or "no" response
		now = ros::Time::now().toSec();    
		stillwaiting = false;

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
*/
	}

	//travel BACK to main office

	//set the header
	goal.target_pose.header.stamp = ros::Time::now();
	goal.target_pose.header.frame_id = "/map";
    
	//set relative x, y, and angle
	goal.target_pose.pose.position.x = 18.0929; //officex; temporarily using closer location (sheldon's office)
	goal.target_pose.pose.position.y = 14.4788; //officey;
	goal.target_pose.pose.position.z = officez;
	goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

	//send the goal
	ac.sendGoal(goal);
    
	//block until the action is completed
	ac.waitForResult();
	
	//Tell Megan the result of delivery
	if (heard_data) {
		//"I delivered the mail."
		S.say(messages[9]);
		pause(1, n);
	} else {
		//"I was not able to deliver the mail!"
		S.say(messages[10]);
		pause(1, n);
	}

	return 0;
}



