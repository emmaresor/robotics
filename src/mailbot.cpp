#include <ros/ros.h>
#include<iostream>
#include<yaml-cpp/yaml.h>

using namespace std;

int main(int argc, char** argv)
{
  
  YAML::Node config = YAML::LoadFile(argv[1]);
  YAML::Node& profs = confg["Professors"];
  YAML::Node& rooms = confg["Rooms"];

/*for (std::size_t i = 0; i < node_test1.size(); i++) {
    const YAML::Node& node_test2 = node_test1[i];
    std::cout << "Id: " << node_test2["id"].as<std::string>() << std::endl;
    std::cout << "hardwareId: " << node_test2["hardwareId"].as<std::string>() << std::endl << std::endl;*/
}
