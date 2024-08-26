#include "gz_match_plugin/action_zones.hpp"

#include <iostream>



ActionZones::ActionZones(){}

ActionZones::ActionZones(Team& team1, Team& team2){
	this->team1 = &team1;
	this->team2 = &team2;
}

ActionZones::ActionZones(Team& team1, Team& team2, gz::sim::World& world){
	this->team1 = &team1;
	this->team2 = &team2;
	this->world = &world;
}



ActionZones::~ActionZones(){}

void ActionZones::updateStart(){

	std::cout << "ActionZones updateStart" << std::endl;

	//spawn a large cube at 1,1,1 in the world

	auto cubeStr = R"(
	<?xml version="1.0" ?>
	<sdf version="1.6">
	  <model name="cube">
			<static>true</static>
	    <pose>1 -0.225 0.225 0 0 0</pose>
	    <link name="link">
	      <visual name="visual">
	        <geometry>
	          <box>
	            <size>0.45 0.45 0.45</size>
	          </box>
	        </geometry>
					<material>
						<ambient>1 0.0 0.0 0.5</ambient>
						<diffuse>1.0 0.0 0.0 0.5</diffuse>
					</material>
	      </visual>
	    </link>
	  </model>
	</sdf>
	)";

	gz::msgs::EntityFactory req;
  gz::msgs::Boolean res;

	req.set_sdf(cubeStr);
	bool result;
	const unsigned int timeout = 1000; // Timeout in milliseconds

	//node 
	gz::transport::Node node;
	std::string service_name = "/world/default/create";

	bool executed = node.Request(service_name,
				req, timeout, res, result);

	if (executed)
		{
			if (result)
				std::cout << "Entity was created : [" << res.data() << "]" << std::endl;
			else
			{
				std::cout << "Service call failed" << std::endl;
				return;
			}
		}
		else
			std::cerr << "Service call timed out" << std::endl;

}


void ActionZones::update(gz::sim::World worldEntity, const gz::sim::EntityComponentManager &_ecm) {
	// See if the robot is inside of this zone, change the color of the zone to green if it is, change it back to red if it's not

	// first let's print the robot's position from within this

	gz::sim::Entity robotEntity = worldEntity.ModelByName(_ecm, "blue_robot");

	std::cout << worldPose(robotEntity, _ecm) << std::endl;
	std::cout << "# of models : " << worldEntity.ModelCount(_ecm) << std::endl;

	// check de bourin ?

	// try changing the color of the cube at each update
	
	gz::sim::Entity cubeEntity = worldEntity.ModelByName(_ecm, "cube");

	if(temp_color){
		//change color to green
		auto visual = _ecm.Component<gz::sim::components::Visual>(cubeEntity);
		auto visual_cmd = new gz::sim::components::VisualCmd();
		// List all the functions and fields of visual_cmd
		std::cout << "visual_cmd functions and fields:" << std::endl;


		//_ecm.SetComponent(cubeEntity, visual_cmd);

		temp_color = false;
	}else{
		//change color to red
		//auto visual = _ecm.Component<gz::sim::components:Visual>(cubeEntity);
		//visual->material->ambient = {1.0, 0.0, 0.0, 0.5};
		//visual->material->diffuse = {1.0, 0.0, 0.0, 0.5};
		//_ecm.SetComponent(cubeEntity, visual);


		temp_color = true;
	}

}




void ActionZones::update() {/*...*/}
void ActionZones::updateEnd() {/*...*/}
void ActionZones::reset() {/*...*/}
void ActionZones::getScores(int* score1, int* score2) {/*...*/}
