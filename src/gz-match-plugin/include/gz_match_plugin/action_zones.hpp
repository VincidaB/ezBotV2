#ifndef ACTION_ZONES_HPP
#define ACTION_ZONES_HPP

#include "gz_match_plugin/game_action.hpp"
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/World.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Visual.hh>
#include <gz/sim/components/VisualCmd.hh>
#include <gz/msgs/Factory.hh>
#include <gz/msgs.hh>
#include <gz/transport/Node.hh>
#include <string>
class ActionZones : public GameAction
{



public:
	ActionZones();
	ActionZones(Team& team1, Team& team2);
	ActionZones(Team& team1, Team& team2, gz::sim::World& world);
	~ActionZones();

	void getScores(int* score1, int* score2);
	void updateStart();
	void updateEnd();
	void update();
	void update(gz::sim::World worldEntity, const gz::sim::EntityComponentManager &_ecm);
	
	void reset();

private:
	int zones= 0;
	bool temp_color = true;


};



#endif