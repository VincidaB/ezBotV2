#ifndef GAME_ACTION_HPP
#define GAME_ACTION_HPP

#include <string>

#include "gz_match_plugin/team.hpp"
#include <gz/sim/World.hh>

class GameAction
{

public:
	GameAction();
	GameAction(Team& team1, Team& team2);
	GameAction(Team& team1, Team& team2, gz::sim::World& world);

	~GameAction();

	
	virtual void getScores(int* score1, int* score2) = 0;
	virtual void updateStart() = 0;
	virtual void updateEnd() = 0;
	virtual void update() = 0;
	virtual void reset() = 0;


	void setWorld(gz::sim::World& world);

	void setTeam1(Team& team);
	void setTeam2(Team& team);

	void setActionName(const std::string& action_name);
	const std::string getActionName();

	// prints both the action name and the scores 
	void printScores();


protected:

	int runningScore1 = 0;
	int runningScore2 = 0; 

	//World world;
	Team* team1;
	Team* team2;

	std::string action_name;

	gz::sim::World* world;


};



#endif // GAME_ACTION_HPP