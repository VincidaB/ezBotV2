#include "gz_match_plugin/game_action.hpp"


GameAction::GameAction(){}

GameAction::GameAction(Team& team1, Team& team2){
	this->team1 = &team1;
	this->team2 = &team2;
}

GameAction::GameAction(Team& team1, Team& team2, gz::sim::World& world){
	this->team1 = &team1;
	this->team2 = &team2;
	this->world = &world;
}

GameAction::~GameAction(){}

void GameAction::setWorld(gz::sim::World& world){
	this->world = &world;
}

void GameAction::setTeam1(Team& team){
	this->team1 = &team;
}

void GameAction::setTeam2(Team& team){
	this->team2 = &team;
}

void GameAction::setActionName(const std::string& action_name){
	this->action_name = action_name;
}

const std::string GameAction::getActionName(){
	return this->action_name;
}

void GameAction::printScores(){
	std::cout << this->action_name << " : " << runningScore1 << " - " << runningScore2 << std::endl;
}