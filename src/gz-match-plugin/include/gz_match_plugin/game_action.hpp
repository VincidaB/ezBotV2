#ifndef GAME_ACTION_HPP
#define GAME_ACTION_HPP

#include "gz-match-plugin/team.hpp"

class GameAction
{

	GameAction();

	void update();

protected:

	Team team1;
	Team team2;


};



#endif // GAME_ACTION_HPP