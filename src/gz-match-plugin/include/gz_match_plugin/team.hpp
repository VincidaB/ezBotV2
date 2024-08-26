#ifndef TEAM_HPP
#define TEAM_HPP


#include <string>
#include <gz/sim/Model.hh>
//#include <gz/sim/Link.hh>
//#include <gz/sim/Util.hh>
//#include <gz/sim/System.hh>
//#include <gz/sim/World.hh>



class Team
{

	Team();
	Team(std::string team_name);
	Team(std::string team_name, std::string team_color);
	~Team();


	void set_team_color(std::string team_color);
	std::string get_team_color();

	void set_team_name(std::string team_name);
	std::string get_team_name();

	void set_robot(gz::sim::Entity robot);
	gz::sim::Entity get_robot();


	void setPrediction(int team_score_estimate);
	int getPrediction();



private:
	std::string color;
	std::string name;

	gz::sim::Entity robot;


	int score_prediction = 0;
	
	int final_score = 0;

};








#endif // TEAM_HPP