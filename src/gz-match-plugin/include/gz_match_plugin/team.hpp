#ifndef TEAM_HPP
#define TEAM_HPP

#include <string>

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


	void add_score(int score);
	void subtract_score(int score);
	int get_score();

	void set_final_score(int final_score);
	int get_final_score();
	
	void set_team_score_estimate(int team_score_estimate);
	int get_team_score_estimate();



private:
	std::string team_color;
	std::string team_name;


	int score = 0;

	int team_score_estimate = 0;
	int final_score = 0;

	void apply_score_estimate();

};








#endif // TEAM_HPP