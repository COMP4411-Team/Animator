#pragma once

#include <string>
#include <map>

class LSystem
{	
public:
	LSystem();
	
	void generate();
	void iterate();
	void draw();
	
	int max_iter{7};
	float yaw_angle{22.5f};
	float pitch_angle{22.5f};
	float roll_angle{22.5f};
	float forward_dist{0.1f};

	float branch_radius1{0.01f};
	float branch_radius2{0.01f};

	bool need_regenerate{true};
	std::map<std::string, std::string> rule;
	std::string str;
	std::string init_str;
};

