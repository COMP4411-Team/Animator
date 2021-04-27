#include "LSystem.h"
#include <FL/gl.h>
#include "modelerdraw.h"

using namespace std;

LSystem::LSystem()
{
	init_str = "A";
	rule["F"] = "S ///// F";
	rule["A"] = "[&FLA]/////[&FLA]///////`[&FLA]";
	rule["S"] = "F";
	rule["L"] = "[Q^^Q][Q\\\\Q]";
}


void LSystem::generate()
{
	str = init_str;
	for (int i = 0; i < max_iter; ++i)
		iterate();
	need_regenerate = false;
}

void LSystem::iterate()
{
	string result;
	for (int i = 0; i < str.length(); ++i)
	{
		bool flag = false;
		for (auto& item : rule)
		{
			if (str.substr(i, 1) != item.first)
				continue;
			
			result += item.second;
			flag = true;
			break;
		}
		if (!flag)	result += str[i];
	}
	
	str = result;
}

void LSystem::draw()
{
	if (need_regenerate)
		generate();
	for (int i = 0; i < str.length(); ++i)
	{
		char token = str[i];
		switch (token)
		{
		case 'F':
			drawCylinder(forward_dist, branch_radius1, branch_radius2);
			glTranslatef(0, 0, forward_dist);
			break;
		case 'G':
			glTranslatef(0, 0, forward_dist);
			break;
		case '+':
			glRotatef(yaw_angle, 0, 1, 0);
			break;
		case '-':
			glRotatef(-yaw_angle, 0, 1, 0);
			break;
		case '^':
			glRotatef(-pitch_angle, 1, 0, 0);
			break;
		case '&':
			glRotatef(pitch_angle, 1, 0, 0);
			break;
		case '/': case '>':
			glRotatef(-roll_angle, 0, 0, 1);
			break;
		case '\\': case '<':
			glRotatef(roll_angle, 0, 0, 1);
			break;
		case '|':
			glRotatef(180, 0, 1, 0);
			break;
		case '[':
			glPushMatrix();
			break;
		case ']':
			glPopMatrix();
			break;
		}
	}
}
