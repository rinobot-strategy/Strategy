/*
 * This file is part of the VSS-SampleStrategy project.
 *
 * This Source Code Form is subject to the terms of the GNU GENERAL PUBLIC LICENSE,
 * v. 3.0. If a copy of the GPL was not distributed with this
 * file, You can obtain one at http://www.gnu.org/licenses/gpl-3.0/.
 */

#ifndef _STRATEGY_H_
#define _STRATEGY_H_

#include "sample.h"
#include "navigation.h"

using namespace std;
using namespace common;

class Strategy : public Sample{
	/*
		Sample :: string main_color;
		Sample :: bool is_debug;
		Sample :: bool real_environment;
		Sample :: int situation;

		Sample :: common::State state;
		Sample :: common::Command commands[3];
		Sample :: common::Debug debug;
	*/
protected:
	float robot_radius;
	float distance_to_stop;
	// field campo;
	btVector3 final;
	bool changePose;
	int defender_line, goalkepper_line;
	int v_max_gol_ef;

public:
	Strategy();	
	navigation Navigation;
	void init(string main_color, bool is_debug, bool real_environment, string ip_receive_state, string ip_send_debug, string ip_send_command, string name);
	void loop();
	void calc_strategy();
	void killer_cpu();
	void goalkepper();
	void defender_root();
	float dist_giro;
	void rotate();

	common::Command calc_cmd_to(btVector3 init, btVector3 final, float distance_to_stop = 10.0);
	common::Command velocity_killer_cpu(btVector3);
	common::Command velocity_defender_root(btVector3);
	common::Command velocity_goalkepper(btVector3);
};

#endif // _STRATEGY_H_