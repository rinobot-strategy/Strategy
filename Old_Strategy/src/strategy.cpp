/*
 * This file is part of the VSS-SampleStrategy project.
 *
 * This Source Code Form is subject to the terms of the GNU GENERAL PUBLIC LICENSE,
 * v. 3.0. If a copy of the GPL was not distributed with this
 * file, You can obtain one at http://www.gnu.org/licenses/gpl-3.0/.
 */

#include "strategy.h"

Strategy::Strategy(){
    main_color = "yellow";
    is_debug = false;
    real_environment = false;
	robot_radius = 8.0;
	distance_to_stop = 5.0;
	changePose = true;
	dist_giro = 8.5;
	defender_line = 25;
	goalkepper_line = 10;
	v_max_gol_ef = 120;
	srand(time(NULL));
}

void Strategy::init(string main_color, bool is_debug, bool real_environment, string ip_receive_state, string ip_send_debug, string ip_send_command, string name){
	init_sample(main_color, is_debug, real_environment, ip_receive_state, ip_send_debug, ip_send_command, name);

	loop();

}

void Strategy::loop(){
	while(true){
		// DON'T REMOVE receive_data();
		receive_state();
		// DON'T REMOVE receive_Data();'
		
		calc_strategy();
		
		if(!real_environment){
			// DON'T REMOVE send_data();
			send_commands();
			// DON'T REMOVE send_data();
		}else{
			// Put your transmission code here
		}

		// DON'T REMOVE
		if(is_debug)
			send_debug();
		// DON'T REMOVE'
	}
}

void Strategy::calc_strategy(){
	if(main_color == "yellow")
		Navigation.set_side("right");
	else
		Navigation.set_side("left");

	killer_cpu();
	defender_root();
	goalkepper();
	for(int i = 0 ; i < 3 ; i++){
		debug.robots_path[i].poses.clear();
	}
	
	debug.robots_path[0].poses.push_back(state.robots[0].pose);
	debug.robots_path[0].poses.push_back(final);
}

void Strategy::killer_cpu(){
	float angle;
	btVector3 goal_atk;
	goal_atk.y = Navigation.centroid_atk.y;
	goal_atk.x = Navigation.centroid_atk.x;
	if(changePose){
		changePose = false;
		final.x = state.ball.x;
		final.y = state.ball.y;
		final.z = 0;
	}

	angle = angulation(goal_atk,state.ball);
	Navigation.set_theta_dir(angle*(pi/180));

	if (Navigation.centroid_atk.x > Navigation.centroid_def.x){
            if (state.ball.x < 30 || state.ball.y < 20  || state.ball.y > 110){
                if(state.ball.y< Navigation.centroid_atk.y /*&& robo_pos.y<centroid_atk.y*/)
                    Navigation.set_theta_dir(-30*pi/180);
                if(state.ball.y>Navigation.centroid_atk.y /*&& robo_pos.y>centroid_atk.y*/)
                    Navigation.set_theta_dir(30*pi/180);
            }
            else{
                Navigation.set_theta_dir((angle)*M_PI/180);  // Seta a orientação do Univector Field
                //    cout << "CPU" << endl;
        }
    }
    else{
            if (state.ball.x > 130 || state.ball.y < 20  || state.ball.y > 110){ //antes 15 e 115

                if(state.ball.y<Navigation.centroid_atk.y /*&& robo_pos.y<centroid_atk.y*/)
                    Navigation.set_theta_dir(-150*pi/180);
                if(state.ball.y>Navigation.centroid_atk.y /*&& robo_pos.y>centroid_atk.y*/)
                    Navigation.set_theta_dir(150*pi/180);
            }
            else{
                Navigation.set_theta_dir((angle)*M_PI/180);  // Seta a orientação do Univector Field                //     cout << "CPU" << endl;
            }
    }

	float d1,d2,d3;
	btVector3 enemy_prox;

	d1 = distancePoint(state.robots[0].pose,state.robots[3].pose);
	d2 = distancePoint(state.robots[0].pose,state.robots[4].pose);
	d3 = distancePoint(state.robots[0].pose,state.robots[5].pose);

 if(d1 <= d2 && d1<=d3)
	enemy_prox = state.robots[3].pose;
 if(d2 <= d1 && d2<=d3)
	enemy_prox = state.robots[4].pose;
if(d3 <= d2 && d3<=d1)
	enemy_prox = state.robots[5].pose;

	Navigation.generate_univector(state.robots[0].pose.y,state.robots[0].pose.x, state.robots[0], state.ball,enemy_prox);

	commands[0] = velocity_killer_cpu(state.robots[0].pose);

}

void Strategy::defender_root(){
		btVector3 meta;

	    if(state.ball.x > 0 && state.ball.y > 0){
			if (Navigation.centroid_def.x < Navigation.centroid_atk.x){
				if (state.ball.x > Navigation.centroid_def.x + defender_line){
					meta.x = Navigation.centroid_def.x + defender_line;
					meta.y = state.ball.y;//centroid_def.y;
				}
				else{
					if(state.ball.y < Navigation.centroid_def.y - 35){
						meta.x = Navigation.centroid_def.x - 5;
						meta.y = Navigation.centroid_def.y - 45;
					}
					else if(state.ball.y > Navigation.centroid_def.y + 35){
						meta.x = Navigation.centroid_def.x - 5;
						meta.y = Navigation.centroid_def.y + 45;     }
					else{
						meta.x = Navigation.centroid_def.x + defender_line;
						meta.y = Navigation.centroid_def.y;
					}
				}
			}else{
				if (state.ball.x < Navigation.centroid_def.x - defender_line){
					meta.x = Navigation.centroid_def.x - defender_line;
					meta.y = state.ball.y;
				}
				else{
					if(state.ball.y < Navigation.centroid_def.y - 35){
						meta.x = Navigation.centroid_def.x + 5;
						meta.y = Navigation.centroid_def.y - 45;
					}
					else if(state.ball.y > Navigation.centroid_def.y + 35){
						meta.x = Navigation.centroid_def.x + 5;
						meta.y = Navigation.centroid_def.y + 45;     }
					else{
						meta.x = Navigation.centroid_def.x - defender_line;
						meta.y = Navigation.centroid_def.y;
					}
				}
			}
		}

	Navigation.fake_cph(state.robots[1] , meta);

	commands[1] = velocity_defender_root(state.robots[1].pose);
	
}

void Strategy::goalkepper(){
		btVector3 meta;
	
		meta.x = Navigation.centroid_def.x;
		if(state.ball.y <= 85 && state.ball.y >= 45)
			meta.y = state.ball.y;
		else if (state.ball.y > 85)
			meta.y = 85;
		else if (state.ball.y < 45)
			meta.y = 45;

	Navigation.fake_cph(state.robots[2], meta );
	
	commands[2] = velocity_goalkepper(state.robots[2].pose);	
}

common::Command Strategy::velocity_killer_cpu(btVector3 robo){
	Command cmd;
	float angulation_robot_goal,alpha;
	float angulation_robot_robot_goal;
   float limiar_theta = 90 , delta_limiar = 10 , last_phi;
	float v, v_delta = 6, v_max = 50;
	float w, kp = 20, kd = 0.03 , l = 8;

	angulation_robot_goal = Navigation.get_angle_cpu()*(180/pi);
	//cout << "X: " << state.robots[3].pose.x << "Y: " << state.robots[3].pose.y << "Z: " << state.robots[3].pose.z << endl;
	alpha = angulation_robot_goal - (robo.z-180);
	alpha = ajusta_angulo(alpha);
	
//	 cout << "Direct: " << angulation_robot_goal << endl;
	// cout << "Robot: " << robo.z - 180 << endl;
	// cout << "Result: " << alpha << endl;
	//cout << endl << "Vx: " << state.robots[0].v_pose.x << " Vy: " << state.robots[0].v_pose.y << endl;

	// PID
    if (fabs(alpha) <= limiar_theta ){
        v = -v_delta*fabs(alpha)/limiar_theta + v_max;
        w = kp*alpha/180 + kd*(alpha - last_phi);
        limiar_theta = 90 - delta_limiar;
    }
    else{
        alpha = ajusta_angulo(alpha+180);
        v = v_delta*fabs(alpha)/limiar_theta - v_max;
        w = kp*alpha/180 + kd*(alpha - last_phi);
        limiar_theta = 90 + delta_limiar;
    }
	last_phi = alpha;
//	cout << "v " << v << "w " << w << endl;

		//Rotate
	if (Navigation.centroid_atk.x > Navigation.centroid_def.x){
		if(distancePoint(robo,state.ball) < dist_giro && (robo.y < 20 || robo.y > 110)){
			if(robo.y < 20){
				cmd.left  = 255;
				cmd.right = -255;
			}else if( robo.y > 110){
				cmd.left  = -255;
				cmd.right = 255;
			}
		}else{
			cmd.left = v+(w*l);
			cmd.right = v-(w*l);
		}
	}else{
		if(distancePoint(robo,state.ball) < dist_giro && (robo.y < 20 || robo.y > 110)){
			if(robo.y < 20){
				cmd.left  = -255;
				cmd.right = 255;
			}else if( robo.y > 110){
				cmd.left  = 255;
				cmd.right = -255;
			}
		}else{
			cmd.left = v+(w*l);
			cmd.right = v-(w*l);
		}
	}

	changePose = true;

	return cmd;
}

common::Command Strategy::velocity_defender_root(btVector3 robo){
	Command cmd;
	float distance_robot_goal;
	float angulation_robot_goal,alpha;
	float angulation_robot_robot_goal;
   float limiar_theta = 90 , delta_limiar = 10 , last_phi;
	float v, v_delta = 6, v_max = 50;
	float w,kp = 10, kd = 0.003 , l = 8;

  if (Navigation.centroid_def.x < Navigation.centroid_atk.x){
        double tempo;
        double aux_position_y;
        tempo = (robo.x + 4.5 - state.ball.x)/state.v_ball.x;
        aux_position_y = state.ball.y - tempo*state.v_ball.y;
        //previsao de bola
		

        if (state.v_ball.x < 0 && state.ball.x > Navigation.centroid_def.x + defender_line){
            if (robo.z > 0){
                v = (aux_position_y-robo.y)/tempo;
                if(v > v_max_gol_ef){
                    v = v_max_gol_ef;
                }
                cmd.right = v - w*l;
                cmd.left = v + w*l;
            }
            else if (robo.z < 0){
                v = -(aux_position_y-robo.y)/tempo;
                if(v > v_max_gol_ef){
                    v = v_max_gol_ef;
                }
                else if(v < -v_max_gol_ef){
                    v = -v_max_gol_ef;
                }
            	cmd.right = v - w*l;
                cmd.left = v + w*l;
            }
            cout << "Prevision Def < Atk" << endl;
        }
		// else if (state.ball.x < Navigation.centroid_def.x + 130 && state.ball.x > Navigation.centroid_def.x + defender_line && robo.x < Navigation.centroid_def.x + defender_line + 10 && robo.x > Navigation.centroid_def.x + defender_line - 10){
        //     //FollowBall            
		// 	if (fabs(state.v_ball.y) < 40)
        //     {
        //         //ball_v.y = (ball_v.y / fabs(ball_v.y)) * 0.4;
        //         if (robo.z > 0 && state.ball.y < robo.y ){
        //             v = fabs(state.v_ball.y) - 0.01*(state.ball.y-robo.y);
        //         }
        //         else if (robo.z > 0 && state.ball.y > robo.y){
        //             v = -fabs(state.v_ball.y) - 0.01*(state.ball.y-robo.y);
        //         }
        //         else if (robo.z < 0 && state.ball.y < robo.y ){
        //             v = -fabs(state.v_ball.y) + 0.01*(state.ball.y-robo.y);
        //         }
        //         else if (robo.z < 0 && state.ball.y > robo.y ){
        //             v = fabs(state.v_ball.y) + 0.01*(state.ball.y-robo.y);
        //         }
        //         cout << "FollowBall" << endl;
        //     }
        //     cmd.right = v - w*l;
        //     cmd.left = v + w*l;
        // }
        else{

	angulation_robot_goal = Navigation.get_angle();
	
	alpha = angulation_robot_goal - (robo.z-180);
	alpha = ajusta_angulo(alpha);

	// PID
    if (fabs(alpha) <= limiar_theta ){
        v = -v_delta*fabs(alpha)/limiar_theta + v_max;
        w = kp*alpha/180 + kd*(alpha - last_phi);
        limiar_theta = 90 - delta_limiar;
    }
    else{
        alpha = ajusta_angulo(alpha+180);
        v = v_delta*fabs(alpha)/limiar_theta - v_max;
        w = kp*alpha/180 + kd*(alpha - last_phi);
        limiar_theta = 90 + delta_limiar;
    }
	last_phi = alpha;

            if (fabs(alpha) > 65 && fabs(alpha) < 115){
                v = 0;
            }
            cmd.right = v - w*l;
            cmd.left = v + w*l;

            //AdjustRobo
	if((distancePoint(robo,Navigation.meta_fake_cph) < 7) && (fabs(robo.z) > 85) && (fabs(robo.z) < 95)){
		cmd.left = 0;
		cmd.right = 0;
	}else if(distancePoint(robo,Navigation.meta_fake_cph) < 7){}
        }
    }
	else{
        double tempo;
        double aux_position_y;
        tempo = (robo.x - 4.5 - state.ball.x)/state.v_ball.x;
        aux_position_y = state.ball.y - tempo*state.v_ball.y;
        //previsao de bola
        if (state.v_ball.x > 0 && state.ball.x < Navigation.centroid_def.x - defender_line){
            if (robo.z > 0){
                v = (aux_position_y-robo.y)/tempo;
                if(v > v_max_gol_ef){
                    v = v_max_gol_ef;
                }
                cmd.right = v - w*l;
                cmd.left = v + w*l;
            }
            else if (robo.z < 0){
                v = -(aux_position_y-robo.y)/tempo;
                if(v > v_max_gol_ef){
                    v = v_max_gol_ef;
                }
                else if(v < -v_max_gol_ef){
                    v = -v_max_gol_ef;
                }
            	cmd.right = v - w*l;
                cmd.left = v + w*l;
            }
            cout << "Prevision Atk < Def" << endl;
        }
        // else if (state.ball.x > Navigation.centroid_def.x - 130 && state.ball.x < Navigation.centroid_def.x - defender_line && robo.x < Navigation.centroid_def.x + defender_line + 10 && robo.x > Navigation.centroid_def.x - defender_line - 10){
        //     //FollowBall
        //     if (fabs(state.v_ball.y) < 100)
        //     {
        //         //ball_v.y = (ball_v.y / fabs(ball_v.y)) * 0.4;
        //         if (robo.z > 0 && state.ball.y < robo.y ){
        //             v = fabs(state.v_ball.y) - 0.01*(state.ball.y-robo.y);
        //         }
        //         else if (robo.z > 0 && state.ball.y > robo.y){
        //             v = -fabs(state.v_ball.y) - 0.01*(state.ball.y-robo.y);
        //         }
        //         else if (robo.z < 0 && state.ball.y < robo.y ){
        //             v = -fabs(state.v_ball.y) + 0.01*(state.ball.y-robo.y);
        //         }
        //         else if (robo.z < 0 && state.ball.y > robo.y ){
        //             v = fabs(state.v_ball.y) + 0.01*(state.ball.y-robo.y);
        //         }
        //         cout << "FollowBall" << endl;
        //     }
        //     cmd.right = v - w*l;
        //     cmd.left = v + w*l;
        // }
        else{
            //Return2Goal
	angulation_robot_goal = Navigation.get_angle();
	alpha = angulation_robot_goal - (robo.z-180);
	alpha = ajusta_angulo(alpha);

	// PID
    if (fabs(alpha) <= limiar_theta ){
        v = -v_delta*fabs(alpha)/limiar_theta + v_max;
        w = kp*alpha/180 + kd*(alpha - last_phi);
        limiar_theta = 90 - delta_limiar;
    }
    else{
        alpha = ajusta_angulo(alpha+180);
        v = v_delta*fabs(alpha)/limiar_theta - v_max;
        w = kp*alpha/180 + kd*(alpha - last_phi);
        limiar_theta = 90 + delta_limiar;
    }
	last_phi = alpha;


            if (fabs(alpha) > 65 && fabs(alpha) < 115){
                v = 0;
            }
            cmd.right = v - w*l;
            cmd.left = v + w*l;

        }
	}


//   if (Navigation.centroid_def.x < Navigation.centroid_atk.x){
//             //FollowBall            
// //if (state.ball.x < Navigation.centroid_def.x + 130 && state.ball.x > Navigation.centroid_def.x + defender_line && robo.x < Navigation.centroid_def.x + defender_line + 10 && robo.x > Navigation.centroid_def.x + defender_line - 10){
// 			if (fabs(state.v_ball.y) < 100)
//             {
//                 if (robo.z > 0 && state.ball.y < robo.y ){
//                     v = -fabs(state.v_ball.y) - 0.01*(state.ball.y-robo.y);
//                 }
//                 else if (robo.z > 0 && state.ball.y > robo.y){
//                     v = fabs(state.v_ball.y) - 0.01*(state.ball.y-robo.y);
//                 }
//                 else if (robo.z < 0 && state.ball.y < robo.y ){
//                     v = fabs(state.v_ball.y) + 0.01*(state.ball.y-robo.y);
//                 }
//                 else if (robo.z < 0 && state.ball.y > robo.y ){
//                     v = -fabs(state.v_ball.y) + 0.01*(state.ball.y-robo.y);
//                 }
//                 cout << "FollowBall" << endl;
//            }
//             cmd.right = v - w*l;
//             cmd.left = v + w*l;
// //}
//   }
//   else{
// //if (state.ball.x > Navigation.centroid_def.x - 130 && state.ball.x < Navigation.centroid_def.x - defender_line && robo.x < Navigation.centroid_def.x + defender_line + 10 && robo.x > Navigation.centroid_def.x - defender_line - 10){
//             // FollowBall
//            if (fabs(state.v_ball.y) < 100)
//            {
//                 if (robo.z > 0 && state.ball.y < robo.y ){
//                     v = -fabs(state.v_ball.y) - 0.01*(state.ball.y-robo.y);
//                 }
//                 else if (robo.z > 0 && state.ball.y > robo.y){
//                     v = fabs(state.v_ball.y) - 0.01*(state.ball.y-robo.y);
//                 }
//                 else if (robo.z < 0 && state.ball.y < robo.y ){
//                     v = fabs(state.v_ball.y) + 0.01*(state.ball.y-robo.y);
//                 }
//                 else if (robo.z < 0 && state.ball.y > robo.y ){
//                     v = -fabs(state.v_ball.y) + 0.01*(state.ball.y-robo.y);
//                 }
//                 cout << "FollowBall" << endl;
//            }
//             cmd.right = v - w*l;
//             cmd.left = v + w*l;
//         //	}
//   }

// Rotate
	if (Navigation.centroid_atk.x > Navigation.centroid_def.x){
		if(distancePoint(robo,state.ball) < dist_giro){
			if(robo.y > state.ball.y){
				cmd.left  = 255;
				cmd.right = -255;
			}else if( robo.y < state.ball.y){
				cmd.left  = -255;
				cmd.right = 255;
			}
		}else{
			cmd.left = v+(w*l);
			cmd.right = v-(w*l);
		}
	}else{
		if(distancePoint(robo,state.ball) < dist_giro){
			if(robo.y > state.ball.y){
				cmd.left  = -255;
				cmd.right = 255;
			}else if( robo.y < state.ball.y){
				cmd.left  = 255;
				cmd.right = -255;
			}
		}else{
			cmd.left = v+(w*l);
			cmd.right = v-(w*l);
		}
	}

	changePose = true;
	return cmd;
}

common::Command Strategy::velocity_goalkepper(btVector3 robo){
	Command cmd;
	float angulation_robot_goal,alpha;
	float angulation_robot_robot_goal;
   float limiar_theta = 90 , delta_limiar = 10 , last_phi;
	float v, v_delta = 6, v_max = 50;
	float w,kp = 10, kd = 0.003 , l = 8;
	

  if (Navigation.centroid_def.x < Navigation.centroid_atk.x){
        double tempo;
        double aux_position_y;
        tempo = (robo.x - state.ball.x)/state.v_ball.x;
        aux_position_y = state.ball.y - tempo*state.v_ball.y;
        //previsao de bola
		

        if (state.v_ball.x < 0 && (aux_position_y > Navigation.centroid_def.y - 25) && (aux_position_y < Navigation.centroid_def.y + 25)){
            if (robo.z > 0){
                v = (aux_position_y-robo.y)/tempo;
                if(v > v_max_gol_ef){
                    v = v_max_gol_ef;
                }
                cmd.right = v - w*l;
                cmd.left = v + w*l;
            }
            else if (robo.z < 0){
                v = -(aux_position_y-robo.y)/tempo;
                if(v > v_max_gol_ef){
                    v = v_max_gol_ef;
                }
                else if(v < -v_max_gol_ef){
                    v = -v_max_gol_ef;
                }
            	cmd.right = v - w*l;
                cmd.left = v + w*l;
            }
            cout << "Prevision Def < Atk" << endl;
        }
		else if (state.ball.x < 75 && state.ball.y > 45 && state.ball.y < 95 && robo.x < Navigation.centroid_def.x + 20){
            //FollowBall            
			if (fabs(state.v_ball.y) < 50)
            {
                //ball_v.y = (ball_v.y / fabs(ball_v.y)) * 0.4;
			if(fabs(robo.y-state.ball.y) < 5){
                        v = 0;
                    }
                    else if (robo.z > 0 && state.ball.y < robo.y ){
                        v = 0.3; // Cinemática 0.005
                    }
                    else if (robo.z > 0 && state.ball.y > robo.y){
                        v = -0.3; //PID 0.03
                    }
                    else if (robo.z < 0 && state.ball.y < robo.y ){
                        v = -0.3;
                    }
                    else if (robo.z < 0 && state.ball.y > robo.y ){
                        v = 0.3;
                    }
			}            
			cmd.right = v - w*l;
            cmd.left = v + w*l;
        }
        else{

	angulation_robot_goal = Navigation.get_angle();
	
	alpha = angulation_robot_goal - (robo.z-180);
	alpha = ajusta_angulo(alpha);

	// PID
    if (fabs(alpha) <= limiar_theta ){
        v = -v_delta*fabs(alpha)/limiar_theta + v_max;
        w = kp*alpha/180 + kd*(alpha - last_phi);
        limiar_theta = 90 - delta_limiar;
    }
    else{
        alpha = ajusta_angulo(alpha+180);
        v = v_delta*fabs(alpha)/limiar_theta - v_max;
        w = kp*alpha/180 + kd*(alpha - last_phi);
        limiar_theta = 90 + delta_limiar;
    }
	last_phi = alpha;

            if (fabs(alpha) > 65 && fabs(alpha) < 115){
                v = 0;
            }
            cmd.right = v - w*l;
            cmd.left = v + w*l;

	if((distancePoint(robo,Navigation.meta_fake_cph) < 7) && (fabs(robo.z) > 85) && (fabs(robo.z) < 95)){
		cmd.left = 0;
		cmd.right = 0;
	}else if(distancePoint(robo,Navigation.meta_fake_cph) < 7){

	}
	}
    }
	else{
        double tempo;
        double aux_position_y;
        tempo = (robo.x - state.ball.x)/state.v_ball.x;
        aux_position_y = state.ball.y - tempo*state.v_ball.y;
        //previsao de bola
        if (state.v_ball.x > 0 && (aux_position_y > Navigation.centroid_def.y - 25) && (aux_position_y < Navigation.centroid_def.y + 25)){
            if (robo.z > 0){
                v = (aux_position_y-robo.y)/tempo;
                if(v > v_max_gol_ef){
                    v = v_max_gol_ef;
                }
                cmd.right = v - w*l;
                cmd.left = v + w*l;
            }
            else if (robo.z < 0){
                v = -(aux_position_y-robo.y)/tempo;
                if(v > v_max_gol_ef){
                    v = v_max_gol_ef;
                }
                else if(v < -v_max_gol_ef){
                    v = -v_max_gol_ef;
                }
            	cmd.right = v - w*l;
                cmd.left = v + w*l;
            }
            cout << "Prevision Atk < Def" << endl;
        }
        else if (state.ball.x > 75 && state.ball.y > 45 && state.ball.y < 95 && robo.x > Navigation.centroid_def.x - 20){
            //FollowBall
            if (fabs(state.v_ball.y) < 50)
            {
                //ball_v.y = (ball_v.y / fabs(ball_v.y)) * 0.4;
                if(fabs(robo.y-state.ball.y) < 5){
                        v = 0;
                    }
                    else if (robo.z > 0 && state.ball.y < robo.y ){
                        v = 0.3; // Cinemática 0.005
                    }
                    else if (robo.z > 0 && state.ball.y > robo.y){
                        v = -0.3; //PID 0.03
                    }
                    else if (robo.z < 0 && state.ball.y < robo.y ){
                        v = -0.3;
                    }
                    else if (robo.z < 0 && state.ball.y > robo.y ){
                        v = 0.3;
                    }
                cout << "FollowBall" << endl;
            }
            cmd.right = v - w*l;
            cmd.left = v + w*l;
        }
        else{
            //Return2Goal
	angulation_robot_goal = Navigation.get_angle();
	alpha = angulation_robot_goal - (robo.z-180);
	alpha = ajusta_angulo(alpha);

	// PID
    if (fabs(alpha) <= limiar_theta ){
        v = -v_delta*fabs(alpha)/limiar_theta + v_max;
        w = kp*alpha/180 + kd*(alpha - last_phi);
        limiar_theta = 90 - delta_limiar;
    }
    else{
        alpha = ajusta_angulo(alpha+180);
        v = v_delta*fabs(alpha)/limiar_theta - v_max;
        w = kp*alpha/180 + kd*(alpha - last_phi);
        limiar_theta = 90 + delta_limiar;
    }
	last_phi = alpha;


            if (fabs(alpha) > 65 && fabs(alpha) < 115){
                v = 0;
            }
            cmd.right = v - w*l;
            cmd.left = v + w*l;

        }
	}

	//Rotate
	if (Navigation.centroid_atk.x > Navigation.centroid_def.x){
		if(distancePoint(robo,state.ball) < dist_giro){
			if(robo.y > state.ball.y){
				cmd.left  = 255;
				cmd.right = -255;
			}else if( robo.y < state.ball.y){
				cmd.left  = -255;
				cmd.right = 255;
			}
		}else{
			cmd.left = v+(w*l);
			cmd.right = v-(w*l);
		}
	}else{
		if(distancePoint(robo,state.ball) < dist_giro){
			if(robo.y > state.ball.y){
				cmd.left  = -255;
				cmd.right = 255;
			}else if( robo.y < state.ball.y){
				cmd.left  = 255;
				cmd.right = -255;
			}
		}else{
			cmd.left = v+(w*l);
			cmd.right = v-(w*l);
		}
	}
  		
	changePose = true;
	return cmd;
}

void Strategy::rotate(){

}