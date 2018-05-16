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
	changePose = true;
	dist_giro = 8.5;
	defender_line = 25;
	goalkepper_line = 10;
	v_max_gol_ef = 120;
	srand(time(NULL));


    // //Open the Serial Port at the desired hardware port. 
    // std::cout << "Tentando abrir a porta serial" << std::endl; 
    // serial_port.Open("/dev/ttyUSB0"); 
    // std::cout << "Porta serial aberta" << std::endl; 
     
    // //Abaixo, config_serial: 
    // // Set the baud rate of the serial port. 
    // serial_port.SetBaudRate(BaudRate::BAUD_57600); 
 
    // // Set the number of data bits. 
    // serial_port.SetCharacterSize(CharacterSize::CHAR_SIZE_8); 
 
    // // Turn off hardware flow control. 
    // serial_port.SetFlowControl(FlowControl::FLOW_CONTROL_NONE); 
 
    // // Disable parity. 
    // serial_port.SetParity(Parity::PARITY_NONE); 
     
    // // Set the number of stop bits. 
    // serial_port.SetStopBits(StopBits::STOP_BITS_1); 

    status_pos1 = 0;
    status_pos2 = 0;
    status_pos3 = 0;
    pwm_const = 100.0;
    limiar_theta = 90 , delta_limiar = 10;
    kp = 15, kd = 0.01 , l = 8;
    v_delta = 0.5, v_max = 0.8, v_goal_max = 0.5, v_set = 0.2;
    k = (7/v_max);
}

void Strategy::init(string main_color, bool is_debug, bool real_environment, string ip_receive_state, string ip_send_debug, string ip_send_command, string name){
	init_sample(main_color, is_debug, real_environment, ip_receive_state, ip_send_debug, ip_send_command, name);

	loop();

}

void Strategy::loop(){
    int contador;
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

        // static int cont = 0;
        // if(cont < 20){
        //     commands[0].left = 0.7;
        //     commands[0].right = 0.7;
        //     cont++;
        // }else{
        //     if(cont > 40){
        //         cont = 0;
        //     }
        //     commands[0].left = -0.7;
        //     commands[0].right = -0.7;
        //     cont++;
        // }

//        serial_port.Send_Velocities(2, commands[0].left/pwm_const + 0.01 , commands[0].right/pwm_const + 0.01 ); 
 //       cout << " R: "<< commands[0].right/pwm_const << " L: "<< commands[0].left/pwm_const << endl; 
		// DON'T REMOVE
		if(is_debug)
			send_debug();
		// DON'T REMOVE'
  //      cout << "Contador: " << contador++ << endl;
	}
}

void Strategy::calc_strategy(){
	if(main_color == "yellow")
		Navigation.set_side("right");
	else
		Navigation.set_side("left");

  // if(status_pos1 == 1)
	    killer_cpu();
  //else
  //   init_position(1);
        
//     if(status_pos2 == 1)        
         defender_root();
//     else
//        init_position(2);

//    if(status_pos3 == 1)
 	    goalkepper();
//     else
//        init_position(3);

	for(int i = 0 ; i < 3 ; i++){
		debug.robots_path[i].poses.clear();
	}
	
	debug.robots_path[0].poses.push_back(state.robots[0].pose);
	debug.robots_path[0].poses.push_back(final);
}

void Strategy::init_position(int val){
    btVector3 meta;
    if(val == 1){
        meta.x = 50;
        meta.y = 45;
        meta.z = 37;
      	commands[0] = go_to(state.robots[0].pose,meta,1);
    }
    if(val == 2){
        meta.x = 40;
        meta.y = 65;
        meta.z = 90;        
      	commands[1] = go_to(state.robots[1].pose,meta,2);
    }
    if(val == 3){
        meta.x = Navigation.centroid_def.x;
        meta.y = Navigation.centroid_def.y;
        meta.z = 90;        
      	commands[2] = go_to(state.robots[2].pose,meta,3);
    }

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
   	//angle = 180;
	Navigation.set_theta_dir(angle*(pi/180));

	if (Navigation.centroid_atk.x > Navigation.centroid_def.x){
            if ( state.ball.y < 15  || state.ball.y > 115){
                if(state.ball.y< Navigation.centroid_atk.y /*&& robo_pos.y<centroid_atk.y*/)
                    Navigation.set_theta_dir(-30*pi/180);
                if(state.ball.y>Navigation.centroid_atk.y /*&& robo_pos.y>centroid_atk.y*/)
                    Navigation.set_theta_dir(30*pi/180);
            }
            else{
                btVector3 aux;
                aux.x = Navigation.centroid_atk.x;
                aux.y = state.robots[0].pose.y;
                float theta = angulation(state.ball,state.robots[0].pose);
                float cat_ad = distancePoint(state.robots[0].pose,aux);
                float cat_op = cat_ad*tan(theta*pi/180);
                float add_pose = cat_op - (65 - state.robots[0].pose. y);
                btVector3 ponto;
                ponto.y = state.robots[0].pose.y + add_pose;
                ponto.x = Navigation.centroid_atk.x;
    
                if((ponto.y < 85) && (ponto.y > 45)){
                    Navigation.set_theta_dir((theta)*M_PI/180);                    
                   // ponto.show();
                }else{
                    Navigation.set_theta_dir((angle)*M_PI/180);  // Seta a orientação do Univector Field
                }
 

                //    cout << "CPU" << endl;
                if(state.ball.x > 150){
                    if(state.ball.y < Navigation.centroid_atk.y)
                        Navigation.set_theta_dir(30*pi/180);
                    if(state.ball.y > Navigation.centroid_atk.y)
                        Navigation.set_theta_dir(-30*pi/180);
                }
        }
    }
    else{
            if (state.ball.y < 20  || state.ball.y > 110){ //antes 15 e 115

                if(state.ball.y<Navigation.centroid_atk.y /*&& robo_pos.y<centroid_atk.y*/)
                    Navigation.set_theta_dir(-150*pi/180);
                if(state.ball.y>Navigation.centroid_atk.y /*&& robo_pos.y>centroid_atk.y*/)
                    Navigation.set_theta_dir(150*pi/180);
            }
            else{
                Navigation.set_theta_dir((angle)*M_PI/180);  // Seta a orientação do Univector Field                //     cout << "CPU" << endl;
                if(state.ball.x < 20){
                    if(state.ball.y < Navigation.centroid_atk.y)
                        Navigation.set_theta_dir(150*pi/180);
                    if(state.ball.y > Navigation.centroid_atk.y)
                        Navigation.set_theta_dir(-150*pi/180);
                }
            }
    }

	float d1,d2,d3;
	btVector3 enemy_prox;

	d1 = distancePoint(state.robots[0].pose,state.robots[1].pose);
	d2 = distancePoint(state.robots[0].pose,state.robots[2].pose);
	d3 = distancePoint(state.robots[0].pose,state.robots[2].pose);

 if(d1 <= d2 && d1<=d3)
	enemy_prox = state.robots[1].pose;
 if(d2 <= d1 && d2<=d3)
	enemy_prox = state.robots[2].pose;  
if(d3 <= d2 && d3<=d1)
	enemy_prox = state.robots[2].pose;

// enemy_prox = state.robots[2].pose;

	Navigation.generate_univector(state.robots[0].pose.y,state.robots[0].pose.x, state.robots[0], state.ball,enemy_prox);

	commands[0] = velocity_killer_cpu(state.robots[0].pose);
//    cout << endl << "R: "<< commands[0].right << " L: " << commands[0].left << endl;

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

	commands[1] = velocity_defender_root(state.robots[1].pose, state.robots[1].v_pose);
	
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
   float last_phi;
	float v;
	float w;

	angulation_robot_goal = Navigation.get_angle_cpu()*(180/pi);
	//cout << "X: " << state.robots[3].pose.x << "Y: " << state.robots[3].pose.y << "Z: " << state.robots[3].pose.z << endl;
	alpha = angulation_robot_goal - (robo.z-180);
	alpha = ajusta_angulo(alpha);
	
	// cout << "Direct: " << angulation_robot_goal << endl;
	// cout << "Robot: " << robo.z - 180 << endl;
	// cout << "Result: " << alpha << endl;
	// cout << endl << "x: " << state.robots[0].pose.x << " Vy: " << state.robots[0].pose.y << endl;

// //    Navegação com cinemática

//        if (fabs(alpha) <= limiar_theta){
//            w = (k*v_max*alpha/180);
//            //w = k*v_max*pow(alpha/180,2)*(alpha/fabs(alpha));
//            v = -v_max*fabs(alpha)/limiar_theta + v_max;
//            limiar_theta = 90 + delta_limiar;
//            cout << "Tras" << endl;
//        }
//        else{
//            alpha = ajusta_angulo(alpha+180);
//            w = k*v_max*alpha/180;
//            //w = k*v_max*pow(alpha/180,2)*(alpha/fabs(alpha));
//            v = v_max*fabs(alpha)/limiar_theta - v_max;
//            limiar_theta = 90 - delta_limiar;
//            cout << "Frente" << endl;
//        }


	//PID
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

	// //PID 2
    // if (fabs(alpha) <= limiar_theta ){
    //     v = -v_delta*fabs(alpha)/limiar_theta + v_max;
    //     w = kp*alpha/180 + kd*(angulation_robot_goal - last_phi);
    //     limiar_theta = 90 - delta_limiar;
    // }
    // else{
    //     alpha = ajusta_angulo(alpha+180);
    //     v = v_delta*fabs(alpha)/limiar_theta - v_max;
    //     w = kp*alpha/180 + kd*(angulation_robot_goal - last_phi);
    //     limiar_theta = 90 + delta_limiar;
    // }
	// last_phi = angulation_robot_goal;

//     double k = 3/0.4;

// v = -v_delta*fabs(alpha)/limiar_theta + v_max;
//         limiar_theta = 90 - delta_limiar;

//         v = 0.2;
//         w = k*0.2*alpha/180;


// 	cout << "v " << v << "w " << w << endl;


    //Atk_situation 
    status_atk_situatin = 0;
    cmd = Atk_situation(robo,v,w);

	//Rotate
    if(status_atk_situatin != 1)
       cmd = rotate(robo,v,w);


	changePose = true;

	return cmd;
}

common::Command Strategy::velocity_defender_root(btVector3 robo, btVector3 robo_vels){
	Command cmd;
	float distance_robot_goal;
	float angulation_robot_goal,alpha;
	float angulation_robot_robot_goal;
    float v,last_phi,w;
    float robo_get_angle = ajusta_angulo(robo.z);
    pow(10.1, 10.1);
    cmd.right = 0;
    cmd.left= 0;


    //if (cont_fps == 0){ // Teste FPS
    
    angulation_robot_goal = Navigation.get_angle();        
    alpha = 90 - (robo_get_angle);
    alpha = ajusta_angulo(alpha);

    //PID
    if (fabs(alpha) <= limiar_theta ){
        w = kp*alpha/180 + kd*(alpha - last_phi);
        limiar_theta = 90 - delta_limiar;
    }
    else{
        alpha = ajusta_angulo(alpha+180);
        w = kp*alpha/180 + kd*(alpha - last_phi);
        limiar_theta = 90 + delta_limiar;
    }

        
        if (Navigation.centroid_atk.x > Navigation.centroid_def.x){
            if (state.ball.x > robo.x && distancePoint(Navigation.centroid_def, robo) > 30){ //Navigation.centroid_def.x - defender_line
                //FollowBall
                if (fabs(state.v_ball.y) < 40)
                {
                    if (robo_get_angle > 0 && state.ball.y < robo.y ){  
                        v = 0.4;
                    }
                    else if (robo_get_angle > 0 && state.ball.y > robo.y){
                        v = -0.4;
                    }
                    else if (robo_get_angle < 0 && state.ball.y < robo.y ){
                        v = -0.4;
                    }
                    else if (robo_get_angle < 0 && state.ball.y > robo.y ){
                        v = 0.4;
                    }
             //       cout << "Defender Old FollowBall" << endl;
                }
                else if (fabs(state.v_ball.y) > 40)
                {
                    //previsao de bola
                    double tempo;
                    double aux_position_y;
                    tempo = (robo.x + 4.5 - state.ball.x)/state.v_ball.x;
                    aux_position_y = state.ball.y - tempo*state.v_ball.y/4;

                    if (aux_position_y > 130)
                        aux_position_y = 130;
                    else if (aux_position_y < 0)
                        aux_position_y = 0;

           //         cout << "Defender Old Prevision" << endl;

                    if (robo_get_angle > 0)
                        v = (aux_position_y-robo.y)/(5*tempo);
                    else if (robo_get_angle < 0)
                        v = -(aux_position_y-robo.y)/(5*tempo);                   
                }

                if(v > v_max_gol_ef){
                    v = v_max_gol_ef;
                }
                else if(v < -v_max_gol_ef){
                    v = -v_max_gol_ef;
                }
                cmd.right = (pwm_const*v) - w*l;
                cmd.left = (pwm_const*v) + w*l;
            }
            if ( (state.ball.x < robo.x || distancePoint(Navigation.centroid_def, robo) < 30 || distancePoint(Navigation.centroid_def, robo) > 80) ){//|| (state.ball.x < robo.x && state.ball.x > 90 && distancePoint(state.ball, robo)< 30) ){
                
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

                if (fabs(alpha) > 65 && fabs(alpha) < 115){
                    v = 0;
                }

                if(v > v_max_gol_ef){
                    v = v_max_gol_ef;
                }
                else if(v < -v_max_gol_ef){
                    v = -v_max_gol_ef;
                }

                cmd.right = (pwm_const*v) - w*l;
                cmd.left = (pwm_const*v) + w*l;
               // cout << "Defender Old ReturnToGoal" << endl;

                //AdjustRobo
                if((distancePoint(robo,Navigation.meta_fake_cph) < 7) && (fabs(robo.z) > 85) && (fabs(robo.z) < 95)){
                    cmd.left = 0;
                    cmd.right = 0;
                }else if(distancePoint(robo,Navigation.meta_fake_cph) < 7){
                    //commands[1] = goalkepper_orientation(state.robots[1].pose, last_phi);
                }
            }
        }

        if (Navigation.centroid_atk.x < Navigation.centroid_def.x){
            if (state.ball.x < robo.x && distancePoint(Navigation.centroid_def, robo) > 30){ //Navigation.centroid_def.x - defender_line
                //FollowBall
                if (fabs(state.v_ball.y) < 40)
                {
                    if (robo_get_angle > 0 && state.ball.y < robo.y ){  
                        v = 0.4;
                    }
                    else if (robo_get_angle > 0 && state.ball.y > robo.y){
                        v = -0.4;
                    }
                    else if (robo_get_angle < 0 && state.ball.y < robo.y ){
                        v = -0.4;
                    }
                    else if (robo_get_angle < 0 && state.ball.y > robo.y ){
                        v = 0.4;
                    }
                 //   cout << "Defender Old FollowBall" << endl;
                }
                else if (fabs(state.v_ball.y) > 40)
                {
                    //previsao de bola
                    double tempo;
                    double aux_position_y;
                    tempo = (robo.x + 4.5 - state.ball.x)/state.v_ball.x;
                    aux_position_y = state.ball.y - tempo*state.v_ball.y/4;

                    if (aux_position_y > 130)
                        aux_position_y = 130;
                    else if (aux_position_y < 0)
                        aux_position_y = 0;

                   // cout << "Defender Old Prevision" << endl;

                    if (robo_get_angle > 0)
                        v = (aux_position_y-robo.y)/(5*tempo);
                    else if (robo_get_angle < 0)
                        v = -(aux_position_y-robo.y)/(5*tempo);                   
                }

                if(v > v_max_gol_ef){
                    v = v_max_gol_ef;
                }
                else if(v < -v_max_gol_ef){
                    v = -v_max_gol_ef;
                }
                cmd.right = (pwm_const*v) - w*l;
                cmd.left = (pwm_const*v) + w*l;
            }
            if ( (state.ball.x > robo.x || distancePoint(Navigation.centroid_def, robo) < 30 || distancePoint(Navigation.centroid_def, robo) > 80) ){//|| (state.ball.x < robo.x && state.ball.x > 90 && distancePoint(state.ball, robo)< 30) ){
                
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

                if (fabs(alpha) > 65 && fabs(alpha) < 115){
                    v = 0;
                }

                if(v > v_max_gol_ef){
                    v = v_max_gol_ef;
                }
                else if(v < -v_max_gol_ef){
                    v = -v_max_gol_ef;
                }

                cmd.right = (pwm_const*v) - w*l;
                cmd.left = (pwm_const*v) + w*l;
                //cout << "Defender Old ReturnToGoal" << endl;

                //AdjustRobo
                if((distancePoint(robo,Navigation.meta_fake_cph) < 7) && (fabs(robo.z) > 85) && (fabs(robo.z) < 95)){
                    cmd.left = 0;
                    cmd.right = 0;
                }else if(distancePoint(robo,Navigation.meta_fake_cph) < 7){
                    //commands[1] = goalkepper_orientation(state.robots[1].pose, last_phi);
                }
            }
        }

        
    /*// Impedir travamento
    float d0, d2, d3, d4, d5;
	int state_robo_prox = 0;

	d0 = distancePoint(state.robots[1].pose,state.robots[0].pose);
    d2 = distancePoint(state.robots[1].pose,state.robots[2].pose);
    d3 = distancePoint(state.robots[1].pose,state.robots[3].pose);
	d4 = distancePoint(state.robots[1].pose,state.robots[4].pose);
	d5 = distancePoint(state.robots[1].pose,state.robots[5].pose);

    if((d0 || d2 || d3 || d4 || d5) < 8)
	    state_robo_prox = 1; 
         
    double velocity_linear_robo = (sqrt(pow(robo_vels.x,2) + pow(robo_vels.y,2)))/100; 
        
        
    if(state_trava_defender == 1){
        cmd.left  = 255;
		cmd.right = -250;
        cont_trava_defender = cont_trava_defender - 1;
        cout << "1" << endl;
        if (cont_trava_defender == 0)
            state_trava_defender = 0;
    }
    else{
        if (fabs(v) > 0.1 && fabs(velocity_linear_robo) < 0.1 && state_robo_prox == 1){
            cont_trava_defender = cont_trava_defender + 1;
            cout << "2" << endl;
            if (cont_trava_defender == 20){
                state_trava_defender = 1;
                cout << "3" << endl;
            }
        }
        else{
            cont_trava_defender = cont_trava_defender - 1;
            cout << "4" << endl;
            if (cont_trava_defender < 0)
                cont_trava_defender = 0;
                cout << "5" << endl;
        }
    }
    cout << "Contador trava" << cont_trava_defender << endl;
    cout << "State trava defender " << state_trava_defender << endl;
    cout << "Vel calculada " << v << endl; 
    cout << "Vel medida " << velocity_linear_robo << endl;//*/


    last_phi = alpha;
    //commands[1] = kick_rotate(state.robots[1].pose);
    
    // Rotate
    cmd = rotate(robo,v,w);

    /*//Teste FPS
    //cont_fps = cont_fps + 1;
    cout << "New FPS" << endl;
    }
    else{
        cmd.left = old_cmd_left_defender;
        cmd.right = old_cmd_right_defender;
        cout << "Old FPS" << endl;
        //if (cont_fps == 1 || cont_fps == 2  )
        //    cont_fps = cont_fps +1 ;
        //else if (cont_fps == 3)
        //    cont_fps = 0;;
    }
    
    old_cmd_left_defender = cmd.left;
    old_cmd_right_defender = cmd.right;
    cout << "Cont FPS " << cont_fps << endl ;
    cout << "Vel esquerda " << cmd.left << endl;
    cout << "Vel direita " << cmd.right << endl; //*/ 

	changePose = true;
	return cmd;
}

common::Command Strategy::velocity_goalkepper(btVector3 robo){
	Command cmd;
	float angulation_robot_goal,alpha;
	float angulation_robot_robot_goal;
	float v,last_phi,w;
	

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
                cmd.right = (pwm_const*v) - w*l;
                cmd.left = (pwm_const*v) + w*l;
            }
            else if (robo.z < 0){
                v = -(aux_position_y-robo.y)/tempo;
                if(v > v_max_gol_ef){
                    v = v_max_gol_ef;
                }
                else if(v < -v_max_gol_ef){
                    v = -v_max_gol_ef;
                }
            	cmd.right = (pwm_const*v) - w*l;
                cmd.left = (pwm_const*v) + w*l;
            }
           // cout << "Prevision Def < Atk" << endl;
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
			cmd.right = (pwm_const*v) - w*l;
            cmd.left = (pwm_const*v) + w*l;
        }
        else{

	angulation_robot_goal = Navigation.get_angle();
	
	alpha = angulation_robot_goal - (robo.z-180);
	alpha = ajusta_angulo(alpha);

	// PID
    if (fabs(alpha) <= limiar_theta ){
        v = -v_delta*fabs(alpha)/limiar_theta + v_goal_max;
        w = kp*alpha/180 + kd*(alpha - last_phi);
        limiar_theta = 90 - delta_limiar;
    }
    else{
        alpha = ajusta_angulo(alpha+180);
        v = v_delta*fabs(alpha)/limiar_theta - v_goal_max;
        w = kp*alpha/180 + kd*(alpha - last_phi);
        limiar_theta = 90 + delta_limiar;
    }
	last_phi = alpha;

            if (fabs(alpha) > 65 && fabs(alpha) < 115){
                v = 0;
            }
            cmd.right = (pwm_const*v) - w*l;
            cmd.left = (pwm_const*v) + w*l;

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
                cmd.right = (pwm_const*v) - w*l;
                cmd.left = (pwm_const*v) + w*l;
            }
            else if (robo.z < 0){
                v = -(aux_position_y-robo.y)/tempo;
                if(v > v_max_gol_ef){
                    v = v_max_gol_ef;
                }
                else if(v < -v_max_gol_ef){
                    v = -v_max_gol_ef;
                }
            	cmd.right = (pwm_const*v) - w*l;
                cmd.left = (pwm_const*v) + w*l;
            }
         //   cout << "Prevision Atk < Def" << endl;
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
            cmd.right = (pwm_const*v) - w*l;
            cmd.left = (pwm_const*v) + w*l;
        }
        else{
            //Return2Goal
	angulation_robot_goal = Navigation.get_angle();
	alpha = angulation_robot_goal - (robo.z-180);
	alpha = ajusta_angulo(alpha);

	// PID
    if (fabs(alpha) <= limiar_theta ){
        v = -v_delta*fabs(alpha)/limiar_theta + v_goal_max;
        w = kp*alpha/180 + kd*(alpha - last_phi);
        limiar_theta = 90 - delta_limiar;
    }
    else{
        alpha = ajusta_angulo(alpha+180);
        v = v_delta*fabs(alpha)/limiar_theta - v_goal_max;
        w = kp*alpha/180 + kd*(alpha - last_phi);
        limiar_theta = 90 + delta_limiar;
    }
	last_phi = alpha;


            if (fabs(alpha) > 65 && fabs(alpha) < 115){
                v = 0;
            }
            cmd.right = (pwm_const*v) - w*l;
            cmd.left = (pwm_const*v) + w*l;

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
			cmd.left = (pwm_const*v)+(w*l);
			cmd.right = (pwm_const*v)-(w*l);
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
			cmd.left = (pwm_const*v)+(w*l);
			cmd.right = (pwm_const*v)-(w*l);
		}
	}
	// cmd.left = 0;
    // cmd.right = 0;
	changePose = true;
	return cmd;
}

common::Command Strategy::go_to(btVector3 robo,btVector3 meta, int stat){
float angle,alpha;
float v,last_phi,w;

Command cmd;
if((robo.x < meta.x+0.9 && robo.x > meta.x-0.9) && (robo.y < meta.y+0.9 && robo.y > meta.y-0.9)){
    meta.x = robo.x;
    meta.y = robo.y;
    if(((robo.z-180) < meta.z+1)&&((robo.z-180) > meta.z-1)){
        setStatus_pos(stat);
       // cout << "Status changed" << endl;
    }else{
       // cout << "z: " << robo.z << " m: " << meta.z << endl;
        if(fabs(robo.z-meta.z)<180){
            cmd.left = 3;
            cmd.right = -3;
        }else{
            cmd.left = -3;
            cmd.right = 3;
        }
    }    
}else{
    // PID
    angle = angulation(meta,robo);
    alpha = angle - (robo.z-180);
    alpha = ajusta_angulo(alpha);

    if (fabs(alpha) <= limiar_theta ){
        v = -v_delta*fabs(alpha)/limiar_theta + v_set;
        w = kp*alpha/180 + kd*(alpha - last_phi);
        limiar_theta = 90 - delta_limiar;
    }
    else{
        alpha = ajusta_angulo(alpha+180);
        v = v_delta*fabs(alpha)/limiar_theta - v_set;
        w = kp*alpha/180 + kd*(alpha - last_phi);
        limiar_theta = 90 + delta_limiar;
    }
	last_phi = alpha;
	cmd.left = (pwm_const*v)+(w*l);
	cmd.right = (pwm_const*v)-(w*l);
    if(distancePoint(meta,robo) < 1.8){
        cmd.left = 0;
        cmd.right = 0;
    }
}
//cout << "robo: (" << robo.x << " , " << robo.y << ")" << endl;
//cout << "ball: " << state.ball.x << "  " << state.ball.y << endl; 
	changePose = true;
return cmd;
}

void Strategy::setStatus_pos(int val){
    if(val == 1)
        status_pos1 = 1;
    if(val == 2)
        status_pos2 = 1;
    if(val == 3)
        status_pos3 = 1;
}

common::Command Strategy::rotate(btVector3 robo, float v, float w){
	Command cmd;
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
			cmd.left = (pwm_const*v)+(w*l);
			cmd.right = (pwm_const*v)-(w*l);
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
			cmd.left = (pwm_const*v)+(w*l);
			cmd.right = (pwm_const*v)-(w*l);
		}
	}
    return cmd;
}

common::Command Strategy::Atk_situation(btVector3 robo, float v, float w){
	Command cmd;
    btVector3 min,max;
    min.x = Navigation.centroid_atk.x;
    max.x = Navigation.centroid_atk.x;
    min.y = Navigation.centroid_atk.y - 20;
    max.y = Navigation.centroid_atk.y + 20; 
    float ang_min = angulation(min,state.ball);
    float ang_max = angulation(max,state.ball);
    float dist = distancePoint(robo,state.ball);
    float angle = angulation(state.ball, robo);

//    cout << endl <<" ang_min: " << ang_min << " ang_max: " << ang_max << " ang " << angle << endl; 
    if ((dist <= 12)&&((angle < ang_max) && (angle > ang_min))){
        status_atk_situatin = 1;
        //cout << "asda " << angle << endl;
        if(robo.z > 135+angle && robo.z < 225+angle){
            v = 1.8; w = 0;
         cout << "Atk_Situation" << endl;    
        }else if(robo.z > 315+angle || robo.z < 45+angle) {
            v = -1.8; w = 0;
          cout << "Atk_Situation2" << endl;    
        }
    }
    cmd.left = (pwm_const*v)+(w*l);
    cmd.right = (pwm_const*v)-(w*l);
    return cmd;
}