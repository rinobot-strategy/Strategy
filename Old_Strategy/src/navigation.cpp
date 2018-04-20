#include "navigation.h"

 navigation::navigation():
 side("right")
 {

 }
 navigation::~navigation(){}

    float navigation::Gaussian_Func(float r){
        float delta = 3.0;
        float G;
        G = pow(M_E,(-pow(r,2)/(2*pow(delta,2))));
        //cout << endl << "G: " << G << endl;
        return G;
    }
    
    float navigation::repulsive_angle(float x, float y, btVector3 pos)
    {
        float alpha;
        if(x - pos.x < 0)
            alpha = atan((y-pos.y)/(x-pos.x))+pi;
        else
            alpha = atan((y-pos.y)/(x-pos.x));

        return alpha;
    }

    float navigation::hyperbolic_spiral(float yi, float xi, btVector3 meta)
    {
    float Kr = 7,g_size,phi;
    float theta, rho, y_aux, yl, yr , phi_cw, phi_ccw;
    float pl[2], pr[2], vec[2];
    float de = 3;
    Vector3d p(xi,yi,1),ph(0,0,0);

    Matrix3d m_trans1(3,3),m_trans2(3,3),m_rot(3,3);
    // Matriz para transladar a bola pra origem
    m_trans1 << 1, 0, -meta.x, 0, 1, -meta.y, 0, 0, 1;
    // Matriz para transladar a bola da origem pra posição original
    m_trans2 << 1 ,0 ,meta.x, 0, 1, meta.y, 0, 0, 1;

    m_rot << cos(-theta_dir),-sin(-theta_dir),0,sin(-theta_dir),cos(-theta_dir),0,0,0,1;

    ph = m_trans2*m_rot*m_trans1*p;

    pl[0] = ph(0);
    pl[1] = ph(1) - de;
    pr[0] = ph(0);
    pr[1] = ph(1) + de;

    y_aux = ph(1) - meta.y;

    yl = y_aux + de;
    yr = y_aux - de;


    rho = sqrt(pow(pl[0]-meta.x,2)+pow(pl[1]-meta.y,2));
    theta = atan2(pl[1]-meta.y,pl[0]-meta.x);

    if (rho > de){
        phi_ccw = theta + pi*(2-((de+Kr)/(rho+Kr)))/2;
    }else{
        phi_ccw = theta + pi*sqrt(rho/de)/2;
    }

    rho = sqrt(pow(pr[0]-meta.x,2)+pow(pr[1]-meta.y,2));
    theta = atan2(pr[1]-meta.y,pr[0]-meta.x);

    if (rho > de){
        phi_cw = theta - pi*(2-((de+Kr)/(rho+Kr)))/2;
    }else{
        phi_cw = theta - pi*sqrt(rho/de)/2;
    }

    vec[0] = (yl*cos(phi_ccw) - yr*cos(phi_cw))/(2*de);
    vec[1] = (yl*sin(phi_ccw) - yr*sin(phi_cw))/(2*de);

    phi = atan2(vec[1],vec[0]) + theta_dir;

    return phi;

    }

    void navigation::generate_univector(float yi, float xi, Robot robo, btVector3 meta, btVector3 enemy){
    
    float k0 = 5;
    float d_min = 10;   // Raio de Influencia do repulsivo
    float r = 15;   // Raio de Influencia do repulsivo
    float norma_s,fih_AUF,fih_TUF;
    float d = distancePoint(robo.pose, enemy);  //distancia entre o robo e o obstaculo
    btVector3 s, enemy_vel, robo_vel, virtual_obj;
    enemy_vel.x = 0;
    enemy_vel.y = 0;

    robo_vel.x = robo.v_pose.x;
    robo_vel.y = robo.v_pose.y;
    //cout << robo_vel.x << " - " << robo_vel.y << endl;
    s.x = k0 * ( enemy_vel.x - robo_vel.x); // Velocidade
    s.y = k0 * (enemy_vel.y - robo_vel.y);
       
    norma_s = sqrt(pow(s.x,2)+ pow(s.y,2));

       if (d >= norma_s)
       {
           virtual_obj.x = enemy.x + s.x;
           virtual_obj.y = enemy.y + s.y;
       }
       else
       {
           virtual_obj.x = enemy.x + (d*s.x/norma_s);
           virtual_obj.y = enemy.y + (d*s.y/norma_s);
       }
//  std::cout << "EX:" << enemy.x << std::endl;
//     std::cout << "EY:" << enemy.y << std::endl;
//     std::cout << "RX:" << robo.pose.x << std::endl;
//     std::cout << "RY:" << robo.pose.y << std::endl;
//      std::cout << "X:" << virtual_obj.x << std::endl;
//     std::cout << "Y:" << virtual_obj.y << std::endl;
    
    fih_TUF = hyperbolic_spiral(robo.pose.y,robo.pose.x,meta);

    // // Repulsivo Inatel
    // fih_AUF = repulsive_angle(robo.pose.x,robo.pose.y,virtual_obj);

    // if (d <= d_min){
    //     the_fih = fih_AUF ;
    // }
    // else{
    //     the_fih = fih_AUF*Gaussian_Func(d - d_min) + fih_TUF*(1-Gaussian_Func(d - d_min));
    //  }
    // //  Fim Repulsivo Inatel

     // Repulsive Tangencial and Spiral

    fih_AUF = tangencial_repulsive(robo,meta,enemy,r);
    fih_AUF = repulsive_spiral(robo,enemy);

    if (d <= r){
        the_fih = fih_AUF ;
        }
    else{
        the_fih = fih_AUF*Gaussian_Func(d - r) + fih_TUF*(1-Gaussian_Func(d - r));
     }
    // Fim do Repulsive tangencial
//    the_fih = fih_AUF;
  the_fih = fih_TUF;

    }

void navigation::set_theta_dir(float val){
    // cout << "t_dir: " << theta_dir*(180/pi) << endl;
    theta_dir = val;
}

float navigation::tangencial_repulsive(Robot robo, btVector3 meta, btVector3 enemy, float r){
    float alpha,omega,zeta,dist_robo_obst;
    int rot;
    float R = 5;
//     omega = repulsive_angle(robo.pose.x,robo.pose.y,meta);  // Angulo entre o robo e a bola
//     zeta = repulsive_angle(enemy.x,enemy.y,meta);                  // Angulo entre o obstaculo e a bola

//    if(omega < 0 && zeta < 0){
//        if(zeta <= omega){
//            rot = -1;
//        }
//        else{
//            rot = 1;
//        }
//    }
//    else{
//        if(zeta <= omega){
//            rot = 1;
//        }
//        else{
//            rot = -1;
//        }
//    }

    dist_robo_obst = distancePoint(robo.pose,enemy);

    alpha = -atan(R/dist_robo_obst);
  //  cout << endl << "repulsive otario: " << alpha*180/pi << endl;
    return alpha;
}

float navigation::repulsive_spiral(Robot robo, btVector3 enemy){
    float d = distancePoint(robo.pose, enemy);  //distancia entre o robo e o obstaculo
    float angle = angulation(robo.pose, enemy)*(pi/180);
    float Kr = 20, de = 8;
    float out;
    
    if(fabs(d) > de){
        out = angle - ((pi/2)*(2-((de+Kr)/(fabs(d+Kr)))));
    }else{
        out = angle - ((pi/2)*(sqrt((fabs(d)/de))));
    }
    //cout << endl << out*(180/pi) << endl;
    return out;
}

void navigation::set_side(string val){
    if(val == "right"){
     centroid_atk.x = 150;
     centroid_atk.y = 65;
     centroid_def.x = 20;
     centroid_def.y = 65;
    }else{
     centroid_def.x = 150;
     centroid_def.y = 65;
     centroid_atk.x = 20;
     centroid_atk.y = 65;
    }        
}

float navigation::get_angle_cpu(){
    return the_fih;
}

float navigation::get_angle(){
    return omega;
}

void navigation::fake_cph(Robot robo, btVector3 meta){
    meta_fake_cph = meta;
    omega = angulation(meta,robo.pose);
}