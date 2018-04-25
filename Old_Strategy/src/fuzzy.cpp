#include "fuzzy.h"
#include <iostream>
#include <iomanip>
#include <cmath>

using namespace std;

MatrixXd parameters(4,3);

//no construtor define as entradas do sistema FD, FC e FA
Fuzzy::Fuzzy(){

    stop = true;
    duniverse_initialized = false;

    mi.resize(2,4);
    limite.resize(16,101);

    pertinencia.resize(3);
    D.resize(16);
    y_output1.resize(101);
    y_output2.resize(101);
    y_baixo.resize(101);
    y_medio1.resize(101);
    y_medio2.resize(101);
    y_alto.resize(101);
    d_universe.resize(101);
    decisao_robo.resize(3);
    input.resize(2);

    parameters << -0.0755, 0.1286 , 0.3286,
                  0.1731, 0.3769, 0.5769,
                  0.4231, 0.625, 0.8269,
                  0.6716, 0.8736, 1.073;

}

Fuzzy::~Fuzzy(){

}

void Fuzzy::run(){
    if(!duniverse_initialized){
        init_duniverse();
        init_funcao_pertinencia();
        duniverse_initialized = true;
    }
    calcula_another_input();
    another_fuzzification();
    another_defuzzification();

    //set_objectives();
}

//Inicio fuzzy 4x4

//COnversão das posições para intervalo 0 1
void Fuzzy::calcula_another_input(){
    if (centroid_atk.x > centroid_def.x )/*ata
    cando para direita*/{
       input(0) = round((ball_pos.x-10)/1.5); //Arrumado para x max 150 e valor inteiro *100
       input(1) = round(ball_pos.y /1.3);
    }
    else if (centroid_atk.x <= centroid_def.x){
       input(0) = round((160 - ball_pos.x) / 1.5); //Arrumado para x max 150 e valor inteiro *100
       input(1) = round((130 - ball_pos.y) / 1.3);
    }
}

void Fuzzy::another_fuzzification(){
    int i=0, j=0, k=0, cont = 0, aux1;
    double aux2 = 0, aux3;

    //Criando matriz de possibilidade 2x4
    for(i=0;i<2;i++)
    {
        for(j=0;j<4;j++)
        {
            aux1 = input(i);
            if(j == 0)
            {
                mi(i,j) = y_baixo(aux1);
            }
            if(j == 1)
            {
                mi(i,j) = y_medio1(aux1);
            }
            if(j == 2)
            {
                mi(i,j) = y_medio2(aux1);
            }
            if(j == 3)
            {
                mi(i,j) = y_alto(aux1);
            }
        }
    }

    for(i=0;i<4;i++)
    {
        for(j=0;j<4;j++)
        {
                aux3 = min_function(mi(0,i),mi(1,j));//Mapeando póssibilidades
                D(cont) = aux3; //armazena os mim das funções de pertinencia
                cont++;
        }

    }

    for(i=0;i<cont;i++)//Percorre o vetor D
    {
        if((i >= 0 && i <= 7))
        {
            for(j=0;j<=100;j++)
            {
                limite(i,j) = min_function(D(i),y_baixo(j)); //Gera o grafico para ser gerado o max
            }
        }
        else if(i >= 8 && i <= 11){
            for(j=0;j<=100;j++)
            {
                limite(i,j) = min_function(D(i),y_medio1(j));
            }
        }
       //Referente ao no fuzzy ALTO
        else if( i >= 12 && i <= 15)
        {
            for(j=0;j<=100;j++)
            {
                limite(i,j) = min_function(D(i),y_alto(j));
            }
        }
    }

    for(i=0;i<=100;i++)
    {
        for(k=0; k < cont; k++)
        {
            aux2 = max_function(limite(k,i),aux2);
        }
        y_output1(i) = aux2;//Grafico resultante do max dos limites gerados anteriormente
        aux2 = 0;
    }

    for(i=0;i<cont;i++)//Percorre o vetor D
    {
       //if((i%2 == 0 && i <=11) || i == 14 || i == 15){
       if(i%2 == 0){
            for(j=0;j<=100;j++)
            {
                limite(i,j) = min_function(D(i),y_medio1(j));
            }
        }
        else
        {
            for(j=0;j<=100;j++)
            {
                limite(i,j) = min_function(D(i),y_medio2(j));
            }
        }
    }

    for(i=0;i<=100;i++)
    {
        for(int k = 0; k < cont; k++)
        {
            aux2 = max_function(limite(k,i),aux2);
        }
        y_output2(i) = aux2;//Grafico resultante do max dos limites gerados anteriormente
        aux2 = 0;
    }

}

void Fuzzy::another_defuzzification(){
    double sum1 = 0,sum2 = 0, aux2;
    int i,j,aux1;
    for(i=0;i<=100;i++)
    {
    //Formula que gera o centro de massa da função de maximo
        sum1 = sum1 + d_universe(i)*y_output1(i);
        sum2 = sum2 + y_output1(i);
    }
    output1 = sum1/sum2; //posição de saida em x

    sum1 = 0;
    sum2 = 0;

    for(i=0;i<=100;i++)
    {
    //Formula que gera o centro de massa da função de maximo
        sum1 = sum1 + d_universe(i)*y_output2(i);
        sum2 = sum2 + y_output2(i);
    }
    output2 = sum1/sum2; //posição de saida em y    

    //Passando saidas para gamefunction
    if(centroid_atk.x > centroid_def.x ){
       output_meta.x = output1*150 +10;
       output_meta.y = output2*130;
    }
    else if (centroid_atk.x <= centroid_def.x ){
       output_meta.x = 160 - output1*150;
       output_meta.y = 130 - output2*130;
    }

}

//Fim fuzzy 4x4

// void Fuzzy::set_objectives(){
//     int i = 0;
//         if (selec_robot.r1.get_channel() != 8 && euclidean_dist(selec_robot.r1.get_pos(),centroid_def) <= euclidean_dist(selec_robot.r2.get_pos(),centroid_def) && euclidean_dist(selec_robot.r1.get_pos(),centroid_def) <= euclidean_dist(selec_robot.r3.get_pos(),centroid_def)){
//             decisao_robo[0] = 4; // goleiro
//             if (euclidean_dist(selec_robot.r2.get_pos(),ball_pos) <= euclidean_dist(selec_robot.r3.get_pos(),ball_pos)){
//                 decisao_robo[1] = 10; // o que vai na bola (killer)
//                 decisao_robo[2] = 11; // o que guarda a posição (guardian)
//             }
//             else{
//                 decisao_robo[1] = 11;
//                 decisao_robo[2] = 10;
//             }
//         }
//         else if (selec_robot.r2.get_channel() != 8 && euclidean_dist(selec_robot.r2.get_pos(),centroid_def) <= euclidean_dist(selec_robot.r3.get_pos(),centroid_def) && euclidean_dist(selec_robot.r2.get_pos(),centroid_def) <= euclidean_dist(selec_robot.r1.get_pos(),centroid_def)){
//             decisao_robo[1] = 4; //goleiro
//             if (euclidean_dist(selec_robot.r1.get_pos(),ball_pos) <= euclidean_dist(selec_robot.r3.get_pos(),ball_pos)){
//                 decisao_robo[0] = 10; // o que vai na bola (killer)
//                 decisao_robo[2] = 11; // o que guarda a posição (guardian)
//             }
//             else{
//                 decisao_robo[0] = 11;
//                 decisao_robo[2] = 10;
//             }
//         }
//         else if( selec_robot.r3.get_channel() != 8 && euclidean_dist(selec_robot.r3.get_pos(),centroid_def) <= euclidean_dist(selec_robot.r1.get_pos(),centroid_def) && euclidean_dist(selec_robot.r3.get_pos(),centroid_def) <= euclidean_dist(selec_robot.r2.get_pos(),centroid_def)){
//             decisao_robo[2] = 4; // goleiro
//             if (euclidean_dist(selec_robot.r1.get_pos(),ball_pos) <= euclidean_dist(selec_robot.r2.get_pos(),ball_pos)){
//                 decisao_robo[0] = 10; // o que vai na bola (killer)
//                 decisao_robo[1] = 11; // o que guarda a posição (guardian)
//             }
//             else{
//                 decisao_robo[0] = 11;
//                 decisao_robo[1] = 10;
//             }
//         }

//         decisao_robo[2] = 4; // goleiro
//         if (euclidean_dist(selec_robot.r1.get_pos(),ball_pos) <= euclidean_dist(selec_robot.r2.get_pos(),ball_pos)){
//             decisao_robo[0] = 10; // o que vai na bola (killer)
//             decisao_robo[1] = 11; // o que guarda a posição (guardian)
//         }
//         else{
//             decisao_robo[0] = 11;
//             decisao_robo[1] = 10;
//         }
// }

double Fuzzy::min_function(double p, double q){
    if(p <= q)
    {
        return p;
    }
    else
        return q;
}

double Fuzzy::max_function(double p, double q){
    if(p >= q)
    {
        return p;
    }
    else
        return q;
}

void Fuzzy::init_duniverse(){
    int i;
    double aux = 0.01;
    for(i=0;i<=100;i++)
    {
        d_universe(i) = i*aux;
    }

}

void Fuzzy::init_funcao_pertinencia(){
    int i;
    for(i=0;i<=100;i++)
    {
        if(d_universe(i) < parameters(0,0) || d_universe(i) > parameters(0,2))
        {
            y_baixo(i) = 0;
        }
        else if(d_universe(i) < parameters(0,1))
        {
            y_baixo(i) = (d_universe(i) - parameters(0,0))/(parameters(0,1) - parameters(0,0));
        }
        else if(d_universe(i) >= parameters(0,1))
        {
            y_baixo(i) = (d_universe(i) - parameters(0,2))/(parameters(0,1) - parameters(0,2));
        }
    }
    for(i=0;i<=100;i++)
    {
        if(d_universe(i) < parameters(1,0) || d_universe(i) > parameters(1,2))
        {
            y_medio1(i) = 0;
        }
        else if(d_universe(i) < parameters(1,1))
        {
            y_medio1(i) = (d_universe(i) - parameters(1,0))/(parameters(1,1) - parameters(1,0));
        }
        else if(d_universe(i) >= parameters(1,1))
        {
            y_medio1(i) = (d_universe(i) - parameters(1,2))/(parameters(1,1) - parameters(1,2));
        }
    }
    for(i=0;i<=100;i++)
    {
        if(d_universe(i) < parameters(2,0) || d_universe(i) > parameters(2,2))
        {
            y_medio2(i) = 0;
        }
        else if(d_universe(i) < parameters(2,1))
        {
            y_medio2(i) = (d_universe(i) - parameters(2,0))/(parameters(2,1) - parameters(2,0));
        }
        else if(d_universe(i) >= parameters(2,1))
        {
            y_medio2(i) = (d_universe(i) - parameters(2,2))/(parameters(2,1) - parameters(2,2));
        }
    }
    for(i=0;i<=100;i++)
    {
        if(d_universe(i) < parameters(3,0) || d_universe(i) > parameters(3,2))
        {
            y_alto(i) = 0;
        }
        else if(d_universe(i) < parameters(3,1))
        {
            y_alto(i) = (d_universe(i) - parameters(3,0))/(parameters(3,1) - parameters(3,0));
        }
        else if(d_universe(i) >= parameters(3,1))
        {
            y_alto(i) = (d_universe(i) - parameters(3,2))/(parameters(3,1) - parameters(3,2));
        }
    }
}

void Fuzzy::set_side(string val){
    if(val == "right"){
     centroid_atk.x = 160;
     centroid_atk.y = 65;
     centroid_def.x = 10;
     centroid_def.y = 65;
    }else{
     centroid_def.x = 160;
     centroid_def.y = 65;
     centroid_atk.x = 10;
     centroid_atk.y = 65;
    }        
}
