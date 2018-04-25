#ifndef FUZZY_H
#define FUZZY_H
#undef Sucess
#include <iostream>
#include "common.h"
#include <eigen3/Eigen/Dense>
#include "math.h"

using namespace common;
using namespace Eigen;
using namespace std; 

class Fuzzy {
private: 
    MatrixXd mi; //grau de pertinencia
    MatrixXd limite;

    VectorXd pertinencia;
    VectorXd D;
    VectorXd y_output1;
    VectorXd y_output2;
    VectorXd y_baixo;
    VectorXd y_medio1;
    VectorXd y_medio2;
    VectorXd y_alto;
    VectorXd d_universe; // Dominio dos termos primarios de entrada e saida
    VectorXd input; //vetor de entrada para calculo de decisao do fuzzy (FD,FC,FA)
    VectorXi decisao_robo;

    double output1; //saida do fuzzy novo
    double output2; //saida do fuzzy novo
    btVector3 output_meta;
    bool stop, duniverse_initialized; //variavel de controle de thread
    bool flag_finish_fuzzy;
    btVector3 centroid_atk, centroid_def; 
    btVector3 ball_pos;

public:
    Fuzzy();
    void run(); //rotina que a thread executa
    void init_duniverse();
    void init_funcao_pertinencia();
    void another_fuzzification();
    void calcula_another_input();
    void another_defuzzification();
    void set_objectives();
    double min_function(double, double);
    double max_function(double, double);
    void set_side(string);
    ~Fuzzy();
};


#endif // FUZZY_H
