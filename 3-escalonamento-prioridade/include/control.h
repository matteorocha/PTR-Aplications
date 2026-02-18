#ifndef CONTROL_H
#define CONTROL_H

#include "matrix.h"
#include "robot.h"
#include "ref_model.h"

// Estrutura para agrupar as saídas do controlador
typedef struct {
    Matrix* v_control; // Vetor v(t) calculado pelo controlador
    Matrix* u_control; // Vetor u(t) calculado pela linearização
    double* p_alpha1;   // Ponteiro para o ganho alpha1
    double* p_alpha2;   // Ponteiro para o ganho alpha2
} Controller;

// --- Protótipos ---

Controller* create_controller(double* alpha1, double* alpha2);
void free_controller(Controller* ctrl);

/**
 * @brief Calcula o vetor de controle v(t) com base no erro.
 * @param ctrl Estrutura do controlador (para armazenar o resultado v).
 * @param robot_state O estado atual do robô (para obter y(t)).
 * @param ref_model O estado atual do modelo de referência (para obter y_m e dot_y_m).
 */
void calculate_controller_output_v(Controller* ctrl, const RobotState* robot_state, const RefModel* ref_model);

/**
 * @brief Calcula a entrada linearizada u(t) a partir de v(t).
 * @param ctrl Estrutura do controlador (para obter v e armazenar u).
 * @param robot_state O estado atual do robô (para obter x(t)).
 */
void calculate_linearization_u(Controller* ctrl, const RobotState* robot_state);

#endif // CONTROL_H