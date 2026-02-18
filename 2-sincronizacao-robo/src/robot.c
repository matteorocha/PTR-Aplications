#include <math.h>
#include <stdlib.h>
#include "robot.h"

// Implementação da criação e inicialização do estado do robô
RobotState* create_robot_state() {
    RobotState* state = (RobotState*) malloc(sizeof(RobotState));
    if (state == NULL) return NULL;

    state->x = create_matrix(3, 1);   // x = [Xc, Yc, theta]^T
    state->u = create_matrix(2, 1);   // u = [v, omega]^T
    state->y = create_matrix(3, 1);   // y é uma cópia de x
    state->y_f = create_matrix(3, 1); // Posição frontal [Xf, Yf, theta]^T

    // calloc em create_matrix já garante que tudo começa em 0, conforme
    return state;
}

// Implementação da liberação de memória
void free_robot_state(RobotState* state) {
    if (state == NULL) return;
    free_matrix(state->x);
    free_matrix(state->u);
    free_matrix(state->y);
    free_matrix(state->y_f);
    free(state);
}

// Implementação da atualização da entrada u(t)
void update_input(RobotState* state, double t) {
    double v = 0.2; // Velocidade linear é constante
    double omega;

    // Lógica para a velocidade angular omega, conforme a fórmula
    if (t < 10.0) {
        omega = 0.2 * M_PI;
    } else {
        omega = -0.2 * M_PI;
    }

    state->u->data[0][0] = v;
    state->u->data[1][0] = omega;
}

// Implementação da atualização do estado x(t)
void update_state(RobotState* state, double dt) {
    double v = state->u->data[0][0];
    double omega = state->u->data[1][0];
    double theta = state->x->data[2][0];

    // Modelo cinemático exatamente como no PDF
    // x1_ponto = v * sin(theta)
    // x2_ponto = v * cos(theta)
    double dx1_dt = v * sin(theta);
    double dx2_dt = v * cos(theta);
    double dx3_dt = omega;

    // Integração de Euler
    state->x->data[0][0] += dx1_dt * dt; // Novo Xc
    state->x->data[1][0] += dx2_dt * dt; // Novo Yc
    state->x->data[2][0] += dx3_dt * dt; // Novo theta

    // A saída y(t) é o próprio estado x(t)
    for (int i = 0; i < 3; i++) {
        state->y->data[i][0] = state->x->data[i][0];
    }
}


// Implementação do cálculo da saída yf(t)
void calculate_output_yf(RobotState* state) {
    double xc = state->x->data[0][0];
    double yc = state->x->data[1][0];
    double theta = state->x->data[2][0];
    double radius = ROBOT_DIAMETER / 2.0;

    state->y_f->data[0][0] = xc + radius * sin(theta); // Posição X frontal
    state->y_f->data[1][0] = yc + radius * cos(theta); // Posição Y frontal
    state->y_f->data[2][0] = theta;                    // A orientação é a mesma
}