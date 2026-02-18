#include <math.h>
#include <stdlib.h>
#include "robot.h"

// Aloca memória para a estrutura RobotState e as matrizes internas.
RobotState* create_robot_state() {
    RobotState* state = (RobotState*) malloc(sizeof(RobotState));
    if (state == NULL) return NULL;

    state->x = create_matrix(3, 1);   // Vetor de estado [Xc, Yc, theta]^T
    state->u = create_matrix(2, 1);   // Vetor de entrada [v, omega]^T
    // A saída y(t) agora é uma matriz 2x1.
    state->y = create_matrix(2, 1);   // Saída [X_frente, Y_frente]^T

    // O estado inicial x(t)=0 para t<=0 é garantido pelo calloc em create_matrix[cite: 117].
    return state;
}

// Libera toda a memória associada a um RobotState.
void free_robot_state(RobotState* state) {
    if (state == NULL) return;
    free_matrix(state->x);
    free_matrix(state->u);
    free_matrix(state->y);
    free(state);
}

// Calcula o próximo estado do robô (integração numérica).
void update_state(RobotState* state, double dt) {
    double v = state->u->data[0][0];
    double omega = state->u->data[1][0];
    double theta = state->x->data[2][0];

    // Modelo cinemático atualizado para o padrão (cos/sin).
    double dx1_dt = v * cos(theta);
    double dx2_dt = v * sin(theta);
    double dx3_dt = omega;

    // Aplica o método de integração de Euler para encontrar o novo estado.
    state->x->data[0][0] += dx1_dt * dt; // Novo Xc
    state->x->data[1][0] += dx2_dt * dt; // Novo Yc
    state->x->data[2][0] += dx3_dt * dt; // Novo theta
}

// Calcula a posição da frente do robô (saída y(t)).
void calculate_output_y(RobotState* state) {
    double xc = state->x->data[0][0];
    double yc = state->x->data[1][0];
    double theta = state->x->data[2][0];
    // Raio R atualizado para 0.3m (D=0.6m).
    double radius = ROBOT_DIAMETER / 2.0;

    // Nova equação de saída
    state->y->data[0][0] = xc + radius * cos(theta); // Posição X da frente
    state->y->data[1][0] = yc + radius * sin(theta); // Posição Y da frente
}