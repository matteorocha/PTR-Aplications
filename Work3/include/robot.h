#ifndef ROBOT_H
#define ROBOT_H

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "matrix.h" // Usaremos a ADT de Matriz

// --- Constantes ---
// Diâmetro do robô atualizado para 0.6m
#define ROBOT_DIAMETER 0.60

// --- Estrutura de Dados ---
// Estrutura simplificada. y agora é a saída 2x1.
typedef struct {
    Matrix* x;   // Vetor de estado [Xc, Yc, theta]^T (3x1)
    Matrix* u;   // Vetor de entrada [v, omega]^T (2x1)
    Matrix* y;   // Vetor de saída [X_frente, Y_frente]^T (2x1)
} RobotState;

// --- Protótipos das Funções ---

RobotState* create_robot_state();
void free_robot_state(RobotState* state);

/**
 * @brief Calcula o próximo estado do robô usando integração de Euler.
 * @param state O estado atual do robô (será modificado para o próximo estado).
 * @param dt O passo de tempo da simulação (delta t).
 */
void update_state(RobotState* state, double dt);

/**
 * @brief Calcula a posição da frente do robô (saída y(t)).
 * @param state O estado atual do robô (para calcular e preencher y).
 */
void calculate_output_y(RobotState* state);

#endif // ROBOT_H