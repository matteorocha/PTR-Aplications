#ifndef ROBOT_H
#define ROBOT_H

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "matrix.h" // Usaremos a ADT de Matriz

// --- Constantes ---
#define ROBOT_DIAMETER 0.30 // Diâmetro do robô em metros (30cm)

// --- Estrutura de Dados ---
// Agrupa todos os vetores relevantes para o estado do robô.
typedef struct {
    Matrix* x;   // Vetor de estado
    Matrix* u;   // Vetor de entrada
    Matrix* y;   // Vetor de saída, igual a x neste caso
    Matrix* y_f; // Vetor de saída do ponto frontal do robô
} RobotState;

// --- Protótipos das Funções ---

/**
 * @brief Aloca e inicializa o estado do robô.
 * Conforme o requisito, o estado inicial é 0.
 * @return Ponteiro para a nova estrutura RobotState.
 */
RobotState* create_robot_state();

/**
 * @brief Libera toda a memória alocada para o estado do robô.
 * @param state O estado do robô a ser liberado.
 */
void free_robot_state(RobotState* state);

/**
 * @brief Atualiza o vetor de entrada u(t) com base no tempo t.
 * @param state O estado do robô (para modificar o vetor u).
 * @param t O tempo atual da simulação.
 */
void update_input(RobotState* state, double t);

/**
 * @brief Calcula o próximo estado do robô usando integração de Euler.
 * @param state O estado atual do robô (será modificado para o próximo estado).
 * @param dt O passo de tempo da simulação (delta t).
 */
void update_state(RobotState* state, double dt);

/**
 * @brief Calcula a posição do ponto frontal do robô.
 * @param state O estado atual do robô (para calcular e preencher y_f).
 */
void calculate_output_yf(RobotState* state);

#endif // ROBOT_H