#ifndef REF_MODEL_H
#define REF_MODEL_H

#include "matrix.h"
#include "reference.h" // Precisamos da definição da estrutura de referência

// Estrutura para o estado do Modelo de Referência
typedef struct {
    Matrix* y_m;     // Vetor de estado [ymx, ymy]^T
    Matrix* dot_y_m; // Derivada do estado [dot_ymx, dot_ymy]^T
    double alpha1;
    double alpha2;
} RefModel;

// --- Protótipos das Funções ---

// Aloca memória e inicializa o modelo de referência com os ganhos alpha.
RefModel* create_ref_model(double alpha1, double alpha2);

// Libera a memória alocada.
void free_ref_model(RefModel* model);

/**
 * @brief Atualiza o estado do modelo de referência para o próximo passo de tempo.
 * @param model A estrutura do modelo de referência (será atualizada).
 * @param ref A trajetória de referência atual (usada como entrada).
 * @param dt O passo de tempo da simulação.
 */
void update_ref_model(RefModel* model, const ReferenceTrajectory* ref, double dt);

#endif // REF_MODEL_H