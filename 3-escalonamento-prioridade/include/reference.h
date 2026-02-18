#ifndef REFERENCE_H
#define REFERENCE_H

#include "matrix.h"

// Estrutura para armazenar o vetor de referência [xref, yref]^T
typedef struct {
    Matrix* ref_xy; // Matriz 2x1
} ReferenceTrajectory;

// --- Protótipos das Funções ---

// Aloca memória para a estrutura da trajetória de referência.
ReferenceTrajectory* create_reference_trajectory();

// Libera a memória alocada.
void free_reference_trajectory(ReferenceTrajectory* ref);

/**
 * @brief Calcula os valores de xref e yref para um dado tempo t.
 * @param ref Ponteiro para a estrutura que armazenará os resultados.
 * @param t O tempo atual da simulação em segundos.
 */
void calculate_reference(ReferenceTrajectory* ref, double t);

#endif // REFERENCE_H