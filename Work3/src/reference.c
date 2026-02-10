#include <math.h>
#include <stdlib.h>
#include "reference.h"
#include "robot.h" // Inclui para ter acesso a M_PI

// Implementação da alocação de memória.
ReferenceTrajectory* create_reference_trajectory() {
    ReferenceTrajectory* ref = (ReferenceTrajectory*) malloc(sizeof(ReferenceTrajectory));
    if (ref == NULL) return NULL;
    
    ref->ref_xy = create_matrix(2, 1);
    return ref;
}

// Implementação da liberação de memória.
void free_reference_trajectory(ReferenceTrajectory* ref) {
    if (ref == NULL) return;
    free_matrix(ref->ref_xy);
    free(ref);
}

// Implementação do cálculo da referência.
void calculate_reference(ReferenceTrajectory* ref, double t) {
    if (ref == NULL || ref->ref_xy == NULL) return;

    // Constante 5/pi usada em ambas as equações.
    const double factor = 5.0 / M_PI;

    // Calcula xref(t) conforme a equação do PDF
    double xref = factor * cos(0.2 * M_PI * t);

    // Calcula yref(t), que é uma função piecewise
    double yref;
    if (t < 10.0) {
        // Para 0 <= t < 10
        yref = factor * sin(0.2 * M_PI * t);
    } else {
        // Para t >= 10
        yref = -factor * sin(0.2 * M_PI * t);
    }

    // Armazena os valores calculados na matriz da estrutura.
    ref->ref_xy->data[0][0] = xref;
    ref->ref_xy->data[1][0] = yref;
}