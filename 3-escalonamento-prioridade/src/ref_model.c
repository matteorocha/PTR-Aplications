#include <stdlib.h>
#include "ref_model.h"

// Implementação da alocação de memória e inicialização.
RefModel* create_ref_model(double alpha1, double alpha2) {
    RefModel* model = (RefModel*) malloc(sizeof(RefModel));
    if (model == NULL) return NULL;
    
    model->y_m = create_matrix(2, 1);     // Estado inicial é 0 por padrão (calloc).
    model->dot_y_m = create_matrix(2, 1); // Derivada também começa em 0.
    model->alpha1 = alpha1;
    model->alpha2 = alpha2;
    
    return model;
}

// Implementação da liberação de memória.
void free_ref_model(RefModel* model) {
    if (model == NULL) return;
    free_matrix(model->y_m);
    free_matrix(model->dot_y_m);
    free(model);
}

// Implementação da simulação do modelo de referência.
void update_ref_model(RefModel* model, const ReferenceTrajectory* ref, double dt) {
    if (model == NULL || ref == NULL) return;

    // Extrai os valores atuais do estado do modelo e da referência.
    double ymx = model->y_m->data[0][0];
    double ymy = model->y_m->data[1][0];
    double xref = ref->ref_xy->data[0][0];
    double yref = ref->ref_xy->data[1][0];

    // Calcula as derivadas do estado do modelo, conforme as equações do PDF.
    // Equação: dot_ymx = alpha1 * (xref - ymx)
    double dot_ymx = model->alpha1 * (xref - ymx);
    // Equação: dot_ymy = alpha2 * (yref - ymy)
    double dot_ymy = model->alpha2 * (yref - ymy);

    // Armazena as derivadas calculadas na estrutura do modelo.
    model->dot_y_m->data[0][0] = dot_ymx;
    model->dot_y_m->data[1][0] = dot_ymy;

    // Aplica o método de Euler para encontrar o novo estado do modelo.
    // novo_valor = valor_antigo + derivada * passo_de_tempo
    ymx += dot_ymx * dt;
    ymy += dot_ymy * dt;

    // Armazena o novo estado na estrutura do modelo.
    model->y_m->data[0][0] = ymx;
    model->y_m->data[1][0] = ymy;
}