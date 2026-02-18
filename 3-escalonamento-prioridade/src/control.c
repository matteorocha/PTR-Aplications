#include <stdlib.h>
#include <math.h>
#include "control.h"

// Aloca memória para a estrutura e matrizes do controlador.
Controller* create_controller(double* alpha1, double* alpha2) {
    Controller* ctrl = (Controller*) malloc(sizeof(Controller));
    if (ctrl == NULL) return NULL;

    ctrl->v_control = create_matrix(2, 1);
    ctrl->u_control = create_matrix(2, 1);
    ctrl->p_alpha1 = alpha1;
    ctrl->p_alpha2 = alpha2;

    return ctrl;
}

// Libera a memória do controlador.
void free_controller(Controller* ctrl) {
    if (ctrl == NULL) return;
    free_matrix(ctrl->v_control);
    free_matrix(ctrl->u_control);
    free(ctrl);
}

// Implementa a Equação (4) do PDF: v(t) = dot_y_m + alpha * (y_m - y)
void calculate_controller_output_v(Controller* ctrl, const RobotState* robot_state, const RefModel* ref_model) {
    // Extrai os valores necessários
    double dot_ymx = ref_model->dot_y_m->data[0][0];
    double dot_ymy = ref_model->dot_y_m->data[1][0];
    double ymx = ref_model->y_m->data[0][0];
    double ymy = ref_model->y_m->data[1][0];
    double y1 = robot_state->y->data[0][0];
    double y2 = robot_state->y->data[1][0];

    // Usa os ponteiros para obter os valores atuais de alpha
    double alpha1 = *(ctrl->p_alpha1);
    double alpha2 = *(ctrl->p_alpha2);

    // Calcula os componentes de v(t)
    double v1 = dot_ymx + alpha1 * (ymx - y1);
    double v2 = dot_ymy + alpha2 * (ymy - y2);

    // Armazena o resultado no vetor v_control
    ctrl->v_control->data[0][0] = v1;
    ctrl->v_control->data[1][0] = v2;
}

// Implementa a Equação: u = L^-1 * v
void calculate_linearization_u(Controller* ctrl, const RobotState* robot_state) {
    double theta = robot_state->x->data[2][0];
    double R = ROBOT_DIAMETER / 2.0;

    // 1. Monta a matriz L(x)
    Matrix* L = create_matrix(2, 2);
    L->data[0][0] = cos(theta);
    L->data[0][1] = -R * sin(theta);
    L->data[1][0] = sin(theta);
    L->data[1][1] = R * cos(theta);

    // 2. Calcula a inversa L^-1(x)
    Matrix* L_inv = inverse(L);
    if (L_inv == NULL) {
        // Se a matriz for singular (determinante = 0), não há como controlar.
        // Apenas definimos a entrada como 0 para segurança.
        ctrl->u_control->data[0][0] = 0;
        ctrl->u_control->data[1][0] = 0;
        free_matrix(L);
        return;
    }

    // 3. Calcula u = L^-1 * v
    Matrix* u = mul_matrix(L_inv, ctrl->v_control);

    // 4. Copia o resultado para a estrutura do controlador
    ctrl->u_control->data[0][0] = u->data[0][0]; // v (velocidade linear)
    ctrl->u_control->data[1][0] = u->data[1][0]; // omega (velocidade angular)

    // 5. Libera a memória das matrizes temporárias
    free_matrix(L);
    free_matrix(L_inv);
    free_matrix(u);
}