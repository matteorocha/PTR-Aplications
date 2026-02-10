#include <stdio.h>
#include <stdlib.h>
#include "matrix.h"

// Aloca dinamicamente uma nova matriz com 'rows' linhas e 'cols' colunas.
// Retorna um ponteiro para a matriz criada ou NULL em caso de falha.
Matrix* create_matrix(int rows, int cols) {
    // 1. Aloca memória para a estrutura principal da matriz.
    Matrix* m = (Matrix*) malloc(sizeof(Matrix));
    if (m == NULL) return NULL;

    m->rows = rows;
    m->cols = cols;

    // 2. Aloca um array de ponteiros para as linhas.
    m->data = (double**) malloc(rows * sizeof(double*));
    if (m->data == NULL) {
        free(m);
        return NULL;
    }

    // 3. Aloca um array de doubles para cada linha (as colunas).
    // calloc inicializa todos os elementos com 0.
    for (int i = 0; i < rows; i++) {
        m->data[i] = (double*) calloc(cols, sizeof(double));
        if (m->data[i] == NULL) {
            // Em caso de falha, liberta toda a memória já alocada para evitar leaks.
            for (int j = 0; j < i; j++) free(m->data[j]);
            free(m->data);
            free(m);
            return NULL;
        }
    }
    return m;
}

// Liberta toda a memória alocada para uma matriz.
void free_matrix(Matrix* m) {
    if (m == NULL) return;
    // 1. Liberta a memória de cada linha.
    for (int i = 0; i < m->rows; i++) {
        free(m->data[i]);
    }
    // 2. Liberta o array de ponteiros das linhas.
    free(m->data);
    // 3. Liberta a estrutura principal.
    free(m);
}

// Soma duas matrizes de mesmas dimensões.
Matrix* add_matrix(Matrix* a, Matrix* b) {
    if (a == NULL || b == NULL || a->rows != b->rows || a->cols != b->cols) return NULL;
    Matrix* result = create_matrix(a->rows, a->cols);
    if (result == NULL) return NULL;

    for (int i = 0; i < a->rows; i++) {
        for (int j = 0; j < a->cols; j++) {
            result->data[i][j] = a->data[i][j] + b->data[i][j];
        }
    }
    return result;
}

// Subtrai duas matrizes de mesmas dimensões.
Matrix* sub_matrix(Matrix* a, Matrix* b) {
    if (a == NULL || b == NULL || a->rows != b->rows || a->cols != b->cols) return NULL;
    Matrix* result = create_matrix(a->rows, a->cols);
    if (result == NULL) return NULL;

    for (int i = 0; i < a->rows; i++) {
        for (int j = 0; j < a->cols; j++) {
            result->data[i][j] = a->data[i][j] - b->data[i][j];
        }
    }
    return result;
}

// Multiplica cada elemento de uma matriz por um escalar 'k'.
Matrix* mul_scalar(Matrix* a, double k) {
    if (a == NULL) return NULL;
    Matrix* result = create_matrix(a->rows, a->cols);
    if (result == NULL) return NULL;

    for (int i = 0; i < a->rows; i++) {
        for (int j = 0; j < a->cols; j++) {
            result->data[i][j] = a->data[i][j] * k;
        }
    }
    return result;
}

// Multiplica duas matrizes. O número de colunas de 'a' deve ser igual ao número de linhas de 'b'.
Matrix* mul_matrix(Matrix* a, Matrix* b) {
    if (a == NULL || b == NULL || a->cols != b->rows) return NULL;
    Matrix* result = create_matrix(a->rows, b->cols);
    if (result == NULL) return NULL;

    for (int i = 0; i < result->rows; i++) {
        for (int j = 0; j < result->cols; j++) {
            for (int k = 0; k < a->cols; k++) {
                result->data[i][j] += a->data[i][k] * b->data[k][j];
            }
        }
    }
    return result;
}

// Calcula a transposta de uma matriz.
Matrix* transpose(Matrix* a) {
    if (a == NULL) return NULL;
    Matrix* result = create_matrix(a->cols, a->rows);
    if (result == NULL) return NULL;

    for (int i = 0; i < a->rows; i++) {
        for (int j = 0; j < a->cols; j++) {
            result->data[j][i] = a->data[i][j];
        }
    }
    return result;
}

// Função auxiliar (estática) para criar uma submatriz removendo uma linha e uma coluna.
// Usada no cálculo do determinante.
static Matrix* submatrix(Matrix* m, int row_to_remove, int col_to_remove) {
    Matrix* sub = create_matrix(m->rows - 1, m->cols - 1);
    if (sub == NULL) return NULL;
    int r = 0;
    for (int i = 0; i < m->rows; i++) {
        if (i == row_to_remove) continue;
        int c = 0;
        for (int j = 0; j < m->cols; j++) {
            if (j == col_to_remove) continue;
            sub->data[r][c] = m->data[i][j];
            c++;
        }
        r++;
    }
    return sub;
}

// Calcula o determinante de uma matriz quadrada usando a Expansão de Laplace.
// A função é recursiva.
double determinant(Matrix* m) {
    if (m == NULL || m->rows != m->cols) return 0.0;
    int n = m->rows;

    // Casos base da recursão.
    if (n == 1) return m->data[0][0];
    if (n == 2) return m->data[0][0] * m->data[1][1] - m->data[0][1] * m->data[1][0];

    // Passo recursivo: expande pela primeira linha.
    double det = 0.0;
    for (int j = 0; j < n; j++) {
        Matrix* sub = submatrix(m, 0, j);
        // Sinal do cofator alterna: + - + - ...
        double sign = (j % 2 == 0) ? 1.0 : -1.0;
        det += sign * m->data[0][j] * determinant(sub);
        free_matrix(sub);
    }
    return det;
}

// Função auxiliar para calcular a matriz de cofatores.
static Matrix* cofactor_matrix(Matrix* m) {
    Matrix* cofactors = create_matrix(m->rows, m->cols);
    if (cofactors == NULL) return NULL;

    for (int i = 0; i < m->rows; i++) {
        for (int j = 0; j < m->cols; j++) {
            Matrix* sub = submatrix(m, i, j);
            double det_sub = determinant(sub);
            free_matrix(sub);
            double sign = ((i + j) % 2 == 0) ? 1.0 : -1.0;
            cofactors->data[i][j] = sign * det_sub;
        }
    }
    return cofactors;
}

// Calcula a inversa de uma matriz quadrada.
// A fórmula é: A⁻¹ = (1/det(A)) * adj(A)
Matrix* inverse(Matrix* m) {
    if (m == NULL || m->rows != m->cols) return NULL;

    // 1. Calcula o determinante. Se for 0, não há inversa.
    double det = determinant(m);
    if (det == 0.0) return NULL;

    // 2. Calcula a matriz adjugada (transposta da matriz de cofatores).
    Matrix* cofactors = cofactor_matrix(m);
    Matrix* adjugate = transpose(cofactors);

    // 3. Multiplica a adjugada pelo inverso do determinante.
    double inv_det = 1.0 / det;
    Matrix* inv_matrix = mul_scalar(adjugate, inv_det);

    // 4. Liberta a memória das matrizes intermediárias.
    free_matrix(cofactors);
    free_matrix(adjugate);

    return inv_matrix;
}
