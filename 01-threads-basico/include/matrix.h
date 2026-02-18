#ifndef MATRIX_H
#define MATRIX_H

// --- Estrutura de Dados ---
// Define a estrutura para representar uma matriz.
// Contém as dimensões (linhas, colunas) e um ponteiro duplo para os dados.
typedef struct {
    int rows;
    int cols;
    double **data;
} Matrix;

// --- Protótipos das Funções ---

// -- Gestão de Memória --
Matrix* create_matrix(int rows, int cols); // Aloca uma nova matriz na memória.
void free_matrix(Matrix* m);              // Liberta a memória alocada para uma matriz.

// -- Operações Aritméticas --
Matrix* add_matrix(Matrix* a, Matrix* b);     // Soma de duas matrizes.
Matrix* sub_matrix(Matrix* a, Matrix* b);     // Subtração de duas matrizes.
Matrix* mul_matrix(Matrix* a, Matrix* b);     // Multiplicação de duas matrizes.
Matrix* mul_scalar(Matrix* a, double k);      // Multiplicação de uma matriz por um escalar.
Matrix* transpose(Matrix* a);                 // Transposta de uma matriz.

// -- Operações Avançadas --
double determinant(Matrix* m);                // Calcula o determinante de uma matriz quadrada.
Matrix* inverse(Matrix* m);                   // Calcula a inversa de uma matriz quadrada.

#endif
