#include <stdio.h>
#include "matrix.h"
#include "integral.h"

// Função de exemplo para ser integrada: f(x) = x².
double f(double x) {
    return x*x;
}

// Função auxiliar para imprimir uma matriz de forma legível no terminal.
void print_matrix(Matrix* m) {
    if (m == NULL) {
        printf("Matriz inválida ou nula.\n");
        return;
    }
    for (int i = 0; i < m->rows; i++) {
        for (int j = 0; j < m->cols; j++) {
            printf("%8.2f ", m->data[i][j]);
        }
        printf("\n");
    }
}

// Função principal que executa os testes das ADTs.
int main() {
    printf("--- INICIANDO TESTES DO AMBIENTE ---\n\n");

    // --- Bloco de Testes da ADT Matrix ---
    printf("===== Testes da ADT Matrix =====\n");

    // Criação de duas matrizes 2x2 para os testes.
    Matrix* A = create_matrix(2, 2);
    Matrix* B = create_matrix(2, 2);

    // Preenchimento das matrizes com dados de exemplo.
    A->data[0][0] = 1; A->data[0][1] = 2;
    A->data[1][0] = 3; A->data[1][1] = 4;

    B->data[0][0] = 5; B->data[0][1] = 6;
    B->data[1][0] = 7; B->data[1][1] = 8;

    printf("Matriz A:\n"); print_matrix(A);
    printf("Matriz B:\n"); print_matrix(B);

    // Execução e impressão dos resultados de cada operação da ADT.
    Matrix* C = add_matrix(A, B);
    printf("\nSoma (A + B):\n"); print_matrix(C);

    Matrix* D = sub_matrix(A, B);
    printf("\nSubtração (A - B):\n"); print_matrix(D);

    Matrix* E = mul_scalar(A, 2.0);
    printf("\nMultiplicação por Escalar (A * 2.0):\n"); print_matrix(E);

    Matrix* F = mul_matrix(A, B);
    printf("\nMultiplicação (A x B):\n"); print_matrix(F);

    Matrix* G = transpose(A);
    printf("\nTransposta de A:\n"); print_matrix(G);

    double detA = determinant(A);
    printf("\nDeterminante de A = %.2f\n", detA);

    Matrix* A_inv = inverse(A);
    printf("\nInversa de A:\n");
    print_matrix(A_inv);

    // Verificação da inversa: A * A_inv deve resultar na matriz identidade.
    if (A_inv != NULL) {
        Matrix* I = mul_matrix(A, A_inv);
        printf("\nVerificação (A * A_inv):\n");
        print_matrix(I);
        free_matrix(I); // Liberta a matriz de verificação.
    }

    // --- Bloco de Testes da ADT Integral ---
    printf("\n===== Testes da ADT Integral =====\n");
    // Chama a função de integração com a função f(x), no intervalo [0, 1], com 1000 passos.
    double result = integral_trapezio(f, 0, 1, 1000);
    printf("Integral de f(x)=x^2 de 0 a 1 = %lf\n", result);


    // --- Bloco de Limpeza de Memória ---
    // É crucial libertar a memória de todas as matrizes criadas para evitar memory leaks.
    printf("\nLiberando memória alocada...\n");
    free_matrix(A);
    free_matrix(B);
    free_matrix(C);
    free_matrix(D);
    free_matrix(E);
    free_matrix(F);
    free_matrix(G);
    free_matrix(A_inv);

    printf("\n--- TESTES FINALIZADOS COM SUCESSO ---\n");

    return 0;
}
