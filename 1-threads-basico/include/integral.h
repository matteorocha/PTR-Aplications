#ifndef INTEGRAL_H
#define INTEGRAL_H

// --- Tipos de Dados ---
// Define um tipo "Func" como um ponteiro para uma função que recebe um double
// e retorna um double. Isso torna a função de integração genérica.
typedef double (*Func)(double);

// --- Protótipos das Funções ---

/**
 * @brief Calcula a integral definida de uma função usando a Regra do Trapézio.
 * @param f A função a ser integrada (passada como ponteiro).
 * @param a Limite inferior de integração.
 * @param b Limite superior de integração.
 * @param n Número de subintervalos (trapézios) a serem usados.
 * @return O valor aproximado da integral.
 */
double integral_trapezio(Func f, double a, double b, int n);

#endif // INTEGRAL_H
