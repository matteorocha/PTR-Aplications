#include <stdio.h>
#include "integral.h"

// Calcula a integral definida de uma função 'f' no intervalo [a, b]
// usando a Regra Composta do Trapézio com 'n' subintervalos.
double integral_trapezio(Func f, double a, double b, int n) {
    if (n <= 0) return 0.0;

    // 1. Calcula a largura (base) de cada trapézio.
    double h = (b - a) / n;
    
    // 2. A fórmula é (h/2) * [f(a) + f(b) + 2 * (soma dos f(x) internos)].
    // Começa a soma com os valores da função nos extremos do intervalo.
    double sum = f(a) + f(b);

    // 3. Itera pelos pontos internos do intervalo, somando 2*f(x) para cada um.
    for (int i = 1; i < n; i++) {
        double x = a + i * h;
        sum += 2.0 * f(x);
    }

    // 4. Finaliza o cálculo multiplicando a soma por h/2.
    return (h / 2.0) * sum;
}
