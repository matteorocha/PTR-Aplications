#define _DEFAULT_SOURCE // Habilita features do POSIX/GNU, como useconds_t

#include <stdio.h>
#include <stdlib.h>
#include <string.h>  // Adicionado para strcmp (comparação de strings)
#include <pthread.h>
#include <unistd.h>
#include <time.h>
#include <math.h>

#include "robot.h"

#define MAX_SAMPLES 700

// --- Variáveis Globais Compartilhadas ---
RobotState* g_robot_state;
pthread_mutex_t g_robot_mutex;
volatile int g_simulation_running = 1;
volatile int g_load_thread_running = 1; // Flag para a nova thread de carga

// --- Protótipos das Funções das Threads ---
void* thread_controle_io(void* filename_arg);
void* thread_simulacao(void* arg);
void* thread_carga(void* arg);
void calculate_and_print_stats(double periods_ms[], int count, double nominal_period_ms);

// --- Função Principal ---
int main(int argc, char *argv[]) {
    int run_with_load = 0;
    const char* output_filename = "data/simulation_sem_carga.txt";

    // Analisa os argumentos da linha de comando
    if (argc == 2 && strcmp(argv[1], "--carga") == 0) {
        run_with_load = 1;
        output_filename = "data/simulation_com_carga.txt";
        printf("Executando simulação COM CARGA.\n");
    } else {
        printf("Executando simulação SEM CARGA.\n");
    }

    printf("Iniciando a simulação do robô...\n");

    g_robot_state = create_robot_state();
    if (g_robot_state == NULL) {
        fprintf(stderr, "Erro ao alocar o estado do robô.\n");
        return 1;
    }
    if (pthread_mutex_init(&g_robot_mutex, NULL) != 0) {
        fprintf(stderr, "Erro ao inicializar o mutex.\n");
        free_robot_state(g_robot_state);
        return 1;
    }

    pthread_t tid1, tid2, tid_carga;
    // Passa o nome do arquivo como argumento para a thread de controle/IO
    pthread_create(&tid1, NULL, thread_controle_io, (void*)output_filename);
    pthread_create(&tid2, NULL, thread_simulacao, NULL);
    if (run_with_load) {
        pthread_create(&tid_carga, NULL, thread_carga, NULL);
    }

    double simulation_time_s = 20.0;
    printf("Simulação em andamento por %.1f segundos...\n", simulation_time_s);
    usleep((useconds_t)(simulation_time_s * 1000000));

    // Sinaliza para as threads terminarem
    g_simulation_running = 0;
    g_load_thread_running = 0;
    printf("Tempo de simulação finalizado. Aguardando threads...\n");

    // Aguarda a finalização das threads
    pthread_join(tid1, NULL);
    pthread_join(tid2, NULL);
    if (run_with_load) {
        pthread_join(tid_carga, NULL);
    }

    // Libera os recursos
    pthread_mutex_destroy(&g_robot_mutex);
    free_robot_state(g_robot_state);

    printf("Simulação finalizada com sucesso.\n");
    return 0;
}

/**
 * @brief Thread 3: Gera carga de CPU.
 */
void* thread_carga(void* arg) {
    (void)arg;
    printf("Thread de Carga iniciada.\n");
    volatile double x = 0.0;
    while (g_load_thread_running) {
        x = sin(x);
    }
    printf("Thread de Carga finalizada.\n");
    return NULL;
}

/**
 * @brief Thread 1: Gera u(t), lê y(t), armazena dados e mede performance.
 */
void* thread_controle_io(void* filename_arg) {
    const char* output_filename = (const char*)filename_arg;
    printf("Thread de Controle/IO (30ms) iniciada.\n");

    FILE* out_file = fopen(output_filename, "w");
    if (out_file == NULL) {
        perror("Erro ao criar o arquivo de saida em data/");
        g_simulation_running = 0;
        return NULL;
    }
    fprintf(out_file, "t(s) v(m/s) w(rad/s) Xc(m) Yc(m) theta(rad) Xf(m) Yf(m)\n");

    double simulation_time = 0.0;
    const double dt = 0.030;

    struct timespec last_time, current_time;
    double periods_ms[MAX_SAMPLES];
    int sample_count = 0;
    clock_gettime(CLOCK_MONOTONIC, &last_time);

    while (g_simulation_running) {
        clock_gettime(CLOCK_MONOTONIC, &current_time);
        double elapsed_ms = (current_time.tv_sec - last_time.tv_sec) * 1000.0 +
                            (current_time.tv_nsec - last_time.tv_nsec) / 1000000.0;
        last_time = current_time;
        if (sample_count < MAX_SAMPLES) {
            periods_ms[sample_count++] = elapsed_ms;
        }

        pthread_mutex_lock(&g_robot_mutex);
        update_input(g_robot_state, simulation_time);
        calculate_output_yf(g_robot_state);
        
        double v = g_robot_state->u->data[0][0];
        double w = g_robot_state->u->data[1][0];
        double xc = g_robot_state->y->data[0][0];
        double yc = g_robot_state->y->data[1][0];
        double theta = g_robot_state->y->data[2][0];
        double xf = g_robot_state->y_f->data[0][0];
        double yf = g_robot_state->y_f->data[1][0];

        pthread_mutex_unlock(&g_robot_mutex);

        fprintf(out_file, "%lf %lf %lf %lf %lf %lf %lf %lf\n",
                simulation_time, v, w, xc, yc, theta, xf, yf);

        simulation_time += dt;
        usleep(30000);
    }
    
    fclose(out_file);
    printf("Thread de Controle/IO finalizada. Dados salvos em %s\n", output_filename);
    calculate_and_print_stats(&periods_ms[1], sample_count - 1, 30.0);
    
    return NULL;
}

/**
 * @brief Thread 2: Lê u(t) e calcula o estado y(t).
 */
void* thread_simulacao(void* arg) {
    (void)arg;
    printf("Thread de Simulação (50ms) iniciada.\n");
    const double dt = 0.050;

    while (g_simulation_running) {
        pthread_mutex_lock(&g_robot_mutex);
        update_state(g_robot_state, dt);
        pthread_mutex_unlock(&g_robot_mutex); // Corrigido de g_mutex para g_robot_mutex
        usleep(50000);
    }

    printf("Thread de Simulação finalizada.\n");
    return NULL;
}

/**
 * @brief Função para calcular e imprimir as estatísticas de tempo.
 */
void calculate_and_print_stats(double periods[], int count, double nominal_period_ms) {
    if (count <= 0) return;

    double sum_t = 0, sum_j = 0;
    double min_t = periods[0], max_t = periods[0];
    double jitters[count];

    for (int i = 0; i < count; i++) {
        double p = periods[i];
        sum_t += p;
        if (p < min_t) min_t = p;
        if (p > max_t) max_t = p;
        jitters[i] = p - nominal_period_ms;
    }
    double mean_t = sum_t / count;

    double min_j = jitters[0], max_j = jitters[0];
    for (int i = 0; i < count; i++) {
        sum_j += jitters[i];
        if (jitters[i] < min_j) min_j = jitters[i];
        if (jitters[i] > max_j) max_j = jitters[i];
    }
    double mean_j = sum_j / count;

    double variance_t = 0, variance_j = 0;
    for (int i = 0; i < count; i++) {
        variance_t += pow(periods[i] - mean_t, 2);
        variance_j += pow(jitters[i] - mean_j, 2);
    }
    variance_t /= count;
    variance_j /= count;

    double std_dev_t = sqrt(variance_t);
    double std_dev_j = sqrt(variance_j);

    printf("\n--- Análise de Tempo da Tarefa de 30ms ---\n");
    printf("| Métrica      | Período T(k) [ms] | Jitter J(k) [ms]  |\n");
    printf("|--------------|-------------------|-------------------|\n");
    printf("| Média        | %17.6f | %17.6f |\n", mean_t, mean_j);
    printf("| Variância    | %17.6f | %17.6f |\n", variance_t, variance_j);
    printf("| Desvio Padrão| %17.6f | %17.6f |\n", std_dev_t, std_dev_j);
    printf("| Mínimo       | %17.6f | %17.6f |\n", min_t, min_j);
    printf("| Máximo       | %17.6f | %17.6f |\n", max_t, max_j);
    printf("------------------------------------------------------\n");
}