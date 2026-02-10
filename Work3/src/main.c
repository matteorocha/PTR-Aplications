#define _DEFAULT_SOURCE // Habilita features do POSIX/GNU, como useconds_t

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <unistd.h>
#include <time.h>
#include <math.h>
#include <sys/select.h> // Para a função select() de input não-bloqueante

// Inclui todos os nossos módulos de código
#include "robot.h"
#include "reference.h"
#include "ref_model.h"
#include "control.h"

#define MAX_SAMPLES 700 // Define o tamanho dos arrays para armazenar as amostras de tempo

// --- "Monitores": Variáveis Globais Partilhadas e Seus Mutexes ---
// Estruturas de dados partilhadas entre as threads, cada uma protegida por um mutex.
RobotState* g_robot_state;
pthread_mutex_t g_robot_mutex;
ReferenceTrajectory* g_reference;
pthread_mutex_t g_reference_mutex;
RefModel* g_ref_model;
pthread_mutex_t g_ref_model_mutex;
Controller* g_controller;
pthread_mutex_t g_controller_mutex;
double g_alpha1 = 2.0, g_alpha2 = 2.0; // Ganhos do controlador, sintonizados para estabilidade
pthread_mutex_t g_gains_mutex;
volatile int g_simulation_running = 1; // Flag para controlar a execução das threads
volatile int g_load_thread_running = 1; // Flag específica para a thread de carga

// --- Protótipos das Funções ---
void* thread_robot_simulation(void* arg);
void* thread_linearization(void* arg);
void* thread_control(void* arg);
void* thread_ref_model_simulation(void* arg);
void* thread_reference_generation(void* arg);
void* thread_ui_and_logging(void* arg);
void* thread_carga(void* arg);
void print_computation_stats(const char* task_name, double times_ms[], int count);
void calculate_and_print_stats(double periods_ms[], int count, double nominal_period_ms);

// --- Função Principal ---
// Orquestra toda a simulação: inicializa, cria as threads, aguarda e limpa os recursos.
int main(int argc, char *argv[]) {
    int run_with_load = 0;
    const char* output_filename = "data/simulation_sem_carga.txt";

    // 1. Analisa os argumentos da linha de comando para ativar o modo "com carga".
    if (argc == 2 && strcmp(argv[1], "--carga") == 0) {
        run_with_load = 1;
        output_filename = "data/simulation_com_carga.txt";
        printf("Executando simulação COM CARGA.\n");
    } else {
        printf("Executando simulação SEM CARGA.\n");
    }

    // 2. Inicializa todas as estruturas de dados e mutexes.
    g_robot_state = create_robot_state();
    g_reference = create_reference_trajectory();
    g_ref_model = create_ref_model(g_alpha1, g_alpha2);
    g_controller = create_controller(&g_alpha1, &g_alpha2);

    pthread_mutex_init(&g_robot_mutex, NULL);
    pthread_mutex_init(&g_reference_mutex, NULL);
    pthread_mutex_init(&g_ref_model_mutex, NULL);
    pthread_mutex_init(&g_controller_mutex, NULL);
    pthread_mutex_init(&g_gains_mutex, NULL);

    // 3. Cria as threads.
    pthread_t tid_robot, tid_linear, tid_control, tid_ref_model, tid_ref_gen, tid_ui, tid_carga;
    
    printf("Iniciando todas as threads...\n");
    pthread_create(&tid_robot, NULL, thread_robot_simulation, NULL);
    pthread_create(&tid_linear, NULL, thread_linearization, NULL);
    pthread_create(&tid_control, NULL, thread_control, NULL);
    pthread_create(&tid_ref_model, NULL, thread_ref_model_simulation, NULL);
    pthread_create(&tid_ref_gen, NULL, thread_reference_generation, NULL);
    pthread_create(&tid_ui, NULL, thread_ui_and_logging, (void*)output_filename);

    if (run_with_load) {
        pthread_create(&tid_carga, NULL, thread_carga, NULL);
    }

    // 4. A thread principal espera 20 segundos.
    printf("Simulação em andamento por 20 segundos...\n");
    usleep(20 * 1000000);

    // 5. Sinaliza o término para as threads.
    printf("Finalizando simulação...\n");
    g_simulation_running = 0;
    g_load_thread_running = 0;

    // 6. Aguarda a finalização de todas as threads (join).
    // Espera a UI primeiro para evitar que ela limpe a tela sobre as estatísticas.
    pthread_join(tid_ui, NULL);
    pthread_join(tid_robot, NULL);
    pthread_join(tid_linear, NULL);
    pthread_join(tid_control, NULL);
    pthread_join(tid_ref_model, NULL);
    pthread_join(tid_ref_gen, NULL);
    if (run_with_load) {
        pthread_join(tid_carga, NULL);
    }
    printf("Todas as threads finalizaram.\n");

    // 7. Libera todos os recursos alocados.
    free_robot_state(g_robot_state);
    free_reference_trajectory(g_reference);
    free_ref_model(g_ref_model);
    free_controller(g_controller);

    pthread_mutex_destroy(&g_robot_mutex);
    pthread_mutex_destroy(&g_reference_mutex);
    pthread_mutex_destroy(&g_ref_model_mutex);
    pthread_mutex_destroy(&g_controller_mutex);
    pthread_mutex_destroy(&g_gains_mutex);

    printf("Simulação concluída com sucesso.\n");
    return 0;
}


// --- Funções das Threads ---

// Thread para gerar carga de CPU no cenário de teste.
void* thread_carga(void* arg) {
    (void)arg;
    printf("Thread de Carga iniciada.\n");
    volatile double x = 0.0;
    while (g_load_thread_running) {
        x = sin(x); // Operação computacional intensiva para ocupar um núcleo de CPU.
    }
    printf("Thread de Carga finalizada.\n");
    return NULL;
}

// Tarefa (a), Período: 30ms. Simula a física do robô.
void* thread_robot_simulation(void* arg) {
    (void)arg;
    const double period_s = 0.030;
    static double computation_times_ms[MAX_SAMPLES];
    static int sample_count = 0;
    struct timespec start_time, end_time;

    while(g_simulation_running) {
        clock_gettime(CLOCK_MONOTONIC, &start_time);
        
        pthread_mutex_lock(&g_controller_mutex);
        pthread_mutex_lock(&g_robot_mutex);

        // Copia o comando u(t) e atualiza o estado do robô.
        g_robot_state->u->data[0][0] = g_controller->u_control->data[0][0];
        g_robot_state->u->data[1][0] = g_controller->u_control->data[1][0];
        update_state(g_robot_state, period_s);
        calculate_output_y(g_robot_state);

        pthread_mutex_unlock(&g_robot_mutex);
        pthread_mutex_unlock(&g_controller_mutex);
        
        clock_gettime(CLOCK_MONOTONIC, &end_time);
        double elapsed_ms = (end_time.tv_sec - start_time.tv_sec) * 1000.0 + (end_time.tv_nsec - start_time.tv_nsec) / 1000000.0;
        if (sample_count < MAX_SAMPLES) computation_times_ms[sample_count++] = elapsed_ms;

        usleep(30000);
    }
    print_computation_stats("Thread Robô (30ms)", &computation_times_ms[1], sample_count - 1);
    return NULL;
}

// Tarefa (b), Período: 40ms. Calcula a linearização por realimentação.
void* thread_linearization(void* arg) {
    (void)arg;
    static double computation_times_ms[MAX_SAMPLES];
    static int sample_count = 0;
    struct timespec start_time, end_time;

    while(g_simulation_running) {
        clock_gettime(CLOCK_MONOTONIC, &start_time);
        
        pthread_mutex_lock(&g_controller_mutex);
        pthread_mutex_lock(&g_robot_mutex);

        // Calcula u(t) = L^-1 * v(t)
        calculate_linearization_u(g_controller, g_robot_state);

        pthread_mutex_unlock(&g_robot_mutex);
        pthread_mutex_unlock(&g_controller_mutex);
        
        clock_gettime(CLOCK_MONOTONIC, &end_time);
        double elapsed_ms = (end_time.tv_sec - start_time.tv_sec) * 1000.0 + (end_time.tv_nsec - start_time.tv_nsec) / 1000000.0;
        if (sample_count < MAX_SAMPLES) computation_times_ms[sample_count++] = elapsed_ms;

        usleep(40000);
    }
    print_computation_stats("Thread Linearização (40ms)", &computation_times_ms[1], sample_count - 1);
    return NULL;
}

// Tarefa (c), Período: 50ms. Calcula o comando de controle v(t).
void* thread_control(void* arg) {
    (void)arg;
    static double computation_times_ms[MAX_SAMPLES];
    static int sample_count = 0;
    struct timespec start_time, end_time;

    while(g_simulation_running) {
        clock_gettime(CLOCK_MONOTONIC, &start_time);
        
        pthread_mutex_lock(&g_controller_mutex);
        pthread_mutex_lock(&g_gains_mutex);
        pthread_mutex_lock(&g_ref_model_mutex);
        pthread_mutex_lock(&g_robot_mutex);

        // Calcula v(t) = dot_y_m + alpha * (y_m - y)
        calculate_controller_output_v(g_controller, g_robot_state, g_ref_model);

        pthread_mutex_unlock(&g_robot_mutex);
        pthread_mutex_unlock(&g_ref_model_mutex);
        pthread_mutex_unlock(&g_gains_mutex);
        pthread_mutex_unlock(&g_controller_mutex);
        
        clock_gettime(CLOCK_MONOTONIC, &end_time);
        double elapsed_ms = (end_time.tv_sec - start_time.tv_sec) * 1000.0 + (end_time.tv_nsec - start_time.tv_nsec) / 1000000.0;
        if (sample_count < MAX_SAMPLES) computation_times_ms[sample_count++] = elapsed_ms;

        usleep(50000);
    }
    print_computation_stats("Thread de Controle (50ms)", &computation_times_ms[1], sample_count - 1);
    return NULL;
}

// Tarefas (d) e (e), Período: 50ms. Simula o modelo de referência.
void* thread_ref_model_simulation(void* arg) {
    (void)arg;
    const double period_s = 0.050;
    static double computation_times_ms[MAX_SAMPLES];
    static int sample_count = 0;
    struct timespec start_time, end_time;

    while(g_simulation_running) {
        clock_gettime(CLOCK_MONOTONIC, &start_time);
        
        pthread_mutex_lock(&g_reference_mutex);
        pthread_mutex_lock(&g_ref_model_mutex);

        // Calcula o próximo estado do modelo de referência.
        update_ref_model(g_ref_model, g_reference, period_s);

        pthread_mutex_unlock(&g_ref_model_mutex);
        pthread_mutex_unlock(&g_reference_mutex);
        
        clock_gettime(CLOCK_MONOTONIC, &end_time);
        double elapsed_ms = (end_time.tv_sec - start_time.tv_sec) * 1000.0 + (end_time.tv_nsec - start_time.tv_nsec) / 1000000.0;
        if (sample_count < MAX_SAMPLES) computation_times_ms[sample_count++] = elapsed_ms;
        
        usleep(50000);
    }
    print_computation_stats("Thread Modelo Ref. (50ms)", &computation_times_ms[1], sample_count - 1);
    return NULL;
}

// Tarefa (f), Período: 120ms. Gera a trajetória de referência.
void* thread_reference_generation(void* arg) {
    (void)arg;
    double t = 0.0;
    const double period_s = 0.120;
    static double computation_times_ms[MAX_SAMPLES];
    static int sample_count = 0;
    struct timespec start_time, end_time;

    while(g_simulation_running) {
        clock_gettime(CLOCK_MONOTONIC, &start_time);

        pthread_mutex_lock(&g_reference_mutex);
        calculate_reference(g_reference, t);
        pthread_mutex_unlock(&g_reference_mutex);
        
        clock_gettime(CLOCK_MONOTONIC, &end_time);
        double elapsed_ms = (end_time.tv_sec - start_time.tv_sec) * 1000.0 + (end_time.tv_nsec - start_time.tv_nsec) / 1000000.0;
        if (sample_count < MAX_SAMPLES) computation_times_ms[sample_count++] = elapsed_ms;

        t += period_s;
        usleep(120000);
    }
    print_computation_stats("Thread Geração Ref. (120ms)", &computation_times_ms[1], sample_count - 1);
    return NULL;
}

// Tarefa (g), Período: 100ms. Exibe dados na UI e grava em ficheiro.
void* thread_ui_and_logging(void* filename_arg) {
    const char* output_filename = (const char*)filename_arg;
    FILE* log_file = fopen(output_filename, "w");
    if (log_file == NULL) {
        perror("Erro ao criar o ficheiro de log");
        g_simulation_running = 0;
        return NULL;
    }
    fprintf(log_file, "t(s) Xc(m) Yc(m) theta(rad) Xref(m) Yref(m)\n");
    double t = 0.0;
    const double period_s = 0.100;
    fd_set readfds;
    struct timeval timeout;
    struct timespec last_time, current_time;
    double periods_ms[MAX_SAMPLES];
    int sample_count = 0;
    clock_gettime(CLOCK_MONOTONIC, &last_time);

    while(g_simulation_running) {
        // Mede o período/jitter desta thread.
        clock_gettime(CLOCK_MONOTONIC, &current_time);
        double elapsed_ms = (current_time.tv_sec - last_time.tv_sec) * 1000.0 + (current_time.tv_nsec - last_time.tv_nsec) / 1000000.0;
        last_time = current_time;
        if (sample_count < MAX_SAMPLES) periods_ms[sample_count++] = elapsed_ms;
        
        pthread_mutex_lock(&g_gains_mutex);
        pthread_mutex_lock(&g_reference_mutex);
        pthread_mutex_lock(&g_robot_mutex);

        // Copia os dados para variáveis locais para exibição.
        double xc = g_robot_state->x->data[0][0];
        double yc = g_robot_state->x->data[1][0];
        double theta = g_robot_state->x->data[2][0];
        double xref = g_reference->ref_xy->data[0][0];
        double yref = g_reference->ref_xy->data[1][0];
        double alpha1 = g_alpha1;
        double alpha2 = g_alpha2;

        pthread_mutex_unlock(&g_robot_mutex);
        pthread_mutex_unlock(&g_reference_mutex);
        pthread_mutex_unlock(&g_gains_mutex);

        // Imprime os dados no terminal.
        system("clear");
        printf("--- Simulação Robô Lab 3 ---\n");
        printf("Tempo: %.2f s\n\n", t);
        printf("Estado do Robô:\n");
        printf("  Xc:    %+6.3f m\n", xc);
        printf("  Yc:    %+6.3f m\n", yc);
        printf("  Theta: %+6.3f rad\n\n", theta);
        printf("Referência:\n");
        printf("  X_ref: %+6.3f m\n", xref);
        printf("  Y_ref: %+6.3f m\n\n", yref);
        printf("Ganhos do Controlador:\n");
        printf("  alpha1: %.2f\n", alpha1);
        printf("  alpha2: %.2f\n\n", alpha2);
        printf(">>> Para alterar, digite novos ganhos (ex: 1.5 2.5) e pressione Enter: \n");

        // Grava os dados no ficheiro de log.
        fprintf(log_file, "%f %f %f %f %f %f\n", t, xc, yc, theta, xref, yref);
        fflush(log_file);

        // Lógica de input não-bloqueante.
        FD_ZERO(&readfds);
        FD_SET(STDIN_FILENO, &readfds);
        timeout.tv_sec = 0;
        timeout.tv_usec = 0;

        if (select(STDIN_FILENO + 1, &readfds, NULL, NULL, &timeout) > 0) {
            char buffer[100];
            double new_alpha1, new_alpha2;
            if (fgets(buffer, sizeof(buffer), stdin) != NULL) {
                if (sscanf(buffer, "%lf %lf", &new_alpha1, &new_alpha2) == 2) {
                    pthread_mutex_lock(&g_gains_mutex);
                    g_alpha1 = new_alpha1;
                    g_alpha2 = new_alpha2;
                    pthread_mutex_unlock(&g_gains_mutex);
                    printf("\n*** Ganhos atualizados para alpha1=%.2f, alpha2=%.2f ***\n", new_alpha1, new_alpha2);
                    sleep(2);
                }
            }
        }

        t += period_s;
        usleep(100000);
    }
    fclose(log_file);
    printf("Dados salvos em %s\n", output_filename);
    // Imprime as estatísticas de jitter apenas para esta thread.
    calculate_and_print_stats(&periods_ms[1], sample_count - 1, 100.0);
    return NULL;
}

// Função para calcular e imprimir estatísticas de tempo de computação.
void print_computation_stats(const char* task_name, double times_ms[], int count) {
    if (count <= 0) return;
    double sum = 0;
    double min_ms = times_ms[0];
    double max_ms = times_ms[0];
    for (int i = 0; i < count; i++) {
        sum += times_ms[i];
        if (times_ms[i] < min_ms) min_ms = times_ms[i];
        if (times_ms[i] > max_ms) max_ms = times_ms[i];
    }
    printf("\n--- Análise de Tempo de Computação: %s ---\n", task_name);
    printf("  - Mínimo: %f ms\n", min_ms);
    printf("  - Médio:  %f ms\n", sum / count);
    printf("  - Máximo: %f ms (Este é o seu 'Ci' estimado)\n", max_ms);
}

// Função para calcular e imprimir estatísticas de Período e Jitter.
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
    printf("\n--- Análise de Tempo da Tarefa de %.0fms ---\n", nominal_period_ms);
    printf("| Métrica      | Período T(k) [ms] | Jitter J(k) [ms]  |\n");
    printf("|--------------|-------------------|-------------------|\n");
    printf("| Média        | %17.6f | %17.6f |\n", mean_t, mean_j);
    printf("| Variância    | %17.6f | %17.6f |\n", variance_t, variance_j);
    printf("| Desvio Padrão| %17.6f | %17.6f |\n", std_dev_t, std_dev_j);
    printf("| Mínimo       | %17.6f | %17.6f |\n", min_t, min_j);
    printf("| Máximo       | %17.6f | %17.6f |\n", max_t, max_j);
    printf("------------------------------------------------------\n");
}