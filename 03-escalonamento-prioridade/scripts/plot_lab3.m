% Script para análise e comparação dos cenários SEM CARGA e COM CARGA
clear all;
clc;
close all;

% --- 1. Constantes e Arquivos ---
R = 0.3; % Raio do robô (D=0.6m)
file_sem_carga = "../data/simulation_sem_carga.txt";
file_com_carga = "../data/simulation_com_carga.txt";

fprintf("Carregando dados...\n");
if (exist(file_sem_carga, 'file') ~= 2 || exist(file_com_carga, 'file') ~= 2)
    error("Arquivos de simulação não encontrados. Execute './main' e './main --carga' primeiro.");
end

% --- 2. Leitura dos dados ---
data_sem = dlmread(file_sem_carga, " ", 1, 0);
data_com = dlmread(file_com_carga, " ", 1, 0);

% --- 3. Extração das 6 colunas existentes ---
t_sem = data_sem(:,1); Xc_sem = data_sem(:,2); Yc_sem = data_sem(:,3); theta_sem = data_sem(:,4);
Xref_sem = data_sem(:,5); Yref_sem = data_sem(:,6);

t_com = data_com(:,1); Xc_com = data_com(:,2); Yc_com = data_com(:,3); theta_com = data_com(:,4);
Xref_com = data_com(:,5); Yref_com = data_com(:,6);

% --- 4. Recalcular a posição da frente do robô (Xf, Yf) ---
Xf_sem = Xc_sem + R * cos(theta_sem);
Yf_sem = Yc_sem + R * sin(theta_sem);

Xf_com = Xc_com + R * cos(theta_com);
Yf_com = Yc_com + R * sin(theta_com);

fprintf("Gerando gráficos comparativos...\n");

%% --- 5. Trajetória Espacial ---
figure('Units','inches','Position',[1 1 6 3]);
hold on; grid on; axis equal;

% Plota as trajetórias da FRENTE do robô (Yf vs Xf)
plot(Yf_sem, Xf_sem, 'b-', 'LineWidth', 1.5);
plot(Yf_com, Xf_com, 'r--', 'LineWidth', 1.5);

% Plota marcadores de início e fim
plot(Yf_sem(1), Xf_sem(1), 'go', 'MarkerSize', 8, 'MarkerFaceColor','g');
plot(Yf_sem(end), Xf_sem(end), 'bx', 'MarkerSize', 10, 'LineWidth',1.5);
plot(Yf_com(1), Xf_com(1), 'mo', 'MarkerSize', 8, 'MarkerFaceColor','m');
plot(Yf_com(end), Xf_com(end), 'rx', 'MarkerSize', 10, 'LineWidth',1.5);

title("Trajetória do Robô - Comparação Sem Carga vs Com Carga");
xlabel("Posição Y (m)"); ylabel("Posição X (m)");
legend("Sem carga","Com carga","Início sem","Fim sem","Início com","Fim com");

print("../data/trajetoria_comparativa.png","-dpng","-r300");

%% --- 6. Evolução temporal de Xc(t) e Yc(t) ---
figure('Units','inches','Position',[1 1 6 3]);
subplot(2,1,1);
plot(t_sem, Xc_sem, 'b-', 'LineWidth', 1.5); hold on;
plot(t_com, Xc_com, 'r--', 'LineWidth', 1.5);
grid on;
title("Evolução de Xc(t)");
xlabel("Tempo (s)"); ylabel("Xc (m)");
legend("Sem carga","Com carga");

subplot(2,1,2);
plot(t_sem, Yc_sem, 'b-', 'LineWidth', 1.5); hold on;
plot(t_com, Yc_com, 'r--', 'LineWidth', 1.5);
grid on;
title("Evolução de Yc(t)");
xlabel("Tempo (s)"); ylabel("Yc (m)");
legend("Sem carga","Com carga");

print("../data/evolucao_temporal.png","-dpng","-r300");

%% --- 7. Frente do robô Yf(t) ---
figure('Units','inches','Position',[1 1 6 3]);
plot(t_sem, Yf_sem, 'b-', 'LineWidth', 1.5); hold on;
plot(t_com, Yf_com, 'r--', 'LineWidth', 1.5);
grid on;
title("Evolução da posição Yf(t) (Frente do robô)");
xlabel("Tempo (s)"); ylabel("Yf (m)");
legend("Sem carga","Com carga");

print("../data/frente_robo.png","-dpng","-r300");

fprintf("Gráficos salvos em ../data/ com sucesso!\n");
