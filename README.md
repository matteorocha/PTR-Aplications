# PTR Applications - Real Time Systems

Este repositório consolida os trabalhos práticos desenvolvidos na disciplina de **Programação de Tempo Real** (Engenharia da Computação - UFAM).  
O foco dos projetos é a aplicação de concorrência, sistemas de controle distribuído e análise temporal em ambiente Linux.

**Aluno:** Matheus Rocha Canto  
**Professor:** André Cavalcante  

---

## 📋 Resumo das Atividades Desenvolvidas

Abaixo, o detalhamento do que foi implementado em cada etapa do projeto:

### 🔹 Trabalho 1: Infraestrutura e ADTs

Foco na estruturação do ambiente de desenvolvimento profissional em C.

- **Automação de Build:** Criação de `Makefiles` com detecção automática de fontes e geração de objetos.  
- **Modularização:** Separação estrita entre interface (`.h`) e implementação (`.c`).  
- **ADT Matrix:** Implementação de uma biblioteca completa de álgebra linear (Soma, Multiplicação, Transposta, Determinante por Laplace e Inversa).  
- **ADT Integral:** Criação de módulo para integração numérica utilizando a **Regra Composta do Trapézio** e ponteiros de função para flexibilidade.

---

### 🔹 Trabalho 2: Simulação e Análise de Jitter

Introdução à programação concorrente e análise de desempenho temporal.

- **Multithreading:** Decomposição do sistema em threads de **Simulação** (Robô) e **Controle/IO** usando `PThreads`.  
- **Stress Testing:** Implementação de uma thread de carga para estressar a CPU e comparar o comportamento do escalonador.  
- **Análise de Jitter:** Coleta de métricas temporais que revelou, contraintuitivamente, maior instabilidade (maior variância) no sistema **sem carga**, devido a processos esporádicos do SO.

---

### 🔹 Trabalho 3: Sistema de Controle Distribuído (Não-RT)

Evolução para um sistema complexo com múltiplas tarefas sincronizadas.

- **Arquitetura Complexa:** Orquestração de 7 threads distintas com periodicidades diferentes (Robô, Linearização, Controle, Modelo de Referência, etc.).  
- **Sincronização:** Uso de **Monitores** e **Mutexes** para garantir a integridade dos dados compartilhados entre as threads.  
- **Análise RMA:** Validação teórica da escalonabilidade usando a Análise de Taxa Monotônica (Liu & Layland), comprovando utilização de CPU < 1%.  
- **Conclusão Prática:** Demonstração das limitações de um SO de propósito geral (Linux) para controle crítico, onde a latência prejudicou o seguimento da trajetória apesar da folga na CPU.

---

## 🛠️ Tecnologias Utilizadas

- **Linguagem:** C (C17)  
- **Bibliotecas:** POSIX Threads (`pthread`), Math (`-lm`)  
- **Ferramentas:** GCC, GNU Make, Git  
- **Análise de Dados:** GNU Octave (scripts `.m`)  

---


## 📂 Estrutura do Projeto

*   **[/01-threads-basico](./01-threads-basico)**: Introdução à criação de pthreads e contexto.
*   **[/02-sincronizacao](./02-sincronizacao)**: Resolução de condições de corrida usando Mutex e Semáforos.
*   **[/03-escalonamento](./03-escalonamento)**: Algoritmo de prioridade para sistemas de tempo real.

---

## 🚀 Git e Instruções de Teste

Para baixar este repositório e executar os testes de cada trabalho, siga o fluxo abaixo utilizando o Git e o terminal Linux.

### 1️⃣ Clonar o Repositório

```bash
git clone https://github.com/matteorocha/PTR-Aplications.git
cd PTR-Aplications

```
### 2️⃣ Executar os Trabalhos

Cada trabalho possui seu próprio Makefile. O fluxo de teste é padronizado:
## ▶️ Trabalho 1 (Testes Unitários das ADTs)

```bash
cd Work1
make        # Compila o projeto    
./main # Executa os testes de Matrizes e Integrais

```
## ▶️ Trabalho 2 (Simulação com/sem Carga)

```bash
cd ../Work2
make
./main 0    # Executa SEM carga de CPU (modo padrão)
./main 1    # Executa COM carga de CPU (modo stress)

📁 Os dados gerados serão salvos na pasta data/.

```
## ▶️ Trabalho 3 (Controle Distribuído)

```bash
cd ../Work3
make
./main
```

## 3️⃣ Visualizar Gráficos (Octave)

Os Trabalhos 2 e 3 geram arquivos de log. Para visualizar os gráficos de trajetória e erro:

### Estando na pasta do trabalho (ex: Work3)

```bash
octave scripts/plot_lab3.m
```

### 📌 Observações
    Este projeto foi desenvolvido em ambiente Linux.

    Recomenda-se a utilização de um kernel padrão (não RT) para reproduzir os experimentos conforme o contexto da disciplina.

    Os resultados de jitter e latência podem variar conforme a carga do sistema e processos em background.