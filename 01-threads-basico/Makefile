# --- Makefile para o Laboratório 1 ---

# --- 1. Variáveis de Configuração ---
CC = gcc
CFLAGS = -Wall -Wextra -g -std=c17 -Iinclude
LDFLAGS = -lm

# --- 2. Definição de Diretórios ---
SRCDIR = src
OBJDIR = obj

# Alvo final do projeto
TARGET = main

# --- 3. Descoberta Automática de Arquivos ---
SOURCES = $(wildcard $(SRCDIR)/*.c)
OBJECTS = $(patsubst $(SRCDIR)/*.c, $(OBJDIR)/%.o, $(SOURCES))


# --- 4. Regras de Compilação ---

# Regra principal: compila o projeto inteiro
all: $(TARGET)

# Regra de Linkagem: cria o executável 'main' na pasta raiz
# IMPORTANTE: A linha de comando abaixo DEVE começar com um caractere TAB, não espaços.
$(TARGET): $(OBJECTS)
	$(CC) $(CFLAGS) $(OBJECTS) -o $(TARGET) $(LDFLAGS)
	@echo "Compilação finalizada! Para executar: ./$(TARGET)"

# Regra de Padrão: compila um arquivo .c para um .o
# IMPORTANTE: As duas linhas de comando abaixo DEVEM começar com um caractere TAB.
$(OBJDIR)/%.o: $(SRCDIR)/%.c
	@mkdir -p $(OBJDIR)
	$(CC) $(CFLAGS) -c $< -o $@

# --- 5. Regras Auxiliares ---

# IMPORTANTE: As linhas de comando abaixo DEVEM começar com um caractere TAB.
clean:
	@echo "Limpando arquivos compilados..."
	rm -rf $(OBJDIR) $(TARGET)

run: all
	./$(TARGET)

# Declara regras que não são arquivos
.PHONY: all clean run
