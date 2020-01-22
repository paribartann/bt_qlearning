
TARGET = main

SRC = src
INC = include
BIN = bin

SOURCE = $(wildcard $(SRC)/*.cpp)
OBJECT = $(patsubst %,$(BIN)/%, $(notdir $(SOURCE:.cpp=.o)))

CC = g++
CFLAGS = -Wall -g -std=c++11 -Wextra -I$(INC)

RED = \033[1;31m
GREEN = \033[1;32m
BLUE = \033[1;34m
YELLOW = \033[1;33m
NC = \033[1;0m

$(BIN)/$(TARGET) : $(OBJECT)
	@echo "$(RED)Linking .....$(NC)"
	$(CC) -o $@ $^
	@echo "$(BLUE)Finished .....$(NC)"

$(BIN)/%.o : $(SRC)/%.cpp
	@echo "$(GREEN)Compiling .....$(NC)"
	$(CC) $(CFLAGS) -c $< -o $@

.PHONY: help run clean

run: $(BIN)/$(TARGET)
		@echo "$(YELLOW)Running .....$(NC)"
		$(BIN)/$(TARGET)

clean:
	rm -f $(OBJECT) $(BIN)/$(TARGET)

help :
	 @echo "src: $(SOURCE)" 
	 @echo "obj: $(OBJECT)"