CC=g++
CFLAGS=-Wall -Wextra -std=c++11
LIBS=$(shell pkg-config libpololu-tic-1 --cflags --libs)
INCLUDE=-Iinclude -I/usr/local/include/libpololu-tic-1
SRC_DIR=src
OBJ_DIR=obj
BIN_DIR=bin

SRC=$(wildcard $(SRC_DIR)/*.cpp)
OBJ=$(patsubst $(SRC_DIR)/%.cpp,$(OBJ_DIR)/%.o,$(SRC))
EXEC=$(BIN_DIR)/tic

all: $(EXEC)

$(EXEC): $(OBJ) $(OBJ_DIR)/tic.o
	@mkdir -p bin
	@echo "Linking $@"
	$(CC) $^ $(LIBS) -o $@

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp
	@mkdir -p obj
	@echo "Compiling $<"
	$(CC) $(CFLAGS) $(INCLUDE) -c $< -o $@

clean:
	@echo "Cleaning"
	rm -rf $(OBJ_DIR) $(BIN_DIR)
