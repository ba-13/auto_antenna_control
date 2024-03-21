CC=g++
CFLAGS=-Wall -Wextra -std=c++11
LIBS=$(shell pkg-config libpololu-tic-1 --cflags --libs)
INCLUDE=-Iinclude -I/usr/local/include/libpololu-tic-1 -I/usr/local/include
SRC_DIR=src
OBJ_DIR=obj
BIN_DIR=bin

SRC=$(wildcard $(SRC_DIR)/*.cpp)
OBJ=$(patsubst $(SRC_DIR)/%.cpp,$(OBJ_DIR)/%.o,$(SRC))
tic=$(BIN_DIR)/tic
move_in_sine=$(BIN_DIR)/move_in_sine
predictor_node=$(BIN_DIR)/predictor_node

all: $(tic) $(move_in_sine) $(predictor_node)

$(tic): $(OBJ_DIR)/tic_lib.o $(OBJ_DIR)/tic.o
	@mkdir -p bin
	@echo "Linking $@"
	$(CC) $^ $(LIBS) -o $@

$(move_in_sine): $(OBJ_DIR)/tic_lib.o $(OBJ_DIR)/move_in_sine.o
	@mkdir -p bin
	@echo "Linking $@"
	$(CC) $^ $(LIBS) -o $@

$(predictor_node): $(OBJ_DIR)/predictor.o $(OBJ_DIR)/predictor_node.o
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
