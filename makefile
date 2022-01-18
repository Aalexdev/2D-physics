# project
VERSION = 0.1

# compiler
CXX = g++.exe
STD_VERSION = c++17
LIBSFLAGS = -lmingw32 -lSDL2main -lSDL2
CFLAGS = -Wall -g3
DEFINES = -DVERSION=$(VERSION)
INCLUDE = include/

# directories
BIN = bin
SRC = src
OBJ = obj
LIB = lib
EXEC = prog

# source files
SRCS = $(wildcard **/*.cpp) $(wildcard $(SRC)/**/*.cpp)
OBJS := $(patsubst %.cpp, $(OBJ)/%.o, $(notdir $(SRCS)))

all: $(EXEC)

release: CFLAGS = -Wall -O2 -DNDEBUG
release: clean
release: $(EXEC)

rebuild: clean
rebuild: $(EXEC)

clean:
	@del $(OBJ)\*.o

run:
	$(BIN)/$(EXEC)

$(EXEC) : $(OBJS)
	$(CXX) -std=$(STD_VERSION) $(OBJ)/*.o -I $(INCLUDE) -L $(LIB) -o $(BIN)\$(EXEC) $(CFLAGS) $(DEFINES) $(LIBSFLAGS)

$(OBJ)/%.o : $(SRC)/%.cpp
	$(CXX) -std=$(STD_VERSION) -o $@ -c $< -I $(INCLUDE) $(DEFINES) $(CFLAGS)

$(OBJ)/%.o : $(SRC)/*/%.cpp
	$(CXX) -std=$(STD_VERSION) -o $@ -c $< -I $(INCLUDE) $(DEFINES) $(CFLAGS)

$(OBJ)/%.o : $(SRC)/*/*/%.cpp
	$(CXX) -std=$(STD_VERSION) -o $@ -c $< -I $(INCLUDE) $(DEFINES) $(CFLAGS)

info:
	@echo -----------------------------------------------------
	@echo info :                
	@echo 	compiler                     : $(CXX)
	@echo 	compiler standart version    : $(STD_VERSION)
	@echo 	flags                        : $(CFLAGS)
	@echo 	defines                      : $(DEFINES)
	@echo 	executable output            : $(EXEC)
	@echo 	libs directory               : $(LIB)
	@echo 	binary directory             : $(BIN)
	@echo 	source code directory        : $(SRC)
	@echo 	compiled object directory    : $(OBJ)
	@echo 	include directory            : $(INCLUDE)
	@echo -----------------------------------------------------

compress:
	7z a -tzip vk_engine_$(VERSION).zip