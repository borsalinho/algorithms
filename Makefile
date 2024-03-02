LIB = s21_graph.a

CC = g++
CCFLAGS = -std=c++17 -Wall -Werror -Wextra
TEST_FLAGS = -lgtest -lgmock -pthread

SRC_FILES =	graph/s21_graph.h graphAlgorithms/s21_graphAlgorithms.h

TEST_DIR = tests/

TEST_FILES = s21_run_tests.cc \
						 s21_graph_load.cc \
						 s21_graph_export.cc \
						 s21_first_search.cc \
						 s21_shortest_path.cc

TEST = $(addprefix $(TEST_DIR), $(TEST_FILES))

all: $(LIB)

$(LIB):
	ar rcs $(LIB) $(SRC_FILES)
	@echo Compiling $(NAME) success!

run:
	$(CC) $(CCFLAGS) $(LIB) main.cc -o run
	./run

test:
	$(CC) $(CCFLAGS) $(TEST) $(TEST_FLAGS) -o s21_run_tests.exe
	./s21_run_tests.exe || true

clean:
	rm -rf s21_graph.a *.o
	rm -rf s21_run_tests.exe
	rm -rf tests/tmp_files/*

clang:
	cp ../materials/linters/.clang-format ./
	clang-format -n containers/*/*.h graph/*.h graphAlgorithms/*.h menu/*.h main.cc
	rm .clang-format

.PHONY: all run clean test $(LIB)
