# rt_cell — RT-планировщик + бинарный протокол (C++20, zero-allocation)
CXX      := g++
CXXFLAGS := -std=c++20 -O2 -Wall -Wextra -Wpedantic -Wshadow -Werror -Iinclude
BIN      := build
HDRS     := include/rt/protocol.hpp include/rt/spsc.hpp include/rt/geometry.hpp include/rt/traj.hpp include/rt/scheduler.hpp

.PHONY: all clean run

all: $(BIN)/test_plain $(BIN)/test_asan $(BIN)/test_tsan

$(BIN)/test_plain: tests/main.cpp $(HDRS)
	@mkdir -p $(BIN)
	$(CXX) $(CXXFLAGS) -o $@ tests/main.cpp

$(BIN)/test_asan: tests/main.cpp $(HDRS)
	@mkdir -p $(BIN)
	$(CXX) $(CXXFLAGS) -DRT_NO_ALLOC_GUARD -DRT_STRESS_N=20000 \
		-fsanitize=address,undefined -fno-omit-frame-pointer -o $@ tests/main.cpp

$(BIN)/test_tsan: tests/main.cpp $(HDRS)
	@mkdir -p $(BIN)
	$(CXX) $(CXXFLAGS) -DRT_NO_ALLOC_GUARD -DRT_STRESS_N=20000 \
		-fsanitize=thread -o $@ tests/main.cpp

run: all
	./$(BIN)/test_plain; ./$(BIN)/test_asan; setarch $$(uname -m) -R timeout 60 ./$(BIN)/test_tsan

clean:
	rm -rf $(BIN)
