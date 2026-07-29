# Builds the ROS-free verifier core: tests and the evaluation harness.
# Requires only g++ with C++20 support.
CXX      ?= g++
CXXFLAGS ?= -std=c++20 -O2 -Wall -Wextra -Wpedantic
CPPFLAGS  = -Iinclude

BUILD := build

.PHONY: all test eval clean

all: test

$(BUILD):
	mkdir -p $(BUILD)

$(BUILD)/verifier.o: src/verifier.cpp include/verifier/*.hpp | $(BUILD)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c $< -o $@

$(BUILD)/test_verifier: tests/test_verifier.cpp $(BUILD)/verifier.o eval/scenarios.hpp | $(BUILD)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $< $(BUILD)/verifier.o -o $@

$(BUILD)/eval: eval/eval_main.cpp $(BUILD)/verifier.o eval/scenarios.hpp | $(BUILD)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $< $(BUILD)/verifier.o -o $@

test: $(BUILD)/test_verifier
	./$(BUILD)/test_verifier

eval: $(BUILD)/eval
	@echo "run: $(BUILD)/eval --maps 50 --seed 42 --out ../reports/results/verifier_eval.csv"

clean:
	rm -rf $(BUILD)
