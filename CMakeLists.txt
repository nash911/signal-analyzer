#-------CMakeList for SignalAnalyzer Project-----------#

cmake_minimum_required (VERSION 2.6.0)
PROJECT(SignalAnalyzer)

add_executable(SignalAnalyzer
  Source/main.cpp
  Source/signal_analyzer.cpp
  Source/signal_analyzer_list.cpp
)

### executable
target_link_libraries(SignalAnalyzer -g)

