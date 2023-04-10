# CMakeLists.txt for the Sparkfun ICM 20948 Arduino library to allow build within ESP-IDF framework

file(GLOB_RECURSE SOURCES ./src/*.c ./src/*.cpp ./src/util/*.c ./src/util/*.cpp)

set(INCLUDE_DIRS ./src ./src/util)

idf_component_register(SRCS ${SOURCES}
                       INCLUDE_DIRS ${INCLUDE_DIRS}
                       REQUIRES arduino)