if(NOT DEFINED CMAKE_MINIMUM_REQUIRED_VERSION)
	cmake_minimum_required(VERSION 3.7)
endif()

project(icm20948 VERSION 0.1.0 LANGUAGES CXX)

add_compile_options(-Wall -Wextra)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/util/ ${CMAKE_CURRENT_SOURCE_DIR}/include)
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/ ${CMAKE_CURRENT_SOURCE_DIR}/util/)

file (GLOB SRC
    "src/*.cpp"
    "util/*.cpp"
)

file (GLOB EG_SRC
    "examples/main.cpp"
)

file (GLOB DMP_SRC
    "examples/DMPtest.cpp"
)

add_library(${PROJECT_NAME} ${SRC} )

target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
)

# target_include_directories(${PROJECT_NAME} PUBLIC include/ )
target_link_libraries(${PROJECT_NAME} -li2c)
add_executable(${PROJECT_NAME}_example ${EG_SRC} )
target_link_libraries(${PROJECT_NAME}_example ${PROJECT_NAME} -li2c)
add_executable(${PROJECT_NAME}_DMP_example ${DMP_SRC} )
target_link_libraries(${PROJECT_NAME}_DMP_example ${PROJECT_NAME} -li2c)
