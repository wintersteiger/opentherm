cmake_minimum_required(VERSION 3.13)

project(OpenThermOstat-CLI VERSION 0.1 LANGUAGES C CXX ASM)

find_package(CURL REQUIRED)
find_library(MOSQUITTO_LIB mosquitto REQUIRED)
find_library(PQXX_LIB pqxx REQUIRED)
find_library(PQ_LIB pq REQUIRED)

add_executable(otcli otcli.cpp)
target_include_directories(otcli PRIVATE ${CURL_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/../../include)
target_link_libraries(otcli PRIVATE ${CURL_LIBRARIES} ${MOSQUITTO_LIB} pthread)

add_executable(mqtt2pq mqtt2pq.cpp)
target_include_directories(mqtt2pq PRIVATE ${CURL_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/../../include)
target_link_libraries(mqtt2pq PRIVATE ${CURL_LIBRARIES} ${MOSQUITTO_LIB} pthread pq pqxx)