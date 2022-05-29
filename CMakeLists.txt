cmake_minimum_required(VERSION 3.1)
enable_language(C)
project(samples)

include(FetchContent)

fetchcontent_declare(
    pigpio_src
    GIT_REPOSITORY https://github.com/joan2937/pigpio
    GIT_TAG v79)
fetchcontent_populate(pigpio_src)

find_package(Threads)

add_library(
    pigpio ${pigpio_src_SOURCE_DIR}/pigpio.c ${pigpio_src_SOURCE_DIR}/command.c
           ${pigpio_src_SOURCE_DIR}/custom.cext)
target_compile_options(pigpio PRIVATE $<$<CONFIG:Debug>:-g3 -O0>
                                      $<$<CONFIG:Relese>:-g0 -O3>)
target_include_directories(pigpio PUBLIC ${pigpio_src_SOURCE_DIR})
target_link_libraries(pigpio PUBLIC Threads::Threads)

add_library(bme280 src/bme280.c)
target_compile_options(bme280 PRIVATE -Wall -Wextra $<$<CONFIG:Debug>:-g3 -O0>
                                      $<$<CONFIG:Relese>:-g0 -O3>)
target_compile_definitions(bme280 PRIVATE $<$<CONFIG:Relese>:NDEBUG>)
target_include_directories(bme280 PUBLIC include)
target_link_libraries(bme280 PUBLIC pigpio)

add_library(ads1115 src/ads1115.c)
target_compile_options(ads1115 PRIVATE -Wall -Wextra $<$<CONFIG:Debug>:-g3 -O0>
                                       $<$<CONFIG:Relese>:-g0 -O3>)
target_compile_definitions(ads1115 PRIVATE $<$<CONFIG:Relese>:NDEBUG>)
target_include_directories(ads1115 PUBLIC include)
target_link_libraries(ads1115 PUBLIC pigpio)

add_executable(bme280_example example/bme280.c)
target_compile_options(
    bme280_example PRIVATE -Wall -Wextra $<$<CONFIG:Debug>:-g3 -O0>
                           $<$<CONFIG:Relese>:-g0 -O3>)
target_compile_definitions(bme280_example PRIVATE $<$<CONFIG:Relese>:NDEBUG>)
target_link_libraries(bme280_example PRIVATE bme280)

add_executable(ads1115_example example/ads1115.c)
target_compile_options(
    ads1115_example PRIVATE -Wall -Wextra $<$<CONFIG:Debug>:-g3 -O0>
                            $<$<CONFIG:Relese>:-g0 -O3>)
target_compile_definitions(ads1115_example PRIVATE $<$<CONFIG:Relese>:NDEBUG>)
target_link_libraries(ads1115_example PRIVATE ads1115)
