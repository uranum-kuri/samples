cmake_minimum_required(VERSION 3.12)
enable_language(CXX)
project(samples)

# add build options to target
function(build_options target)
    target_compile_options(
        ${target} PRIVATE -Wall -Wextra $<$<CONFIG:Debug>:-g3 -O0>
                          $<$<CONFIG:Relese>:-g0 -O3>)
    target_compile_features(${target} PRIVATE cxx_std_20)
    target_compile_definitions(${target} PRIVATE $<$<CONFIG:Relese>:NDEBUG>)
    set_target_properties(${target} PROPERTIES CXX_EXTENSIONS OFF)
endfunction(build_options)

add_executable(hello src/hello.cpp)
build_options(hello)
