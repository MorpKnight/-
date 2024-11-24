cmake_minimum_required(VERSION 3.5)
set(EXTRA_COMPONENT_DIRS ./components/esp-dl)
add_compile_options(-fdiagnostics-color=always)
set(EXTRA_COMPONENT_DIRS $ENV{IDF_PATH}/examples/common_components/protocol_examples_common) # Untuk wifi
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(deploy)
        