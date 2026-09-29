cmake_minimum_required(VERSION 3.21)
project(YG_MC_HOST LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
find_package(Qt6 6.8 REQUIRED COMPONENTS Widgets)
include(CTest)

add_library(host_shared STATIC
    ../shared/checksum/crc32.c
    ../shared/image/image_transfer.c
    ../shared/protocol/yg_protocol_crc.c
    ../shared/protocol/yg_protocol_frame_codec.c
    ../shared/protocol/yg_protocol_can_id.c
    ../shared/protocol/yg_protocol_upgrade_wire.c)
target_include_directories(host_shared PUBLIC .. ../shared/image ../shared/protocol)
add_library(host_services STATIC services/upgrade_plan.cpp services/parameter_sandbox.cpp
    protocol/parameter_wire.cpp)
target_include_directories(host_services PUBLIC .)
target_link_libraries(host_services PUBLIC host_shared Qt6::Core)
add_library(host_widgets STATIC gui/maintenance_window.cpp)
target_link_libraries(host_widgets PUBLIC host_services Qt6::Widgets)
add_executable(YG_MC_HOST WIN32 main.cpp)
target_link_libraries(YG_MC_HOST PRIVATE host_widgets)

function(host_resources target)
    qt_add_resources(${target} host_ui PREFIX "/ui" BASE resources FILES resources/theme.qss)
    qt_add_resources(${target} host_target PREFIX "/contracts" BASE "${CMAKE_CURRENT_SOURCE_DIR}/../shared/targets"
        FILES ../shared/targets/vector_mini_st_g431.json)
    qt_add_resources(${target} host_fixture PREFIX "/fixtures" BASE "${CMAKE_CURRENT_SOURCE_DIR}/../tests/fixtures"
        FILES ../tests/fixtures/loader_image_v3.json)
endfunction()
host_resources(YG_MC_HOST)
if(BUILD_TESTING)
    add_executable(host_maintenance_test tests/maintenance_test.cpp)
    target_link_libraries(host_maintenance_test PRIVATE host_widgets)
    host_resources(host_maintenance_test)
    add_test(NAME host_maintenance_offline COMMAND host_maintenance_test)
    set_tests_properties(host_maintenance_offline PROPERTIES ENVIRONMENT "QT_QPA_PLATFORM=offscreen" TIMEOUT 15)
endif()
