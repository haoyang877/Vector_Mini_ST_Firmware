cmake_minimum_required(VERSION 3.21)
project(YG_MC_HOST VERSION 1.4.1 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
find_package(Qt6 6.8 REQUIRED COMPONENTS Widgets)
include(CTest)
include(GNUInstallDirs)

add_library(host_shared STATIC
    ../shared/checksum/crc32.c
    ../shared/image/image_transfer.c
    ../shared/protocol/yg_protocol_crc.c
    ../shared/protocol/yg_protocol_frame_codec.c
    ../shared/protocol/yg_protocol_can_id.c
    ../shared/protocol/yg_protocol_upgrade_wire.c)
target_include_directories(host_shared PUBLIC .. ../shared/image ../shared/protocol)
add_library(host_services STATIC services/upgrade_plan.cpp services/device_readback.cpp)
target_include_directories(host_services PUBLIC .)
target_link_libraries(host_services PUBLIC host_shared Qt6::Core)
add_library(host_widgets STATIC gui/maintenance_window.cpp gui/device_connection.cpp)
target_link_libraries(host_widgets PUBLIC host_services Qt6::Widgets)
target_compile_definitions(host_widgets PUBLIC YG_MC_HOST_VERSION=\"${PROJECT_VERSION}\")
add_executable(YG_MC_HOST WIN32 main.cpp)
target_link_libraries(YG_MC_HOST PRIVATE host_widgets)
# 只复制源码适配器；SDK DLL 和 Python 运行时由用户在连接时明确提供。
file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/ports")
file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/contracts")
configure_file(device_reader.py "${CMAKE_CURRENT_BINARY_DIR}/device_reader.py" COPYONLY)
configure_file(upgrade_client.py "${CMAKE_CURRENT_BINARY_DIR}/upgrade_client.py" COPYONLY)
configure_file(ports/controlcanfd.py "${CMAKE_CURRENT_BINARY_DIR}/ports/controlcanfd.py" COPYONLY)
configure_file(../shared/targets/vector_mini_st_g431.json
    "${CMAKE_CURRENT_BINARY_DIR}/contracts/vector_mini_st_g431.json" COPYONLY)

# 构建和独立部署共用同一份目标契约；Qt 原生部署负责运行库，不复制整个源码仓库。
install(TARGETS YG_MC_HOST RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}")
install(FILES device_reader.py upgrade_client.py DESTINATION "${CMAKE_INSTALL_BINDIR}")
install(FILES ports/controlcanfd.py DESTINATION "${CMAKE_INSTALL_BINDIR}/ports")
install(FILES ../shared/targets/vector_mini_st_g431.json
    DESTINATION "${CMAKE_INSTALL_BINDIR}/contracts")
install(FILES README.md DESTINATION .)
# 保留目标路径引号，支持含空格的 Windows 工作目录。
qt_generate_deploy_script(TARGET YG_MC_HOST OUTPUT_SCRIPT host_deploy_script CONTENT "
qt_deploy_runtime_dependencies(
    EXECUTABLE \"$<TARGET_FILE:YG_MC_HOST>\"
    GENERATE_QT_CONF
    NO_TRANSLATIONS)
")
install(SCRIPT "${host_deploy_script}")

function(host_resources target)
    qt_add_resources(${target} host_ui PREFIX "/ui" BASE resources FILES resources/theme.qss)
    qt_add_resources(${target} host_target PREFIX "/contracts" BASE "${CMAKE_CURRENT_SOURCE_DIR}/../shared/targets"
        FILES ../shared/targets/vector_mini_st_g431.json)
endfunction()
host_resources(YG_MC_HOST)
if(BUILD_TESTING)
    add_executable(host_core_test tests/core_test.cpp)
    target_link_libraries(host_core_test PRIVATE host_widgets)
    host_resources(host_core_test)
    qt_add_resources(host_core_test host_test_fixture PREFIX "/fixtures"
        BASE "${CMAKE_CURRENT_SOURCE_DIR}/../tests/fixtures"
        FILES ../tests/fixtures/loader_image_v3.json)
    add_test(NAME host_core_offline COMMAND host_core_test)
    set_tests_properties(host_core_offline PROPERTIES ENVIRONMENT "QT_QPA_PLATFORM=offscreen" TIMEOUT 15)
endif()
