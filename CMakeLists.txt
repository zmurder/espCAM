# For more information about build system see
# https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html
# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

# 固件版本嵌入镜像（esp_app_desc.version）：以 main/app_version.h 的 APP_VERSION 为准。
# 不设置时，项目在 git 仓库内 IDF 会默认嵌 git describe（如 "1d3f83d-dirty"），
# 与手动维护的"年月日_当日序号"版本号不一致，PC 端"目标固件"显示会混乱
file(STRINGS "${CMAKE_CURRENT_LIST_DIR}/main/app_version.h" _appver_line REGEX "^#define APP_VERSION")
string(REGEX REPLACE "^#define[ \t]+APP_VERSION[ \t]+\"([^\"]*)\".*$" "\\1" PROJECT_VER "${_appver_line}")
unset(_appver_line)
# file(STRINGS) 只在 configure 期执行一次：不加下面这行，改 app_version.h 只会重编译代码里的宏，
# 镜像里嵌的版本串会冻结在最后一次 configure 时的值（代码 ACK 版本与镜像版本不一致的根源）
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${CMAKE_CURRENT_LIST_DIR}/main/app_version.h")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(espCAM_WIFI)
