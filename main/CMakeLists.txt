# 模型文件嵌入到 rodata
set(MODEL_FILE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../model/litepose_esp32s3_test.espdl")

idf_build_get_property(component_targets __COMPONENT_TARGETS)
if ("___idf_espressif__esp-dl" IN_LIST component_targets)
    idf_component_get_property(espdl_dir espressif__esp-dl COMPONENT_DIR)
elseif("___idf_esp-dl" IN_LIST component_targets)
    idf_component_get_property(espdl_dir esp-dl COMPONENT_DIR)
endif()

if(espdl_dir)
    set(cmake_dir ${espdl_dir}/fbs_loader/cmake)
    include(${cmake_dir}/utilities.cmake)
endif()

set(embed_files ${MODEL_FILE_PATH})

idf_component_register(SRCS  "app_main.c" "cam.c" "udp_camera_client.c" "wifi_config_manager.c" "wifi_manager.c" "led.c" "dns_server.c" "audio_player.c" "posture_model.cpp"
                    INCLUDE_DIRS "."
                    EMBED_FILES ${embed_files})

target_add_aligned_binary_data(${COMPONENT_LIB} ${embed_files} BINARY)
