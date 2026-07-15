# 模型文件嵌入到 rodata - 使用新量化模型 pose_model_int8_final（4 关键点 COCO Pose）
set(MODEL_FILE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../model/pose_model_int8_final.espdl")

# 测试用静态图片（POSTURE_TEST_IMAGE_MODE=1 时推理此图，验证 ESP 处理）
set(TEST_IMAGE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../model/3202402.jpg")

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

# 嵌入文件列表（模型 + 测试图）
set(embed_files ${MODEL_FILE_PATH} ${TEST_IMAGE_PATH})

idf_component_register(SRCS  "app_main.c" "cam.c" "udp_camera_client.c" "wifi_config_manager.c" "wifi_manager.c" "led.c" "dns_server.c" "audio_player.c" "posture_model.cpp"
                    INCLUDE_DIRS "."
                    EMBED_FILES ${embed_files})

target_add_aligned_binary_data(${COMPONENT_LIB} ${embed_files} BINARY)