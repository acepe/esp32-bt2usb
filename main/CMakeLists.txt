idf_component_register(SRCS "main.cpp" "bt_keyboard.cpp" "${app_sources}"
                       REQUIRES esp_hid nvs_flash driver)

                       component_compile_options(-Wno-error=format= -Wno-format)