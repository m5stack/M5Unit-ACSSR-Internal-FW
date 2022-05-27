

if(IDF_VERSION_MAJOR GREATER_EQUAL 4)
    idf_component_register(SRCS "boardio.c" "i2c_slave.c" "modbusslave.c" "neopixel.c" "nvs_sync.c" "nvs_memory.c"
        INCLUDE_DIRS "include"
        REQUIRES log nvs_flash)
else()
        set(COMPONENT_SRCDIRS src)
        set(COMPONENT_ADD_INCLUDEDIRS src)
        set(COMPONENT_REQUIRES log nvs_flash mdns wpa_supplicant lwip esp_http_server)
        set(COMPONENT_EMBED_FILES src/style.css src/code.js src/index.html)
        register_component()
endif()
                
