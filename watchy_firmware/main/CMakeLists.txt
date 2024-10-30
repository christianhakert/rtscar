idf_component_register(SRCS "main.c" INCLUDE_DIRS ".")
idf_build_set_property(COMPILE_OPTIONS "-DLOG_LOCAL_LEVEL=ESP_LOG_DEBUG" APPEND)
