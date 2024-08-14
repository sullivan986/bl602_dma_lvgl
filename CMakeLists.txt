cmake_minimum_required(VERSION 3.15)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
include(proj.conf)

find_package(bouffalo_sdk REQUIRED HINTS $ENV{BL_SDK_BASE})

sdk_set_main_file(main.c)

project(spi_dma)

sdk_add_include_directories(include)

aux_source_directory(src SRC)
target_sources(app PRIVATE 
    ${SRC}
)


# LVGL9
set(lvgl9_dir "/home/sullivan/sdk/lvgl")
file(GLOB lvgl9_all_src 
    "${lvgl9_dir}/demos/*.c"
    "${lvgl9_dir}/demos/*/*.c"
    "${lvgl9_dir}/demos/*/*/*.c"
    "${lvgl9_dir}/demos/*/*/*/*.c"
    "${lvgl9_dir}/src/*.c"
    "${lvgl9_dir}/src/*/*.c"
    "${lvgl9_dir}/src/*/*/*.c"
    "${lvgl9_dir}/src/*/*/*/*.c"
    "${lvgl9_dir}/src/*/*/*/*/*.c"
)
target_sources(app PRIVATE 
    ${lvgl9_all_src}
)
sdk_add_include_directories(${lvgl9_dir})
sdk_add_include_directories(${lvgl9_dir}/demos)
sdk_add_compile_definitions(LV_TICK_PERIOD_MS=1)