project(device_init)

add_library(${PROJECT_NAME} 
    ${SDK_LOCATION}/platform/service/device_init/src/sl_device_init_dcdc_s2.c
    ${SDK_LOCATION}/platform/service/device_init/src/sl_device_init_emu_s2.c
    ${SDK_LOCATION}/platform/service/device_init/src/sl_device_init_hfxo_s2.c
    ${SDK_LOCATION}/platform/service/device_init/src/sl_device_init_lfrco.c
    ${SDK_LOCATION}/platform/service/device_init/src/sl_device_init_lfxo_s2.c
    ${SDK_LOCATION}/platform/service/device_init/src/sl_device_init_nvic.c
)

target_include_directories(${PROJECT_NAME} PUBLIC
    ${SDK_LOCATION}/platform/service/device_init/inc
    ${CMAKE_SOURCE_DIR}/config/sdk
    )

target_link_libraries(${PROJECT_NAME} slcommon emlib)
