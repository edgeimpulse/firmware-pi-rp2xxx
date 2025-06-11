cmake_minimum_required(VERSION 3.15)

include(pico_sdk_import.cmake)
set(FREERTOS_KERNEL_PATH ${CMAKE_CURRENT_SOURCE_DIR}/FreeRTOS-Kernel)
include(FreeRTOS_Kernel_import.cmake)

set(MODEL_FOLDER ei-model)
set(EI_SDK_FOLDER ${MODEL_FOLDER}/edge-impulse-sdk)
set(DRIVERS Sensors)
set(EI_PLATFORM_FOLDER edge-impulse)
set(FIRMWARE_SDK_FOLDER firmware-sdk)

if(PICO_BOARD STREQUAL "pico")
    set(RP_PROJCT_NAME ei_rp2040_firmware)
else()
    set(RP_PROJCT_NAME ei_rp2350_firmware)
endif()

project(${RP_PROJCT_NAME} C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)


pico_sdk_init()

add_definitions(-DEIDSP_QUANTIZE_FILTERBANK=0
                -DFREERTOS_ENABLED=0
               -DARM_MATH_LOOPUNROLL
                                )

add_executable(${PROJECT_NAME} src/main.cpp)

pico_enable_stdio_usb(${RP_PROJCT_NAME} 1)
pico_enable_stdio_uart(${RP_PROJCT_NAME} 0)

include(${EI_SDK_FOLDER}/cmake/utils.cmake)
pico_generate_pio_header(${RP_PROJCT_NAME} ${CMAKE_CURRENT_LIST_DIR}/${DRIVERS}/PDM/src/pdm.pio)


target_link_libraries(${RP_PROJCT_NAME}
                      pico_async_context_freertos
                      FreeRTOS-Kernel-Heap4
                      pico_stdlib
                      pico_unique_id
                      hardware_gpio
                      hardware_adc
                      hardware_i2c
                      hardware_irq
                      hardware_flash
                      hardware_pio
                      hardware_dma)

if(PICO_CYW43_SUPPORTED)
    target_link_libraries(${RP_PROJCT_NAME} pico_cyw43_arch_none)
endif()


target_compile_definitions(${RP_PROJCT_NAME} PRIVATE
    configNUMBER_OF_CORES=2
    )


target_include_directories(${RP_PROJCT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/FreeRTOS-Kernel/include
    ${MODEL_FOLDER}
    ${MODEL_FOLDER}/classifer
    ${MODEL_FOLDER}/tflite-model
    ${MODEL_FOLDER}/model-parameters
    ${EI_SDK_FOLDER}
    ${EI_SDK_FOLDER}/third_party/ruy
    ${EI_SDK_FOLDER}/third_party/gemmlowp
    ${EI_SDK_FOLDER}/third_party/flatbuffers/include
    ${EI_SDK_FOLDER}/third_party
    ${EI_SDK_FOLDER}/tensorflow
    ${EI_SDK_FOLDER}/dsp
    ${EI_SDK_FOLDER}/classifier
    ${EI_SDK_FOLDER}/anomaly
    ${EI_SDK_FOLDER}/CMSIS/NN/Include
    ${EI_SDK_FOLDER}/CMSIS/DSP/PrivateInclude
    ${EI_SDK_FOLDER}/CMSIS/DSP/Include
    ${EI_SDK_FOLDER}/CMSIS/Core/Include
    ${EI_SDK_FOLDER}/porting
    ${EI_PLATFORM_FOLDER}
    ${EI_PLATFORM_FOLDER}/ingestion-sdk-c
    ${EI_PLATFORM_FOLDER}/ingestion-sdk-platform/raspberry-rp2xxx
    ${EI_PLATFORM_FOLDER}/ingestion-sdk-platform/sensors
    ${EI_PLATFORM_FOLDER}/inference
    ${FIRMWARE_SDK_FOLDER}/QCBOR/inc
    ${FIRMWARE_SDK_FOLDER}
    ${FIRMWARE_SDK_FOLDER}/at-server
    ${DRIVERS}/Arduino/core/Wire/src
    ${DRIVERS}/Arduino/libs/Seeed_Arduino_UltrasonicRanger
    ${DRIVERS}/Arduino/libs/Arduino_LSM6DSOX/src
    ${DRIVERS}/DHT11
    ${DRIVERS}/ADXL345
    ${DRIVERS}/PDM/src/include
)

include_directories(${INCLUDES})

RECURSIVE_FIND_FILE(SOURCE_FILES "${CMAKE_CURRENT_SOURCE_DIR}/ei-model/edge-impulse-sdk" "*.cpp")
RECURSIVE_FIND_FILE(CC_FILES "${CMAKE_CURRENT_SOURCE_DIR}/ei-model/edge-impulse-sdk" "*.cc")
RECURSIVE_FIND_FILE(S_FILES "${CMAKE_CURRENT_SOURCE_DIR}/ei-model/edge-impulse-sdk" "*.s")
RECURSIVE_FIND_FILE(C_FILES "${CMAKE_CURRENT_SOURCE_DIR}/ei-model/edge-impulse-sdk" "*.c")
RECURSIVE_FIND_FILE(MODEL_FILES "${CMAKE_CURRENT_SOURCE_DIR}/ei-model/tflite-model" "*.cpp")
RECURSIVE_FIND_FILE(PORTING_FILES "${CMAKE_CURRENT_SOURCE_DIR}/ei-model/edge-impulse-sdk/porting/raspberry" "*.cpp")

RECURSIVE_FIND_FILE(PLATFORM_FILES "edge-impulse/ingestion-sdk-platform/raspberry-rp2xxx" "*.cpp")
RECURSIVE_FIND_FILE(SENSORS_FILES "edge-impulse/ingestion-sdk-platform/sensors" "*.cpp")
RECURSIVE_FIND_FILE(INGESTION_FILES "edge-impulse/ingestion-sdk-c/" "*.cpp")
RECURSIVE_FIND_FILE(INFERENCE_FILES "edge-impulse/inference" "*.cpp")
RECURSIVE_FIND_FILE(MBEDTLS_FILES "edge-impulse/mbedtls_hmac_sha256_sw/mbedtls/src" "*.c")
RECURSIVE_FIND_FILE(FIRMWARE_SDK_FILES "firmware-sdk" "*.cpp")
RECURSIVE_FIND_FILE(QCBOR_FILES "firmware-sdk/QCBOR/src" "*.c")
RECURSIVE_FIND_FILE(DRIVER_FILES "Sensors" "*.cpp")
RECURSIVE_FIND_FILE(MIC_FILES "Sensors/PDM/src" "*.c")

list(APPEND SOURCE_FILES ${S_FILES})
list(APPEND SOURCE_FILES ${C_FILES})
list(APPEND SOURCE_FILES ${CC_FILES})
list(APPEND SOURCE_FILES ${MODEL_FILES})
list(APPEND SOURCE_FILES ${PLATFORM_FILES})
list(APPEND SOURCE_FILES ${SENSORS_FILES})
list(APPEND SOURCE_FILES ${REPL_FILES})
list(APPEND SOURCE_FILES ${INGESTION_FILES})
list(APPEND SOURCE_FILES ${INFERENCE_FILES})
list(APPEND SOURCE_FILES ${QCBOR_FILES})
list(APPEND SOURCE_FILES ${MBEDTLS_FILES})
list(APPEND SOURCE_FILES ${PORTING_FILES})
list(APPEND SOURCE_FILES ${FIRMWARE_SDK_FILES})
list(APPEND SOURCE_FILES ${DRIVER_FILES})
list(APPEND SOURCE_FILES ${MIC_FILES})

target_sources(${RP_PROJCT_NAME} PRIVATE ${SOURCE_FILES})
pico_add_extra_outputs(${RP_PROJCT_NAME})
