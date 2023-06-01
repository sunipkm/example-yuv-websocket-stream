cmake_minimum_required(VERSION 3.1)
project(example-yuv-websocket-stream)

set(CMAKE_CXX_STANDARD 11)

if(NOT UNIX)
    message( FATAL_ERROR "This example only targets Linux systems with V4L2 development libs installed!" )
endif(NOT UNIX)

add_library(example-yuv-websocket-stream-lib
        src/AppComponent.hpp
        src/SwaggerComponent.hpp
        src/controller/CamAPIController.hpp
        src/controller/CamAPIController.cpp
        src/backend/V4LGrabber.hpp
        src/backend/V4LGrabber.cpp
        src/backend/ImageWSListener.hpp
        src/backend/ImageWSListener.cpp
        src/backend/Utils.hpp
        src/backend/Utils.cpp
        src/dto/DTOs.hpp)

add_library(libASICamera2 STATIC IMPORTED)
add_library(libCameraUnit_ASI STATIC IMPORTED)

set_target_properties(libASICamera2 PROPERTIES IMPORTED_LOCATION /usr/local/lib/libASICamera2.a)
set_target_properties(libCameraUnit_ASI PROPERTIES IMPORTED_LOCATION /usr/local/lib/libCameraUnit_ASI.a)

## include directories

target_include_directories(example-yuv-websocket-stream-lib PUBLIC src /usr/local/include)


## link libs

find_package(oatpp            1.3.0 REQUIRED)
find_package(oatpp-swagger    1.3.0 REQUIRED)
find_package(oatpp-websocket  1.3.0 REQUIRED)

target_link_libraries(example-yuv-websocket-stream-lib
        PUBLIC oatpp::oatpp
        PUBLIC oatpp::oatpp-swagger
        PUBLIC oatpp::oatpp-websocket
        PUBLIC libCameraUnit_ASI
        PUBLIC libASICamera2
        PUBLIC usb-1.0
        PUBLIC cfitsio
)

## define path to swagger-ui res folder

add_definitions(-DOATPP_SWAGGER_RES_PATH="${OATPP_BASE_DIR}/bin/oatpp-swagger/res")
add_definitions(-DWWW_FOLDER="${CMAKE_CURRENT_SOURCE_DIR}/res")

## add executables

add_executable(example-yuv-websocket-stream-exe
        src/App.cpp
)

target_link_libraries(example-yuv-websocket-stream-exe example-yuv-websocket-stream-lib)

add_executable(example-yuv-websocket-stream-test
        test/tests.cpp
)
target_link_libraries(example-yuv-websocket-stream-test example-yuv-websocket-stream-lib)

enable_testing()
add_test(project-tests example-yuv-websocket-stream-test)