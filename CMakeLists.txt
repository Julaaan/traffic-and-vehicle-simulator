cmake_minimum_required(VERSION 3.24)
project(VehicleSimulator VERSION 0.3.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
foreach(config DEBUG RELEASE RELWITHDEBINFO MINSIZEREL)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${config} "${CMAKE_BINARY_DIR}/bin")
endforeach()

option(VSIM_BUILD_VIEWER "Build the SDL2 desktop viewer" ON)

include(FetchContent)
set(JSON_BuildTests OFF CACHE BOOL "" FORCE)
set(JSON_Install OFF CACHE BOOL "" FORCE)
FetchContent_Declare(nlohmann_json
    URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz
    URL_HASH SHA256=d6c65aca6b1ed68e7a182f4757257b107ae403032760ed6ef121c9d55e81757d
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
FetchContent_MakeAvailable(nlohmann_json)
file(MAKE_DIRECTORY "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/licenses")
configure_file("${nlohmann_json_SOURCE_DIR}/LICENSE.MIT"
    "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/licenses/nlohmann-json-LICENSE.MIT" COPYONLY)

find_package(Threads REQUIRED)
add_library(sim_core
    src/core/Dynamics.cpp
    src/core/RoadNetwork.cpp
    src/core/Collision.cpp
    src/core/World.cpp
    src/core/Simulation.cpp
    src/core/Batch.cpp
    src/core/RoutePlanner.cpp)
target_include_directories(sim_core PUBLIC include)
target_link_libraries(sim_core PUBLIC Threads::Threads)
target_compile_features(sim_core PUBLIC cxx_std_20)
target_compile_definitions(sim_core PUBLIC VSIM_VERSION="${PROJECT_VERSION}")

add_library(sim_io src/io/Serialization.cpp src/io/Replay.cpp src/io/DynamicsValidation.cpp)
target_link_libraries(sim_io PUBLIC sim_core nlohmann_json::nlohmann_json)

add_library(sim_scenarios src/scenario/ScenarioLoader.cpp)
target_link_libraries(sim_scenarios PUBLIC sim_core PRIVATE sim_io)
set(tinyxml2_BUILD_TESTING OFF CACHE BOOL "" FORCE)
set(tinyxml2_INSTALL_PKGCONFIG OFF CACHE BOOL "" FORCE)
FetchContent_Declare(tinyxml2
    URL https://github.com/leethomason/tinyxml2/archive/refs/tags/10.0.0.tar.gz
    URL_HASH SHA256=3bdf15128ba16686e69bce256cc468e76c7b94ff2c7f391cc5ec09e40bff3839
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
FetchContent_MakeAvailable(tinyxml2)
configure_file("${tinyxml2_SOURCE_DIR}/LICENSE.txt"
    "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/licenses/TinyXML2-LICENSE.txt" COPYONLY)
add_library(sim_osm src/osm/OsmReader.cpp src/osm/OsmNetworkBuilder.cpp)
target_link_libraries(sim_osm PUBLIC sim_core PRIVATE tinyxml2::tinyxml2)
add_library(sim_app src/app/Options.cpp src/app/Output.cpp)
target_include_directories(sim_app PUBLIC src/app)
target_link_libraries(sim_app PUBLIC sim_scenarios sim_osm sim_io)

add_executable(sim_cli src/app/Cli.cpp)
target_link_libraries(sim_cli PRIVATE sim_app)
add_executable(sim_validate src/app/Validate.cpp)
target_link_libraries(sim_validate PRIVATE sim_io)
add_executable(sim_bench src/app/Bench.cpp)
target_link_libraries(sim_bench PRIVATE sim_scenarios sim_osm sim_io)
set(VSIM_TARGETS sim_core sim_io sim_scenarios sim_osm sim_app sim_cli sim_validate sim_bench)

if(VSIM_BUILD_VIEWER)
    set(SDL_SHARED OFF CACHE BOOL "" FORCE)
    set(SDL_STATIC ON CACHE BOOL "" FORCE)
    set(SDL_TEST OFF CACHE BOOL "" FORCE)
    set(SDL_TESTS OFF CACHE BOOL "" FORCE)
    set(SDL2_DISABLE_INSTALL ON CACHE BOOL "" FORCE)
    FetchContent_Declare(sdl2
        URL https://github.com/libsdl-org/SDL/releases/download/release-2.30.11/SDL2-2.30.11.tar.gz
        URL_HASH SHA256=8b8d4aef2038533da814965220f88f77d60dfa0f32685f80ead65e501337da7f
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
    FetchContent_MakeAvailable(sdl2)
    configure_file("${sdl2_SOURCE_DIR}/LICENSE.txt"
        "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/licenses/SDL2-LICENSE.txt" COPYONLY)
    FetchContent_Declare(stb_truetype
        URL https://raw.githubusercontent.com/nothings/stb/a7d4c2175630e1ea5b6f8dd00ab96fed0b97b96d/stb_truetype.h
        URL_HASH SHA256=ecd30b05e0dd4fea3a13c26810dd9e1992dc379049482c393d5a19e6b5090aab
        DOWNLOAD_NO_EXTRACT TRUE)
    FetchContent_MakeAvailable(stb_truetype)
    add_executable(sim_viewer src/viewer/Viewer.cpp src/viewer/Draw.cpp src/viewer/Font.cpp)
    target_include_directories(sim_viewer PRIVATE src/viewer)
    target_include_directories(sim_viewer SYSTEM PRIVATE "${stb_truetype_SOURCE_DIR}")
    target_compile_definitions(sim_viewer PRIVATE SDL_MAIN_HANDLED)
    target_link_libraries(sim_viewer PRIVATE sim_app SDL2::SDL2-static)
    list(APPEND VSIM_TARGETS sim_viewer)
endif()

foreach(target ${VSIM_TARGETS})
    if(MSVC)
        target_compile_options(${target} PRIVATE /W4 /permissive- /utf-8)
    else()
        target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
    endif()
endforeach()

file(GLOB VSIM_DATA_FILES CONFIGURE_DEPENDS RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
    scenarios/*.json maps/*.osm profiles/*.json references/*.json references/*.csv)
foreach(data_file ${VSIM_DATA_FILES})
    configure_file("${data_file}" "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${data_file}" COPYONLY)
endforeach()
