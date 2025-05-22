cmake_minimum_required(VERSION 3.10)
project(AllegroHandController)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

find_package(Threads REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

add_library(canAPI STATIC
    canAPI.cpp
)

target_link_libraries(canAPI
    PUBLIC Threads::Threads
)

add_library(rsp STATIC
    RockScissorsPaper.cpp
)

add_executable(AllegroHandControl
    main.cpp
)

# --------------------------------------
# Finding the BHand Library
# --------------------------------------
# Please specify the location of libBHand.so in the system.
# For example, if you have /usr/local/lib/libBHand.so, add it to PATHS below or
# You can use find_library yourself.
find_library(BHAND_LIB
    NAMES BHand BHandLib
    PATHS /usr/local/lib /usr/lib
)

if (NOT BHAND_LIB)
    message(FATAL_ERROR "BHand library not found, please check installation path.")
endif()

# --------------------------------------
# Link order: candrv → rsp → Threads → BHand
# --------------------------------------
target_link_libraries(AllegroHandControl
    PRIVATE canAPI
            rsp
            Threads::Threads
            ${BHAND_LIB}
)
