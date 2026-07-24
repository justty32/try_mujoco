cmake_minimum_required(VERSION 3.20)
project(robot C)

set(CMAKE_C_STANDARD 11)

set(MUJOCO_DIR "C:/dev/mujoco")

# Fetch and build GLFW from source (statically, same as MuJoCo's own samples)
include(FetchContent)
FetchContent_Declare(
    glfw
    GIT_REPOSITORY https://github.com/glfw/glfw.git
    GIT_TAG        3.4
)
set(GLFW_BUILD_DOCS     OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS    OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(GLFW_INSTALL        OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(glfw)

add_executable(robot
    src/main.c
    src/sim.c
    src/controller.c
    src/viewer.c
)

target_include_directories(robot PRIVATE
    src
    "${MUJOCO_DIR}/include"
)
target_link_libraries(robot PRIVATE
    "${MUJOCO_DIR}/lib/mujoco.lib"
    glfw
    opengl32
)

# Copy mujoco.dll next to the executable after build
add_custom_command(TARGET robot POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${MUJOCO_DIR}/bin/mujoco.dll"
        "$<TARGET_FILE_DIR:robot>/mujoco.dll"
    COMMENT "Copying mujoco.dll"
)
