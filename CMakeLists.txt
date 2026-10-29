cmake_minimum_required(VERSION 3.20)
project(robot C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

set(MUJOCO_DIR "C:/dev/mujoco" CACHE PATH "MuJoCo installation directory")

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

add_executable(seven_cylinders_test
    tests/seven_cylinders_test.c
)
add_executable(controller_pose_test
    tests/controller_pose_test.c
    src/sim.c
    src/controller.c
)
target_include_directories(controller_pose_test PRIVATE src "${MUJOCO_DIR}/include")
target_link_libraries(controller_pose_test PRIVATE "${MUJOCO_DIR}/lib/mujoco.lib")
add_custom_command(TARGET controller_pose_test POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${MUJOCO_DIR}/bin/mujoco.dll"
        "$<TARGET_FILE_DIR:controller_pose_test>/mujoco.dll"
)

target_include_directories(robot PRIVATE
    src
    "${MUJOCO_DIR}/include"
)
target_include_directories(seven_cylinders_test PRIVATE
    "${MUJOCO_DIR}/include"
)
target_link_libraries(robot PRIVATE
    "${MUJOCO_DIR}/lib/mujoco.lib"
    glfw
    opengl32
)
target_link_libraries(seven_cylinders_test PRIVATE
    "${MUJOCO_DIR}/lib/mujoco.lib"
)

# Copy mujoco.dll next to the executable after build
add_custom_command(TARGET robot POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${MUJOCO_DIR}/bin/mujoco.dll"
        "$<TARGET_FILE_DIR:robot>/mujoco.dll"
    COMMENT "Copying mujoco.dll"
)

add_custom_command(TARGET seven_cylinders_test POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${MUJOCO_DIR}/bin/mujoco.dll"
        "$<TARGET_FILE_DIR:seven_cylinders_test>/mujoco.dll"
    COMMENT "Copying mujoco.dll for model test"
)

enable_testing()
add_test(
    NAME seven_cylinders_model
    COMMAND seven_cylinders_test "${CMAKE_SOURCE_DIR}/assets/seven_cylinders.xml"
)
add_test(
    NAME controller_pose
    COMMAND controller_pose_test "${CMAKE_SOURCE_DIR}/assets/seven_cylinders.xml"
)
