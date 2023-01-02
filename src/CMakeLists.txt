cmake_minimum_required(VERSION 3.13)
project(VehicleSim)

add_subdirectory(examples)
add_subdirectory(graphics)
add_subdirectory(input)
add_subdirectory(physics)
add_subdirectory(util)

set(BULLET_PHYSICS_ROOT
        ../external/bullet
        )

set(VehicleTester_SRCS
        main.cpp
        VehicleSim.cpp
        )

add_executable(${PROJECT_NAME}
        ${VehicleTester_SRCS}
        )

target_include_directories(${PROJECT_NAME}
        PUBLIC
        ${BULLET_PHYSICS_ROOT}/src
        ${BULLET_PHYSICS_ROOT}/btgui
        ${BULLET_PHYSICS_ROOT}/examples
        ${BULLET_PHYSICS_ROOT}/examples/ThirdPartyLibs/glad
        ${PROJECT_SOURCE_DIR}
        )

target_link_libraries(${PROJECT_NAME}
        OpenGLWindow
        graphics
        physics
        util
        )

#some code to support OpenGL and Glew cross platform
IF (WIN32)
    target_include_directories(${PROJECT_NAME}
            PUBLIC
            ${BULLET_PHYSICS_ROOT}/btgui/OpenGLWindow/GlewWindows
            )
    add_definitions(-DGLEW_STATIC)
    target_link_libraries(${PROJECT_NAME}
            ${OPENGL_gl_LIBRARY}
            ${OPENGL_glu_LIBRARY}
            )
ELSE(WIN32)
    IF(APPLE)
        find_library(COCOA NAMES Cocoa)
        message(${COCOA})
        target_link_libraries(${PROJECT_NAME}
                ${COCOA}
                ${OPENGL_gl_LIBRARY}
                ${OPENGL_glu_LIBRARY}
                )
    ELSE(APPLE)
        target_include_directories(${PROJECT_NAME}
                PUBLIC
                ${BULLET_PHYSICS_ROOT}/btgui/OpenGLWindow/GlewWindows
        )
        add_definitions("-DGLEW_INIT_OPENGL11_FUNCTIONS=1")
        add_definitions("-DGLEW_STATIC")
        add_definitions("-DGLEW_DYNAMIC_LOAD_ALL_GLX_FUNCTIONS=1")

        find_package(Threads)
        target_link_libraries(${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT} ${DL} )
    ENDIF(APPLE)
ENDIF(WIN32)

if(MSVC)
    if (NOT USE_MSVC_RUNTIME_LIBRARY_DLL)
        # We statically link to reduce dependencies
        foreach(flag_var CMAKE_CXX_FLAGS CMAKE_CXX_FLAGS_DEBUG CMAKE_CXX_FLAGS_RELEASE CMAKE_CXX_FLAGS_MINSIZEREL CMAKE_CXX_FLAGS_RELWITHDEBINFO CMAKE_C_FLAGS CMAKE_C_FLAGS_DEBUG CMAKE_C_FLAGS_RELEASE CMAKE_C_FLAGS_MINSIZEREL CMAKE_C_FLAGS_RELWITHDEBINFO )
            if(${flag_var} MATCHES "/MD")
                string(REGEX REPLACE "/MD" "/MT" ${flag_var} "${${flag_var}}")
            endif(${flag_var} MATCHES "/MD")
            if(${flag_var} MATCHES "/MDd")
                string(REGEX REPLACE "/MDd" "/MTd" ${flag_var} "${${flag_var}}")
            endif(${flag_var} MATCHES "/MDd")
        endforeach(flag_var)
    endif (NOT USE_MSVC_RUNTIME_LIBRARY_DLL)
endif(MSVC)