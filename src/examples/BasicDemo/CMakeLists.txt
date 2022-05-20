# App_BasicExample is a minimal sample creating, stepping and deleting a Bullet dynamics world
cmake_minimum_required(VERSION 3.13)
project(App_BasicExample_Michael)

SET(BULLET_PHYSICS_ROOT ../../../external/bullet)
SET(BULLET_PHYSICS_EXAMPLES ../../../external/bullet/examples)

INCLUDE_DIRECTORIES(
        ../../../external/bullet/src
)

LINK_LIBRARIES(
 BulletDynamics BulletCollision LinearMath
)

IF (WIN32)
	ADD_EXECUTABLE(${PROJECT_NAME}
            BasicExample.cpp
            main.cpp
		${BULLET_PHYSICS_ROOT}/build3/bullet.rc
	)
ELSE()
	ADD_EXECUTABLE(${PROJECT_NAME}
            BasicExample.cpp
            main.cpp
	)
ENDIF()

IF (INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)
			SET_TARGET_PROPERTIES(App_BasicExample PROPERTIES  DEBUG_POSTFIX "_Debug")
			SET_TARGET_PROPERTIES(App_BasicExample PROPERTIES  MINSIZEREL_POSTFIX "_MinsizeRel")
			SET_TARGET_PROPERTIES(App_BasicExample PROPERTIES  RELWITHDEBINFO_POSTFIX "_RelWithDebugInfo")
ENDIF(INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)





#################
# Standalone BasicExampleGui using OpenGL (but not the example browser)

INCLUDE_DIRECTORIES(
        ${BULLET_PHYSICS_ROOT}/src
        ${BULLET_PHYSICS_ROOT}/btgui
        ${BULLET_PHYSICS_ROOT}/examples
        ${BULLET_PHYSICS_ROOT}/examples/ThirdPartyLibs/glad
)


SET(AppBasicExampleGui_SRCS
        BasicExample.cpp
        ${BULLET_PHYSICS_ROOT}/build3/bullet.rc
	    ${BULLET_PHYSICS_EXAMPLES}/StandaloneMain/main_opengl_single_example.cpp
        ${BULLET_PHYSICS_EXAMPLES}/ExampleBrowser/OpenGLGuiHelper.cpp
        ${BULLET_PHYSICS_EXAMPLES}/ExampleBrowser/GL_ShapeDrawer.cpp
        ${BULLET_PHYSICS_EXAMPLES}/ExampleBrowser/CollisionShape2TriangleMesh.cpp
        ${BULLET_PHYSICS_EXAMPLES}/Utils/b3Clock.cpp
)

#this define maps StandaloneExampleCreateFunc to the right 'CreateFunc'
ADD_DEFINITIONS(-DB3_USE_STANDALONE_EXAMPLE)

LINK_LIBRARIES(
        BulletDynamics BulletCollision LinearMath OpenGLWindow Bullet3Common 
)

#some code to support OpenGL and Glew cross platform
IF (WIN32)
        INCLUDE_DIRECTORIES(
             ${BULLET_PHYSICS_ROOT}/btgui/OpenGLWindow/GlewWindows
        )
        ADD_DEFINITIONS(-DGLEW_STATIC)
	LINK_LIBRARIES( ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY} )
ELSE(WIN32)
        IF(APPLE)
                find_library(COCOA NAMES Cocoa)
                MESSAGE(${COCOA})
                link_libraries(${COCOA} ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY})

        ELSE(APPLE)
                INCLUDE_DIRECTORIES(
                        ${BULLET_PHYSICS_ROOT}/btgui/OpenGLWindow/GlewWindows
                )
                ADD_DEFINITIONS("-DGLEW_INIT_OPENGL11_FUNCTIONS=1")
                ADD_DEFINITIONS("-DGLEW_STATIC")
                ADD_DEFINITIONS("-DGLEW_DYNAMIC_LOAD_ALL_GLX_FUNCTIONS=1")

                FIND_PACKAGE(Threads)
                LINK_LIBRARIES( ${CMAKE_THREAD_LIBS_INIT} ${DL} )
        ENDIF(APPLE)
ENDIF(WIN32)


ADD_EXECUTABLE(${PROJECT_NAME}_Gui
                ${AppBasicExampleGui_SRCS}
)


IF (INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)
                        SET_TARGET_PROPERTIES(AppBasicExampleGui PROPERTIES  DEBUG_POSTFIX "_Debug")
                        SET_TARGET_PROPERTIES(AppBasicExampleGui PROPERTIES  MINSIZEREL_POSTFIX "_MinsizeRel")
                        SET_TARGET_PROPERTIES(AppBasicExampleGui PROPERTIES  RELWITHDEBINFO_POSTFIX "_RelWithDebugInfo")
ENDIF(INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)


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