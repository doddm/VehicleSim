cmake_minimum_required(VERSION 3.13)
project(bulletGame)

set(CMAKE_CXX_STANDARD 14)

add_subdirectory(external/bullet)
add_subdirectory(src)

set(INCLUDE_DIRS
        external/bullet/src
        )

target_include_directories(${PROJECT_NAME} PUBLIC ${INCLUDE_DIRS})

target_link_directories(${PROJECT_NAME} PUBLIC lib)
target_link_libraries(${PROJECT_NAME} BulletDynamics BulletCollision LinearMath Bullet3Common)

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