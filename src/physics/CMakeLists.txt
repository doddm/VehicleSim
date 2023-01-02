project(physics)

set(BULLET_PHYSICS_ROOT ../../external/bullet)

add_library(${PROJECT_NAME}
        Aerodynamics.cpp
        Raycast.cpp
        Tire.cpp
        Vehicle.cpp
        )

target_include_directories( ${PROJECT_NAME}
        PUBLIC
        ${PROJECT_SOURCE_DIR}
        ${BULLET_PHYSICS_ROOT}/src
        )
