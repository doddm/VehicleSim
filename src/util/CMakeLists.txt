project(util)

set(BULLET_PHYSICS_EXAMPLES ../../external/bullet/examples)

add_library(${PROJECT_NAME}
        DebugUtil.h
        ${BULLET_PHYSICS_EXAMPLES}/Utils/b3Clock.cpp
        )

target_include_directories( ${PROJECT_NAME}
        PUBLIC
        ${PROJECT_SOURCE_DIR}
        )