project(graphics)

set(BULLET_PHYSICS_ROOT ../../external/bullet)
set(BULLET_PHYSICS_EXAMPLES ../../external/bullet/examples)

add_library(${PROJECT_NAME}
        ${BULLET_PHYSICS_EXAMPLES}/ExampleBrowser/CollisionShape2TriangleMesh.cpp
        OpenGLGuiHelper.cpp
        )

target_include_directories(${PROJECT_NAME}
        PUBLIC
        ${BULLET_PHYSICS_ROOT}/src
        ${BULLET_PHYSICS_ROOT}/examples
        ..
        )

target_link_libraries(${PROJECT_NAME}
        BulletDynamics
        BulletCollision
        LinearMath
        Bullet3Common
        )
