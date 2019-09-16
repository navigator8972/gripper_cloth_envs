# HelloWorld is a minimal sample creating, stepping and deleting a Bullet dynamics world
set(PROJECT App_GripperCloth)

set(BULLET_PHYSICS_ROOT_DIR $ENV{BULLET_PHYSICS_ROOT_DIR})
set(BULLET_PHYSICS_LIBRARIES_DIR ${BULLET_PHYSICS_ROOT_DIR}/build_cmake)

INCLUDE_DIRECTORIES(
${BULLET_PHYSICS_ROOT_DIR}/src
${BULLET_PHYSICS_ROOT_DIR}/examples
${BULLET_PHYSICS_ROOT_DIR}/examples/ThirdPartyLibs/glad
)

ADD_DEFINITIONS(-DB3_USE_STANDALONE_EXAMPLE)

LINK_DIRECTORIES(
 ${BULLET_PHYSICS_LIBRARIES_DIR}/src/BulletDynamics 
 ${BULLET_PHYSICS_LIBRARIES_DIR}/src/BulletCollision 
 ${BULLET_PHYSICS_LIBRARIES_DIR}/src/LinearMath 
 ${BULLET_PHYSICS_LIBRARIES_DIR}/examples/OpenGLWindow
 ${BULLET_PHYSICS_LIBRARIES_DIR}/examples/ExampleBrowser 
 ${BULLET_PHYSICS_LIBRARIES_DIR}/src/Bullet3Common
 ${BULLET_PHYSICS_LIBRARIES_DIR}/src/BulletSoftBody
)

LINK_LIBRARIES(  pthread ${DL} )

ADD_DEFINITIONS("-DGLEW_INIT_OPENGL11_FUNCTIONS=1")
ADD_DEFINITIONS("-DGLEW_STATIC")
ADD_DEFINITIONS("-DGLEW_DYNAMIC_LOAD_ALL_GLX_FUNCTIONS=1")
#note this is important if bullet3 is built with default double script
ADD_DEFINITIONS("-DBT_USE_DOUBLE_PRECISION=1")

SET(PROJECT_GUI_SRCS
        App_GripperCloth.cpp
		main_opengl_single_example.cpp
        ${BULLET_PHYSICS_ROOT_DIR}/build3/bullet.rc
        main_opengl_single_example.cpp
)

ADD_EXECUTABLE(${PROJECT}Gui
                ${PROJECT_GUI_SRCS})

#using global path to ensure linking against latest Bullet. There might be 2.8.7 installed because of pybullet
target_link_libraries(${PROJECT}Gui 
	${BULLET_PHYSICS_LIBRARIES_DIR}/examples/ExampleBrowser/libBulletExampleBrowserLib.so 
	${BULLET_PHYSICS_LIBRARIES_DIR}/src/BulletSoftBody/libBulletSoftBody.so 
	${BULLET_PHYSICS_LIBRARIES_DIR}/src/Bullet3Common/libBullet3Common.so 
	${BULLET_PHYSICS_LIBRARIES_DIR}/src/BulletDynamics/libBulletDynamics.so 
	${BULLET_PHYSICS_LIBRARIES_DIR}/src/BulletCollision/libBulletCollision.so 
	${BULLET_PHYSICS_LIBRARIES_DIR}/src/LinearMath/libLinearMath.so 
	${BULLET_PHYSICS_LIBRARIES_DIR}/examples/OpenGLWindow/libOpenGLWindow.so)

IF (INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)
                        SET_TARGET_PROPERTIES(${PROJECT}Gui PROPERTIES  DEBUG_POSTFIX "_Debug")
                        SET_TARGET_PROPERTIES(${PROJECT}Gui PROPERTIES  MINSIZEREL_POSTFIX "_MinsizeRel")
                        SET_TARGET_PROPERTIES(${PROJECT}Gui PROPERTIES  RELWITHDEBINFO_POSTFIX "_RelWithDebugInfo")
ENDIF(INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)