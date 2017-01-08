# HelloWorld is a minimal sample creating, stepping and deleting a Bullet dynamics world

set (BULLET_PHYSICS_SOURCE_DIR "../../")
set(CMAKE_CXX_FLAGS "-O3 -g -std=c++1z")

INCLUDE_DIRECTORIES(
${BULLET_PHYSICS_SOURCE_DIR}/src
)
# set (CMAKE_CXX_STANDARD 14)
link_directories("../../bin")
add_definitions("-D BT_FORCE_DOUBLE_FUNCTIONS")
add_definitions("-D B3_FORCE_DOUBLE_FUNCTIONS")
LINK_LIBRARIES(
 BulletDynamics_gmake_x64_release BulletCollision_gmake_x64_release LinearMath_gmake_x64_release
)

IF (WIN32)
	ADD_EXECUTABLE(App_HelloWorld
		HelloWorld.cpp
		${BULLET_PHYSICS_SOURCE_DIR}/build3/bullet.rc
	)
ELSE()
	ADD_EXECUTABLE(App_HelloWorld
		HelloWorld.cpp
	)
ENDIF()




IF (INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)
			SET_TARGET_PROPERTIES(App_HelloWorld PROPERTIES  DEBUG_POSTFIX "_Debug")
			SET_TARGET_PROPERTIES(App_HelloWorld PROPERTIES  MINSIZEREL_POSTFIX "_MinsizeRel")
			SET_TARGET_PROPERTIES(App_HelloWorld PROPERTIES  RELWITHDEBINFO_POSTFIX "_RelWithDebugInfo")
ENDIF(INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)
