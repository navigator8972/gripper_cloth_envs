cmake_minimum_required (VERSION 2.8.11)


set(PROJECT SnippetGripperCloth)
set(SOURCE SnippetGripperCloth.cpp SnippetGripperClothRender.cpp)

set(PHYSX_ROOT_DIR $ENV{PHYSX_ROOT_PATH})
set(PHYSX_VERSION_DIR "PhysX_3.4")
set(PHYSX_INCLUDE ${PHYSX_ROOT_DIR}/${PHYSX_VERSION_DIR}/Include)
set(PHYSX_SNIPPET_INCLUDE ${PHYSX_ROOT_DIR}/${PHYSX_VERSION_DIR}/Snippets)
set(PHYSX_PXSHARE_INCLUDE ${PHYSX_ROOT_DIR}/PxShared/include)

SET(SNIPPET_PLATFORM_SOURCES
        ${PHYSX_ROOT_DIR}/${PHYSX_VERSION_DIR}/Snippets/SnippetCommon/ClassicMain.cpp
)

#for 3.4
set(PHYSX_LIBRARIES libPhysX3Extensions.a libPhysX3_x64.so libPxPvdSDK_x64.so libPhysX3Vehicle.a libPhysX3CharacterKinematic_x64.so libPhysX3Cooking_x64.so libPhysX3Common_x64.so libPxFoundation_x64.so libSnippetUtils.a libSnippetRender.a)

#set(PHYSX_LIBRARIES libPhysXExtensions_static_64.a libPhysX_static_64.a libPhysXPvdSDK_static_64.a libPhysXVehicle_static_64.a libPhysXCharacterKinematic_static_64.a libPhysXCooking_static_64.a libPhysXCommon_static_64.a libPhysXFoundation_static_64.a libSnippetUtils_static_64.a libSnippetRender_static_64.a)
#set(PHYSX_LIBRARIES PhysXExtensions PhysX PhysXPvdSDK PhysXVehicle PhysXCharacterKinematic PhysXCooking PhysXCommon PhysXFoundation SnippetUtils)

add_compile_definitions(RENDER_SNIPPET)
add_definitions(-D_DEBUG)

link_directories(${PHYSX_ROOT_DIR}/${PHYSX_VERSION_DIR}/Bin/linux64 ${PHYSX_ROOT_DIR}/${PHYSX_VERSION_DIR}/Lib/linux64 ${PHYSX_ROOT_DIR}/${PHYSX_VERSION_DIR}/Snippets/lib/linux64 /usr/lib/x86_64-linux-gnu)

add_executable(SnippetGripperCloth ${SOURCE} ${SNIPPET_PLATFORM_SOURCES})
target_include_directories(SnippetGripperCloth PRIVATE ${PHYSX_INCLUDE} ${PHYSX_SNIPPET_INCLUDE} ${PHYSX_PXSHARE_INCLUDE})

target_link_libraries(SnippetGripperCloth ${PHYSX_LIBRARIES} GL GLU glut X11 rt pthread dl -Wl,-rpath='${ORIGIN}' -no-pie)
