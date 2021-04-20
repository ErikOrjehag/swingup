cmake_minimum_required( VERSION 3.0.0 )
project( bullet VERSION 0.1.0 )

set( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} )

#SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -ffast-math -O3")

include( CTest )
enable_testing()

find_package( ACADO REQUIRED )

include_directories(
    ${ACADO_INCLUDE_DIRS} 
    codegen 
    codegen/qpoases
    codegen/qpoases/INCLUDE
    codegen/qpoases/SRC
)

add_executable( genmpc genmpc.cpp)
#add_executable( genmpc genwalk.cpp)

target_link_libraries( genmpc ${ACADO_SHARED_LIBRARIES} )

set( CPACK_PROJECT_NAME ${PROJECT_NAME} )
set( CPACK_PROJECT_VERSION ${PROJECT_VERSION} )
include( CPack )



