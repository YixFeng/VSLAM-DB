PROJECT( bundle_adjustment )
CMAKE_MINIMUM_REQUIRED( VERSION 2.8 )

set( CMAKE_CXX_FLAGS "-std=c++11" )
set( CMAKE_BUILD_TYPE Release )

LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake )

find_package( OpenCV 4 REQUIRED )
find_package( Pangolin REQUIRED )
find_package( Eigen3 REQUIRED )
find_package( Sophus REQUIRED )
find_package( G2O REQUIRED )
find_package( CSparse REQUIRED ) #用来生成稀疏矩阵

include_directories( ${EIGEN3_INCLUDE_DIRS}
                     ${PROJECT_SOURCE_DIR}/include
                     ${OpenCV_INCLUDE_DIRS}
                     ${G2O_INCLUDE_DIRS}
                     ${Pangolin_INCLUDE_DIRS}
                     ${CSPARSE_INCLUDE_DIR} # 不知道为什么这个是DIR，DIRS就会找不到cs.h
                     ${Sophus_INCLUDE_DIRS} )

SET( G2O_LIBS g2o_csparse_extension g2o_stuff g2o_core cxsparse
        cholmod g2o_cli g2o_simulator g2o_solver_slam2d_linear
        g2o_types_icp g2o_types_slam2d g2o_solver_csparse g2o_solver_structure_only
        g2o_types_sba g2o_types_slam3d g2o_solver_dense g2o_parser g2o_solver_pcg
        g2o_types_data g2o_types_sim3 )

add_subdirectory( src )