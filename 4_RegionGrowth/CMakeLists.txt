cmake_minimum_required(VERSION 3.9)

project(LevealDBTry)
 
 
#设定编译参数
set(CMAKE_CXX_STANDARD 11)


 
#设定头文件路径
include_directories("D:/PCL 1.12.1/include/pcl-1.12"
                    "D:/PCL 1.12.1/3rdParty/Boost/include/boost-1_78"
                    "D:/PCL 1.12.1/3rdParty/Eigen/eigen3"
                    "D:/PCL 1.12.1/3rdParty/FLANN/include"
                    "D:/PCL 1.12.1/3rdParty/Qhull/include"
                    "D:/PCL 1.12.1/3rdParty/VTK/include/vtk-9.1"
                    )

 
 
#设定链接库的路径（一般使用第三方非系统目录下的库）
link_directories("D:/PCL 1.12.1/lib"
                 "D:/PCL 1.12.1/3rdParty/Boost/lib"
                 "D:/PCL 1.12.1/3rdParty/FLANN/lib"
                 "D:/PCL 1.12.1/3rdParty/Qhull/lib"
                 "D:/PCL 1.12.1/3rdParty/VTK/lib"
                 )


#生成可执行文件
set(NAME1 "RegionGrowth")
add_executable(${NAME1}   RegionGrowth.cpp)

find_package(Qt5 COMPONENTS Widgets REQUIRED)


#就是g++ 编译选项中-l后的内容，不要有多余空格
target_link_libraries(${NAME1}  Qt5::Widgets
pcl_common.lib
pcl_features.lib
pcl_filters.lib
pcl_io.lib
pcl_io_ply.lib
pcl_kdtree.lib
pcl_keypoints.lib
pcl_ml.lib
pcl_octree.lib
pcl_outofcore.lib
pcl_people.lib
pcl_recognition.lib
pcl_registration.lib
pcl_sample_consensus.lib
pcl_search.lib
pcl_segmentation.lib
pcl_stereo.lib
pcl_surface.lib
pcl_tracking.lib
pcl_visualization.lib
vtkChartsCore-9.1.lib
vtkCommonColor-9.1.lib
vtkCommonComputationalGeometry-9.1.lib
vtkCommonCore-9.1.lib
vtkCommonDataModel-9.1.lib
vtkCommonExecutionModel-9.1.lib
vtkCommonMath-9.1.lib
vtkCommonMisc-9.1.lib
vtkCommonSystem-9.1.lib
vtkCommonTransforms-9.1.lib
vtkDICOMParser-9.1.lib
vtkDomainsChemistry-9.1.lib
vtkDomainsChemistryOpenGL2-9.1.lib
vtkdoubleconversion-9.1.lib
vtkexodusII-9.1.lib
vtkexpat-9.1.lib
vtkFiltersAMR-9.1.lib
vtkFiltersCore-9.1.lib
vtkFiltersExtraction-9.1.lib
vtkFiltersFlowPaths-9.1.lib
vtkFiltersGeneral-9.1.lib
vtkFiltersGeneric-9.1.lib
vtkFiltersGeometry-9.1.lib
vtkFiltersHybrid-9.1.lib
vtkFiltersHyperTree-9.1.lib
vtkFiltersImaging-9.1.lib
vtkFiltersModeling-9.1.lib
vtkFiltersParallel-9.1.lib
vtkFiltersParallelImaging-9.1.lib
vtkFiltersPoints-9.1.lib
vtkFiltersProgrammable-9.1.lib
vtkFiltersSelection-9.1.lib
vtkFiltersSMP-9.1.lib
vtkFiltersSources-9.1.lib
vtkFiltersStatistics-9.1.lib
vtkFiltersTexture-9.1.lib
vtkFiltersTopology-9.1.lib
vtkFiltersVerdict-9.1.lib
vtkfreetype-9.1.lib
vtkGeovisCore-9.1.lib
vtkgl2ps-9.1.lib
vtkglew-9.1.lib
vtkhdf5-9.1.lib
vtkhdf5_hl-9.1.lib
vtkImagingColor-9.1.lib
vtkImagingCore-9.1.lib
vtkImagingFourier-9.1.lib
vtkImagingGeneral-9.1.lib
vtkImagingHybrid-9.1.lib
vtkImagingMath-9.1.lib
vtkImagingMorphological-9.1.lib
vtkImagingSources-9.1.lib
vtkImagingStatistics-9.1.lib
vtkImagingStencil-9.1.lib
vtkInfovisCore-9.1.lib
vtkInfovisLayout-9.1.lib
vtkInteractionImage-9.1.lib
vtkInteractionStyle-9.1.lib
vtkInteractionWidgets-9.1.lib
vtkIOAMR-9.1.lib
vtkIOAsynchronous-9.1.lib
vtkIOCityGML-9.1.lib
vtkIOCore-9.1.lib
vtkIOEnSight-9.1.lib
vtkIOExodus-9.1.lib
vtkIOExport-9.1.lib
vtkIOExportGL2PS-9.1.lib
vtkIOExportPDF-9.1.lib
vtkIOGeometry-9.1.lib
vtkIOImage-9.1.lib
vtkIOImport-9.1.lib
vtkIOInfovis-9.1.lib
vtkIOLegacy-9.1.lib
vtkIOLSDyna-9.1.lib
vtkIOMINC-9.1.lib
vtkIOMotionFX-9.1.lib
vtkIOMovie-9.1.lib
vtkIONetCDF-9.1.lib
vtkIOOggTheora-9.1.lib
vtkIOParallel-9.1.lib
vtkIOParallelXML-9.1.lib
vtkIOPLY-9.1.lib
vtkIOSegY-9.1.lib
vtkIOSQL-9.1.lib
vtkIOTecplotTable-9.1.lib
vtkIOVeraOut-9.1.lib
vtkIOVideo-9.1.lib
vtkIOXML-9.1.lib
vtkIOXMLParser-9.1.lib
vtkjpeg-9.1.lib
vtkjsoncpp-9.1.lib
vtklibharu-9.1.lib
vtklibproj-9.1.lib
vtklibxml2-9.1.lib
vtkloguru-9.1.lib
vtklz4-9.1.lib
vtklzma-9.1.lib
vtkmetaio-9.1.lib
vtknetcdf-9.1.lib
vtkogg-9.1.lib
vtkParallelCore-9.1.lib
vtkParallelDIY-9.1.lib
vtkpng-9.1.lib
vtkpugixml-9.1.lib
vtkRenderingAnnotation-9.1.lib
vtkRenderingContext2D-9.1.lib
vtkRenderingContextOpenGL2-9.1.lib
vtkRenderingCore-9.1.lib
vtkRenderingFreeType-9.1.lib
vtkRenderingGL2PSOpenGL2-9.1.lib
vtkRenderingImage-9.1.lib
vtkRenderingLabel-9.1.lib
vtkRenderingLOD-9.1.lib
vtkRenderingOpenGL2-9.1.lib
vtkRenderingSceneGraph-9.1.lib
vtkRenderingUI-9.1.lib
vtkRenderingVolume-9.1.lib
vtkRenderingVolumeOpenGL2-9.1.lib
vtkRenderingVtkJS-9.1.lib
vtksqlite-9.1.lib
vtksys-9.1.lib
vtkTestingRendering-9.1.lib
vtktheora-9.1.lib
vtktiff-9.1.lib
vtkverdict-9.1.lib
vtkViewsContext2D-9.1.lib
vtkViewsCore-9.1.lib
vtkViewsInfovis-9.1.lib
vtkWrappingTools-9.1.lib
vtkzlib-9.1.lib
)

# 程序运行时不打开黑窗
# Set_Target_Properties(${NAME1} PROPERTIES LINK_FLAGS_RELEASE "/SUBSYSTEM:WINDOWS /ENTRY:mainCRTStartup") 
 
