#ifndef PCLVIEWER_H
#define PCLVIEWER_H

#ifndef Q_MOC_RUN

#include <iostream>

// Qt
#include <QMainWindow>
#include <QFileDialog>

// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

// Eigen Library
#include <Eigen/Dense>

// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>

#include <stairs/preanalysis.h>

#include <stairs/regions.h>

#include <stairs/regiongrowing.h>
#include <stairs/voxSAC.h>
#include <stairs/splitmerge.h>

#include <stairs/planeshape.h>
#include <stairs/recognition.h>

#include <stairs/Stairs.h>

#include <vtkVersion.h>
#include <vtkPlaneSource.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>

#include <vtkLookupTable.h>
#include <vtkFloatArray.h>
#include <vtkCellData.h>

#endif

typedef pcl::PointXYZ PointT;
typedef pcl::PointNormal PointNT;
typedef pcl::Normal Normal;
typedef pcl::PointXYZRGB PointTC;

typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<PointNT> PointCloudN;
typedef pcl::PointCloud<Normal> NormalCloud;
typedef pcl::PointCloud<PointTC> PointCloudC;

namespace Ui
{
  class PCLViewer;
}

class PCLViewer : public QMainWindow
{
  Q_OBJECT

public:
  explicit PCLViewer (QWidget *parent = 0);
  ~PCLViewer ();

public slots:
  /** @brief Triggered whenever the "Load file" button is clicked */
  void loadFileButtonPressed();

  void regionGrowingButtonPressed();

  void voxelSacButtonPressed();
  void splitMergeButtonPressed();

  void filterCloud();
  void preanalysis();
  void makeColored(PointCloudT::Ptr tempCloud);

  void drawAllSol();

  void loadFile(std::string selecFile);
  void loadSettings(std::string fileName);

  void psGo(int mehtod);
//  void psStairRailGo();

  void psShow();

  void unmarkButtons();

  void rgAtSeed();
  void rgAtNeigh();
  void rgRdSeed();
  void rgRdNeigh();
  void showOrig();

  void recStairs();

  void showSegRegMap();
  void showSegNorMap();

  void showPREregMap();
  void showPREnorMap();

  void completeRun();

  void refine();

  void showFilterRegMap();

protected:
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  PointCloudT::Ptr mainCloud;

  PointCloudT::Ptr refCloud;
  PointCloudT::Ptr showRefCloud;

  PointCloudT::Ptr prepCloud;

  NormalCloud::Ptr prepNomalCloud;
  PointCloudT floorPC;

  PointCloudC::Ptr activeCloud;
  PointCloudC::Ptr visibleCloud;
  PointCloudC::Ptr visibleFloor;
  PointCloudT::Ptr backCloud;
  PointCloudT::Ptr restCloud;

  PointCloudT centerCloud;

  PointCloudC prepNorMap;

  PointCloudC rgRegMap;
  PointCloudC rgNorMap;

  PointCloudC filterRegMap;
  PointCloudC filterNorMap;

  regions regSeg;

  regions filteredRegions;
  StairVector detectedStairs;
  regions activeRegion;

  regions stairTreads;
  regions stairRises;

  regions stairRiseFilteredRegions;
  regions stairTreadFilteredRegions;

  planeshape psProc;

  bool refCreated;
  bool showBackground;

  int autoCounter;
  int autoCounter2;
  int autoCounter3;
  int measuredPoints;

  std::stack<vtkSmartPointer<vtkActor> > vtkPointVec;

  bool loadingFile;

	Eigen::Vector3d camPos1;
	Eigen::Vector3d camView1;
	Eigen::Vector3d port1;

	Eigen::Vector3d camPos2;
	Eigen::Vector3d camView2;
	Eigen::Vector3d port2;

  std::string currFileName;

private:
  Ui::PCLViewer *ui;

  double downsampleRaster;

  bool floorVisible;
  bool origRGActive;

  double loadTime;
  double preTime;
  double neTime;
  double rgTime;
  double rgOrigTime;
  double rgOrigChillTime;
  double vsTime;
  double vsNeighTime;
  double smTime;
  double smNeighTime;
};

#endif // PCLVIEWER_H

