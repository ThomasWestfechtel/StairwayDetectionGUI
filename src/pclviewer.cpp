#include <stairs/pclviewer.h>
#include <../build/ui_pclviewer.h>

//using namespace std;

PCLViewer::PCLViewer (QWidget *parent) :
  QMainWindow (parent),
  ui (new Ui::PCLViewer)
{
  ui->setupUi (this);
  this->setWindowTitle ("Stair detector");

  // Set up the QVTK window
  viewer.reset (new pcl::visualization::PCLVisualizer ("viewer",false));
  ui->qvtkWidget->SetRenderWindow (viewer->getRenderWindow ());
  viewer->setupInteractor (ui->qvtkWidget->GetInteractor (), ui->qvtkWidget->GetRenderWindow ());
  ui->qvtkWidget->update ();

  showBackground = true;
  loadSettings("Settings.txt");

  measuredPoints = 1356000;

  loadingFile = false;
  refCreated =false;
  origRGActive = false;

  // Connect "Load" button
  connect (ui->pushButton_load, SIGNAL(clicked ()), this, SLOT(loadFileButtonPressed ()));

  // Connect button for whole Stairway Detection
  connect (ui->pushButton_fullRun, SIGNAL(clicked ()), this, SLOT(completeRun ()));

  // Connect buttons to see different processing results
  connect (ui->pushButton_showOrig, SIGNAL(clicked ()), this, SLOT(showOrig ()));
  connect (ui->pushButton_rgRegMap, SIGNAL(clicked ()), this, SLOT(showSegRegMap ()));
  connect (ui->pushButton_rgNorMap, SIGNAL(clicked ()), this, SLOT(showSegNorMap ()));
  connect (ui->pushButton_PREregMap, SIGNAL(clicked ()), this, SLOT(showPREregMap ()));
  connect (ui->pushButton_PREnorMap, SIGNAL(clicked ()), this, SLOT(showPREnorMap ()));
  connect (ui->pushButton_psShowBoth, SIGNAL(clicked ()), this, SLOT(psShow ()));
  connect(ui->pushButton_drawAllSol,SIGNAL(clicked ()), this, SLOT(drawAllSol ()));


  // Connect buttons that are mutually exclusive
  connect (ui->checkBox_rgAtSeed, SIGNAL(clicked ()), this, SLOT(rgAtSeed ()));
  connect (ui->checkBox_rgAtNeigh, SIGNAL(clicked ()), this, SLOT(rgAtNeigh ()));
  connect (ui->checkBox_rgRdSeed, SIGNAL(clicked ()), this, SLOT(rgRdSeed ()));
  connect (ui->checkBox_rgRdNeigh, SIGNAL(clicked ()), this, SLOT(rgRdNeigh ()));


  mainCloud.reset (new PointCloudT);
  prepCloud.reset (new PointCloudT);
  prepNomalCloud.reset(new NormalCloud);

  activeCloud.reset (new PointCloudC);
  visibleCloud.reset (new PointCloudC);

  floorVisible=true;

	camPos1 <<-6.67228,-6.76472,5.13496;
	camView1 << 0.248512,0.311137,0.917298;
	port1 << 0.353083,1.48043,0.435009;

	camPos2 << -4.46804,-1.46859,3.29821;
	camView2 <<0.408491,0.0417931,0.911805;
	port2 << 0.338004,0.049671,1.0755;

  viewer->addPointCloud (activeCloud, "cloud");
  viewer->setBackgroundColor (255,255,255);
  viewer->resetCamera ();
//  viewer->addCoordinateSystem(1.0,2.0,0.0,-1.0);
  ui->qvtkWidget->update ();

  autoCounter = 0;
}



// Stair Detection

void PCLViewer::recStairs()
{
	psProc.setInputRegions(regSeg);

    double pfS = pcl::getTime();
    Eigen::Vector2f psWidth;
    psWidth[0] = ui ->doubleSpinBox_psScWidthMin->value();
    psWidth[1] = ui ->doubleSpinBox_psScWidthMax->value();

    Eigen::Vector2f psHeight;
    psHeight[0] = ui ->doubleSpinBox_psScHeightMin->value();
    psHeight[1] = ui ->doubleSpinBox_psScHeightMax->value();

    Eigen::Vector2f psDepth;
    psDepth[0] = ui ->doubleSpinBox_psScDepthMin->value();
    psDepth[1] = ui ->doubleSpinBox_psScDepthMax->value();


    psProc.setAngleMargin(ui ->doubleSpinBox_psScMarg->value());
    psProc.setWidthReq(psWidth);
    psProc.setRiserHeight(psHeight);
    psProc.setTreadDepth(psDepth);


    psProc.filterSc(stairTreads, stairRises);

    double pfE = pcl::getTime();

    double refS = pcl::getTime();
    refine();
    double refE = pcl::getTime();
}


void PCLViewer::refine()
{
    //filteredRegions
    centerCloud.clear();

    recognition stairDetect;

//    stairDetect.setGraphMeth((ui->checkBox_recSearchMeth->isChecked()));

    int mode = ui->comboBox_recInputMode->currentIndex();

	stairDetect.setInputRegions(regSeg);

    Eigen::Vector2f nDistance;
    nDistance[0] = ui ->doubleSpinBox_recNDMin->value();
    nDistance[1] = ui ->doubleSpinBox_recNDMax->value();

    Eigen::Vector2f pDistance;
    pDistance[0] = ui ->doubleSpinBox_recPDMin->value();
    pDistance[1] = ui ->doubleSpinBox_recPDMax->value();


    Eigen::Vector2f psWidth;
    psWidth[0] = ui ->doubleSpinBox_psScWidthMin->value();
    psWidth[1] = ui ->doubleSpinBox_psScWidthMax->value();
    stairDetect.setWidthReq(psWidth);

    stairDetect.setParFlag(ui->checkBox_recParFlag->isChecked());
    stairDetect.setParAngle(ui ->doubleSpinBox_recParAngle->value());

    stairDetect.setNDFlag(ui->checkBox_recNDFlag->isChecked());
    stairDetect.setNDistance(nDistance);

    stairDetect.setPDFlag(ui->checkBox_recPDFlag->isChecked());
    stairDetect.setPDistance(pDistance);
//
    stairDetect.setFloorFlag(false);// true for known floor - false for unknown (floor should be at z = 0.0)
    stairDetect.setWidthFlag(ui->checkBox_recWidthFlag->isChecked());
//
    stairDetect.setUpdateFlag(false);
    stairDetect.setOptimizeFlag(ui->checkBox_recOptimize->isChecked());

    stairDetect.setStairRailFlag(false);

    stairDetect.setStairTreadRegions(stairTreads);
    stairDetect.setStairRiseRegions(stairRises);

    stairDetect.run(detectedStairs);
    std::cout<<"Size is: "<<detectedStairs.size()<<std::endl;

    drawAllSol();


	  ui->pushButton_PREregMap->setEnabled(true);
	  ui->pushButton_PREnorMap->setEnabled(true);
	  ui->pushButton_rgRegMap->setEnabled(true);
	  ui->pushButton_rgNorMap->setEnabled(true);
	  ui->pushButton_psShowBoth->setEnabled(true);
	  ui->pushButton_drawAllSol->setEnabled(true);
}


void PCLViewer::drawAllSol ()
{
	  unmarkButtons();
	  ui->pushButton_drawAllSol->setStyleSheet("background-color: lightblue");
	filterRegMap.clear();
	std::cout<<"Size is: "<<detectedStairs.size()<<std::endl;
	if(detectedStairs.size()>0)
	{
		for(int stairCoeffIdx =0; stairCoeffIdx < detectedStairs.size(); stairCoeffIdx++)
		{
			Stairs stairCoefficients;
			stairCoefficients = detectedStairs.at(stairCoeffIdx);

			float steigung = atan(stairCoefficients.dir[2] / sqrt(pow(stairCoefficients.dir[0],2) + pow(stairCoefficients.dir[1],2)));

			std::cout<<std::endl<<"Step depth:   "<<round(1000*sqrt(pow(stairCoefficients.dir[0],2) + pow(stairCoefficients.dir[1],2)))<<std::endl;
			std::cout<<"Step height:  "<<round(1000*stairCoefficients.dir[2])<<std::endl;
			std::cout<<"Step width:   "<<round(1000*stairCoefficients.width)<<std::endl;
			std::cout<<"Slope is:     "<<round(100*steigung/M_PI*180)<<std::endl;
			std::cout<<"Amount of stair parts: "<<stairCoefficients.size()<<std::endl<<std::endl;

			float stairAngle = atan2(stairCoefficients.dir[1],stairCoefficients.dir[0]);
			float xStairDist = stairCoefficients.pos[0];
			float yStairDist = stairCoefficients.pos[1];

			Eigen::Vector2f sepDist;
			sepDist[0] = cos(stairAngle) * xStairDist + sin(stairAngle) * yStairDist;
			sepDist[1] = - sin(stairAngle) * xStairDist + cos(stairAngle) * yStairDist;

			std::cout<<"Dist in X is: "<<round(1000*(stairCoefficients.pos[0]-0.2))<<std::endl;
			std::cout<<"Dist in Y is: "<<round(1000*stairCoefficients.pos[1])<<std::endl;

	//        std::cout<<"Distance is:  "<<round(1000*sqrt(pow(stairCoefficients.pos[0]-0.2,2) + pow(stairCoefficients.pos[1],2)))<<std::endl<<std::endl;

			std::cout<<"Dist par is:  "<<round(1000*sepDist[0])<<std::endl;
			std::cout<<"Dist ort is:  "<<round(1000*sepDist[1])<<std::endl;
			std::cout<<"Anchor point is: "<<stairCoefficients.anchPoint<<std::endl;

//	        std::cout<<"Dist in X is: "<<round(1000*(stairCoefficients.pos[0]))<<" mm"<<std::endl;
//			std::cout<<"Dist in Y is: "<<round(1000*stairCoefficients.pos[1])<<" mm"<<std::endl;


			std::cout<<"Angle is:     "<<round(100*atan2(stairCoefficients.dir[1],stairCoefficients.dir[0])/M_PI*180)<<std::endl;

			if(ui->checkBox_recPartCol->isChecked())
				filterRegMap += detectedStairs.getColoredParts(stairCoeffIdx);
			else
				filterRegMap += detectedStairs.getColoredCloud(stairCoeffIdx);
		}
		activeRegion = detectedStairs.getAllRegions();

		filterNorMap = detectedStairs.getColoredParts();

		showFilterRegMap();
	}


	viewer->removeAllShapes();

	while(!vtkPointVec.empty())
	{
		vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
		actor = vtkPointVec.top();
		vtkPointVec.pop();
		viewer->getRenderWindow()->GetRenderers()->GetFirstRenderer()->RemoveActor(actor);
	}


	for(int stairSol = 0; stairSol < detectedStairs.size(); stairSol ++)
	{
		Stairs stairCoefficients = detectedStairs.at(stairSol);



		Eigen::Vector3f coeffDir = stairCoefficients.dir;

		Eigen::Vector3f stairNorm;
		stairNorm = stairCoefficients.dir;
		stairNorm[2] = 0;
		stairNorm.normalize();

		Eigen::Vector3f tempDir;
		tempDir << 0,0,1;

		Eigen::Vector3f widthDir;
		widthDir = stairNorm.cross(tempDir);

		widthDir.normalize();

		Eigen::Vector3f stairBegin = stairCoefficients.pos;


		if(stairCoefficients.anchPoint == 1)
		{
			stairBegin[0] += widthDir[0] * stairCoefficients.width;
			stairBegin[1] += widthDir[1] * stairCoefficients.width;
		}
		if(stairCoefficients.anchPoint == 2)
		{
			stairBegin[0] += widthDir[0] * stairCoefficients.width * 0.5;
			stairBegin[1] += widthDir[1] * stairCoefficients.width * 0.5;
		}

		for(int step = stairCoefficients.stairOffset; step < stairCoefficients.stairCount + stairCoefficients.stairOffset; step++)
		{


			Eigen::Vector3f stepPos;
			stepPos = stairBegin + step * stairCoefficients.dir;


			// Add the stair risers ///
			{
				  // Create cell data
				  vtkSmartPointer<vtkFloatArray> cellData = vtkSmartPointer<vtkFloatArray>::New();
				  for (int i = 0; i < 1; i++)
				  {
					  cellData->InsertNextValue(i + 1);
				  }

				  // Create a lookup table to map cell data to colors
				  vtkSmartPointer<vtkLookupTable> lut = vtkSmartPointer<vtkLookupTable>::New();
				  int tableSize = std::max(1, 1);
				  lut->SetNumberOfTableValues(tableSize);
				  lut->Build();

				  // Fill in a few known colors, the rest will be generated if needed
				  lut->SetTableValue(0, 1.0000 - 0.1 * stairSol, 0, 0, 0.5); // Tomato


				vtkSmartPointer<vtkPlaneSource> planeSource = vtkSmartPointer<vtkPlaneSource>::New();
				planeSource->SetOrigin(stepPos[0], stepPos[1], stepPos[2] );

				planeSource->SetPoint1(stepPos[0] - widthDir[0] * stairCoefficients.width, stepPos[1] - widthDir[1] * stairCoefficients.width, stepPos[2] );
				planeSource->SetPoint2(stepPos[0], stepPos[1], stepPos[2] - stairCoefficients.dir[2]);

				planeSource->Update();

				planeSource->GetOutput()->GetCellData()->SetScalars(cellData);

				vtkPolyData* plane = planeSource->GetOutput();


				// Create a mapper and actor
				vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();

				mapper->SetInputData(plane);
					mapper->SetScalarRange(0, 1);
					mapper->SetLookupTable(lut);

				vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
				actor->SetMapper(mapper);

				viewer->getRenderWindow()->GetRenderers()->GetFirstRenderer()->AddActor(actor);
				vtkPointVec.push(actor);
			}
			     // Add stair treads
			{
				  // Create cell data
				  vtkSmartPointer<vtkFloatArray> cellData = vtkSmartPointer<vtkFloatArray>::New();
				  for (int i = 0; i < 1; i++)
				  {
					  cellData->InsertNextValue(i + 1);
				  }

				  // Create a lookup table to map cell data to colors
				  vtkSmartPointer<vtkLookupTable> lut = vtkSmartPointer<vtkLookupTable>::New();
				  int tableSize = std::max(1, 1);
				  lut->SetNumberOfTableValues(tableSize);
				  lut->Build();

				  // Fill in a few known colors, the rest will be generated if needed
				  lut->SetTableValue(0, 0, 0, 1 - 0.1 * stairSol, 0.5); // Blueish

				vtkSmartPointer<vtkPlaneSource> planeSource = vtkSmartPointer<vtkPlaneSource>::New();
				planeSource->SetOrigin(stepPos[0], stepPos[1], stepPos[2] );

				planeSource->SetPoint1(stepPos[0] - widthDir[0] * stairCoefficients.width, stepPos[1] - widthDir[1] * stairCoefficients.width, stepPos[2] );
				planeSource->SetPoint2(stepPos[0] + coeffDir[0], stepPos[1] + coeffDir[1], stepPos[2] );

				planeSource->Update();

				planeSource->GetOutput()->GetCellData()->SetScalars(cellData);

				vtkPolyData* plane = planeSource->GetOutput();


				// Create a mapper and actor
				vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();

				mapper->SetInputData(plane);
					mapper->SetScalarRange(0, 1);
					mapper->SetLookupTable(lut);

				vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
				actor->SetMapper(mapper);

				viewer->getRenderWindow()->GetRenderers()->GetFirstRenderer()->AddActor(actor);
				vtkPointVec.push(actor);
			}

		}
	}
}


void PCLViewer::completeRun()
{
    double preAS = pcl::getTime();
    preanalysis();
    double preAE = pcl::getTime();
    std::cout<<"Preanalysis took: "<<preAE-preAS<<std::endl;
    int mode = ui->comboBox_recInputMode->currentIndex();

    regSeg.clear();
    double segS = pcl::getTime();
    if(mode == 0)
    {
        regionGrowingButtonPressed();
        psProc.setInputRegions(regSeg);
    }
    if(mode == 1)
    {
        voxelSacButtonPressed();
        psProc.setInputRegions(regSeg);
    }
    if(mode ==2)
    {
        splitMergeButtonPressed();
        psProc.setInputRegions(regSeg);
    }

    double segE = pcl::getTime();
    std::cout<<"Segmentation took: "<<segE-segS<<std::endl;

    double pfS = pcl::getTime();
    Eigen::Vector2f psWidth;
    psWidth[0] = ui ->doubleSpinBox_psScWidthMin->value();
    psWidth[1] = ui ->doubleSpinBox_psScWidthMax->value();

    Eigen::Vector2f psHeight;
    psHeight[0] = ui ->doubleSpinBox_psScHeightMin->value();
    psHeight[1] = ui ->doubleSpinBox_psScHeightMax->value();

    Eigen::Vector2f psDepth;
    psDepth[0] = ui ->doubleSpinBox_psScDepthMin->value();
    psDepth[1] = ui ->doubleSpinBox_psScDepthMax->value();


    psProc.setAngleMargin(ui ->doubleSpinBox_psScMarg->value());
    psProc.setWidthReq(psWidth);
    psProc.setRiserHeight(psHeight);
    psProc.setTreadDepth(psDepth);

    stairTreads.clear();
    stairRises.clear();
    psProc.filterSc(stairTreads, stairRises);

    double pfE = pcl::getTime();
    std::cout<<"Plane filter took: "<<pfE-pfS<<std::endl;

    std::cout<<"Stair Treads:"<<stairTreads.size()<<std::endl;
    std::cout<<"Stair Rises:"<<stairRises.size()<<std::endl;

    double refS = pcl::getTime();
    refine();
    double refE = pcl::getTime();
    std::cout<<"Refinement took: "<<refE-refS<<std::endl;
    std::cout<<"Total time  took: "<<refE-preAS<<std::endl<<std::endl;
}

void PCLViewer::psShow()
{
	psGo(1);
	PointCloudC planeFinderCloud;
	for(int regID = 0; regID < stairRises.size(); regID++)
	{
		for(int pointID = 0; pointID < stairRises.at(regID).segmentCloud.size(); pointID++)
		{
			PointTC colPoint;
			colPoint.x = stairRises.at(regID).segmentCloud.at(pointID).x;
			colPoint.y = stairRises.at(regID).segmentCloud.at(pointID).y;
			colPoint.z = stairRises.at(regID).segmentCloud.at(pointID).z;
			colPoint.r = 255;
			colPoint.g = 0;
			colPoint.b = 0;
			planeFinderCloud.push_back(colPoint);
		}
	}

	for(int regID = 0; regID < stairTreads.size(); regID++)
	{
		for(int pointID = 0; pointID < stairTreads.at(regID).segmentCloud.size(); pointID++)
		{
			PointTC colPoint;
			colPoint.x = stairTreads.at(regID).segmentCloud.at(pointID).x;
			colPoint.y = stairTreads.at(regID).segmentCloud.at(pointID).y;
			colPoint.z = stairTreads.at(regID).segmentCloud.at(pointID).z;
			colPoint.r = 0;
			colPoint.g = 0;
			colPoint.b = 255;
			planeFinderCloud.push_back(colPoint);
		}
	}
	filterNorMap = planeFinderCloud;
    filterRegMap = planeFinderCloud;
    showFilterRegMap();

	  unmarkButtons();
	  ui->pushButton_psShowBoth->setStyleSheet("background-color: lightblue");
}

void PCLViewer::unmarkButtons()
{
	  ui->pushButton_showOrig->setStyleSheet("background-color: none");
	  ui->pushButton_PREregMap->setStyleSheet("background-color: none");
	  ui->pushButton_PREnorMap->setStyleSheet("background-color: none");
	  ui->pushButton_rgRegMap->setStyleSheet("background-color: none");
	  ui->pushButton_rgNorMap->setStyleSheet("background-color: none");
	  ui->pushButton_psShowBoth->setStyleSheet("background-color: none");
	  ui->pushButton_drawAllSol->setStyleSheet("background-color: none");
}

void PCLViewer::psGo(int method)
{

	psProc.setInputRegions(regSeg);


    Eigen::Vector2f psWidth;
    psWidth[0] = ui ->doubleSpinBox_psScWidthMin->value();
    psWidth[1] = ui ->doubleSpinBox_psScWidthMax->value();

    Eigen::Vector2f psHeight;
    psHeight[0] = ui ->doubleSpinBox_psScHeightMin->value();
    psHeight[1] = ui ->doubleSpinBox_psScHeightMax->value();

    Eigen::Vector2f psDepth;
    psDepth[0] = ui ->doubleSpinBox_psScDepthMin->value();
    psDepth[1] = ui ->doubleSpinBox_psScDepthMax->value();


    psProc.setAngleMargin(ui ->doubleSpinBox_psScMarg->value());
    psProc.setWidthReq(psWidth);
    psProc.setRiserHeight(psHeight);
    psProc.setTreadDepth(psDepth);


    psProc.filterSc(stairTreads,stairRises);
    if(method == 0)
    {
        activeRegion=stairTreads;
        filterRegMap = stairTreads.getColoredCloud();
        filterNorMap = stairTreads.getNormalMap();
    }
    else
    {
        activeRegion=stairRises;
        filterRegMap = stairRises.getColoredCloud();
        filterNorMap = stairRises.getNormalMap();
    }
    showFilterRegMap();
}


void PCLViewer::showFilterRegMap()
{
	viewer->removeAllShapes();
	while(!vtkPointVec.empty())
	{
		vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
		actor = vtkPointVec.top();
		vtkPointVec.pop();
		viewer->getRenderWindow()->GetRenderers()->GetFirstRenderer()->RemoveActor(actor);
	}
    *activeCloud=filterRegMap;
    filterCloud();

	  unmarkButtons();
	  ui->pushButton_showOrig->setStyleSheet("background-color: lightblue");
}

void PCLViewer::rgAtNeigh()
{
    ui->checkBox_rgAtSeed->setChecked(not(ui->checkBox_rgAtNeigh->isChecked()));
}

void PCLViewer::rgAtSeed()
{
    ui->checkBox_rgAtNeigh->setChecked(not(ui->checkBox_rgAtSeed->isChecked()));
}

void PCLViewer::rgRdNeigh()
{
    ui->checkBox_rgRdSeed->setChecked(not(ui->checkBox_rgRdNeigh->isChecked()));
}

void PCLViewer::rgRdSeed()
{
    ui->checkBox_rgRdNeigh->setChecked(not(ui->checkBox_rgRdSeed->isChecked()));
}

void
PCLViewer::showPREregMap()
{
	viewer->removeAllShapes();
	while(!vtkPointVec.empty())
	{
		vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
		actor = vtkPointVec.top();
		vtkPointVec.pop();
		viewer->getRenderWindow()->GetRenderers()->GetFirstRenderer()->RemoveActor(actor);
	}
    makeColored(prepCloud);
    filterCloud();


	  unmarkButtons();
	  ui->pushButton_PREregMap->setStyleSheet("background-color: lightblue");
}

void
PCLViewer::showPREnorMap()
{
	viewer->removeAllShapes();
	while(!vtkPointVec.empty())
	{
		vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
		actor = vtkPointVec.top();
		vtkPointVec.pop();
		viewer->getRenderWindow()->GetRenderers()->GetFirstRenderer()->RemoveActor(actor);
	}
	activeCloud = prepNorMap.makeShared();
    filterCloud();

	  unmarkButtons();
	  ui->pushButton_PREnorMap->setStyleSheet("background-color: lightblue");
}

void
PCLViewer::showSegRegMap()
{
	viewer->removeAllShapes();
	while(!vtkPointVec.empty())
	{
		vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
		actor = vtkPointVec.top();
		vtkPointVec.pop();
		viewer->getRenderWindow()->GetRenderers()->GetFirstRenderer()->RemoveActor(actor);
	}
	activeCloud = rgRegMap.makeShared();
	filterCloud();

	  unmarkButtons();
	  ui->pushButton_rgRegMap->setStyleSheet("background-color: lightblue");
}

void
PCLViewer::showSegNorMap()
{
	viewer->removeAllShapes();
	while(!vtkPointVec.empty())
	{
		vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
		actor = vtkPointVec.top();
		vtkPointVec.pop();
		viewer->getRenderWindow()->GetRenderers()->GetFirstRenderer()->RemoveActor(actor);
	}
	activeCloud = rgNorMap.makeShared();
	filterCloud();

	  unmarkButtons();
	  ui->pushButton_rgNorMap->setStyleSheet("background-color: lightblue");
}


void
PCLViewer::regionGrowingButtonPressed()
{
//    std::cout<<"Starting Region Growing"<<std::endl;
    double rg_start = pcl::getTime();
    RegionGrowing reGrow;

    reGrow.setClusterSize(ui->spinBox_rgClustSize->value());
    reGrow.setNoNeigh(ui->spinBox_rgNoNeigh->value());
    reGrow.setSmoothFlag(ui->checkBox_rgAtSeed->isChecked());
    reGrow.setSmoothTresh(ui->doubleSpinBox_rgAtAngle->value());
    reGrow.setResFlag(ui->checkBox_rgRdSeed->isChecked());
    reGrow.setResTresh(ui->doubleSpinBox_rgRdDist->value());
    reGrow.setCurvFlag(ui->checkBox_rgCurv->isChecked());
    reGrow.setCurvThresh(ui->doubleSpinBox_rgCurvThresh->value());
    reGrow.setUpdateFlag(ui->checkBox_rgUpd->isChecked());
    reGrow.setPointUpdateFlag(ui->checkBox_rgPw->isChecked());
    reGrow.setUpdateInterval (ui->spinBox_rgUpdInterval->value());



    regSeg.clear();
    reGrow.setInputCloud(prepCloud);
    reGrow.setNormalCloud(prepNomalCloud);
    reGrow.run(regSeg);

    double rg_end = pcl::getTime();
    rgTime = rg_end-rg_start;
    rgRegMap = regSeg.getColoredCloud();
    rgNorMap = regSeg.getNormalMap();
}


void
PCLViewer::voxelSacButtonPressed()
{
//    std::cout<<"Starting Voxel SAC"<<std::endl;

    double vs_start = pcl::getTime();

    boost::shared_ptr <PointCloudT> tempCloud (new PointCloudT);
    boost::shared_ptr <NormalCloud> tempNormal (new NormalCloud);
    *tempCloud = *prepCloud;
    *tempNormal = *prepNomalCloud;

    regSeg.clear();

	voxSAC voxSAC;

	float horAngleLRF;
	horAngleLRF = 360;
	Eigen::Vector2f verAngleLRF;
	verAngleLRF[0] = 48;
	verAngleLRF[1] = 150;

	voxSAC.setCloudSize(measuredPoints);
	voxSAC.setHorizontalMeasurementAngle(horAngleLRF);
	voxSAC.setVerticalMeasurementAngle(verAngleLRF);

	voxSAC.setIterations(ui->spinBox_vsIter->value());
	voxSAC.setMinVoxSize(ui->doubleSpinBox_vsMinVoxSize->value());
	voxSAC.setDecreaseGrowFactor(ui->spinBox_vsV2Dec->value());
	voxSAC.setMinGrowSize(ui->doubleSpinBox_vsMinGrowSize->value());

	voxSAC.setPlaneInitMaxDist(ui->doubleSpinBox_vsInitPMaxD->value());
	voxSAC.setPlaneInitMaxAng(ui->doubleSpinBox_vsInitAngMax->value());
	voxSAC.setPlaneInitDensity(ui->doubleSpinBox_vsInitPDens->value());
	voxSAC.setAngularInitCompFlag(ui->checkBox_vsInitCompAngle->isChecked());
	voxSAC.setSampleDensity(downsampleRaster);

	voxSAC.setPlaneSacAngle(ui->doubleSpinBox_vsSacPlaneMaxAngle->value());
	voxSAC.setPlaneSacDist(ui->doubleSpinBox_vsSacPlaneMaxDist->value());
	voxSAC.setPlaneSacGrowFactor(ui->doubleSpinBox_vsSacPlaneGrowFactor->value());
	voxSAC.setPlaneSacCompFlag(ui->checkBox_vsSacPlaneCompAngle->isChecked());

	voxSAC.setPlaneSacUpdate(ui->checkBox_vsPlaneUpdate->isChecked());
	voxSAC.setPlaneSacPointUpdate(ui->checkBox_vsPlanePw->isChecked());
	voxSAC.setPlaneSacUpdateInterval(ui->spinBox_vsPlaneUpdateInterval->value());

	voxSAC.setMinPlaneSize(ui->spinBox_vsMinPlaneSize->value());

	voxSAC.setInputCloud(tempCloud);
	voxSAC.setNormalCloud(tempNormal);

	voxSAC.run(regSeg);

	regSeg.analyse();

    tempCloud.reset();
    tempNormal.reset();
    double vs_end = pcl::getTime();
    vsTime = vs_end-vs_start;

    rgRegMap = regSeg.getColoredCloud();
    rgNorMap = regSeg.getNormalMap();
}

void
PCLViewer::splitMergeButtonPressed()
{
    PointCloudT::Ptr tempCloud (new PointCloudT);
    NormalCloud::Ptr tempNormal (new NormalCloud);
    *tempCloud = *prepCloud;
    *tempNormal = *prepNomalCloud;

//    std::cout<<"Starting Octree Split and Merge Segmentation"<<std::endl;
    double sam_start = pcl::getTime();
    double minResolution =ui->doubleSpinBox_smMinVoxSize->value();
    splitMerge sam;
    sam.setMinResolution(minResolution);
    sam.setInputCloud(tempCloud);
    sam.setNormalCloud(tempNormal);
    sam.setIterationCount(ui->spinBox_smIter->value());
    sam.setInitDist(ui->doubleSpinBox_smInitMaxDist->value());
    sam.setMergeMaxAngle(ui->doubleSpinBox_smMergeMaxAngle->value());
    sam.setMergeMaxDist(ui->doubleSpinBox_smMergeMaxDist->value());
    sam.setUpdateFlag(ui->checkBox_smUpdate->isChecked());
    sam.setUpdateInterval(ui->spinBox_smUpdateInterval->value());
    sam.setMinInlier(ui->spinBox_smMinInliers->value());
    sam.setMergeFlag(ui->checkBox_smMerge->isChecked());
    sam.splitProcess();
    double sam_half = pcl::getTime();
    std::cout<<"Split process took: "<<sam_half - sam_start<<std::endl;
//    std::cout<<"Starting Merging Process"<<std::endl;
    sam.mergeProcess(regSeg);
    double sam_end = pcl::getTime();
    sam.getNeighTime(smNeighTime);
    smTime = sam_end-sam_start;

    rgRegMap = regSeg.getColoredCloud();
    rgNorMap = regSeg.getNormalMap();
}



void
PCLViewer::filterCloud()
{
    visibleCloud.reset (new PointCloudC);
    visibleFloor.reset (new PointCloudC);

	viewer->removeAllShapes();
	while(!vtkPointVec.empty())
	{
		vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
		actor = vtkPointVec.top();
		vtkPointVec.pop();
		viewer->getRenderWindow()->GetRenderers()->GetFirstRenderer()->RemoveActor(actor);
	}
	ui->qvtkWidget->update ();
	for(int pointCounter=0; pointCounter<activeCloud->size();pointCounter++)
	{
		visibleCloud->push_back(activeCloud->at(pointCounter));
	}


    if(floorVisible)
    {
        for(int pointCounter=0; pointCounter<floorPC.size();pointCounter++)
        {
			PointTC tempPoint;
			tempPoint.x = floorPC.at(pointCounter).x;
			tempPoint.y = floorPC.at(pointCounter).y;
			tempPoint.z = floorPC.at(pointCounter).z;
			tempPoint.r=0;
			tempPoint.g=0;
			tempPoint.b=0;
			visibleFloor->push_back(tempPoint);
        }
    }

    viewer->removePointCloud ("cloud");
//    std::cout<<"There are "<<activeCloud->size()<<" points in the cloud"<<std::endl;
    viewer->addPointCloud (visibleCloud, "cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud");
    viewer->removePointCloud ("floor");
    viewer->addPointCloud (visibleFloor, "floor");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 0.5, "floor");
    //std::cout<<prepCloud->size()<<std::endl;

    viewer->removePointCloud ("greyCloud");

    if(showBackground)
    {
    	backCloud.reset (new PointCloudT);
        for(int pointCounter=0; pointCounter<prepCloud->size();pointCounter++)
        {
			PointT tempPoint;
			tempPoint.x = prepCloud->at(pointCounter).x;
			tempPoint.y = prepCloud->at(pointCounter).y;
			tempPoint.z = prepCloud->at(pointCounter).z;
			backCloud->push_back(tempPoint);
        }
		viewer->removePointCloud ("greyCloud");
		viewer->addPointCloud (backCloud, "greyCloud");
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 0.5, "greyCloud");
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0,0,0, "greyCloud");
    }

    ui->qvtkWidget->update ();
}



void
PCLViewer::loadFileButtonPressed ()
{
	loadingFile = true;
	std::ifstream fileFile ("filePos.txt");
	std::string loadFileString;
	getline(fileFile,loadFileString);
	fileFile.close();
	QString filename = QFileDialog::getOpenFileName (this, tr ("Open cloud"), loadFileString.c_str(), tr ("Point cloud data (*.pcd *.ply)"));
	if (filename.isEmpty ())
	return;

	loadFileString = filename.toStdString();
	loadFile(loadFileString);
	size_t ending = loadFileString.find_last_of('/');
	loadFileString.erase(ending, loadFileString.size()-ending);
	std::ofstream newfileFile ("filePos.txt");
	newfileFile << loadFileString;
}


void PCLViewer::loadFile(std::string selecFile)
{
	  currFileName = selecFile;
	  size_t repPos = currFileName.find(".pcd");

	  PCL_INFO("File chosen: %s\n", selecFile.c_str ());


	  double loadStart = pcl::getTime();

	  int return_status;

	    return_status = pcl::io::loadPCDFile (selecFile, *mainCloud);

	  double loadEnd = pcl::getTime();
	  loadTime = loadEnd-loadStart;
	  std::cout<<"Loading took: "<<loadEnd-loadStart<<std::endl;

	  if (return_status != 0)
	  {
	    PCL_ERROR("Error reading point cloud %s\n", selecFile.c_str ());
	    return;
	  }

	  prepCloud.reset (new PointCloudT);
	  prepNomalCloud.reset(new NormalCloud);
	  showOrig();
	  measuredPoints = mainCloud->size();
	  std::cout<<"Cloud has "<<measuredPoints<<" Points."<<std::endl;


	  ui->pushButton_fullRun->setEnabled(true);
	  ui->pushButton_showOrig->setEnabled(true);

	  ui->pushButton_PREregMap->setEnabled(false);
	  ui->pushButton_PREnorMap->setEnabled(false);
	  ui->pushButton_rgRegMap->setEnabled(false);
	  ui->pushButton_rgNorMap->setEnabled(false);
	  ui->pushButton_psShowBoth->setEnabled(false);
	  ui->pushButton_drawAllSol->setEnabled(false);

	  unmarkButtons();
	  ui->pushButton_showOrig->setStyleSheet("background-color: lightblue");

	  //viewer->resetCamera ();
	  ui->qvtkWidget->update ();


	  refCreated =false;
	  loadingFile = false;
}

void
PCLViewer::showOrig()
{
	viewer->removeAllShapes();
	while(!vtkPointVec.empty())
	{
		vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
		actor = vtkPointVec.top();
		vtkPointVec.pop();
		viewer->getRenderWindow()->GetRenderers()->GetFirstRenderer()->RemoveActor(actor);
	}
	ui->qvtkWidget->update ();
    makeColored(mainCloud);
    filterCloud();
}


void
PCLViewer::preanalysis()
{
//    std::cout<<"Starting Pre Analysis"<<std::endl;
    Preanalysis pre;

    double preStart = pcl::getTime();

    pre.setDsFlag(ui->checkBox_dsActive->isChecked());
    pre.setDsMethod(false);
    pre.setDsResolution(ui->doubleSpinBox_dsResolution->value());
    downsampleRaster = ui->doubleSpinBox_dsResolution->value();
    int sm = 0;

    pre.setNeSearch(sm);
    pre.setSearchNeighbours(ui->spinBox_kSearch->value());
//    std::cout<<"Counter: "<<autoCounter2<<std::endl;
//    pre.setSearchNeighbours(autoCounter2 * 8);


    pre.setGpAngle(ui->doubleSpinBox_gfAngle->value());
    pre.setGpActive(ui->checkBox_gpActive->isChecked());

    pre.setPfAngle(ui->doubleSpinBox_pfAngle->value());
    pre.setPfActive(ui->checkBox_pfActive->isChecked());

    pre.setFsActive(ui->checkBox_floorSeperation->isChecked());
    pre.setFsAngle(ui->doubleSpinBox_fsAngle->value());
    pre.setFsRange(ui->doubleSpinBox_fsRange->value());

    pre.setNeMethod(0);

    prepNorMap.clear();

    *prepCloud = *mainCloud;

    pre.run(prepCloud, prepNomalCloud, prepNorMap, floorPC);

    regSeg.setFloorCloud(floorPC);

    double preEnd = pcl::getTime();

    pre.getNeTime(neTime);

    preTime = preEnd-preStart;
}

void
PCLViewer::makeColored(PointCloudT::Ptr tempCloud)
{
    activeCloud.reset (new PointCloudC);
    for(size_t tempIter=0; tempIter < tempCloud->size(); tempIter++)
    {
        PointTC colPoint;
        colPoint.x = tempCloud->at(tempIter).x;
        colPoint.y = tempCloud->at(tempIter).y;
        colPoint.z = tempCloud->at(tempIter).z;
        colPoint.r = 0;
        colPoint.g = 0;
        colPoint.b = 0;
        activeCloud->push_back(colPoint);
    }
}



void
PCLViewer::loadSettings(std::string fileName)
{

	std::ifstream readFile;
	try {
		readFile.open(fileName.c_str());
	}
	catch (std::ifstream::failure e) {
		std::cout << "Exception opening/reading file";
		return;
	}
	std::string linestream;
	std::string option;
	std::stringstream lines;

	int state =0;
	bool change = false;
	while(getline(readFile,linestream))
	{
		change = false;
		lines.str("");
		lines.clear();
		option="";
		option.clear();
		lines << linestream;
		lines >> option;
		if(option.find("DOUBLESPINBOXES")!=std::string::npos)
		{
			state =1;
			change=true;
		}
		if(option.find("SPINBOXES")!=std::string::npos && not(change))
		{
			state =2;
			change=true;
		}
		if(option.find("CHECKBOXES")!=std::string::npos)
		{
			state =3;
			change=true;
		}
		if(option.find("COMBOBOX")!=std::string::npos)
		{
			state =4;
			change=true;
		}

		if(state==1 && not(change))
		{
			QString optionName = QString::fromStdString(option);
			QList<QDoubleSpinBox *> allDSB = ui->centralwidget->findChildren<QDoubleSpinBox *>(optionName);
			double value;
			lines >> value;
			allDSB[0]->setValue(value);

		}
		if(state==2 && not(change))
		{
			QString optionName = QString::fromStdString(option);
			QList<QSpinBox *> allDSB = ui->centralwidget->findChildren<QSpinBox *>(optionName);
			int value;
			lines >> value;
			allDSB[0]->setValue(value);
		}
		if(state==3 && not(change))
		{
			QString optionName = QString::fromStdString(option);
			QList<QCheckBox *> allDSB = ui->centralwidget->findChildren<QCheckBox *>(optionName);
			bool value;
			lines >> value;
			allDSB[0]->setChecked(value);
		}
		if(state==4 && not(change))
		{
			QString optionName = QString::fromStdString(option);
			QList<QComboBox *> allDSB = ui->centralwidget->findChildren<QComboBox *>(optionName);
			int value;
			lines >> value;
			allDSB[0]->setCurrentIndex(value);
		}
	}
}

PCLViewer::~PCLViewer ()
{
	delete ui;
}
