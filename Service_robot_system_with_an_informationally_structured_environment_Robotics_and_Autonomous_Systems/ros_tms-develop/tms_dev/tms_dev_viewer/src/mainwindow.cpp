//------------------------------------------------------------------------------
// @file   : mainwindow.cpp
// @brief  : set QT SIGNAL and QT SLOT and setting mainwindow
// @author : Yoonseok Pyo, Masahide Tanaka
// @version: Ver0.9.6 (since 2012.05.17)
// @date   : 2012.11.19
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Includes
//------------------------------------------------------------------------------
#include <QString>

#include "mainwindow.h"

//------------------------------------------------------------------------------
// Implementation
//------------------------------------------------------------------------------
MainWindow::MainWindow(QNode *node, QWidget *parent) : QMainWindow(parent), qnode(node)
{
  //--------------------------------------------------------------------------
  QIcon icon;
  icon.addFile(QString::fromUtf8(":/images/fssv.ico"), QSize(), QIcon::Normal, QIcon::Off);
  setWindowIcon(icon);

  setupUi(this);
  setWindowTitle("TMS VIEWER");
  readSettings();

  //--------------------------------------------------------------------------
  imageScaleCombo = new QComboBox;
  QStringList scales;
  scales << tr("50%") << tr("75%") << tr("100%") << tr("125%") << tr("150%");
  imageScaleCombo->addItems(scales);
  imageScaleCombo->setCurrentIndex(2);
  connect(imageScaleCombo, SIGNAL(currentIndexChanged(QString)), this, SLOT(zoomEvent(QString)));
  toolBarZoom->addWidget(imageScaleCombo);

  //--------------------------------------------------------------------------
  connect(this->actionNew, SIGNAL(triggered()), this, SLOT(close()));
  connect(this->actionCapture, SIGNAL(triggered()), this, SLOT(captureEvent()));
  connect(this->actionZoomIn, SIGNAL(triggered()), this, SLOT(zoomInEvent()));
  connect(this->actionZoomOut, SIGNAL(triggered()), this, SLOT(zoomOutEvent()));

  // Gird
  connect(this->radioGridSquare, SIGNAL(clicked()), this, SLOT(eventChangeOption()));
  connect(this->radioGridCircle, SIGNAL(clicked()), this, SLOT(eventChangeOption()));
  connect(this->radioGridOff, SIGNAL(clicked()), this, SLOT(eventChangeOption()));

  // Laser
  connect(this->ckbStatic, SIGNAL(clicked()), this, SLOT(eventChangeOption()));

  connect(this->ckbDirectAll, SIGNAL(clicked()), this, SLOT(eventChangeOption()));
  connect(this->ckbReflectAll, SIGNAL(clicked()), this, SLOT(eventChangeOption()));

  connect(this->ckbDirect, SIGNAL(clicked()), this, SLOT(eventChangeOption()));
  connect(this->ckbReflect, SIGNAL(clicked()), this, SLOT(eventChangeOption()));

  connect(this->radioLaserViewPoint, SIGNAL(clicked()), this, SLOT(eventChangeOption()));
  connect(this->radioLaserViewLine, SIGNAL(clicked()), this, SLOT(eventChangeOption()));
  connect(this->radioLaserViewOff, SIGNAL(clicked()), this, SLOT(eventChangeOption()));

  // Occlusion
  connect(this->radioOcclusionViewPoint, SIGNAL(clicked()), this, SLOT(eventChangeOption()));
  connect(this->radioOcclusionViewFace, SIGNAL(clicked()), this, SLOT(eventChangeOption()));
  connect(this->radioOcclusionViewOn, SIGNAL(clicked()), this, SLOT(eventChangeOption()));
  connect(this->radioOcclusionViewOff, SIGNAL(clicked()), this, SLOT(eventChangeOption()));

  // Cluster
  connect(this->radioClusterViewOn, SIGNAL(clicked()), this, SLOT(eventChangeOption()));
  connect(this->radioClusterViewOff, SIGNAL(clicked()), this, SLOT(eventChangeOption()));
  connect(this->ckbClusterTagView, SIGNAL(clicked()), this, SLOT(eventChangeOption()));

  // Class
  connect(this->radioClassViewOn, SIGNAL(clicked()), this, SLOT(eventChangeOption()));
  connect(this->radioClassViewOff, SIGNAL(clicked()), this, SLOT(eventChangeOption()));
  connect(this->ckbClassTagView, SIGNAL(clicked()), this, SLOT(eventChangeOption()));
  connect(this->ckbTwoColors, SIGNAL(clicked()), this, SLOT(eventChangeOption()));

  // UnknownClass
  connect(this->radioUnknownClassViewOn, SIGNAL(clicked()), this, SLOT(eventChangeOption()));
  connect(this->radioUnknownClassViewOff, SIGNAL(clicked()), this, SLOT(eventChangeOption()));
  connect(this->ckbUnknownClassTagView, SIGNAL(clicked()), this, SLOT(eventChangeOption()));
  connect(this->ckbUnknownTwoColors, SIGNAL(clicked()), this, SLOT(eventChangeOption()));

  // Furniture
  connect(this->radioFurnitureViewOn, SIGNAL(clicked()), this, SLOT(eventChangeOption()));
  connect(this->radioFurnitureViewOff, SIGNAL(clicked()), this, SLOT(eventChangeOption()));
  connect(this->ckbFurnitureTagView, SIGNAL(clicked()), this, SLOT(eventChangeOption()));

  // Path
  connect(this->ckbPlanPathView, SIGNAL(clicked()), this, SLOT(eventChangeOption()));
  connect(this->ckbRobotPathView, SIGNAL(clicked()), this, SLOT(eventChangeOption()));
  connect(this->ckbWagonPathView, SIGNAL(clicked()), this, SLOT(eventChangeOption()));
  connect(this->radioPathViewOn, SIGNAL(clicked()), this, SLOT(eventChangeOption()));
  connect(this->radioPathViewOff, SIGNAL(clicked()), this, SLOT(eventChangeOption()));

  // RPS_MAP
  connect(this->ckbCollisionAreaView, SIGNAL(clicked()), this, SLOT(eventChangeOption()));
  connect(this->ckbVoronoiLineView, SIGNAL(clicked()), this, SLOT(eventChangeOption()));
  connect(this->radioRPS_MAPViewOn, SIGNAL(clicked()), this, SLOT(eventChangeOption()));
  connect(this->radioRPS_MAPViewOff, SIGNAL(clicked()), this, SLOT(eventChangeOption()));

  // Smartpal
  connect(this->radioSmartpalViewOn, SIGNAL(clicked()), this, SLOT(eventChangeOption()));
  connect(this->radioSmartpalViewOff, SIGNAL(clicked()), this, SLOT(eventChangeOption()));
  connect(this->ckbSmartpalTagView, SIGNAL(clicked()), this, SLOT(eventChangeOption()));

  connect(this->ckbSmartpalTrajectory, SIGNAL(clicked()), this, SLOT(trajectoryOption()));

  // Roomba
  connect(this->radioRoombaViewOn, SIGNAL(clicked()), this, SLOT(eventChangeOption()));
  connect(this->radioRoombaViewOff, SIGNAL(clicked()), this, SLOT(eventChangeOption()));
  connect(this->ckbRoombaTagView, SIGNAL(clicked()), this, SLOT(eventChangeOption()));

  connect(this->ckbRoombaTrajectory, SIGNAL(clicked()), this, SLOT(trajectoryOption()));

  // Wagon
  connect(this->radioWagonViewOn, SIGNAL(clicked()), this, SLOT(eventChangeOption()));
  connect(this->radioWagonViewOff, SIGNAL(clicked()), this, SLOT(eventChangeOption()));
  connect(this->ckbWagonTagView, SIGNAL(clicked()), this, SLOT(eventChangeOption()));

  // Chair
  connect(this->radioChairViewOn, SIGNAL(clicked()), this, SLOT(eventChangeOption()));
  connect(this->radioChairViewOff, SIGNAL(clicked()), this, SLOT(eventChangeOption()));
  connect(this->ckbChairTagView, SIGNAL(clicked()), this, SLOT(eventChangeOption()));

  // PersonTrajectory
  connect(this->radioPersonTrajectoryViewOn, SIGNAL(clicked()), this, SLOT(eventChangeOption()));
  connect(this->radioPersonTrajectoryViewOff, SIGNAL(clicked()), this, SLOT(eventChangeOption()));
  connect(this->ckbPersonTrajectoryTagView, SIGNAL(clicked()), this, SLOT(eventChangeOption()));

  // Unknown Object
  connect(this->radioUnknownObjectViewOn, SIGNAL(clicked()), this, SLOT(eventChangeOption()));
  connect(this->radioUnknownObjectViewOff, SIGNAL(clicked()), this, SLOT(eventChangeOption()));
  connect(this->ckbUnknownObjectTagView, SIGNAL(clicked()), this, SLOT(eventChangeOption()));

  // Wagon (ISS)
  connect(this->radioWagonIssViewOn, SIGNAL(clicked()), this, SLOT(eventChangeOption()));
  connect(this->radioWagonIssViewOff, SIGNAL(clicked()), this, SLOT(eventChangeOption()));
  connect(this->ckbWagonIssTagView, SIGNAL(clicked()), this, SLOT(eventChangeOption()));

  // WheelChair
  connect(this->radioWheelChairViewOn, SIGNAL(clicked()), this, SLOT(eventChangeOption()));
  connect(this->radioWheelChairViewOff, SIGNAL(clicked()), this, SLOT(eventChangeOption()));
  connect(this->ckbWheelChairTagView, SIGNAL(clicked()), this, SLOT(eventChangeOption()));

  // Object (ICS)
  connect(this->radioObjectViewOn, SIGNAL(clicked()), this, SLOT(eventChangeOption()));
  connect(this->radioObjectViewOff, SIGNAL(clicked()), this, SLOT(eventChangeOption()));
  connect(this->ckbObjectTagView, SIGNAL(clicked()), this, SLOT(eventChangeOption()));

  // Intensity
  connect(this->radioIntensityViewOn, SIGNAL(clicked()), this, SLOT(eventChangeOption()));
  connect(this->radioIntensityViewOff, SIGNAL(clicked()), this, SLOT(eventChangeOption()));
  connect(this->ckbIntensityTagView, SIGNAL(clicked()), this, SLOT(eventChangeOption()));
  connect(this->pbtApply, SIGNAL(clicked()), this, SLOT(processIntensityViewValue()));

  // Data Table Tap
  connect(this->tableView, SIGNAL(clicked(const QModelIndex &)), this, SLOT(pressdIndex(QModelIndex)));

  //--------------------------------------------------------------------------
  viewer = new Viewer(node, parent);
  eventChangeOption();
  trajectoryOption();
  processIntensityViewValue();
  this->setCentralWidget(viewer);

  //--------------------------------------------------------------------------
  table_model = new QStandardItemModel(0, 5, this);
  this->tableView->setModel(table_model);
  table_selectionModel = new QItemSelectionModel(table_model);
  this->tableView->setSelectionModel(table_selectionModel);
  this->tableView->setSelectionBehavior(QAbstractItemView::SelectRows);

  this->table_model->setHeaderData(0, Qt::Horizontal, "Degree");
  this->table_model->setHeaderData(1, Qt::Horizontal, "Distance");
  this->table_model->setHeaderData(2, Qt::Horizontal, "Intensity");
  this->table_model->setHeaderData(3, Qt::Horizontal, "IntrinsicIntensity");
  this->table_model->setHeaderData(4, Qt::Horizontal, "AcuteAngle");

  for (int i = 0; i < viewer->m_dLrf_scan_max_count; i++)
  {
    this->table_model->insertRows(i, 1, QModelIndex());
    this->table_model->setData(table_model->index(i, 0, QModelIndex()), QString("%1").arg(0));
    this->table_model->setData(table_model->index(i, 1, QModelIndex()), QString("%1").arg(0));
    this->table_model->setData(table_model->index(i, 2, QModelIndex()), QString("%1").arg(0));
    this->table_model->setData(table_model->index(i, 3, QModelIndex()), QString("%1").arg(0));
    this->table_model->setData(table_model->index(i, 4, QModelIndex()), QString("%1").arg(0));
  }

  //--------------------------------------------------------------------------
  QTimer *timer;
  timer = new QTimer(this);
  connect(timer, SIGNAL(timeout()), this, SLOT(viewData()));
  timer->start(50);  // 50ms
}

//------------------------------------------------------------------------------
MainWindow::~MainWindow()
{
}

//------------------------------------------------------------------------------
void MainWindow::pressdIndex(const QModelIndex &index)
{
  viewer->m_iDataViewRowIndex = index.row();
}

//------------------------------------------------------------------------------
void MainWindow::zoomEvent(const QString &scale)
{
  double newScale = scale.left(scale.indexOf(tr("%"))).toDouble();
  viewer->m_viewer_scale = newScale / 666;
}

//------------------------------------------------------------------------------
void MainWindow::zoomInEvent()
{
  viewer->m_viewer_scale += 0.01;
}

//------------------------------------------------------------------------------
void MainWindow::zoomOutEvent()
{
  viewer->m_viewer_scale -= 0.01;
}

//------------------------------------------------------------------------------
void MainWindow::captureEvent()
{
  // capture screen
  QPixmap originalPixmap = QPixmap();

  originalPixmap = QPixmap::grabWindow(this->winId());

  QString format = "png";
  QString fileName = QDir::homePath() + tr("/") + QDate::currentDate().toString("yyyyMMdd") +
                     QTime::currentTime().toString("hhmmss") + tr(".") + format;

  if (!fileName.isEmpty())
    originalPixmap.save(fileName, format.toAscii());
}

//------------------------------------------------------------------------------
void MainWindow::viewData()
{
  for (int i = 0; i < viewer->m_dLrf_scan_max_count; i++)
  {
    this->table_model->setData(table_model->index(i, 0, QModelIndex()), QString::number(i * 0.25, 'f', 2));
    this->table_model->setData(table_model->index(i, 1, QModelIndex()),
                               QString::number(viewer->m_dUtm30LX_raw_data[i].fDistance, 'f', 0));
    this->table_model->setData(table_model->index(i, 2, QModelIndex()),
                               QString::number(viewer->m_dUtm30LX_raw_data[i].fIntensity, 'f', 0));
    this->table_model->setData(table_model->index(i, 3, QModelIndex()),
                               QString::number(viewer->m_dUtm30LX_raw_data[i].fIntrinsicIntensity, 'f', 0));
    this->table_model->setData(table_model->index(i, 4, QModelIndex()),
                               QString::number(viewer->m_dUtm30LX_raw_data[i].fAcuteAngle * RAD2DEG, 'f', 0));
  }
}

//------------------------------------------------------------------------------
void MainWindow::processIntensityViewValue()
{
  QString qsMin = leMinValue->text();
  QString qsMax = leMaxValue->text();

  viewer->m_iMinIntensityView = qsMin.toInt();
  viewer->m_iMaxIntensityView = qsMax.toInt();
}

//------------------------------------------------------------------------------
void MainWindow::eventChangeOption()
{
  //--------------------------------------------------------------------------
  // Grid
  if (this->radioGridSquare->isChecked())
  {
    viewer->m_iModeGrid = MODE_GRID_SQUARE;
  }

  if (this->radioGridCircle->isChecked())
  {
    viewer->m_iModeGrid = MODE_GRID_CIRCLE;
  }

  if (this->radioGridOff->isChecked())
  {
    viewer->m_iModeGrid = MODE_GRID_OFF;
  }

  //--------------------------------------------------------------------------
  // Laser
  if (this->ckbStatic->isChecked())
    viewer->m_bViewStaticPoint = true;
  else
    viewer->m_bViewStaticPoint = false;

  if (this->ckbDirectAll->isChecked())
    viewer->m_bViewDirectPointAll = true;
  else
    viewer->m_bViewDirectPointAll = false;

  if (this->ckbReflectAll->isChecked())
    viewer->m_bViewReflectPointAll = true;
  else
    viewer->m_bViewReflectPointAll = false;

  if (this->ckbDirect->isChecked())
    viewer->m_bViewDirectPoint = true;
  else
    viewer->m_bViewDirectPoint = false;

  if (this->ckbReflect->isChecked())
    viewer->m_bViewReflectPoint = true;
  else
    viewer->m_bViewReflectPoint = false;

  if (this->radioLaserViewPoint->isChecked())
  {
    viewer->m_iViewLaser = MODE_LASER_POINT;
  }

  if (this->radioLaserViewLine->isChecked())
  {
    viewer->m_iViewLaser = MODE_LASER_LINE;
  }

  if (this->radioLaserViewOff->isChecked())
  {
    viewer->m_iViewLaser = MODE_LASER_OFF;
  }

  //--------------------------------------------------------------------------
  // Occlusion
  if (this->radioOcclusionViewPoint->isChecked())
  {
    viewer->m_iViewOcclusion = MODE_OCCLUSION_POINT;
  }

  if (this->radioOcclusionViewFace->isChecked())
  {
    viewer->m_iViewOcclusion = MODE_OCCLUSION_FACE;
  }

  if (this->radioOcclusionViewOn->isChecked())
  {
    viewer->m_iViewOcclusion = MODE_OCCLUSION_ON;
  }

  if (this->radioOcclusionViewOff->isChecked())
  {
    viewer->m_iViewOcclusion = MODE_OCCLUSION_OFF;
  }

  //--------------------------------------------------------------------------
  // Cluster
  if (this->radioClusterViewOn->isChecked())
  {
    viewer->m_iViewCluster = MODE_CLUSTER_ON;
  }

  if (this->radioClusterViewOff->isChecked())
  {
    viewer->m_iViewCluster = MODE_CLUSTER_OFF;
  }

  if (this->ckbClusterTagView->isChecked())
    viewer->m_bTagViewCluster = true;
  else
    viewer->m_bTagViewCluster = false;

  //--------------------------------------------------------------------------
  // Class
  if (this->radioClassViewOn->isChecked())
  {
    viewer->m_iViewClass = MODE_CLASS_ON;
  }

  if (this->radioClassViewOff->isChecked())
  {
    viewer->m_iViewClass = MODE_CLASS_OFF;
  }

  if (this->ckbClassTagView->isChecked())
    viewer->m_bTagViewClass = true;
  else
    viewer->m_bTagViewClass = false;

  if (this->ckbTwoColors->isChecked())
  {
    viewer->m_bViewTwoColors = true;
    QString qsIntensityTheshold = leIntensityThreshold->text();
    viewer->m_iIntensityThreshold = qsIntensityTheshold.toInt();
  }
  else
    viewer->m_bViewTwoColors = false;

  //--------------------------------------------------------------------------
  // UnknownClass
  if (this->radioUnknownClassViewOn->isChecked())
  {
    viewer->m_iViewUnknownClass = MODE_UNKNOWN_CLASS_ON;
  }

  if (this->radioUnknownClassViewOff->isChecked())
  {
    viewer->m_iViewUnknownClass = MODE_UNKNOWN_CLASS_OFF;
  }

  if (this->ckbUnknownClassTagView->isChecked())
    viewer->m_bTagViewUnknownClass = true;
  else
    viewer->m_bTagViewUnknownClass = false;

  if (this->ckbUnknownTwoColors->isChecked())
  {
    viewer->m_bViewUnknownTwoColors = true;
    QString qsUnknownIntensityTheshold = leUnknownIntensityThreshold->text();
    viewer->m_iUnknownIntensityThreshold = qsUnknownIntensityTheshold.toInt();
  }
  else
    viewer->m_bViewUnknownTwoColors = false;

  //--------------------------------------------------------------------------
  // Furniture
  if (this->radioFurnitureViewOn->isChecked())
  {
    viewer->m_iViewFurniture = MODE_FURNITURE_ON;
  }

  if (this->radioFurnitureViewOff->isChecked())
  {
    viewer->m_iViewFurniture = MODE_FURNITURE_OFF;
  }

  if (this->ckbFurnitureTagView->isChecked())
    viewer->m_bTagViewFurniture = true;
  else
    viewer->m_bTagViewFurniture = false;

  //--------------------------------------------------------------------------
  // Path
  if (this->ckbPlanPathView->isChecked())
    viewer->m_bPlanPathView = true;
  else
    viewer->m_bPlanPathView = false;

  if (this->ckbRobotPathView->isChecked())
    viewer->m_bRobotPathView = true;
  else
    viewer->m_bRobotPathView = false;

  if (this->ckbWagonPathView->isChecked())
    viewer->m_bWagonPathView = true;
  else
    viewer->m_bWagonPathView = false;

  if (this->radioPathViewOn->isChecked())
  {
    viewer->m_iViewPath = MODE_PATH_ON;
  }

  if (this->radioPathViewOff->isChecked())
  {
    viewer->m_iViewPath = MODE_PATH_OFF;
  }

  //--------------------------------------------------------------------------
  // RPS_MAP
  if (this->ckbCollisionAreaView->isChecked())
    viewer->m_bCollisionAreaViewRPS_MAP = true;
  else
    viewer->m_bCollisionAreaViewRPS_MAP = false;

  if (this->ckbVoronoiLineView->isChecked())
    viewer->m_bvoronoiLineViewRPS_MAP = true;
  else
    viewer->m_bvoronoiLineViewRPS_MAP = false;

  if (this->radioRPS_MAPViewOn->isChecked())
  {
    viewer->m_iViewRPS_MAP = MODE_RPS_MAP_ON;
  }

  if (this->radioRPS_MAPViewOff->isChecked())
  {
    viewer->m_iViewRPS_MAP = MODE_RPS_MAP_OFF;
  }

  //--------------------------------------------------------------------------
  // Smartpal
  if (this->radioSmartpalViewOn->isChecked())
  {
    viewer->m_iViewSmartpal = MODE_SMARTPAL_ON;
  }

  if (this->radioSmartpalViewOff->isChecked())
  {
    viewer->m_iViewSmartpal = MODE_SMARTPAL_OFF;
  }

  if (this->ckbSmartpalTagView->isChecked())
    viewer->m_bTagViewSmartpal = true;
  else
    viewer->m_bTagViewSmartpal = false;

  //--------------------------------------------------------------------------
  // Roomba
  if (this->radioRoombaViewOn->isChecked())
  {
    viewer->m_iViewRoomba = MODE_ROOMBA_ON;
  }

  if (this->radioRoombaViewOff->isChecked())
  {
    viewer->m_iViewRoomba = MODE_ROOMBA_OFF;
  }

  if (this->ckbRoombaTagView->isChecked())
    viewer->m_bTagViewRoomba = true;
  else
    viewer->m_bTagViewRoomba = false;

  //--------------------------------------------------------------------------
  // Wagon
  if (this->radioWagonViewOn->isChecked())
  {
    viewer->m_iViewWagon = MODE_WAGON_ON;
  }

  if (this->radioWagonViewOff->isChecked())
  {
    viewer->m_iViewWagon = MODE_WAGON_OFF;
  }

  if (this->ckbWagonTagView->isChecked())
    viewer->m_bTagViewWagon = true;
  else
    viewer->m_bTagViewWagon = false;

  //--------------------------------------------------------------------------
  // Chair
  if (this->radioChairViewOn->isChecked())
  {
    viewer->m_iViewChair = MODE_CHAIR_ON;
  }

  if (this->radioChairViewOff->isChecked())
  {
    viewer->m_iViewChair = MODE_CHAIR_OFF;
  }

  if (this->ckbChairTagView->isChecked())
    viewer->m_bTagViewChair = true;
  else
    viewer->m_bTagViewChair = false;

  //--------------------------------------------------------------------------
  // PersonTrajectory
  if (this->radioPersonTrajectoryViewOn->isChecked())
  {
    viewer->m_iViewPersonTrajectory = MODE_PERSON_TRAJECTORY_ON;
  }

  if (this->radioPersonTrajectoryViewOff->isChecked())
  {
    viewer->m_iViewPersonTrajectory = MODE_PERSON_TRAJECTORY_OFF;
  }

  if (this->ckbPersonTrajectoryTagView->isChecked())
    viewer->m_bTagViewPersonTrajectory = true;
  else
    viewer->m_bTagViewPersonTrajectory = false;

  //--------------------------------------------------------------------------
  // Unknown Object
  if (this->radioUnknownObjectViewOn->isChecked())
  {
    viewer->m_iViewUnknownObject = MODE_UNKNOEN_OBJECT_ON;
  }

  if (this->radioUnknownObjectViewOff->isChecked())
  {
    viewer->m_iViewUnknownObject = MODE_UNKNOEN_OBJECT_OFF;
  }

  if (this->ckbUnknownObjectTagView->isChecked())
    viewer->m_bTagViewUnknownObject = true;
  else
    viewer->m_bTagViewUnknownObject = false;

  //--------------------------------------------------------------------------
  // Wagon ISS
  if (this->radioWagonIssViewOn->isChecked())
  {
    viewer->m_iViewWagonIss = MODE_WAGON_ISS_ON;
  }

  if (this->radioWagonIssViewOff->isChecked())
  {
    viewer->m_iViewWagonIss = MODE_WAGON_ISS_OFF;
  }

  if (this->ckbWagonIssTagView->isChecked())
    viewer->m_bTagViewWagonIss = true;
  else
    viewer->m_bTagViewWagonIss = false;

  //--------------------------------------------------------------------------
  // wheelChair
  if (this->radioWheelChairViewOn->isChecked())
  {
    viewer->m_iViewWheelChair = MODE_WHEELCHAIR_ON;
  }

  if (this->radioWheelChairViewOff->isChecked())
  {
    viewer->m_iViewWheelChair = MODE_WHEELCHAIR_OFF;
  }

  if (this->ckbWheelChairTagView->isChecked())
    viewer->m_bTagViewWheelChair = true;
  else
    viewer->m_bTagViewWheelChair = false;

  //--------------------------------------------------------------------------
  // Object ICS
  if (this->radioObjectViewOn->isChecked())
  {
    viewer->m_iViewObject = MODE_OBJECT_ON;
  }

  if (this->radioObjectViewOff->isChecked())
  {
    viewer->m_iViewObject = MODE_OBJECT_OFF;
  }

  if (this->ckbObjectTagView->isChecked())
    viewer->m_bTagViewObject = true;
  else
    viewer->m_bTagViewObject = false;

  //--------------------------------------------------------------------------
  // Intensity
  if (this->radioIntensityViewOn->isChecked())
  {
    viewer->m_iModeIntensity = MODE_INTENSITY_ON;
  }
  if (this->radioIntensityViewOff->isChecked())
  {
    viewer->m_iModeIntensity = MODE_INTENSITY_OFF;
  }

  if (this->ckbIntensityTagView->isChecked())
    viewer->m_bTagViewIntensity = true;
  else
    viewer->m_bTagViewIntensity = false;

  //--------------------------------------------------------------------------
}

//------------------------------------------------------------------------------
void MainWindow::trajectoryOption()
{
  //--------------------------------------------------------------------------
  // smartpal
  if (this->ckbSmartpalTrajectory->isChecked())
  {
    viewer->m_tSmartpalTrajectoryStartTime = ros::Time::now() + ros::Duration(9 * 60 * 60);  // GMT +9
    viewer->m_bSmartpalTrajectory = true;
  }
  else
    viewer->m_bSmartpalTrajectory = false;

  //--------------------------------------------------------------------------
  // roomba
  if (this->ckbRoombaTrajectory->isChecked())
  {
    viewer->m_tRoombaTrajectoryStartTime = ros::Time::now() + ros::Duration(9 * 60 * 60);  // GMT +9
    viewer->m_bRoombaTrajectory = true;
  }
  else
    viewer->m_bRoombaTrajectory = false;
}

//------------------------------------------------------------------------------
void MainWindow::readSettings()
{
  bool checked = false;
  QString text = "";

  QSettings settings("Qt-ROS_Package", "fss_viewer");

  //--------------------------------------------------------------------------
  // Window info
  restoreGeometry(settings.value("geometry").toByteArray());
  // restoreState(settings.value("windowState").toByteArray());

  //--------------------------------------------------------------------------
  // Grid
  checked = settings.value("radioGridSquare", false).toBool();
  radioGridSquare->setChecked(checked);
  checked = settings.value("radioGridCircle", false).toBool();
  radioGridCircle->setChecked(checked);
  checked = settings.value("radioGridOff", false).toBool();
  radioGridOff->setChecked(checked);

  //--------------------------------------------------------------------------
  // Laser
  checked = settings.value("ckbStatic", false).toBool();
  ckbStatic->setChecked(checked);
  checked = settings.value("ckbDirectAll", false).toBool();
  ckbDirectAll->setChecked(checked);
  checked = settings.value("ckbReflectAll", false).toBool();
  ckbReflectAll->setChecked(checked);
  checked = settings.value("ckbDirect", false).toBool();
  ckbDirect->setChecked(checked);
  checked = settings.value("ckbReflect", false).toBool();
  ckbReflect->setChecked(checked);
  checked = settings.value("radioLaserViewPoint", false).toBool();
  radioLaserViewPoint->setChecked(checked);
  checked = settings.value("radioLaserViewLine", false).toBool();
  radioLaserViewLine->setChecked(checked);
  checked = settings.value("radioLaserViewOff", false).toBool();
  radioLaserViewOff->setChecked(checked);

  //--------------------------------------------------------------------------
  // Occlusion
  checked = settings.value("radioOcclusionViewPoint", false).toBool();
  radioOcclusionViewPoint->setChecked(checked);
  checked = settings.value("radioOcclusionViewFace", false).toBool();
  radioOcclusionViewFace->setChecked(checked);
  checked = settings.value("radioOcclusionViewOn", false).toBool();
  radioOcclusionViewOn->setChecked(checked);
  checked = settings.value("radioOcclusionViewOff", false).toBool();
  radioOcclusionViewOff->setChecked(checked);

  //--------------------------------------------------------------------------
  // Cluster
  checked = settings.value("radioClusterViewOn", false).toBool();
  radioClusterViewOn->setChecked(checked);
  checked = settings.value("radioClusterViewOff", false).toBool();
  radioClusterViewOff->setChecked(checked);
  checked = settings.value("ckbClusterTagView", false).toBool();
  ckbClusterTagView->setChecked(checked);

  //--------------------------------------------------------------------------
  // Class
  checked = settings.value("radioClassViewOn", false).toBool();
  radioClassViewOn->setChecked(checked);
  checked = settings.value("radioClassViewOff", false).toBool();
  radioClassViewOff->setChecked(checked);
  checked = settings.value("ckbClassTagView", false).toBool();
  ckbClassTagView->setChecked(checked);
  checked = settings.value("ckbTwoColors", false).toBool();
  ckbTwoColors->setChecked(checked);
  text = settings.value("leIntensityThreshold", "0").toString();
  leIntensityThreshold->setText(text);

  //--------------------------------------------------------------------------
  // UnknownClass
  checked = settings.value("radioUnknownClassViewOn", false).toBool();
  radioUnknownClassViewOn->setChecked(checked);
  checked = settings.value("radioUnknownClassViewOff", false).toBool();
  radioUnknownClassViewOff->setChecked(checked);
  checked = settings.value("ckbUnknownClassTagView", false).toBool();
  ckbUnknownClassTagView->setChecked(checked);
  checked = settings.value("ckbUnknownTwoColors", false).toBool();
  ckbUnknownTwoColors->setChecked(checked);
  text = settings.value("leUnknownIntensityThreshold", "0").toString();
  leUnknownIntensityThreshold->setText(text);

  //--------------------------------------------------------------------------
  // Furniture
  checked = settings.value("radioFurnitureViewOn", false).toBool();
  radioFurnitureViewOn->setChecked(checked);
  checked = settings.value("radioFurnitureViewOff", false).toBool();
  radioFurnitureViewOff->setChecked(checked);
  checked = settings.value("ckbFurnitureTagView", false).toBool();
  ckbFurnitureTagView->setChecked(checked);

  //--------------------------------------------------------------------------
  // Path
  checked = settings.value("ckbPlanPathView", false).toBool();
  ckbPlanPathView->setChecked(checked);
  checked = settings.value("ckbRobotPathView", false).toBool();
  ckbRobotPathView->setChecked(checked);
  checked = settings.value("ckbWagonPathView", false).toBool();
  ckbWagonPathView->setChecked(checked);
  checked = settings.value("radioPathViewOn", false).toBool();
  radioPathViewOn->setChecked(checked);
  checked = settings.value("radioPathViewOff", false).toBool();
  radioPathViewOff->setChecked(checked);

  //--------------------------------------------------------------------------
  // RPS_MAP
  checked = settings.value("ckbCollisionAreaView", false).toBool();
  ckbCollisionAreaView->setChecked(checked);
  checked = settings.value("ckbvoronoiLineView", false).toBool();
  ckbVoronoiLineView->setChecked(checked);
  checked = settings.value("radioRPS_MAPViewOn", false).toBool();
  radioRPS_MAPViewOn->setChecked(checked);
  checked = settings.value("radioRPS_MAPViewOff", false).toBool();
  radioRPS_MAPViewOff->setChecked(checked);

  //--------------------------------------------------------------------------
  // Smartpal
  checked = settings.value("radioSmartpalViewOn", false).toBool();
  radioSmartpalViewOn->setChecked(checked);
  checked = settings.value("radioSmartpalViewOff", false).toBool();
  radioSmartpalViewOff->setChecked(checked);
  checked = settings.value("ckbSmartpalTrajectory", false).toBool();
  ckbSmartpalTrajectory->setChecked(checked);
  checked = settings.value("ckbSmartpalTagView", false).toBool();
  ckbSmartpalTagView->setChecked(checked);

  //--------------------------------------------------------------------------
  // Roomba
  checked = settings.value("radioRoombaViewOn", false).toBool();
  radioRoombaViewOn->setChecked(checked);
  checked = settings.value("radioRoombaViewOff", false).toBool();
  radioRoombaViewOff->setChecked(checked);
  checked = settings.value("ckbRoombaTrajectory", false).toBool();
  ckbRoombaTrajectory->setChecked(checked);
  checked = settings.value("ckbRoombaTagView", false).toBool();
  ckbRoombaTagView->setChecked(checked);

  //--------------------------------------------------------------------------
  // Wagon
  checked = settings.value("radioWagonViewOn", false).toBool();
  radioWagonViewOn->setChecked(checked);
  checked = settings.value("radioWagonViewOff", false).toBool();
  radioWagonViewOff->setChecked(checked);
  checked = settings.value("ckbWagonTagView", false).toBool();
  ckbWagonTagView->setChecked(checked);

  //--------------------------------------------------------------------------
  // Chair
  checked = settings.value("radioChairViewOn", false).toBool();
  radioChairViewOn->setChecked(checked);
  checked = settings.value("radioChairViewOff", false).toBool();
  radioChairViewOff->setChecked(checked);
  checked = settings.value("ckbChairTagView", false).toBool();
  ckbChairTagView->setChecked(checked);

  //--------------------------------------------------------------------------
  // PersonTrajectory
  checked = settings.value("radioPersonTrajectoryViewOn", false).toBool();
  radioPersonTrajectoryViewOn->setChecked(checked);
  checked = settings.value("radioPersonTrajectoryViewOff", false).toBool();
  radioPersonTrajectoryViewOff->setChecked(checked);
  checked = settings.value("ckbPersonTrajectoryTagView", false).toBool();
  ckbPersonTrajectoryTagView->setChecked(checked);

  //--------------------------------------------------------------------------
  // UnknownObject
  checked = settings.value("radioUnknownObjectViewOn", false).toBool();
  radioUnknownObjectViewOn->setChecked(checked);
  checked = settings.value("radioUnknownObjectViewOff", false).toBool();
  radioUnknownObjectViewOff->setChecked(checked);
  checked = settings.value("ckbUnknownObjectTagView", false).toBool();
  ckbUnknownObjectTagView->setChecked(checked);

  //--------------------------------------------------------------------------
  // Wagon ISS
  checked = settings.value("radioWagonIssViewOn", false).toBool();
  radioWagonIssViewOn->setChecked(checked);
  checked = settings.value("radioWagonIssViewOff", false).toBool();
  radioWagonIssViewOff->setChecked(checked);
  checked = settings.value("ckbWagonIssTagView", false).toBool();
  ckbWagonIssTagView->setChecked(checked);

  //--------------------------------------------------------------------------
  // WheelChair
  checked = settings.value("radioWheelChairViewOn", false).toBool();
  radioWheelChairViewOn->setChecked(checked);
  checked = settings.value("radioWheelChairViewOff", false).toBool();
  radioWheelChairViewOff->setChecked(checked);
  checked = settings.value("ckbWheelChairTagView", false).toBool();
  ckbWheelChairTagView->setChecked(checked);

  //--------------------------------------------------------------------------
  // Object ICS
  checked = settings.value("radioObjectViewOn", false).toBool();
  radioObjectViewOn->setChecked(checked);
  checked = settings.value("radioObjectViewOff", false).toBool();
  radioObjectViewOff->setChecked(checked);
  checked = settings.value("ckbObjectTagView", false).toBool();
  ckbObjectTagView->setChecked(checked);

  //--------------------------------------------------------------------------
  // Intensity
  checked = settings.value("radioIntensityViewOn", false).toBool();
  radioIntensityViewOn->setChecked(checked);
  checked = settings.value("radioIntensityViewOff", false).toBool();
  radioIntensityViewOff->setChecked(checked);
  checked = settings.value("ckbIntensityTagView", false).toBool();
  ckbIntensityTagView->setChecked(checked);
  text = settings.value("leMinValue", "0").toString();
  leMinValue->setText(text);
  text = settings.value("leMaxValue", "0").toString();
  leMaxValue->setText(text);
}

//------------------------------------------------------------------------------
void MainWindow::writeSettings()
{
  QSettings settings("Qt-ROS_Package", "fss_viewer");

  //--------------------------------------------------------------------------
  // Window info
  settings.setValue("geometry", saveGeometry());
  settings.setValue("windowState", saveState());

  //--------------------------------------------------------------------------
  // Grid
  settings.setValue("radioGridSquare", QVariant(radioGridSquare->isChecked()));
  settings.setValue("radioGridCircle", QVariant(radioGridCircle->isChecked()));
  settings.setValue("radioGridOff", QVariant(radioGridOff->isChecked()));

  //--------------------------------------------------------------------------
  // Laser
  settings.setValue("ckbStatic", QVariant(ckbStatic->isChecked()));
  settings.setValue("ckbDirectAll", QVariant(ckbDirectAll->isChecked()));
  settings.setValue("ckbReflectAll", QVariant(ckbReflectAll->isChecked()));
  settings.setValue("ckbDirect", QVariant(ckbDirect->isChecked()));
  settings.setValue("ckbReflect", QVariant(ckbReflect->isChecked()));
  settings.setValue("radioLaserViewPoint", QVariant(radioLaserViewPoint->isChecked()));
  settings.setValue("radioLaserViewLine", QVariant(radioLaserViewLine->isChecked()));
  settings.setValue("radioLaserViewOff", QVariant(radioLaserViewOff->isChecked()));

  //--------------------------------------------------------------------------
  // Occlusion
  settings.setValue("radioOcclusionViewPoint", QVariant(radioOcclusionViewPoint->isChecked()));
  settings.setValue("radioOcclusionViewFace", QVariant(radioOcclusionViewFace->isChecked()));
  settings.setValue("radioOcclusionViewOn", QVariant(radioOcclusionViewOn->isChecked()));
  settings.setValue("radioOcclusionViewOff", QVariant(radioOcclusionViewOff->isChecked()));

  //--------------------------------------------------------------------------
  // Cluster
  settings.setValue("radioClusterViewOn", QVariant(radioClusterViewOn->isChecked()));
  settings.setValue("radioClusterViewOff", QVariant(radioClusterViewOff->isChecked()));
  settings.setValue("ckbClusterTagView", QVariant(ckbClusterTagView->isChecked()));

  //--------------------------------------------------------------------------
  // Class
  settings.setValue("radioClassViewOn", QVariant(radioClassViewOn->isChecked()));
  settings.setValue("radioClassViewOff", QVariant(radioClassViewOff->isChecked()));
  settings.setValue("ckbClassTagView", QVariant(ckbClassTagView->isChecked()));
  settings.setValue("ckbTwoColors", QVariant(ckbTwoColors->isChecked()));
  settings.setValue("leIntensityThreshold", QVariant(leIntensityThreshold->text()));

  //--------------------------------------------------------------------------
  // UnknownClass
  settings.setValue("radioUnknownClassViewOn", QVariant(radioUnknownClassViewOn->isChecked()));
  settings.setValue("radioUnknownClassViewOff", QVariant(radioUnknownClassViewOff->isChecked()));
  settings.setValue("ckbUnknownClassTagView", QVariant(ckbUnknownClassTagView->isChecked()));
  settings.setValue("ckbUnknownTwoColors", QVariant(ckbUnknownTwoColors->isChecked()));
  settings.setValue("leUnknownIntensityThreshold", QVariant(leUnknownIntensityThreshold->text()));

  //--------------------------------------------------------------------------
  // Furniture
  settings.setValue("radioFurnitureViewOn", QVariant(radioFurnitureViewOn->isChecked()));
  settings.setValue("radioFurnitureViewOff", QVariant(radioFurnitureViewOff->isChecked()));
  settings.setValue("ckbFurnitureTagView", QVariant(ckbFurnitureTagView->isChecked()));

  //--------------------------------------------------------------------------
  // Path
  settings.setValue("radioPathViewOn", QVariant(radioPathViewOn->isChecked()));
  settings.setValue("radioPathViewOff", QVariant(radioPathViewOff->isChecked()));

  //--------------------------------------------------------------------------
  // Smartpal
  settings.setValue("radioSmartpalViewOn", QVariant(radioSmartpalViewOn->isChecked()));
  settings.setValue("radioSmartpalViewOff", QVariant(radioSmartpalViewOff->isChecked()));
  settings.setValue("ckbSmartpalTrajectory", QVariant(ckbSmartpalTrajectory->isChecked()));
  settings.setValue("ckbSmartpalTagView", QVariant(ckbSmartpalTagView->isChecked()));

  //--------------------------------------------------------------------------
  // Roomba
  settings.setValue("radioRoombaViewOn", QVariant(radioRoombaViewOn->isChecked()));
  settings.setValue("radioRoombaViewOff", QVariant(radioRoombaViewOff->isChecked()));
  settings.setValue("ckbRoombaTrajectory", QVariant(ckbRoombaTrajectory->isChecked()));
  settings.setValue("ckbRoombaTagView", QVariant(ckbRoombaTagView->isChecked()));

  //--------------------------------------------------------------------------
  // Wagon
  settings.setValue("radioWagonViewOn", QVariant(radioWagonViewOn->isChecked()));
  settings.setValue("radioWagonViewOff", QVariant(radioWagonViewOff->isChecked()));
  settings.setValue("ckbWagonTagView", QVariant(ckbWagonTagView->isChecked()));

  //--------------------------------------------------------------------------
  // Chair
  settings.setValue("radioChairViewOn", QVariant(radioChairViewOn->isChecked()));
  settings.setValue("radioChairViewOff", QVariant(radioChairViewOff->isChecked()));
  settings.setValue("ckbChairTagView", QVariant(ckbChairTagView->isChecked()));

  //--------------------------------------------------------------------------
  // PersonTrajectory
  settings.setValue("radioPersonTrajectoryViewOn", QVariant(radioPersonTrajectoryViewOn->isChecked()));
  settings.setValue("radioPersonTrajectoryViewOff", QVariant(radioPersonTrajectoryViewOff->isChecked()));
  settings.setValue("ckbPersonTrajectoryTagView", QVariant(ckbPersonTrajectoryTagView->isChecked()));

  //--------------------------------------------------------------------------
  // UnknownObject
  settings.setValue("radioUnknownObjectViewOn", QVariant(radioUnknownObjectViewOn->isChecked()));
  settings.setValue("radioUnknownObjectViewOff", QVariant(radioUnknownObjectViewOff->isChecked()));
  settings.setValue("ckbUnknownObjectTagView", QVariant(ckbUnknownObjectTagView->isChecked()));

  //--------------------------------------------------------------------------
  // Wagon ISS
  settings.setValue("radioWagonIssViewOn", QVariant(radioWagonIssViewOn->isChecked()));
  settings.setValue("radioWagonIssViewOff", QVariant(radioWagonIssViewOff->isChecked()));
  settings.setValue("ckbWagonIssTagView", QVariant(ckbWagonIssTagView->isChecked()));

  //--------------------------------------------------------------------------
  // WheelChair
  settings.setValue("radioWheelChairViewOn", QVariant(radioWheelChairViewOn->isChecked()));
  settings.setValue("radioWheelChairViewOff", QVariant(radioWheelChairViewOff->isChecked()));
  settings.setValue("ckbWheelChairTagView", QVariant(ckbWheelChairTagView->isChecked()));

  //--------------------------------------------------------------------------
  // Object ICS
  settings.setValue("radioObjectViewOn", QVariant(radioObjectViewOn->isChecked()));
  settings.setValue("radioObjectViewOff", QVariant(radioObjectViewOff->isChecked()));
  settings.setValue("ckbObjectTagView", QVariant(ckbObjectTagView->isChecked()));

  //--------------------------------------------------------------------------
  // Intensity
  settings.setValue("radioIntensityViewOn", QVariant(radioIntensityViewOn->isChecked()));
  settings.setValue("radioIntensityViewOff", QVariant(radioIntensityViewOff->isChecked()));
  settings.setValue("ckbIntensityTagView", QVariant(ckbIntensityTagView->isChecked()));
  settings.setValue("leMinValue", QVariant(leMinValue->text()));
  settings.setValue("leMaxValue", QVariant(leMaxValue->text()));
}

//------------------------------------------------------------------------------
void MainWindow::closeEvent(QCloseEvent *event)
{
  writeSettings();
  QMainWindow::closeEvent(event);

  if (ros::isStarted())
  {
    ros::shutdown();
    ros::waitForShutdown();
  }
  wait();
}

//------------------------------------------------------------------------------
