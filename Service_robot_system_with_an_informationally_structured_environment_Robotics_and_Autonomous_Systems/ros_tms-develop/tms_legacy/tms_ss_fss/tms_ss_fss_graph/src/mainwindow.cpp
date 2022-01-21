//------------------------------------------------------------------------------
// @file   : main.cpp
// @brief  : implement mainwindow
// @author : Yoonseok Pyo
// @version: Ver0.6 (since 2012.06.05)
// @date   : 2012.11.26
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Includes
//------------------------------------------------------------------------------
#include <QtGui>
#include <QMessageBox>

#include <iostream>

#include "../include/tms_ss_fss_graph/mainwindow.hpp"

//------------------------------------------------------------------------------
// Namespaces
//------------------------------------------------------------------------------
namespace fss_graph
{
using namespace Qt;

//------------------------------------------------------------------------------
// Implementation [MainWindow]
//------------------------------------------------------------------------------
MainWindow::MainWindow(int argc, char **argv, QWidget *parent) : QMainWindow(parent), qnode(argc, argv)
{
  QIcon icon;
  icon.addFile(QString::fromUtf8(":/images/fssg.ico"), QSize(), QIcon::Normal, QIcon::Off);
  setWindowIcon(icon);

  setWindowTitle("FSS Graph (v0.6.0)");

  //--------------------------------------------------------------------------
  // ros init
  //--------------------------------------------------------------------------
  if (!qnode.init())
  {
    showNoMasterMessage();
  }

  QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

  //--------------------------------------------------------------------------
  // plot 1 init
  //--------------------------------------------------------------------------
  QRect qGeometry;

  plot1 = new QwtPlot(this);

  QPoint qpSetPlot1Geometry(20, 40);
  qGeometry.setTopLeft(qpSetPlot1Geometry);
  plot1->setGeometry(qGeometry);

  plot1->setTitle("FSS Graph (Distance)");
  plot1->setCanvasBackground(Qt::white);
  plot1->setAxisTitle(QwtPlot::yLeft, "Distance [mm]");
  plot1->setAxisTitle(QwtPlot::xBottom, "Scanning Angle [degree]");
  plot1->setAxisScale(QwtPlot::yLeft, 0.0, 8000.0);
  plot1->setAxisScale(QwtPlot::xBottom, 0.0, 180);
  plot1->insertLegend(new QwtLegend());

  QwtPlotGrid *grid1 = new QwtPlotGrid();
  grid1->attach(plot1);

  curve1 = new QwtPlotCurve();
  curve1->setTitle("distance");
  curve1->setPen(QPen(Qt::blue, 1)), curve1->setRenderHint(QwtPlotItem::RenderAntialiased, true);

  curve1->setSymbol(new QwtSymbol(QwtSymbol::Ellipse, QBrush(Qt::yellow), QPen(Qt::red, 1), QSize(1, 1)));
  plot1->resize(900, 400);

  //--------------------------------------------------------------------------
  // plot 2 init
  //--------------------------------------------------------------------------
  plot2 = new QwtPlot(this);

  QPoint qpSetPlot2Geometry(20, 470);
  qGeometry.setTopLeft(qpSetPlot2Geometry);
  plot2->setGeometry(qGeometry);

  plot2->setTitle("FSS Graph (Intensity)");
  plot2->setCanvasBackground(Qt::white);
  plot2->setAxisTitle(QwtPlot::yLeft, "Intensity");
  plot2->setAxisTitle(QwtPlot::xBottom, "Scanning Angle [degree]");
  plot2->setAxisScale(QwtPlot::yLeft, 0.0, 18000.0);
  plot2->setAxisScale(QwtPlot::xBottom, 0.0, 180);
  plot2->insertLegend(new QwtLegend());

  QwtPlotGrid *grid2 = new QwtPlotGrid();
  grid2->attach(plot2);

  curve2 = new QwtPlotCurve();
  curve2->setTitle("intensity");
  curve2->setPen(QPen(Qt::darkGreen, 1)), curve2->setRenderHint(QwtPlotItem::RenderAntialiased, true);

  curve2->setSymbol(new QwtSymbol(QwtSymbol::Ellipse, QBrush(Qt::yellow), QPen(Qt::red, 1), QSize(1, 1)));
  plot2->resize(900, 400);

  //--------------------------------------------------------------------------
  // plot 3 init
  //--------------------------------------------------------------------------
  plot3 = new QwtPlot(this);

  QPoint qpSetPlot3Geometry(920, 470);
  qGeometry.setTopLeft(qpSetPlot3Geometry);
  plot3->setGeometry(qGeometry);

  plot3->setTitle("LRF Graph (Intrinsic Intensity)");
  plot3->setCanvasBackground(Qt::white);
  plot3->setAxisTitle(QwtPlot::yLeft, "Intrinsic Intensity");
  plot3->setAxisTitle(QwtPlot::xBottom, "Scanning Angle [degree]");
  plot3->setAxisScale(QwtPlot::yLeft, 0.0, 18000.0);
  plot3->setAxisScale(QwtPlot::xBottom, 0.0, 180);
  plot3->insertLegend(new QwtLegend());

  QwtPlotGrid *grid3 = new QwtPlotGrid();
  grid3->attach(plot3);

  curve3 = new QwtPlotCurve();
  curve3->setTitle("intensity");
  curve3->setPen(QPen(Qt::darkGreen, 1));
  curve3->setRenderHint(QwtPlotItem::RenderAntialiased, true);

  curve3->setSymbol(new QwtSymbol(QwtSymbol::Ellipse, QBrush(Qt::yellow), QPen(Qt::red, 1), QSize(1, 1)));

  plot3->resize(900, 400);

  //--------------------------------------------------------------------------
  QwtIntervalSymbol *errorBar = new QwtIntervalSymbol(QwtIntervalSymbol::Bar);
  errorBar->setWidth(4);
  errorBar->setPen(QPen(Qt::gray, 1));

  intervalCurve1 = new QwtPlotIntervalCurve("range");
  intervalCurve1->setStyle(QwtPlotIntervalCurve::NoCurve);
  intervalCurve1->setSymbol(errorBar);

  intervalCurve2 = new QwtPlotIntervalCurve("range");
  intervalCurve2->setStyle(QwtPlotIntervalCurve::NoCurve);
  intervalCurve2->setSymbol(errorBar);

  intervalCurve3 = new QwtPlotIntervalCurve("range");
  intervalCurve3->setStyle(QwtPlotIntervalCurve::NoCurve);
  intervalCurve3->setSymbol(errorBar);

  //--------------------------------------------------------------------------
  // LeftButton for the zooming
  // MidButton for the panning
  // RightButton: zoom out by 1
  // Ctrl+RighButton: zoom out to full size

  zoomer1 = new QwtPlotZoomer(plot1->canvas());
  zoomer1->setRubberBandPen(QColor(Qt::gray));
  zoomer1->setTrackerPen(QColor(Qt::black));
  zoomer1->setMousePattern(QwtEventPattern::MouseSelect2, Qt::RightButton, Qt::ControlModifier);
  zoomer1->setMousePattern(QwtEventPattern::MouseSelect3, Qt::RightButton);

  panner1 = new QwtPlotPanner(plot1->canvas());
  panner1->setMouseButton(Qt::MidButton);

  zoomer2 = new QwtPlotZoomer(plot2->canvas());
  zoomer2->setRubberBandPen(QColor(Qt::gray));
  zoomer2->setTrackerPen(QColor(Qt::black));
  zoomer2->setMousePattern(QwtEventPattern::MouseSelect2, Qt::RightButton, Qt::ControlModifier);
  zoomer2->setMousePattern(QwtEventPattern::MouseSelect3, Qt::RightButton);

  panner2 = new QwtPlotPanner(plot2->canvas());
  panner2->setMouseButton(Qt::MidButton);

  zoomer3 = new QwtPlotZoomer(plot3->canvas());
  zoomer3->setRubberBandPen(QColor(Qt::gray));
  zoomer3->setTrackerPen(QColor(Qt::black));
  zoomer3->setMousePattern(QwtEventPattern::MouseSelect2, Qt::RightButton, Qt::ControlModifier);
  zoomer3->setMousePattern(QwtEventPattern::MouseSelect3, Qt::RightButton);

  panner3 = new QwtPlotPanner(plot3->canvas());
  panner3->setMouseButton(Qt::MidButton);

  connect(zoomer1, SIGNAL(zoomed(const QRectF &)), this, SLOT(updateZoom1()));
  connect(zoomer2, SIGNAL(zoomed(const QRectF &)), this, SLOT(updateZoom2()));
  connect(zoomer3, SIGNAL(zoomed(const QRectF &)), this, SLOT(updateZoom3()));

  //--------------------------------------------------------------------------
  // toolbar
  //--------------------------------------------------------------------------
  QToolBar *toolBar = new QToolBar(this);

  captureAct = new QAction(QIcon(":/images/capture.png"), tr("capture"), this);
  connect(captureAct, SIGNAL(triggered()), this, SLOT(captureEvent()));

  QToolButton *btnExport1 = new QToolButton(toolBar);
  btnExport1->setText("Export1");
  btnExport1->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
  connect(btnExport1, SIGNAL(clicked()), this, SLOT(exportPlot1()));

  QToolButton *btnExport2 = new QToolButton(toolBar);
  btnExport2->setText("Export2");
  btnExport2->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
  connect(btnExport2, SIGNAL(clicked()), this, SLOT(exportPlot2()));

  QToolButton *btnExport3 = new QToolButton(toolBar);
  btnExport3->setText("Export3");
  btnExport3->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
  connect(btnExport3, SIGNAL(clicked()), this, SLOT(exportPlot3()));

  ckbViewFluctuation = new QCheckBox();
  ckbViewFluctuation->setObjectName(QString::fromUtf8("ckbViewFluctuation"));
  ckbViewFluctuation->setText("ViewFluctuation");
  ckbViewFluctuation->setChecked(false);
  connect(ckbViewFluctuation, SIGNAL(clicked()), this, SLOT(changeOption()));

  ckbViewOnlyCluster = new QCheckBox();
  ckbViewOnlyCluster->setObjectName(QString::fromUtf8("ckbViewOnlyCluster"));
  ckbViewOnlyCluster->setText("ViewOnlyCluster");
  ckbViewOnlyCluster->setChecked(false);
  connect(ckbViewOnlyCluster, SIGNAL(clicked()), this, SLOT(changeOption()));

  leMaxIntensity = new QLineEdit();
  leMaxIntensity->setText("18000");
  leMaxIntensity->setFixedWidth(70);
  leMaxIntensity->setCursor(QCursor(Qt::IBeamCursor));

  refreshAct = new QAction(QIcon(":/images/refresh.png"), tr("refresh"), this);
  connect(refreshAct, SIGNAL(triggered()), this, SLOT(refresh()));

  toolBar->addAction(captureAct);

  toolBar->addWidget(btnExport1);
  toolBar->addWidget(btnExport2);
  toolBar->addWidget(btnExport3);

  toolBar->addWidget(ckbViewOnlyCluster);
  toolBar->addWidget(ckbViewFluctuation);

  toolBar->addWidget(leMaxIntensity);
  toolBar->addAction(refreshAct);

  addToolBar(toolBar);

  changeOption();

  //--------------------------------------------------------------------------
  // QTtimer init
  //--------------------------------------------------------------------------
  QTimer *timer = new QTimer(this);
  connect(timer, SIGNAL(timeout()), this, SLOT(plot_graph()));
  timer->start(25);  // 25ms = 0.025sec
}

//------------------------------------------------------------------------------
MainWindow::~MainWindow()
{
}

//------------------------------------------------------------------------------
// Implementation [Slots]
//------------------------------------------------------------------------------
void MainWindow::plot_graph()
{
  QPolygonF points1;
  QPolygonF points2;
  QPolygonF points3;

  QVector< QwtIntervalSample > vDistanceRangeData(721);
  QVector< QwtIntervalSample > vIntensityRangeData(721);
  QVector< QwtIntervalSample > vIntrinsicIntensityRangeData(721);

  for (int i = 0; i < 721; i++)
  {
    if (bViewOnlyCluster == true)
    {
      if (qnode.m_stTmepData[i].bIsForwardPoint == true)
      {
        vDistanceRangeData[i] = QwtIntervalSample(
            double(i) * 0.25, QwtInterval(qnode.m_stMinErrorData[i].fDistance, qnode.m_stMaxErrorData[i].fDistance));

        vIntensityRangeData[i] = QwtIntervalSample(
            double(i) * 0.25, QwtInterval(qnode.m_stMinErrorData[i].fIntensity, qnode.m_stMaxErrorData[i].fIntensity));

        vIntrinsicIntensityRangeData[i] =
            QwtIntervalSample(double(i) * 0.25, QwtInterval(qnode.m_stMinErrorData[i].fIntrinsicIntensity,
                                                            qnode.m_stMaxErrorData[i].fIntrinsicIntensity));

        points1 << QPointF(i * 0.25, qnode.m_stTmepData[i].fDistance);
        points2 << QPointF(i * 0.25, qnode.m_stTmepData[i].fIntensity);
        points3 << QPointF(i * 0.25, qnode.m_stTmepData[i].fIntrinsicIntensity);
      }
      else
      {
        vDistanceRangeData[i] = QwtIntervalSample(double(i) * 0.25, QwtInterval(0, 0));
        vIntensityRangeData[i] = QwtIntervalSample(double(i) * 0.25, QwtInterval(0, 0));
        vIntrinsicIntensityRangeData[i] = QwtIntervalSample(double(i) * 0.25, QwtInterval(0, 0));

        points1 << QPointF(i * 0.25, 0.0);
        points2 << QPointF(i * 0.25, 0.0);
        points3 << QPointF(i * 0.25, 0.0);
      }
    }
    else
    {
      vDistanceRangeData[i] = QwtIntervalSample(
          double(i) * 0.25, QwtInterval(qnode.m_stMinErrorData[i].fDistance, qnode.m_stMaxErrorData[i].fDistance));

      vIntensityRangeData[i] = QwtIntervalSample(
          double(i) * 0.25, QwtInterval(qnode.m_stMinErrorData[i].fIntensity, qnode.m_stMaxErrorData[i].fIntensity));

      vIntrinsicIntensityRangeData[i] =
          QwtIntervalSample(double(i) * 0.25, QwtInterval(qnode.m_stMinErrorData[i].fIntrinsicIntensity,
                                                          qnode.m_stMaxErrorData[i].fIntrinsicIntensity));

      points1 << QPointF(i * 0.25, qnode.m_stTmepData[i].fDistance);
      points2 << QPointF(i * 0.25, qnode.m_stTmepData[i].fIntensity);
      points3 << QPointF(i * 0.25, qnode.m_stTmepData[i].fIntrinsicIntensity);
    }
  }

  if (bViewFluctuation == true)
  {
    intervalCurve1->setSamples(vDistanceRangeData);
    intervalCurve1->attach(plot1);
  }
  curve1->setSamples(points1);
  curve1->attach(plot1);
  plot1->replot();

  if (bViewFluctuation == true)
  {
    intervalCurve2->setSamples(vIntensityRangeData);
    intervalCurve2->attach(plot2);
  }
  curve2->setSamples(points2);
  curve2->attach(plot2);
  plot2->replot();

  if (bViewFluctuation == true)
  {
    intervalCurve3->setSamples(vIntrinsicIntensityRangeData);
    intervalCurve3->attach(plot3);
  }
  curve3->setSamples(points3);
  curve3->attach(plot3);
  plot3->replot();
}

//------------------------------------------------------------------------------
void MainWindow::showNoMasterMessage()
{
  QMessageBox msgBox;
  msgBox.setText("Couldn't find the ros master.");
  msgBox.exec();
  close();
}

//------------------------------------------------------------------------------
void MainWindow::closeEvent(QCloseEvent *event)
{
  QMainWindow::closeEvent(event);
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
void MainWindow::exportPlot1()
{
  QString format = "png";
  QString fileName = QDir::homePath() + tr("/") + QDate::currentDate().toString("yyyyMMdd") +
                     QTime::currentTime().toString("hhmmss") + tr(".") + format;

#ifndef QT_NO_FILEDIALOG

  const QList< QByteArray > imageFormats = QImageWriter::supportedImageFormats();

  QStringList filter;
  filter += "PDF Documents (*.pdf)";
  filter += "Postscript Documents (*.ps)";

  if (imageFormats.size() > 0)
  {
    QString imageFilter("Images (");
    for (int i = 0; i < imageFormats.size(); i++)
    {
      if (i > 0)
        imageFilter += " ";

      imageFilter += "*.";
      imageFilter += imageFormats[i];
    }
    imageFilter += ")";

    filter += imageFilter;
  }

  fileName = QFileDialog::getSaveFileName(this, "Export File Name", fileName, filter.join(";;"), NULL,
                                          QFileDialog::DontConfirmOverwrite);
#endif

  if (!fileName.isEmpty())
  {
    QwtPlotRenderer renderer;
    renderer.setDiscardFlag(QwtPlotRenderer::DiscardBackground, false);
    renderer.renderDocument(plot1, fileName, QSizeF(300, 200), 85);
  }
}

//------------------------------------------------------------------------------
void MainWindow::exportPlot2()
{
  QString format = "png";
  QString fileName = QDir::homePath() + tr("/") + QDate::currentDate().toString("yyyyMMdd") +
                     QTime::currentTime().toString("hhmmss") + tr(".") + format;

#ifndef QT_NO_FILEDIALOG

  const QList< QByteArray > imageFormats = QImageWriter::supportedImageFormats();

  QStringList filter;
  filter += "PDF Documents (*.pdf)";
  filter += "Postscript Documents (*.ps)";

  if (imageFormats.size() > 0)
  {
    QString imageFilter("Images (");
    for (int i = 0; i < imageFormats.size(); i++)
    {
      if (i > 0)
        imageFilter += " ";

      imageFilter += "*.";
      imageFilter += imageFormats[i];
    }
    imageFilter += ")";

    filter += imageFilter;
  }

  fileName = QFileDialog::getSaveFileName(this, "Export File Name", fileName, filter.join(";;"), NULL,
                                          QFileDialog::DontConfirmOverwrite);
#endif

  if (!fileName.isEmpty())
  {
    QwtPlotRenderer renderer;
    renderer.setDiscardFlag(QwtPlotRenderer::DiscardBackground, false);
    renderer.renderDocument(plot2, fileName, QSizeF(300, 200), 85);
  }
}

//------------------------------------------------------------------------------
void MainWindow::exportPlot3()
{
  QString format = "png";
  QString fileName = QDir::homePath() + tr("/") + QDate::currentDate().toString("yyyyMMdd") +
                     QTime::currentTime().toString("hhmmss") + tr(".") + format;

#ifndef QT_NO_FILEDIALOG

  const QList< QByteArray > imageFormats = QImageWriter::supportedImageFormats();

  QStringList filter;
  filter += "PDF Documents (*.pdf)";
  filter += "Postscript Documents (*.ps)";

  if (imageFormats.size() > 0)
  {
    QString imageFilter("Images (");
    for (int i = 0; i < imageFormats.size(); i++)
    {
      if (i > 0)
        imageFilter += " ";

      imageFilter += "*.";
      imageFilter += imageFormats[i];
    }
    imageFilter += ")";

    filter += imageFilter;
  }

  fileName = QFileDialog::getSaveFileName(this, "Export File Name", fileName, filter.join(";;"), NULL,
                                          QFileDialog::DontConfirmOverwrite);
#endif

  if (!fileName.isEmpty())
  {
    QwtPlotRenderer renderer;
    renderer.setDiscardFlag(QwtPlotRenderer::DiscardBackground, false);
    renderer.renderDocument(plot3, fileName, QSizeF(300, 200), 85);
  }
}

//------------------------------------------------------------------------------
void MainWindow::updateZoom1()
{
  zoomer2->setZoomStack(zoomer1->zoomStack(), zoomer1->zoomRectIndex());
  zoomer3->setZoomStack(zoomer1->zoomStack(), zoomer1->zoomRectIndex());
}

//------------------------------------------------------------------------------
void MainWindow::updateZoom2()
{
  zoomer1->setZoomStack(zoomer2->zoomStack(), zoomer2->zoomRectIndex());
  zoomer3->setZoomStack(zoomer2->zoomStack(), zoomer2->zoomRectIndex());
}

//------------------------------------------------------------------------------
void MainWindow::updateZoom3()
{
  zoomer1->setZoomStack(zoomer3->zoomStack(), zoomer3->zoomRectIndex());
  zoomer2->setZoomStack(zoomer3->zoomStack(), zoomer3->zoomRectIndex());
}
/*
//------------------------------------------------------------------------------
void MainWindow::updateMove1()
{
    zoomer1->setZoomStack(zoomer2->zoomStack(),zoomer2->zoomRectIndex());
}

//------------------------------------------------------------------------------
void MainWindow::updateMove2()
{
    zoomer2->setZoomStack(zoomer1->zoomStack(),zoomer1->zoomRectIndex());
}
*/

//------------------------------------------------------------------------------
void MainWindow::changeOption()
{
  if (ckbViewOnlyCluster->isChecked())
    bViewOnlyCluster = true;
  else
    bViewOnlyCluster = false;

  if (ckbViewFluctuation->isChecked())
    bViewFluctuation = true;
  else
    bViewFluctuation = false;
}

//------------------------------------------------------------------------------
void MainWindow::refresh()
{
  plot2->setAxisScale(QwtPlot::yLeft, 0.0, leMaxIntensity->text().toFloat());
  plot3->setAxisScale(QwtPlot::yLeft, 0.0, leMaxIntensity->text().toFloat());
}
//------------------------------------------------------------------------------
}  // namespace fss_graph

//------------------------------------------------------------------------------
