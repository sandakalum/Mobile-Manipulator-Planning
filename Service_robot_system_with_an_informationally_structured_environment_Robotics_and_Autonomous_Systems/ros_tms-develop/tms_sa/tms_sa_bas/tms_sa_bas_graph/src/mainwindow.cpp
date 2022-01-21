//------------------------------------------------------------------------------
// @file   : main.cpp
// @brief  : implement mainwindow
// @author : Yoonseok Pyo, Masahide Tanaka
// @version: Ver0.4 (since 2012.06.05)
// @date   : 2012.11.26
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Includes
//------------------------------------------------------------------------------
#include <QtGui>
#include <QMessageBox>

#include <iostream>

#include "mainwindow.hpp"

//------------------------------------------------------------------------------
// Namespaces
//------------------------------------------------------------------------------
namespace bas_graph
{
using namespace Qt;

//------------------------------------------------------------------------------
class YaxisScaleDraw : public QwtScaleDraw
{
public:
  YaxisScaleDraw()
  {
    setTickLength(QwtScaleDiv::MajorTick, 0);
    setTickLength(QwtScaleDiv::MinorTick, 0);
    setTickLength(QwtScaleDiv::MediumTick, 6);

    setLabelAlignment(Qt::AlignLeft | Qt::AlignVCenter);

    // setSpacing( 15 );
  }

  virtual QwtText label(double value) const
  {
    QString qsYLabel[8] = {"lost", "walking", "standing", "near chair", "sitting on chair", "sitting on bed", "on bed",
                           ""};
    if (value >= 0 && value <= 7)
      return qsYLabel[(unsigned int)value];
    else
      return qsYLabel[7];
  }
};

//------------------------------------------------------------------------------
// Implementation [MainWindow]
//------------------------------------------------------------------------------
MainWindow::MainWindow(int argc, char **argv, QWidget *parent) : QMainWindow(parent), qNode(argc, argv)
{
  //--------------------------------------------------------------------------
  QIcon icon;
  icon.addFile(QString::fromUtf8(":/images/basg.ico"), QSize(), QIcon::Normal, QIcon::Off);
  setWindowIcon(icon);

  setWindowTitle("BAS Graph (v0.4.0)");

  //--------------------------------------------------------------------------
  readSettings();
  createActions();
  createToolBars();

  //--------------------------------------------------------------------------
  // ros init
  //--------------------------------------------------------------------------
  if (!qNode.init())
  {
    showNoMasterMessage();
  }

  QObject::connect(&qNode, SIGNAL(rosShutdown()), this, SLOT(close()));

  //--------------------------------------------------------------------------
  // plot init
  //--------------------------------------------------------------------------
  qwtAnalysePlot = new QwtPlot(this);
  qwtResultPlot = new QwtPlot(this);

  QRect qGeometry;

  QPoint qSetAnalysePlotGeometry(20, 30);
  qGeometry.setTopLeft(qSetAnalysePlotGeometry);
  qwtAnalysePlot->setGeometry(qGeometry);

  qwtAnalysePlot->setTitle("BAS Analyse Graph");
  qwtAnalysePlot->setCanvasBackground(Qt::white);
  qwtAnalysePlot->setAxisTitle(QwtPlot::yLeft, "behavior list");
  qwtAnalysePlot->setAxisTitle(QwtPlot::xBottom, "time [ms]");
  // qwtAnalysePlot->setAxisAutoScale(QwtPlot::xBottom);
  qwtAnalysePlot->setAxisScale(QwtPlot::yLeft, 0, 7);
  qwtAnalysePlot->setAxisScaleDraw(QwtPlot::yLeft, new YaxisScaleDraw());
  qwtAnalysePlot->insertLegend(new QwtLegend());

  QPoint qSetResultPlotGeometry(20, 450);
  qGeometry.setTopLeft(qSetResultPlotGeometry);
  qwtResultPlot->setGeometry(qGeometry);

  qwtResultPlot->setTitle("BAS Result Graph");
  qwtResultPlot->setCanvasBackground(Qt::white);
  qwtResultPlot->setAxisTitle(QwtPlot::yLeft, "behavior list");
  qwtResultPlot->setAxisTitle(QwtPlot::xBottom, "time [ms]");
  // qwtResultPlot->setAxisScale(QwtPlot::xBottom, 0.0, 200);
  qwtResultPlot->setAxisScale(QwtPlot::yLeft, 0, 7);
  qwtResultPlot->setAxisScaleDraw(QwtPlot::yLeft, new YaxisScaleDraw());
  qwtResultPlot->insertLegend(new QwtLegend());

  QwtPlotGrid *qwtAnalysePlotGrid = new QwtPlotGrid();
  qwtAnalysePlotGrid->attach(qwtAnalysePlot);

  QwtPlotGrid *qwtResultPlotGrid = new QwtPlotGrid();
  qwtResultPlotGrid->attach(qwtResultPlot);

  qwtAnalysePlot->resize(900, 400);
  qwtResultPlot->resize(900, 400);

  qwtCurveBehaviorWalking = new QwtPlotCurve();
  qwtCurveBehaviorWalking->setTitle("Walking");
  qwtCurveBehaviorWalking->setPen(QPen(Qt::blue, 4));
  qwtCurveBehaviorWalking->setRenderHint(QwtPlotItem::RenderAntialiased, true);
  qwtCurveBehaviorWalking->setSymbol(
      new QwtSymbol(QwtSymbol::Ellipse, QBrush(Qt::blue), QPen(Qt::blue, 1), QSize(1, 1)));

  qwtCurveBehaviorSitting = new QwtPlotCurve();
  qwtCurveBehaviorSitting->setTitle("Sitting");
  qwtCurveBehaviorSitting->setPen(QPen(Qt::red, 4));
  qwtCurveBehaviorSitting->setRenderHint(QwtPlotItem::RenderAntialiased, true);
  qwtCurveBehaviorSitting->setSymbol(new QwtSymbol(QwtSymbol::Ellipse, QBrush(Qt::red), QPen(Qt::red, 1), QSize(1, 1)));

  qwtCurveBehaviorSleeping = new QwtPlotCurve();
  qwtCurveBehaviorSleeping->setTitle("lying");
  qwtCurveBehaviorSleeping->setPen(QPen(Qt::darkGreen, 4));
  qwtCurveBehaviorSleeping->setRenderHint(QwtPlotItem::RenderAntialiased, true);
  qwtCurveBehaviorSleeping->setSymbol(
      new QwtSymbol(QwtSymbol::Ellipse, QBrush(Qt::darkGreen), QPen(Qt::darkGreen, 1), QSize(1, 1)));

  qwtCurveBehaviorMerged = new QwtPlotCurve();
  qwtCurveBehaviorMerged->setTitle("Result");
  qwtCurveBehaviorMerged->setPen(QPen(Qt::black, 4)),
      qwtCurveBehaviorMerged->setRenderHint(QwtPlotItem::RenderAntialiased, true);
  qwtCurveBehaviorMerged->setSymbol(
      new QwtSymbol(QwtSymbol::Ellipse, QBrush(Qt::black), QPen(Qt::black, 1), QSize(1, 1)));

  //--------------------------------------------------------------------------
  // LeftButton for the zooming
  // MidButton for the panning
  // RightButton: zoom out by 1
  // Ctrl+RighButton: zoom out to full size

  zoomer1 = new QwtPlotZoomer(qwtAnalysePlot->xBottom, qwtAnalysePlot->yLeft, qwtAnalysePlot->canvas());
  zoomer1->setRubberBandPen(QColor(Qt::gray));
  zoomer1->setTrackerPen(QColor(Qt::black));
  zoomer1->setMousePattern(QwtEventPattern::MouseSelect2, Qt::RightButton, Qt::ControlModifier);
  zoomer1->setMousePattern(QwtEventPattern::MouseSelect3, Qt::RightButton);

  panner1 = new QwtPlotPanner(qwtAnalysePlot->canvas());
  panner1->setMouseButton(Qt::MidButton);

  zoomer2 = new QwtPlotZoomer(qwtResultPlot->canvas());
  zoomer2->setRubberBandPen(QColor(Qt::gray));
  zoomer2->setTrackerPen(QColor(Qt::black));
  zoomer2->setMousePattern(QwtEventPattern::MouseSelect2, Qt::RightButton, Qt::ControlModifier);
  zoomer2->setMousePattern(QwtEventPattern::MouseSelect3, Qt::RightButton);

  panner2 = new QwtPlotPanner(qwtResultPlot->canvas());
  panner2->setMouseButton(Qt::MidButton);

  connect(zoomer1, SIGNAL(zoomed(const QRectF &)), this, SLOT(updateZoom2()));
  connect(zoomer2, SIGNAL(zoomed(const QRectF &)), this, SLOT(updateZoom1()));

  connect(panner1, SIGNAL(panned(int, int)), this, SLOT(updateMove2()));
  connect(panner2, SIGNAL(panned(int, int)), this, SLOT(updateMove1()));

  //--------------------------------------------------------------------------
  // QTtimer init
  //--------------------------------------------------------------------------
  QTimer *timer = new QTimer(this);
  connect(timer, SIGNAL(timeout()), this, SLOT(plotGraph()));
  timer->start(25);  // 25ms = 0.025sec
}

//------------------------------------------------------------------------------
MainWindow::~MainWindow()
{
}

//------------------------------------------------------------------------------
// Implementation [Slots]
//------------------------------------------------------------------------------
void MainWindow::plotGraph()
{
  qreal width = 0;

  QPolygonF qBehaviorWalkingData;
  QPolygonF qBehaviorSittingData;
  QPolygonF qBehaviorSleepingData;
  QPolygonF qBehaviorMergedData;

  for (unsigned int i = 0; i < qNode.m_bas_behavior_data.iBehaviorWalking.size(); i++)
  {
    qBehaviorWalkingData << QPointF(i + 1, qNode.m_bas_behavior_data.iBehaviorWalking[i]);
    qBehaviorSittingData << QPointF(i + 1, qNode.m_bas_behavior_data.iBehaviorSitting[i]);
    qBehaviorSleepingData << QPointF(i + 1, qNode.m_bas_behavior_data.iBehaviorSleeping[i]);
    qBehaviorMergedData << QPointF(i + 1, qNode.m_bas_behavior_data.iBehaviorMerged[i]);
  }

  width = qNode.m_bas_behavior_data.iBehaviorWalking.size();

  QRectF qrDataSize(0.0, 0.0, width, 7.0);
  zoomer1->setZoomBase(qrDataSize);
  zoomer2->setZoomBase(qrDataSize);

  qwtCurveBehaviorWalking->setSamples(qBehaviorWalkingData);
  qwtCurveBehaviorSitting->setSamples(qBehaviorSittingData);
  qwtCurveBehaviorSleeping->setSamples(qBehaviorSleepingData);
  qwtCurveBehaviorMerged->setSamples(qBehaviorMergedData);

  qwtCurveBehaviorWalking->attach(qwtAnalysePlot);
  qwtCurveBehaviorSitting->attach(qwtAnalysePlot);
  qwtCurveBehaviorSleeping->attach(qwtAnalysePlot);

  qwtCurveBehaviorMerged->attach(qwtResultPlot);

  qwtAnalysePlot->replot();
  qwtResultPlot->replot();

  // QRectF k = zoomer1->zoomBase();
  // printf("%f, %f, %f, %f, %f\n",k.x(), k.y(), k.width(), k.height(), width);
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
void MainWindow::createActions()
{
  captureAct = new QAction(QIcon(":/images/capture.png"), tr("capture"), this);
  connect(captureAct, SIGNAL(triggered()), this, SLOT(captureEvent()));

  analysePlotSaveAct = new QAction(QIcon(":/images/filesave.png"), tr("analysePlotSave"), this);
  analysePlotSaveAct->setShortcuts(QKeySequence::Save);
  analysePlotSaveAct->setStatusTip(tr("Save the document to disk"));
  connect(analysePlotSaveAct, SIGNAL(triggered()), this, SLOT(exportAnalysePlot()));

  resultPlotSaveAct = new QAction(QIcon(":/images/filesave.png"), tr("resultPlotSave"), this);
  resultPlotSaveAct->setStatusTip(tr("Save the document to disk"));
  connect(resultPlotSaveAct, SIGNAL(triggered()), this, SLOT(exportResultPlot()));
}

//------------------------------------------------------------------------------
void MainWindow::createToolBars()
{
  toolBar = addToolBar(tr("FileToolBar"));
  toolBar->addAction(captureAct);
  toolBar->addAction(analysePlotSaveAct);
  toolBar->addAction(resultPlotSaveAct);
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
void MainWindow::exportAnalysePlot()
{
  exportPlot(1);
}

//------------------------------------------------------------------------------
void MainWindow::exportResultPlot()
{
  exportPlot(2);
}

//------------------------------------------------------------------------------
void MainWindow::exportPlot(int iPlotID)
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
    if (iPlotID == 1)
      renderer.renderDocument(qwtAnalysePlot, fileName, QSizeF(300, 200), 85);
    else
      renderer.renderDocument(qwtResultPlot, fileName, QSizeF(300, 200), 85);
  }
}

//------------------------------------------------------------------------------
void MainWindow::readSettings()
{
  QSettings settings("Qt-ROS_Package", "bas_graph");
  QPoint pos = settings.value("pos", QPoint(200, 200)).toPoint();
  QSize size = settings.value("size", QSize(400, 400)).toSize();
  resize(size);
  move(pos);
}

//------------------------------------------------------------------------------
void MainWindow::writeSettings()
{
  QSettings settings("Qt-ROS_Package", "bas_graph");
  settings.setValue("pos", pos());
  settings.setValue("size", size());
}

//------------------------------------------------------------------------------
void MainWindow::closeEvent(QCloseEvent *event)
{
  writeSettings();
  event->accept();
}

//------------------------------------------------------------------------------
void MainWindow::resizeEvent(QResizeEvent *event)
{
  QRect qGeometry;
  QRect r = contentsRect();
  r.setHeight(r.height() / 2 - 40);
  r.setWidth(r.width() - 50);

  QPoint qSetAnalysePlotGeometry(20, 30);
  qGeometry.setTopLeft(qSetAnalysePlotGeometry);
  qwtAnalysePlot->setGeometry(qGeometry);

  QPoint qSetResultPlotGeometry(20, r.height() + 50);
  qGeometry.setTopLeft(qSetResultPlotGeometry);
  qwtResultPlot->setGeometry(qGeometry);

  qwtAnalysePlot->resize(r.width(), r.height());
  qwtResultPlot->resize(r.width(), r.height());
}

//------------------------------------------------------------------------------
void MainWindow::updateZoom1()
{
  zoomer1->setZoomStack(zoomer2->zoomStack(), zoomer2->zoomRectIndex());
}

//------------------------------------------------------------------------------
void MainWindow::updateZoom2()
{
  zoomer2->setZoomStack(zoomer1->zoomStack(), zoomer1->zoomRectIndex());
}

//------------------------------------------------------------------------------
void MainWindow::updateMove1()
{
  zoomer1->setZoomStack(zoomer2->zoomStack(), zoomer2->zoomRectIndex());
}

//------------------------------------------------------------------------------
void MainWindow::updateMove2()
{
  zoomer2->setZoomStack(zoomer1->zoomStack(), zoomer1->zoomRectIndex());
}

//------------------------------------------------------------------------------
}  // namespace bas_graph

//------------------------------------------------------------------------------
