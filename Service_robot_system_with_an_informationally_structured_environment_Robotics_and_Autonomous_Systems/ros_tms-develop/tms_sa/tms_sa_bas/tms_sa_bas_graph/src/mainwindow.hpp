#ifndef MAIN_WINDOW_HPP
#define MAIN_WINDOW_HPP

//------------------------------------------------------------------------------
// Includes
//------------------------------------------------------------------------------
#include <QtGui/QMainWindow>
#include <qapplication.h>

#include <qwt_plot.h>
#include <qwt_plot_marker.h>
#include <qwt_plot_curve.h>
#include <qwt_plot_grid.h>
#include <qwt_symbol.h>
#include <qwt_legend.h>

#include <qwt_plot_intervalcurve.h>
#include <qwt_interval_symbol.h>
#include <qwt_plot_zoomer.h>
#include <qwt_plot_panner.h>
#include <qwt_plot_renderer.h>
#include <qwt_scale_draw.h>

#include <qtoolbar.h>
#include <qtoolbutton.h>

#include "qnode.hpp"

//------------------------------------------------------------------------------
// Namespaces
//------------------------------------------------------------------------------
namespace bas_graph
{
//------------------------------------------------------------------------------
// Implementation [MainWindow]
//------------------------------------------------------------------------------
class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  MainWindow(int argc, char **argv, QWidget *parent = 0);
  ~MainWindow();
  void showNoMasterMessage();

public Q_SLOTS:
  void plotGraph();
  void captureEvent();
  void exportAnalysePlot();
  void exportResultPlot();
  void exportPlot(int iPlotID);
  void updateZoom1();
  void updateZoom2();
  void updateMove1();
  void updateMove2();

private:
  void createActions();
  void createToolBars();
  void readSettings();
  void writeSettings();

  void resizeEvent(QResizeEvent *event);
  void closeEvent(QCloseEvent *event);

  QToolBar *toolBar;
  QAction *captureAct;
  QAction *analysePlotSaveAct;
  QAction *resultPlotSaveAct;

  QNode qNode;

  QwtPlot *qwtAnalysePlot;
  QwtPlot *qwtResultPlot;

  QwtPlotCurve *qwtCurveBehaviorWalking;
  QwtPlotCurve *qwtCurveBehaviorSitting;
  QwtPlotCurve *qwtCurveBehaviorSleeping;
  QwtPlotCurve *qwtCurveBehaviorMerged;

  QwtPlotZoomer *zoomer1;
  QwtPlotZoomer *zoomer2;

  QwtPlotPanner *panner1;
  QwtPlotPanner *panner2;
};

//------------------------------------------------------------------------------
}  // namespace bas_graph
#endif  // MAIN_WINDOW_HPP

//------------------------------------------------------------------------------
