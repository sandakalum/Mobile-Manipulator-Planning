#ifndef MAIN_WINDOW_HPP
#define MAIN_WINDOW_HPP

//------------------------------------------------------------------------------
// Includes
//------------------------------------------------------------------------------
#include <QtGui/QMainWindow>
#include <QtGui/QCheckBox>
#include <qapplication.h>

#include <qtoolbar.h>
#include <qtoolbutton.h>

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

#include "qnode.hpp"

//------------------------------------------------------------------------------
// Namespaces
//------------------------------------------------------------------------------
namespace fss_graph
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
  void closeEvent(QCloseEvent *event);
  void showNoMasterMessage();

public Q_SLOTS:
  void plot_graph();
  void captureEvent();
  void exportPlot1();
  void exportPlot2();
  void exportPlot3();
  void updateZoom1();
  void updateZoom2();
  void updateZoom3();
  void changeOption();
  void refresh();

private:
  QNode qnode;
  QAction *captureAct;
  QCheckBox *ckbViewOnlyCluster;
  QCheckBox *ckbViewFluctuation;
  QLineEdit *leMaxIntensity;
  QAction *refreshAct;
  QwtPlot *plot1;
  QwtPlot *plot2;
  QwtPlot *plot3;
  QwtPlotCurve *curve1;
  QwtPlotCurve *curve2;
  QwtPlotCurve *curve3;
  QwtPlotIntervalCurve *intervalCurve1;
  QwtPlotIntervalCurve *intervalCurve2;
  QwtPlotIntervalCurve *intervalCurve3;

  QwtPlotZoomer *zoomer1;
  QwtPlotZoomer *zoomer2;
  QwtPlotZoomer *zoomer3;

  QwtPlotPanner *panner1;
  QwtPlotPanner *panner2;
  QwtPlotPanner *panner3;

  bool bViewFluctuation;
  bool bViewOnlyCluster;
};
//------------------------------------------------------------------------------
}  // namespace fss_graph
#endif  // MAIN_WINDOW_HPP
