#ifndef MAIN_HXX
#define MAIN_HXX
#define NUM_L 300
#include <QtWidgets>
#include <string.h>
#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QDateTimeAxis>
#include <QtCharts/QValueAxis>
using namespace QtCharts;
class Main : public QMainWindow
{
  private:
  QWidget central_widget;
  QGridLayout layout;
  QSpinBox ki_select;
  QSpinBox kp_select;
  QSpinBox kd_select;
  QSpinBox temp_coefficient;
  QPushButton run_sim_btn;
  QChartView temp_v_time_chart;
  QLineSeries temp_over_time_series;
  public:
    Main(QWidget* parent = nullptr) : 
      QMainWindow(parent),
      ki_select(),
      kp_select(),
      kd_select(),
      temp_coefficient(),
      run_sim_btn("Run Simulation"),
      temp_v_time_chart()
    {
      layout.addWidget(&temp_coefficient, 0,0);
      layout.addWidget(&run_sim_btn,1,0);
      layout.addWidget(&kp_select,0,1);
      layout.addWidget(&ki_select,1,1);
      layout.addWidget(&kd_select,2,1);
      layout.addWidget(&temp_v_time_chart,3,0,1,-1);
      central_widget.setLayout(&layout);
      this->setCentralWidget(&central_widget);
    }
};

#endif
