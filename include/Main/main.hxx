#ifndef MAIN_HXX
#define MAIN_HXX
#define NUM_L 300
#include <QtWidgets>
#include <string.h>
#include <QtCharts/QChartView>
#include <QtCharts/QSplineSeries>
#include <QtCharts/QDateTimeAxis>
#include <QtCharts/QValueAxis>
using namespace QtCharts;

/** @brief Inputbox container */
struct InputBox : public QGroupBox {
  QVBoxLayout layout;
  QSpinBox select;
  InputBox(std::string title, QWidget* parent = nullptr) : QGroupBox(QString::fromStdString(title),parent), select(parent) {
    layout.addWidget(&select);
    this->setLayout(&layout);
    this->setStyleSheet("QGroupBox {background: #242424;} QGroupBox:Title { color: white; }");
  }

};

struct PIDOutput : public QGroupBox {
  QGridLayout layout;
  QChartView chart_view;
  QChart chart;
  QSplineSeries temp_over_time_series;

  PIDOutput(std::string title, QWidget* parent = nullptr) : QGroupBox(QString::fromStdString(title),parent) {
    layout.addWidget(&chart_view); 
    this->setLayout(&layout);
    this->setStyleSheet("QGroupBox {background: #242424;} QGroupBox:Title { color: white; }");
  }

};

class Main : public QMainWindow
{
  private:
  QWidget central_widget;
  QGridLayout layout;
  InputBox ki_select;
  InputBox kp_select;
  InputBox kd_select;
  InputBox temp_coefficient;
  QPushButton run_sim_btn;
  PIDOutput pid_output;
  public:
    Main(QWidget* parent = nullptr) : 
      QMainWindow(parent),
      ki_select("KI",this),
      kp_select("KP",this),
      kd_select("KD",this),
      temp_coefficient("Temp Slope",this),
      run_sim_btn("Run Simulation"),
      pid_output("PID Simulation",this)
    {
      this->resize(900,600);
      layout.addWidget(&run_sim_btn,0,0,1,-1);
      layout.addWidget(&temp_coefficient, 1,0,1,-1);
      layout.addWidget(&kp_select,2,0);
      layout.addWidget(&ki_select,2,1);
      layout.addWidget(&kd_select,2,2);
      layout.addWidget(&pid_output,4,0,1,-1);
      central_widget.setLayout(&layout);
      this->setCentralWidget(&central_widget);
    }
};

#endif
