#ifndef MAIN_HXX
#define MAIN_HXX
#define NUM_L 300
#include <iostream>
#include <QtWidgets>
#include <chrono>
#include <thread>
#include <cmath>
#include <ctime>
#include <string.h>
#include <QtCharts/QChartView>
#include <QtCharts/QSplineSeries>
#include <QtCharts/QDateTimeAxis>
#include <QtCharts/QValueAxis>
#include <iomanip>
using namespace QtCharts;


/** @brief Wrapper of Set values for simulation */
struct SimData
{
  double target_temp;
  double duration;
  double temp_c;
  double dt;
  double kp;
  double ki;
  double kd;
  double disp_c;
  double ambient_temp;

  std::shared_ptr<QSplineSeries> temp_series;
  std::shared_ptr<QLineSeries> target_series;

  // Move constructor and Copy Constructor
  SimData(SimData&&) noexcept = default;
  SimData& operator=(SimData&&) noexcept = default;
  SimData() = default; 

  // Constructor with parameter list to support brace-enclosed initialization
  SimData(double target_temp, double duration, double temp_c, double dt, double kp, double
      ki, double kd, 
      double disp_c, double ambient_temp)
    : target_temp(target_temp),
    duration(duration),
    temp_c(temp_c),
    dt(dt),
    kp(kp),
    ki(ki),
    kd(kd),
    disp_c(disp_c),
    ambient_temp(ambient_temp),
    temp_series(std::make_shared<QSplineSeries>()),
    target_series(std::make_shared<QLineSeries>())
  {}

  friend std::ostream& operator<<(std::ostream& os, const SimData& sd) {
    os << "SIM Data"
      << "\nTarget Temperature:\t"     << std::setw(5) <<  sd.target_temp
      << "\nAmbient Temperature:\t"     << std::setw(5) <<  sd.ambient_temp
      << "\nDuration:\t\t"             << std::setw(5) <<  sd.duration
      << "\nTemp Constant:\t\t"        << std::setw(5) <<  sd.temp_c 
      << "\nDisipation Constant:\t"    << std::setw(5) <<  sd.disp_c 
      << "\nSample Time:\t\t"          << std::setw(5) <<  sd.dt 
      << "\nKP:\t\t\t"                 << std::setw(5) <<  sd.kp 
      << "\nKI:\t\t\t"                 << std::setw(5) <<  sd.ki 
      << "\nKD:\t\t\t"                 << std::setw(5) <<  sd.kd << std::endl;
    return os;
  }
};



/** @brief Heater object that simulates the heating and temperature system 
 **        this model will use the Single Node Approach                   */
struct Heater
{
  // Heater Variables
  /** @brief constructor of Heater */
  std::shared_ptr<SimData> sd;
  const double WATER_HC = 4186;
  const double WATER_MASS = 100; // grams of water
  const double HEATING_PWR = 1300; // watts of power
  double current_temp;
  double elapsed_time;
  explicit Heater(std::shared_ptr<SimData> sd) {
    this->sd = sd;
    current_temp = sd->ambient_temp;
  }
  /** @brief calculate the new temperature of heater given PID calculation */
  inline double calculate_temp(double pid_gain) {
    //Increment elapsed time
    elapsed_time += sd->dt;

    // Dissipation amount: proportional to the temperature difference and dissipation constant
    double temp_diff = current_temp - sd->ambient_temp;
    double dis_amt = (sd->disp_c * temp_diff) / (WATER_MASS * WATER_HC);

    // Heat addition due to PID-controlled power
    double heat_input = (pid_gain * HEATING_PWR) / (WATER_MASS * WATER_HC);

    // Net temperature change
    double dT = heat_input - dis_amt;

    // Update current temperature
    current_temp += dT * sd->dt*1000;

    return current_temp;
  }

};


// @brief PID object to handle the PID system and calculate the gain given the error of the system */
struct PID
{
  // PID Variables
  std::shared_ptr<SimData> sd;
  double current_temp;
  double target_temp;
  double current_gain;
  //PID calculation Variables
  double integral_val;
  double previous_error;
  double integral_limit = 100.0;
  double derivative_val = 0.0;
  /** @brief constructor of PID object */
  PID(std::shared_ptr<SimData> sd) {
    current_temp = sd->ambient_temp;
    target_temp = sd->target_temp;
    current_gain = 0.0;
    integral_val = 0.0;
    previous_error = 0.0;
    this->sd = sd;

  }
  /** @breif sets the current_temp given by the Heater object */
  void set_current_temp(double temp) { this->current_temp = temp; }
  /** @brief calculate the gain given system data and current error */
  double calc_gain(){

    // calculate error and P in PID
    double error =  sd->target_temp - current_temp;
    auto proportional_val = error;

    // calculate I in PID
    auto sample_accum = error * sd->dt;
    integral_val += sample_accum;
    if(integral_val > integral_limit) integral_val = integral_limit;
    if(integral_val < -integral_limit) integral_val = -integral_limit;

    // calculate D in PID
    derivative_val = (sd->dt != 0) ? (error - previous_error) / sd->dt : 0.0;
    previous_error = error;
    // calculate PID pre-mapped output


    auto pre_gain = (1.5 * proportional_val) + (0.05 * integral_val) + (0.01 * derivative_val);
    // map the pre_gain to a value between 0 to 1
    //    auto mapped_gain = 1 - (1 / (1 + abs(pre_gain)) );
    auto mapped_gain = std::clamp(pre_gain, 0.0, 1.0); // Linear mapping
    if(mapped_gain > 1.0) mapped_gain = 1.0;
    if(mapped_gain < 0.0) mapped_gain = 0.0;
    current_gain = mapped_gain;
    return mapped_gain;
  }

  friend std::ostream& operator<<(std::ostream& os, PID& pid){
    os << "PID OUTPUT\n---------------------------\n"
      << "Error:        " << pid.previous_error << std::endl
      << "Gain:         " << pid.current_gain << std::endl
      << "Integral:     " << pid.integral_val << std::endl
      << "Derivative:   " << pid.derivative_val << std::endl
      << "Current Temp: " << pid.current_temp << std::endl
      << "Target Temp: " << pid.target_temp << std::endl;
    return os;
  }

};



/** @brief Inputbox container */
struct InputBox : public QGroupBox {
  // InputBox Variables
  QVBoxLayout layout;
  QDoubleSpinBox select;
  /** @brief Constructor of Input Box */
  InputBox(std::string title, QWidget* parent = nullptr) : QGroupBox(QString::fromStdString(title),parent), select(parent) {
    layout.addWidget(&select);
    select.setRange(0,1000);
    select.setSingleStep(.01);
    this->setLayout(&layout);
    this->setStyleSheet("QGroupBox {background: #242424;} QGroupBox:Title { color: white; }");
  }
  /** @brief getter for internal value */
  double get_data() { return this->select.value(); }

};

/** @brief Handles the inputs from the user to perform the specified simulation of the PID */
struct PIDInput : public QGroupBox {
  // PIDInput Variables
  QGridLayout layout;
  InputBox target_temp;
  InputBox duration;
  InputBox ki;
  InputBox kp;
  InputBox kd;
  InputBox dt;
  InputBox ambient_temp;
  InputBox disp_coefficient;
  /** @brief PIDInput Constructor */
  PIDInput(std::string title, QWidget* parent = nullptr) : 
    QGroupBox(QString::fromStdString(title),parent),
    target_temp("Target Temperature",this),
    duration("Duration",this),
    ki("KI",this),
    kp("KP",this),
    kd("KD",this),
    dt("samplling time (milliseconds)",this),
    disp_coefficient("Disipation Constant",this),
    ambient_temp("ambient temp",this)
  {
    // Set Default Values
    target_temp.select.setValue(60);
    duration.select.setValue(30);
    ki.select.setValue(0.01);
    kp.select.setValue(1.5);
    kd.select.setValue(0.05);
    dt.select.setValue(1);
    ambient_temp.select.setValue(20);
    disp_coefficient.select.setValue(1);

    this->setStyleSheet("QGroupBox {background: #242424;  } QGroupBox:Title { color: white; }");
    layout.addWidget(&duration, 0,0,1,-1);
    layout.addWidget(&target_temp, 1,0,1,-1);
    layout.addWidget(&ambient_temp, 2,0); 
    layout.addWidget(&disp_coefficient, 2,1);
    layout.addWidget(&dt, 2,2);
    layout.addWidget(&kp,3,0);
    layout.addWidget(&ki,3,1);
    layout.addWidget(&kd,3,2);
    this->setLayout(&layout);
  }
  /** @brief Get the inputed simulation data */
  SimData get_sim_data() {
    return (SimData){ target_temp.get_data(),
      duration.get_data(),
      0,
      dt.get_data(),
      kp.get_data(),
      ki.get_data(),
      kd.get_data(),
      disp_coefficient.get_data(),
      ambient_temp.get_data()
    };
  }

};



struct RelayPWMOutput : public QGroupBox
{
  std::shared_ptr<SimData> sd;
  QGridLayout layout;
  QLCDNumber duty_cycle;
  QLCDNumber duration;
  RelayPWMOutput(std::string title, QWidget* parent) : QGroupBox(QString::fromStdString(title), parent)
  {
    layout.addWidget(&duty_cycle,1,0);
    layout.addWidget(&duration,0,0);
    this->setLayout(&layout);
  }

  void generate_pulse_width(double prop_on)
  {
    // generate a lineSeries that depicts the PWM signal for that period given
    // sample time

  }
};


/** @brief Handles the output of the PID output */
struct PIDOutput : public QGroupBox {
  QGridLayout layout;
  QChartView chart_view;
  QChart chart;
  RelayPWMOutput r_pwm;
  QValueAxis temp_axis;
  QValueAxis time_axis;

  /** @brief Constructor
    @param title of the chart */
  PIDOutput(std::string title, QWidget* parent = nullptr) : QGroupBox(QString::fromStdString(title),parent), r_pwm("Relay PWM",this) {
    layout.addWidget(&chart_view,0,0);
    layout.addWidget(&r_pwm,0,1);
    // Set up chart in SimOut
    this->chart_view.setChart(&this->chart);

    chart.addAxis(&temp_axis, Qt::AlignLeft);
    temp_axis.setRange(0,100);
    temp_axis.setTitleText("Temperature (C)");
    time_axis.setRange(0,100);
    time_axis.setTitleText("Time (ms)");
    chart.addAxis(&time_axis, Qt::AlignBottom);

    this->setLayout(&layout);
    this->setStyleSheet("QGroupBox {background: #242424;} QGroupBox:Title { color: white; }");
    this->chart.setTitle("Temperature Over Time");
  }



};


struct Simulation
{
  PID pid;
  Heater heater;
  std::shared_ptr<SimData> sd;
  std::shared_ptr<PIDOutput> pid_out;
  std::chrono::time_point<std::chrono::high_resolution_clock> start_time;
  std::chrono::time_point<std::chrono::high_resolution_clock> end_time;

  explicit Simulation(std::shared_ptr<SimData> sim_data,std::shared_ptr<PIDOutput> pid_out) : pid(sim_data), heater(sim_data) {
    sd = sim_data;
    this->pid_out = pid_out;
    start_time = std::chrono::high_resolution_clock::now();
    end_time = std::chrono::high_resolution_clock::now();
  }
  void run()
  {
    sd->target_series = std::make_unique<QLineSeries>();
    sd->temp_series = std::make_unique<QSplineSeries>();
    for(int i = 0; i < sd->duration; i++)
    {
      sd->target_series->append(i,sd->target_temp);
    }

    pid_out->chart.addSeries(sd->target_series.get());
    pid_out->chart.addSeries(sd->temp_series.get());
    sd->target_series->attachAxis(&pid_out->temp_axis);
    sd->temp_series->attachAxis(&pid_out->temp_axis);
    sd->target_series->attachAxis(&pid_out->time_axis);
    sd->temp_series->attachAxis(&pid_out->time_axis);


    double sim_time = 0.0;
    while(sim_time <= static_cast<double>(sd->duration))
    {

      auto pid_gain = pid.calc_gain();
      auto temp = heater.calculate_temp(pid_gain);
      pid.set_current_temp(temp);

      //plot data
      sd->temp_series->append(sim_time,temp);

      pid_out->chart.update();
      // Dynamically adjust axis range
      adjustAxisRange(sd->temp_series.get(), &pid_out->temp_axis);
      adjustXAxisRange(sd->temp_series.get(), &pid_out->time_axis);
      sim_time += sd->dt;
    }

  }

  // Dynamically adjust range based on the series' data
  void adjustAxisRange(QLineSeries* series, QValueAxis* axis) {
    if (!series->points().isEmpty()) {
      qreal min = series->points().first().y();
      qreal max = series->points().first().y()+50;
      for (const QPointF& point : series->points()) {
        if (point.y() < min) min =
          point.y();
        if (point.y() > max)
          max = point.y();
      }
      axis->setRange(min,max);
    }
  }
  // Dynamically adjust range based on the series' data
  void adjustXAxisRange(QLineSeries* series, QValueAxis* axis) {
    if (!series->points().isEmpty()) {
      qreal min = series->points().first().x();
      qreal max = series->points().first().x();
      for (const QPointF& point : series->points()) {
        if (point.x() < min) min =
          point.y();
        if (point.x() > max)
          max = point.x();
      }
      axis->setRange(min,max);
    }
  }


  /** @brief get current elapsed time since start of PID */
  double get_time() {
    end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> e_time = end_time - start_time;
    return e_time.count();
  }
};

/** @brief Main Object of Top level program */
struct Main : public QMainWindow {
  // Main Variables
  QWidget central_widget;
  QPushButton run_sim_btn;
  QGridLayout layout;
  PIDInput pid_input;
  std::shared_ptr<PIDOutput> pid_output;
  std::shared_ptr<SimData> sd;
  bool running;
  /** @brief Constructor of Main object */
  Main(QWidget* parent = nullptr) : 
    QMainWindow(parent),
    pid_input("PID Input", this),
    pid_output(std::make_shared<PIDOutput>("PID Simulation",this)),
    run_sim_btn("Run Simulation") {
      running = false;
      QObject::connect(&run_sim_btn, &QPushButton::pressed, this, &Main::run);

      this->setMenuWidget(&run_sim_btn);
      layout.addWidget(&pid_input,0,0);
      layout.addWidget(pid_output.get(),1,0);
      central_widget.setLayout(&layout);
      this->setCentralWidget(&central_widget);
      this->resize(1000,1000);
    }
  void run()
  {
    running = !running;

    this->sd = std::make_shared<SimData>(pid_input.get_sim_data());
    if(running)
    {

      std::cout << "Simulation Started\n----------------------------\n" << *sd;
      run_sim_btn.setText("Reset Simulation");
      run_sim_btn.setStyleSheet("background-color: red; color: white;");
      Simulation sim(sd,pid_output);
      sim.run();
    }
    else
    {
      std::cout << "----------------------------\nSimulation Stopped\n----------------------------\n";
      run_sim_btn.setText("Run Simulation");
      run_sim_btn.setStyleSheet("background-color: white; color: black;");
    }

  }

};

#endif
