#include <QtWidgets>
#include <QApplication>
#include <Main/main.hxx>
#include <QDateTime>
#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QDateTimeAxis>
#include <QtCharts/QValueAxis>

using namespace QtCharts;

int main(int argc, char* argv[])
{
  QApplication app(argc, argv);
  Main m;
  m.show();

  return QApplication::exec();
}
