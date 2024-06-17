 
 #ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include <QtCharts>
#include <QVBoxLayout>

#include <QTableWidget>

#include <QThread>

#include "CarAxis.h"

#define AXIS_Y_MAX 12000

#define AXIS_X_MIN 0
#define AXIS_X_MAX 20000
#define AXIS_X_OFFSET 200

#define LIMIT_LINE_WIDTH 5

#define AXIS_LINE_WIDTH 20

#define WEIGHT_LIMIT 10000

inline QString carTypeStr(int carType)
{
    switch(carType){
    case -1:
        return "light";
    case 0:
        return "medium";
    case 1:
        return "heavy";
    default:
        return "undefined";
    }
}


class CarAxisWidget : public QWidget
{
    Q_OBJECT

public:
    CarAxisWidget(QWidget *parent = nullptr) : QWidget(parent), m_showLimit(true)
    {
        auto mainLayout = new QVBoxLayout();

        m_chart = new QChart();
        m_view = new QChartView(m_chart);

        m_axisX = new QValueAxis();
        m_axisX->setRange(AXIS_X_MIN, AXIS_X_MAX);
        m_chart->addAxis(m_axisX, Qt::AlignBottom);

        m_axisY = new QValueAxis();
        m_axisY->setRange(0, AXIS_Y_MAX);
        m_chart->addAxis(m_axisY, Qt::AlignLeft);

        m_limitSeries = new QLineSeries();
        m_limitSeries->setPen(QPen(QBrush("red"), LIMIT_LINE_WIDTH));
        m_limitSeries->setVisible(m_showLimit);

        for(int i = m_axisX->min(); i <= static_cast<int>(m_axisX->max()); ++i) m_limitSeries->append(i, WEIGHT_LIMIT);
        m_chart->addSeries(m_limitSeries);

        m_limitSeries->attachAxis(m_axisX);
        m_limitSeries->attachAxis(m_axisY);

        m_chart->legend()->setVisible(false);

        mainLayout->addWidget(m_view);

        setLayout(mainLayout);
    }

    CarAxisWidget(const std::vector<CarAxisInfo> &axes, QWidget *parent = nullptr) : CarAxisWidget(parent)
    {
        setCarAxisData(axes);
    }

    ~CarAxisWidget()
    {

    }

    void setCarAxisData(const std::vector<CarAxisInfo> &axes)
    {
        for(auto axis : m_axisSeries) {
            m_chart->removeSeries(axis);
            delete axis;
        }
        m_axisSeries.clear();

        for(size_t i = 0; i < axes.size(); ++i) {
            auto series = new QLineSeries();

            series->setPen(QPen(QBrush("grey"), AXIS_LINE_WIDTH));

            series->append(axes[i].coord_axis + AXIS_X_OFFSET, 0);
            series->append(axes[i].coord_axis + AXIS_X_OFFSET, axes[i].weit_ax_abs);

            m_chart->addSeries(series);

            series->attachAxis(m_axisX);
            series->attachAxis(m_axisY);

            m_axisSeries.push_back(series);
        }

        m_chart->removeSeries(m_limitSeries);
        m_chart->addSeries(m_limitSeries);
        m_limitSeries->attachAxis(m_axisX);
        m_limitSeries->attachAxis(m_axisY);
    }

    void setMaxAxisX(double max)
    {
        m_axisX->setMax(max);

        m_limitSeries->clear();
        for(int i = 0; i <= static_cast<int>(max); ++i) {
            m_limitSeries->append(i, WEIGHT_LIMIT);;
        }
    }

    void setMaxAxisY(double max)
    {
        m_axisY->setMax(max);
    }

    void setLimit(double limit)
    {
        for(int i = 0; i < m_limitSeries->count(); ++i) {
            m_limitSeries->replace(i, m_limitSeries->at(i).x(), limit);
        }
    }

private:
    QChart *m_chart;
    QChartView *m_view;

    QValueAxis *m_axisX;
    QValueAxis *m_axisY;

    std::vector<QLineSeries *> m_axisSeries;
    QLineSeries *m_limitSeries;

    bool m_showLimit;
};


QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

signals:
    void loadStarted();
    void loadFinished();

private:
    Ui::MainWindow *ui;

    QTableWidget *m_table;

    CarAxisWidget *m_axisWidget;

    std::vector<CarInfo> m_carsInfo;

    QProgressBar *m_progressBar;

    ProgressBarHelper *m_progressBarHelper;

    void loadFile(QString fileName);
};
#endif // MAINWINDOW_H
