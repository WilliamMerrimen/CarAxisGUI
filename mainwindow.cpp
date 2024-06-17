#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QDebug>
#include <QtConcurrent/QtConcurrent>

bool overLimit(const std::vector<CarAxisInfo> &axes, double limit)
{
    for(auto &axis : axes) if(axis.weit_ax_abs > limit) return true;

    return false;
}

QTableWidgetItem *createItem(const QString &text, QVariant data = QVariant())
{
    auto item = new QTableWidgetItem(text);
    item->setData(Qt::UserRole, data);
    return item;
}

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    auto topLayout = new QVBoxLayout(ui->centralwidget);

    auto mainLayout = new QHBoxLayout();

    m_progressBar = new QProgressBar();
    ui->statusbar->addWidget(m_progressBar, 1);
    m_progressBar->setTextVisible(false);
    m_progressBar->setVisible(false);

    m_progressBarHelper = new ProgressBarHelper();


    // Create AxisWidget
    m_axisWidget = new CarAxisWidget();
    m_axisWidget->setVisible(false);

    // Create Table
    m_table = new QTableWidget();

    m_table->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_table->setSelectionBehavior(QAbstractItemView::SelectRows);
    m_table->setSelectionMode(QAbstractItemView::SingleSelection);


    mainLayout->addWidget(m_table);
    mainLayout->addWidget(m_axisWidget);


    auto loadLayout = new QHBoxLayout();

    auto lineEdit = new QLineEdit();
    lineEdit->setReadOnly(true);

    auto pushButton = new QPushButton("Load File");
    connect(pushButton, &QPushButton::clicked, this, [=](){
        QString fileName = QFileDialog::getOpenFileName(this, "Open File", "", tr("Data Files (*.dat)"));
        if(fileName.isEmpty()) {
            QMessageBox::warning(this, "Open File", "Can't open file");
            return;
        }

        lineEdit->setText(fileName);

        loadFile(fileName);
    });

    loadLayout->addWidget(lineEdit);
    loadLayout->addWidget(pushButton);

    topLayout->addLayout(loadLayout);

    topLayout->addLayout(mainLayout);


    // Connections
    connect(m_table, &QTableWidget::itemSelectionChanged, this, [=]() {

        auto selectedItems = m_table->selectedItems();

        if(selectedItems.isEmpty()) return;

        int row = selectedItems.first()->row();

        int index = m_table->item(row, 0)->data(Qt::UserRole).toInt();
        // qDebug() << row << index;

        m_axisWidget->setCarAxisData(m_carsInfo[index].AxisInfo);
        m_axisWidget->setVisible(true);
    });

    connect(m_table->horizontalHeader(), &QHeaderView::sectionClicked, this, [=](int logicalIndex) {
        m_table->sortItems(logicalIndex);
    });

    connect(this, &MainWindow::loadStarted, this, [=](){
        pushButton->setDisabled(true);
        m_progressBar->setVisible(true);
        m_progressBarHelper->resetProgress();
    });

    connect(this, &MainWindow::loadFinished, this, [=](){
        size_t axisMaxNum = 0;
        int carLenghtMax = 0;
        for(size_t i = 0; i < m_carsInfo.size(); ++i) {
            if(m_carsInfo[i].AxisInfo.size() > axisMaxNum) axisMaxNum = m_carsInfo[i].AxisInfo.size();

            int carLenght = m_carsInfo[i].AxisInfo.back().coord_axis - m_carsInfo[i].AxisInfo.front().coord_axis;

            if(carLenght > carLenghtMax) carLenghtMax = carLenght;
        }

        QStringList headerLabels;
        headerLabels << "#" << "Type" << "Speed, km/h" << "DateTime";

        for(size_t i = 0; i < axisMaxNum; ++i) {
            headerLabels << QString("Axis %1, kg").arg(i);
        }

        m_axisWidget->setVisible(false);
        m_table->clear();

        m_table->setRowCount(m_carsInfo.size());
        m_table->setColumnCount(headerLabels.size());

        m_table->setHorizontalHeaderLabels(headerLabels);

        m_table->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);

        for(quint64 i = 0; i < m_carsInfo.size(); ++i) {
            m_table->setItem(i, 0, createItem(QString::number(m_carsInfo[i].num_car), i));

            m_table->setItem(i, 1, createItem(carTypeStr(m_carsInfo[i].type), i));

            m_table->setItem(i, 2, createItem(QString::number(m_carsInfo[i].speed)));

            m_table->setItem(i, 3, createItem(m_carsInfo[i].date_time.toString("dd.MM.yyyy_hh:mm:ss.zzz")));

            for(size_t j = 0; j < m_carsInfo[i].AxisInfo.size(); ++j) {
                m_table->setItem(i, 4+j, createItem(QString::number(m_carsInfo[i].AxisInfo[j].weit_ax_abs)));
            }

            if(overLimit(m_carsInfo[i].AxisInfo, WEIGHT_LIMIT)) {
                for(int c = 0; c < m_table->columnCount(); ++c) {
                    auto item = m_table->item(i, c);
                    if(item) item->setBackground(QBrush("red"));
                    else {
                        item = new QTableWidgetItem();
                        item->setBackground(QBrush("red"));
                        m_table->setItem(i, c, item);
                    }
                }
            }
        }

        pushButton->setEnabled(true);
        m_progressBar->setVisible(false);
    });

    connect(m_progressBarHelper, &ProgressBarHelper::progress, this, [=](int p){
        // qDebug() << p;
        m_progressBar->setValue(p);
    });
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::loadFile(QString fileName)
{

    QtConcurrent::run([=](){
        emit loadStarted();

        m_carsInfo.clear();

        auto start = std::chrono::steady_clock::now();

        fnGetCarInfo(fileName, m_carsInfo, m_progressBarHelper);

        auto end = std::chrono::steady_clock::now();

        qDebug() << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

        emit loadFinished();
    });
}

// void MainWindow::loadFile(QString fileName)
// {
//     m_carsInfo.clear();

//     auto start = std::chrono::steady_clock::now();

//     fnGetCarInfo(fileName, m_carsInfo);

//     auto end = std::chrono::steady_clock::now();

//     qDebug() << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

//     size_t axisMaxNum = 0;
//     int carLenghtMax = 0;
//     for(size_t i = 0; i < m_carsInfo.size(); ++i) {
//         if(m_carsInfo[i].AxisInfo.size() > axisMaxNum) axisMaxNum = m_carsInfo[i].AxisInfo.size();

//         int carLenght = m_carsInfo[i].AxisInfo.back().coord_axis - m_carsInfo[i].AxisInfo.front().coord_axis;

//         if(carLenght > carLenghtMax) carLenghtMax = carLenght;
//     }

//     QStringList headerLabels;
//     headerLabels << "#" << "Type" << "Speed, km/h" << "DateTime";

//     for(size_t i = 0; i < axisMaxNum; ++i) {
//         headerLabels << QString("Axis %1, kg").arg(i);
//     }

//     m_axisWidget->setVisible(false);
//     m_table->clear();

//     m_table->setRowCount(m_carsInfo.size());
//     m_table->setColumnCount(headerLabels.size());

//     m_table->setHorizontalHeaderLabels(headerLabels);

//     m_table->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);

//     for(quint64 i = 0; i < m_carsInfo.size(); ++i) {
//         m_table->setItem(i, 0, createItem(QString::number(m_carsInfo[i].num_car), i));

//         m_table->setItem(i, 1, createItem(carTypeStr(m_carsInfo[i].type), i));

//         m_table->setItem(i, 2, createItem(QString::number(m_carsInfo[i].speed)));

//         m_table->setItem(i, 3, createItem(m_carsInfo[i].date_time.toString("dd.MM.yyyy_hh:mm:ss.zzz")));

//         for(size_t j = 0; j < m_carsInfo[i].AxisInfo.size(); ++j) {
//             m_table->setItem(i, 4+j, createItem(QString::number(m_carsInfo[i].AxisInfo[j].weit_ax_abs)));
//         }

//         if(overLimit(m_carsInfo[i].AxisInfo, WEIGHT_LIMIT)) {
//             for(int c = 0; c < m_table->columnCount(); ++c) {
//                 auto item = m_table->item(i, c);
//                 if(item) item->setBackground(QBrush("red"));
//                 else {
//                     item = new QTableWidgetItem();
//                     item->setBackground(QBrush("red"));
//                     m_table->setItem(i, c, item);
//                 }
//             }
//         }
//     }
// }
