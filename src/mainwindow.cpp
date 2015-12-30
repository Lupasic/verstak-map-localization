#include "mainwindow.h"

MainWindow::MainWindow(int argc, char **argv, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    fl(0)
{
    ui->setupUi(this);
    setupPlot();
    ui->groupBox->hide();
    m_plblX = new QLabel(this);
    m_plblY = new QLabel(this);
    ui->map_view->set_arg(argc,argv);
    ui->statusBar->addWidget(m_plblX);
    ui->statusBar->addWidget(m_plblY);
    connect(ui->map_view,SIGNAL(coords(qreal,qreal)),this,SLOT(on_map_view_changed(qreal,qreal)));
    connect(ui->twist_view->xAxis, SIGNAL(rangeChanged(QCPRange)), this, SLOT(xAxisChanged(QCPRange)));
    connect(ui->twist_view->yAxis, SIGNAL(rangeChanged(QCPRange)), this, SLOT(yAxisChanged(QCPRange)));
    connect(ui->get_graph_button,SIGNAL(toggled(bool)),this,SLOT(on_graph_check_box_clicked(bool)));
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_map_view_changed(qreal xx, qreal yy)
{
    m_plblX->setText("X_metr = " + QString().setNum(xx));
    m_plblY->setText("Y_metr = " + QString().setNum(yy));
}

void MainWindow::setupPlot() //отрисовка графика
{
    // The following plot setup is mostly taken from the plot demos:
    ui->twist_view->addGraph();
    ui->twist_view->graph()->setPen(QPen(Qt::blue));
    ui->twist_view->graph()->setBrush(QBrush(QColor(0, 0, 255, 20)));
    ui->twist_view->addGraph();
    ui->twist_view->graph()->setPen(QPen(Qt::red));
    twist = ui->map_view->get_twist();
    time = ui->map_view->get_time();
    ui->twist_view->xAxis->setLabel("time, sec");
    ui->twist_view->yAxis->setLabel("twist, m/sec");
    ui->twist_view->graph()->setData(time, twist);
    ui->twist_view->axisRect()->setupFullAxesBox(true);
    ui->twist_view->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
}


void MainWindow::on_arrow_button_clicked(bool checked)
{
    if(checked ==false && fl%2==0)
    {
        ui->map_view->arrow_link(false);
        fl++;
    }
    else
    {
        ui->map_view->arrow_link(true);
        fl++;
    }
}

void MainWindow::on_get_graph_button_clicked()
{
    setupPlot();
}

void MainWindow::on_del_graph_button_clicked()
{
    ui->twist_view->clearGraphs();
}

