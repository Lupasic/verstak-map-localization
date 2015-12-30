#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <qt4/QtGui/QMainWindow>
#include <qt4/QtGui/QtGui>
#include <qt4/QtCore/QtCore>
#include "../../../build/project_yamap/ui_mainwindow.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(int argc, char** argv, QWidget *parent = 0);
    ~MainWindow();
    void setupPlot();

public Q_SLOTS:

    void on_map_view_changed(qreal xx, qreal yy);

    void on_arrow_button_clicked(bool checked);

    void on_get_graph_button_clicked();

    void on_del_graph_button_clicked();


private:
    int fl;
    QLabel* m_plblX;
    QLabel* m_plblY;
    Ui::MainWindow *ui;
    QVector<double> twist, time;
};

#endif // MAINWINDOW_H
