#ifndef MAP_H
#define MAP_H

#include <qt4/QtGui/QWidget>
#include <qt4/QtGui/QtGui>
#include "qnode.h"
#include "yamcache.h"
#include "mercator_convertions.h"

//                     1
//система координат 1  0 (-1)
//                   (-1)


class Map : public QWidget
{
    Q_OBJECT
public:
    explicit Map(QWidget *parent = 0);
    void set_arg(int c, char** d){this->i_argc=c; this->i_argv=d;}
    QVector<double> get_twist(){return twistt;}
    QVector<double> get_time(){return timet;}
    QNode qnode;
    ~Map();

public Q_SLOTS:
    void onTimer();
    void arrow_link(bool check);

Q_SIGNALS:
    void coords(qreal xx,qreal yy);

private:
    char** i_argv;
    int i_argc;
    bool fl_arrow_unlink, fl_angle; // fl - флаг для отвязки стрелки, fl_angle для события при перемещении карты с отвязанной стрелкой(исправление ротейта)
    QPointF dP,dP1; //дельта точка(на сколько произошло перемещение) и суммарная дельта точка(используется для перемещения карты при отвязанной стрелке
    QPointF sP; //стартовая точка
    QTimer* guiTimer; //таймер
    YaMCache *MC;
    int gpsprov; //счетчик для проверки gps
    double lat, lon, lat_clear, lon_clear,lat_odo, lon_odo; // широта и долгота двух топиков(калман и чистые данные и одо ), которые получаются через топик
    QPointF startR; //Точка откуда начал передвижение робот
    qreal ang,rad_p, ang_old;  //угол
    qreal zoomscale; // нужна для масштаба(сохраняется предыдущее состояние zoom2scale
    qreal dop; //показывал периметр примерного нахождения
    QColor c; // Цвет при ловле или нет сигнала
    QVector<QPointF> cross; // массив целевых точек
    QVector<double> twistt, timet;
    int zoom_arrow_switch;

protected:
    //обработки
    void paintEvent(QPaintEvent *event); //отрисовка тела
    void paintArrow (QPainter *painter); //отрисовка стрелки
    void paintArrow_clear (QPainter *painter); //отрисовка точки погрешности
    void paintStartCross (QPainter *painter); //отрисовка начальной точки
    void paintCrosses(QPainter *painter); //отрисовка крестов
    void mousePressEvent(QMouseEvent *event); //нажатие кнопки мышки
    void mouseReleaseEvent(QMouseEvent *event); //отжатие
    void mouseMoveEvent(QMouseEvent *event); //движение мышки
    void wheelEvent(QWheelEvent *event);  // колесико мыши
    void resizeEvent(QResizeEvent *event); // отрисовка при изменении масштаба
};

#endif // MAP_H
