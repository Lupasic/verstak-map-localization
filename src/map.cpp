#include "map.h"
#include <iostream>
#include <ros/ros.h>


Map::Map(QWidget *parent) :
    QWidget(parent),
    fl_arrow_unlink(false),
    fl_angle(false),
    dP1(0,0),
    gpsprov(0),
    qnode(i_argc,i_argv)
{
    std::string b;
    QString br;
    ros::NodeHandle n("~");
    n.getParam("adres",b);
    n.getParam("zoom_arrow_switch",zoom_arrow_switch);
    br.append(b.c_str());
    qnode.init();
    MC = new YaMCache(br);
    guiTimer = new QTimer(this);
    connect(guiTimer, SIGNAL(timeout()), this, SLOT(onTimer()));
    guiTimer->start( 100 );
    zoomscale=MC->zoom2scale();

}

Map::~Map()
{
    delete MC;
}

void Map::onTimer()
{
    qnode.setgpps(gpsprov);
    c=Qt::red;
    gpsprov++;
    if((gpsprov - qnode.gethear())<1)
        c=Qt::green;
    else Qt::red;
    qnode.update();
    lat = qnode.getlat();
    lon = qnode.getln();
    lat_clear =qnode.getlatclear();
    lon_clear=qnode.getlnclear();
    rad_p=qnode.getrad();
    ang = qnode.getang();

    ang = ang*180/M_PI;
    ang = -90-ang;

    timet.append(qnode.gettime());
    twistt.append(qnode.gettwist_lin());
    startR.setX(-qnode.getxpose());
    startR.setY(-qnode.getypose());


    //    std::cout << "lat= " << lat << " lon= " << lon << std::endl;
    //    std::cout << "latMC= " << MC->getLatitude() << " lonMC= " << MC->getLongitude() << std::endl;
    //    std::cout << "scale=" << MC->zoom2scale() << std::endl;
    //     std::cout << "startRx=" << startR.x() << " startRy=" << startR.y() << std::endl;
    if(fl_arrow_unlink==false) // чтобы стрелка всегда была в центре
    {
        MC->setPoint(lat, lon);
        MC->update();
    }
    update();
}

void Map::arrow_link(bool check)
{
    dP1.setX(0);
    dP1.setY(0);
    if(check==true) //стрелка привязана
    {
        MC->setPoint(lat, lon);

        MC->update();
        fl_arrow_unlink=false;
    }
    else //суть этого блока, что мы перемещаем систему координат в точку, откуда начал движение робот, это позволяет не возиться с преобразованиями
    {
        QPointF del;
        del.setX(-startR.x()/MC->zoom2scale());
        del.setY(startR.y()/MC->zoom2scale());
        fl_arrow_unlink=true;
        MC->pan(del);
        MC->update();
        lat_odo = MC->getLatitude();
        lon_odo = MC->getLongitude();
    }
}


void Map::paintEvent(QPaintEvent *event)
{
    QPainter painter(this);
    painter.fillRect(rect(), Qt::white);
    painter.translate(dP);// транслирует точку, что позволяет перетаскивать мышку там, куда не отрендерилось(дает плавность в работе)
    MC->render(&painter, rect()); // отображает картинку тайловую
    painter.translate(this->rect().center());// определяет центр экрана
    painter.save();
    paintArrow(&painter);
    painter.restore();
    painter.save();
    paintArrow_clear(&painter);
    painter.restore();
    painter.save();
    paintStartCross(&painter);
    painter.restore();
    painter.save();
    if (!cross.isEmpty())
        paintCrosses(&painter);
    painter.restore();
}

void Map::paintArrow(QPainter *painter)
{
    painter->setBrush(c);
    QPainterPath path;
    if(fl_arrow_unlink==true)
        painter->translate((dP1.x()-startR.x()/MC->zoom2scale()),(dP1.y()+startR.y()/MC->zoom2scale()));
    if(fl_angle==true)
        painter->rotate(ang_old);
    else
        painter->rotate(ang);
    if(MC->getZoom()>zoom_arrow_switch)
    {
        //отрисовка недострелки при обычном масштабе
        path.moveTo(-4,-8);
        path.lineTo(+4,-8);
        path.lineTo(+4,+0);
        path.lineTo(+0,+8);
        path.lineTo(-4,+0);
        path.lineTo(-4,-8);
        painter->drawPath(path);
    }
    else
    {
        //отрисовка крутой стрелки при большом масштабе
        path.lineTo(+10,-6);
        path.lineTo(+0,+20);
        path.lineTo(+0,+0);
        path.lineTo(-10,-6);
        path.lineTo(+0,+20);
        painter->drawPath(path);
    }
}

void Map::paintArrow_clear(QPainter *painter)
{
    double x,y;
    x=(metr_from_1grad_lat(deg_rad(lat))*(lat-lat_clear))/MC->zoom2scale();
    y =(metr_from_1grad_lon(deg_rad(lat))*(lon-lon_clear))/MC->zoom2scale();

    //std::cout << " icha po odo " << sqrt(pow(startR.y(),2)+pow(startR.x(),2)) << std::endl;
    // std::cout << "raznicha po lengh " << sqrt(pow((metr_from_1grad_lat(deg_rad(lat))*(lat-lat_clear)),2) + pow(metr_from_1grad_lon(deg_rad(lat))*(lon-lon_clear),2)) <<std::endl;

    if(fl_arrow_unlink==true)
        painter->translate((dP1.x()-startR.x()/MC->zoom2scale()),(dP1.y()+startR.y()/MC->zoom2scale()));
    painter->translate(x,-y);
    painter->setBrush(Qt::blue);
    painter->drawEllipse(0,0,4,4);
    painter->setBrush(Qt::cyan);
    painter->drawEllipse(0,0,rad_p,rad_p);
}

void Map::paintStartCross(QPainter *painter)
{
    if(fl_arrow_unlink==false)
        painter->translate(startR.x()/MC->zoom2scale(),-startR.y()/MC->zoom2scale());
    else
        painter->translate(dP1);
    painter->setBrush(Qt::red);
    QPainterPath path;
    path.moveTo(+2,+2);
    path.lineTo(-2,-2);
    path.moveTo(-2,+2);
    path.lineTo(+2,-2);
    painter->drawPath(path);
}

void Map::paintCrosses(QPainter *painter)
{
    int z;
    if(fl_arrow_unlink==false)
        painter->translate(startR.x()/MC->zoom2scale(),-startR.y()/MC->zoom2scale());
    else
        painter->translate(dP1);;
    for(z=0;z<cross.size();z++)
    {
        painter->save();
        painter->translate(cross[z]/MC->zoom2scale());
        painter->setBrush(Qt::red);
        painter->drawEllipse(0,0,4,4);
        painter->restore();
    }
}

void Map::mousePressEvent(QMouseEvent *event)
{
    ang_old=ang;
    fl_angle=true;
    sP = event->pos();
    QPointF point= (event->pos() - this->rect().center() - dP1)*MC->zoom2scale();
    if(event->button()==Qt::MiddleButton)// эмуляция сигнала для статус бара
    {
        if(fl_arrow_unlink==false)
            Q_EMIT coords((event->posF().x()-this->rect().center().x())*MC->zoom2scale()-startR.x(),(-event->posF().y()+this->rect().center().y())*MC->zoom2scale()-startR.y());
        else
            Q_EMIT coords((event->posF().x()-this->rect().center().x()-dP1.x())*MC->zoom2scale(),(-event->posF().y()+this->rect().center().y()+dP1.y())*MC->zoom2scale());
    }
    if(event->button()==Qt::RightButton && fl_arrow_unlink==true)
    {
        cross.append(point);
        qnode.settaskpoint(point.x(),point.y());
    }
}


void Map::mouseReleaseEvent(QMouseEvent *event)
{
    fl_angle=false;
    dP = event->pos() - sP;
    dP1+=dP; // нужно для того чтобы когда отвязана мышь я мог перемещать систему координат
    MC->pan(dP);
    MC->update();
    dP = QPoint();
    update();
}

void Map::mouseMoveEvent(QMouseEvent *event)
{
    dP = event->pos() - sP;
    update();
}

void Map::wheelEvent(QWheelEvent *event)
{
    zoomscale = MC->zoom2scale();
    int zoom = MC->getZoom();
    // std::cout<<"get zoom " <<MC->getZoom()<<std::endl;
    if ( event->delta() > 0 ) MC->setZoom(zoom + 1);
    else MC->setZoom(zoom - 1);
    MC->update();
    dP1=dP1*zoomscale/MC->zoom2scale(); // чтобы нивелировать мастшаб
    //std::cout << "zoom2scale 2 " << MC->zoom2scale() <<std::endl;
    update();
}

void Map::resizeEvent(QResizeEvent *event)
{
    MC->resize(event->size());
    MC->update();
    update();
}

