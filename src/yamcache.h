#ifndef YAMCACHE_H
#define YAMCACHE_H

#include <qt4/QtGui/QtGui>
#include <qt4/QtCore/QtCore>
#include <iostream>
#include <stdint.h>
#include "mercator_convertions.h"


//тайловая или плиточная графика - метод создания больших изображений
// tile size in pixels
const int tdim = 128; // не трогать


//tile from smallest pixel at zoom
inline double tile(double p, int zoom)
{  // 1 << (23 - zoom) == (number of smallest pixels) / number of pixels at zoom
    return ( p / (1 << (23 - zoom) ) ) / (double)tdim;
}
//smallest pixel from tile at zoom
inline double pixel( double t, int zoom)
{
    return ( t * tdim ) * (1 << (23 - zoom ));
}

//input degrees
inline QPointF tileForCoordinate(qreal lat, qreal lng, int zoom)
{
    return QPointF (tile(yampoint_x(merc_x(lng)), zoom - 1),
                    tile(yampoint_y(merc_y(lat)), zoom - 1));
}

inline qreal longitudeFromTile(qreal tx, int zoom )
{
    return lon_x( merc_x_fromyam(pixel(tx, zoom - 1)) );
}

inline qreal latitudeFromTile(qreal ty, int zoom)
{
    return lat_y( merc_y_fromyam(pixel(ty, zoom - 1)) );
}

template <class TT, class T >
TT mortonCode(T x, T y)
{
    TT res(0);
    for( unsigned int i = 0; i<sizeof(T)*8; i++)
    {
        res |= ( (1<<i)&x ) << (i);
        res |= ( (1<<i)&y ) << (i+1);
    }
    return res;
}

inline uint qHash(const QPoint& p)
{
    return p.x() * 18 ^ p.y();
}




class YaMCache
{
protected:
    //Размер картинки
    int width;
    int height;
    //Нынешний уровень зума
    int zoom;
    //meter /pix
    qreal scale;
    //Координаты кэш центра
    qreal latitude;
    qreal longitude;
    //Координаты центра на карте в пикселях
    QPointF centerP;

public:
    YaMCache(const QString& file);
    virtual ~YaMCache();
    const QPointF& getCenter(){ return centerP; }

    qreal getLatitude() const{return latitude;}
    qreal getLongitude() const{return longitude;}
    qreal getHeight() const { return height; }
    qreal getWidth() const { return width; }
    void setCache(const QString& file); //установка кэш файла(прописывается в мейне путь)

protected:

    QPoint m_offset; //offset between top left tile and image top left corner
    //rect in tiles units on whole map at this zoom level
    QRect m_tilesRect;
    QImage m_emptyTile;
    QHash<QPoint, QImage> m_tileImages; //cache for cache
    QString cache;//директория с яндекс кэшем
    bool loadFromFile(QString& fn, uint8_t x, uint8_t y, QImage& image); //загружает файл из файла яндекс кэша
    void getTile(int x,int y,int z, QImage& image);//читает файл из файла кэша
    void updateHash();//обновляет хэш учитывая новую позицию и зум
    QRect tileRect(const QPoint &tp) //считает рект тайла на текущем кэше
    {
        QPoint t = tp - m_tilesRect.topLeft();
        int x = t.x() * tdim + m_offset.x();
        int y = t.y() * tdim + m_offset.y();
        return QRect(x, y, tdim, tdim);
    }
public:

    void pan(QPointF delta);// передвижение изображения
    void pan(QPoint delta);
    void resize(QSize s);  //пересчет размеров
    void setZoom(int z);
    int getZoom() {return zoom;}
    int setScale(qreal s); //установить шкалу
    void setPoint(qreal lat, qreal lon) { latitude = lat; longitude = lon;  }
    void update() { updateHash(); }
    void render(QPainter * painter, const QRect &rect);//отрендерить изображение
    void getImage(QImage &image);// получить изображение
    qreal zoom2scale(); //Возвращает шкалу текущего зума и координаты в метрах/пикселях
};

#endif // YAMCACHE_H
