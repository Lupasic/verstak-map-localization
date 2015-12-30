#include "yamcache.h"
#include <ros/ros.h>
#include <list>
#include <vector>

//#define DOUT(x, x1) std::cout<<x<<" "<<x1<<std::endl;
#define DOUT(x, x1)
//Настройки
#define W 400 //ширина
#define H 300 //высота
#define Z 17 //зум
#define La 55.80719070
#define Lo 37.99699076

YaMCache::YaMCache(const QString& file ):
    width(W)
  , height(H)
  , zoom(Z)
  , latitude(La)
  , longitude(Lo)
  , cache(file)
{
    m_emptyTile = QImage(tdim, tdim, QImage::Format_ARGB6666_Premultiplied);
    m_emptyTile.fill(Qt::lightGray); //окраска где нет карты
}

void YaMCache::setCache(const QString& file)
{
    cache = file;
    m_tileImages.clear();
    updateHash();
}

bool YaMCache::loadFromFile(QString& fn, uint8_t x, uint8_t y, QImage& image)
{
    static QFile File;
    static int table_seek = 0;
    //new file prepare file
    if ( File.fileName() != fn )
    {
        if ( File.isOpen() ) File.close();
        File.setFileName(fn);

        if ( !File.open( QIODevice::ReadOnly ) )
        {
            std::cout<<"error open file "<<File.fileName().data()<<std::endl;
            File.setFileName("");
            return false;
        }
        char yndx[4] ;
        //проверка яндекс кэш
        File.read(yndx, 4);
        if ( yndx[0] != 'Y' || yndx[1] != 'N' || yndx[2] != 'D' || yndx[3] != 'X'  )
        {
            std::cout<<" error in cache file : no YNDX"<<std::endl;
            return false;
        }
        int res;
        File.read((char*)&res, 2);

        File.read((char*)&table_seek, sizeof(table_seek));
        char aut[3];
        File.read((char*)&aut, sizeof(aut));
    }
    int num = mortonCode<uint16_t, uint8_t>(x,y);
    //	std::cout<<" morton("<<(int)x<<","<<(int)y<<") = "<<num<<std::endl;

    if ( ! File.seek(table_seek + 6* num))
    {
        std::cout<<"error in table "<<std::endl;
        return false;
    }
    //read image seek
    int sk;
    File.read((char*)&sk, 4);
    if (sk <= table_seek )
    {
        std::cout<<" no image in cache"<<std::endl;
        return false;
    }
    //image size
    uint16_t sz;
    File.read((char*)&sz, 2);

    if ( !File.seek(sk) )
    {
        std::cout<<" error seek image"<<std::endl;
        return false;
    }
    //	assert(sz > 0 && sz < 20000 );
    std::vector<unsigned char> data(sz);

    if (  File.read((char*)&data[0], sz) <= 0)
    {
        std::cout<<" error read image "<<std::endl;
        return false;
    }

    image = QImage::fromData(&data[0], sz);
    return true;
}


void YaMCache::getTile(int x,int y,int z, QImage& image)
{
    if ( x < 0 || y < 0 )
    {
        image = m_emptyTile;
        return;
    }
    //last bytes
    uint8_t i = x;
    uint8_t j = y;

    //lists with hex
    std::list<uint8_t> listX, listY;
    //for all hex at this zoom except last
    for( int i = ( (z - 1)/4  ); i>1; i--)
    {
        uint8_t hex = (x >> (i*4)) & 0xF;
        listX.push_back(hex);
        hex = ( y >> (i*4) ) & 0xF;
        listY.push_back(hex);
    }

    //form result list xy -xy -xy ...
    std::list<uint8_t> res;
    while( ! ( listX.empty()&&listY.empty() ) )
    {
        if ( ! listX.empty() )
        {
            res.push_front(listX.back()<<4);
            listX.pop_back();
        }
        else res.push_front(0);

        if ( ! listY.empty() )
        {
            res.front() += listY.back();
            listY.pop_back();
        }
    }
    QString fileName(cache);
    fileName.append(QString("/%1/").arg(z-1) );
    if ( res.empty()) fileName.append("00/");
    else
        //add dirs from res
        for( std::list<uint8_t>::iterator it = res.begin(); it!= res.end(); it++)
            fileName.append(QString("%1/").arg(*it, 2, 16));

    //add map type
    fileName[fileName.length() - 1] = '1';
    DOUT("load file ", fileName.toAscii().data())
            DOUT(x,y)
            if ( loadFromFile(fileName, i, j, image) ) return;
    //else
    DOUT("load previouse zoom", z-1)
            //no file - get tile from previous scale
            if ( z > 1 )
    {
        QImage big;
        getTile(x/2, y/2, z-1, big);
        //select quarter of big pixmap
        int qx(0), qy(0);
        if ( x%2 == 1 ) qx = 1;
        if ( y%2 == 1 ) qy = 1;
        image = big.copy(qx*tdim/2, qy*tdim/2, tdim/2, tdim/2).scaled(tdim, tdim);
        return;
    }
    //otherwise
    image =  m_emptyTile;

}

void YaMCache::updateHash()
{
    if (width <= 0 || height <= 0)
        return;

    QPointF ct = tileForCoordinate(latitude, longitude, zoom);
    //   std::cout<<"zoom "<<zoom<<" center tile "<<latitude<<" / "<<longitude<<" x ="<<ct.x()<<" "<<ct.y()<<std::endl;
    //    double s = zoom2scale();
    //    std::cout<<" scale = "<<s<< " meter in pixel"<<std::endl;
    //    //std::cout<<" zoom  = "<<setScale(s)<<std::endl;
    qreal tx = ct.x();
    qreal ty = ct.y();

    // pixel top-left corner of the center tile on submap
    int xp = width / 2 - (tx - floor(tx)) * tdim;
    int yp = height / 2 - (ty - floor(ty)) * tdim;

    centerP.setX((double)xp + (double)tdim*(tx - floor(tx)));
    centerP.setY((double)yp + (double)tdim*(ty - floor(ty)));

    //half size of submap in tiles apprx(>=)
    int xa = (xp + tdim - 1) / tdim;
    int ya = (yp + tdim - 1) / tdim;
    // first tile vertical and horizontal
    int xs = static_cast<int>(tx) - xa;
    int ys = static_cast<int>(ty) - ya;

    // offset in pixels for top-left tile w.r.t submap topleft (i.e. origin)
    m_offset = QPoint(xp - xa * tdim, yp - ya * tdim);

    // last tile vertical and horizontal
    int xe = static_cast<int>(tx) + (width - xp - 1) / tdim;
    int ye = static_cast<int>(ty) + (height - yp - 1) / tdim;

    // build a rect in tiles on Map
    m_tilesRect = QRect(xs, ys, xe - xs + 1, ye - ys + 1);

    // purge unused spaces
    QRect bound = m_tilesRect.adjusted(-2, -2, 2, 2);
    // foreach(QPoint tp, m_tileImages.keys())
    Q_FOREACH(QPoint tp, m_tileImages.keys())
        if (!bound.contains(tp))
            m_tileImages.remove(tp);

    //load images from file system
    for (int x = 0; x <= m_tilesRect.width(); ++x)
        for (int y = 0; y <= m_tilesRect.height(); ++y)
        {
            QPoint tp = m_tilesRect.topLeft() + QPoint(x, y);
            if (!m_tileImages.contains(tp)) {
                getTile(tp.x(), tp.y(), zoom, m_tileImages[tp]);
                //				QImage pm =  getTile(tp.x(), tp.y(), zoom);
                //				if ( !pm.isNull() )	m_tileImages[tp] = pm;
                //				else std::cout<<" no image "<<tp.x()<<" "<<tp.y()<<std::endl;
            }
        }
}

void YaMCache::resize(QSize s)
{
    width = s.width();
    height = s.height();
}

void YaMCache::pan(QPoint delta)
{
    //	std::cout<<" int pan "<<std::endl;
    pan(QPointF(delta));
}

void YaMCache::pan(QPointF delta)
{
    QPointF dx = delta / qreal(tdim);
    //    std::cout<<"pan dx = "<<dx.x()<<" "<<dx.y()<<std::endl;
    QPointF center = tileForCoordinate(latitude, longitude, zoom) - dx;
    //    std::cout<<"new center "<<center.x()<<" "<<center.y()<<std::endl;
    latitude = latitudeFromTile(center.y(), zoom);
    longitude = longitudeFromTile(center.x(), zoom);
}



void YaMCache::setZoom(int z)
{
    if ( z > 18 ) z = 18;
    if ( z< 1 ) z = 1;
    if (zoom != z ) m_tileImages.clear();
    zoom = z;
}

qreal YaMCache::zoom2scale()
{
    qreal lat = deg_rad(latitude);
    //length of circle at this latitude / number of pixels
    return 2*M_PI*R_MAJOR*cos(lat) / sqrt( 1 - ECCENT*ECCENT*sin(lat)*sin(lat) ) /
            (tdim * (1 << zoom ));
}

int YaMCache::setScale(qreal s)
{
    qreal lat = deg_rad(latitude);
    int z = 2*M_PI*R_MAJOR*cos(lat) / sqrt( 1 - ECCENT*ECCENT*sin(lat)*sin(lat) ) /
            (tdim * s);
    int zo = 1;
    while (z)
    {
        z=z>>1;
        zo ++;
    }
    setZoom(zo-2);
    return zoom;
}



void YaMCache::render(QPainter * p, const QRect &rect)
{
    for (int x = 0; x <= m_tilesRect.width(); ++x)
        for (int y = 0; y <= m_tilesRect.height(); ++y) {
            //coordinates of tile in m_tilesRect
            QPoint tp(x + m_tilesRect.left(), y + m_tilesRect.top());
            //box in pixels in submap coordinates
            QRect box = tileRect(tp);
            //            std::cout<<"x = "<<x<<" y = "<<y<<std::endl;
            if (rect.intersects(box))
            {
                if (m_tileImages.contains(tp))
                {
                    //                	std::cout<<" draw map "<<tp.x()<<" "<<tp.y()<<std::endl;
                    p->drawImage(box, m_tileImages.value(tp));
                }
                else
                {
                    //                	std::cout<<" draw empty"<<std::endl;
                    p->drawImage(box, m_emptyTile);
                }
            }
        }
}

void YaMCache::getImage( QImage &image)
{
    QPainter p(&image);
    render(&p, image.rect());
}

YaMCache::~YaMCache()
{
}
