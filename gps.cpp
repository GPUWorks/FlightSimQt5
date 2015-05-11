#include "gps.h"
#include "ui_gps.h"
#include <QTimer>
#include <math.h>
#include <QPixmap>

#include<QKeyEvent>
#include <QPainter>




static const double Pi = 3.14159265358979323846264338327950288419717;


GPS::GPS(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::GPS)
{
    ui->setupUi(this);

    Avion=(QPixmap) *(ui->avion->pixmap());
    x=ui->mapa->pos().x();
    y=ui->mapa->pos().y();


    yaw=0;
    velocidad=0;
    pitch=0;
    lastYaw=0;


    QTimer *timer = new QTimer(this);
    timer->connect(timer, SIGNAL(timeout()),
                   this, SLOT(update()));
    timer->start(200);
}

GPS::~GPS()
{
    delete ui;
}

void GPS::update(){

    double dx,dy;
    int pix_por_km=500;
    if(velocidad>0){
        x=ui->mapa->pos().x();
        y=ui->mapa->pos().y();

        if(lastYaw!=yaw){
            ui->avion->setPixmap(rotatePixmap(360-yaw));
            lastYaw=yaw;
        }

        dy = (double) cos((yaw)*Pi/180)*cos((pitch)*Pi/180)*(velocidad/3600)*0.2*pix_por_km;
        dx = (double)sin((yaw)*Pi/180)*cos((pitch)*Pi/180)*(velocidad/3600)*0.2*pix_por_km;

        y+=dy;
        x+=dx;
        if((y<33 && x<33) && (y>-5502 && x>-10165) ){
           ui->mapa->move(x,y);
        }else{
            if(y>=33){
                y=-5500;
            }

            if(x>=33){
                x=-10160;
            }

            if(y<=-5502){
                y=30;
            }

            if(x<=-10165){
                x=30;
            }
            ui->mapa->move(x,y);
        }

    }


}

void GPS::setYaw(float Yaw){
    this->yaw=Yaw;
}

void GPS::setVelocidad(float velocidad){
    this->velocidad=velocidad;
}

void GPS::setPitch(float pitch){
    this->pitch=pitch;
}

QPixmap GPS::rotatePixmap(int angle){
    QSize size = Avion.size();
    QPixmap rotatedPixmap(size);
    rotatedPixmap.fill(QColor::fromRgb(0, 0, 0, 0)); //the new pixmap must be transparent.
    QPainter* p = new QPainter(&rotatedPixmap);
    p->translate(size.height()/2,size.height()/2);
    p->rotate(angle);
    p->translate(-size.height()/2,-size.height()/2);
    p->drawPixmap(0, 0, Avion);
    p->end();
    delete p;
    return rotatedPixmap;




}
