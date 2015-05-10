#include "widget.h"
#include "ui_widget.h"
#include<QKeyEvent>
#include<QMessageBox>
#include <QPainter>
#include <math.h>
#include <QTimer>


static const double Pi = 3.14159265358979323846264338327950288419717;

Widget::Widget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Widget)
{
    ui->setupUi(this);

    Tank=(QPixmap) *(ui->tank->pixmap());
    pos=ui->tank->pos();
    angle=90;
    varX=0;
    varY=0;
    QTimer *timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(updateTank()));
    timer->start(50);

}

Widget::~Widget()
{
    delete ui;
}

void Widget::updateTank(){
    int x,y;
    x=ui->tank->pos().x();
    y=ui->tank->pos().y();
    if(ui->up->isDown() || ui->down->isDown() ){
        varX+=cos((angle)*Pi/180);
        if(abs(varX)>1){
            int Xinc=round(varX);
            varX = varX - Xinc;
            if(ui->up->isDown() )
                x=x+Xinc;
            else
                x=x-Xinc;
        }
        varY+=sin((angle)*Pi/180);
        if(abs(varY)>1){
            int Yinc=round(varY);
            varY = varY - Yinc;
            if(ui->up->isDown() )
                y=y-Yinc;
            else
                y=y+Yinc;
        }
        ui->tank->move(x,y);
    }else if(ui->right->isDown()){
        angle--;
        ui->tank->setPixmap(rotatePixmap((90-angle)));
        if(angle == 360) angle = 0;
    }else if(ui->left->isDown()){
        angle++;
        ui->tank->setPixmap(rotatePixmap((90-angle)));
        if(angle == 0) angle = 360;
    }
}

QPixmap Widget::rotatePixmap(int angle){
    QSize size = Tank.size();
    QPixmap rotatedPixmap(size);
    rotatedPixmap.fill(QColor::fromRgb(0, 0, 0, 0)); //the new pixmap must be transparent.
    QPainter* p = new QPainter(&rotatedPixmap);
    p->translate(size.height()/2,size.height()/2);
    p->rotate(angle);
    p->translate(-size.height()/2,-size.height()/2);
    p->drawPixmap(0, 0, Tank);
    p->end();
    delete p;
    return rotatedPixmap;
}
