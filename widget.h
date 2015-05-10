#ifndef WIDGET_H
#define WIDGET_H

#include <QWidget>
#include <QPoint>

namespace Ui {
class Widget;
}

class Widget : public QWidget
{
    Q_OBJECT

public:
    explicit Widget(QWidget *parent = 0);
    ~Widget();
     QPixmap rotatePixmap(int angle);

private slots:
    void updateTank();

private:
    Ui::Widget *ui;
    QPixmap Tank;
    QPoint pos;
    QTimer *timer;
    int angle;
    double varX;
    double varY;


};

#endif // WIDGET_H
