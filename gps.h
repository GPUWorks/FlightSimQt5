#ifndef GPS_H
#define GPS_H

#include <QWidget>
#include <QPixmap>

namespace Ui {
class GPS;
}

class GPS : public QWidget
{
    Q_OBJECT

public:
    explicit GPS(QWidget *parent = 0);
    ~GPS();
    void setYaw(float Yaw);
    void setVelocidad(float velocidad);
    void setPitch(float pitch);

private:
    QPixmap rotatePixmap(int angle);

private slots:
    void update();


private:
    Ui::GPS *ui;
    float yaw;
    float lastYaw;
    float velocidad;
    float pitch;
    QPixmap Avion;
    float x,y;

};

#endif // GPS_H
