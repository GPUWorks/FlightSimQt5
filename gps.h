#ifndef GPS_H
#define GPS_H

#include <QWidget>

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
    Ui::GPS *ui;
    float yaw;
    float velocidad;
    float pitch;

};

#endif // GPS_H
