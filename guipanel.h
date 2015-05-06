#ifndef GUIPANEL_H
#define GUIPANEL_H

// Cabecera de la clase de clase GUIPANEL.

#include <QWidget>
#include <QSerialPort> // Ahora QSerialPort esta integrado en Qt5.3
#include <qwt_dial_needle.h>
#include <qwt_analog_clock.h>
#include <QTime>


namespace Ui {
class GUIPanel;
}

class GUIPanel : public QWidget
{
    Q_OBJECT

public:
    //GUIPanel(QWidget *parent = 0);
    explicit GUIPanel(QWidget *parent = 0);
    ~GUIPanel(); // Da problemas

// Estas funciones las añade automaticamente QTCreator
private slots:
    void on_pingButton_clicked();
    void on_runButton_clicked();
    void readRequest();
    void on_serialPortComboBox_currentIndexChanged(const QString &arg1);
    void on_statusButton_clicked();
    void updateFlightTime();

    void on_refreshButton_clicked();
    void on_speedSlider_sliderReleased();

private: // funciones privadas. Las debe añadir el programador
    void pingDevice();
    void startSlave();
    void refreshPorts();
    void processError(const QString &s);
    void activateRunButton();
    void disableWidgets();
    void enableWidgets();
    void initWidgets();
    void initClock();

private:  // Variables privadas; excepto ui, las debe añadir el programador
    Ui::GUIPanel *ui;
    int transactionCount;
    bool connected;
    QSerialPort serial;
    QByteArray request;
    QTime flightTime;
    QTimer *autoPilot; // Para crear un Timer que controle el parpadeo del indicador de autopiloto habilitado

};

#endif // GUIPANEL_H
