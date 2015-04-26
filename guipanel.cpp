// Clase GUIPANEL implementa las funciones a ejecutar cuando se realizan acciones en los componentes (widgets)
// de su interfaz gráfico asociado.

#include "guipanel.h"     // Cabecera de la clase
#include "ui_guipanel.h"  // Cabecera de componentes del interfaz gráfico
#include <QSerialPort>    // Ahora QSerialPort esta integrado en Qt5.3
#include <QSerialPortInfo>    // Ahora QSerialPort esta integrado en Qt5.3
#include <QMessageBox>    // Se deben incluir cabeceras a los componentes que se vayan a crear en la clase
// y que no estén incluidos en el interfaz gráfico. En este caso, la ventana de PopUp
// que se muestra al recibir un PING de respuesta


#include <QPainter>       
#include <QTimer>

#if QT_VERSION < 0x040000
#include <QColorGroup>    
typedef QColorGroup Palette;
#else
typedef QPalette Palette;
#endif

extern "C" {
#include "protocol.h"    // Cabecera de funciones de gestión de tramas; se indica que está en C, ya que QTs
// se integra en C++, y el C puede dar problemas si no se indica.
}

GUIPanel::GUIPanel(QWidget *parent) :  // Inicializacion de variables
    QWidget(parent),
    ui(new Ui::GUIPanel)               // Indica que guipanel.ui es el interfaz grafico de la clase
  , transactionCount(0)
{
    ui->setupUi(this);                // Conecta la clase con su interfaz gráfico.
    setWindowTitle(tr("Simulador de Vuelo")); // Título de la ventana

    connected=false;                 // Todavía no hemos establecido la conexión USB
    refreshPorts();                  // Examina los puertos disponibles a ver si hay una conexion USB a la TIVA
    // Nótese que todos los componentes del interfaz gráfico se acceden como "ui->nombreComponente"
    ui->serialPortComboBox->setFocus();   // Para que esta sea la ventana activa de inicio, en caso
    // de que haya varias

    // Las funciones CONNECT son la base del funcionamiento de QT; conectan dos componentes
    // o elementos del sistema; uno que GENERA UNA SEÑAL; y otro que EJECUTA UNA FUNCION (SLOT) al recibir dicha señal.
    // En el ejemplo se conecta la señal readyRead(), que envía el componente que controla el puerto USB serie (serial),
    // con la propia clase PanelGUI, para que ejecute su funcion readRequest() en respuesta.
    // De esa forma, en cuanto el puerto serie esté preparado para leer, se lanza una petición de datos por el
    // puerto serie.El envío de readyRead por parte de "serial" es automatico, sin necesidad de instrucciones
    // del programador
    connect(&serial, SIGNAL(readyRead()), this, SLOT(readRequest()));

    // Inicializa componentes para los controles y mandos de la cabina

    initClock(); // Pinta y configura el componente del reloj de vuelo
    initSpeedometer(); // Pinta y configura el componente de control de velocidad
    initYawCompass();  // Pinta y configura el componente del control de direccion (Yaw)
    initRollCompass(); // Pinta y configura el componente del control de alabeo (Roll)
    initPitchCompass(); // Pinta y configura el componente del control de angulo de ataque (Pitch)

    // Configura otros controles e indicadores del GUI
    ui->introFuel->setRange(0,100); // Rango de fuel(litros)*SOLO EJEMPLO->QUITAR*
    ui->Fuel->setAlarmBrush(QBrush(Qt::blue)); // Color del indicador de fuel
    ui->Fuel->setFillBrush(QBrush(Qt::blue));  // Color del indicador de fuel (zona de alarma)
    ui->Fuel->setAlarmEnabled(true);           // Para diferenciar entre zona normal y de alarma

    // Deshabilita controles hasta que nos conectemos
    //disableWidgets();  // TODO: Esto lo descomentaremos en la aplicación final, para que los mandos
    // y elementos no esten activos de inicio
    ui->pingButton->setEnabled(false);    // Se deshabilita el botón de ping del interfaz gráfico, hasta que
    // se haya establecido conexión
    // Configuración inicial del indicadores varios
    initWidgets();
}

GUIPanel::~GUIPanel() // Destructor de la clase
{
    delete ui;   // Borra el interfaz gráfico asociado a la clase
}

// Deshabilita los widgets mientras no queramos que funcionen
void GUIPanel::disableWidgets(){
    // Esconde controles de PRUEBA (*QUITAR en la aplicacion -> solo de ejemplo)
    ui->yawDial->setVisible(false);
    ui->rollSlider->setVisible(false);
    ui->pitchSlider->setVisible(false);
    ui->introFuel->setVisible(false);

    // Deshabilita controles e indicadores para que no funcionen hasta que nos conectemos a la TIVA

    ui->d_clock->setEnabled(false);
    ui->Fuel->setEnabled(false);
    ui->speedometer->setEnabled(false);
    ui->speedSlider->setEnabled(false);
    ui->height->setEnabled(false);
    ui->YawCompass->setEnabled(false);
    ui->RollCompass->setEnabled(false);
    ui->PitchCompass->setEnabled(false);
}

// Habilita los widgets para poder utilizarlos
void GUIPanel::enableWidgets(){
    // Muestra controles de PRUEBA (*QUITAR -> solo de ejemplo)
    ui->yawDial->setVisible(true);
    ui->rollSlider->setVisible(true);
    ui->pitchSlider->setVisible(true);
    ui->introFuel->setVisible(true);

    // Habilita controles e indicadores

    ui->d_clock->setEnabled(true);
    ui->Fuel->setEnabled(true);
    ui->speedometer->setEnabled(true);
    ui->speedSlider->setEnabled(true);
    ui->height->setEnabled(true);
    ui->YawCompass->setEnabled(true);
    ui->RollCompass->setEnabled(true);
    ui->PitchCompass->setEnabled(true);
    ui->pingButton->setEnabled(true);

    // No se puede cambiar el puerto hasta detener la aplicación con el boton de STOP
    ui->serialPortComboBox->setEnabled(false);
}

// Estado inicial de los widgets --> estado inicial del avion simulado
void GUIPanel::initWidgets(){
    // Configuración inicial del indicadores varios
    ui->Fuel->setValue(100); // Valor inicial de fuel
    ui->height->setValue(3000); // Valor inicial del altimetro
    ui->brokenGlass->setVisible(false); // No queremos mostrar la cabina rota!!! (por ahora)
    ui->speedometer->setValue(60);       // Velocidad inicial de velocidad en el indicador
    ui->speedSlider->setValue(60);       // Control del velocidad: Quitar en aplicación final

    ui->RollCompass->setValue(90); // Estado inicial del giroscopio de ROLL
    ui->rollSlider->setValue(90);  // Control de ROLL: Quitar en aplicacion final

    ui->YawCompass->setValue(0); // Estado inicial del giroscopo de PITCH
    ui->yawDial->setValue(180);  // Control de Pitch: Quitar en aplicación final

    ui->AutoPilot->setVisible(false); // Etiqueta de "Piloto automatico" no es visible
}


// Funcion que se ocupa de procesar una TRAMA recibida a través  del puerto USB. Esta función deberá ser
// parcialmente modificada por los estudiantes para añadir nuevas respuestas a nuevos comandos de trama
void GUIPanel::readRequest()
{
    int posicion,tam;
    char * frame;          // Puntero a zona de memoria donde reside la trama recibida
    unsigned char command; // Para almacenar el comando de la trama entrante

    request.append(serial.readAll()); // Añade el contenido del puerto serie USB al array de bytes 'request'
    // así vamos acumulando  en el array la información que va llegando

    // Busca la posición del primer byte de fin de trama (0xFD) en el array
    posicion=request.indexOf((char)STOP_FRAME_CHAR,0);
    //Metodo poco eficiente pero seguro...
    while (posicion>0)
    {
        frame=request.data(); // Puntero de trama al inicio del array de bytes
        tam=posicion-1;       //Caracter de inicio y fin no cuentan en el tamaño
        // Descarta posibles bytes erroneos que no se correspondan con el byte de inicio de trama
        while (((*frame)!=(char)START_FRAME_CHAR)&&(tam>0)) // Casting porque Qt va en C++ (en C no hace falta)
        {
            frame++;  // Descarta el caracter erroneo
            tam--;    // como parte de la trama recibida
        }
        // A. Disponemos de una trama encapsulada entre dos caracteres de inicio y fin, con un tamaño 'tam'
        if (tam >  0)
        {   //Paquete aparentemente correcto, se puede procesar
            frame++;  //Quito el byte de arranque (START_FRAME_CHAR, 0xFC)
            //Y proceso normalmente el paquete
            // Paso 1: Destuffing y cálculo del CRC. Si todo va bien, obtengo la trama
            // con valores actualizados y sin bytes de checksum
            tam=destuff_and_check_checksum((unsigned char *)frame,tam);
            // El tamaño se actualiza (he quitado 2bytes de CRC, mas los de "stuffing")
            if (tam>=0)
            {
                //El paquete está bien, luego procedo a tratarlo.
                command=decode_command_type(frame,0); // Obtencion del byte de Comando

                // Ventana popUP para el caso de comando PING; no te deja definirla en un "caso"
                QMessageBox ventanaca(QMessageBox::Information,tr("Evento"),tr(" RESPUESTA A PING RECIBIDA"),QMessageBox::Ok,this,Qt::Popup);
                ventanaca.setStyleSheet("background-color: lightgrey");

                switch(command) // Segun el comando tengo que hacer cosas distintas
                {


                //Por ahora no se han implementado todos los comandos

                /****AQUI ES DONDE LOS ESTUDIANTES DEBEN AÑADIR NUEVAS RESPUESTAS ANTE LOS COMANDOS QUE SE ENVIEN DESDE LA TIVA **/
                case COMANDO_PING:  // Algunos comandos no tiene parametros
                    // Crea una ventana popup con el mensaje indicado
                    //statusLabel->setText(tr("  RESPUESTA A PING RECIBIDA"));
                    ventanaca.exec();
                    break;

                case COMANDO_NO_IMPLEMENTADO:
                {
                    // En otros comandos hay que extraer los parametros de la trama y copiarlos
                    // a una estructura para poder procesar su informacion
                    PARAM_COMANDO_NO_IMPLEMENTADO parametro;
                    extract_packet_command_param(frame,sizeof(parametro),&parametro);
                    // Muestra en una etiqueta (statuslabel) del GUI el mensaje
                    ui->statusLabel->setText(tr("  Comando rechazado,"));
                }
                    break;

                    //Falta por implementar la recepcion de mas tipos de comando

                default:
                    ui->statusLabel->setText(tr("  Recibido paquete inesperado,"));
                    break;
                }
            }
        }
        else
        {
            // B. La trama no está completa... no lo procesa, y de momento no digo nada
            ui->statusLabel->setText(tr(" Fallo trozo paquete recibido"));
        }
        request.remove(0,posicion+1); // Se elimina todo el trozo de información erroneo del array de bytes
        posicion=request.indexOf((char)STOP_FRAME_CHAR,0); // Y se busca el byte de fin de la siguiente trama
    }
}


// Funciones auxiliares y no asociadas al envio de comandos

// Explora los interfaces USB existentes en el sistema y los pone en el componente ComboBox
// del interfaz gráfico
void GUIPanel::refreshPorts(){
    ui->serialPortComboBox->clear();
    foreach (const QSerialPortInfo &info, QSerialPortInfo::availablePorts()){
        // qDebug permite obtener mensajes en la consola de depuración (ver fichero .pro)
        qDebug() << "\nPort:" << info.portName();
        qDebug() << "Location:" << info.systemLocation();
        qDebug() << "Description:" << info.description();
        qDebug() << "Manufacturer:" << info.manufacturer();
        qDebug() << "Vendor Identifier:" << info.vendorIdentifier();
        qDebug() << "Product Identifier:" << info.productIdentifier();
        // La descripción nos permite que SOLO aparezcan los interfaces tipo USB serial
        if((info.description().contains("Virtual COM Port")) || (info.description().contains("TivaWare USB serial port") ))
            ui->serialPortComboBox->addItem(info.portName());
    }
}

// Establecimiento de la comunicación USB serie a través del interfaz seleccionado en la comboBox, tras pulsar el
// botón RUN del interfaz gráfico. Se establece una comunicacion a 9600bps 8N1 y sin control de flujo en el objeto
// 'serial' que es el que gestiona la comunicación USB serie en el interfaz QT
void GUIPanel::startSlave()
{
    if (serial.portName() != ui->serialPortComboBox->currentText() || ui->serialPortComboBox->count()==0) {
        serial.close();
        serial.setPortName(ui->serialPortComboBox->currentText());

        if (!serial.open(QIODevice::ReadWrite)) {
            processError(tr("No puedo abrir el puerto %1, error code %2")
                         .arg(serial.portName()).arg(serial.error()));
            return;
        }

        if (!serial.setBaudRate(9600)) {
            processError(tr("No puedo establecer tasa de 9600bps en el puerto %1, error code %2")
                         .arg(serial.portName()).arg(serial.error()));
            return;
        }

        if (!serial.setDataBits(QSerialPort::Data8)) {
            processError(tr("No puedo establecer 8bits de datos en el puerto %1, error code %2")
                         .arg(serial.portName()).arg(serial.error()));
            return;
        }

        if (!serial.setParity(QSerialPort::NoParity)) {
            processError(tr("NO puedo establecer parida en el puerto %1, error code %2")
                         .arg(serial.portName()).arg(serial.error()));
            return;
        }

        if (!serial.setStopBits(QSerialPort::OneStop)) {
            processError(tr("No puedo establecer 1bitStop en el puerto %1, error code %2")
                         .arg(serial.portName()).arg(serial.error()));
            return;
        }

        if (!serial.setFlowControl(QSerialPort::NoFlowControl)) {
            processError(tr("No puedo establecer el control de flujo en el puerto %1, error code %2")
                         .arg(serial.portName()).arg(serial.error()));
            return;
        }
    }

    // Si la conexión se realiza, se cambia la imagen del boton de Start a una imagen de Stop
    ui->runButton->setIcon(QIcon(QPixmap(":/img/EngineButtonStop.png")));

    // Se indica que se ha realizado la conexión en la etiqueta 'statusLabel'
    ui->statusLabel->setText(tr("  Ejecucion, conectado al puerto %1.")
                             .arg(ui->serialPortComboBox->currentText()));

    // Y se habilitan los controles
    enableWidgets();

    // Variable indicadora de conexión a TRUE, para que se permita enviar comandos en respuesta
    // a eventos del interfaz gráfico
    connected=true;
}

// Funcion auxiliar de procesamiento de errores de comunicación (usada por startSlave)
void GUIPanel::processError(const QString &s)
{
    activateRunButton(); // Activa el botón RUN
    refreshPorts();
    // Muestra en la etiqueta de estado la razón del error (notese la forma de pasar argumentos a la cadena de texto)
    ui->statusLabel->setText(tr("  Detenido, %1.").arg(s));
}


// Funcion de habilitacion del boton de inicio/conexion
void GUIPanel::activateRunButton()
{
    ui->runButton->setEnabled(true);
    ui->runButton->setIcon(QIcon(QPixmap(":/img/EngineButtonStart.png")));
}


// Funciones SLOT que se crean automaticamente desde QTDesigner al activar una señal de un WIdget del interfaz gráfico
// Se suelen asociar a funciones auxiliares, en muchos caso, por comodidad.

// SLOT asociada a modificación de la comboBox
void GUIPanel::on_serialPortComboBox_currentIndexChanged(const QString &arg1)
{
    activateRunButton();  // Llama a funcion de habilitacion del boton RUN
}

// SLOT asociada a pulsación del botón RUN
void GUIPanel::on_runButton_clicked(){
    if(!connected){
        // Si esta con el icono Start
        startSlave(); // Aquí podriamos haber escrito todo el codigo de 'startSlave' en vez de llamarla.
        if (connected){ // Para que no se intenten enviar datos si la conexion USB no esta activa
            ui->brokenGlass->setVisible(false); // Quita la imagen de "cabina rota"
            // Crear una trama con el comando COMANDO_START, para iniciar el funcionamiento de la
            // aplicación --> TO_DO
        }
    }else{ // Si esta con el icono Stop
        // Crear una trama con el comando COMANDO_STOP, para detener el funcionamiento de la
        // aplicación  --> TO_DO
        ui->statusLabel->setText(tr("  Detenido: sin conexion USB") );
        activateRunButton();
        ui->serialPortComboBox->setEnabled(true);
        connected = false;
        disableWidgets();
        initWidgets();
    }
}

// SLOT asociada a pulsación del botón PING
void GUIPanel::on_pingButton_clicked()
{
    pingDevice();
}

// SLOT asociada al borrado del mensaje de estado al pulsar el boton
void GUIPanel::on_statusButton_clicked()
{
    ui->statusLabel->setText(tr(""));
}

// Funciones de usuario asociadas a la ejecucion de comandos. La estructura va a ser muy parecida en casi todos los
// casos. Se va a crear una trama de un tamaño maximo (100), y se le van a introducir los elementos de
// num_secuencia, comando, y parametros.

// Envío de un comando PING
void GUIPanel::pingDevice()
{
    char paquete[MAX_FRAME_SIZE];
    int size;

    if (connected) // Para que no se intenten enviar datos si la conexion USB no esta activa
    {
        // El comando PING no necesita parametros; de ahí el NULL, y el 0 final.
        // No vamos a usar el mecanismo de numeracion de tramas; pasamos un 0 como n de trama
        size=create_frame((unsigned char *)paquete, COMANDO_PING, NULL, 0, MAX_FRAME_SIZE);
        // Si la trama se creó correctamente, se escribe el paquete por el puerto serie USB
        if (size>0) serial.write(paquete,size);
    }
}

// Función de inicializacion y configuracion del reloj de  a bordo:
void GUIPanel::initClock(){

    // Esto de aqui abajo es para cambiar las agujas "originales" de hora,minuto, y segundo
    const QColor knobColor = QColor(Qt::gray).light(130);
    for ( int i = 0; i < QwtAnalogClock::NHands; i++)
    {
        QColor handColor = QColor(Qt::gray).light(150);
        int width = 5;

        if ( i == QwtAnalogClock::SecondHand )
        {
            handColor = Qt::gray;
            width = 1;
        }
        QwtDialSimpleNeedle *hand = new QwtDialSimpleNeedle(
                    QwtDialSimpleNeedle::Arrow, true, handColor, knobColor);
        hand->setWidth(width);
        ui->d_clock->scaleDraw()->setSpacing(2);
        ui->d_clock->setHand((QwtAnalogClock::Hand)i, hand);
    }
    // No quiero que se muestre la aguja del segundero
    ui->d_clock->setHand(QwtAnalogClock::SecondHand, NULL);

    // Tamaño de las marcas del reloj (4 pixel)
    ui->d_clock->scaleDraw()->setTickLength( QwtScaleDiv::MajorTick, 4 );
    ui->d_clock->scaleDraw()->setSpacing(2); // Distancia marcas del borde exterior
    // de la esfera


    flightTime = QTime::currentTime(); // Se inicia en principio con la hora del sistema
    ui->d_clock->setTime(flightTime);    // Se sincroniza el reloj gráfico con el objeto Qtime

    // CONTROL POR TIMER QT --> BORRAR EN VERSION FINAL PARA CONTROL POR COMANDO DE TIVA
    QTimer *timer = new QTimer(ui->d_clock);
    // Reloj de a bordo del sistema
    timer->connect(timer, SIGNAL(timeout()),
                   this, SLOT(updateFlightTime()));
    timer->start(1000);
}

//* CAMBIAR PARA QUE CADA SEGUNDO DE TIEMPO REAL SE MUESTRE COMO UN MINUTO EN EL RELOJ
// DE A BORDO Y PARA QUE SEA LA TIVA LA QUE ACTUALICE EL RELOJ, Y NO UN TIMER QT  *//
void GUIPanel::updateFlightTime(){
    flightTime = flightTime.addSecs(1); // flightTime.addSecs(60);
    ui->d_clock->setTime(flightTime);
}


// Funcion que configura e inicializa la esfera de control de velocidad
void GUIPanel::initSpeedometer(){

    // Aspecto de la escala
    QwtRoundScaleDraw *scaleDraw = new QwtRoundScaleDraw();
    scaleDraw->setSpacing(2);
    scaleDraw->setTickLength( QwtScaleDiv::MinorTick, 0 );
    scaleDraw->setTickLength( QwtScaleDiv::MediumTick, 4 );
    scaleDraw->setTickLength( QwtScaleDiv::MajorTick, 8 );
    scaleDraw->enableComponent(QwtAbstractScaleDraw::Backbone, true);
    scaleDraw->enableComponent(QwtAbstractScaleDraw::Labels, true);
    ui->speedometer->setScaleDraw(scaleDraw);
    ui->speedometer->setWrapping(false);  // La aguja no puede superar los valores
    // maximo o minimo
    ui->speedometer->setReadOnly(true); // La aguja no se puede mover con el raton

    // Para que no dibuje la esfera completa, sino un arco de 270º centrado en la mitad
    ui->speedometer->setOrigin(135);
    ui->speedometer->setScaleArc(0.0, 270.0);

    // Configuracion de la aguja del instrumento, y de los ticks
    QwtDialSimpleNeedle *needle = new QwtDialSimpleNeedle(
                QwtDialSimpleNeedle::Arrow, true, Qt::red,
                QColor(Qt::gray).light(130));
    ui->speedometer->setNeedle(needle);
    ui->speedometer->setScale(0.0, 240.0); // Rango de valores del instrumento
    ui->speedometer->setScaleMaxMajor(12); // Numero max de marcas de etiqueta "grandes"
    ui->speedometer->setScaleMaxMinor(5); // Numero max de marcas de etiqueta "pequeñas"
    ui->speedometer->scaleDraw()->setSpacing(2); // Distancia ticks del borde exterior
    // de la esfera y las etiquetas

    // Palanca de control para modificar la velocidad
    // Si se decide controlar la velocidad con un potenciometro desde la TIVA,
    // quitar este control y realizar su labor en los comandos recibidos de TIVA
    // Una mayoría de estos cambios se pueden hacer DIRECTAMENTE en QTCreator
    ui->speedSlider->setTrough(true); // La escala tiene "Muescas"*/
    // Etiquetas y ticks del control
    ui->speedSlider->setUpperBound(240); // Valor máximo de etiqueta del slider
    ui->speedSlider->setLowerBound(0);   // Valor mínimo de etiqueta del slider
    ui->speedSlider->setScaleMaxMajor(6); // Número max de marcas de etiqueta "grandes"
    ui->speedSlider->setScaleMaxMinor(3); // Número max de marcas de etiqueta "pequeñas"
    ui->speedSlider->setScaleStepSize(60); // Cada cuanto hay una marca

    ui->speedSlider->setTotalSteps(120); // Cada vez que muevo el slider incremento en 2 (240/120)

    // No es realista que los cambios de velocidad sean "instantaneos";por ello, al
    // establecer una velocidad, usamos un Timer para ir alcanzando poco a poco la
    // velocidad fijada, usando el "slot" "changeSpeed".
    // TIMER para llamar a la función changeSpeed cada 50ms
    QTimer *timer = new QTimer(ui->speedometer);
    timer->connect(timer, SIGNAL(timeout()),
                   this, SLOT(changeSpeed()));
    timer->start(50);

}

// Ejemplo de SLOT que aumenta y disminuye paulatinamente la velocidad
// Se llama (por parte de un TIMER) cada 50ms 
void GUIPanel::changeSpeed() // Slider Controlled
{
    double thrust;
    static double posOffset = 2;
    static double negOffset = 4;
    if(ui->Fuel->value() > 0 )
        thrust = ui->speedSlider->value();
    else
        thrust = 0;
    double speed = ui->speedometer->value();
    if(speed != thrust){
        if ( (speed < thrust))
            ui->speedometer->setValue(speed + posOffset);
        else if (speed > thrust)
        {
            ui->speedometer->setValue(speed - negOffset);
        }
    }
}

// Configuracion e inicialización de la esfera de "dirección" (YAW) del avion
void GUIPanel::initYawCompass(){
    // TODO: hacer usando como ejemplo las configuraciones de las esferas PITCH y ROLL y el
    // dibujo del aspecto buscado en el control
    ui->YawCompass->setMode(QwtDial::RotateScale); // Modo rotacion escala
    //ui->YawCompass->setMode(QwtDial::RotateNeedle); // Modo rotacion aguja
    // Aguja tipo "brujula"
    ui->YawCompass->setNeedle(new QwtCompassMagnetNeedle(QwtCompassMagnetNeedle::ThinStyle));
    ui->YawCompass->setValue(0); // Aguja hacia al N al inicio
}


// Configuración e inicialización de la esfera de "alabeo" (ROLL)
void GUIPanel::initRollCompass(){

    // Color del fondo de la esfera
    Palette colorGroup;
    colorGroup.setColor(Palette::Base, QColor(Qt::darkGray).dark(180));
    colorGroup.setColor(Palette::Foreground,QColor(Qt::darkGray).dark(280));
    colorGroup.setColor(Palette::Text, Qt::white);
    ui->RollCompass->setPalette(colorGroup);

    // Aguja tipo "brujula"
    ui->RollCompass->setNeedle(new QwtCompassMagnetNeedle(QwtCompassMagnetNeedle::ThinStyle));
    ui->RollCompass->setMode(QwtDial::RotateNeedle); // Lo que gira es la aguja

    // Escala para la esfera
    // Etiquetas y ticks/marcas
    QMap<double, QString> map; // Indicamos que etiquetas queremos que salgan
    map.insert(110, "R");      // y donde
    map.insert(250, "L");
    QwtCompassScaleDraw *rollScDraw = new QwtCompassScaleDraw(map);
    // Longitud de las marcas
    rollScDraw->setTickLength(QwtScaleDiv::MinorTick,0);  // Solo queremos
    rollScDraw->setTickLength(QwtScaleDiv::MediumTick,0); // que aparezcan las
    rollScDraw->setTickLength(QwtScaleDiv::MajorTick,10);  // marcas grandes
    // Establecemos donde irán los Ticks (0 esta al "Norte" de la "brujula"
    QList< double > ticks;
    ticks.append(70.0);
    ticks.append(90.0);
    ticks.append(110.0);
    ticks.append(250.0);
    ticks.append(270.0);
    ticks.append(291.0); // no 290, porque si no, no sale..(no se porqué)

    // Divisiones de la escala
    QwtScaleDiv myScDiv=rollScDraw->scaleDiv(); // Trabajamos con la escala

    myScDiv.setInterval(QwtInterval(0,360)); // Rango 0-360º
    // Solo aparecen ticks en la escala "mayor" (el resto está vacío)
    myScDiv.setTicks(QwtScaleDiv::MinorTick,QList<double>());
    myScDiv.setTicks(QwtScaleDiv::MediumTick,QList<double>());
    myScDiv.setTicks(QwtScaleDiv::MajorTick,ticks);
    // Indicamos que no aparezca el "circulo interior" pero si, las etiquetas
    // y las marcas
    rollScDraw->enableComponent(QwtAbstractScaleDraw::Labels,true);
    rollScDraw->enableComponent(QwtAbstractScaleDraw::Ticks,true);
    rollScDraw->enableComponent(QwtAbstractScaleDraw::Backbone,false);
    rollScDraw->setSpacing(10); // Distancia al centro de la corona de ticks
    // Asignamos el componente de escala al compas
    ui->RollCompass->setScaleDraw(rollScDraw);
    // .. y el componente de divisiones a la escala
    rollScDraw->setScaleDiv(myScDiv); //(si se hace en otro orden NO funciona)

    ui->RollCompass->setValue(90); // Posicion inicial (0 es el Norte)
    ui->RollCompass->setWrapping(false); // La aguja no puede superar los valores máximo o minimo

    //Esto de abajo permite que la etiqueta salga de color blanco
    ui->NoPitchLabel->setText("<font color='white'>No Pitch</font>");
}

// Configuracion e inicialización de la esfera de "angulo de ataque" (PITCH) del avion

void GUIPanel::initPitchCompass(){
    // Color del fondo de la esfera
    Palette colorGroup;
    colorGroup.setColor(Palette::Base, Qt::darkBlue);
    colorGroup.setColor(Palette::Foreground,
                        QColor(Qt::darkBlue).dark(120));
    colorGroup.setColor(Palette::Text, Qt::white);
    ui->PitchCompass->setPalette(colorGroup);
    ui->PitchCompass->setMode(QwtDial::RotateNeedle); // Modo rotacion escala

    // Aguja tipo "rayo" // COMENTADO PARA USAR ROTACION DE PIXMAP
    ui->PitchCompass->setNeedle(new QwtDialSimpleNeedle(QwtDialSimpleNeedle::Arrow));
    ui->PitchCompass->setValue(90); // Aguja hacia al E al inicio
    ui->pitchSlider->setMinimum(45);
    ui->pitchSlider->setMaximum(135);

    // Escala para la esfera
    // Etiquetas y ticks/marcas
    QMap<double, QString> map; // Indicamos que etiquetas queremos que salgan
    QString label;
    label.sprintf("45");
    map.insert(45, label);
    label.sprintf("0");
    map.insert(90, label);
    label.sprintf("-45");
    map.insert(135, label);
    QwtCompassScaleDraw *pitchScDraw = new QwtCompassScaleDraw(map);

    // Longitud de las marcas
    pitchScDraw ->setTickLength(QwtScaleDiv::MinorTick,0);  // Solo queremos
    pitchScDraw->setTickLength(QwtScaleDiv::MediumTick,0); // que aparezcan las
    pitchScDraw ->setTickLength(QwtScaleDiv::MajorTick,10);  // marcas grandes
    // Establecemos donde irán los Ticks (0 esta al "Norte" de la "brujula"
    QList< double > ticks;
    ticks.append(45.0);
    ticks.append(67.0);
    ticks.append(90.0);
    ticks.append(112.0);
    ticks.append(135.0);

    // Divisiones de la escala
    QwtScaleDiv myScDiv=pitchScDraw->scaleDiv(); // Trabajamos con la escala

    myScDiv.setInterval(QwtInterval(0,360)); // Rango 0-360º
    // Solo aparecen ticks en la escala "mayor" (el resto está vacío)
    myScDiv.setTicks(QwtScaleDiv::MinorTick,QList<double>());
    myScDiv.setTicks(QwtScaleDiv::MediumTick,QList<double>());
    myScDiv.setTicks(QwtScaleDiv::MajorTick,ticks);
    // Indicamos que aparezca el "circulo interior",etiquetas y  marcas
    pitchScDraw->enableComponent(QwtAbstractScaleDraw::Labels,true);
    pitchScDraw->enableComponent(QwtAbstractScaleDraw::Ticks,true);
    pitchScDraw->enableComponent(QwtAbstractScaleDraw::Backbone,true);
    pitchScDraw->setSpacing(8); // Distancia al centro de la corona de ticks
    // Asignamos el componente de escala al compas
    ui->PitchCompass->setScaleDraw(pitchScDraw);
    // .. y el componente de divisiones a la escala
    pitchScDraw->setScaleDiv(myScDiv); //(si se hace en otro orden NO funciona)

    ui->PitchCompass->setReadOnly(true);
    ui->PitchCompass->setWrapping(false); // La aguja no puede superar los valores máximo o minimo
}

// Slot asociado al control de estado de gasolina -> en la aplicación final esto debe
// asociarse al comando recibido de TIVA actualizando el estado de la gasolina
void GUIPanel::on_introFuel_valueChanged(int arg1)
{
    ui->Fuel->setValue(arg1);
    if(arg1<20)
        ui->Fuel->setFillBrush(QBrush(Qt::red));
    else
        ui->Fuel->setFillBrush(QBrush(Qt::blue));
}

// Slot asociada a la variacion de "direccion" (YAW) --> en la aplicación final esto debe
// asociarse al comando recibido de TIVA actualizando el estado de dirección, en funcion del
// potenciometro asociado
void GUIPanel::on_yawDial_valueChanged(int value)
{
    // En el control por el dial, se le suma 180, porque el 0 esta en el N en la esfera, y en el
    // S en el dial de control.
    if(value < 180)
        ui->YawCompass->setValue(value + 180);
    else
        ui->YawCompass->setValue(value-180);
}

// Slot asociada a la variacion de "alabeo" (ROLL) --> en la aplicación final esto debe
// asociarse al comando recibido de TIVA actualizando el estado de alabeo, en funcion del
// potenciometro asociado
void GUIPanel::on_rollSlider_valueChanged(int value)
{
    ui->RollCompass->setValue((double)value);
}

// Slot asociado al boton de refresco, por si conectamos la TIVA despues de poner en marcha el GUI
void GUIPanel::on_refreshButton_clicked()
{
    refreshPorts();
}

// Slot asociada a la variacion de "ataque" (pitch) --> en la aplicación final esto debe
// asociarse al comando recibido de TIVA actualizando el estado de ataque, en funcion del
// potenciometro asociado
void GUIPanel::on_pitchSlider_valueChanged(int value)
{
    ui->PitchCompass->setValue((double)value);
}

// Slot asociada a cambios en el estado de la palanca de velocidad. Además de afectar a la
// esfera correspondiente, se deberá enviar un comando COMANDO_SPEED a la TIVA, con indicaciones
// de la velocidad, ya que este valor se usará allí
void GUIPanel::on_speedSlider_sliderReleased()
{
    // TODO: Creacion de un paquete de datos tipo COMANDO_SPEED, en el que se envíe una
    // indicacion (valor absoluto, porcentaje,...) de la velocidad actual
    if (connected) // Para que no se intenten enviar datos si la conexion USB no esta activa
    {
        // TODO
    }
}
