#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QPointF>
#include <QtCharts/QChartView>
#include <QTimer>
#include <QVector>

#include "serial_backend.h"

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QLineEdit;
class QLineSeries;
class QSlider;
class QPushButton;
class QSpinBox;
class QToolButton;
class QTextEdit;
class QPlainTextEdit;
class QTabWidget;
class QChart;
class QValueAxis;
class QMouseEvent;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow() override;

private slots:
    void refreshPorts();
    void openClosePort();
    void sendData();
    void clearLog();
    void sendPresetCommand(const QString &text);
    void pollSerial();

private:
    class ChartView : public QChartView
    {
    public:
        explicit ChartView(QWidget *parent = nullptr);

    protected:
        void mousePressEvent(QMouseEvent *event) override;
        void mouseMoveEvent(QMouseEvent *event) override;
    };

    void setupUi();
    void setupControlPage(QWidget *page);
    void setupAnglePage(QWidget *page);
    void setupSpeedPage(QWidget *page);
    void setupIqRefPage(QWidget *page);
    void setupSpeedMeasPage(QWidget *page);
    void setupDebugPage(QWidget *page);
    void appendLog(const QString &text);
    void appendDebug(const QString &text);
    static void setLabelTextIfChanged(QLabel *label, const QString &text);
    void setConnectedUi(bool connected);
    void processLine(const QString &line);
    void handleTelemetryLine(const QString &line);
    void updateAngle(double mechDeg, double appDeg);
    void updateSpeed(double speedRpm);
    void refreshAngleChart();
    void refreshSpeedChart();
    void refreshIqRefChart();
    void refreshSpeedMeasChart();
    void zoomAngleWindow(int delta);
    void zoomSpeedWindow(int delta);
    void zoomIqRefWindow(int delta);
    void zoomSpeedMeasWindow(int delta);
    void resetAngleWindow();
    void resetSpeedWindow();
    void resetIqRefWindow();
    void resetSpeedMeasWindow();
    QTimer m_refreshTimer;
    QTimer m_pollTimer;
    SerialBackend m_serial;

    QTabWidget *m_tabs = nullptr;
    QWidget *m_controlPage = nullptr;
    QWidget *m_anglePage = nullptr;
    QWidget *m_speedPage = nullptr;
    QWidget *m_iqRefPage = nullptr;
    QWidget *m_speedMeasPage = nullptr;
    QWidget *m_debugPage = nullptr;

    QComboBox *m_portCombo = nullptr;
    QComboBox *m_baudCombo = nullptr;
    QComboBox *m_dataBitsCombo = nullptr;
    QComboBox *m_parityCombo = nullptr;
    QComboBox *m_stopBitsCombo = nullptr;
    QComboBox *m_flowCombo = nullptr;
    QCheckBox *m_hexSendCheck = nullptr;
    QCheckBox *m_hexViewCheck = nullptr;
    QCheckBox *m_autoScrollCheck = nullptr;
    QCheckBox *m_autoTxNewlineCheck = nullptr;
    QLineEdit *m_sendEdit = nullptr;
    QPushButton *m_openButton = nullptr;
    QPushButton *m_refreshButton = nullptr;
    QPushButton *m_sendButton = nullptr;
    QPushButton *m_startButton = nullptr;
    QPushButton *m_stopButton = nullptr;
    QPushButton *m_speedModeButton = nullptr;
    QPushButton *m_posModeButton = nullptr;
    QPushButton *m_openLoopButton = nullptr;
    QPushButton *m_setSpeedButton = nullptr;
    QPushButton *m_setKpButton = nullptr;
    QPushButton *m_setKiButton = nullptr;
    QPushButton *m_getIdButton = nullptr;
    QPushButton *m_setIdButton = nullptr;
    QPushButton *m_startCalibButton = nullptr;
    QPushButton *m_stopCalibButton = nullptr;
    QPushButton *m_readFlashButton = nullptr;
    QPushButton *m_clearFlashButton = nullptr;
    QDoubleSpinBox *m_speedSpin = nullptr;
    QDoubleSpinBox *m_kpSpin = nullptr;
    QDoubleSpinBox *m_kiSpin = nullptr;
    QSpinBox *m_nodeIdSpin = nullptr;
    QSpinBox *m_calibModeSpin = nullptr;
    QTextEdit *m_logEdit = nullptr;
    QPlainTextEdit *m_debugEdit = nullptr;
    QPushButton *m_debugClearButton = nullptr;
    QString m_lastLogText;
    QString m_lastDebugText;

    QChartView *m_angleChartView = nullptr;
    QLineSeries *m_angleMechSeries = nullptr;
    QLineSeries *m_angleAppSeries = nullptr;
    QValueAxis *m_angleAxisX = nullptr;
    QValueAxis *m_angleAxisY = nullptr;
    QLabel *m_statusAngle = nullptr;
    QLabel *m_statusAngleLink = nullptr;
    QLabel *m_angleDetailLabel = nullptr;
    QSlider *m_angleTimeSlider = nullptr;
    QToolButton *m_angleZoomInButton = nullptr;
    QToolButton *m_angleZoomOutButton = nullptr;
    QToolButton *m_angleZoomResetButton = nullptr;
    bool m_angleSliderDragging = false;

    QChartView *m_speedChartView = nullptr;
    QLineSeries *m_speedSeries = nullptr;
    QValueAxis *m_speedAxisX = nullptr;
    QValueAxis *m_speedAxisY = nullptr;
    QLabel *m_statusSpeed = nullptr;
    QLabel *m_speedLink = nullptr;
    QSlider *m_speedTimeSlider = nullptr;
    QToolButton *m_speedZoomInButton = nullptr;
    QToolButton *m_speedZoomOutButton = nullptr;
    QToolButton *m_speedZoomResetButton = nullptr;
    bool m_speedSliderDragging = false;

    QChartView *m_iqRefChartView = nullptr;
    QLineSeries *m_iqRawSeries = nullptr;
    QLineSeries *m_iqRefSeries = nullptr;
    QLineSeries *m_iqIdSeries = nullptr;
    QValueAxis *m_iqRefAxisX = nullptr;
    QValueAxis *m_iqRefAxisY = nullptr;
    QLabel *m_iqRefStatus = nullptr;
    QLabel *m_iqRefLink = nullptr;
    QLabel *m_iqRefDetailLabel = nullptr;
    QLabel *m_iqRefPeakLabel = nullptr;
    QLabel *m_iqRawStatus = nullptr;
    QLabel *m_iqRawLink = nullptr;
    QLabel *m_iqIdStatus = nullptr;
    QLabel *m_iqIdLink = nullptr;
    QLabel *m_iqIdDetailLabel = nullptr;
    QLabel *m_iqIdPeakLabel = nullptr;
    QSlider *m_iqRefTimeSlider = nullptr;
    QToolButton *m_iqRefZoomInButton = nullptr;
    QToolButton *m_iqRefZoomOutButton = nullptr;
    QToolButton *m_iqRefZoomResetButton = nullptr;
    bool m_iqRefSliderDragging = false;
    QVector<QPointF> m_iqRawPoints;
    QVector<QPointF> m_iqRefPoints;
    QVector<QPointF> m_iqIdPoints;
    qint64 m_iqRawSampleIndex = 0;
    qint64 m_iqRefSampleIndex = 0;
    qint64 m_iqIdSampleIndex = 0;
    int m_iqRefWindowSize = 1800;
    bool m_iqRefAutoFollow = true;
    double m_lastIqRawAmp = 0.0;
    double m_lastIqRefAmp = 0.0;
    qint64 m_lastIqRawUiTick = 0;
    qint64 m_lastIqRefUiTick = 0;

    QChartView *m_speedMeasChartView = nullptr;
    QLineSeries *m_speedMeasSeries = nullptr;
    QLineSeries *m_speedErrSeries = nullptr;
    QValueAxis *m_speedMeasAxisX = nullptr;
    QValueAxis *m_speedMeasAxisY = nullptr;
    QValueAxis *m_speedErrAxis = nullptr;
    QLabel *m_speedMeasStatus = nullptr;
    QLabel *m_speedMeasLink = nullptr;
    QLabel *m_speedMeasDetailLabel = nullptr;
    QLabel *m_speedMeasPeakLabel = nullptr;
    QSlider *m_speedMeasTimeSlider = nullptr;
    QToolButton *m_speedMeasZoomInButton = nullptr;
    QToolButton *m_speedMeasZoomOutButton = nullptr;
    QToolButton *m_speedMeasZoomResetButton = nullptr;
    bool m_speedMeasSliderDragging = false;
    QVector<QPointF> m_speedMeasPoints;
    QVector<QPointF> m_speedErrPoints;
    qint64 m_speedMeasSampleIndex = 0;
    int m_speedMeasWindowSize = 1800;
    bool m_speedMeasAutoFollow = true;
    double m_lastSpeedMeasPu = 0.0;
    double m_lastSpeedErrorPu = 0.0;
    qint64 m_lastSpeedMeasUiTick = 0;

    QVector<QPointF> m_speedPoints;
    QVector<QPointF> m_speedIqPoints;
    QVector<QPointF> m_speedIdPoints;
    qint64 m_speedSampleIndex = 0;
    int m_speedWindowSize = 1800;
    bool m_angleAutoFollow = true;
    bool m_speedAutoFollow = true;
    double m_lastSpeedRpm = 0.0;
    double m_lastIqAmp = 0.0;
    double m_lastIdAmp = 0.0;
    qint64 m_lastAngleUiTick = 0;
    qint64 m_lastSpeedUiTick = 0;

    QLabel *m_statusLink = nullptr;
    QVector<QPointF> m_angleMechPoints;
    QVector<QPointF> m_angleAppPoints;
    qint64 m_sampleIndex = 0;
    qint64 m_lastTelemetryTick = 0;
    int m_angleWindowSize = 1800;
    QString m_rxLineBuffer;
    QPointF m_lastMousePos;
};

#endif // MAINWINDOW_H
