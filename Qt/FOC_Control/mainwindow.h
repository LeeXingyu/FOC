#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QPointF>
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
class QChartView;
class QValueAxis;

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
    void setupUi();
    void setupControlPage(QWidget *page);
    void setupAnglePage(QWidget *page);
    void setupSpeedPage(QWidget *page);
    void setupDebugPage(QWidget *page);
    void appendLog(const QString &text);
    void appendDebug(const QString &text);
    void setConnectedUi(bool connected);
    void processLine(const QString &line);
    void handleTelemetryLine(const QString &line);
    void updateAngle(double mechDeg, double appDeg);
    void updateSpeed(double speedRpm, double iqAmp, double idAmp);
    void refreshAngleChart();
    void refreshSpeedChart();
    void zoomAngleWindow(int delta);
    void zoomSpeedWindow(int delta);
    void resetAngleWindow();
    void resetSpeedWindow();
    static bool parseKeyValueFloat(const QString &line, const QString &key, double *value);

    QTimer m_refreshTimer;
    QTimer m_pollTimer;
    SerialBackend m_serial;

    QTabWidget *m_tabs = nullptr;
    QWidget *m_controlPage = nullptr;
    QWidget *m_anglePage = nullptr;
    QWidget *m_speedPage = nullptr;
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
    QLineSeries *m_speedIqSeries = nullptr;
    QLineSeries *m_speedIdSeries = nullptr;
    QValueAxis *m_speedAxisX = nullptr;
    QValueAxis *m_speedAxisY = nullptr;
    QValueAxis *m_speedAxisIq = nullptr;
    QLabel *m_statusSpeed = nullptr;
    QLabel *m_speedLink = nullptr;
    QLabel *m_speedDetailLabel = nullptr;
    QSlider *m_speedTimeSlider = nullptr;
    QToolButton *m_speedZoomInButton = nullptr;
    QToolButton *m_speedZoomOutButton = nullptr;
    QToolButton *m_speedZoomResetButton = nullptr;
    bool m_speedSliderDragging = false;
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
};

#endif // MAINWINDOW_H
