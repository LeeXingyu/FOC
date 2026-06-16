#include "mainwindow.h"

#include <QApplication>
#include <QByteArray>
#include <QChart>
#include <QChartView>
#include <QCheckBox>
#include <QComboBox>
#include <QDateTime>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QFrame>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QLineSeries>
#include <QPainter>
#include <QPointF>
#include <QPushButton>
#include <QPlainTextEdit>
#include <QRegularExpression>
#include <QScrollBar>
#include <QSlider>
#include <QSpinBox>
#include <QStatusBar>
#include <QTabWidget>
#include <QTextEdit>
#include <QToolButton>
#include <QVBoxLayout>
#include <QValueAxis>

namespace
{
QString fmt6(double v) { return QString::number(v, 'f', 6); }
}

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    setupUi();

    connect(&m_refreshTimer, &QTimer::timeout, this, &MainWindow::refreshPorts);
    connect(&m_pollTimer, &QTimer::timeout, this, &MainWindow::pollSerial);
    connect(&m_pollTimer, &QTimer::timeout, this, [this]() {
        if (m_statusLink == nullptr)
        {
            return;
        }
        const qint64 now = QDateTime::currentMSecsSinceEpoch();
        if ((m_lastTelemetryTick != 0) && ((now - m_lastTelemetryTick) > 1000))
        {
            m_statusLink->setText(tr("Waiting telemetry..."));
        }
    });

    m_refreshTimer.start(1500);
    m_pollTimer.start(20);
    refreshPorts();
    setConnectedUi(false);
    statusBar()->showMessage(tr("Ready"));
}

MainWindow::~MainWindow()
{
    if (m_serial.isOpen())
    {
        m_serial.close();
    }
}

void MainWindow::setupUi()
{
    setWindowTitle(tr("FOC CDC Debug Tool"));
    resize(1480, 920);

    auto *central = new QWidget(this);
    setCentralWidget(central);
    auto *root = new QVBoxLayout(central);

    m_tabs = new QTabWidget(central);
    m_controlPage = new QWidget(m_tabs);
    m_anglePage = new QWidget(m_tabs);
    m_speedPage = new QWidget(m_tabs);
    m_debugPage = new QWidget(m_tabs);
    setupControlPage(m_controlPage);
    setupAnglePage(m_anglePage);
    setupSpeedPage(m_speedPage);
    setupDebugPage(m_debugPage);
    m_tabs->addTab(m_controlPage, tr("Control"));
    m_tabs->addTab(m_anglePage, tr("Angle"));
    m_tabs->addTab(m_speedPage, tr("Speed"));
    m_tabs->addTab(m_debugPage, tr("Debug"));

    root->addWidget(m_tabs, 1);
}

void MainWindow::setupDebugPage(QWidget *page)
{
    auto *layout = new QVBoxLayout(page);
    auto *title = new QLabel(tr("Raw CDC lines, telemetry and parsed debug"), page);
    m_debugClearButton = new QPushButton(tr("Clear Debug"), page);
    m_debugEdit = new QPlainTextEdit(page);
    m_debugEdit->setReadOnly(true);
    auto *topRow = new QHBoxLayout();
    topRow->addWidget(title, 1);
    topRow->addWidget(m_debugClearButton);
    layout->addLayout(topRow);
    layout->addWidget(m_debugEdit, 1);

    connect(m_debugClearButton, &QPushButton::clicked, this, [this]() {
        if (m_debugEdit != nullptr)
        {
            m_debugEdit->clear();
        }
    });
}

void MainWindow::setupControlPage(QWidget *page)
{
    auto *root = new QVBoxLayout(page);

    auto *connGroup = new QGroupBox(tr("CDC / Serial Connection"), page);
    auto *connGrid = new QGridLayout(connGroup);
    m_portCombo = new QComboBox(connGroup);
    m_refreshButton = new QPushButton(tr("Refresh"), connGroup);
    m_openButton = new QPushButton(tr("Open"), connGroup);
    m_baudCombo = new QComboBox(connGroup);
    m_dataBitsCombo = new QComboBox(connGroup);
    m_parityCombo = new QComboBox(connGroup);
    m_stopBitsCombo = new QComboBox(connGroup);
    m_flowCombo = new QComboBox(connGroup);
    m_hexSendCheck = new QCheckBox(tr("Hex Send"), connGroup);
    m_hexViewCheck = new QCheckBox(tr("Hex View"), connGroup);
    m_autoScrollCheck = new QCheckBox(tr("Auto Scroll"), connGroup);
    m_autoTxNewlineCheck = new QCheckBox(tr("Auto Newline"), connGroup);

    m_baudCombo->addItems({tr("9600"), tr("19200"), tr("38400"), tr("57600"), tr("115200"), tr("230400"), tr("460800"), tr("921600")});
    m_baudCombo->setCurrentText(tr("115200"));
    m_dataBitsCombo->addItems({tr("8"), tr("7"), tr("6"), tr("5")});
    m_parityCombo->addItems({tr("None"), tr("Even"), tr("Odd")});
    m_stopBitsCombo->addItems({tr("1"), tr("1.5"), tr("2")});
    m_flowCombo->addItems({tr("None"), tr("RTS/CTS"), tr("XON/XOFF")});
    m_autoScrollCheck->setChecked(true);
    m_autoTxNewlineCheck->setChecked(true);

    connGrid->addWidget(new QLabel(tr("Port"), connGroup), 0, 0);
    connGrid->addWidget(m_portCombo, 0, 1);
    connGrid->addWidget(m_refreshButton, 0, 2);
    connGrid->addWidget(m_openButton, 0, 3);
    connGrid->addWidget(new QLabel(tr("Baud"), connGroup), 1, 0);
    connGrid->addWidget(m_baudCombo, 1, 1);
    connGrid->addWidget(new QLabel(tr("Data"), connGroup), 1, 2);
    connGrid->addWidget(m_dataBitsCombo, 1, 3);
    connGrid->addWidget(new QLabel(tr("Parity"), connGroup), 2, 0);
    connGrid->addWidget(m_parityCombo, 2, 1);
    connGrid->addWidget(new QLabel(tr("Stop"), connGroup), 2, 2);
    connGrid->addWidget(m_stopBitsCombo, 2, 3);
    connGrid->addWidget(new QLabel(tr("Flow"), connGroup), 3, 0);
    connGrid->addWidget(m_flowCombo, 3, 1);
    connGrid->addWidget(m_hexSendCheck, 3, 2);
    connGrid->addWidget(m_hexViewCheck, 3, 3);
    connGrid->addWidget(m_autoScrollCheck, 4, 0);
    connGrid->addWidget(m_autoTxNewlineCheck, 4, 1);

    auto *presetGroup = new QGroupBox(tr("CDC Commands"), page);
    auto *presetGrid = new QGridLayout(presetGroup);
    m_speedSpin = new QDoubleSpinBox(presetGroup);
    m_speedSpin->setRange(-5000.0, 5000.0);
    m_speedSpin->setDecimals(2);
    m_speedSpin->setSingleStep(10.0);
    m_speedSpin->setValue(100.0);

    m_kpSpin = new QDoubleSpinBox(presetGroup);
    m_kpSpin->setRange(0.0, 1000.0);
    m_kpSpin->setDecimals(4);
    m_kpSpin->setSingleStep(0.01);
    m_kpSpin->setValue(1.0);

    m_kiSpin = new QDoubleSpinBox(presetGroup);
    m_kiSpin->setRange(0.0, 10000.0);
    m_kiSpin->setDecimals(4);
    m_kiSpin->setSingleStep(0.5);
    m_kiSpin->setValue(4.0);

    m_nodeIdSpin = new QSpinBox(presetGroup);
    m_nodeIdSpin->setRange(0, 15);
    m_nodeIdSpin->setValue(1);

    m_calibModeSpin = new QSpinBox(presetGroup);
    m_calibModeSpin->setRange(0, 255);
    m_calibModeSpin->setValue(0);

    m_startButton = new QPushButton(tr("Start"), presetGroup);
    m_stopButton = new QPushButton(tr("Stop"), presetGroup);
    m_speedModeButton = new QPushButton(tr("Speed Mode"), presetGroup);
    m_posModeButton = new QPushButton(tr("Position Mode"), presetGroup);
    m_openLoopButton = new QPushButton(tr("Open Loop"), presetGroup);
    m_setSpeedButton = new QPushButton(tr("Set Speed"), presetGroup);
    m_setKpButton = new QPushButton(tr("Set Kp"), presetGroup);
    m_setKiButton = new QPushButton(tr("Set Ki"), presetGroup);
    m_getIdButton = new QPushButton(tr("Get ID"), presetGroup);
    m_setIdButton = new QPushButton(tr("Set ID"), presetGroup);
    m_startCalibButton = new QPushButton(tr("Start Calib"), presetGroup);
    m_stopCalibButton = new QPushButton(tr("Stop Calib"), presetGroup);
    m_readFlashButton = new QPushButton(tr("Read Flash"), presetGroup);
    m_clearFlashButton = new QPushButton(tr("Clear Flash"), presetGroup);

    presetGrid->addWidget(new QLabel(tr("Speed rpm"), presetGroup), 0, 0);
    presetGrid->addWidget(m_speedSpin, 0, 1);
    presetGrid->addWidget(m_setSpeedButton, 0, 2);
    presetGrid->addWidget(new QLabel(tr("Speed Kp"), presetGroup), 1, 0);
    presetGrid->addWidget(m_kpSpin, 1, 1);
    presetGrid->addWidget(m_setKpButton, 1, 2);
    presetGrid->addWidget(new QLabel(tr("Speed Ki"), presetGroup), 2, 0);
    presetGrid->addWidget(m_kiSpin, 2, 1);
    presetGrid->addWidget(m_setKiButton, 2, 2);
    presetGrid->addWidget(new QLabel(tr("Node ID"), presetGroup), 3, 0);
    presetGrid->addWidget(m_nodeIdSpin, 3, 1);
    presetGrid->addWidget(m_getIdButton, 3, 2);
    presetGrid->addWidget(new QLabel(tr("Calib mode"), presetGroup), 4, 0);
    presetGrid->addWidget(m_calibModeSpin, 4, 1);
    presetGrid->addWidget(m_setIdButton, 4, 2);
    presetGrid->addWidget(m_startButton, 5, 0);
    presetGrid->addWidget(m_stopButton, 5, 1);
    presetGrid->addWidget(m_speedModeButton, 5, 2);
    presetGrid->addWidget(m_posModeButton, 6, 0);
    presetGrid->addWidget(m_openLoopButton, 6, 1);
    presetGrid->addWidget(m_startCalibButton, 6, 2);
    presetGrid->addWidget(m_stopCalibButton, 7, 0);
    presetGrid->addWidget(m_readFlashButton, 7, 1);
    presetGrid->addWidget(m_clearFlashButton, 7, 2);

    auto *ioRow = new QHBoxLayout();
    m_sendEdit = new QLineEdit(page);
    m_sendButton = new QPushButton(tr("Send"), page);
    ioRow->addWidget(new QLabel(tr("Tx"), page));
    ioRow->addWidget(m_sendEdit, 1);
    ioRow->addWidget(m_sendButton);

    m_logEdit = new QTextEdit(page);
    m_logEdit->setReadOnly(true);

    root->addWidget(connGroup);
    root->addWidget(presetGroup);
    root->addLayout(ioRow);
    root->addWidget(m_logEdit, 1);

    connect(m_refreshButton, &QPushButton::clicked, this, &MainWindow::refreshPorts);
    connect(m_openButton, &QPushButton::clicked, this, &MainWindow::openClosePort);
    connect(m_sendButton, &QPushButton::clicked, this, &MainWindow::sendData);
    connect(m_sendEdit, &QLineEdit::returnPressed, this, &MainWindow::sendData);

    connect(m_startButton, &QPushButton::clicked, this, [this]() { sendPresetCommand("start"); });
    connect(m_stopButton, &QPushButton::clicked, this, [this]() { sendPresetCommand("stop"); });
    connect(m_speedModeButton, &QPushButton::clicked, this, [this]() { sendPresetCommand("speedmode"); });
    connect(m_posModeButton, &QPushButton::clicked, this, [this]() { sendPresetCommand("posmode"); });
    connect(m_openLoopButton, &QPushButton::clicked, this, [this]() { sendPresetCommand("vfmode"); });
    connect(m_setSpeedButton, &QPushButton::clicked, this, [this]() { sendPresetCommand(QStringLiteral("speed %1").arg(m_speedSpin->value(), 0, 'f', 3)); });
    connect(m_setKpButton, &QPushButton::clicked, this, [this]() { sendPresetCommand(QStringLiteral("kp %1").arg(m_kpSpin->value(), 0, 'f', 4)); });
    connect(m_setKiButton, &QPushButton::clicked, this, [this]() { sendPresetCommand(QStringLiteral("ki %1").arg(m_kiSpin->value(), 0, 'f', 4)); });
    connect(m_getIdButton, &QPushButton::clicked, this, [this]() { sendPresetCommand("nodeget"); });
    connect(m_setIdButton, &QPushButton::clicked, this, [this]() { sendPresetCommand(QStringLiteral("nodeset %1").arg(m_nodeIdSpin->value())); });
    connect(m_startCalibButton, &QPushButton::clicked, this, [this]() { sendPresetCommand(QStringLiteral("calib %1").arg(m_calibModeSpin->value())); });
    connect(m_stopCalibButton, &QPushButton::clicked, this, [this]() { sendPresetCommand("calibstop"); });
    connect(m_readFlashButton, &QPushButton::clicked, this, [this]() { sendPresetCommand("flashread"); });
    connect(m_clearFlashButton, &QPushButton::clicked, this, [this]() { sendPresetCommand("flashclear"); });
}

void MainWindow::setupAnglePage(QWidget *page)
{
    auto *root = new QVBoxLayout(page);

    auto *statusGroup = new QGroupBox(tr("Angle Value"), page);
    auto *statusLayout = new QGridLayout(statusGroup);
    m_statusAngle = new QLabel(tr("0.000000"), statusGroup);
    m_statusAngleLink = new QLabel(tr("Waiting telemetry..."), statusGroup);
    m_angleDetailLabel = new QLabel(tr("MECH: 0.000000 | APP: 0.000000"), statusGroup);
    statusLayout->addWidget(new QLabel(tr("Angle deg"), statusGroup), 0, 0);
    statusLayout->addWidget(m_statusAngle, 0, 1);
    statusLayout->addWidget(new QLabel(tr("Link"), statusGroup), 0, 2);
    statusLayout->addWidget(m_statusAngleLink, 0, 3);
    statusLayout->addWidget(m_angleDetailLabel, 1, 0, 1, 4);

    m_angleMechSeries = new QLineSeries(page);
    m_angleMechSeries->setName(tr("MECH"));
    m_angleAppSeries = new QLineSeries(page);
    m_angleAppSeries->setName(tr("APP"));

    auto *chartBox = new QGroupBox(tr("Angle Trend"), page);
    auto *layout = new QVBoxLayout(chartBox);
    auto *chart = new QChart();
    chart->addSeries(m_angleMechSeries);
    chart->addSeries(m_angleAppSeries);
    chart->createDefaultAxes();
    m_angleAxisX = qobject_cast<QValueAxis *>(chart->axes(Qt::Horizontal).first());
    m_angleAxisY = qobject_cast<QValueAxis *>(chart->axes(Qt::Vertical).first());
    if (m_angleAxisX != nullptr)
    {
        m_angleAxisX->setTitleText(tr("Sample"));
    }
    if (m_angleAxisY != nullptr)
    {
        m_angleAxisY->setTitleText(tr("Angle deg"));
        m_angleAxisY->setRange(-370.0, 370.0);
        m_angleAxisY->setTickCount(11);
        m_angleAxisY->setLabelFormat("%.1f");
        m_angleAxisY->setReverse(false);
    }
    chart->legend()->setVisible(true);
    chart->setTitle(tr("Rotor Angle Compare"));
    m_angleChartView = new QChartView(chart, chartBox);
    m_angleChartView->setRenderHint(QPainter::Antialiasing);
    layout->addWidget(m_angleChartView);

    m_angleTimeSlider = new QSlider(Qt::Horizontal, chartBox);
    m_angleTimeSlider->setRange(0, 0);
    m_angleTimeSlider->setSingleStep(1);
    m_angleTimeSlider->setPageStep(20);
    layout->addWidget(m_angleTimeSlider);

    auto *zoomRow = new QHBoxLayout();
    m_angleZoomOutButton = new QToolButton(chartBox);
    m_angleZoomOutButton->setText(tr("-"));
    m_angleZoomInButton = new QToolButton(chartBox);
    m_angleZoomInButton->setText(tr("+"));
    m_angleZoomResetButton = new QToolButton(chartBox);
    m_angleZoomResetButton->setText(tr("Reset"));
    zoomRow->addWidget(new QLabel(tr("Zoom"), chartBox));
    zoomRow->addWidget(m_angleZoomOutButton);
    zoomRow->addWidget(m_angleZoomInButton);
    zoomRow->addWidget(m_angleZoomResetButton);
    zoomRow->addStretch(1);
    layout->addLayout(zoomRow);

    root->addWidget(statusGroup);
    root->addWidget(chartBox, 1);

    connect(m_angleTimeSlider, &QSlider::valueChanged, this, [this]() {
        if (!m_angleSliderDragging)
        {
            refreshAngleChart();
        }
    });
    connect(m_angleTimeSlider, &QSlider::sliderPressed, this, [this]() {
        m_angleSliderDragging = true;
        m_angleAutoFollow = false;
    });
    connect(m_angleTimeSlider, &QSlider::sliderReleased, this, [this]() {
        m_angleSliderDragging = false;
        refreshAngleChart();
    });
    connect(m_angleZoomInButton, &QToolButton::clicked, this, [this]() { zoomAngleWindow(-300); });
    connect(m_angleZoomOutButton, &QToolButton::clicked, this, [this]() { zoomAngleWindow(300); });
    connect(m_angleZoomResetButton, &QToolButton::clicked, this, [this]() { resetAngleWindow(); });
}

void MainWindow::setupSpeedPage(QWidget *page)
{
    auto *root = new QVBoxLayout(page);

    auto *statusGroup = new QGroupBox(tr("Speed Value"), page);
    auto *statusLayout = new QGridLayout(statusGroup);
    m_statusSpeed = new QLabel(tr("0.000000"), statusGroup);
    m_speedLink = new QLabel(tr("Waiting telemetry..."), statusGroup);
    m_speedDetailLabel = new QLabel(tr("SPEED: 0.000000 rpm | ID: 0.000000 A | IQ: 0.000000 A"), statusGroup);
    statusLayout->addWidget(new QLabel(tr("Speed rpm"), statusGroup), 0, 0);
    statusLayout->addWidget(m_statusSpeed, 0, 1);
    statusLayout->addWidget(new QLabel(tr("Link"), statusGroup), 0, 2);
    statusLayout->addWidget(m_speedLink, 0, 3);
    statusLayout->addWidget(m_speedDetailLabel, 1, 0, 1, 4);

    auto *chartBox = new QGroupBox(tr("Speed Trend"), page);
    auto *layout = new QVBoxLayout(chartBox);
    auto *chart = new QChart();
    m_speedSeries = new QLineSeries(page);
    m_speedSeries->setName(tr("Speed RPM"));
    m_speedIqSeries = new QLineSeries(page);
    m_speedIqSeries->setName(tr("Iq"));
    m_speedIdSeries = new QLineSeries(page);
    m_speedIdSeries->setName(tr("Id"));
    chart->addSeries(m_speedSeries);
    chart->addSeries(m_speedIqSeries);
    chart->addSeries(m_speedIdSeries);
    chart->createDefaultAxes();
    m_speedAxisX = qobject_cast<QValueAxis *>(chart->axes(Qt::Horizontal).first());
    m_speedAxisY = qobject_cast<QValueAxis *>(chart->axes(Qt::Vertical).first());
    if (m_speedAxisX != nullptr)
    {
        m_speedAxisX->setTitleText(tr("Sample"));
    }
    if (m_speedAxisY != nullptr)
    {
        m_speedAxisY->setTitleText(tr("RPM"));
        m_speedAxisY->setRange(-3000.0, 3000.0);
        m_speedAxisY->setTickCount(11);
        m_speedAxisY->setLabelFormat("%.0f");
    }
    m_speedAxisIq = new QValueAxis(chart);
    m_speedAxisIq->setTitleText(tr("Current (A)"));
    m_speedAxisIq->setRange(-0.5, 0.5);
    m_speedAxisIq->setTickCount(7);
    m_speedAxisIq->setLabelFormat("%.3f");
    chart->addAxis(m_speedAxisIq, Qt::AlignRight);
    m_speedIqSeries->attachAxis(m_speedAxisIq);
    m_speedIdSeries->attachAxis(m_speedAxisIq);
    if (m_speedAxisY != nullptr)
    {
        m_speedIqSeries->detachAxis(m_speedAxisY);
        m_speedIdSeries->detachAxis(m_speedAxisY);
    }
    chart->legend()->setVisible(true);
    chart->setTitle(tr("Speed / Current Trend"));
    m_speedChartView = new QChartView(chart, chartBox);
    m_speedChartView->setRenderHint(QPainter::Antialiasing);
    layout->addWidget(m_speedChartView);

    m_speedTimeSlider = new QSlider(Qt::Horizontal, chartBox);
    m_speedTimeSlider->setRange(0, 0);
    m_speedTimeSlider->setSingleStep(1);
    m_speedTimeSlider->setPageStep(20);
    layout->addWidget(m_speedTimeSlider);

    auto *zoomRow = new QHBoxLayout();
    m_speedZoomOutButton = new QToolButton(chartBox);
    m_speedZoomOutButton->setText(tr("-"));
    m_speedZoomInButton = new QToolButton(chartBox);
    m_speedZoomInButton->setText(tr("+"));
    m_speedZoomResetButton = new QToolButton(chartBox);
    m_speedZoomResetButton->setText(tr("Reset"));
    zoomRow->addWidget(new QLabel(tr("Zoom"), chartBox));
    zoomRow->addWidget(m_speedZoomOutButton);
    zoomRow->addWidget(m_speedZoomInButton);
    zoomRow->addWidget(m_speedZoomResetButton);
    zoomRow->addStretch(1);
    layout->addLayout(zoomRow);

    root->addWidget(statusGroup);
    root->addWidget(chartBox, 1);

    connect(m_speedTimeSlider, &QSlider::valueChanged, this, [this]() {
        if (!m_speedSliderDragging)
        {
            refreshSpeedChart();
        }
    });
    connect(m_speedTimeSlider, &QSlider::sliderPressed, this, [this]() {
        m_speedSliderDragging = true;
        m_speedAutoFollow = false;
    });
    connect(m_speedTimeSlider, &QSlider::sliderReleased, this, [this]() {
        m_speedSliderDragging = false;
        refreshSpeedChart();
    });
    connect(m_speedZoomInButton, &QToolButton::clicked, this, [this]() { zoomSpeedWindow(-300); });
    connect(m_speedZoomOutButton, &QToolButton::clicked, this, [this]() { zoomSpeedWindow(300); });
    connect(m_speedZoomResetButton, &QToolButton::clicked, this, [this]() { resetSpeedWindow(); });
}

void MainWindow::refreshPorts()
{
    const QString currentData = m_portCombo->currentData().toString();
    m_portCombo->clear();

    const auto ports = SerialBackend::availablePorts();
    for (const auto &info : ports)
    {
        QString label = info.portName;
        if (!info.friendlyName.isEmpty())
        {
            label += QStringLiteral(" - ") + info.friendlyName;
        }
        m_portCombo->addItem(label, info.portName);
    }

    if (!currentData.isEmpty())
    {
        const int index = m_portCombo->findData(currentData);
        if (index >= 0)
        {
            m_portCombo->setCurrentIndex(index);
        }
    }
}

void MainWindow::openClosePort()
{
    if (m_serial.isOpen())
    {
        m_serial.close();
        m_angleAutoFollow = true;
        m_speedAutoFollow = true;
        m_angleSliderDragging = false;
        m_speedSliderDragging = false;
        appendLog(tr("[%1] Port closed").arg(QDateTime::currentDateTime().toString("HH:mm:ss")));
        setConnectedUi(false);
        statusBar()->showMessage(tr("Disconnected"));
        return;
    }

    const QString portName = m_portCombo->currentData().toString();
    if (portName.isEmpty())
    {
        appendLog(tr("No serial port selected."));
        return;
    }

    QString error;
    if (!m_serial.open(portName,
                       m_baudCombo->currentText().toInt(),
                       m_dataBitsCombo->currentText().toInt(),
                       m_parityCombo->currentIndex(),
                       m_stopBitsCombo->currentIndex(),
                       m_flowCombo->currentIndex(),
                       &error))
    {
        appendLog(tr("Open failed: %1").arg(error.isEmpty() ? m_serial.errorString() : error));
        statusBar()->showMessage(tr("Open failed"));
        return;
    }

    appendLog(tr("[%1] Port opened: %2").arg(QDateTime::currentDateTime().toString("HH:mm:ss"), portName));
    setConnectedUi(true);
    statusBar()->showMessage(tr("Connected: %1").arg(portName));
}

void MainWindow::sendData()
{
    if (!m_serial.isOpen())
    {
        appendLog(tr("Port is not open."));
        return;
    }

    const QString text = m_sendEdit->text().trimmed();
    if (text.isEmpty())
    {
        return;
    }

    QByteArray payload = text.toUtf8();
    if (m_autoTxNewlineCheck->isChecked())
    {
        payload.append('\n');
    }

    const qint64 written = m_serial.write(payload);
    if (written < 0)
    {
        appendLog(tr("Send failed: %1").arg(m_serial.errorString()));
        return;
    }

    appendLog(tr("Tx: %1").arg(text));
    m_sendEdit->clear();
}

void MainWindow::sendPresetCommand(const QString &text)
{
    if (!m_serial.isOpen())
    {
        appendLog(tr("Port is not open."));
        return;
    }

    QByteArray payload = text.toUtf8();
    payload.append('\n');
    const qint64 written = m_serial.write(payload);
    if (written < 0)
    {
        appendLog(tr("Send failed: %1").arg(m_serial.errorString()));
        return;
    }
    appendLog(tr("Tx: %1").arg(text));
}

void MainWindow::clearLog()
{
    m_logEdit->clear();
    m_sampleIndex = 0;
    if (m_debugEdit != nullptr)
    {
        m_debugEdit->clear();
    }
    m_rxLineBuffer.clear();
    if (m_angleTimeSlider != nullptr)
    {
        m_angleTimeSlider->setRange(0, 0);
        m_angleTimeSlider->setValue(0);
    }
    if (m_speedSeries != nullptr)
    {
        m_speedSeries->clear();
    }
    m_speedPoints.clear();
    m_speedSampleIndex = 0;
    if (m_speedTimeSlider != nullptr)
    {
        m_speedTimeSlider->setRange(0, 0);
        m_speedTimeSlider->setValue(0);
    }
    m_angleAutoFollow = true;
    m_speedAutoFollow = true;
    m_angleSliderDragging = false;
    m_speedSliderDragging = false;
}

void MainWindow::pollSerial()
{
    if (!m_serial.isOpen())
    {
        return;
    }

    const QByteArray data = m_serial.readAll();
    if (data.isEmpty())
    {
        return;
    }

    QString chunk;
    chunk.reserve(data.size());
    for (unsigned char c : data)
    {
        if ((c == '\r') || (c == '\n') || (c == '\t') || (c >= 0x20 && c <= 0x7E))
        {
            chunk.append(QChar(static_cast<ushort>(c)));
        }
    }
    m_rxLineBuffer.append(chunk);
    int newlinePos = -1;
    while ((newlinePos = m_rxLineBuffer.indexOf(QRegularExpression("[\\r\\n]"))) >= 0)
    {
        QString line = m_rxLineBuffer.left(newlinePos).trimmed();
        m_rxLineBuffer.remove(0, newlinePos + 1);
        if (!line.isEmpty())
        {
            processLine(line);
        }
        while (!m_rxLineBuffer.isEmpty() && (m_rxLineBuffer.front() == '\r' || m_rxLineBuffer.front() == '\n'))
        {
            m_rxLineBuffer.remove(0, 1);
        }
    }
}

void MainWindow::processLine(const QString &line)
{
    if (line.startsWith(QStringLiteral("TEL ")))
    {
        handleTelemetryLine(line);
        return;
    }

    if (line.startsWith(QStringLiteral("DBG ")))
    {
        appendDebug(line);
        return;
    }
}

bool MainWindow::parseKeyValueFloat(const QString &line, const QString &key, double *value)
{
    const QRegularExpression rx(QStringLiteral("\\b%1=([-+0-9.eE]+)").arg(QRegularExpression::escape(key)),
                                QRegularExpression::CaseInsensitiveOption);
    const QRegularExpressionMatch match = rx.match(line);
    if (!match.hasMatch())
    {
        return false;
    }

    bool ok = false;
    const double v = match.captured(1).toDouble(&ok);
    if (!ok)
    {
        return false;
    }

    *value = v;
    return true;
}

void MainWindow::handleTelemetryLine(const QString &line)
{
    double mechDeg = 0.0;
    double appDeg = 0.0;
    double iqAmp = 0.0;
    double idAmp = 0.0;
    double speedRpm = 0.0;
    const bool hasMech = parseKeyValueFloat(line, QStringLiteral("MECH"), &mechDeg);
    const bool hasApp = parseKeyValueFloat(line, QStringLiteral("APP"), &appDeg);
    const bool hasIq = parseKeyValueFloat(line, QStringLiteral("IQ"), &iqAmp) ||
                       parseKeyValueFloat(line, QStringLiteral("IQR"), &iqAmp);
    const bool hasId = parseKeyValueFloat(line, QStringLiteral("ID"), &idAmp);
    const bool hasSpeed = parseKeyValueFloat(line, QStringLiteral("SPEED"), &speedRpm) ||
                          parseKeyValueFloat(line, QStringLiteral("SPD"), &speedRpm);

    if (hasMech || hasApp)
    {
        updateAngle(mechDeg, appDeg);
    }

    if (hasSpeed)
    {
        m_lastSpeedRpm = speedRpm;
    }
    if (hasIq)
    {
        m_lastIqAmp = iqAmp;
    }
    if (hasId)
    {
        m_lastIdAmp = idAmp;
    }

    if (hasSpeed || hasIq || hasId)
    {
        updateSpeed(m_lastSpeedRpm, m_lastIqAmp, m_lastIdAmp);
    }

    if (hasMech || hasApp || hasIq || hasSpeed)
    {
        m_lastTelemetryTick = QDateTime::currentMSecsSinceEpoch();
        if (m_statusLink != nullptr)
        {
            m_statusLink->setText(tr("Telemetry OK"));
        }
    }
}

void MainWindow::updateAngle(double mechDeg, double appDeg)
{
    if (m_angleMechSeries == nullptr || m_angleAppSeries == nullptr)
    {
        return;
    }

    const qint64 x = m_angleMechPoints.size();
    m_angleMechPoints.append(QPointF(x, mechDeg));
    m_angleAppPoints.append(QPointF(x, appDeg));
    ++m_sampleIndex;

    constexpr int kMaxPoints = 7200;
    while (m_angleMechPoints.size() > kMaxPoints)
    {
        m_angleMechPoints.removeFirst();
        m_angleAppPoints.removeFirst();
    }

    if (m_angleTimeSlider != nullptr)
    {
        const int maxOffset = qMax(0, m_angleMechPoints.size() - 1);
        m_angleTimeSlider->setRange(0, maxOffset);
        if (m_angleAutoFollow)
        {
            m_angleTimeSlider->blockSignals(true);
            m_angleTimeSlider->setValue(qMax(0, m_angleMechPoints.size() - m_angleWindowSize));
            m_angleTimeSlider->blockSignals(false);
        }
    }

    const qint64 now = QDateTime::currentMSecsSinceEpoch();
    if ((now - m_lastAngleUiTick) >= 10 || m_angleSliderDragging)
    {
        refreshAngleChart();
        m_lastAngleUiTick = now;
    }

    if (m_statusAngle != nullptr)
    {
        m_statusAngle->setText(fmt6(mechDeg));
    }
    if (m_angleDetailLabel != nullptr)
    {
        m_angleDetailLabel->setText(tr("MECH: %1 | APP: %2")
                                        .arg(fmt6(mechDeg), fmt6(appDeg)));
    }
    if (m_statusAngleLink != nullptr)
    {
        m_statusAngleLink->setText(tr("Telemetry OK"));
    }
}

void MainWindow::updateSpeed(double speedRpm, double iqAmp, double idAmp)
{
    if (m_speedSeries == nullptr || m_speedIqSeries == nullptr || m_speedIdSeries == nullptr)
    {
        return;
    }

    const qint64 x = m_speedPoints.size();
    m_speedPoints.append(QPointF(x, speedRpm));
    m_speedIqPoints.append(QPointF(x, iqAmp));
    m_speedIdPoints.append(QPointF(x, idAmp));
    ++m_speedSampleIndex;

    constexpr int kMaxPoints = 7200;
    while (m_speedPoints.size() > kMaxPoints)
    {
        m_speedPoints.removeFirst();
        m_speedIqPoints.removeFirst();
        m_speedIdPoints.removeFirst();
    }

    if (m_speedTimeSlider != nullptr)
    {
        const int maxOffset = qMax(0, m_speedPoints.size() - 1);
        m_speedTimeSlider->setRange(0, maxOffset);
        if (m_speedAutoFollow)
        {
            m_speedTimeSlider->blockSignals(true);
            m_speedTimeSlider->setValue(qMax(0, m_speedPoints.size() - m_speedWindowSize));
            m_speedTimeSlider->blockSignals(false);
        }
    }

    const qint64 now = QDateTime::currentMSecsSinceEpoch();
    if ((now - m_lastSpeedUiTick) >= 20 || m_speedSliderDragging)
    {
        refreshSpeedChart();
        m_lastSpeedUiTick = now;
    }

    if (m_statusSpeed != nullptr)
    {
        m_statusSpeed->setText(fmt6(speedRpm));
    }
    if (m_speedDetailLabel != nullptr)
    {
        m_speedDetailLabel->setText(tr("SPEED: %1 rpm | ID: %2 A | IQ: %3 A")
                                        .arg(fmt6(speedRpm), fmt6(idAmp), fmt6(iqAmp)));
    }
    if (m_speedLink != nullptr)
    {
        m_speedLink->setText(tr("Telemetry OK"));
    }
}

void MainWindow::refreshAngleChart()
{
    if (m_angleMechSeries == nullptr || m_angleAppSeries == nullptr || m_angleAxisX == nullptr || m_angleAxisY == nullptr)
    {
        return;
    }

    const int count = m_angleMechPoints.size();
    if (count <= 0)
    {
        m_angleMechSeries->clear();
        m_angleAppSeries->clear();
        return;
    }

    const int startIndex = qBound(0, m_angleTimeSlider != nullptr ? m_angleTimeSlider->value() : 0, qMax(0, count - 1));
    const int endIndex = qMin(count, startIndex + m_angleWindowSize);
    const int size = qMax(0, endIndex - startIndex);

    m_angleMechSeries->replace(m_angleMechPoints.mid(startIndex, size));
    m_angleAppSeries->replace(m_angleAppPoints.mid(startIndex, size));

    if (m_angleMechSeries != nullptr)
    {
        m_angleMechSeries->attachAxis(m_angleAxisX);
        m_angleMechSeries->attachAxis(m_angleAxisY);
    }
    if (m_angleAppSeries != nullptr)
    {
        m_angleAppSeries->attachAxis(m_angleAxisX);
        m_angleAppSeries->attachAxis(m_angleAxisY);
    }
    m_angleAxisX->setRange(startIndex, qMax(startIndex + 1, endIndex - 1));
    m_angleAxisX->setLabelFormat("%.0f");

    m_angleAxisY->setRange(-370.0, 370.0);
    m_angleAxisY->setTickCount(11);
    m_angleAxisY->setLabelFormat("%.1f");
    m_angleAxisY->setReverse(false);

}

void MainWindow::refreshSpeedChart()
{
    if (m_speedSeries == nullptr || m_speedIqSeries == nullptr || m_speedIdSeries == nullptr || m_speedAxisX == nullptr || m_speedAxisY == nullptr || m_speedAxisIq == nullptr)
    {
        return;
    }

    const int count = m_speedPoints.size();
    if (count <= 0)
    {
        m_speedSeries->clear();
        m_speedIqSeries->clear();
        m_speedIdSeries->clear();
        return;
    }

    const int startIndex = qBound(0, m_speedTimeSlider != nullptr ? m_speedTimeSlider->value() : 0, qMax(0, count - 1));
    const int endIndex = qMin(count, startIndex + m_speedWindowSize);
    const int size = qMax(0, endIndex - startIndex);

    m_speedSeries->replace(m_speedPoints.mid(startIndex, size));
    m_speedIqSeries->replace(m_speedIqPoints.mid(startIndex, size));
    m_speedIdSeries->replace(m_speedIdPoints.mid(startIndex, size));

    m_speedAxisX->setRange(startIndex, qMax(startIndex + 1, endIndex - 1));
    m_speedAxisX->setLabelFormat("%.0f");

    double minSpeed = m_speedPoints[startIndex].y();
    double maxSpeed = minSpeed;
    for (int i = startIndex; i < endIndex; ++i)
    {
        minSpeed = qMin(minSpeed, m_speedPoints[i].y());
        maxSpeed = qMax(maxSpeed, m_speedPoints[i].y());
    }

    const double span = qMax(50.0, maxSpeed - minSpeed);
    const double margin = qMax(50.0, span * 0.25);
    const double speedMin = qBound(-10000.0, minSpeed - margin, 10000.0);
    const double speedMax = qBound(-10000.0, maxSpeed + margin, 10000.0);
    m_speedAxisY->setRange(speedMin, speedMax);
    m_speedAxisY->setTickCount(11);
    m_speedAxisY->setLabelFormat("%.0f");

    if (!m_speedIqPoints.isEmpty())
    {
        double minIq = m_speedIqPoints[startIndex].y();
        double maxIq = minIq;
        for (int i = startIndex; i < endIndex; ++i)
        {
            minIq = qMin(minIq, m_speedIqPoints[i].y());
            maxIq = qMax(maxIq, m_speedIqPoints[i].y());
        }

        Q_UNUSED(minIq);
        Q_UNUSED(maxIq);
        m_speedAxisIq->setRange(-0.5, 0.5);
    }
    else
    {
        m_speedAxisIq->setRange(-0.5, 0.5);
    }
    m_speedAxisIq->setTickCount(7);
    m_speedAxisIq->setLabelFormat("%.3f");
}

void MainWindow::zoomAngleWindow(int delta)
{
    m_angleWindowSize = qBound(100, m_angleWindowSize + delta, 6000);
    m_angleAutoFollow = false;
    refreshAngleChart();
}

void MainWindow::zoomSpeedWindow(int delta)
{
    m_speedWindowSize = qBound(100, m_speedWindowSize + delta, 6000);
    m_speedAutoFollow = false;
    refreshSpeedChart();
}

void MainWindow::resetAngleWindow()
{
    m_angleWindowSize = 1800;
    m_angleAutoFollow = true;
    if (m_angleTimeSlider != nullptr)
    {
        m_angleTimeSlider->setValue(qMax(0, m_angleMechPoints.size() - m_angleWindowSize));
    }
    refreshAngleChart();
}

void MainWindow::resetSpeedWindow()
{
    m_speedWindowSize = 1800;
    m_speedAutoFollow = true;
    if (m_speedTimeSlider != nullptr)
    {
        m_speedTimeSlider->setValue(qMax(0, m_speedPoints.size() - m_speedWindowSize));
    }
    refreshSpeedChart();
}
void MainWindow::appendLog(const QString &text)
{
    m_logEdit->append(text);
    if (m_autoScrollCheck->isChecked())
    {
        m_logEdit->verticalScrollBar()->setValue(m_logEdit->verticalScrollBar()->maximum());
    }
}

void MainWindow::appendDebug(const QString &text)
{
    if (m_debugEdit == nullptr)
    {
        return;
    }
    m_debugEdit->appendPlainText(QStringLiteral("[%1] %2")
                                     .arg(QDateTime::currentDateTime().toString("HH:mm:ss.zzz"), text));
    m_debugEdit->verticalScrollBar()->setValue(m_debugEdit->verticalScrollBar()->maximum());
}

void MainWindow::setConnectedUi(bool connected)
{
    m_openButton->setText(connected ? tr("Close") : tr("Open"));
    m_portCombo->setEnabled(!connected);
    m_refreshButton->setEnabled(!connected);
    m_baudCombo->setEnabled(!connected);
    m_dataBitsCombo->setEnabled(!connected);
    m_parityCombo->setEnabled(!connected);
    m_stopBitsCombo->setEnabled(!connected);
    m_flowCombo->setEnabled(!connected);
    if (!connected)
    {
        if (m_angleTimeSlider != nullptr)
        {
            m_angleTimeSlider->setEnabled(true);
        }
        if (m_speedTimeSlider != nullptr)
        {
            m_speedTimeSlider->setEnabled(true);
        }
    }
}
