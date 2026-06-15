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
    m_debugPage = new QWidget(m_tabs);
    setupControlPage(m_controlPage);
    setupAnglePage(m_anglePage);
    setupDebugPage(m_debugPage);
    m_tabs->addTab(m_controlPage, tr("Control"));
    m_tabs->addTab(m_anglePage, tr("Angle"));
    m_tabs->addTab(m_debugPage, tr("Debug"));

    root->addWidget(m_tabs, 1);
}

void MainWindow::setupDebugPage(QWidget *page)
{
    auto *layout = new QVBoxLayout(page);
    auto *title = new QLabel(tr("Raw CDC lines and parsed angle updates"), page);
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
    m_angleDetailLabel = new QLabel(tr("POS: 0.000000 | MECH: 0.000000 | APP: 0.000000 | IQ: 0.000000"), statusGroup);
    statusLayout->addWidget(new QLabel(tr("Angle deg"), statusGroup), 0, 0);
    statusLayout->addWidget(m_statusAngle, 0, 1);
    statusLayout->addWidget(new QLabel(tr("Link"), statusGroup), 0, 2);
    statusLayout->addWidget(m_statusAngleLink, 0, 3);
    statusLayout->addWidget(m_angleDetailLabel, 1, 0, 1, 4);

    m_angleMeasuredSeries = new QLineSeries(page);
    m_angleMeasuredSeries->setName(tr("Measured Angle"));
    m_angleMechSeries = new QLineSeries(page);
    m_angleMechSeries->setName(tr("Offset Mechanical Angle"));
    m_angleAppSeries = new QLineSeries(page);
    m_angleAppSeries->setName(tr("Electrical Angle"));
    m_angleIqSeries = new QLineSeries(page);
    m_angleIqSeries->setName(tr("Iq"));

    auto *chartBox = new QGroupBox(tr("Angle Trend"), page);
    auto *layout = new QVBoxLayout(chartBox);
    auto *chart = new QChart();
    chart->addSeries(m_angleMeasuredSeries);
    chart->addSeries(m_angleMechSeries);
    chart->addSeries(m_angleAppSeries);
    chart->addSeries(m_angleIqSeries);
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
    m_angleAxisIq = new QValueAxis(chart);
    m_angleAxisIq->setTitleText(tr("Iq (A)"));
    m_angleAxisIq->setRange(-1.0, 1.0);
    m_angleAxisIq->setTickCount(7);
    m_angleAxisIq->setLabelFormat("%.3f");
    chart->addAxis(m_angleAxisIq, Qt::AlignRight);
    m_angleIqSeries->attachAxis(m_angleAxisIq);
    if (m_angleAxisY != nullptr)
    {
        m_angleIqSeries->detachAxis(m_angleAxisY);
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

    root->addWidget(statusGroup);
    root->addWidget(chartBox, 1);

    connect(m_angleTimeSlider, &QSlider::valueChanged, this, [this]() {
        refreshAngleChart();
    });
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
    if (m_angleMeasuredSeries != nullptr)
    {
        m_angleMeasuredSeries->clear();
    }
    m_angleMeasuredPoints.clear();
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

    m_rxLineBuffer.append(QString::fromLocal8Bit(data));
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

    Q_UNUSED(line);
}

bool MainWindow::parseKeyValueFloat(const QString &line, const QString &key, double *value)
{
    const QRegularExpression rx(QStringLiteral("\\b%1=([-+0-9.]+)").arg(QRegularExpression::escape(key)));
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
    double pos = 0.0;
    double mechDeg = 0.0;
    double appDeg = 0.0;
    double iqAmp = 0.0;
    const bool hasPos = parseKeyValueFloat(line, QStringLiteral("POS"), &pos);
    const bool hasMechDeg = parseKeyValueFloat(line, QStringLiteral("MECH"), &mechDeg);
    const bool hasAppDeg = parseKeyValueFloat(line, QStringLiteral("APP"), &appDeg);
    const bool hasIq = parseKeyValueFloat(line, QStringLiteral("IQ"), &iqAmp);
    if (!hasPos || !hasMechDeg || !hasAppDeg || !hasIq)
    {
        return;
    }

    updateAngle(pos, mechDeg, appDeg, iqAmp);
    appendDebug(tr("TEL parsed POS=%1 MECH=%2 APP=%3 IQ=%4")
                    .arg(pos, 0, 'f', 6)
                    .arg(mechDeg, 0, 'f', 6)
                    .arg(appDeg, 0, 'f', 6)
                    .arg(iqAmp, 0, 'f', 6));
    m_lastTelemetryTick = QDateTime::currentMSecsSinceEpoch();
    if (m_statusLink != nullptr)
    {
        m_statusLink->setText(tr("Telemetry OK"));
    }
}

void MainWindow::updateAngle(double measuredDeg, double mechDeg, double appDeg, double iqAmp)
{
    if (m_angleMeasuredSeries == nullptr || m_angleMechSeries == nullptr || m_angleAppSeries == nullptr || m_angleIqSeries == nullptr)
    {
        return;
    }

    m_angleMeasuredPoints.append(QPointF(m_sampleIndex, measuredDeg));
    m_angleMechPoints.append(QPointF(m_sampleIndex, mechDeg));
    m_angleAppPoints.append(QPointF(m_sampleIndex, appDeg));
    m_angleIqPoints.append(QPointF(m_sampleIndex, iqAmp));
    ++m_sampleIndex;

    constexpr int kMaxPoints = 1200;
    while (m_angleMeasuredPoints.size() > kMaxPoints)
    {
        m_angleMeasuredPoints.removeFirst();
        m_angleMechPoints.removeFirst();
        m_angleAppPoints.removeFirst();
        m_angleIqPoints.removeFirst();
    }

    if (m_angleTimeSlider != nullptr)
    {
        const int maxOffset = qMax(0, m_angleMeasuredPoints.size() - m_angleWindowSize);
        m_angleTimeSlider->setMaximum(maxOffset);
        if (m_angleTimeSlider->value() > maxOffset)
        {
            m_angleTimeSlider->setValue(maxOffset);
        }
    }

    refreshAngleChart();

    if (m_statusAngle != nullptr)
    {
        m_statusAngle->setText(fmt6(measuredDeg));
    }
    if (m_angleDetailLabel != nullptr)
    {
        m_angleDetailLabel->setText(tr("POS: %1 | MECH: %2 | APP: %3 | IQ: %4")
                                        .arg(fmt6(measuredDeg), fmt6(mechDeg), fmt6(appDeg), fmt6(iqAmp)));
    }
    if (m_statusAngleLink != nullptr)
    {
        m_statusAngleLink->setText(tr("Telemetry OK"));
    }
}

void MainWindow::refreshAngleChart()
{
    if (m_angleMeasuredSeries == nullptr || m_angleMechSeries == nullptr || m_angleAppSeries == nullptr || m_angleIqSeries == nullptr || m_angleAxisX == nullptr || m_angleAxisY == nullptr || m_angleAxisIq == nullptr)
    {
        return;
    }

    const int count = m_angleMeasuredPoints.size();
    if (count <= 0)
    {
        m_angleMeasuredSeries->clear();
        m_angleMechSeries->clear();
        m_angleAppSeries->clear();
        m_angleIqSeries->clear();
        return;
    }

    const int startIndex = qBound(0, m_angleTimeSlider != nullptr ? m_angleTimeSlider->value() : 0, qMax(0, count - 1));
    const int endIndex = qMin(count, startIndex + m_angleWindowSize);
    const int size = qMax(0, endIndex - startIndex);

    m_angleMeasuredSeries->replace(m_angleMeasuredPoints.mid(startIndex, size));
    m_angleMechSeries->replace(m_angleMechPoints.mid(startIndex, size));
    m_angleAppSeries->replace(m_angleAppPoints.mid(startIndex, size));
    m_angleIqSeries->replace(m_angleIqPoints.mid(startIndex, size));

    if (m_angleMeasuredSeries != nullptr)
    {
        m_angleMeasuredSeries->attachAxis(m_angleAxisX);
        m_angleMeasuredSeries->attachAxis(m_angleAxisY);
    }
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
    if (m_angleIqSeries != nullptr)
    {
        m_angleIqSeries->attachAxis(m_angleAxisX);
        m_angleIqSeries->attachAxis(m_angleAxisIq);
        m_angleIqSeries->detachAxis(m_angleAxisY);
    }

    m_angleAxisX->setRange(startIndex, qMax(startIndex + 1, endIndex - 1));
    m_angleAxisX->setLabelFormat("%.0f");

    m_angleAxisY->setRange(-370.0, 370.0);
    m_angleAxisY->setTickCount(11);
    m_angleAxisY->setLabelFormat("%.1f");
    m_angleAxisY->setReverse(false);

    if (!m_angleIqPoints.isEmpty())
    {
        double minIq = m_angleIqPoints.first().y();
        double maxIq = minIq;
        for (const QPointF &pt : std::as_const(m_angleIqPoints))
        {
            minIq = qMin(minIq, pt.y());
            maxIq = qMax(maxIq, pt.y());
        }

        const double span = qMax(0.05, maxIq - minIq);
        const double margin = qMax(0.05, span * 0.25);
        m_angleAxisIq->setRange(minIq - margin, maxIq + margin);
    }
    else
    {
        m_angleAxisIq->setRange(-1.0, 1.0);
    }
    m_angleAxisIq->setTickCount(7);
    m_angleAxisIq->setLabelFormat("%.3f");
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
}
