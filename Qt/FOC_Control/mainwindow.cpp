#include "mainwindow.h"

#include <QApplication>
#include <QByteArray>
#include <QChart>
#include <QChartView>
#include <QBrush>
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
#include <QPen>
#include <QPointF>
#include <QPushButton>
#include <QPlainTextEdit>
#include <QScrollBar>
#include <QSlider>
#include <QSpinBox>
#include <QMouseEvent>
#include <QToolTip>
#include <QStatusBar>
#include <QTabWidget>
#include <QTextEdit>
#include <QToolButton>
#include <QVBoxLayout>
#include <QValueAxis>
#include <limits>

namespace
{
QString fmt6(double v) { return QString::number(v, 'f', 6); }

double currentPlotTimeSeconds(qint64 *baseMs)
{
    const qint64 now = QDateTime::currentMSecsSinceEpoch();
    if ((baseMs == nullptr) || (*baseMs == 0))
    {
        if (baseMs != nullptr)
        {
            *baseMs = now;
        }
        return 0.0;
    }

    return (double)(now - *baseMs) / 1000.0;
}

constexpr int kCia402ModeProfilePosition = 1;
constexpr int kCia402ModeProfileVelocity = 3;
constexpr int kCia402ModeProfileTorque = 4;
constexpr int kCia402ModeCyclicSyncPosition = 8;
constexpr int kCia402ModeCyclicSyncVelocity = 9;
constexpr int kCia402ModeCyclicSyncTorque = 10;
constexpr double kLiveChartWindowSeconds = 30.0;

int findLiveWindowStart(const QVector<QPointF> &points, double windowSeconds)
{
    if (points.isEmpty())
    {
        return 0;
    }

    const double firstX = points.first().x();
    const double lastX = points.last().x();
    const double startX = qMax(firstX, lastX - windowSeconds);
    int startIndex = 0;
    while ((startIndex + 1) < points.size() && points.at(startIndex).x() < startX)
    {
        ++startIndex;
    }
    return startIndex;
}

QString fmtHex16(unsigned int value)
{
    return QStringLiteral("0x%1").arg(value, 4, 16, QLatin1Char('0')).toUpper();
}

QString decodeCia402State(unsigned int statusword)
{
    if ((statusword & 0x004FU) == 0x0000U)
    {
        return QStringLiteral("Not Ready");
    }
    if ((statusword & 0x004FU) == 0x0040U)
    {
        return QStringLiteral("Switch On Disabled");
    }
    if ((statusword & 0x006FU) == 0x0021U)
    {
        return QStringLiteral("Ready To Switch On");
    }
    if ((statusword & 0x006FU) == 0x0023U)
    {
        return QStringLiteral("Switched On");
    }
    if ((statusword & 0x006FU) == 0x0027U)
    {
        return QStringLiteral("Operation Enabled");
    }
    if ((statusword & 0x006FU) == 0x0007U)
    {
        return QStringLiteral("Quick Stop Active");
    }
    if ((statusword & 0x004FU) == 0x000FU)
    {
        return QStringLiteral("Fault Reaction Active");
    }
    if ((statusword & 0x004FU) == 0x0008U)
    {
        return QStringLiteral("Fault");
    }
    return QStringLiteral("Unknown");
}

QString decodeCia402Mode(unsigned int mode)
{
    switch ((int)mode)
    {
    case kCia402ModeProfilePosition: return QStringLiteral("Profile Position");
    case kCia402ModeProfileVelocity: return QStringLiteral("Profile Velocity");
    case kCia402ModeProfileTorque: return QStringLiteral("Profile Torque");
    case kCia402ModeCyclicSyncPosition: return QStringLiteral("Cyclic Sync Position");
    case kCia402ModeCyclicSyncVelocity: return QStringLiteral("Cyclic Sync Velocity");
    case kCia402ModeCyclicSyncTorque: return QStringLiteral("Cyclic Sync Torque");
    default: return QStringLiteral("Unknown");
    }
}

QString decodeControlMode(unsigned int mode)
{
    switch (mode)
    {
    case 0U: return QStringLiteral("Torque");
    case 1U: return QStringLiteral("Speed");
    case 2U: return QStringLiteral("Position");
    case 3U: return QStringLiteral("Open Loop");
    default: return QStringLiteral("Unknown");
    }
}

QString decodeAxisState(unsigned int state)
{
    switch (state)
    {
    case 0U: return QStringLiteral("Undefined");
    case 1U: return QStringLiteral("Idle");
    case 2U: return QStringLiteral("Offset Calib");
    case 3U: return QStringLiteral("Encoder Calib");
    case 4U: return QStringLiteral("Param Calib");
    case 5U: return QStringLiteral("Run");
    case 6U: return QStringLiteral("Fault Now");
    case 7U: return QStringLiteral("Fault Over");
    case 8U: return QStringLiteral("Current Tune");
    case 9U: return QStringLiteral("Speed Tune");
    default: return QStringLiteral("Unknown");
    }
}

QString decodeParamState(unsigned int state)
{
    switch (state)
    {
    case 0U: return QStringLiteral("Idle");
    case 1U: return QStringLiteral("Prepare");
    case 2U: return QStringLiteral("Lock Check");
    case 3U: return QStringLiteral("Run");
    case 4U: return QStringLiteral("Done");
    case 5U: return QStringLiteral("Fault");
    default: return QStringLiteral("Unknown");
    }
}

QString decodeErrorState(unsigned int err)
{
    if (err == 0U)
    {
        return QStringLiteral("No Error");
    }
    return QStringLiteral("Fault Active");
}

QString formatDecodedUInt(unsigned int value, const QString &desc, bool hexValue = false)
{
    const QString valueText = hexValue ? fmtHex16(value) : QString::number(value);
    return QStringLiteral("%1 (%2)").arg(valueText, desc);
}

static void trimPointBuffer(QVector<QPointF> &points, int maxPoints, int dropChunk = 256)
{
    if (points.size() <= maxPoints)
    {
        return;
    }

    const int excess = points.size() - maxPoints;
    const int dropCount = qMax(excess, dropChunk);
    points.erase(points.begin(), points.begin() + qMin(dropCount, points.size()));
}
}

MainWindow::ChartView::ChartView(QWidget *parent)
    : QChartView(parent)
{
    setMouseTracking(true);
    setRubberBand(QChartView::NoRubberBand);
}

void MainWindow::ChartView::mousePressEvent(QMouseEvent *event)
{
    QChartView::mousePressEvent(event);
    if (event->button() == Qt::LeftButton)
    {
        QMetaObject::invokeMethod(this, [this, eventPos = event->pos()]() {
            if (chart() == nullptr)
            {
                return;
            }

            const QPointF scenePos = mapToScene(eventPos);
            const QList<QAbstractSeries *> seriesList = chart()->series();
            qreal bestDist2 = std::numeric_limits<qreal>::max();
            QString bestText;

            for (QAbstractSeries *abstractSeries : seriesList)
            {
                auto *series = qobject_cast<QLineSeries *>(abstractSeries);
                if (series == nullptr || !series->isVisible())
                {
                    continue;
                }

                const QVector<QPointF> points = series->pointsVector();
                for (const QPointF &point : points)
                {
                    const QPointF scenePoint = chart()->mapToPosition(point, series);
                    const qreal dx = scenePoint.x() - scenePos.x();
                    const qreal dy = scenePoint.y() - scenePos.y();
                    const qreal dist2 = (dx * dx) + (dy * dy);
                    if (dist2 < bestDist2)
                    {
                        bestDist2 = dist2;
                        bestText = QStringLiteral("%1\nx=%2\ny=%3")
                                       .arg(series->name(),
                                            QString::number(point.x(), 'f', 0),
                                            QString::number(point.y(), 'f', 3));
                    }
                }
            }

            if (!bestText.isEmpty())
            {
                QToolTip::showText(mapToGlobal(eventPos), bestText, this);
            }
        }, Qt::QueuedConnection);
    }
}

void MainWindow::ChartView::mouseMoveEvent(QMouseEvent *event)
{
    QChartView::mouseMoveEvent(event);
    if (chart() != nullptr)
    {
        const QPointF scenePos = mapToScene(event->pos());
        const QList<QAbstractSeries *> seriesList = chart()->series();
        qreal bestDist2 = std::numeric_limits<qreal>::max();
        QString bestText;

        for (QAbstractSeries *abstractSeries : seriesList)
        {
            auto *series = qobject_cast<QLineSeries *>(abstractSeries);
            if (series == nullptr || !series->isVisible())
            {
                continue;
            }

            const QVector<QPointF> points = series->pointsVector();
            for (const QPointF &point : points)
            {
                const QPointF scenePoint = chart()->mapToPosition(point, series);
                const qreal dx = scenePoint.x() - scenePos.x();
                const qreal dy = scenePoint.y() - scenePos.y();
                const qreal dist2 = (dx * dx) + (dy * dy);
                if (dist2 < bestDist2)
                {
                    bestDist2 = dist2;
                    bestText = QStringLiteral("%1\nx=%2\ny=%3")
                                   .arg(series->name(),
                                        QString::number(point.x(), 'f', 0),
                                        QString::number(point.y(), 'f', 3));
                }
            }
        }

        if (!bestText.isEmpty())
        {
            QToolTip::showText(mapToGlobal(event->pos()), bestText, this);
        }
    }
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
        if ((now - m_lastChartRefreshTick) >= 50)
        {
            refreshAngleChart();
            refreshSpeedChart();
            refreshIqRefChart();
            refreshSpeedMeasChart();
            m_lastChartRefreshTick = now;
        }
        if ((m_lastTelemetryTick != 0) && ((now - m_lastTelemetryTick) > 1000))
        {
            m_statusLink->setText(tr("Waiting telemetry..."));
        }
    });
    connect(m_tabs, &QTabWidget::currentChanged, this, [this]() {
        if (m_tabs == nullptr)
        {
            return;
        }

        const QWidget *page = m_tabs->currentWidget();
        if (page == m_anglePage)
        {
            refreshAngleChart();
        }
        else if (page == m_speedPage)
        {
            refreshSpeedChart();
        }
        else if (page == m_iqRefPage)
        {
            refreshIqRefChart();
        }
        else if (page == m_speedMeasPage)
        {
            refreshSpeedMeasChart();
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
    m_iqRefPage = new QWidget(m_tabs);
    m_speedMeasPage = new QWidget(m_tabs);
    m_debugPage = new QWidget(m_tabs);
    setupControlPage(m_controlPage);
    setupAnglePage(m_anglePage);
    setupSpeedPage(m_speedPage);
    setupIqRefPage(m_iqRefPage);
    setupSpeedMeasPage(m_speedMeasPage);
    setupDebugPage(m_debugPage);
    m_tabs->addTab(m_controlPage, tr("Device"));
    m_tabs->addTab(m_anglePage, tr("Angle"));
    m_tabs->addTab(m_speedPage, tr("Speed"));
    m_tabs->addTab(m_iqRefPage, tr("Iq Compare"));
    m_tabs->addTab(m_speedMeasPage, tr("Speed Meas"));
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
    const auto normalizeConnectionWidget = [](QWidget *widget) {
        if (widget == nullptr)
        {
            return;
        }
        widget->setMinimumHeight(24);
        widget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    };
    const QList<QWidget *> connectionWidgets = {
        m_portCombo, m_refreshButton, m_openButton, m_baudCombo,
        m_dataBitsCombo, m_parityCombo, m_stopBitsCombo, m_flowCombo,
        m_hexSendCheck, m_hexViewCheck, m_autoScrollCheck, m_autoTxNewlineCheck
    };
    for (QWidget *widget : connectionWidgets)
    {
        normalizeConnectionWidget(widget);
    }
    m_refreshButton->setToolTip(tr("Refresh the local serial port list."));
    m_openButton->setToolTip(tr("Open or close the selected CDC port."));
    m_portCombo->setToolTip(tr("Select the USB CDC port exposed by the controller."));
    m_baudCombo->setToolTip(tr("Default serial baud rate for this tool."));
    m_hexSendCheck->setToolTip(tr("Send manual Tx text as hex bytes instead of ASCII."));
    m_hexViewCheck->setToolTip(tr("Show received serial data in hex format."));
    m_autoScrollCheck->setToolTip(tr("Keep the receive log pinned to the newest line."));
    m_autoTxNewlineCheck->setToolTip(tr("Append newline automatically when sending manual commands."));

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

    auto *presetGroup = new QGroupBox(tr("Maintenance"), page);
    auto *presetGrid = new QGridLayout(presetGroup);
    m_kpSpin = new QDoubleSpinBox(presetGroup);
    m_kpSpin->setRange(0.0, 1000.0);
    m_kpSpin->setDecimals(4);
    m_kpSpin->setSingleStep(0.01);
    m_kpSpin->setValue(0.39);

    m_kiSpin = new QDoubleSpinBox(presetGroup);
    m_kiSpin->setRange(0.0, 10000.0);
    m_kiSpin->setDecimals(4);
    m_kiSpin->setSingleStep(0.5);
    m_kiSpin->setValue(0.874);

    m_nodeIdSpin = new QSpinBox(presetGroup);
    m_nodeIdSpin->setRange(0, 15);
    m_nodeIdSpin->setValue(1);

    m_setKpButton = new QPushButton(tr("Set Kp"), presetGroup);
    m_setKiButton = new QPushButton(tr("Set Ki"), presetGroup);
    m_getIdButton = new QPushButton(tr("Get ID"), presetGroup);
    m_setIdButton = new QPushButton(tr("Set ID (RAM)"), presetGroup);
    m_startButton = new QPushButton(tr("Quick Start"), presetGroup);
    m_stopButton = new QPushButton(tr("Quick Stop"), presetGroup);
    m_startRsTuneButton = new QPushButton(tr("Start Current Tune"), presetGroup);
    m_startFullTuneButton = new QPushButton(tr("Measure Params"), presetGroup);
    m_stopCalibButton = new QPushButton(tr("Stop Tune"), presetGroup);
    m_readFlashButton = new QPushButton(tr("Read Flash"), presetGroup);
    m_clearFlashButton = new QPushButton(tr("Clear Flash"), presetGroup);
    m_readFlashButton->setEnabled(false);
    m_clearFlashButton->setEnabled(false);
    m_readFlashButton->setToolTip(tr("Flash persistence is disabled in the current firmware."));
    m_clearFlashButton->setToolTip(tr("Flash persistence is disabled in the current firmware."));
    m_setIdButton->setToolTip(tr("Applies the node ID in RAM only. It is not saved to flash."));
    m_startRsTuneButton->setToolTip(tr("Starts the current autotune chain only."));
    m_startFullTuneButton->setToolTip(tr("Starts the full motor-parameter measurement chain."));
    m_stopCalibButton->setToolTip(tr("Stops the active tuning flow."));
    m_kpSpin->setToolTip(tr("Speed loop Kp in the current firmware unit."));
    m_kiSpin->setToolTip(tr("Speed loop Ki in the current firmware unit."));
    m_startButton->setToolTip(tr("Quick shortcut for CiA402 start controlword."));
    m_stopButton->setToolTip(tr("Quick shortcut for CiA402 stop controlword."));
    m_getIdButton->setToolTip(tr("Read the current node ID from the device."));
    m_setKpButton->setToolTip(tr("Write the speed-loop Kp parameter to the running device."));
    m_setKiButton->setToolTip(tr("Write the speed-loop Ki parameter to the running device."));

    presetGrid->addWidget(new QLabel(tr("Speed Kp"), presetGroup), 0, 0);
    presetGrid->addWidget(m_kpSpin, 0, 1);
    presetGrid->addWidget(m_setKpButton, 0, 2);
    presetGrid->addWidget(new QLabel(tr("Speed Ki"), presetGroup), 1, 0);
    presetGrid->addWidget(m_kiSpin, 1, 1);
    presetGrid->addWidget(m_setKiButton, 1, 2);
    presetGrid->addWidget(new QLabel(tr("Node ID"), presetGroup), 2, 0);
    presetGrid->addWidget(m_nodeIdSpin, 2, 1);
    presetGrid->addWidget(m_getIdButton, 2, 2);
    presetGrid->addWidget(new QLabel(tr("Node ID apply"), presetGroup), 3, 0);
    presetGrid->addWidget(m_setIdButton, 3, 1, 1, 2);
    presetGrid->addWidget(new QLabel(tr("Quick motion"), presetGroup), 4, 0);
    presetGrid->addWidget(m_startButton, 4, 1);
    presetGrid->addWidget(m_stopButton, 4, 2);
    presetGrid->addWidget(m_startRsTuneButton, 5, 0);
    presetGrid->addWidget(m_startFullTuneButton, 5, 1);
    presetGrid->addWidget(m_stopCalibButton, 5, 2);
    presetGrid->addWidget(m_readFlashButton, 6, 0);
    presetGrid->addWidget(m_clearFlashButton, 6, 1);

    const auto normalizeMaintenanceWidget = [](QWidget *widget) {
        if (widget == nullptr)
        {
            return;
        }
        widget->setMinimumHeight(24);
        widget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    };
    const QList<QWidget *> maintenanceWidgets = {
        m_kpSpin, m_kiSpin, m_nodeIdSpin, m_setKpButton, m_setKiButton,
        m_getIdButton, m_setIdButton, m_startButton, m_stopButton,
        m_startRsTuneButton, m_startFullTuneButton, m_stopCalibButton,
        m_readFlashButton, m_clearFlashButton
    };
    for (QWidget *widget : maintenanceWidgets)
    {
        normalizeMaintenanceWidget(widget);
    }
    presetGrid->setColumnMinimumWidth(0, 120);
    presetGrid->setColumnMinimumWidth(1, 120);
    presetGrid->setColumnMinimumWidth(2, 120);

    auto *cia402Panel = new QWidget(page);
    setupCia402Page(cia402Panel);

    auto *ioRow = new QHBoxLayout();
    m_sendEdit = new QLineEdit(page);
    m_sendButton = new QPushButton(tr("Send"), page);
    m_sendEdit->setPlaceholderText(tr("Manual text command, for example: cia402 status"));
    m_sendEdit->setToolTip(tr("Manual command entry. Useful for protocol debugging or temporary commands not mapped to buttons."));
    m_sendButton->setToolTip(tr("Send the manual text command to the device."));
    ioRow->addWidget(new QLabel(tr("Tx"), page));
    ioRow->addWidget(m_sendEdit, 1);
    ioRow->addWidget(m_sendButton);

    m_logEdit = new QTextEdit(page);
    m_logEdit->setReadOnly(true);

    auto *bodyRow = new QHBoxLayout();
    bodyRow->setSpacing(12);
    presetGroup->setMaximumWidth(460);
    cia402Panel->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);

    const int bodyPanelHeight = qMax(presetGroup->sizeHint().height(), cia402Panel->sizeHint().height());
    presetGroup->setFixedHeight(bodyPanelHeight);
    cia402Panel->setFixedHeight(bodyPanelHeight);
    const QList<QGroupBox *> cia402Groups =
        cia402Panel->findChildren<QGroupBox *>(QString(), Qt::FindDirectChildrenOnly);
    for (QGroupBox *group : cia402Groups)
    {
        group->setFixedHeight(bodyPanelHeight);
    }

    bodyRow->addWidget(presetGroup, 0, Qt::AlignTop);
    bodyRow->addWidget(cia402Panel, 1, Qt::AlignTop);

    root->addWidget(connGroup);
    root->addLayout(bodyRow);
    root->addLayout(ioRow);
    root->addWidget(m_logEdit, 1);

    connect(m_refreshButton, &QPushButton::clicked, this, &MainWindow::refreshPorts);
    connect(m_openButton, &QPushButton::clicked, this, &MainWindow::openClosePort);
    connect(m_sendButton, &QPushButton::clicked, this, &MainWindow::sendData);
    connect(m_sendEdit, &QLineEdit::returnPressed, this, &MainWindow::sendData);

    connect(m_setKpButton, &QPushButton::clicked, this, [this]() { sendPresetCommand(QStringLiteral("sp %1").arg(m_kpSpin->value(), 0, 'f', 4)); });
    connect(m_setKiButton, &QPushButton::clicked, this, [this]() { sendPresetCommand(QStringLiteral("si %1").arg(m_kiSpin->value(), 0, 'f', 4)); });
    connect(m_getIdButton, &QPushButton::clicked, this, [this]() { sendPresetCommand("nodeget"); });
    connect(m_setIdButton, &QPushButton::clicked, this, [this]() { sendPresetCommand(QStringLiteral("nodeset %1").arg(m_nodeIdSpin->value())); });
    connect(m_startButton, &QPushButton::clicked, this, [this]() { sendPresetCommand("cia402 cw 15"); });
    connect(m_stopButton, &QPushButton::clicked, this, [this]() { sendPresetCommand("cia402 cw 0"); });
    connect(m_startRsTuneButton, &QPushButton::clicked, this, [this]() { sendPresetCommand("calib 0"); });
    connect(m_startFullTuneButton, &QPushButton::clicked, this, [this]() { sendPresetCommand("paramstart"); });
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
        m_angleAxisX->setTitleText(tr("Time (s)"));
    }
    if (m_angleAxisY != nullptr)
    {
        m_angleAxisY->setTitleText(tr("Angle deg"));
        m_angleAxisY->setRange(0.0, 360.0);
        m_angleAxisY->setTickCount(11);
        m_angleAxisY->setLabelFormat("%.1f");
        m_angleAxisY->setReverse(false);
    }
    chart->legend()->setVisible(true);
    chart->setTitle(tr("Rotor Angle Compare"));
    m_angleChartView = new ChartView(chartBox);
    m_angleChartView->setChart(chart);
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
    statusLayout->addWidget(new QLabel(tr("Speed rpm"), statusGroup), 0, 0);
    statusLayout->addWidget(m_statusSpeed, 0, 1);
    statusLayout->addWidget(new QLabel(tr("Link"), statusGroup), 0, 2);
    statusLayout->addWidget(m_speedLink, 0, 3);

    auto *chartBox = new QGroupBox(tr("Speed Trend"), page);
    auto *layout = new QVBoxLayout(chartBox);
    auto *chart = new QChart();
    m_speedSeries = new QLineSeries(page);
    m_speedSeries->setName(tr("Speed RPM"));
    m_speedRefSeries = new QLineSeries(page);
    m_speedRefSeries->setName(tr("Speed Ref RPM"));
    chart->addSeries(m_speedSeries);
    chart->addSeries(m_speedRefSeries);
    chart->createDefaultAxes();
    m_speedAxisX = qobject_cast<QValueAxis *>(chart->axes(Qt::Horizontal).first());
    m_speedAxisY = qobject_cast<QValueAxis *>(chart->axes(Qt::Vertical).first());
    if (m_speedAxisX != nullptr)
    {
        m_speedAxisX->setTitleText(tr("Time (s)"));
    }
    if (m_speedAxisY != nullptr)
    {
        m_speedAxisY->setTitleText(tr("RPM"));
        m_speedAxisY->setRange(-3000.0, 3000.0);
        m_speedAxisY->setTickCount(11);
        m_speedAxisY->setLabelFormat("%.0f");
    }
    chart->legend()->setVisible(true);
    chart->setTitle(tr("Speed Trend"));
    m_speedChartView = new ChartView(chartBox);
    m_speedChartView->setChart(chart);
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

void MainWindow::setupIqRefPage(QWidget *page)
{
    auto *root = new QVBoxLayout(page);

    auto *statusGroup = new QGroupBox(tr("Iq Compare"), page);
    auto *statusLayout = new QGridLayout(statusGroup);
    m_iqRawStatus = new QLabel(tr("0.000000"), statusGroup);
    m_iqRawLink = new QLabel(tr("Waiting telemetry..."), statusGroup);
    m_iqRefStatus = new QLabel(tr("0.000000"), statusGroup);
    m_iqRefLink = new QLabel(tr("Waiting telemetry..."), statusGroup);
    m_iqRefDetailLabel = new QLabel(tr("Iq_ref: 0.000000 A"), statusGroup);
    m_iqRefPeakLabel = new QLabel(tr("IQR: [0.000000, 0.000000] A"), statusGroup);
    m_iqIdStatus = new QLabel(tr("0.000000"), statusGroup);
    m_iqIdLink = new QLabel(tr("Waiting telemetry..."), statusGroup);
    m_iqIdDetailLabel = new QLabel(tr("Id: 0.000000 A"), statusGroup);
    m_iqIdPeakLabel = new QLabel(tr("Id: [0.000000, 0.000000] A"), statusGroup);
    statusLayout->addWidget(new QLabel(tr("Iq_raw"), statusGroup), 0, 0);
    statusLayout->addWidget(m_iqRawStatus, 0, 1);
    statusLayout->addWidget(new QLabel(tr("Link"), statusGroup), 0, 2);
    statusLayout->addWidget(m_iqRawLink, 0, 3);
    statusLayout->addWidget(new QLabel(tr("Iq_ref"), statusGroup), 1, 0);
    statusLayout->addWidget(m_iqRefStatus, 1, 1);
    statusLayout->addWidget(new QLabel(tr("Link"), statusGroup), 1, 2);
    statusLayout->addWidget(m_iqRefLink, 1, 3);
    statusLayout->addWidget(new QLabel(tr("Id"), statusGroup), 2, 0);
    statusLayout->addWidget(m_iqIdStatus, 2, 1);
    statusLayout->addWidget(new QLabel(tr("Link"), statusGroup), 2, 2);
    statusLayout->addWidget(m_iqIdLink, 2, 3);
    statusLayout->addWidget(m_iqRefDetailLabel, 3, 0, 1, 4);
    statusLayout->addWidget(m_iqIdDetailLabel, 4, 0, 1, 4);
    statusLayout->addWidget(m_iqRefPeakLabel, 5, 0, 1, 4);
    statusLayout->addWidget(m_iqIdPeakLabel, 6, 0, 1, 4);

    auto *chartBox = new QGroupBox(tr("Iq Compare Trend"), page);
    auto *layout = new QVBoxLayout(chartBox);
    auto *chart = new QChart();
    m_iqRawSeries = new QLineSeries(page);
    m_iqRawSeries->setName(tr("Iq_raw"));
    chart->addSeries(m_iqRawSeries);
    m_iqRefSeries = new QLineSeries(page);
    m_iqRefSeries->setName(tr("Iq_ref"));
    chart->addSeries(m_iqRefSeries);
    m_iqIdSeries = new QLineSeries(page);
    m_iqIdSeries->setName(tr("Id"));
    chart->addSeries(m_iqIdSeries);
    chart->createDefaultAxes();
    m_iqRefAxisX = qobject_cast<QValueAxis *>(chart->axes(Qt::Horizontal).first());
    m_iqRefAxisY = qobject_cast<QValueAxis *>(chart->axes(Qt::Vertical).first());
    if (m_iqRefAxisX != nullptr)
    {
        m_iqRefAxisX->setTitleText(tr("Time (s)"));
    }
    if (m_iqRefAxisY != nullptr)
    {
        m_iqRefAxisY->setTitleText(tr("Iq (A)"));
        m_iqRefAxisY->setRange(-2.5, 2.5);
        m_iqRefAxisY->setTickCount(11);
        m_iqRefAxisY->setLabelFormat("%.3f");
    }
    chart->legend()->setVisible(true);
    chart->setTitle(tr("Iq Compare Trend"));
    m_iqRefChartView = new ChartView(chartBox);
    m_iqRefChartView->setChart(chart);
    m_iqRefChartView->setRenderHint(QPainter::Antialiasing);
    layout->addWidget(m_iqRefChartView);

    m_iqRefTimeSlider = new QSlider(Qt::Horizontal, chartBox);
    m_iqRefTimeSlider->setRange(0, 0);
    m_iqRefTimeSlider->setSingleStep(1);
    m_iqRefTimeSlider->setPageStep(20);
    layout->addWidget(m_iqRefTimeSlider);

    auto *zoomRow = new QHBoxLayout();
    m_iqRefZoomOutButton = new QToolButton(chartBox);
    m_iqRefZoomOutButton->setText(tr("-"));
    m_iqRefZoomInButton = new QToolButton(chartBox);
    m_iqRefZoomInButton->setText(tr("+"));
    m_iqRefZoomResetButton = new QToolButton(chartBox);
    m_iqRefZoomResetButton->setText(tr("Reset"));
    zoomRow->addWidget(new QLabel(tr("Zoom"), chartBox));
    zoomRow->addWidget(m_iqRefZoomOutButton);
    zoomRow->addWidget(m_iqRefZoomInButton);
    zoomRow->addWidget(m_iqRefZoomResetButton);
    zoomRow->addStretch(1);
    layout->addLayout(zoomRow);

    root->addWidget(statusGroup);
    root->addWidget(chartBox, 1);

    connect(m_iqRefTimeSlider, &QSlider::valueChanged, this, [this]() {
        if (!m_iqRefSliderDragging)
        {
            refreshIqRefChart();
        }
    });
    connect(m_iqRefTimeSlider, &QSlider::sliderPressed, this, [this]() {
        m_iqRefSliderDragging = true;
        m_iqRefAutoFollow = false;
    });
    connect(m_iqRefTimeSlider, &QSlider::sliderReleased, this, [this]() {
        m_iqRefSliderDragging = false;
        refreshIqRefChart();
    });
    connect(m_iqRefZoomInButton, &QToolButton::clicked, this, [this]() { zoomIqRefWindow(-300); });
    connect(m_iqRefZoomOutButton, &QToolButton::clicked, this, [this]() { zoomIqRefWindow(300); });
    connect(m_iqRefZoomResetButton, &QToolButton::clicked, this, [this]() { resetIqRefWindow(); });
}

void MainWindow::setupSpeedMeasPage(QWidget *page)
{
    auto *root = new QVBoxLayout(page);

    auto *statusGroup = new QGroupBox(tr("Speed Meas Value"), page);
    auto *statusLayout = new QGridLayout(statusGroup);
    m_speedMeasStatus = new QLabel(tr("0.000000"), statusGroup);
    m_speedMeasLink = new QLabel(tr("Waiting telemetry..."), statusGroup);
    m_speedMeasDetailLabel = new QLabel(tr("SpeedMeas: 0.000000 rpm | SpeedErr: 0.000000 rpm"), statusGroup);
    m_speedMeasPeakLabel = new QLabel(tr("SpeedMeas: [0.000000, 0.000000] rpm | SpeedErr: [0.000000, 0.000000] rpm"), statusGroup);
    statusLayout->addWidget(new QLabel(tr("Speed Meas"), statusGroup), 0, 0);
    statusLayout->addWidget(m_speedMeasStatus, 0, 1);
    statusLayout->addWidget(new QLabel(tr("Link"), statusGroup), 0, 2);
    statusLayout->addWidget(m_speedMeasLink, 0, 3);
    statusLayout->addWidget(m_speedMeasDetailLabel, 1, 0, 1, 4);
    statusLayout->addWidget(m_speedMeasPeakLabel, 2, 0, 1, 4);

    auto *chartBox = new QGroupBox(tr("Speed Meas Trend"), page);
    auto *layout = new QVBoxLayout(chartBox);
    auto *chart = new QChart();
    m_speedMeasSeries = new QLineSeries(page);
    m_speedMeasSeries->setName(tr("speedMeas_rpm"));
    m_speedErrSeries = new QLineSeries(page);
    m_speedErrSeries->setName(tr("speedError_rpm"));
    chart->addSeries(m_speedMeasSeries);
    chart->addSeries(m_speedErrSeries);
    chart->createDefaultAxes();
    m_speedMeasAxisX = qobject_cast<QValueAxis *>(chart->axes(Qt::Horizontal).first());
    m_speedMeasAxisY = qobject_cast<QValueAxis *>(chart->axes(Qt::Vertical).first());
    if (m_speedMeasAxisX != nullptr)
    {
        m_speedMeasAxisX->setTitleText(tr("Time (s)"));
    }
    if (m_speedMeasAxisY != nullptr)
    {
        m_speedMeasAxisY->setTitleText(tr("speedMeas_rpm"));
        m_speedMeasAxisY->setRange(-50.0, 50.0);
        m_speedMeasAxisY->setTickCount(9);
        m_speedMeasAxisY->setLabelFormat("%.0f");
    }
    m_speedErrAxis = new QValueAxis(chart);
    m_speedErrAxis->setTitleText(tr("speedError_rpm"));
    m_speedErrAxis->setRange(-50.0, 50.0);
    m_speedErrAxis->setTickCount(9);
    m_speedErrAxis->setLabelFormat("%.0f");
    chart->addAxis(m_speedErrAxis, Qt::AlignRight);
    m_speedErrSeries->attachAxis(m_speedErrAxis);
    if (m_speedMeasAxisY != nullptr)
    {
        m_speedErrSeries->detachAxis(m_speedMeasAxisY);
    }
    chart->legend()->setVisible(true);
    chart->setTitle(tr("Speed Measurement / Error (rpm)"));
    m_speedMeasChartView = new ChartView(chartBox);
    m_speedMeasChartView->setChart(chart);
    m_speedMeasChartView->setRenderHint(QPainter::Antialiasing);
    layout->addWidget(m_speedMeasChartView);

    m_speedMeasTimeSlider = new QSlider(Qt::Horizontal, chartBox);
    m_speedMeasTimeSlider->setRange(0, 0);
    m_speedMeasTimeSlider->setSingleStep(1);
    m_speedMeasTimeSlider->setPageStep(20);
    layout->addWidget(m_speedMeasTimeSlider);

    auto *zoomRow = new QHBoxLayout();
    m_speedMeasZoomOutButton = new QToolButton(chartBox);
    m_speedMeasZoomOutButton->setText(tr("-"));
    m_speedMeasZoomInButton = new QToolButton(chartBox);
    m_speedMeasZoomInButton->setText(tr("+"));
    m_speedMeasZoomResetButton = new QToolButton(chartBox);
    m_speedMeasZoomResetButton->setText(tr("Reset"));
    zoomRow->addWidget(new QLabel(tr("Zoom"), chartBox));
    zoomRow->addWidget(m_speedMeasZoomOutButton);
    zoomRow->addWidget(m_speedMeasZoomInButton);
    zoomRow->addWidget(m_speedMeasZoomResetButton);
    zoomRow->addStretch(1);
    layout->addLayout(zoomRow);

    root->addWidget(statusGroup);
    root->addWidget(chartBox, 1);

    connect(m_speedMeasTimeSlider, &QSlider::valueChanged, this, [this]() {
        if (!m_speedMeasSliderDragging)
        {
            refreshSpeedMeasChart();
        }
    });
    connect(m_speedMeasTimeSlider, &QSlider::sliderPressed, this, [this]() {
        m_speedMeasSliderDragging = true;
        m_speedMeasAutoFollow = false;
    });
    connect(m_speedMeasTimeSlider, &QSlider::sliderReleased, this, [this]() {
        m_speedMeasSliderDragging = false;
        refreshSpeedMeasChart();
    });
    connect(m_speedMeasZoomInButton, &QToolButton::clicked, this, [this]() { zoomSpeedMeasWindow(-300); });
    connect(m_speedMeasZoomOutButton, &QToolButton::clicked, this, [this]() { zoomSpeedMeasWindow(300); });
    connect(m_speedMeasZoomResetButton, &QToolButton::clicked, this, [this]() { resetSpeedMeasWindow(); });
}

void MainWindow::setupCia402Page(QWidget *page)
{
    auto *root = new QHBoxLayout(page);
    root->setSpacing(12);

    auto *statusGroup = new QGroupBox(tr("CiA402 Status"), page);
    auto *statusLayout = new QGridLayout(statusGroup);
    m_cia402StatusWordLabel = new QLabel(tr("0x0000"), statusGroup);
    m_cia402ModeLabel = new QLabel(tr("0"), statusGroup);
    m_cia402AxisStateLabel = new QLabel(tr("0"), statusGroup);
    m_cia402ErrorLabel = new QLabel(tr("0"), statusGroup);
    m_cia402ControlModeLabel = new QLabel(tr("0"), statusGroup);
    m_cia402ParamStateLabel = new QLabel(tr("0"), statusGroup);
    m_cia402SpeedRefLabel = new QLabel(tr("0.000000"), statusGroup);
    m_cia402TorqueRefLabel = new QLabel(tr("0.000000"), statusGroup);
    m_paramValidLabel = new QLabel(tr("0x00"), statusGroup);
    m_paramRsLabel = new QLabel(tr("0.000000"), statusGroup);
    m_paramLdLabel = new QLabel(tr("0.000000"), statusGroup);
    m_paramLqLabel = new QLabel(tr("0.000000"), statusGroup);
    m_paramKeLabel = new QLabel(tr("0.000000"), statusGroup);

    statusLayout->addWidget(new QLabel(tr("Statusword"), statusGroup), 0, 0);
    statusLayout->addWidget(m_cia402StatusWordLabel, 0, 1);
    statusLayout->addWidget(new QLabel(tr("Mode"), statusGroup), 1, 0);
    statusLayout->addWidget(m_cia402ModeLabel, 1, 1);
    statusLayout->addWidget(new QLabel(tr("Axis"), statusGroup), 2, 0);
    statusLayout->addWidget(m_cia402AxisStateLabel, 2, 1);
    statusLayout->addWidget(new QLabel(tr("Error"), statusGroup), 3, 0);
    statusLayout->addWidget(m_cia402ErrorLabel, 3, 1);
    statusLayout->addWidget(new QLabel(tr("CtrlMode"), statusGroup), 4, 0);
    statusLayout->addWidget(m_cia402ControlModeLabel, 4, 1);
    statusLayout->addWidget(new QLabel(tr("Param"), statusGroup), 5, 0);
    statusLayout->addWidget(m_cia402ParamStateLabel, 5, 1);
    statusLayout->addWidget(new QLabel(tr("Speed Ref"), statusGroup), 6, 0);
    statusLayout->addWidget(m_cia402SpeedRefLabel, 6, 1);
    statusLayout->addWidget(new QLabel(tr("Torque Ref"), statusGroup), 7, 0);
    statusLayout->addWidget(m_cia402TorqueRefLabel, 7, 1);
    statusLayout->addWidget(new QLabel(tr("Param Valid"), statusGroup), 8, 0);
    statusLayout->addWidget(m_paramValidLabel, 8, 1);
    statusLayout->addWidget(new QLabel(tr("Rs"), statusGroup), 9, 0);
    statusLayout->addWidget(m_paramRsLabel, 9, 1);
    statusLayout->addWidget(new QLabel(tr("Ld"), statusGroup), 10, 0);
    statusLayout->addWidget(m_paramLdLabel, 10, 1);
    statusLayout->addWidget(new QLabel(tr("Lq"), statusGroup), 11, 0);
    statusLayout->addWidget(m_paramLqLabel, 11, 1);
    statusLayout->addWidget(new QLabel(tr("Ke"), statusGroup), 12, 0);
    statusLayout->addWidget(m_paramKeLabel, 12, 1);
    statusLayout->setColumnMinimumWidth(0, 90);
    statusLayout->setColumnStretch(1, 1);

    auto *cmdGroup = new QGroupBox(tr("CiA402 Commands"), page);
    auto *cmdGrid = new QGridLayout(cmdGroup);
    m_cia402ControlWordSpin = new QSpinBox(cmdGroup);
    m_cia402ControlWordSpin->setRange(0, 65535);
    m_cia402ControlWordSpin->setDisplayIntegerBase(16);
    m_cia402ControlWordSpin->setPrefix(QStringLiteral("0x"));
    m_cia402ControlWordSpin->setValue(0x000F);

    m_cia402ModeCombo = new QComboBox(cmdGroup);
    m_cia402ModeCombo->addItem(tr("Profile Position"), kCia402ModeProfilePosition);
    m_cia402ModeCombo->addItem(tr("Profile Velocity"), kCia402ModeProfileVelocity);
    m_cia402ModeCombo->addItem(tr("Profile Torque"), kCia402ModeProfileTorque);
    m_cia402ModeCombo->addItem(tr("Cyclic Sync Position"), kCia402ModeCyclicSyncPosition);
    m_cia402ModeCombo->addItem(tr("Cyclic Sync Velocity"), kCia402ModeCyclicSyncVelocity);
    m_cia402ModeCombo->addItem(tr("Cyclic Sync Torque"), kCia402ModeCyclicSyncTorque);

    m_cia402SpeedRefSpin = new QDoubleSpinBox(cmdGroup);
    m_cia402SpeedRefSpin->setRange(-10000.0, 10000.0);
    m_cia402SpeedRefSpin->setDecimals(2);
    m_cia402SpeedRefSpin->setSingleStep(10.0);
    m_cia402SpeedRefSpin->setValue(100.0);

    m_cia402TorqueRefSpin = new QDoubleSpinBox(cmdGroup);
    m_cia402TorqueRefSpin->setRange(-50.0, 50.0);
    m_cia402TorqueRefSpin->setDecimals(3);
    m_cia402TorqueRefSpin->setSingleStep(0.1);
    m_cia402TorqueRefSpin->setValue(0.5);

    m_cia402SendCwButton = new QPushButton(tr("Send CW"), cmdGroup);
    m_cia402SendModeButton = new QPushButton(tr("Send Mode"), cmdGroup);
    m_cia402SendSpeedButton = new QPushButton(tr("Send Speed"), cmdGroup);
    m_cia402SendTorqueButton = new QPushButton(tr("Send Torque"), cmdGroup);
    m_cia402QueryButton = new QPushButton(tr("Query Status"), cmdGroup);
    m_paramQueryButton = new QPushButton(tr("Query Params"), cmdGroup);
    auto *faultButton = new QPushButton(tr("Fault Reset"), cmdGroup);
    auto *shutdownButton = new QPushButton(tr("Shutdown"), cmdGroup);
    auto *switchOnButton = new QPushButton(tr("Switch On"), cmdGroup);
    auto *enableOpButton = new QPushButton(tr("Enable Op"), cmdGroup);

    const auto normalizeCia402Widget = [](QWidget *widget, int minimumWidth) {
        if (widget == nullptr)
        {
            return;
        }
        widget->setMinimumSize(minimumWidth, 24);
        widget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    };
    normalizeCia402Widget(m_cia402ControlWordSpin, 180);
    normalizeCia402Widget(m_cia402ModeCombo, 180);
    normalizeCia402Widget(m_cia402SpeedRefSpin, 180);
    normalizeCia402Widget(m_cia402TorqueRefSpin, 180);
    const QList<QWidget *> cia402Buttons = {
        m_cia402SendCwButton, m_cia402SendModeButton, m_cia402SendSpeedButton,
        m_cia402SendTorqueButton, m_cia402QueryButton, m_paramQueryButton,
        faultButton, shutdownButton, switchOnButton, enableOpButton
    };
    for (QWidget *button : cia402Buttons)
    {
        normalizeCia402Widget(button, 140);
    }

    cmdGrid->addWidget(new QLabel(tr("Controlword"), cmdGroup), 0, 0);
    cmdGrid->addWidget(m_cia402ControlWordSpin, 0, 1);
    cmdGrid->addWidget(m_cia402SendCwButton, 0, 2);
    cmdGrid->addWidget(faultButton, 0, 3);
    cmdGrid->addWidget(shutdownButton, 1, 0);
    cmdGrid->addWidget(switchOnButton, 1, 1);
    cmdGrid->addWidget(enableOpButton, 1, 2);
    cmdGrid->addWidget(new QLabel(tr("Mode"), cmdGroup), 2, 0);
    cmdGrid->addWidget(m_cia402ModeCombo, 2, 1);
    cmdGrid->addWidget(m_cia402SendModeButton, 2, 2);
    cmdGrid->addWidget(m_cia402QueryButton, 2, 3);
    cmdGrid->addWidget(new QLabel(tr("Speed rpm"), cmdGroup), 3, 0);
    cmdGrid->addWidget(m_cia402SpeedRefSpin, 3, 1);
    cmdGrid->addWidget(m_cia402SendSpeedButton, 3, 2);
    cmdGrid->addWidget(new QLabel(tr("Torque A"), cmdGroup), 4, 0);
    cmdGrid->addWidget(m_cia402TorqueRefSpin, 4, 1);
    cmdGrid->addWidget(m_cia402SendTorqueButton, 4, 2);
    cmdGrid->addWidget(m_paramQueryButton, 4, 3);
    cmdGrid->setColumnMinimumWidth(0, 105);
    cmdGrid->setColumnMinimumWidth(1, 190);
    cmdGrid->setColumnMinimumWidth(2, 150);
    cmdGrid->setColumnMinimumWidth(3, 150);

    m_cia402ControlWordSpin->setToolTip(tr("Manual CiA402 controlword entry for custom state transitions."));
    m_cia402ModeCombo->setToolTip(tr("Select the CiA402 operation mode to send to the device."));
    m_cia402SpeedRefSpin->setToolTip(tr("Target speed command in rpm."));
    m_cia402TorqueRefSpin->setToolTip(tr("Target torque/current related command used by firmware mapping."));
    m_cia402SendCwButton->setToolTip(tr("Send the current controlword value."));
    m_cia402SendModeButton->setToolTip(tr("Send the selected CiA402 mode."));
    m_cia402SendSpeedButton->setToolTip(tr("Send the current speed reference."));
    m_cia402SendTorqueButton->setToolTip(tr("Send the current torque reference."));
    m_cia402QueryButton->setToolTip(tr("Read back the latest CiA402 status summary from firmware."));
    m_paramQueryButton->setToolTip(tr("Read back Rs/Ld/Lq/Ke and parameter-valid flags from firmware."));

    statusGroup->setMinimumWidth(360);
    statusGroup->setMaximumWidth(460);
    statusGroup->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);
    cmdGroup->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Maximum);
    root->addWidget(statusGroup, 0);
    root->addWidget(cmdGroup, 1);

    connect(m_cia402SendCwButton, &QPushButton::clicked, this, [this]() {
        sendPresetCommand(QStringLiteral("cia402 cw %1").arg(m_cia402ControlWordSpin->value()));
    });
    connect(faultButton, &QPushButton::clicked, this, [this]() {
        sendPresetCommand(QStringLiteral("cia402 cw 128"));
    });
    connect(shutdownButton, &QPushButton::clicked, this, [this]() {
        sendPresetCommand(QStringLiteral("cia402 cw 6"));
    });
    connect(switchOnButton, &QPushButton::clicked, this, [this]() {
        sendPresetCommand(QStringLiteral("cia402 cw 7"));
    });
    connect(enableOpButton, &QPushButton::clicked, this, [this]() {
        sendPresetCommand(QStringLiteral("cia402 cw 15"));
    });
    connect(m_cia402SendModeButton, &QPushButton::clicked, this, [this]() {
        sendPresetCommand(QStringLiteral("cia402 mode %1").arg(m_cia402ModeCombo->currentData().toInt()));
    });
    connect(m_cia402SendSpeedButton, &QPushButton::clicked, this, [this]() {
        sendPresetCommand(QStringLiteral("cia402 speed %1").arg(m_cia402SpeedRefSpin->value(), 0, 'f', 2));
    });
    connect(m_cia402SendTorqueButton, &QPushButton::clicked, this, [this]() {
        sendPresetCommand(QStringLiteral("cia402 torque %1").arg(m_cia402TorqueRefSpin->value(), 0, 'f', 3));
    });
    connect(m_cia402QueryButton, &QPushButton::clicked, this, [this]() {
        sendPresetCommand(QStringLiteral("cia402 status"));
    });
    connect(m_paramQueryButton, &QPushButton::clicked, this, [this]() {
        sendPresetCommand(QStringLiteral("params"));
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
    m_plotTimeBaseMs = 0;
    m_lastChartRefreshTick = 0;
    m_lastCia402Statusword = 0U;
    m_lastCia402Mode = 0U;
    m_lastAxisState = 0U;
    m_lastAxisError = 0U;
    m_lastControlMode = 0U;
    m_lastParamState = 0U;
    m_lastTelemetrySequence = 0U;
    m_lastTelemetryDeviceTick = 0U;
    if (m_debugEdit != nullptr)
    {
        m_debugEdit->clear();
    }
    m_rxLineBuffer.clear();
    m_angleMechPoints.clear();
    m_angleAppPoints.clear();
    if (m_angleMechSeries != nullptr)
    {
        m_angleMechSeries->clear();
    }
    if (m_angleAppSeries != nullptr)
    {
        m_angleAppSeries->clear();
    }
    if (m_angleTimeSlider != nullptr)
    {
        m_angleTimeSlider->setRange(0, 0);
        m_angleTimeSlider->setValue(0);
    }
    if (m_speedSeries != nullptr)
    {
        m_speedSeries->clear();
    }
    if (m_speedRefSeries != nullptr)
    {
        m_speedRefSeries->clear();
    }
    m_speedPoints.clear();
    m_speedRefPoints.clear();
    m_speedSampleIndex = 0;
    if (m_speedTimeSlider != nullptr)
    {
        m_speedTimeSlider->setRange(0, 0);
        m_speedTimeSlider->setValue(0);
    }

    if (m_iqRefSeries != nullptr)
    {
        m_iqRefSeries->clear();
    }
    if (m_iqRawSeries != nullptr)
    {
        m_iqRawSeries->clear();
    }
    m_iqRawPoints.clear();
    m_iqRefPoints.clear();
    if (m_iqIdSeries != nullptr)
    {
        m_iqIdSeries->clear();
    }
    m_iqIdPoints.clear();
    m_iqRawSampleIndex = 0;
    m_iqRefSampleIndex = 0;
    m_iqIdSampleIndex = 0;
    if (m_iqRefTimeSlider != nullptr)
    {
        m_iqRefTimeSlider->setRange(0, 0);
        m_iqRefTimeSlider->setValue(0);
    }
    setLabelTextIfChanged(m_iqRefPeakLabel, tr("Iq_ref: [0.000000, 0.000000] A"));
    setLabelTextIfChanged(m_iqIdPeakLabel, tr("Id: [0.000000, 0.000000] A"));
    m_iqRefAutoFollow = true;
    m_iqRefSliderDragging = false;
    m_lastIqRefUiTick = 0;

    if (m_speedMeasSeries != nullptr)
    {
        m_speedMeasSeries->clear();
    }
    if (m_speedErrSeries != nullptr)
    {
        m_speedErrSeries->clear();
    }
    m_speedMeasPoints.clear();
    m_speedErrPoints.clear();
    m_speedMeasSampleIndex = 0;
    if (m_speedMeasTimeSlider != nullptr)
    {
        m_speedMeasTimeSlider->setRange(0, 0);
        m_speedMeasTimeSlider->setValue(0);
    }
    if (m_speedMeasPeakLabel != nullptr)
    {
        m_speedMeasPeakLabel->setText(tr("SpeedMeas: [0.000000, 0.000000] rpm | SpeedErr: [0.000000, 0.000000] rpm"));
    }

    setLabelTextIfChanged(m_cia402StatusWordLabel, formatDecodedUInt(0U, decodeCia402State(0U), true));
    setLabelTextIfChanged(m_cia402ModeLabel, formatDecodedUInt(0U, decodeCia402Mode(0U)));
    setLabelTextIfChanged(m_cia402AxisStateLabel, formatDecodedUInt(0U, decodeAxisState(0U)));
    setLabelTextIfChanged(m_cia402ErrorLabel, formatDecodedUInt(0U, decodeErrorState(0U)));
    setLabelTextIfChanged(m_cia402ControlModeLabel, formatDecodedUInt(0U, decodeControlMode(0U)));
    setLabelTextIfChanged(m_cia402ParamStateLabel, formatDecodedUInt(0U, decodeParamState(0U)));
    setLabelTextIfChanged(m_cia402SpeedRefLabel, tr("0.000000"));
    setLabelTextIfChanged(m_cia402TorqueRefLabel, tr("0.000000"));
    setLabelTextIfChanged(m_paramValidLabel, tr("0x00"));
    setLabelTextIfChanged(m_paramRsLabel, tr("0.000000"));
    setLabelTextIfChanged(m_paramLdLabel, tr("0.000000"));
    setLabelTextIfChanged(m_paramLqLabel, tr("0.000000"));
    setLabelTextIfChanged(m_paramKeLabel, tr("0.000000"));
    if (m_statusLink != nullptr)
    {
        m_statusLink->setText(tr("Waiting telemetry..."));
    }

    m_speedMeasAutoFollow = true;
    m_speedMeasSliderDragging = false;
    m_lastSpeedMeasUiTick = 0;

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
    while ((newlinePos = m_rxLineBuffer.indexOf('\n')) >= 0)
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

    if (line.startsWith(QStringLiteral("CIA402 ")))
    {
        handleTelemetryLine(line);
        appendDebug(line);
        return;
    }

    if (line.startsWith(QStringLiteral("PARAM ")))
    {
        handleParamLine(line);
        appendDebug(line);
        return;
    }
}

namespace
{
bool parseKeyValueFloatFast(const QString &line, const char *key, double *value)
{
    if (value == nullptr)
    {
        return false;
    }

    const QString keyStr = QString::fromLatin1(key);
    const QString needle = keyStr + QLatin1Char('=');
    const int keyPos = line.indexOf(needle, 0, Qt::CaseInsensitive);
    if (keyPos < 0)
    {
        return false;
    }

    int endPos = keyPos + needle.size();
    while (endPos < line.size())
    {
        const QChar ch = line.at(endPos);
        if (!((ch.isDigit()) || (ch == '+') || (ch == '-') || (ch == '.') || (ch == 'e') || (ch == 'E')))
        {
            break;
        }
        ++endPos;
    }

    bool ok = false;
    const double v = line.mid(keyPos + needle.size(), endPos - (keyPos + needle.size())).toDouble(&ok);
    if (!ok)
    {
        return false;
    }

    *value = v;
    return true;
}

bool parseKeyValueUIntFast(const QString &line, const char *key, unsigned int *value)
{
    if (value == nullptr)
    {
        return false;
    }

    const QString keyStr = QString::fromLatin1(key);
    const QString needle = keyStr + QLatin1Char('=');
    const int keyPos = line.indexOf(needle, 0, Qt::CaseInsensitive);
    if (keyPos < 0)
    {
        return false;
    }

    int endPos = keyPos + needle.size();
    while (endPos < line.size())
    {
        const QChar ch = line.at(endPos);
        if (!((ch.isDigit()) || (ch == 'x') || (ch == 'X') ||
              ((ch >= 'a') && (ch <= 'f')) || ((ch >= 'A') && (ch <= 'F'))))
        {
            break;
        }
        ++endPos;
    }

    bool ok = false;
    const unsigned int v = line.mid(keyPos + needle.size(), endPos - (keyPos + needle.size())).toUInt(&ok, 0);
    if (!ok)
    {
        return false;
    }

    *value = v;
    return true;
}
}

void MainWindow::handleTelemetryLine(const QString &line)
{
    if (!m_angleSliderDragging)
    {
        m_angleAutoFollow = true;
    }
    if (!m_speedSliderDragging)
    {
        m_speedAutoFollow = true;
    }
    if (!m_iqRefSliderDragging)
    {
        m_iqRefAutoFollow = true;
    }
    if (!m_speedMeasSliderDragging)
    {
        m_speedMeasAutoFollow = true;
    }

    double mechDeg = 0.0;
    double appDeg = 0.0;
    double iqRawAmp = 0.0;
    double iqRefAmp = 0.0;
    double idAmp = 0.0;
    double speedRpm = 0.0;
    double speedMeasRpm = 0.0;
    double speedErrRpm = 0.0;
    double speedRefRpm = 0.0;
    double torqueRefA = 0.0;
    double torqueActA = 0.0;
    unsigned int cia402Sw = 0U;
    unsigned int cia402Mo = 0U;
    unsigned int cia402AxisState = 0U;
    unsigned int cia402Error = 0U;
    unsigned int cia402CtrlMode = 0U;
    unsigned int cia402ParamState = 0U;
    unsigned int telemetrySequence = 0U;
    unsigned int telemetryDeviceTick = 0U;
    const bool hasTelemetrySequence = parseKeyValueUIntFast(line, "SEQ", &telemetrySequence);
    const bool hasTelemetryDeviceTick = parseKeyValueUIntFast(line, "TMS", &telemetryDeviceTick);
    const bool hasMech = parseKeyValueFloatFast(line, "MECH", &mechDeg);
    const bool hasApp = parseKeyValueFloatFast(line, "APP", &appDeg);
    const bool hasIqRaw = parseKeyValueFloatFast(line, "IQRAW", &iqRawAmp);
    const bool hasIqRef = parseKeyValueFloatFast(line, "IQR", &iqRefAmp);
    const bool hasId = parseKeyValueFloatFast(line, "ID", &idAmp);
    const bool hasSpeedMeas = parseKeyValueFloatFast(line, "SPMR", &speedMeasRpm);
    const bool hasSpeedErr = parseKeyValueFloatFast(line, "SPER", &speedErrRpm);
    const bool hasSpeedRef = parseKeyValueFloatFast(line, "SPREF", &speedRefRpm);
    const bool hasTorqueRef = parseKeyValueFloatFast(line, "TREF", &torqueRefA);
    const bool hasTorqueAct = parseKeyValueFloatFast(line, "TACT", &torqueActA);
    const bool hasCia402Sw = parseKeyValueUIntFast(line, "SW", &cia402Sw);
    const bool hasCia402Mo = parseKeyValueUIntFast(line, "MO", &cia402Mo);
    const bool hasCia402AxisState = parseKeyValueUIntFast(line, "AST", &cia402AxisState);
    const bool hasCia402Error = parseKeyValueUIntFast(line, "ERR", &cia402Error);
    const bool hasCia402CtrlMode = parseKeyValueUIntFast(line, "CTRL", &cia402CtrlMode);
    const bool hasCia402ParamState = parseKeyValueUIntFast(line, "PST", &cia402ParamState);
    const bool hasSpeed = parseKeyValueFloatFast(line, "SPEED", &speedRpm) ||
                          parseKeyValueFloatFast(line, "SPD", &speedRpm);
    const bool hasAnyTelemetry = hasMech || hasApp || hasIqRaw || hasIqRef || hasId || hasSpeed ||
                                 hasSpeedMeas || hasSpeedErr || hasSpeedRef || hasTorqueRef ||
                                  hasTorqueAct || hasCia402Sw || hasCia402Mo || hasCia402AxisState ||
                                 hasCia402Error || hasCia402CtrlMode || hasCia402ParamState ||
                                 hasTelemetrySequence || hasTelemetryDeviceTick;

    if (hasMech || hasApp)
    {
        updateAngle(mechDeg, appDeg);
    }

    if (hasSpeed)
    {
        m_lastSpeedRpm = speedRpm;
    }
    if (hasSpeedRef)
    {
        m_lastSpeedRefRpm = speedRefRpm;
    }
    if (hasIqRaw)
    {
        m_lastIqRawAmp = iqRawAmp;
    }
    if (hasIqRef)
    {
        m_lastIqRefAmp = iqRefAmp;
    }
    if (hasId)
    {
        m_lastIdAmp = idAmp;
    }
    if (hasSpeedMeas)
    {
        m_lastSpeedMeasPu = speedMeasRpm;
    }
    if (hasSpeedErr)
    {
        m_lastSpeedErrorPu = speedErrRpm;
    }
    if (hasCia402Sw)
    {
        m_lastCia402Statusword = cia402Sw;
    }
    if (hasTelemetrySequence)
    {
        m_lastTelemetrySequence = telemetrySequence;
    }
    if (hasTelemetryDeviceTick)
    {
        m_lastTelemetryDeviceTick = telemetryDeviceTick;
    }
    if (hasCia402Mo)
    {
        m_lastCia402Mode = cia402Mo;
    }
    if (hasCia402AxisState)
    {
        m_lastAxisState = cia402AxisState;
        setLabelTextIfChanged(m_cia402AxisStateLabel, formatDecodedUInt(cia402AxisState, decodeAxisState(cia402AxisState)));
    }
    if (hasCia402Error)
    {
        m_lastAxisError = cia402Error;
        setLabelTextIfChanged(m_cia402ErrorLabel, formatDecodedUInt(cia402Error, decodeErrorState(cia402Error)));
    }
    if (hasCia402CtrlMode)
    {
        m_lastControlMode = cia402CtrlMode;
        setLabelTextIfChanged(m_cia402ControlModeLabel, formatDecodedUInt(cia402CtrlMode, decodeControlMode(cia402CtrlMode)));
    }
    if (hasCia402ParamState)
    {
        m_lastParamState = cia402ParamState;
        setLabelTextIfChanged(m_cia402ParamStateLabel, formatDecodedUInt(cia402ParamState, decodeParamState(cia402ParamState)));
    }
    if (hasCia402Sw)
    {
        setLabelTextIfChanged(m_cia402StatusWordLabel,
                              formatDecodedUInt(m_lastCia402Statusword,
                                                decodeCia402State(m_lastCia402Statusword),
                                                true));
    }
    if (hasCia402Mo)
    {
        setLabelTextIfChanged(m_cia402ModeLabel, formatDecodedUInt(m_lastCia402Mode, decodeCia402Mode(m_lastCia402Mode)));
    }
    if (hasSpeedRef)
    {
        setLabelTextIfChanged(m_cia402SpeedRefLabel, fmt6(speedRefRpm));
    }
    if (hasTorqueRef)
    {
        setLabelTextIfChanged(m_cia402TorqueRefLabel, fmt6(torqueRefA));
    }
    if ((m_statusLink != nullptr) && (hasCia402Sw || hasCia402Mo || hasCia402AxisState || hasCia402Error))
    {
        setLabelTextIfChanged(m_statusLink,
                              tr("%1 | %2 | %3 | %4")
                                  .arg(decodeCia402State(m_lastCia402Statusword),
                                       decodeCia402Mode(m_lastCia402Mode),
                                       decodeAxisState(m_lastAxisState),
                                       decodeErrorState(m_lastAxisError)));
    }

    if (hasSpeed || hasIqRaw || hasIqRef || hasId || hasSpeedRef || hasTorqueRef || hasTorqueAct)
    {
        updateSpeed(m_lastSpeedRpm);
    }
    if ((m_speedLink != nullptr) && hasSpeedRef)
    {
        setLabelTextIfChanged(m_speedLink,
                              tr("Telemetry OK | Ref=%1 rpm | SEQ=%2")
                                  .arg(fmt6(m_lastSpeedRefRpm),
                                       QString::number(m_lastTelemetrySequence)));
    }

    if (hasIqRaw)
    {
        setLabelTextIfChanged(m_iqRawStatus, fmt6(iqRawAmp));
        setLabelTextIfChanged(m_iqRawLink, tr("Telemetry OK"));
        const double x = currentPlotTimeSeconds(&m_plotTimeBaseMs);
        m_iqRawPoints.append(QPointF(x, iqRawAmp));
        ++m_iqRawSampleIndex;
        trimPointBuffer(m_iqRawPoints, 7200);
        m_lastIqRawUiTick = QDateTime::currentMSecsSinceEpoch();
    }

    if (hasIqRef)
    {
        setLabelTextIfChanged(m_iqRefStatus, fmt6(iqRefAmp));
        setLabelTextIfChanged(m_iqRefDetailLabel, tr("Iq_ref: %1 A").arg(fmt6(iqRefAmp)));
        setLabelTextIfChanged(m_iqRefLink, tr("Telemetry OK"));
        const double x = currentPlotTimeSeconds(&m_plotTimeBaseMs);
        m_iqRefPoints.append(QPointF(x, iqRefAmp));
        ++m_iqRefSampleIndex;
        trimPointBuffer(m_iqRefPoints, 7200);
        if (m_iqRefTimeSlider != nullptr)
        {
            const int maxOffset = qMax(0, m_iqRefPoints.size() - 1);
            m_iqRefTimeSlider->setRange(0, maxOffset);
            if (m_iqRefAutoFollow)
            {
                m_iqRefTimeSlider->blockSignals(true);
                m_iqRefTimeSlider->setValue(findLiveWindowStart(m_iqRawPoints, kLiveChartWindowSeconds));
                m_iqRefTimeSlider->blockSignals(false);
            }
        }
        m_lastIqRefUiTick = QDateTime::currentMSecsSinceEpoch();
    }

    if (hasId)
    {
        setLabelTextIfChanged(m_iqIdStatus, fmt6(idAmp));
        setLabelTextIfChanged(m_iqIdDetailLabel, tr("Id: %1 A").arg(fmt6(idAmp)));
        setLabelTextIfChanged(m_iqIdLink, tr("Telemetry OK"));
        const double x = currentPlotTimeSeconds(&m_plotTimeBaseMs);
        m_iqIdPoints.append(QPointF(x, idAmp));
        ++m_iqIdSampleIndex;
        trimPointBuffer(m_iqIdPoints, 7200);
    }

    if (hasSpeedMeas || hasSpeedErr)
    {
        const double x = currentPlotTimeSeconds(&m_plotTimeBaseMs);
        m_speedMeasPoints.append(QPointF(x, speedMeasRpm));
        m_speedErrPoints.append(QPointF(x, speedErrRpm));
        ++m_speedMeasSampleIndex;
        trimPointBuffer(m_speedMeasPoints, 7200);
        trimPointBuffer(m_speedErrPoints, 7200);
        if (m_speedMeasTimeSlider != nullptr)
        {
            const int maxOffset = qMax(0, m_speedMeasPoints.size() - 1);
            m_speedMeasTimeSlider->setRange(0, maxOffset);
            if (m_speedMeasAutoFollow)
            {
                m_speedMeasTimeSlider->blockSignals(true);
                m_speedMeasTimeSlider->setValue(findLiveWindowStart(m_speedMeasPoints, kLiveChartWindowSeconds));
                m_speedMeasTimeSlider->blockSignals(false);
            }
        }
        if (m_speedMeasStatus != nullptr)
        {
            setLabelTextIfChanged(m_speedMeasStatus, fmt6(speedMeasRpm));
        }
        if (m_speedMeasDetailLabel != nullptr)
        {
            setLabelTextIfChanged(m_speedMeasDetailLabel,
                                  tr("SpeedMeas: %1 rpm | SpeedErr: %2 rpm").arg(fmt6(speedMeasRpm), fmt6(speedErrRpm)));
        }
        setLabelTextIfChanged(m_speedMeasLink, tr("Telemetry OK"));
        m_lastSpeedMeasUiTick = QDateTime::currentMSecsSinceEpoch();
        refreshSpeedMeasChart();
    }

    if (hasAnyTelemetry)
    {
        m_lastTelemetryTick = QDateTime::currentMSecsSinceEpoch();
        if (m_statusLink != nullptr)
        {
            if (hasCia402Sw || hasCia402Mo)
            {
                setLabelTextIfChanged(m_statusLink,
                                      tr("%1 | %2 | SEQ=%3")
                                          .arg(decodeCia402State(m_lastCia402Statusword),
                                               decodeCia402Mode(m_lastCia402Mode),
                                               QString::number(m_lastTelemetrySequence)));
            }
            else
            {
                setLabelTextIfChanged(m_statusLink,
                                      tr("Telemetry OK | SEQ=%1")
                                          .arg(QString::number(m_lastTelemetrySequence)));
            }
        }
    }

    if (hasIqRaw || hasIqRef || hasId)
    {
        refreshIqRefChart();
    }
}

void MainWindow::handleParamLine(const QString &line)
{
    double rs = 0.0;
    double ld = 0.0;
    double lq = 0.0;
    double ke = 0.0;
    unsigned int valid = 0U;
    unsigned int pst = 0U;

    const bool hasValid = parseKeyValueUIntFast(line, "VALID", &valid);
    const bool hasRs = parseKeyValueFloatFast(line, "RS", &rs);
    const bool hasLd = parseKeyValueFloatFast(line, "LD", &ld);
    const bool hasLq = parseKeyValueFloatFast(line, "LQ", &lq);
    const bool hasKe = parseKeyValueFloatFast(line, "KE", &ke);
    const bool hasPst = parseKeyValueUIntFast(line, "PST", &pst);

    if (hasValid && m_paramValidLabel != nullptr)
    {
        setLabelTextIfChanged(m_paramValidLabel,
                              QStringLiteral("0x%1").arg(valid, 2, 16, QLatin1Char('0')).toUpper());
    }
    if (hasRs && m_paramRsLabel != nullptr)
    {
        setLabelTextIfChanged(m_paramRsLabel, fmt6(rs));
    }
    if (hasLd && m_paramLdLabel != nullptr)
    {
        setLabelTextIfChanged(m_paramLdLabel, fmt6(ld));
    }
    if (hasLq && m_paramLqLabel != nullptr)
    {
        setLabelTextIfChanged(m_paramLqLabel, fmt6(lq));
    }
    if (hasKe && m_paramKeLabel != nullptr)
    {
        setLabelTextIfChanged(m_paramKeLabel, fmt6(ke));
    }
    if (hasPst && m_cia402ParamStateLabel != nullptr)
    {
        setLabelTextIfChanged(m_cia402ParamStateLabel, formatDecodedUInt(pst, decodeParamState(pst)));
    }
}

void MainWindow::updateAngle(double mechDeg, double appDeg)
{
    if (m_angleMechSeries == nullptr || m_angleAppSeries == nullptr)
    {
        return;
    }

    const double x = currentPlotTimeSeconds(&m_plotTimeBaseMs);
    m_angleMechPoints.append(QPointF(x, mechDeg));
    m_angleAppPoints.append(QPointF(x, appDeg));
    ++m_sampleIndex;

    trimPointBuffer(m_angleMechPoints, 7200);
    trimPointBuffer(m_angleAppPoints, 7200);

    if (m_angleTimeSlider != nullptr)
    {
        const int maxOffset = qMax(0, m_angleMechPoints.size() - 1);
        m_angleTimeSlider->setRange(0, maxOffset);
        if (m_angleAutoFollow)
        {
            m_angleTimeSlider->blockSignals(true);
            m_angleTimeSlider->setValue(findLiveWindowStart(m_angleMechPoints, kLiveChartWindowSeconds));
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
        setLabelTextIfChanged(m_statusAngle, fmt6(mechDeg));
    }
    if (m_angleDetailLabel != nullptr)
    {
        setLabelTextIfChanged(m_angleDetailLabel,
                              tr("MECH: %1 | APP: %2").arg(fmt6(mechDeg), fmt6(appDeg)));
    }
    setLabelTextIfChanged(m_statusAngleLink, tr("Telemetry OK"));
}

void MainWindow::updateSpeed(double speedRpm)
{
    if (m_speedSeries == nullptr)
    {
        return;
    }

    const double x = currentPlotTimeSeconds(&m_plotTimeBaseMs);
    m_speedPoints.append(QPointF(x, speedRpm));
    m_speedRefPoints.append(QPointF(x, m_lastSpeedRefRpm));
    ++m_speedSampleIndex;

    trimPointBuffer(m_speedPoints, 7200);
    trimPointBuffer(m_speedRefPoints, 7200);

    if (m_speedTimeSlider != nullptr)
    {
        const int maxOffset = qMax(0, m_speedPoints.size() - 1);
        m_speedTimeSlider->setRange(0, maxOffset);
        if (m_speedAutoFollow)
        {
            m_speedTimeSlider->blockSignals(true);
            m_speedTimeSlider->setValue(findLiveWindowStart(m_speedPoints, kLiveChartWindowSeconds));
            m_speedTimeSlider->blockSignals(false);
        }
    }

    const qint64 now = QDateTime::currentMSecsSinceEpoch();
    if ((now - m_lastSpeedUiTick) >= 50 || m_speedSliderDragging)
    {
        refreshSpeedChart();
        m_lastSpeedUiTick = now;
    }

    if (m_statusSpeed != nullptr)
    {
        setLabelTextIfChanged(m_statusSpeed, fmt6(speedRpm));
    }
    setLabelTextIfChanged(m_speedLink, tr("Telemetry OK"));
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

    const int sliderStart = qBound(0, m_angleTimeSlider != nullptr ? m_angleTimeSlider->value() : 0, qMax(0, count - 1));
    const int startIndex = m_angleAutoFollow ? findLiveWindowStart(m_angleMechPoints, kLiveChartWindowSeconds) : sliderStart;
    const int endIndex = m_angleAutoFollow ? count : qMin(count, startIndex + m_angleWindowSize);
    const int size = qMax(0, endIndex - startIndex);

    const QVector<QPointF> mechVisible = m_angleMechPoints.mid(startIndex, size);
    const QVector<QPointF> appVisible = m_angleAppPoints.mid(startIndex, size);
    m_angleMechSeries->replace(mechVisible);
    m_angleAppSeries->replace(appVisible);

    if (!mechVisible.isEmpty())
    {
        const double xMin = mechVisible.first().x();
        const double xMax = mechVisible.last().x();
        m_angleAxisX->setRange(xMin, qMax(xMin + 0.001, xMax));
    }
    m_angleAxisX->setLabelFormat("%.1f");

    m_angleAxisY->setRange(0.0, 360.0);
    m_angleAxisY->setTickCount(7);
    m_angleAxisY->setLabelFormat("%.1f");
    m_angleAxisY->setReverse(false);

}

void MainWindow::refreshSpeedChart()
{
    if (m_speedSeries == nullptr || m_speedAxisX == nullptr || m_speedAxisY == nullptr)
    {
        return;
    }

    const int count = m_speedPoints.size();
    if (count <= 0)
    {
        m_speedSeries->clear();
        return;
    }

    const int sliderStart = qBound(0, m_speedTimeSlider != nullptr ? m_speedTimeSlider->value() : 0, qMax(0, count - 1));
    const int startIndex = m_speedAutoFollow ? findLiveWindowStart(m_speedPoints, kLiveChartWindowSeconds) : sliderStart;
    const int endIndex = m_speedAutoFollow ? count : qMin(count, startIndex + m_speedWindowSize);
    const int size = qMax(0, endIndex - startIndex);

    const QVector<QPointF> visible = m_speedPoints.mid(startIndex, size);
    const QVector<QPointF> refVisible = m_speedRefPoints.mid(startIndex, size);
    m_speedSeries->replace(visible);
    if (m_speedRefSeries != nullptr)
    {
        m_speedRefSeries->replace(refVisible);
    }

    if (!visible.isEmpty() || !refVisible.isEmpty())
    {
        const QVector<QPointF> &xPoints = !visible.isEmpty() ? visible : refVisible;
        const double xMin = xPoints.first().x();
        const double xMax = xPoints.last().x();
        m_speedAxisX->setRange(xMin, qMax(xMin + 0.001, xMax));
    }
    m_speedAxisX->setLabelFormat("%.1f");

    double minSpeed = m_speedPoints[startIndex].y();
    double maxSpeed = minSpeed;
    for (int i = startIndex; i < endIndex; ++i)
    {
        minSpeed = qMin(minSpeed, m_speedPoints[i].y());
        maxSpeed = qMax(maxSpeed, m_speedPoints[i].y());
        if (i < m_speedRefPoints.size())
        {
            minSpeed = qMin(minSpeed, m_speedRefPoints[i].y());
            maxSpeed = qMax(maxSpeed, m_speedRefPoints[i].y());
        }
    }

    const double span = qMax(50.0, maxSpeed - minSpeed);
    const double margin = qMax(50.0, span * 0.25);
    const double speedMin = qBound(-10000.0, minSpeed - margin, 10000.0);
    const double speedMax = qBound(-10000.0, maxSpeed + margin, 10000.0);
    m_speedAxisY->setRange(speedMin, speedMax);
    m_speedAxisY->setTickCount(11);
    m_speedAxisY->setLabelFormat("%.0f");
}

void MainWindow::refreshSpeedMeasChart()
{
    if (m_speedMeasSeries == nullptr || m_speedErrSeries == nullptr || m_speedMeasAxisX == nullptr || m_speedMeasAxisY == nullptr || m_speedErrAxis == nullptr)
    {
        return;
    }

    const int count = m_speedMeasPoints.size();
    if (count <= 0)
    {
        m_speedMeasSeries->clear();
        m_speedErrSeries->clear();
        return;
    }

    const int sliderStart = qBound(0, m_speedMeasTimeSlider != nullptr ? m_speedMeasTimeSlider->value() : 0, qMax(0, count - 1));
    const int startIndex = m_speedMeasAutoFollow ? findLiveWindowStart(m_speedMeasPoints, kLiveChartWindowSeconds) : sliderStart;
    const int endIndex = m_speedMeasAutoFollow ? count : qMin(count, startIndex + m_speedMeasWindowSize);
    const int size = qMax(0, endIndex - startIndex);

    const QVector<QPointF> measVisible = m_speedMeasPoints.mid(startIndex, size);
    const QVector<QPointF> errVisible = m_speedErrPoints.mid(startIndex, size);
    m_speedMeasSeries->replace(measVisible);
    m_speedErrSeries->replace(errVisible);
    if (!measVisible.isEmpty())
    {
        const double xMin = measVisible.first().x();
        const double xMax = measVisible.last().x();
        m_speedMeasAxisX->setRange(xMin, qMax(xMin + 0.001, xMax));
    }
    m_speedMeasAxisX->setLabelFormat("%.1f");

    if (size > 0 && m_speedMeasPeakLabel != nullptr)
    {
        double minMeas = m_speedMeasPoints[startIndex].y();
        double maxMeas = minMeas;
        double minErr = m_speedErrPoints[startIndex].y();
        double maxErr = minErr;
        for (int i = startIndex; i < endIndex; ++i)
        {
            minMeas = qMin(minMeas, m_speedMeasPoints[i].y());
            maxMeas = qMax(maxMeas, m_speedMeasPoints[i].y());
            minErr = qMin(minErr, m_speedErrPoints[i].y());
            maxErr = qMax(maxErr, m_speedErrPoints[i].y());
        }

        const double measSpan = qMax(1.0, maxMeas - minMeas);
        const double measMargin = qMax(1.0, measSpan * 0.25);
        const double measMin = minMeas - measMargin;
        const double measMax = maxMeas + measMargin;
        m_speedMeasAxisY->setRange(measMin, measMax);
        m_speedMeasAxisY->setTickCount(9);
        m_speedMeasAxisY->setLabelFormat("%.0f");

        const double errSpan = qMax(0.05, maxErr - minErr);
        const double errMargin = qMax(0.05, errSpan * 0.25);
        const double errMin = minErr - errMargin;
        const double errMax = maxErr + errMargin;
        m_speedErrAxis->setRange(errMin, errMax);
        m_speedErrAxis->setTickCount(9);
        m_speedErrAxis->setLabelFormat("%.3f");

        m_speedMeasPeakLabel->setText(tr("SpeedMeas: [%1, %2] rpm | SpeedErr: [%3, %4] rpm")
                                          .arg(fmt6(minMeas), fmt6(maxMeas), fmt6(minErr), fmt6(maxErr)));
    }
    else
    {
        m_speedMeasAxisY->setRange(-1.0, 1.0);
        m_speedMeasAxisY->setTickCount(9);
        m_speedMeasAxisY->setLabelFormat("%.0f");
        m_speedErrAxis->setRange(-1.0, 1.0);
        m_speedErrAxis->setTickCount(9);
        m_speedErrAxis->setLabelFormat("%.3f");
    }
}

void MainWindow::refreshIqRefChart()
{
    if (m_iqRefSeries == nullptr || m_iqRawSeries == nullptr || m_iqIdSeries == nullptr || m_iqRefAxisX == nullptr || m_iqRefAxisY == nullptr)
    {
        return;
    }

    if ((m_iqRefPoints.isEmpty() && m_iqRawPoints.isEmpty() && m_iqIdPoints.isEmpty()) || m_lastTelemetryTick == 0)
    {
        return;
    }

    const int refCount = m_iqRefPoints.size();
    const int rawCount = m_iqRawPoints.size();
    const int idCount = m_iqIdPoints.size();
    const int count = qMax(qMax(refCount, rawCount), idCount);
    const int sliderStart = qBound(0, m_iqRefTimeSlider != nullptr ? m_iqRefTimeSlider->value() : 0, qMax(0, count - 1));
    const int startIndex = m_iqRefAutoFollow ? findLiveWindowStart(m_iqRawPoints, kLiveChartWindowSeconds) : sliderStart;
    const int endIndex = m_iqRefAutoFollow ? count : qMin(count, startIndex + m_iqRefWindowSize);
    const int size = qMax(0, endIndex - startIndex);

    const QVector<QPointF> rawVisible = (startIndex < rawCount) ? m_iqRawPoints.mid(startIndex, qMin(size, rawCount - startIndex)) : QVector<QPointF>();
    const QVector<QPointF> idVisible = (startIndex < idCount) ? m_iqIdPoints.mid(startIndex, qMin(size, idCount - startIndex)) : QVector<QPointF>();
    const QVector<QPointF> refVisible = (startIndex < refCount) ? m_iqRefPoints.mid(startIndex, qMin(size, refCount - startIndex)) : QVector<QPointF>();

    m_iqRawSeries->replace(rawVisible);
    m_iqRefSeries->replace(refVisible);
    m_iqIdSeries->replace(idVisible);
    double xMin = 0.0;
    double xMax = 1.0;
    bool hasXRange = false;
    const auto updateXRange = [&hasXRange, &xMin, &xMax](const QVector<QPointF> &points) {
        if (points.isEmpty())
        {
            return;
        }

        if (!hasXRange)
        {
            xMin = points.first().x();
            xMax = points.last().x();
            hasXRange = true;
            return;
        }

        xMin = qMin(xMin, points.first().x());
        xMax = qMax(xMax, points.last().x());
    };
    updateXRange(rawVisible);
    updateXRange(refVisible);
    updateXRange(idVisible);
    if (hasXRange)
    {
        m_iqRefAxisX->setRange(xMin, qMax(xMin + 0.001, xMax));
    }
    m_iqRefAxisX->setLabelFormat("%.1f");
    m_iqRefAxisY->setRange(-2.5, 2.5);
    m_iqRefAxisY->setTickCount(11);
    m_iqRefAxisY->setLabelFormat("%.3f");
    if (size > 0)
    {
        double minIqRef = 0.0;
        double maxIqRef = 0.0;
        bool hasIqRef = false;
        for (const QPointF &p : refVisible)
        {
            if (!hasIqRef)
            {
                minIqRef = p.y();
                maxIqRef = p.y();
                hasIqRef = true;
            }
            else
            {
                minIqRef = qMin(minIqRef, p.y());
                maxIqRef = qMax(maxIqRef, p.y());
            }
        }

        double minId = 0.0;
        double maxId = 0.0;
        bool hasId = false;
        for (const QPointF &p : idVisible)
        {
            if (!hasId)
            {
                minId = p.y();
                maxId = p.y();
                hasId = true;
            }
            else
            {
                minId = qMin(minId, p.y());
                maxId = qMax(maxId, p.y());
            }
        }

        if (m_iqRefPeakLabel != nullptr)
        {
            if (hasIqRef)
            {
                m_iqRefPeakLabel->setText(tr("Iq_ref: [%1, %2] A").arg(fmt6(minIqRef), fmt6(maxIqRef)));
            }
            else
            {
                m_iqRefPeakLabel->setText(tr("Iq_ref: [0.000000, 0.000000] A"));
            }
        }
        if (m_iqRawStatus != nullptr)
        {
            if (!rawVisible.isEmpty())
            {
                m_iqRawStatus->setText(fmt6(m_lastIqRawAmp));
            }
            else
            {
                m_iqRawStatus->setText(tr("0.000000"));
            }
        }
        if (m_iqRawLink != nullptr)
        {
            m_iqRawLink->setText(!rawVisible.isEmpty() ? tr("Telemetry OK") : tr("Waiting telemetry..."));
        }
        if (m_iqIdStatus != nullptr)
        {
            if (hasId)
            {
                m_iqIdStatus->setText(fmt6(m_lastIdAmp));
            }
            else
            {
                m_iqIdStatus->setText(tr("0.000000"));
            }
        }
        if (m_iqIdLink != nullptr)
        {
            m_iqIdLink->setText(!idVisible.isEmpty() ? tr("Telemetry OK") : tr("Waiting telemetry..."));
        }
        if (m_iqIdPeakLabel != nullptr)
        {
            if (hasId)
            {
                m_iqIdPeakLabel->setText(tr("Id: [%1, %2] A").arg(fmt6(minId), fmt6(maxId)));
            }
            else
            {
                m_iqIdPeakLabel->setText(tr("Id: [0.000000, 0.000000] A"));
            }
        }
    }
    else
    {
        if (m_iqRawStatus != nullptr)
        {
            m_iqRawStatus->setText(tr("0.000000"));
        }
        if (m_iqRawLink != nullptr)
        {
            m_iqRawLink->setText(tr("Waiting telemetry..."));
        }
        if (m_iqRefPeakLabel != nullptr)
        {
            m_iqRefPeakLabel->setText(tr("Iq_ref: [0.000000, 0.000000] A"));
        }
        if (m_iqIdStatus != nullptr)
        {
            m_iqIdStatus->setText(tr("0.000000"));
        }
        if (m_iqIdLink != nullptr)
        {
            m_iqIdLink->setText(tr("Waiting telemetry..."));
        }
        if (m_iqIdPeakLabel != nullptr)
        {
            m_iqIdPeakLabel->setText(tr("Id: [0.000000, 0.000000] A"));
        }
    }
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

void MainWindow::zoomIqRefWindow(int delta)
{
    m_iqRefWindowSize = qBound(100, m_iqRefWindowSize + delta, 6000);
    m_iqRefAutoFollow = false;
    refreshIqRefChart();
}

void MainWindow::zoomSpeedMeasWindow(int delta)
{
    m_speedMeasWindowSize = qBound(100, m_speedMeasWindowSize + delta, 6000);
    m_speedMeasAutoFollow = false;
    refreshSpeedMeasChart();
}

void MainWindow::resetAngleWindow()
{
    m_angleWindowSize = 1800;
    m_angleAutoFollow = true;
    if (m_angleTimeSlider != nullptr)
    {
        m_angleTimeSlider->setValue(findLiveWindowStart(m_angleMechPoints, kLiveChartWindowSeconds));
    }
    refreshAngleChart();
}

void MainWindow::resetSpeedWindow()
{
    m_speedWindowSize = 600;
    m_speedAutoFollow = true;
    if (m_speedTimeSlider != nullptr)
    {
        m_speedTimeSlider->setValue(findLiveWindowStart(m_speedPoints, kLiveChartWindowSeconds));
    }
    refreshSpeedChart();
}

void MainWindow::resetIqRefWindow()
{
    m_iqRefWindowSize = 1800;
    m_iqRefAutoFollow = true;
    if (m_iqRefTimeSlider != nullptr)
    {
        m_iqRefTimeSlider->setValue(findLiveWindowStart(m_iqRawPoints, kLiveChartWindowSeconds));
    }
    if (m_iqRefPeakLabel != nullptr)
    {
        m_iqRefPeakLabel->setText(tr("Iq_ref: [0.000000, 0.000000] A"));
    }
    if (m_iqIdPeakLabel != nullptr)
    {
        m_iqIdPeakLabel->setText(tr("Id: [0.000000, 0.000000] A"));
    }
    refreshIqRefChart();
}

void MainWindow::resetSpeedMeasWindow()
{
    m_speedMeasWindowSize = 1800;
    m_speedMeasAutoFollow = true;
    if (m_speedMeasTimeSlider != nullptr)
    {
        m_speedMeasTimeSlider->setValue(findLiveWindowStart(m_speedMeasPoints, kLiveChartWindowSeconds));
    }
    if (m_speedMeasPeakLabel != nullptr)
    {
        m_speedMeasPeakLabel->setText(tr("SPMR: [0.000000, 0.000000] rpm | SPER: [0.000000, 0.000000] rpm"));
    }
    refreshSpeedMeasChart();
}

void MainWindow::appendLog(const QString &text)
{
    if (m_logEdit == nullptr)
    {
        return;
    }

    if (text == m_lastLogText)
    {
        return;
    }
    m_lastLogText = text;

    m_logEdit->document()->setMaximumBlockCount(500);
    m_logEdit->append(text);
    if (m_autoScrollCheck != nullptr && m_autoScrollCheck->isChecked())
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

    const QString line = QStringLiteral("[%1] %2")
                             .arg(QDateTime::currentDateTime().toString("HH:mm:ss.zzz"), text);
    if (line == m_lastDebugText)
    {
        return;
    }
    m_lastDebugText = line;

    m_debugEdit->document()->setMaximumBlockCount(500);
    m_debugEdit->appendPlainText(line);
    m_debugEdit->verticalScrollBar()->setValue(m_debugEdit->verticalScrollBar()->maximum());
}

void MainWindow::setLabelTextIfChanged(QLabel *label, const QString &text)
{
    if (label != nullptr && label->text() != text)
    {
        label->setText(text);
    }
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
