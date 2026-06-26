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
    m_tabs->addTab(m_controlPage, tr("Control"));
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
    chart->addSeries(m_speedSeries);
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
        m_iqRefAxisX->setTitleText(tr("Sample"));
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
        m_speedMeasAxisX->setTitleText(tr("Sample"));
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
}

void MainWindow::handleTelemetryLine(const QString &line)
{
    double mechDeg = 0.0;
    double appDeg = 0.0;
    double iqRawAmp = 0.0;
    double iqRefAmp = 0.0;
    double idAmp = 0.0;
    double speedRpm = 0.0;
    double speedMeasRpm = 0.0;
    double speedErrRpm = 0.0;
    const bool hasMech = parseKeyValueFloatFast(line, "MECH", &mechDeg);
    const bool hasApp = parseKeyValueFloatFast(line, "APP", &appDeg);
    const bool hasIqRaw = parseKeyValueFloatFast(line, "IQRAW", &iqRawAmp);
    const bool hasIqRef = parseKeyValueFloatFast(line, "IQR", &iqRefAmp);
    const bool hasId = parseKeyValueFloatFast(line, "ID", &idAmp);
    const bool hasSpeedMeas = parseKeyValueFloatFast(line, "SPMR", &speedMeasRpm);
    const bool hasSpeedErr = parseKeyValueFloatFast(line, "SPER", &speedErrRpm);
    const bool hasSpeed = parseKeyValueFloatFast(line, "SPEED", &speedRpm) ||
                          parseKeyValueFloatFast(line, "SPD", &speedRpm);
    const bool hasAnyTelemetry = hasMech || hasApp || hasIqRaw || hasIqRef || hasId || hasSpeed ||
                                  hasSpeedMeas || hasSpeedErr;

    if (hasMech || hasApp)
    {
        updateAngle(mechDeg, appDeg);
    }

    if (hasSpeed)
    {
        m_lastSpeedRpm = speedRpm;
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

    if (hasSpeed || hasIqRaw || hasIqRef || hasId)
    {
        updateSpeed(m_lastSpeedRpm);
    }

    if (hasIqRaw)
    {
        setLabelTextIfChanged(m_iqRawStatus, fmt6(iqRawAmp));
        setLabelTextIfChanged(m_iqRawLink, tr("Telemetry OK"));
        const qint64 x = m_iqRawSampleIndex;
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
        const qint64 x = m_iqRefSampleIndex;
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
                m_iqRefTimeSlider->setValue(qMax(0, m_iqRefPoints.size() - m_iqRefWindowSize));
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
        const qint64 x = m_iqIdSampleIndex;
        m_iqIdPoints.append(QPointF(x, idAmp));
        ++m_iqIdSampleIndex;
        trimPointBuffer(m_iqIdPoints, 7200);
    }

    if (hasSpeedMeas || hasSpeedErr)
    {
        const qint64 x = m_speedMeasSampleIndex;
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
                m_speedMeasTimeSlider->setValue(qMax(0, m_speedMeasPoints.size() - m_speedMeasWindowSize));
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
        if (m_tabs != nullptr && m_tabs->currentWidget() == m_speedMeasPage)
        {
            refreshSpeedMeasChart();
        }
    }

    if (hasAnyTelemetry)
    {
        m_lastTelemetryTick = QDateTime::currentMSecsSinceEpoch();
        if (m_statusLink != nullptr)
        {
            setLabelTextIfChanged(m_statusLink, tr("Telemetry OK"));
        }
    }

    if (hasIqRaw || hasIqRef || hasId)
    {
        if (m_tabs != nullptr && m_tabs->currentWidget() == m_iqRefPage)
        {
            refreshIqRefChart();
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

    trimPointBuffer(m_angleMechPoints, 7200);
    trimPointBuffer(m_angleAppPoints, 7200);

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

    const qint64 x = m_speedPoints.size();
    m_speedPoints.append(QPointF(x, speedRpm));
    ++m_speedSampleIndex;

    trimPointBuffer(m_speedPoints, 7200);

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
    if ((now - m_lastSpeedUiTick) >= 50 || m_speedSliderDragging)
    {
        if (m_tabs != nullptr && m_tabs->currentWidget() == m_speedPage)
        {
            refreshSpeedChart();
        }
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

    const int startIndex = qBound(0, m_angleTimeSlider != nullptr ? m_angleTimeSlider->value() : 0, qMax(0, count - 1));
    const int endIndex = qMin(count, startIndex + m_angleWindowSize);
    const int size = qMax(0, endIndex - startIndex);

    m_angleMechSeries->replace(m_angleMechPoints.mid(startIndex, size));
    m_angleAppSeries->replace(m_angleAppPoints.mid(startIndex, size));

    m_angleAxisX->setRange(startIndex, qMax(startIndex + 1, endIndex - 1));
    m_angleAxisX->setLabelFormat("%.0f");

    m_angleAxisY->setRange(-370.0, 370.0);
    m_angleAxisY->setTickCount(11);
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

    const int startIndex = qBound(0, m_speedTimeSlider != nullptr ? m_speedTimeSlider->value() : 0, qMax(0, count - 1));
    const int endIndex = qMin(count, startIndex + m_speedWindowSize);
    const int size = qMax(0, endIndex - startIndex);

    m_speedSeries->replace(m_speedPoints.mid(startIndex, size));

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

    const int startIndex = qBound(0, m_speedMeasTimeSlider != nullptr ? m_speedMeasTimeSlider->value() : 0, qMax(0, count - 1));
    const int endIndex = qMin(count, startIndex + m_speedMeasWindowSize);
    const int size = qMax(0, endIndex - startIndex);

    m_speedMeasSeries->replace(m_speedMeasPoints.mid(startIndex, size));
    m_speedErrSeries->replace(m_speedErrPoints.mid(startIndex, size));
    m_speedMeasAxisX->setRange(startIndex, qMax(startIndex + 1, endIndex - 1));
    m_speedMeasAxisX->setLabelFormat("%.0f");

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
    const int startIndex = qBound(0, m_iqRefTimeSlider != nullptr ? m_iqRefTimeSlider->value() : 0, qMax(0, count - 1));
    const int endIndex = qMin(count, startIndex + m_iqRefWindowSize);
    const int size = qMax(0, endIndex - startIndex);

    QVector<QPointF> rawVisible;
    QVector<QPointF> refVisible;
    QVector<QPointF> idVisible;
    rawVisible.reserve(size);
    refVisible.reserve(size);
    for (const QPointF &p : m_iqRawPoints)
    {
        if (p.x() >= startIndex && p.x() < endIndex)
        {
            rawVisible.append(p);
        }
    }
    for (const QPointF &p : m_iqIdPoints)
    {
        if (p.x() >= startIndex && p.x() < endIndex)
        {
            idVisible.append(p);
        }
    }
    for (const QPointF &p : m_iqRefPoints)
    {
        if (p.x() >= startIndex && p.x() < endIndex)
        {
            refVisible.append(p);
        }
    }

    m_iqRawSeries->replace(rawVisible);
    m_iqRefSeries->replace(refVisible);
    m_iqIdSeries->replace(idVisible);
    m_iqRefAxisX->setRange(startIndex, qMax(startIndex + 1, endIndex - 1));
    m_iqRefAxisX->setLabelFormat("%.0f");
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

void MainWindow::resetIqRefWindow()
{
    m_iqRefWindowSize = 1800;
    m_iqRefAutoFollow = true;
    if (m_iqRefTimeSlider != nullptr)
    {
        m_iqRefTimeSlider->setValue(qMax(0, m_iqRefPoints.size() - m_iqRefWindowSize));
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
        m_speedMeasTimeSlider->setValue(qMax(0, m_speedMeasPoints.size() - m_speedMeasWindowSize));
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
