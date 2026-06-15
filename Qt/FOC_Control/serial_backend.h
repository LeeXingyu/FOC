#ifndef SERIAL_BACKEND_H
#define SERIAL_BACKEND_H

#include <QByteArray>
#include <QList>
#include <QString>

#ifdef Q_OS_WIN
#include <windows.h>
#endif

class SerialBackend
{
public:
    struct PortInfo
    {
        QString portName;
        QString friendlyName;
    };

    static QList<PortInfo> availablePorts();

    SerialBackend();
    ~SerialBackend();

    bool open(const QString &portName,
              int baudRate,
              int dataBits,
              int parity,
              int stopBits,
              int flowControl,
              QString *errorString = nullptr);
    void close();
    bool isOpen() const;
    qint64 write(const QByteArray &data, QString *errorString = nullptr);
    QByteArray readAll();
    QString errorString() const;

private:
    QString m_errorString;
#ifdef Q_OS_WIN
    HANDLE m_handle = INVALID_HANDLE_VALUE;
#endif
};

#endif // SERIAL_BACKEND_H
