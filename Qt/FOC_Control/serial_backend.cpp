#include "serial_backend.h"

#ifdef Q_OS_WIN

#include <QList>
#include <QStringList>
#include <QVector>
#include <devguid.h>
#include <initguid.h>
#include <setupapi.h>

namespace
{
QString readPortNameFromDeviceInfo(HDEVINFO devInfo, SP_DEVINFO_DATA &devInfoData)
{
    QString portName;
    HKEY hKey = SetupDiOpenDevRegKey(devInfo, &devInfoData, DICS_FLAG_GLOBAL, 0, DIREG_DEV, KEY_READ);
    if (hKey == INVALID_HANDLE_VALUE)
    {
        return portName;
    }

    wchar_t value[256] = {0};
    DWORD valueType = 0;
    DWORD valueSize = sizeof(value);
    if (RegQueryValueExW(hKey, L"PortName", nullptr, &valueType, reinterpret_cast<LPBYTE>(value), &valueSize) == ERROR_SUCCESS)
    {
        portName = QString::fromWCharArray(value);
    }

    RegCloseKey(hKey);
    return portName;
}
}

QList<SerialBackend::PortInfo> SerialBackend::availablePorts()
{
    QList<PortInfo> result;
    HDEVINFO devInfo = SetupDiGetClassDevsW(&GUID_DEVCLASS_PORTS, nullptr, nullptr, DIGCF_PRESENT);
    if (devInfo == INVALID_HANDLE_VALUE)
    {
        return result;
    }

    SP_DEVINFO_DATA devInfoData;
    devInfoData.cbSize = sizeof(devInfoData);

    for (DWORD index = 0; SetupDiEnumDeviceInfo(devInfo, index, &devInfoData); ++index)
    {
        wchar_t friendlyName[256] = {0};
        DWORD requiredSize = 0;
        QString friendly;
        if (SetupDiGetDeviceRegistryPropertyW(devInfo,
                                              &devInfoData,
                                              SPDRP_FRIENDLYNAME,
                                              nullptr,
                                              reinterpret_cast<PBYTE>(friendlyName),
                                              sizeof(friendlyName),
                                              &requiredSize))
        {
            friendly = QString::fromWCharArray(friendlyName);
        }

        const QString portName = readPortNameFromDeviceInfo(devInfo, devInfoData);
        if (!portName.isEmpty())
        {
            result.push_back({portName, friendly});
        }
    }

    SetupDiDestroyDeviceInfoList(devInfo);
    return result;
}

SerialBackend::SerialBackend() = default;

SerialBackend::~SerialBackend()
{
    close();
}

bool SerialBackend::open(const QString &portName,
                         int baudRate,
                         int dataBits,
                         int parity,
                         int stopBits,
                         int flowControl,
                         QString *errorString)
{
    close();

    const QString devicePath = QStringLiteral("\\\\.\\%1").arg(portName);
    const std::wstring widePath = devicePath.toStdWString();
    m_handle = CreateFileW(widePath.c_str(),
                           GENERIC_READ | GENERIC_WRITE,
                           0,
                           nullptr,
                           OPEN_EXISTING,
                           0,
                           nullptr);
    if (m_handle == INVALID_HANDLE_VALUE)
    {
        m_errorString = QStringLiteral("CreateFile failed for %1").arg(portName);
        if (errorString != nullptr)
        {
            *errorString = m_errorString;
        }
        return false;
    }

    DCB dcb;
    SecureZeroMemory(&dcb, sizeof(dcb));
    dcb.DCBlength = sizeof(dcb);
    if (!GetCommState(m_handle, &dcb))
    {
        m_errorString = QStringLiteral("GetCommState failed");
        close();
        if (errorString != nullptr)
        {
            *errorString = m_errorString;
        }
        return false;
    }

    dcb.BaudRate = static_cast<DWORD>(baudRate);
    dcb.ByteSize = static_cast<BYTE>(dataBits);
    dcb.fBinary = TRUE;
    dcb.fParity = (parity != 0);
    dcb.Parity = static_cast<BYTE>(parity);
    dcb.StopBits = static_cast<BYTE>(stopBits);
    dcb.fOutxCtsFlow = (flowControl == 1);
    dcb.fRtsControl = (flowControl == 1) ? RTS_CONTROL_HANDSHAKE : RTS_CONTROL_ENABLE;
    dcb.fOutX = (flowControl == 2);
    dcb.fInX = (flowControl == 2);

    if (!SetCommState(m_handle, &dcb))
    {
        m_errorString = QStringLiteral("SetCommState failed");
        close();
        if (errorString != nullptr)
        {
            *errorString = m_errorString;
        }
        return false;
    }

    COMMTIMEOUTS timeouts;
    SecureZeroMemory(&timeouts, sizeof(timeouts));
    timeouts.ReadIntervalTimeout = 1;
    timeouts.ReadTotalTimeoutConstant = 1;
    timeouts.ReadTotalTimeoutMultiplier = 1;
    timeouts.WriteTotalTimeoutConstant = 10;
    timeouts.WriteTotalTimeoutMultiplier = 10;
    SetCommTimeouts(m_handle, &timeouts);
    SetupComm(m_handle, 1 << 16, 1 << 16);
    PurgeComm(m_handle, PURGE_RXCLEAR | PURGE_TXCLEAR);

    if (errorString != nullptr)
    {
        errorString->clear();
    }
    return true;
}

void SerialBackend::close()
{
#ifdef Q_OS_WIN
    if (m_handle != INVALID_HANDLE_VALUE)
    {
        CloseHandle(m_handle);
        m_handle = INVALID_HANDLE_VALUE;
    }
#endif
}

bool SerialBackend::isOpen() const
{
#ifdef Q_OS_WIN
    return m_handle != INVALID_HANDLE_VALUE;
#else
    return false;
#endif
}

qint64 SerialBackend::write(const QByteArray &data, QString *errorString)
{
#ifdef Q_OS_WIN
    if (m_handle == INVALID_HANDLE_VALUE)
    {
        m_errorString = QStringLiteral("Port is not open");
        if (errorString != nullptr)
        {
            *errorString = m_errorString;
        }
        return -1;
    }

    DWORD written = 0;
    if (!WriteFile(m_handle, data.constData(), static_cast<DWORD>(data.size()), &written, nullptr))
    {
        m_errorString = QStringLiteral("WriteFile failed");
        if (errorString != nullptr)
        {
            *errorString = m_errorString;
        }
        return -1;
    }

    if (errorString != nullptr)
    {
        errorString->clear();
    }
    return static_cast<qint64>(written);
#else
    Q_UNUSED(data);
    Q_UNUSED(errorString);
    return -1;
#endif
}

QByteArray SerialBackend::readAll()
{
#ifdef Q_OS_WIN
    QByteArray out;
    if (m_handle == INVALID_HANDLE_VALUE)
    {
        return out;
    }

    DWORD errors = 0;
    COMSTAT stat;
    SecureZeroMemory(&stat, sizeof(stat));
    if (!ClearCommError(m_handle, &errors, &stat))
    {
        return out;
    }

    if (stat.cbInQue == 0)
    {
        return out;
    }

    out.resize(static_cast<int>(stat.cbInQue));
    DWORD readBytes = 0;
    if (!ReadFile(m_handle, out.data(), stat.cbInQue, &readBytes, nullptr))
    {
        out.clear();
        return out;
    }

    out.resize(static_cast<int>(readBytes));
    return out;
#else
    return {};
#endif
}

QString SerialBackend::errorString() const
{
    return m_errorString;
}

#else

QList<SerialBackend::PortInfo> SerialBackend::availablePorts()
{
    return {};
}

SerialBackend::SerialBackend() = default;
SerialBackend::~SerialBackend() = default;
bool SerialBackend::open(const QString &, int, int, int, int, int, QString *) { return false; }
void SerialBackend::close() {}
bool SerialBackend::isOpen() const { return false; }
qint64 SerialBackend::write(const QByteArray &, QString *) { return -1; }
QByteArray SerialBackend::readAll() { return {}; }
QString SerialBackend::errorString() const { return {}; }

#endif
