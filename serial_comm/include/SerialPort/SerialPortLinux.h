#ifndef SERIALPORTLINUX_H
#define SERIALPORTLINUX_H

#include <string>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <cerrno>

enum serialParity {
    PARITY_NONE = 0,
    PARITY_ODD,
    PARITY_EVEN
};

class SerialPortLinux
{
    public:
        SerialPortLinux();
        virtual ~SerialPortLinux();

        void SetPort(std::string devicePath) { m_portStr = devicePath; }

        //! The baudrate should be a B115200 type of number...
        void SetBaudRate(int baudrate) { m_baudrate = baudrate; }
        void SetParity(serialParity par) { m_parity = par; }

        int Open(std::string devicePath, int baudrate, serialParity par);

        int Close();

        void FlushInput();

        int sendBytes(unsigned char* buffer, int len);
        int readBytes(unsigned char* buffer, int len);
        
        bool isOpen() { return m_isOpen; }

    protected:
    private:
		bool m_isOpen;
        std::string m_portStr;
        int m_baudrate;
        serialParity m_parity;
        int m_fd;	// file descriptor for device handle
};

#endif // SERIALPORTLINUX_H
