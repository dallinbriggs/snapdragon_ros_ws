#include <stdio.h>
#include <string.h>
#include "SerialPort/SerialPortLinux.h"
#include <unistd.h>
#include <errno.h>

SerialPortLinux::SerialPortLinux()
:	m_isOpen(false)
{
    m_fd = 0;
}

SerialPortLinux::~SerialPortLinux()
{
    //dtor
}

int SerialPortLinux::sendBytes(unsigned char* buffer, int len)
{
	if (!m_isOpen) {
		return -1;
	}
	return write(m_fd, buffer, len);
}

int SerialPortLinux::readBytes(unsigned char* buffer, int len)
{
	if (!m_isOpen) {
		printf("Not open yet\n");
		return -1;
	}
	
	return read(m_fd, buffer, len);
}

int SerialPortLinux::Open(std::string devicePath, int baudrate, serialParity par)
{
	struct termios terminalSettings;
	// http://linux.die.net/man/2/open

	m_fd = open(devicePath.c_str(), O_RDWR | O_NOCTTY | O_NDELAY | O_FSYNC | O_NONBLOCK);
	if (m_fd < 0) {
		printf("Failed to open serial port %s\n", devicePath.c_str());
		printf("open error: %s\n", strerror(errno));
		return m_fd;
	}

	// clear terminal attributes
	memset(&terminalSettings, 0, sizeof(struct termios));

	//setReceiveCallback();

	//! Set control flags:
	terminalSettings.c_cflag = baudrate | CS8 | CLOCAL | CREAD;

	//! Set Input Flags:
	terminalSettings.c_iflag = IGNPAR; // ONLCR? - do we want that?

	//! Set Output Flags:
	// Don't set any flags in c_oflag for now.  Leave alone.

	//! Set up non-canonical mode:
	terminalSettings.c_cc[VTIME] = 0;
	terminalSettings.c_cc[VMIN] = 1;

	//! Set the settings!
	tcsetattr(m_fd, TCSANOW, &terminalSettings);

	//! flush out input and output buffers:
	tcflush(m_fd, TCOFLUSH);
	tcflush(m_fd, TCIFLUSH);
	
	m_isOpen = true;
	
	printf("Opened Serial Port: %s\n", devicePath.c_str());
	return 0;
}

void SerialPortLinux::FlushInput()
{
    tcflush(m_fd, TCIFLUSH);
}

int SerialPortLinux::Close()
{
	if (!m_isOpen) {
		return -1;
	}
		
	close(m_fd);
	m_fd = 0;
	m_isOpen = false;
	printf("Closed Serial Port\n");
	return 0;
}
