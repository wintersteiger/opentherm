#include <stdio.h>
#include <fcntl.h>
#include <errno.h>
#include <asm/termios.h>

extern "C" int ioctl (int __fd, unsigned long int __request, ...) __THROW;

void setspeed(int fd, int speed)
{
  struct termios2 tio;
  ioctl(fd, TCGETS2, &tio);
  tio.c_cflag &= ~CBAUD;
  tio.c_cflag |= BOTHER;
  tio.c_ispeed = speed;
  tio.c_ospeed = speed;
  int r = ioctl(fd, TCSETS2, &tio);

  if (r != 0) {
    perror("ioctl");
  }
}
