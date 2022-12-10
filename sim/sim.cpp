#include <stdio.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>

#include <cstdint>

#include "setspeed.h"

int main(int argc, char* argv[])
{
  if (argc != 2) {
    printf("missing arg\n");
    return -1;
  }

  int fd = open(argv[1], O_RDWR);

  int speed = 2000;

  setspeed(fd, speed);

  uint64_t zero = 0;
  for (uint8_t i=0; i < 2147483647; i++)
  {
    // write(fd, &zero, sizeof(zero));
    write(fd, &i, sizeof(i));
    // for (size_t j=0; j < 2; j++)
    //write(fd, &zero, sizeof(zero));
  }

  close(fd);

  return 0;
}

