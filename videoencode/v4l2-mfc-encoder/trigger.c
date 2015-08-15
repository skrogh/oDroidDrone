#include <stdio.h>
#include <string.h>

#include <unistd.h>

#include <sys/mman.h>
#include <sys/stat.h>        /* For mode constants */
#include <fcntl.h>           /* For O_* constants */

int main(int argc, char *argv[])
{
	printf("Triggering a new image capture!\n");

  int fd = shm_open("/videoEncoder", O_RDWR, 0600);

  char c[2] = "\n\0";
  write( fd, c, 1);

	return 0;
}
