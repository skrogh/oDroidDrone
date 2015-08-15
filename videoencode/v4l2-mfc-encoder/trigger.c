#include <stdio.h>
#include <string.h>

int main(int argc, char *argv[])
{
	printf("Triggering a new image capture!\n");

  int fd = shm_open("/videoEncoder", O_RDWR, 0600);

  

	return 0;
}
