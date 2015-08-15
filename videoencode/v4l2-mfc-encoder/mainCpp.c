#include <cstdio>
#include <cstring>

#include <sys/mman.h>
#include <sys/stat.h>        /* For mode constants */
#include <fcntl.h>           /* For O_* constants */
#include <sys/eventfd.h>

#include "encoder.h"
#include "args.h"

int main(int argc, char *argv[])
{
	struct options opts;

	printf("mfc codec encoding example application\n"
	       "Andrzej Hajda <a.hajda@samsung.com>\n"
	       "Copyright 2012 Samsung Electronics Co., Ltd.\n\n");

	if (parse_args(&opts, argc, argv)) {
		print_usage(argv[0]);
		return 1;
	}

	return encoderStart( &opts );
}
