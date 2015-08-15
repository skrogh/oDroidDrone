extern "C" {
  #include "encoder.h"
  #include "args.h"
}

#include <iostream>

int main(int argc, char *argv[])
{
	struct options opts;

	std::cout << "mfc codec encoding example application" << std:endl
	       "Original by Andrzej Hajda <a.hajda@samsung.com>" << std:endl
	       "Copyright 2012 Samsung Electronics Co., Ltd." << std:endl
         "openCV interface by Søren Andersen." << std:endl
         "Copyright 2015" << std:endl << std:endl;

	if (parse_args(&opts, argc, argv)) {
		print_usage(argv[0]);
		return 1;
	}


	return encoderStart( &opts );
}
