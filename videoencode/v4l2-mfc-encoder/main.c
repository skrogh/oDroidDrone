/*
 * mfc codec encoding example application
 * Andrzej Hajda <a.hajda@samsung.com>
 *
 * Main file of the application.
 *
 * Copyright 2012 Samsung Electronics Co., Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include <stdio.h>
#include <string.h>

#include "encoder.h"

int main(int argc, char *argv[])
{
	struct options opts;

	printf("mfc codec encoding example application\n"
	       "Andrzej Hajda <a.hajda@samsung.com>\n"
	       "Copyright 2012 Samsung Electronics Co., Ltd.\n\n");

	if (parse_args(&opts, "/dev/video9", "video.avi", 640, 480, "h264")) {
		print_usage(argv[0]);
		return 1;
	}


	return encoderStart( &opts );
}
