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

#include "common.h"
#include "encoder.h"
#include "in_demo.h"
#include "out_file.h"
#include "io_dev.h"
#include "mfc.h"
#include "v4l_dev.h"

/*
* Makes a blokcing call and starts the encoder, looping over the given array and encoding
* Arguments are simply the optins block:
struct options {
	char *mfc_name;
	char *out_name;
	int codec;
	int width;
	int height;
	int duration;
	int rate;
	int nctrls;
	int ctrls[MAX_CTRLS][2];
};
*/
int encoderStart( struct options *opts )
{
	struct io_dev *input;
	struct io_dev *mfc;
	struct io_dev *output;
	int i;

	struct io_dev *chain[3] = {};


	input = in_demo_create(opts->width, opts->height, opts->encoderFd);
	if (input == NULL)
		return 1;
	chain[0] = input;

	input->io[DIR_OUT].limit = opts->duration;

	mfc = mfc_create(opts->mfc_name);
	if (mfc == NULL)
		return 1;
	chain[1] = mfc;

	if (mfc_set_fmt(mfc, DIR_IN, opts->width, opts->height))
		return 1;

	if (mfc_set_codec(mfc, DIR_OUT, opts->codec))
		return 1;

	if (mfc_set_rate(mfc, opts->rate))
		return 1;

	for (i = 0; i < opts->nctrls; ++i)
		mfc_set_mpeg_control(mfc, opts->ctrls[i][0], opts->ctrls[i][1]);

	output = out_file_create(opts->out_name);
	if (output == NULL)
		return 1;
	chain[2] = output;

	if (dev_bufs_create(input, mfc, MFC_ENC_IN_NBUF))
		return 1;

	if (dev_bufs_create(mfc, output, MFC_ENC_OUT_NBUF))
		return 1;

	if (process_chain(chain, array_len(chain)))
		return 1;

	for (i = 0; i < array_len(chain); ++i)
		chain[i]->ops->destroy(chain[i]);

	return 0;
}
