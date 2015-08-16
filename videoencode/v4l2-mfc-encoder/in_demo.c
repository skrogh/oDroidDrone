/*
 * mfc codec encoding example application
 * Andrzej Hajda <a.hajda@samsung.com>
 *
 * Sample input device.
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

#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <arm_neon.h>

#include "common.h"
#include "in_demo.h"
#include "func_dev.h"

#define SQUARE_SIZE 5
#define MIN(X,Y) ((X) < (Y) ? (X) : (Y))

struct in_demo_priv {
	int width;
	int height;
	char **NU12_ARRAY;
};

static int in_demo_read(struct io_dev *dev, int nbufs, char **bufs, int *lens)
{
	dbg( "Processing input image\n" );
	dbg( "consuming trigger\n" );
	char c[8];
	char a = read( dev->fd, c, 8);
	dbg( "consumed trigger\n" );


	struct in_demo_priv *p;
	int size;

	p = dev->priv;

	if (nbufs != 2)
		return -1;

	size = p->width * p->height;
	if (size > lens[0] || size > 2 * lens[1]) {
		err("Size=%d len=%d,%d", size, lens[0], lens[1]);
		return -1;
	}

	memset( bufs[0], 0, size );
	memset( bufs[1], 128, size / 2 );
	//memcpy(bufs[0], p->NU12_ARRAY[0], size);
	//memcpy(bufs[1], p->NU12_ARRAY[1], size / 2);

	// copy blue
	int8_t* src = p->NU12_ARRAY[2];
	int8_t* luma = bufs[0];
	int i;
	for( i = 0; i < size/16; i++ ) {
		int8x16x3_t bgr = vld3q_s8( src ); //load 16 pixels at 8-bits into 3 channels
		vst1q_s8( luma, bgr.val[0] );
		src += 16*3;
		luma += 16;
	}


	return size;
}

static int in_demo_destroy(struct io_dev *dev)
{
	free(dev->priv);

	return func_destroy(dev);
}

static struct io_dev_ops in_demo_ops = { .read = in_demo_read,
					 .req_bufs = func_req_bufs,
					 .enq_buf = func_enq_buf,
					 .deq_buf = func_deq_buf,
					 .destroy = in_demo_destroy
					};

struct io_dev *in_demo_create(int width, int height, int encoderFd, char **NU12_ARRAY)
{
	struct io_dev *dev;
	struct in_demo_priv *priv;

	dev = malloc(sizeof(*dev));
	memzero(*dev);

	priv = malloc(sizeof(struct in_demo_priv));
	priv->width = width;
	priv->height = height;
	priv->NU12_ARRAY = NU12_ARRAY;

	dev->fd = encoderFd;
	dev->io[DIR_IN].type = IO_NONE;
	dev->io[DIR_OUT].type = IO_FUNC;
	dev->ops = &in_demo_ops;

	dev->priv = priv;

	return dev;
}
