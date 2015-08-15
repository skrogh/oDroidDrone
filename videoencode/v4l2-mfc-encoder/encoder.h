/*
 * mfc codec encoding example application
 * Andrzej Hajda <a.hajda@samsung.com>
 *
 * Argument parser header file.
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

#ifndef ENCODER_H
#define ENCODER_H

#include "common.h"

#define MAX_CTRLS 100

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
int encoderStart( struct options *opts );

#endif