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
	int encoderFd;
	int codec;
	int width;
	int height;
	int duration;
	int rate;
	int nctrls;
	int ctrls[MAX_CTRLS][2];
	char **NU12_ARRAY;
};

/*
 Spawn encoder thread and start waiting for images
Arguments are simply the optins block:
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
void encoderStart( struct options *opts );

/*
	Trigger new conversion
*/
void encoderTriggerConversion(  struct options *opts );

/*
	options:		<structure>						-	Options structure to initialize
	mfc_name: 	<path>								-	Path to mfc encoder
	out_name:		<path>								-	Path to output file
	width:			<int>									-	Image width
	height:			<int>									-	Image height
 	codex:			<codec>[,param[=val]]	-	Codex and parameters used.
																		-	Parameters can be seen in ctrls struct above
	NU12_ARRAY:	<array of arrays>			-	Memory area fro imae to be encoded.
																		-	Must be (width*height)+(width*height)/2
*/
int parse_args(struct options *opts,
		char *mfc_name, char *out_name, int width, int height, char codex[],
		char **NU12_ARRAY);

#endif
