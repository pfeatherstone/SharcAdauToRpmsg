/*
 * init.h
 *
 *  Created on: 12 May 2024
 *      Author: pf
 */

#ifndef INIT_H_
#define INIT_H_

/* RPMsg headers*/
#include <rpmsg_platform.h>
#include <rpmsg_lite.h>
#include <rpmsg_ns.h>

/* Simple driver headers */
#include "twi_simple.h"
#include "sport_simple.h"
#include "pcg_simple.h"


int rpmsg_init (
	struct rpmsg_lite_instance* ctx
);

int rpmsg_init_channel (
	struct rpmsg_lite_instance* 			ctx,
	struct rpmsg_lite_ept_static_context*	ept,
	uint32_t 								addr,
	const char*								name,
	rl_ept_rx_cb_t							clb,
	void*									priv_ptr
);

void rpmsg_free_channel (
	struct rpmsg_lite_instance* 			ctx,
	struct rpmsg_lite_ept_static_context*	ept,
	const char*								name
);

typedef enum {
	RPMSG_START = 0, // ARM 	-> SHARC
	RPMSG_STARTED,	 // SHARC	-> ARM
	RPMSG_STOP,		 // ARM		-> SHARC
	RPMSG_STOPPED	 // SHARC	-> ARM
} rpmsg_type_t;

struct rpmsg_packet
{
	uint8_t 	type;
	uint8_t 	pad;
	uint16_t 	seq;
};

#endif /* INIT_H_ */
