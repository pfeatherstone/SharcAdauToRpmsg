/*****************************************************************************
 * AdauToRpmsg_Core1.c
 *****************************************************************************/

#include <sys/platform.h>
#include "adi_initialize.h"
#include "init.h"

/** 
 * If you want to use command program arguments, then place them in the following string. 
 */
char __argv_string[] = "";

struct app_state
{
	struct codec_state 						codec;
	struct rpmsg_lite_instance 				ctx;
	struct rpmsg_lite_ept_static_context 	ept;
	const char*								ept_name;
	uint32_t								ept_addr;
	bool 									running;
	uint16_t 								seq;
};

int32_t rpmsg_callback(void *payload, uint32_t payload_len, uint32_t src, void *priv)
{
	struct app_state* ctx = (struct app_state*)priv;

	if (payload_len >= sizeof(struct rpmsg_packet))
	{
		struct rpmsg_packet* req = (struct rpmsg_packet*)payload;
		struct rpmsg_packet rep = {.pad = 0, .seq = 0};
		bool do_send = false;

		switch(req->type)
		{
		case RPMSG_START:
			rep.type 		= RPMSG_STARTED;
			ctx->running 	= true;
			ctx->seq 		= 0;
			do_send 		= true;
			break;
		case RPMSG_STOP:
			rep.type 		= RPMSG_STOPPED;
			ctx->running 	= false;
			do_send 		= true;
			break;
		default:
			break;
		}

		if (do_send)
		{
			rpmsg_lite_send (
				&ctx->ctx,
				&ctx->ept.ept,
				ctx->ept_addr,
				(char*)&rep,
				sizeof(rep),
				100);
		}
	}

	/* nothing to do */
	return RL_SUCCESS;
}

void onAudio(void* buffer, uint32_t size, void *usrPtr)
{
	struct app_state* ctx = (struct app_state*)usrPtr;

	// Received audio. Send over RPMsg
	// Do we need a synchronization primitive (something like a mutex lock but for bare metal) ?

	if (ctx->running)
	{
		struct rpmsg_packet rep;
		rep.type = RPMSG_DATA;
		rep.seq  = ctx->seq++;

		rpmsg_lite_send (
			&ctx->ctx,
			&ctx->ept.ept,
			ctx->ept_addr,
			(char*)&rep,
			sizeof(rep),
			100);

		rpmsg_lite_send (
			&ctx->ctx,
			&ctx->ept.ept,
			ctx->ept_addr,
			buffer,
			size,
			100);
	}
}

static struct app_state CTX;

int main(int argc, char *argv[])
{
	/**
	 * Initialize managed drivers and/or services that have been added to 
	 * the project.
	 * @return zero on success 
	 */
	adi_initComponents();
	
	// Initialize context
	memset(&CTX.codec, 0, sizeof(struct codec_state));
	CTX.ept_name 	= "sharc_audio";
	CTX.ept_addr	= 155;
	CTX.running 	= false;
	CTX.seq 		= 0;

	// Initialize memory
	heap_initialize();

	// Initialize codec
	adau1761_init(&CTX.codec, NULL, &onAudio, &CTX);

	// Initialize RPMsg
	rpmsg_init(&CTX.ctx);
	rpmsg_init_channel(&CTX.ctx, &CTX.ept, CTX.ept_addr, CTX.ept_name, &rpmsg_callback, &CTX);


	// Cleanup
	rpmsg_free_channel(&CTX.ctx, &CTX.ept, CTX.ept_name);
	rpmsg_lite_deinit(&CTX.ctx);

	return 0;
}

