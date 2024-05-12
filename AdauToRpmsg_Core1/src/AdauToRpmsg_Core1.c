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
	struct rpmsg_lite_instance 				ctx;
	struct rpmsg_lite_ept_static_context 	ept;
	const char*								ept_name;
	uint32_t								ept_addr;
	bool 									running;
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

static struct app_state CTX = {
	.ept_name = "sharc_audio",
	.ept_addr = 155,
	.running  = false
};

int main(int argc, char *argv[])
{
	/**
	 * Initialize managed drivers and/or services that have been added to 
	 * the project.
	 * @return zero on success 
	 */
	adi_initComponents();
	
	/* Begin adding your custom code here */
	rpmsg_init(&CTX.ctx);
	rpmsg_init_channel(&CTX.ctx, &CTX.ept, CTX.ept_addr, CTX.ept_name, &rpmsg_callback, &CTX);

	/* cleanup */
	rpmsg_free_channel(&CTX.ctx, &CTX.ept, CTX.ept_name);
	rpmsg_lite_deinit(&CTX.ctx);

	return 0;
}

