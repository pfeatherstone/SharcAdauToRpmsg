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
};

static struct app_state CTX = {
	.ept_name = "sharc_audio",
	.ept_addr = 155
};

int32_t rpmsg_callback(void *payload, uint32_t payload_len, uint32_t src, void *priv)
{
	/* nothing to do */
	return RL_SUCCESS;
}

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
	rpmsg_init_channel(&CTX.ctx, &CTX.ept, CTX.ept_addr, CTX.ept_name, &rpmsg_callback, NULL);

	/* cleanup */
	rpmsg_free_channel(&CTX.ctx, &CTX.ept, CTX.ept_name);
	rpmsg_lite_deinit(&CTX.ctx);

	return 0;
}

