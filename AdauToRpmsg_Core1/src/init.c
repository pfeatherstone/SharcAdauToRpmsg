/*
 * init.c
 *
 *  Created on: 12 May 2024
 *      Author: pf
 */

/* Standard library headers */
#include <string.h>
#include <assert.h>

/* ADI headers */
#include <sys/adi_core.h>
#include <sruSC589.h>

/* Simple driver headers */
#include "pcg_simple.h"
#include "umm_malloc.h"

/* Local headers */
#include "init.h"
#include "adc/adau1761.h"

#define ADI_RESOURCE_TABLE_INIT_MAGIC 		(0xADE0AD0E)
#define ADI_RESOURCE_TABLE_SHARC1_OFFSET 	(0x400) //1KiB

#define SYSTEM_MCLK_RATE               		(24576000)
#define SYSTEM_SAMPLE_RATE             		(48000)
#define SYSTEM_BLOCK_SIZE              		(64)

#define BITP_PADS0_DAI0_IE_MCLK 			BITP_PADS0_DAI0_IE_PB06
#define DAI0_MCLK_PIN 						6

/*
 * Expected resource table layout in the shared memory.
 * Initialized by ARM.
 */
RL_PACKED_BEGIN
struct sharc_resource_table {
	struct resource_table table_hdr;
	unsigned int offset[1];
	struct fw_rsc_vdev rpmsg_vdev;
	struct fw_rsc_vdev_vring vring[2];
}RL_PACKED_END;

RL_PACKED_BEGIN
struct adi_resource_table{
	uint8_t tag[16];
	uint32_t version;
	uint32_t initialized;
	uint32_t reserved[8];

	struct sharc_resource_table tbl;
}RL_PACKED_END;


const struct adi_resource_table rsc_tbl_local = {
		.tag = "AD-RESOURCE-TBL",
		.version = 1,
		.initialized = 0,
		.tbl.table_hdr = {
			/* resource table header */
			1, 								 /* version */
			1, /* number of table entries */
			{0, 0,},					 /* reserved fields */
		},
		.tbl.offset = {offsetof(struct sharc_resource_table, rpmsg_vdev),
		},
		.tbl.rpmsg_vdev = {RSC_VDEV, /* virtio dev type */
			7, /* it's rpmsg virtio */
			1, /* kick sharc0 */
			/* 1<<0 is VIRTIO_RPMSG_F_NS bit defined in virtio_rpmsg_bus.c */
			1<<0, 0, 0, 0, /* dfeatures, gfeatures, config len, status */
			2, /* num_of_vrings */
			{0, 0,}, /* reserved */
		},
		.tbl.vring = {
			{(uint32_t)-1, VRING_ALIGN, 512, 1, 0}, /* da allocated by remoteproc driver */
			{(uint32_t)-1, VRING_ALIGN, 512, 1, 0}, /* da allocated by remoteproc driver */
		},
};

/*
 * Two resource tables, one for each core.
 * The ___MCAPI_common_start address is defined in app.ldf
 */
extern "asm" struct adi_resource_table ___MCAPI_common_start;
volatile struct adi_resource_table *adi_resource_table;
volatile struct sharc_resource_table *resource_table;

/*
 * Helper struct which represents memory ranges used by a vring.
 */
struct _mem_range{
	uint32_t start;
	uint32_t end;
};

/*
 * Helper function which reads memory ranges used by a vring.
 */
void vring_get_descriptor_range(volatile struct fw_rsc_vdev_vring *vring, struct _mem_range *range)
{
	struct vring_desc *desc = (struct vring_desc *)vring->da;
	range->start 			= (uint32_t)desc;
	range->end 				= (uint32_t)desc + vring_size(vring->num, vring->align);
}

void vring_get_buffer_range(volatile struct fw_rsc_vdev_vring *vring, struct _mem_range *range)
{
	struct vring_desc *desc = (struct vring_desc *)vring->da;
	const uint32_t num 		= 2 * vring->num; // vring0 descriptor has pointer to buffers for both vrings
	range->start 			= (uint32_t)desc->addr;
	range->end 				= (uint32_t)desc->addr + num * (RL_BUFFER_PAYLOAD_SIZE +16);
}

void init_rsc_tbl(void)
{
	switch(adi_core_id()){
	case ADI_CORE_ARM:
		return;
	case ADI_CORE_SHARC0:
		adi_resource_table 	= &___MCAPI_common_start;
		resource_table 		= &___MCAPI_common_start.tbl;
		break;
	case ADI_CORE_SHARC1:
		adi_resource_table 	= (struct adi_resource_table *)((uint32_t)&___MCAPI_common_start + ADI_RESOURCE_TABLE_SHARC1_OFFSET);
		resource_table 		= &adi_resource_table->tbl;
		break;
	default:
		// should never happen
		break;
	}

	/* Don't initialize if remoteproc driver has already */
	if(strcmp((const char *)adi_resource_table->tag, (const char *)rsc_tbl_local.tag))
	{
		*adi_resource_table = rsc_tbl_local;

		switch(adi_core_id()){
		case ADI_CORE_ARM:
			return;
		case ADI_CORE_SHARC0:
			adi_resource_table->tbl.rpmsg_vdev.notifyid = 1;
			adi_resource_table->tbl.vring[0].notifyid = 1;
			adi_resource_table->tbl.vring[1].notifyid = 1;
			break;
		case ADI_CORE_SHARC1:
			adi_resource_table->tbl.rpmsg_vdev.notifyid = 2;
			adi_resource_table->tbl.vring[0].notifyid = 2;
			adi_resource_table->tbl.vring[1].notifyid = 2;
			break;
		default:
			// should never happen
			break;
		}
	}
}

int rsc_tbl_ready(void)
{
	/* 0x1 acknowledge, 0x2 driver found, 0x4 driver ready*/
	return resource_table->rpmsg_vdev.status == 7;
}

/*
 * Initialize rpmsg channel to ARM core
 */
int rpmsg_init (
	struct rpmsg_lite_instance* ctx
)
{
	struct rpmsg_lite_instance *rpmsg_instance;
	adiCacheStatus status;
	struct _mem_range range0;
	struct _mem_range range1;

	init_rsc_tbl();
	while(!rsc_tbl_ready()){
		/* Wait for resource table to be initialized by ARM*/
	}

	// Get memory range which needs disabled cache
	// Read vring descriptors memory range
	vring_get_descriptor_range(&resource_table->vring[0], &range0);
	vring_get_descriptor_range(&resource_table->vring[1], &range1);
	range0.start = min(range0.start, range1.start);
	range0.end = max(range0.end, range1.end);
	// Disable cache for the descriptors memory range
	status = adi_cache_set_range ((void *)range0.start,
						(void *)(range0.end),
						adi_cache_rr6,
						adi_cache_noncacheable_range);

	// Read vring buffer memory range
	// vring1 has its own descriptors but share buffers with vring0
	vring_get_buffer_range(&resource_table->vring[0], &range1);
	// Disable cache for the vring buffer range
	status = adi_cache_set_range ((void *)range1.start,
						(void *)(range1.end),
						adi_cache_rr7,
						adi_cache_noncacheable_range);

	if(rpmsg_lite_remote_init(
			(void*)&resource_table->rpmsg_vdev,
			RL_PLATFORM_SHARC_ARM_LINK_ID,
			RL_SHM_VDEV,
			ctx) == RL_NULL) {
		return -1;
	}

	adi_resource_table->initialized = ADI_RESOURCE_TABLE_INIT_MAGIC;

	/*
	 * Wait until ARM notifies the channel is up.
	 */
	while(!rpmsg_lite_is_link_up(ctx));

	return 0;
}

int rpmsg_init_channel (
	struct rpmsg_lite_instance* 			ctx,
	struct rpmsg_lite_ept_static_context*	ept,
	uint32_t 								addr,
	const char*								name,
	rl_ept_rx_cb_t							clb,
	void*									priv_ptr
)
{
	struct rpmsg_lite_endpoint *rpmsg_ept = rpmsg_lite_create_ept (
			ctx,
			addr,
			clb,
			priv_ptr,
			ept);

	if(rpmsg_ept == RL_NULL){
		return -1;
	}

	const int ret = rpmsg_ns_announce (
		ctx,
		rpmsg_ept,
		name,
		RL_NS_CREATE);

	return ret == RL_SUCCESS ? 0 : -1;
}

void rpmsg_free_channel (
	struct rpmsg_lite_instance* 			ctx,
	struct rpmsg_lite_ept_static_context*	ept,
	const char*								name
)
{
	rpmsg_ns_announce (
		ctx,
		&ept->ept,
		name,
		RL_NS_DESTROY);

	rpmsg_lite_destroy_ept(ctx, &ept->ept);
}

struct codec_state
{
	sTWI* 		adau1761TwiHandle;
	sSPORT*		codecSportOutHandle;
	sSPORT*		codecSportInHandle;
    void*		codecAudioIn[2];
    void*		codecAudioOut[2];
    unsigned 	codecAudioInLen;
	unsigned 	codecAudioOutLen;
};

/***********************************************************************
 * This function allocates audio buffers in L3 cached memory and
 * initializes a single SPORT using the simple SPORT driver.
 **********************************************************************/
static sSPORT* single_sport_init (
	SPORT_SIMPLE_PORT 			sport,
    SPORT_SIMPLE_CONFIG* 		cfg,
	SPORT_SIMPLE_AUDIO_CALLBACK cb,
    void**						pingPongPtrs,
	unsigned*					pingPongLen,
	void*						usrPtr,
    bool 						cached,
	SPORT_SIMPLE_RESULT* 		result
)
{
    sSPORT *sportHandle;
    SPORT_SIMPLE_RESULT sportResult;
    uint32_t dataBufferSize;
    int i;

    /* Open a handle to the SPORT */
    sportResult = sport_open(sport, &sportHandle);
    if (sportResult != SPORT_SIMPLE_SUCCESS) {
        if (result) {
            *result = sportResult;
        }
        return(NULL);
    }

    /* Copy application callback info */
    cfg->callBack = cb;
    cfg->usrPtr = usrPtr;

    /* Allocate audio buffers if not already allocated */
    dataBufferSize = sport_buffer_size(cfg);
    for (i = 0; i < 2; i++) {
        if (!cfg->dataBuffers[i]) {
            cfg->dataBuffers[i] = umm_malloc_heap_aligned(
                UMM_SDRAM_HEAP, dataBufferSize, sizeof(uint32_t));
            memset(cfg->dataBuffers[i], 0, dataBufferSize);
        }
    }
    cfg->dataBuffersCached = cached;
    cfg->syncDMA = true;

    /* Configure the SPORT */
    sportResult = sport_configure(sportHandle, cfg);

    /* Save ping pong data pointers */
    if (pingPongPtrs) {
        pingPongPtrs[0] = cfg->dataBuffers[0];
        pingPongPtrs[1] = cfg->dataBuffers[1];
    }
    if (pingPongLen) {
        *pingPongLen = dataBufferSize;
    }
    if (result) {
        *result = sportResult;
    }

    return(sportHandle);
}

/***********************************************************************
 * Simple SPORT driver 8-ch TDM settings
 **********************************************************************/
SPORT_SIMPLE_CONFIG cfgTDM8x1 = {
    .clkDir 		= SPORT_SIMPLE_CLK_DIR_SLAVE,
    .fsDir 			= SPORT_SIMPLE_FS_DIR_MASTER,
    .dataDir 		= SPORT_SIMPLE_DATA_DIR_UNKNOWN,
    .bitClkOptions 	= SPORT_SIMPLE_CLK_FALLING,
    .fsOptions 		= SPORT_SIMPLE_FS_OPTION_EARLY,
    .tdmSlots 		= SPORT_SIMPLE_TDM_8,
    .wordSize 		= SPORT_SIMPLE_WORD_SIZE_32BIT,
    .dataEnable 	= SPORT_SIMPLE_ENABLE_PRIMARY,
    .frames 		= SYSTEM_BLOCK_SIZE,
};

void adau1761_sport_init (
	struct codec_state *context,
	SPORT_SIMPLE_AUDIO_CALLBACK codecAudioOut,
	SPORT_SIMPLE_AUDIO_CALLBACK codecAudioIn
)
{
    SPORT_SIMPLE_CONFIG sportCfg;
    SPORT_SIMPLE_RESULT sportResult;
    unsigned len;

    /* SPORT0A: CODEC data out */
    sportCfg 					= cfgTDM8x1;
    sportCfg.dataDir 			= SPORT_SIMPLE_DATA_DIR_TX;
    sportCfg.fsDir 				= SPORT_SIMPLE_FS_DIR_MASTER;
    sportCfg.dataBuffersCached 	= false;
    memcpy(sportCfg.dataBuffers, context->codecAudioOut, sizeof(sportCfg.dataBuffers));
    context->codecSportOutHandle = single_sport_init(
        SPORT0A, &sportCfg, codecAudioOut,
        NULL, &len, context, false, NULL
    );
    assert(context->codecAudioOutLen == len);


    /* SPORT0B: CODEC data in */
    sportCfg 					= cfgTDM8x1;
    sportCfg.dataDir 			= SPORT_SIMPLE_DATA_DIR_RX;
    sportCfg.fsDir 				= SPORT_SIMPLE_FS_DIR_SLAVE;
    sportCfg.dataBuffersCached 	= false;
    memcpy(sportCfg.dataBuffers, context->codecAudioIn, sizeof(sportCfg.dataBuffers));
    context->codecSportInHandle = single_sport_init(
        SPORT0B, &sportCfg, codecAudioIn,
        NULL, &len, context, false, NULL
    );
    assert(context->codecAudioInLen == len);

    /* Start SPORT0A/B */
    sportResult = sport_start(context->codecSportOutHandle, true);
    sportResult = sport_start(context->codecSportInHandle, true);
}

void adau1761_sport_deinit(struct codec_state *context)
{
    if (context->codecSportOutHandle) {
        sport_close(&context->codecSportOutHandle);
    }
    if (context->codecSportInHandle) {
        sport_close(&context->codecSportInHandle);
    }
}

/***********************************************************************
 * PCGA generates 12.288 MHz TDM8 BCLK from 24.576 MCLK/BCLK
 **********************************************************************/
void adau1761_cfg_pcg(void)
{
    /* Configure static PCG A parameters */
    PCG_SIMPLE_CONFIG pcg_a = {
        .pcg 					= PCG_A,         	// PCG A
        .clk_src 				= PCG_SRC_DAI_PIN,	// Sourced from DAI
        .clk_in_dai_pin 		= DAI0_MCLK_PIN, 	// Sourced from DAI MCLK pin
        .lrclk_clocks_per_frame = 256,   // Not used
        .sync_to_fs = false
    };

    /* Configure the PCG BCLK depending on the cfgTDM8x1 SPORT config */
    pcg_a.bitclk_div = SYSTEM_MCLK_RATE / (cfgTDM8x1.wordSize * cfgTDM8x1.tdmSlots * SYSTEM_SAMPLE_RATE);
    assert(pcg_a.bitclk_div > 0);

    /* This sets everything up */
    pcg_open(&pcg_a);
    pcg_enable(PCG_A, true);
}

/***********************************************************************
 * ADAU1761 CODEC / SPORT0 / SRU initialization (TDM8 clock slave)
 **********************************************************************/
static void sru_config_sharc_sam_adau1761_slave(void)
{
    SRU(HIGH, DAI0_PBEN01_I);        // ADAU1761 DAC data is an output
    SRU(LOW,  DAI0_PBEN02_I);        // ADAU1761 ADC data is an input
    SRU(HIGH, DAI0_PBEN03_I);        // ADAU1761 CLK is an output
    SRU(HIGH, DAI0_PBEN04_I);        // ADAU1761 FS is an output

    SRU(PCG0_CLKA_O, SPT0_ACLK_I);   // PCG-A BCLK to SPORT0A BCLK
    SRU(PCG0_CLKA_O, SPT0_BCLK_I);   // PCG-A BCLK to SPORT0A BCLK
    SRU(PCG0_CLKA_O, DAI0_PB03_I);   // PCG-A BCLK to ADAU1761

    SRU(SPT0_AFS_O, DAI0_PB04_I);    // SPORT0A FS to ADAU1761
    SRU(SPT0_AFS_O, SPT0_BFS_I);     // SPORT0A FS to SPORT0B

    SRU(DAI0_PB02_O, SPT0_BD0_I);     // ADAU1761 ADC pin to SPORT0B input
    SRU(SPT0_AD0_O,  DAI0_PB01_I);    // SPORT0A output to ADAU1761 DAC pin
}

#define SAM_ADAU1761_I2C_ADDR  (0x38)

void adau1761_init (
	struct codec_state* 		context,
	SPORT_SIMPLE_AUDIO_CALLBACK codecAudioOut,
	SPORT_SIMPLE_AUDIO_CALLBACK codecAudioIn
)
{
    /* Configure the DAI routing */
    sru_config_sharc_sam_adau1761_slave();

    /* Configure the 1761 bit clock PCG */
    adau1761_cfg_pcg();

    /* Initialize the CODEC */
    init_adau1761(context->adau1761TwiHandle, SAM_ADAU1761_I2C_ADDR);

    /* Initialize the CODEC SPORTs */
    adau1761_sport_init(context, codecAudioOut, codecAudioIn);
}
