/*
** adi_initialize.c source file generated on May 14, 2024 at 16:06:02.
**
** Copyright (C) 2000-2024 Analog Devices Inc., All Rights Reserved.
**
** This file is generated automatically. You should not modify this source file,
** as your changes will be lost if this source file is re-generated.
*/

#include <sys/platform.h>
#include <services/int/adi_sec.h>

#include "adi_initialize.h"


int32_t adi_initComponents(void)
{
	int32_t result = 0;

	result = adi_sec_Init();


	return result;
}

