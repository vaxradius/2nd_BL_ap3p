//*****************************************************************************
//
// Copyright (c) 2020, Ambiq Micro
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
// contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
//
// Third party software included in this distribution is subject to the
// additional license terms as defined in the /docs/licenses directory.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is part of revision 2.4.2 of the AmbiqSuite Development Package.
//
//*****************************************************************************

#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"

#include "am_bootloader.h"
#include "ios_fifo.h"

#define FW_LINK_ADDRESS (0x10000)

#define START_ADDR 0x10000000
#define TOTAL_LENGTH (30*1024)

__asm void am_bootloader_clear_image_run(am_bootloader_image_t *psImage);

bool if_ramdump_required(void)
{
	return true;
}

void spi_ramdump(void)
{
	uint32_t numWritten = 0;
	uint32_t ui32UsedSpace = 0;
	uint32_t ui32LeftSpace = 0;
	uint32_t ui32TotalSent = 0;

	//
    // Enable the IOS
    //
    ios_set_up();

    //
    // Enable interrupts so we can receive messages from the boot host.
    //
    am_hal_interrupt_master_enable();

	while(ui32TotalSent < TOTAL_LENGTH)
	{
		am_hal_ios_fifo_space_left(g_pIOSHandle, &ui32LeftSpace);

		if(ui32LeftSpace >= 512)
		{
			am_hal_ios_fifo_write(g_pIOSHandle, (uint8_t *)(START_ADDR+ui32TotalSent), (ui32TotalSent+512<=TOTAL_LENGTH? 512:(512-((ui32TotalSent+512)-TOTAL_LENGTH))), &numWritten);
			ui32TotalSent += numWritten;
		}
        // If we were Idle - need to inform Host if there is new data
        if (g_iosState == AM_IOSTEST_SLAVE_STATE_NODATA)
        {
            am_hal_ios_fifo_space_used(g_pIOSHandle, &ui32UsedSpace);
            if (ui32UsedSpace)
            {
                g_iosState = AM_IOSTEST_SLAVE_STATE_DATA;
                inform_host();
            }
        }
	}
}

//*****************************************************************************
//
// Main
//
//*****************************************************************************
int
main(void)
{
   	am_bootloader_image_t sImage;
    //
    // Set the clock frequency.
    //
    am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_SYSCLK_MAX, 0);

    //
    // Set the default cache configuration
    //
    am_hal_cachectrl_config(&am_hal_cachectrl_defaults);
    am_hal_cachectrl_enable();

    //
    // Configure the board for low power operation.
    //
    am_bsp_low_power_init();

    //
    // Initialize the printf interface for ITM output
    //
    am_bsp_itm_printf_enable();

    //
    // Print the banner.
    //
    am_util_stdio_terminal_clear();
    am_util_stdio_printf("2nd Bootloader\n\n");

	if(if_ramdump_required())
		spi_ramdump();

	// Jump to application firmware
	sImage.pui32LinkAddress = (uint32_t *)FW_LINK_ADDRESS;
	am_bootloader_clear_image_run(&sImage);
	
    //
    // Loop forever while sleeping.
    //
    while (1)
    {
        //
        // Go to Deep Sleep.
        //
        am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);
    }
}
