/**
 * Copyright (c) 2016 - 2018, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */

#include <stdbool.h>
#include <stdint.h>
#include "app_error.h"
#include "app_util_platform.h"
#include "sdk_config.h"
#include "spi_flash.h"
#include "spi_flash_block.h"


static bool m_is_init = false;

ret_code_t spi_flash_block_init(void)
{
    ret_code_t err_code;

//    if (m_is_init)
//    {
//        err_code = NRF_ERROR_INVALID_STATE;
//    }
//    else
    {
        // initialize SPI
        err_code = spi_flash_init();
    }
    
    if (err_code == NRF_SUCCESS)
    {
        m_is_init = true;
    }

    return err_code;
}

ret_code_t spi_flash_block_uninit(void)
{
    ret_code_t err_code;
    
    if (m_is_init)
    {
        spi_flash_uninit();

        m_is_init = false;

        err_code = NRF_SUCCESS;
    }
    else
    {
        err_code = NRF_ERROR_INVALID_STATE;
    }
    
    return err_code;    
}

ret_code_t spi_flash_block_read_data(uint32_t addr, uint8_t *buff, uint32_t len)
{
    ret_code_t err_code = NRF_SUCCESS;

    if (!m_is_init)
    {
        err_code = NRF_ERROR_INVALID_STATE;
    }
    else if (addr >= FLASH_SIZE_BYTE || addr + len >= FLASH_SIZE_BYTE)
    {
        err_code = NRF_ERROR_INVALID_PARAM;
    }
    else
    {
        err_code = spi_flash_read_data(addr);

        if (err_code == NRF_SUCCESS)
        {
            err_code = spi_flash_data_get(buff, len);
        }
    }

    return err_code;
}
