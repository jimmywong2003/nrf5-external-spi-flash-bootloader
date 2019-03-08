///**
// * Copyright (c) 2016 - 2018, Nordic Semiconductor ASA
// *
// * All rights reserved.
// *
// * Redistribution and use in source and binary forms, with or without modification,
// * are permitted provided that the following conditions are met:
// *
// * 1. Redistributions of source code must retain the above copyright notice, this
// *    list of conditions and the following disclaimer.
// *
// * 2. Redistributions in binary form, except as embedded into a Nordic
// *    Semiconductor ASA integrated circuit in a product or a software update for
// *    such product, must reproduce the above copyright notice, this list of
// *    conditions and the following disclaimer in the documentation and/or other
// *    materials provided with the distribution.
// *
// * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
// *    contributors may be used to endorse or promote products derived from this
// *    software without specific prior written permission.
// *
// * 4. This software, with or without modification, must only be used with a
// *    Nordic Semiconductor ASA integrated circuit.
// *
// * 5. Any software provided in binary form under this license must not be reverse
// *    engineered, decompiled, modified and/or disassembled.
// *
// * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
// * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
// * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
// * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
// * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
// * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
// * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// *
// */
//
#include "boards.h"
#include "nrf_power.h"
#include "nrf_soc.h"
#include "nrf_clock.h"
#include "nrf_drv_clock.h"
//
//#include "nrf_bootloader_app_start.h"
//#include "nrf_bootloader_info.h"
//#include "nrf_dfu_settings.h"
//#include "nrf_dfu_utils.h"
//#include "nrf_dfu_transport.h"
//#include "nrf_dfu_req_handler.h"
//#include "app_scheduler.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
//
//#include "spi_flash_block.h"
//#include "spi_flash_misc.h"
//
#ifdef NRF_DFU_DEBUG_VERSION
#include "nrf_delay.h"
#endif //NRF_DFU_DEBUG_VERSION
//
//#define MAIN_APPLICATION_START_ADDR             (SD_SIZE_GET(MBR_SIZE))
//
//__ALIGN(4) static uint8_t m_spi_buff[CODE_PAGE_SIZE];
//
//#ifndef CODE_REGION_1_START
//#define CODE_REGION_1_START                 SD_SIZE_GET(MBR_SIZE)
//#endif
//
//uint32_t nrf_dfu_bank0_start_addr(void)
//{
//    if (SD_PRESENT)
//    {
//        return ALIGN_TO_PAGE(SD_SIZE_GET(MBR_SIZE));
//    }
//    else
//    {
//        return MBR_SIZE;
//    }
//}
//
//uint32_t nrf_dfu_app_start_address(void)
//{
//    return nrf_dfu_bank0_start_addr();
//}
//
//bool nrf_dfu_app_is_valid(bool do_crc)
//{
//    NRF_LOG_DEBUG("Enter nrf_dfu_app_is_valid");
//    if (s_dfu_settings.bank_0.bank_code != NRF_DFU_BANK_VALID_APP)
//    {
//       // Bank 0 has no valid app. Nothing to boot
//       NRF_LOG_DEBUG("No valid app to boot.");
//       return false;
//    }
//
//    // If CRC == 0, the CRC check is skipped.
//    if (do_crc && (s_dfu_settings.bank_0.image_crc != 0))
//    {
//        uint32_t crc = crc32_compute((uint8_t*) nrf_dfu_app_start_address(),
//                                     s_dfu_settings.bank_0.image_size,
//                                     NULL);
//
//        if (crc != s_dfu_settings.bank_0.image_crc)
//        {
//            // CRC does not match with what is stored.
//            NRF_LOG_DEBUG("CRC check of app failed. Return %d", NRF_DFU_DEBUG);
//            return NRF_DFU_DEBUG;
//        }
//    }
//
//    NRF_LOG_DEBUG("Return true. App was valid");
//    return true;
//}
//
//
///**
// * Round up val to the next page boundry
// */
//static uint32_t align_to_page(uint32_t val)
//{
//        return ((val + CODE_PAGE_SIZE - 1 ) &~(CODE_PAGE_SIZE - 1));
//}
//
//static uint32_t nrf_dfu_mbr_compare(uint32_t * p_ptr1, uint32_t * p_ptr2, uint32_t len)
//{
//        uint32_t ret_val;
//        uint32_t const len_words = len / sizeof(uint32_t);
//
//        sd_mbr_command_t command =
//        {
//                .command = SD_MBR_COMMAND_COMPARE,
//                .params.compare.ptr1 = p_ptr1,
//                .params.compare.ptr2 = p_ptr2,
//                .params.compare.len = len_words
//        };
//
//        ret_val = sd_mbr_command(&command);
//
//        return ret_val;
//}
//
//static void nrf_dfu_invalidate_bank(nrf_dfu_bank_t * p_bank)
//{
//        // Set the bank-code to invalid, and reset size/CRC
//        memset(p_bank, 0, sizeof(nrf_dfu_bank_t));
//
//        // Reset write pointer after completed operation
//        s_dfu_settings.write_offset = 0;
//
//        // Reset SD size
//        s_dfu_settings.sd_size = 0;
//
//        // Promote dual bank layout
//        s_dfu_settings.bank_layout = NRF_DFU_BANK_LAYOUT_DUAL;
//
//        // Signify that bank 0 is empty
//        s_dfu_settings.bank_current = NRF_DFU_CURRENT_BANK_0;
//}
//
///** @brief Function to continue application update.
// *
// * @details This function will be called after reset if there is a valid application in Bank1
// *          required to be copied down to Bank 0.
// *
// * @param[in]       src_addr            Source address of the application to copy from Bank1 to Bank0.
// *
// * @retval  NRF_SUCCESS                 Continuation was successful.
// * @retval  NRF_ERROR_NULL              Invalid data during compare.
// * @retval  FS_ERR_UNALIGNED_ADDR       A call to fstorage was not aligned to a page boundary or the address was not word aliged.
// * @retval  FS_ERR_INVALID_ADDR         The destination of a call to fstorage does not point to
// *                                      the start of a flash page or the operation would
// *                                      go beyond the flash memory boundary.
// * @retval  FS_ERR_NOT_INITIALIZED      The fstorage module is not initialized.
// * @retval  FS_ERR_INVALID_CFG          The initialization of the fstorage module is invalid.
// * @retval  FS_ERR_NULL_ARG             A call to fstorage had an invalid NULL argument.
// * @retval  FS_ERR_INVALID_ARG          A call to fstorage had invalid arguments.
// * @retval  FS_ERR_QUEUE_FULL           If the internal operation queue of the fstorage module is full.
// * @retval  FS_ERR_FAILURE_SINCE_LAST   If an error occurred in another transaction and fstorage cannot continue before
// *                                      the event has been dealt with.
// */
//static uint32_t nrf_dfu_app_continue(uint32_t src_addr)
//{
//        // This function is only in use when new app is present in Bank 1
//        uint32_t const image_size  = s_dfu_settings.bank_1.image_size;
//        uint32_t const split_size  = CODE_PAGE_SIZE;// Must be page aligned
//
//        uint32_t ret_val     = NRF_SUCCESS;
//        uint32_t target_addr = MAIN_APPLICATION_START_ADDR + s_dfu_settings.write_offset;
//        uint32_t length_left = (image_size - s_dfu_settings.write_offset);
//        uint32_t cur_len;
//        uint32_t crc;
//
//        ret_val = spi_flash_block_init();
//        if (ret_val != NRF_SUCCESS)
//        {
//                return ret_val;
//        }
//
//        // if GPREGRET has SPI_2 bit set, try using the 2nd DFU image instead
//        if (nrf_power_gpregret_get() & BOOTLOADER_DFU_SPI_2_BIT_MASK)
//        {
//                spi_flash_use_image_addr_2();
//        }
//
//        src_addr = spi_flash_get_fw_bin_addr();
//
//        NRF_LOG_DEBUG("Enter nrf_dfu_app_continue");
//
//        src_addr += s_dfu_settings.write_offset;
//
//        
//        // Copy the application down safely
//        do
//        {
//                cur_len = (length_left > split_size) ? split_size : length_left;
//
//                // Erase the target page
//                ret_val = nrf_dfu_flash_erase(target_addr, split_size / CODE_PAGE_SIZE, NULL);
//                if (ret_val != NRF_SUCCESS)
//                {
//                        return ret_val;
//                }
//
//                // Read SPI flash
//                ret_val = spi_flash_block_read_data(src_addr, m_spi_buff, ALIGN_NUM(sizeof(uint32_t), cur_len));
//                if (ret_val != NRF_SUCCESS)
//                {
//                        return ret_val;
//                }
//
//                // Flash one page
//                ret_val = nrf_dfu_flash_store(target_addr, (uint32_t*)m_spi_buff, cur_len, NULL);
//                if (ret_val != NRF_SUCCESS)
//                {
//                        return ret_val;
//                }
//
//                ret_val = nrf_dfu_mbr_compare((uint32_t*)target_addr, (uint32_t*)m_spi_buff, cur_len);
//                if (ret_val != NRF_SUCCESS)
//                {
//                        // We will not retry the copy
//                        NRF_LOG_ERROR("Invalid data during compare: target: 0x%08x, src: 0x%08x", target_addr, src_addr);
//                        return ret_val;
//                }
//
//                s_dfu_settings.write_offset += cur_len;
//                ret_val = nrf_dfu_settings_write(NULL);
//
//                target_addr += cur_len;
//                src_addr += cur_len;
//
//                length_left -= cur_len;
//        }
//        while (length_left > 0);
//
//        // Check the CRC of the copied data. Enable if so.
//        crc = crc32_compute((uint8_t*)MAIN_APPLICATION_START_ADDR, image_size, NULL);
//
//        if (crc == s_dfu_settings.bank_1.image_crc)
//        {
//                NRF_LOG_DEBUG("Setting app as valid");
//                s_dfu_settings.bank_0.bank_code = NRF_DFU_BANK_VALID_APP;
//                s_dfu_settings.bank_0.image_crc = crc;
//                s_dfu_settings.bank_0.image_size = image_size;
//        }
//        else
//        {
//                NRF_LOG_ERROR("CRC computation failed for copied app: "
//                              "src crc: 0x%08x, res crc: 0x%08x",
//                              s_dfu_settings.bank_1.image_crc,
//                              crc);
//        }
//
//        nrf_dfu_invalidate_bank(&s_dfu_settings.bank_1);
//        ret_val = nrf_dfu_settings_write(NULL);
//
//        return ret_val;
//}
//
//static uint32_t nrf_dfu_continue_bank(nrf_dfu_bank_t * p_bank, uint32_t src_addr, uint32_t * p_enter_dfu_mode)
//{
//        uint32_t ret_val = NRF_SUCCESS;
//
//        switch (p_bank->bank_code)
//        {
//        case NRF_DFU_BANK_VALID_APP:
//                NRF_LOG_DEBUG("Valid App");
//                if(s_dfu_settings.bank_current == NRF_DFU_CURRENT_BANK_1)
//                {
//                        // Only continue copying if valid app in Bank 1
//                        ret_val = nrf_dfu_app_continue(src_addr);
//                }
//                break;
//
//        case NRF_DFU_BANK_INVALID:
//        default:
//                NRF_LOG_ERROR("Single: Invalid bank");
//                break;
//        }
//
//        return ret_val;
//}
//
//uint32_t nrf_dfu_continue(uint32_t * p_enter_dfu_mode)
//{
//        uint32_t ret_val;
//        nrf_dfu_bank_t    * p_bank;
//        uint32_t src_addr = CODE_REGION_1_START;
//
//        NRF_LOG_DEBUG("Enter nrf_dfu_continue");
//
//        if (s_dfu_settings.bank_layout == NRF_DFU_BANK_LAYOUT_SINGLE )
//        {
//                p_bank = &s_dfu_settings.bank_0;
//        }
//        else if(s_dfu_settings.bank_current == NRF_DFU_CURRENT_BANK_0)
//        {
//                p_bank = &s_dfu_settings.bank_0;
//        }
//        else
//        {
//                p_bank = &s_dfu_settings.bank_1;
//                src_addr += align_to_page(s_dfu_settings.bank_0.image_size);
//        }
//
//        ret_val = nrf_dfu_continue_bank(p_bank, src_addr, p_enter_dfu_mode);
//        return ret_val;
//}
//
//uint32_t nrf_dfu_spi_init(void)
//{
//        uint32_t ret_val = NRF_SUCCESS;
//
//        NRF_LOG_DEBUG("In real nrf_dfu_init");
//
//        ret_val = nrf_dfu_spi_req_handler_init();
//        if (ret_val != NRF_SUCCESS)
//        {
//            NRF_LOG_ERROR("Could not initalize nrf_dfu_spi_req_handler_init: 0x%08x", ret_val);
//            return ret_val;
//        }
//
//        // Should not be reached!
//        NRF_LOG_INFO("After real nrf_dfu_spi_init");
//        return NRF_SUCCESS;
//}

bool nrf_dfu_app_is_valid(bool do_crc)
{
    NRF_LOG_DEBUG("Enter nrf_dfu_app_is_valid");
    if (s_dfu_settings.bank_0.bank_code != NRF_DFU_BANK_VALID_APP)
    {
       // Bank 0 has no valid app. Nothing to boot
       NRF_LOG_DEBUG("No valid app to boot.");
       return false;
    }

    // If CRC == 0, the CRC check is skipped.
    if (do_crc && (s_dfu_settings.bank_0.image_crc != 0))
    {
        uint32_t crc = crc32_compute((uint8_t*) nrf_dfu_app_start_address(),
                                     s_dfu_settings.bank_0.image_size,
                                     NULL);

        if (crc != s_dfu_settings.bank_0.image_crc)
        {
            // CRC does not match with what is stored.
            NRF_LOG_DEBUG("CRC check of app failed. Return %d", NRF_DFU_DEBUG);
            return NRF_DFU_DEBUG;
        }
    }

    NRF_LOG_DEBUG("Return true. App was valid");
    return true;
}




