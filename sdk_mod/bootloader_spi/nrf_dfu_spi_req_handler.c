/**
 * Copyright (c) 2016 - 2017, Nordic Semiconductor ASA
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


#include "nrf_dfu_spi_req_handler.h"

#include <stdint.h>
#include <stdbool.h>
//#include "nrf_dfu.h"
#include "nrf_dfu_types.h"
#include "nrf_dfu_spi_req_handler.h"

//#include "nrf_dfu_handling_error.h"

//#include "nrf_dfu_settings.h"
//#include "nrf_dfu_transport.h"
//#include "nrf_dfu_utils.h"
#include "nrf_dfu_types.h"
#include "nrf_dfu_flash.h"
#include "nrf_bootloader_info.h"
#include "nrf_delay.h"
#include "nrf_power.h"
#include "app_util.h"

#include "nrf_dfu_flash.h"
#include "nrf_fstorage.h"
//#include "nrf_fstorage_sd.h"
#include "nrf_fstorage_nvmc.h"
//#include "nrf_crypto.h"

#include "crc32.h"
#ifdef SOFTDEVICE_PRESENT
#include "nrf_sdm.h"
#endif
#include "sdk_macros.h"

#include "spi_flash_block.h"
#include "spi_flash_misc.h"
//#include "sha256.h"

#define NRF_LOG_MODULE_NAME dfu_spi_req_handling
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
NRF_LOG_MODULE_REGISTER();

#ifndef CODE_REGION_1_START
#define CODE_REGION_1_START                 SD_SIZE_GET(MBR_SIZE)
#endif

#define MAIN_APPLICATION_START_ADDR             (SD_SIZE_GET(MBR_SIZE))

/** @brief Total size of the region between the SoftDevice and the bootloader.
 */
#define DFU_REGION_TOTAL_SIZE               ((* (uint32_t *)NRF_UICR_BOOTLOADER_START_ADDRESS) - CODE_REGION_1_START)


typedef struct
{
        uint32_t length;                                /**< Init packet length. */
        uint8_t data[INIT_COMMAND_MAX_SIZE];            /**< Init packet body. */
} nrf_spi_flash_init_data;


/**@brief Generic type to hold pointer to value and length
 */
typedef struct
{
        uint8_t * p_value;
        uint32_t length;

} nrf_value_length_t;

/**
 * @brief DFU operation result code.
 */
typedef enum
{
    NRF_DFU_RES_CODE_INVALID                 = 0x00,    //!< Invalid opcode.
    NRF_DFU_RES_CODE_SUCCESS                 = 0x01,    //!< Operation successful.
    NRF_DFU_RES_CODE_OP_CODE_NOT_SUPPORTED   = 0x02,    //!< Opcode not supported.
    NRF_DFU_RES_CODE_INVALID_PARAMETER       = 0x03,    //!< Missing or invalid parameter value.
    NRF_DFU_RES_CODE_INSUFFICIENT_RESOURCES  = 0x04,    //!< Not enough memory for the data object.
    NRF_DFU_RES_CODE_INVALID_OBJECT          = 0x05,    //!< Data object does not match the firmware and hardware requirements, the signature is wrong, or parsing the command failed.
    NRF_DFU_RES_CODE_UNSUPPORTED_TYPE        = 0x07,    //!< Not a valid object type for a Create request.
    NRF_DFU_RES_CODE_OPERATION_NOT_PERMITTED = 0x08,    //!< The state of the DFU process does not allow this operation.
    NRF_DFU_RES_CODE_OPERATION_FAILED        = 0x0A,    //!< Operation failed.
    NRF_DFU_RES_CODE_EXT_ERROR               = 0x0B,    //!< Extended error. The next byte of the response contains the error code of the extended error (see @ref nrf_dfu_ext_error_code_t.
} nrf_dfu_result_t;

static nrf_spi_flash_init_data m_init_data;     /**< Init packet loaded from SPI flash. */

//__ALIGN(4) static uint8_t m_data_buf[FLASH_BUFFER_COUNT][FLASH_BUFFER_LENGTH];

#define DFU_SETTINGS_INIT_COMMAND_OFFSET        offsetof(nrf_dfu_settings_t, init_command)          //<! Offset in the settings struct where the InitCommand is located.

static uint16_t m_data_buf_pos;                 /**< The number of bytes written in the current buffer. */
static uint8_t m_current_data_buffer;           /**< Index of the current data buffer. Must be between 0 and FLASH_BUFFER_COUNT - 1. */

static uint32_t m_firmware_start_addr;          /**< Start address of the current firmware image. */
static uint32_t m_firmware_size_req;            /**< The size of the entire firmware image. Defined by the init command. */

extern nrf_dfu_settings_t s_dfu_settings;
static bool m_valid_init_packet_present;        /**< Global variable holding the current flags indicating the state of the DFU process. */

static uint32_t m_crc32;

static nrf_value_length_t init_packet_data = {0};

static spi_flash_init_cmd_t m_init_cmd;

extern uint8_t m_dfu_settings_buffer[CODE_PAGE_SIZE];

static void reset_device(void)
{
  #ifdef NRF_DFU_DEBUG_VERSION
        NRF_LOG_DEBUG("Reset.");
        NRF_LOG_FLUSH();
        nrf_delay_ms(100);
  #endif
        NVIC_SystemReset();
}







/** @brief Function for computing CRC-32 of DFU binary stored in SPI flash.
 */
static uint32_t spi_crc32_compute(uint32_t * p_crc,
                                  uint32_t addr,
                                  uint32_t len)
{
        uint32_t ret_val = NRF_SUCCESS;
        uint8_t m_spi_buff[255 * 2];
        uint32_t pos;
        uint32_t stp;

        *p_crc = 0;

        for (pos = 0; pos < len && ret_val == NRF_SUCCESS; pos += stp)
        {
                stp = MIN(len - pos, sizeof(m_spi_buff));

                ret_val = spi_flash_block_read_data(addr + pos, m_spi_buff, stp);
                if (ret_val != NRF_SUCCESS)
                        break;

                *p_crc = crc32_compute(m_spi_buff, stp, p_crc);
        }

        return ret_val;
}

static void on_dfu_spi_complete(nrf_fstorage_evt_t * p_evt)
{
        NRF_LOG_DEBUG("All flash operations have completed.");
        reset_device();
}


static uint32_t nrf_dfu_settings_crc_get(void)
{
    // The crc is calculated from the s_dfu_settings struct, except the crc itself and the init command
    return crc32_compute((uint8_t*)&s_dfu_settings + 4, DFU_SETTINGS_INIT_COMMAND_OFFSET  - 4, NULL);
}

void nrf_dfu_spi_settings_init(bool sd_irq_initialized)
{
    NRF_LOG_DEBUG("Running nrf_dfu_settings_init(sd_irq_initialized=%s).",
                  sd_irq_initialized ? (uint32_t)"true" : (uint32_t)"false");

    ret_code_t rc = nrf_dfu_flash_init(sd_irq_initialized);
    if (rc != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("nrf_dfu_flash_init() failed with error: %x", rc);
        APP_ERROR_HANDLER(rc);
    }

    // Copy the DFU settings out of flash and into a buffer in RAM.
    memcpy((void*)&s_dfu_settings, m_dfu_settings_buffer, sizeof(nrf_dfu_settings_t));

    if (s_dfu_settings.crc != 0xFFFFFFFF)
    {
        // CRC is set. Content must be valid
        uint32_t crc = nrf_dfu_settings_crc_get();
        if (crc == s_dfu_settings.crc)
        {
            return;
        }
    }

    // Reached if the page is erased or CRC is wrong.
    NRF_LOG_DEBUG("Resetting bootloader settings.");

    memset(&s_dfu_settings, 0x00, sizeof(nrf_dfu_settings_t));
    s_dfu_settings.settings_version = NRF_DFU_SETTINGS_VERSION;

    rc = nrf_dfu_settings_write(NULL);
    if (rc != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("nrf_dfu_flash_write() failed with error: %x", rc);
        APP_ERROR_HANDLER(rc);
    }
}

static uint32_t nrf_dfu_find_cache(uint32_t size_req, bool dual_bank_only, uint32_t * p_address)
{
    uint32_t free_size =  DFU_REGION_TOTAL_SIZE - DFU_APP_DATA_RESERVED;
    nrf_dfu_bank_t * p_bank;

    NRF_LOG_DEBUG("Enter nrf_dfu_find_cache");

    if (p_address == NULL)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    // Simple check whether the size requirement can be met
    if(free_size < size_req)
    {
        NRF_LOG_DEBUG("No way to fit the new firmware on device");
        return NRF_ERROR_NO_MEM;
    }

    NRF_LOG_DEBUG("Bank content");
    NRF_LOG_DEBUG("Bank type: %d", s_dfu_settings.bank_layout);
    NRF_LOG_DEBUG("Bank 0 code: 0x%02x: Size: %d", s_dfu_settings.bank_0.bank_code, s_dfu_settings.bank_0.image_size);
    NRF_LOG_DEBUG("Bank 1 code: 0x%02x: Size: %d", s_dfu_settings.bank_1.bank_code, s_dfu_settings.bank_1.image_size);

    // Using SPI flash, no code flash is used as buffer.
    (*p_address) = MAIN_APPLICATION_START_ADDR;
    s_dfu_settings.bank_layout = NRF_DFU_BANK_LAYOUT_DUAL;
    s_dfu_settings.bank_current = NRF_DFU_CURRENT_BANK_1;
    p_bank = &s_dfu_settings.bank_1;
    NRF_LOG_DEBUG("Using bank 1");

    // Set the bank-code to invalid, and reset size/CRC
    memset(p_bank, 0, sizeof(nrf_dfu_bank_t));

    // Store the Firmware size in the bank for continuations
    p_bank->image_size = size_req;
    return NRF_SUCCESS;
}

static nrf_dfu_result_t dfu_handle_prevalidate(spi_flash_init_cmd_t const * p_init, uint8_t * p_init_cmd, uint32_t init_cmd_len)
{
        uint32_t err_code;
        uint32_t fw_version = 0;
        //uint32_t m_firmware_start_addr;          /**< Start address of the current firmware image. */
       // uint32_t m_firmware_size_req;
        // check for init command found during decoding
        if (!p_init_cmd || !init_cmd_len)
        {
                //return ext_error_set(NRF_DFU_EXT_ERROR_INIT_COMMAND_INVALID);
                return NRF_DFU_RES_CODE_EXT_ERROR;
        }

        // Get the application FW version
        fw_version = s_dfu_settings.app_version;

        NRF_LOG_INFO("Req version: %d, Expected: %d", p_init->fw_version, fw_version);

// Check of init command FW version
        if (p_init->fw_version < fw_version)
        {
                NRF_LOG_ERROR("FW version too low");
                //return ext_error_set(NRF_DFU_EXT_ERROR_FW_VERSION_FAILURE);
                return NRF_DFU_RES_CODE_EXT_ERROR;
        }


        // Get the update size
        m_firmware_size_req = p_init->app_size;

        // Instead of checking each type with has-check, check the result of the size_req to
// Validate its content.
// Instead of checking each type with has-check, check the result of the size_req to
// Validate its content.
        if (m_firmware_size_req == 0)
        {
                NRF_LOG_ERROR("No FW size");
                return NRF_DFU_RES_CODE_INVALID_PARAMETER;
        }

// Find the location to place the DFU updates
        err_code = nrf_dfu_find_cache(m_firmware_size_req, false, &m_firmware_start_addr);
        if (err_code != NRF_SUCCESS)
        {
                NRF_LOG_ERROR("Can't find room for update");
                return NRF_DFU_RES_CODE_INSUFFICIENT_RESOURCES;
        }

        NRF_LOG_INFO("Write address set to 0x%08x", m_firmware_start_addr);

        NRF_LOG_INFO("DFU prevalidate SUCCESSFUL!");

        return NRF_DFU_RES_CODE_SUCCESS;
}


static uint32_t dfu_decode_commmand(void)
{

    if (s_dfu_settings.progress.command_size != sizeof(m_init_cmd))
    {
        NRF_LOG_ERROR("Init command size invalid");
        return 0;
    }

    spi_flash_init_cmd_t const *p_init = (spi_flash_init_cmd_t const *) s_dfu_settings.init_command;
    m_init_cmd = *p_init;

    if (spi_flash_verify_init(&m_init_cmd) != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("Init command CRC failed");
        return 0;
    }

    init_packet_data.p_value = s_dfu_settings.init_command;
    init_packet_data.length = s_dfu_settings.progress.command_size;

    return 1;
}

static nrf_dfu_result_t dfu_handle_command(spi_flash_init_cmd_t const * p_command)
{
        nrf_dfu_result_t ret_val = NRF_DFU_RES_CODE_SUCCESS;

        ret_val = dfu_handle_prevalidate(p_command, init_packet_data.p_value, init_packet_data.length);
        if (ret_val == NRF_DFU_RES_CODE_SUCCESS)
        {
                NRF_LOG_INFO("Prevalidate OK.");

                // This saves the init command to flash
                NRF_LOG_INFO("Saving init command...");
                if (nrf_dfu_settings_write(NULL) != NRF_SUCCESS)
                {
                        return NRF_DFU_RES_CODE_OPERATION_FAILED;
                }
        }
        else
        {
                NRF_LOG_ERROR("Prevalidate failed!");
        }
        return ret_val;
}

static nrf_dfu_result_t nrf_dfu_postvalidate(spi_flash_init_cmd_t const * p_init)
{
        uint32_t err_code;
        nrf_dfu_result_t res_code = NRF_DFU_RES_CODE_SUCCESS;
        nrf_dfu_bank_t           * p_bank;
//        uint32_t m_firmware_size_req;

        // Verify CRC
        err_code = spi_crc32_compute(&m_crc32, spi_flash_get_fw_bin_addr(), m_firmware_size_req);
        if (err_code != NRF_SUCCESS)
        {
                res_code = NRF_DFU_RES_CODE_OPERATION_FAILED;
        }
        else if (p_init->app_crc != m_crc32)
        {
                NRF_LOG_ERROR("Hash failure");
                //res_code = ext_error_set(NRF_DFU_EXT_ERROR_VERIFICATION_FAILED);
        }
        else
        {
                // Set firmware image CRC
                s_dfu_settings.progress.firmware_image_crc = m_crc32;
        }

        if (s_dfu_settings.bank_current == NRF_DFU_CURRENT_BANK_0)
        {
                NRF_LOG_INFO("Current bank is bank 0");
                p_bank = &s_dfu_settings.bank_0;
        }
        else if (s_dfu_settings.bank_current == NRF_DFU_CURRENT_BANK_1)
        {
                NRF_LOG_INFO("Current bank is bank 1");
                p_bank = &s_dfu_settings.bank_1;
        }
        else
        {
                NRF_LOG_ERROR("Internal error, invalid current bank");
                return NRF_DFU_RES_CODE_OPERATION_FAILED;
        }

        // Finalize postvalidation by updating DFU settings.
        if (res_code == NRF_DFU_RES_CODE_SUCCESS)
        {
                NRF_LOG_INFO("Successfully ran the postvalidation check!");

                p_bank->bank_code = NRF_DFU_BANK_VALID_APP;
                // If this update is in bank 1, invalidate bank 0 before starting copy routine.
                if (s_dfu_settings.bank_current == NRF_DFU_CURRENT_BANK_1)
                {
                        NRF_LOG_INFO("Invalidating old application in bank 0.");
                        s_dfu_settings.bank_0.bank_code = NRF_DFU_BANK_INVALID;
                }


                s_dfu_settings.app_version = p_init->fw_version;

                // Calculate CRC32 for image
                p_bank->image_crc = s_dfu_settings.progress.firmware_image_crc;
                p_bank->image_size = m_firmware_size_req;
        }
        else
        {
                p_bank->bank_code = NRF_DFU_BANK_INVALID;

                // Calculate CRC32 for image
                p_bank->image_crc = 0;
                p_bank->image_size = 0;
        }

        // Set the progress to zero and remove the last command
        memset(&s_dfu_settings.progress, 0, sizeof(dfu_progress_t));
        memset(s_dfu_settings.init_command, 0xFF, INIT_COMMAND_MAX_SIZE);
        s_dfu_settings.write_offset = 0;

        // Store the settings to flash and reset after that
        if (nrf_dfu_settings_write(on_dfu_spi_complete) != NRF_SUCCESS)
        {
                res_code = NRF_DFU_RES_CODE_OPERATION_FAILED;
        }
//        if (nrf_dfu_settings_write(NULL) != NRF_SUCCESS)

        return res_code;

}

/** @brief Function for processing init packet.
 */
static nrf_dfu_result_t nrf_dfu_init_cmd_proc(void)
{
        nrf_dfu_result_t ret_val = NRF_DFU_RES_CODE_SUCCESS;

        if (dfu_decode_commmand() != true)
        {
                return NRF_DFU_RES_CODE_INVALID_OBJECT;
        }

        // We have a valid DFU packet
        {
                NRF_LOG_INFO("Handling unsigned command");

                ret_val = dfu_handle_command(&m_init_cmd);
        }

        if (ret_val == NRF_DFU_RES_CODE_SUCCESS)
        {
                // Setting DFU to initialized
                NRF_LOG_INFO("Setting DFU flag to initialized");
                m_valid_init_packet_present = true;
        }

        return ret_val;
}


/** @brief Function for loading DFU init packet from SPI flash.
 */
static ret_code_t nrf_dfu_init_cmd_load(void)
{
        ret_code_t err_code;

        // Reset all progress.
        s_dfu_settings.write_offset = 0;
        memset(&s_dfu_settings.progress, 0x00, sizeof(dfu_progress_t));

        uint8_t spi_2_bit = 0x00;

        // if we do not have a valid application in place,
        // try the 2nd DFU image in SPI flash
        if (!nrf_dfu_app_is_valid(true))
        {
                spi_flash_use_image_addr_2();

                spi_2_bit |= BOOTLOADER_DFU_SPI_2_BIT_MASK;
        }

        // Write SPI_2 bit in GPREGRET register.

        nrf_power_gpregret_set(nrf_power_gpregret_get() | spi_2_bit);


        // read SPI flash
        uint32_t dfu_dat_addr = spi_flash_get_fw_dat_addr();
        err_code = spi_flash_block_read_data(dfu_dat_addr, (uint8_t *)&m_init_data, sizeof(m_init_data));

        if (err_code == NRF_SUCCESS)
        {
                if (m_init_data.length == 0 || m_init_data.length > INIT_COMMAND_MAX_SIZE)
                {
                        NRF_LOG_ERROR("SPI flash init packet size is invalid");

                        err_code = NRF_ERROR_DATA_SIZE;
                }
                else
                {
                        // Set DFU settings data
                        memcpy(s_dfu_settings.init_command, m_init_data.data, m_init_data.length);
                        s_dfu_settings.progress.command_offset     = m_init_data.length;
                        s_dfu_settings.progress.command_size       = m_init_data.length;
                }
        }
        return err_code;
}


uint32_t nrf_dfu_spi_req_handler_init(void)
{
        ret_code_t ret_val;
        nrf_dfu_result_t result;

        NRF_LOG_DEBUG("Inside nrf_dfu_req_handler_init");

        ret_val = nrf_dfu_flash_init(false);
        VERIFY_SUCCESS(ret_val);

        ret_val = spi_flash_block_init();

        if (ret_val == NRF_SUCCESS)
                ret_val = nrf_dfu_init_cmd_load();

        if (ret_val == NRF_SUCCESS)
        {
                nrf_dfu_init_cmd_proc();

                if (m_valid_init_packet_present)
                {
                        nrf_dfu_result_t res_code;

                        NRF_LOG_INFO("Doing postvalidate");
                        res_code = nrf_dfu_postvalidate(&m_init_cmd);

                        if (res_code != NRF_DFU_RES_CODE_SUCCESS)
                                ret_val = NRF_ERROR_INVALID_DATA;
                }
                else
                        ret_val = NRF_ERROR_INVALID_DATA;
        }

        spi_flash_block_uninit();

        /* Initialize extended error handling with "No error" as the most recent error. */
//        result = ext_error_set(NRF_DFU_EXT_ERROR_NO_ERROR);
//        UNUSED_RETURN_VALUE(result);

        if (ret_val != NRF_SUCCESS)
        {
                reset_device();
        }
}
