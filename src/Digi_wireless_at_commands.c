/*
 * Copyright (c) 2025 IED
 *
 */

/** @file
 *
 * @brief Managment of AT commands received through Zigbee.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <zboss_api.h>

#include "Digi_profile.h"
#include "zigbee_aps.h"
#include "Digi_wireless_at_commands.h"

LOG_MODULE_REGISTER(Digi_wireless_at_commands, LOG_LEVEL_DBG);

static bool b_pending_ping_cmd;             // A ping command has been received and is pending to be replied
static uint8_t ping_first_char;             // First char of ping command
static uint8_t ping_second_char;            // Second char of ping command
static bool b_pending_read_cmd;             // A read AT command has been received and is pending to be replied
static enum wireless_at_read_cmd_e read_cmd;       // Enumerative of read AT command
static uint8_t read_cmd_first_char;                // First char of read AT command
static uint8_t read_cmd_second_char;               // Second char of read AT command
static uint8_t read_cmd_sequence_number;           // Read AT command sequence number
static uint8_t read_cmd_reply_size;                // Number of bytes of reply
static uint8_t read_cmd_reply[MAX_SIZE_AT_COMMAND_REPLY]; // Buffer containing the reply

/**@brief This function initializes the Digi_wireless_at_commands firmware module
 *
 */
void digi_wireless_at_init(void)
{
    b_pending_ping_cmd = false;
    b_pending_read_cmd = false;
    read_cmd = NO_SUPPORTED_EXT_READ_AT_CMD;
}

/**@brief This function evaluates if the last received APS frame is a Digi's ping command
 *
 * @param[in]   input_data   Pointer to payload of received APS frame
 * @param[in]   size_of_input_data   Payload size
 *
 * @retval True It is a ping command
 * @retval False Otherwise
 */
bool is_a_ping_command(uint8_t* input_data, int16_t size_of_input_data)
{
    bool b_return = false;
    if (size_of_input_data == 2) // We got this value with the sniffer and reverse engineering
    {
        b_return = true;
        b_pending_ping_cmd = true;
        ping_first_char = input_data[0];
        ping_second_char = input_data[1];
    }
    return b_return;
}

/**@brief This function evaluates if the last received APS frame is a Digi's read AT command
 *
 * @param[in]   input_data   Pointer to payload of received APS frame
 * @param[in]   size_of_input_data   Payload size
 *
 * @retval True It is a read AT command
 * @retval False Otherwise
 */
bool is_a_digi_read_at_command(uint8_t* input_data, int16_t size_of_input_data)
{
    bool b_return = false;
    enum wireless_at_read_cmd_e received_cmd = NO_SUPPORTED_EXT_READ_AT_CMD;
    uint8_t i, j;
    uint16_t uitemp;
    uint64_t ultemp;
    if (size_of_input_data == 16) // We got this value with the sniffer and reverse engineering
    {
        if ((input_data[1] == 0) && (input_data[2] == 2) && (input_data[12] == 0) && (input_data[13] == 0))
        // We got the above values with the sniffer and reverse engineering
        {
            if (input_data[14] == 'A')
            {
                if (input_data[15] == 'I')
                {
                    received_cmd = EXT_READ_AT_AI;
                    read_cmd_reply_size = 1;
                    read_cmd_reply[0] = 0;
                }
                else if (input_data[15] == 'R')
                {
                    received_cmd = EXT_READ_AT_AR;
                    read_cmd_reply_size = 1;
                    read_cmd_reply[0] = 255;
                }
            }
            else if (input_data[14] == 'B')
            {
                if (input_data[15] == 'D')
                {
                    received_cmd = EXT_READ_AT_BD;
                    read_cmd_reply_size = 1;
                    read_cmd_reply[0] = 4;
                }
                else if (input_data[15] == 'H')
                {
                    received_cmd = EXT_READ_AT_BH;
                    read_cmd_reply_size = 1;
                    read_cmd_reply[0] = 0;
                }
            }
            else if (input_data[14] == 'C')
            {
                if (input_data[15] == 'C')
                {
                    received_cmd = EXT_READ_AT_CC;
                    read_cmd_reply_size = 1;
                    read_cmd_reply[0] = '+';
                }
                else if (input_data[15] == 'E')
                {
                    received_cmd = EXT_READ_AT_CE;
                    read_cmd_reply_size = 1;
                    read_cmd_reply[0] = 0;
                }
                else if (input_data[15] == 'H')
                {
                    received_cmd = EXT_READ_AT_CH;
                    read_cmd_reply_size = 1;
                    read_cmd_reply[0] = (uint8_t)zb_get_current_channel();
                }
                else if (input_data[15] == 'I')
                {
                    received_cmd = EXT_READ_AT_CI;
                    read_cmd_reply_size = 2;
                    read_cmd_reply[0] = 0;
                    read_cmd_reply[1] = 0x11;                    
                }
                else if (input_data[15] == 'R')
                {
                    received_cmd = EXT_READ_AT_CR;
                    read_cmd_reply_size = 1;
                    read_cmd_reply[0] = 3;
                }
                else if (input_data[15] == 'T')
                {
                    received_cmd = EXT_READ_AT_CT;
                    read_cmd_reply_size = 2;
                    read_cmd_reply[0] = 0;
                    read_cmd_reply[1] = 0x64; 
                }                
            }
            else if (input_data[14] == 'D')
            {
                if (input_data[15] == '0')
                {
                    received_cmd = EXT_READ_AT_D0;
                    read_cmd_reply_size = 1;
                    read_cmd_reply[0] = 1;
                }
                else if (input_data[15] == '1')
                {
                    received_cmd = EXT_READ_AT_D1;
                    read_cmd_reply_size = 1;
                    read_cmd_reply[0] = 0;
                }
                else if (input_data[15] == '2')
                {
                    received_cmd = EXT_READ_AT_D2;
                    read_cmd_reply_size = 1;
                    read_cmd_reply[0] = 0;
                }
                else if (input_data[15] == '3')
                {
                    received_cmd = EXT_READ_AT_D3;
                    read_cmd_reply_size = 1;
                    read_cmd_reply[0] = 0;
                }
                else if (input_data[15] == '4')
                {
                    received_cmd = EXT_READ_AT_D4;
                    read_cmd_reply_size = 1;
                    read_cmd_reply[0] = 0;
                }
                else if (input_data[15] == '5')
                {
                    received_cmd = EXT_READ_AT_D5;
                    read_cmd_reply_size = 1;
                    read_cmd_reply[0] = 1;
                }
                else if (input_data[15] == '6')
                {
                    received_cmd = EXT_READ_AT_D6;
                    read_cmd_reply_size = 1;
                    read_cmd_reply[0] = 0;
                }
                else if (input_data[15] == '7')
                {
                    received_cmd = EXT_READ_AT_D7;
                    read_cmd_reply_size = 1;
                    read_cmd_reply[0] = 1;
                }
                else if (input_data[15] == '8')
                {
                    received_cmd = EXT_READ_AT_D8;
                    read_cmd_reply_size = 1;
                    read_cmd_reply[0] = 1;
                }
                else if (input_data[15] == '9')
                {
                    received_cmd = EXT_READ_AT_D9;
                    read_cmd_reply_size = 1;
                    read_cmd_reply[0] = 1;
                }
                else if (input_data[15] == 'B')
                {
                    received_cmd = EXT_READ_AT_DB;
                    read_cmd_reply_size = 1;
                    read_cmd_reply[0] = 50;
                } 
                else if (input_data[15] == 'D')
                {
                    received_cmd = EXT_READ_AT_DD;  //TODO
                    read_cmd_reply_size = 4;
                    read_cmd_reply[0] = 0;
                    read_cmd_reply[1] = 0;
                    read_cmd_reply[2] = 0;
                    read_cmd_reply[3] = 1;
                }
                else if (input_data[15] == 'E')
                {
                    received_cmd = EXT_READ_AT_DE;
                    read_cmd_reply_size = 1;
                    read_cmd_reply[0] = 0xE8;
                }
                else if (input_data[15] == 'H')
                {
                    received_cmd = EXT_READ_AT_DH;
                    read_cmd_reply_size = 4;
                    read_cmd_reply[0] = 0;
                    read_cmd_reply[1] = 0;
                    read_cmd_reply[2] = 0;
                    read_cmd_reply[3] = 0;
                }
                else if (input_data[15] == 'L')
                {
                    received_cmd = EXT_READ_AT_DL;
                    read_cmd_reply_size = 4;
                    read_cmd_reply[0] = 0;
                    read_cmd_reply[1] = 0;
                    read_cmd_reply[2] = 0;
                    read_cmd_reply[3] = 0;
                }                
            }
            else if (input_data[14] == 'E')
            {
                if (input_data[15] == 'A')
                {
                    received_cmd = EXT_READ_AT_EA;  //TODO
                    read_cmd_reply_size = 2;
                    read_cmd_reply[0] = 0;
                    read_cmd_reply[1] = 1;
                }
                else if (input_data[15] == 'E')
                {
                    received_cmd = EXT_READ_AT_EE;
                    read_cmd_reply_size = 1;
                    read_cmd_reply[0] = 1;
                }
                else if (input_data[15] == 'O')
                {
                    received_cmd = EXT_READ_AT_EO;
                    read_cmd_reply_size = 1;
                    read_cmd_reply[0] = 0;
                }
            }
            else if (input_data[14] == 'G')
            {
                if (input_data[15] == 'T')
                {
                    received_cmd = EXT_READ_AT_GT;
                    read_cmd_reply_size = 2;
                    read_cmd_reply[0] = 0x03;
                    read_cmd_reply[1] = 0xE8;
                }
            }
            else if (input_data[14] == 'H')
            {
                if (input_data[15] == 'V')
                {
                    received_cmd = EXT_READ_AT_HV;  //TODO
                    read_cmd_reply_size = 2;
                    read_cmd_reply[0] = 0x52;
                    read_cmd_reply[1] = 0x42;
                }
            }
            else if (input_data[14] == 'I')
            {
                if (input_data[15] == 'C')
                {
                    received_cmd = EXT_READ_AT_IC;
                    read_cmd_reply_size = 2;
                    read_cmd_reply[0] = 0;
                    read_cmd_reply[1] = 0;
                }
                else if (input_data[15] == 'D')
                {
                    received_cmd = EXT_READ_AT_ID;
                    read_cmd_reply_size = 8;
                    ultemp = digi_at_get_parameter_id();
                    read_cmd_reply[0] = (uint8_t)((ultemp >> 56) & 0x00000000000000FF);
                    read_cmd_reply[1] = (uint8_t)((ultemp >> 48) & 0x00000000000000FF);
                    read_cmd_reply[2] = (uint8_t)((ultemp >> 40) & 0x00000000000000FF);
                    read_cmd_reply[3] = (uint8_t)((ultemp >> 32) & 0x00000000000000FF);
                    read_cmd_reply[4] = (uint8_t)((ultemp >> 24) & 0x00000000000000FF);
                    read_cmd_reply[5] = (uint8_t)((ultemp >> 16) & 0x00000000000000FF);
                    read_cmd_reply[6] = (uint8_t)((ultemp >> 8)  & 0x00000000000000FF);
                    read_cmd_reply[7] = (uint8_t)(ultemp & 0x00000000000000FF);
                }
                else if (input_data[15] == 'I')
                {
                    received_cmd = EXT_READ_AT_II;
                    read_cmd_reply_size = 2;
                    read_cmd_reply[0] = 0xFF;
                    read_cmd_reply[1] = 0xFF;
                }
                else if (input_data[15] == 'R')
                {
                    received_cmd = EXT_READ_AT_IR;
                    read_cmd_reply_size = 2;
                    read_cmd_reply[0] = 0;
                    read_cmd_reply[1] = 0;
                }
            }                
            else if (input_data[14] == 'J')
            {
                if (input_data[15] == 'N')
                {
                    received_cmd = EXT_READ_AT_JN;
                    read_cmd_reply_size = 1;
                    read_cmd_reply[0] = 0;
                }
                else if (input_data[15] == 'V')
                {
                    received_cmd = EXT_READ_AT_JV;
                    read_cmd_reply_size = 1;
                    read_cmd_reply[0] = 1;
                }
            } 
            else if (input_data[14] == 'K')
            {
                if (input_data[15] == 'Y')
                {
                    received_cmd = EXT_READ_AT_KY;
                    read_cmd_reply_size = 1;
                    read_cmd_reply[0] = 0;
                }
            }
            else if (input_data[14] == 'L')
            {
                if (input_data[15] == 'T')
                {
                    received_cmd = EXT_READ_AT_LT;
                    read_cmd_reply_size = 1;
                    read_cmd_reply[0] = 0;
                }
            }            
            else if (input_data[14] == 'M')
            {
                if (input_data[15] == 'P')
                {
                    received_cmd = EXT_READ_AT_MP;
                    read_cmd_reply_size = 2;
                    read_cmd_reply[0] = 0xFF;
                    read_cmd_reply[1] = 0xFE;
                }
                else if (input_data[15] == 'Y')
                {
                    received_cmd = EXT_READ_AT_MY;
                    read_cmd_reply_size = 2;
                    uitemp = (uint16_t)zb_get_short_address();
                    read_cmd_reply[0] = (uint8_t)(uitemp >> 8);
                    read_cmd_reply[1] = (uint8_t)uitemp;
                }
            }
            else if (input_data[14] == 'N')
            {
                if (input_data[15] == 'B')
                {
                    received_cmd = EXT_READ_AT_NB;
                    read_cmd_reply_size = 1;
                    read_cmd_reply[0] = 0;
                }
                else if (input_data[15] == 'C')
                {
                    received_cmd = EXT_READ_AT_NC;
                    read_cmd_reply_size = 1;
                    read_cmd_reply[0] = 20;
                }
                else if (input_data[15] == 'H')
                {
                    received_cmd = EXT_READ_AT_NH;
                    read_cmd_reply_size = 1;
                    read_cmd_reply[0] = 30;
                }
                else if (input_data[15] == 'I')
                {
                    received_cmd = EXT_READ_AT_NI; 
                    read_cmd_reply_size = zb_conf_get_extended_node_identifier(&read_cmd_reply[0]);
                    if (read_cmd_reply_size == 0)
                    {
                        read_cmd_reply_size = 1;
                        read_cmd_reply[0] = ' ';
                    }
                }
                else if (input_data[15] == 'J')
                {
                    received_cmd = EXT_READ_AT_NJ;
                    read_cmd_reply_size = 1;
                    read_cmd_reply[0] = 255;
                }
                else if (input_data[15] == 'K')
                {
                    received_cmd = EXT_READ_AT_NK;
                    read_cmd_reply_size = 16;
                    for (i = 0; i < 16; i++) read_cmd_reply[i] = 0;
                }
                else if (input_data[15] == 'P')
                {
                    received_cmd = EXT_READ_AT_NP;
                    read_cmd_reply_size = 1;
                    read_cmd_reply[0] = 255;
                }
                else if (input_data[15] == 'T')
                {
                    received_cmd = EXT_READ_AT_NT;
                    read_cmd_reply_size = 1;
                    read_cmd_reply[0] = 60;
                }
                else if (input_data[15] == 'W')
                {
                    received_cmd = EXT_READ_AT_NW;
                    read_cmd_reply_size = 2;
                    read_cmd_reply[0] = 0;
                    read_cmd_reply[1] = 10;
                }
            }
            else if (input_data[14] == 'O')
            {
                if (input_data[15] == 'I')
                {
                    received_cmd = EXT_READ_AT_OI;  // TODO
                    read_cmd_reply_size = 2;
                    read_cmd_reply[0] = 0x00;
                    read_cmd_reply[1] = 0x01;
                }
                else if (input_data[15] == 'P')
                {
                    received_cmd = EXT_READ_AT_OP;  // TODO
                    read_cmd_reply_size = 8;
                    read_cmd_reply[0] = 0x00;
                    read_cmd_reply[1] = 0x00;
                    read_cmd_reply[2] = 0x00;
                    read_cmd_reply[3] = 0x00;
                    read_cmd_reply[4] = 0x00;
                    read_cmd_reply[5] = 0x00;
                    read_cmd_reply[6] = 0x00;
                    read_cmd_reply[7] = 0x01;
                }
            }
            else if (input_data[14] == 'P')
            {
                if (input_data[15] == '2')
                {
                    received_cmd = EXT_READ_AT_P2;
                    read_cmd_reply_size = 1;
                    read_cmd_reply[0] = 0;
                }
                else if (input_data[15] == '3')
                {
                    received_cmd = EXT_READ_AT_P3;
                    read_cmd_reply_size = 1;
                    read_cmd_reply[0] = 1;
                }
                else if (input_data[15] == '4')
                {
                    received_cmd = EXT_READ_AT_P4;
                    read_cmd_reply_size = 1;
                    read_cmd_reply[0] = 1;
                }
                else if (input_data[15] == '5')
                {
                    received_cmd = EXT_READ_AT_P5;
                    read_cmd_reply_size = 1;
                    read_cmd_reply[0] = 1;
                }
                else if (input_data[15] == '6')
                {
                    received_cmd = EXT_READ_AT_P6;
                    read_cmd_reply_size = 1;
                    read_cmd_reply[0] = 1;
                }
                else if (input_data[15] == '7')
                {
                    received_cmd = EXT_READ_AT_P7;
                    read_cmd_reply_size = 1;
                    read_cmd_reply[0] = 1;
                }
                else if (input_data[15] == '8')
                {
                    received_cmd = EXT_READ_AT_P8;
                    read_cmd_reply_size = 1;
                    read_cmd_reply[0] = 1;
                }
                else if (input_data[15] == '9')
                {
                    received_cmd = EXT_READ_AT_P9;
                    read_cmd_reply_size = 1;
                    read_cmd_reply[0] = 1;
                }
                else if (input_data[15] == 'D')
                {
                    received_cmd = EXT_READ_AT_PD;
                    read_cmd_reply_size = 2;
                    read_cmd_reply[0] = 0x00;
                    read_cmd_reply[1] = 0x00;
                    read_cmd_reply[2] = 0xE7;
                    read_cmd_reply[3] = 0xFF;
                }
                else if (input_data[15] == 'L')
                {
                    received_cmd = EXT_READ_AT_PL;
                    read_cmd_reply_size = 1;
                    read_cmd_reply[0] = 4;
                }
                else if (input_data[15] == 'O')
                {
                    received_cmd = EXT_READ_AT_PO;
                    read_cmd_reply_size = 1;
                    read_cmd_reply[0] = 0;
                }
                else if (input_data[15] == 'P')
                {
                    received_cmd = EXT_READ_AT_PP;
                    read_cmd_reply_size = 1;
                    read_cmd_reply[0] = 8;
                }
                else if (input_data[15] == 'R')
                {
                    received_cmd = EXT_READ_AT_PR;
                    read_cmd_reply_size = 4;
                    read_cmd_reply[0] = 0x00;
                    read_cmd_reply[1] = 0x00;
                    read_cmd_reply[2] = 0xE7;
                    read_cmd_reply[3] = 0xFF;
                }
            }
            else if (input_data[14] == 'R')
            {
                if (input_data[15] == 'O')
                {
                    received_cmd = EXT_READ_AT_RO;
                    read_cmd_reply_size = 1;
                    read_cmd_reply[0] = 3;
                }
            }
            else if (input_data[14] == 'S')
            {
                if (input_data[15] == 'B')
                {
                    received_cmd = EXT_READ_AT_SB;
                    read_cmd_reply_size = 1;
                    read_cmd_reply[0] = 0;
                }
                else if (input_data[15] == 'C')
                {
                    received_cmd = EXT_READ_AT_SC;
                    read_cmd_reply_size = 2;
                    read_cmd_reply[0] = 0x07;
                    read_cmd_reply[1] = 0xFF;
                }
                else if (input_data[15] == 'D')
                {
                    received_cmd = EXT_READ_AT_SD;
                    read_cmd_reply_size = 1;
                    read_cmd_reply[0] = 3;
                }
                else if (input_data[15] == 'E')
                {
                    received_cmd = EXT_READ_AT_SE;
                    read_cmd_reply_size = 1;
                    read_cmd_reply[0] = 0xE8;
                }
                else if (input_data[15] == 'M')
                {
                    received_cmd = EXT_READ_AT_SM;
                    read_cmd_reply_size = 1;
                    read_cmd_reply[0] = 0;
                }
                else if (input_data[15] == 'N')
                {
                    received_cmd = EXT_READ_AT_SN;
                    read_cmd_reply_size = 2;
                    read_cmd_reply[0] = 0;
                    read_cmd_reply[1] = 1;
                }
                else if (input_data[15] == 'O')
                {
                    received_cmd = EXT_READ_AT_SO;
                    read_cmd_reply_size = 1;
                    read_cmd_reply[0] = 0;
                }
                else if (input_data[15] == 'P')
                {
                    received_cmd = EXT_READ_AT_SP;
                    read_cmd_reply_size = 2;
                    read_cmd_reply[0] = 0;
                    read_cmd_reply[1] = 32;
                }
                else if (input_data[15] == 'T')
                {
                    received_cmd = EXT_READ_AT_ST;
                    read_cmd_reply_size = 2;
                    read_cmd_reply[0] = 13;
                    read_cmd_reply[1] = 88;
                }
            }
            else if (input_data[14] == 'T')
            {
                if (input_data[15] == 'P')
                {
                    received_cmd = EXT_READ_AT_TP;  //TODO
                    read_cmd_reply_size = 2;
                    read_cmd_reply[0] = 0x00;
                    read_cmd_reply[1] = 0x16;
                }
            }
            else if (input_data[14] == 'V')
            {
                if (input_data[15] == '+')
                {
                    received_cmd = EXT_READ_AT_Vplus;
                    read_cmd_reply_size = 2;
                    read_cmd_reply[0] = 0;
                    read_cmd_reply[1] = 0;
                }
                else if (input_data[15] == 'R')
                {
                    received_cmd = EXT_READ_AT_VR;
                    read_cmd_reply_size = 2;
                    uitemp = digi_at_get_parameter_vr();
                    read_cmd_reply[0] = (uint8_t)(uitemp >> 8);
                    read_cmd_reply[1] = (uint8_t)uitemp;
                }
            }
            else if (input_data[14] == 'W')
            {
                if (input_data[15] == 'H')
                {
                    received_cmd = EXT_READ_AT_WH;
                    read_cmd_reply_size = 2;
                    read_cmd_reply[0] = 0;
                    read_cmd_reply[1] = 0;
                }
            }
            else if (input_data[14] == 'Z')
            {
                if (input_data[15] == 'S')
                {
                    received_cmd = EXT_READ_AT_ZS;
                    read_cmd_reply_size = 1;
                    read_cmd_reply[0] = 2;
                }
            }
            else if (input_data[14] == '%')
            {
                if (input_data[15] == 'V')
                {
                    received_cmd = EXT_READ_AT_percV;
                    read_cmd_reply_size = 2;
                    read_cmd_reply[0] = 0x0C;
                    read_cmd_reply[1] = 0xE4;
                }
            }
            if (received_cmd != NO_SUPPORTED_EXT_READ_AT_CMD)
            {
                b_return = true;
                read_cmd = received_cmd;
                read_cmd_sequence_number = input_data[3];
                read_cmd_first_char = input_data[14];
                read_cmd_second_char = input_data[15];
                b_pending_read_cmd = true;
            }
        }
    }
    return b_return;
}

/**@brief This function checks if there is a read AT command received through Zigbee pending to be replied, and, in that case
 *        schedules the function that will reply to that command.
 *
 * @retval True If the transmission of a reply to a read AT command has been scheduled
 * @retval False Otherwise
 */
void digi_wireless_read_at_command_manager(void)
{
    if (b_pending_ping_cmd)
    {
        b_pending_ping_cmd = false;
        digi_wireless_ping_reply();        
    }
    if (b_pending_read_cmd)
    {
        b_pending_read_cmd = false;
        digi_wireless_read_at_cmd_reply();
    }
}

/**@brief This function places in the APS output frame queue the reply to a read AT command received through Zigbee.
*
*/
bool digi_wireless_read_at_cmd_reply(void)
{
    bool b_return = false;
    if (read_cmd >= NUMBER_OF_WIRELESS_AT_READ_COMMANDS)
    {
        LOG_ERR("Not supported command");
        return b_return;
    }
    if ((read_cmd_reply_size < 1) || (read_cmd_reply_size > MAX_SIZE_AT_COMMAND_REPLY))
    {
        LOG_ERR("Size of reply out of range");
        return b_return;
    }

    if( zigbee_aps_get_output_frame_buffer_free_space() )
    {
        uint8_t i;
        aps_output_frame_t element;

        element.dst_addr.addr_short = COORDINATOR_SHORT_ADDRESS;
        element.profile_id = DIGI_PROFILE_ID;
        element.cluster_id = DIGI_AT_COMMAND_REPLY_CLUSTER;
        element.src_endpoint = DIGI_AT_COMMAND_SOURCE_ENDPOINT;
        element.dst_endpoint = DIGI_AT_COMMAND_DESTINATION_ENDPOINT;
        i = 0;
        element.payload[i++] = read_cmd_sequence_number;
        element.payload[i++] = read_cmd_first_char; 
        element.payload[i++] = read_cmd_second_char;
        element.payload[i++] = 0;
        for (uint8_t j=0; j<read_cmd_reply_size; j++)
        {
            element.payload[i++] = read_cmd_reply[j];
        }
        element.payload_size = (zb_uint8_t)i;
        LOG_WRN("Wireless AT command reply");
        if( enqueue_aps_frame(&element) ) b_return = true;
    }

    if( !b_return ) LOG_ERR("Not free space of aps output frame queue");

    return b_return;
}

/**@brief This function places in the APS output frame queue the reply to a ping command received through Zigbee.
*
*/
bool digi_wireless_ping_reply(void)
{
    bool b_return = false;

    if( zigbee_aps_get_output_frame_buffer_free_space() )
    {
        uint8_t i;
        aps_output_frame_t element;

        element.dst_addr.addr_short = COORDINATOR_SHORT_ADDRESS;
        element.profile_id = DIGI_PROFILE_ID;
        element.cluster_id = DIGI_AT_PONG_CLUSTER;
        element.src_endpoint = DIGI_AT_PONG_SOURCE_ENDPOINT;
        element.dst_endpoint = DIGI_AT_PONG_DESTINATION_ENDPOINT;
        i = 0;
        element.payload[i++] = ping_first_char;
        element.payload[i++] = ping_second_char; 
        element.payload_size = (zb_uint8_t)i;
        LOG_WRN("Ping command reply");
        if( enqueue_aps_frame(&element) ) b_return = true;
    }

    if( !b_return ) LOG_ERR("Not free space of aps output frame queue");

    return b_return;
}
