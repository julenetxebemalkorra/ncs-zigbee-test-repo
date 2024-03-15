/*
 * Copyright (c) 2024 IED
 *
 */

/** @file
 *
 * @brief Managment of the UART0, which is connected to the TCU.
 */

//#include <zephyr/kernel.h>

// UART async includes
//#include <zephyr/device.h>
//#include <zephyr/devicetree.h>
//#include <zephyr/sys/ring_buffer.h>
//#include <zephyr/drivers/gpio.h>
//#include <zephyr/drivers/uart.h>
//#include <string.h>

#include <zephyr/kernel.h>
#include "Digi_At_commands.h"
#include "tcu_Uart.h"

static struct xbee_parameters_t xbee_parameters; // Xbee's parameters
static struct xbee_parameter_comando_at_t xbee_parameter_comando_at[NUMBER_OF_PARAMETER_AT_COMMANDS];

/**@brief This function initializes the Digi_At_commands firmware module
 *
 */
void digi_at_init(void)
{
    digi_at_init_xbee_parameters();
    digi_at_init_xbee_parameter_command();
}

/**@brief This function initializes with default values the Xbee parameters
 * 
 */
void digi_at_init_xbee_parameters(void)
{
    xbee_parameters.at_vr = 1;     // Xbee's FW version
    xbee_parameters.at_hv = 1;     // Xbee's HW version
    xbee_parameters.at_sh = 0;     // High part of the MAC address
    xbee_parameters.at_sl = 1;     // Low part of the MAC address
    xbee_parameters.at_jv = 1;     // Node join verification
    xbee_parameters.at_nj = 0xFF;  // Node join time
    xbee_parameters.at_nw = 10;    // Network watchdog
    xbee_parameters.at_id = 0;     // Extended pan id
    xbee_parameters.at_ce = 0;     // Coordinator enabled
    xbee_parameters.at_ai = 0xFF;  // Association indication
    xbee_parameters.at_ch = 0;     // Operation channel
    xbee_parameters.at_my = 0;     // Short address
    xbee_parameters.at_zs = 2;     // Xbee's Zigbee stack profile (2 = ZigBee-PRO)
    xbee_parameters.at_bd = 4;     // Xbee's UART baud rate (4 = 19200)
    xbee_parameters.at_nb = 0;     // Xbee's UART parity (0 = None)
    xbee_parameters.at_ni[0] = ' ';  // Node identifier (Blank)
    xbee_parameters.at_ni[1] = '\0';  // Node identifier (null string).
}

/**@brief This function initializes the structure used to handle the AT commands
 * used to read/write parameters
 */
void digi_at_init_xbee_parameter_command(void) // TODO This structure currently is not used. It was part of a major change that eventually I did not implement
{                                              // TODO We should either use it, or delete it.
    // Xbee's FW version
    xbee_parameter_comando_at[AT_VR].first_char = 'V';
    xbee_parameter_comando_at[AT_VR].second_char = 'R';
    xbee_parameter_comando_at[AT_VR].data = (uint8_t*)&xbee_parameters.at_vr;
    xbee_parameter_comando_at[AT_VR].size_of_data = 2;
    xbee_parameter_comando_at[AT_VR].b_numeric_data = true;
    xbee_parameter_comando_at[AT_VR].b_read_only = true;

    // Xbee's HW version
    xbee_parameter_comando_at[AT_HV].first_char = 'H';
    xbee_parameter_comando_at[AT_HV].second_char = 'V';
    xbee_parameter_comando_at[AT_HV].data = (uint8_t*)&xbee_parameters.at_hv;
    xbee_parameter_comando_at[AT_HV].size_of_data = 2;
    xbee_parameter_comando_at[AT_HV].b_numeric_data = true;
    xbee_parameter_comando_at[AT_HV].b_read_only = true;

    // High part of the MAC address
    xbee_parameter_comando_at[AT_SH].first_char = 'S';
    xbee_parameter_comando_at[AT_SH].second_char = 'H';
    xbee_parameter_comando_at[AT_SH].data = (uint8_t*)&xbee_parameters.at_sh;
    xbee_parameter_comando_at[AT_SH].size_of_data = 4;
    xbee_parameter_comando_at[AT_SH].b_numeric_data = true;
    xbee_parameter_comando_at[AT_SH].b_read_only = true;

    // Low part of the MAC address
    xbee_parameter_comando_at[AT_SL].first_char = 'S';
    xbee_parameter_comando_at[AT_SL].second_char = 'L';
    xbee_parameter_comando_at[AT_SL].data = (uint8_t*)&xbee_parameters.at_sl;
    xbee_parameter_comando_at[AT_SL].size_of_data = 4;
    xbee_parameter_comando_at[AT_SL].b_numeric_data = true;
    xbee_parameter_comando_at[AT_SL].b_read_only = true;

    // "Coordinator join verification" parameter
    xbee_parameter_comando_at[AT_JV].first_char = 'J';
    xbee_parameter_comando_at[AT_JV].second_char = 'V';
    xbee_parameter_comando_at[AT_JV].data = (uint8_t*)&xbee_parameters.at_jv;
    xbee_parameter_comando_at[AT_JV].size_of_data = 1;
    xbee_parameter_comando_at[AT_JV].b_numeric_data = true;
    xbee_parameter_comando_at[AT_JV].b_read_only = false;

    // "node join time" parameter
    xbee_parameter_comando_at[AT_NJ].first_char = 'N';
    xbee_parameter_comando_at[AT_NJ].second_char = 'J';
    xbee_parameter_comando_at[AT_NJ].data = (uint8_t*)&xbee_parameters.at_nj;
    xbee_parameter_comando_at[AT_NJ].size_of_data = 1;
    xbee_parameter_comando_at[AT_NJ].b_numeric_data = true;
    xbee_parameter_comando_at[AT_NJ].b_read_only = false;

    // "Network watchdog" parameter
    xbee_parameter_comando_at[AT_NW].first_char = 'N';
    xbee_parameter_comando_at[AT_NW].second_char = 'W';
    xbee_parameter_comando_at[AT_NW].data = (uint8_t*)&xbee_parameters.at_nw;
    xbee_parameter_comando_at[AT_NW].size_of_data = 2;
    xbee_parameter_comando_at[AT_NW].b_numeric_data = true;
    xbee_parameter_comando_at[AT_NW].b_read_only = false;

    // "Extended pan id" parameter
    xbee_parameter_comando_at[AT_ID].first_char = 'I';
    xbee_parameter_comando_at[AT_ID].second_char = 'D';
    xbee_parameter_comando_at[AT_ID].data = (uint8_t*)&xbee_parameters.at_id;
    xbee_parameter_comando_at[AT_ID].size_of_data = 8;
    xbee_parameter_comando_at[AT_ID].b_numeric_data = true;
    xbee_parameter_comando_at[AT_ID].b_read_only = false;

    // "Node identifier string" parameter
    xbee_parameter_comando_at[AT_NI].first_char = 'N';
    xbee_parameter_comando_at[AT_NI].second_char = 'I';
    xbee_parameter_comando_at[AT_NI].data = (uint8_t*)&xbee_parameters.at_ni;
    xbee_parameter_comando_at[AT_NI].size_of_data = 32;
    xbee_parameter_comando_at[AT_NI].b_numeric_data = false;
    xbee_parameter_comando_at[AT_NI].b_read_only = false;

    // "Coordinator enabled" parameter
    xbee_parameter_comando_at[AT_CE].first_char = 'C';
    xbee_parameter_comando_at[AT_CE].second_char = 'E';
    xbee_parameter_comando_at[AT_CE].data = (uint8_t*)&xbee_parameters.at_ce;
    xbee_parameter_comando_at[AT_CE].size_of_data = 1;
    xbee_parameter_comando_at[AT_CE].b_numeric_data = true;
    xbee_parameter_comando_at[AT_CE].b_read_only = false;

    // "Association indication" parameter
    xbee_parameter_comando_at[AT_AI].first_char = 'A';
    xbee_parameter_comando_at[AT_AI].second_char = 'I';
    xbee_parameter_comando_at[AT_AI].data = (uint8_t*)&xbee_parameters.at_ai;
    xbee_parameter_comando_at[AT_AI].size_of_data = 1;
    xbee_parameter_comando_at[AT_AI].b_numeric_data = true;
    xbee_parameter_comando_at[AT_AI].b_read_only = true;

    // "Operation channel" parameter
    xbee_parameter_comando_at[AT_CH].first_char = 'C';
    xbee_parameter_comando_at[AT_CH].second_char = 'H';
    xbee_parameter_comando_at[AT_CH].data = (uint8_t*)&xbee_parameters.at_ch;
    xbee_parameter_comando_at[AT_CH].size_of_data = 1;
    xbee_parameter_comando_at[AT_CH].b_numeric_data = true;
    xbee_parameter_comando_at[AT_CH].b_read_only = true;

    // "Short address" parameter
    xbee_parameter_comando_at[AT_MY].first_char = 'M';
    xbee_parameter_comando_at[AT_MY].second_char = 'Y';
    xbee_parameter_comando_at[AT_MY].data = (uint8_t*)&xbee_parameters.at_my;
    xbee_parameter_comando_at[AT_MY].size_of_data = 2;
    xbee_parameter_comando_at[AT_MY].b_numeric_data = true;
    xbee_parameter_comando_at[AT_MY].b_read_only = true;

    // "Zigbee stack profile" parameter
    xbee_parameter_comando_at[AT_ZS].first_char = 'Z';
    xbee_parameter_comando_at[AT_ZS].second_char = 'S';
    xbee_parameter_comando_at[AT_ZS].data = (uint8_t*)&xbee_parameters.at_zs;
    xbee_parameter_comando_at[AT_ZS].size_of_data = 1;
    xbee_parameter_comando_at[AT_ZS].b_numeric_data = true;
    xbee_parameter_comando_at[AT_ZS].b_read_only = false;

    // "baud rate" parameter
    xbee_parameter_comando_at[AT_BD].first_char = 'B';
    xbee_parameter_comando_at[AT_BD].second_char = 'D';
    xbee_parameter_comando_at[AT_BD].data = (uint8_t*)&xbee_parameters.at_bd;
    xbee_parameter_comando_at[AT_BD].size_of_data = 2;
    xbee_parameter_comando_at[AT_BD].b_numeric_data = true;
    xbee_parameter_comando_at[AT_BD].b_read_only = false;

    // "Parity" parameter
    xbee_parameter_comando_at[AT_NB].first_char = 'N';
    xbee_parameter_comando_at[AT_NB].second_char = 'B';
    xbee_parameter_comando_at[AT_NB].data = (uint8_t*)&xbee_parameters.at_nb;
    xbee_parameter_comando_at[AT_NB].size_of_data = 1;
    xbee_parameter_comando_at[AT_NB].b_numeric_data = true;
    xbee_parameter_comando_at[AT_NB].b_read_only = false;
}

/**@brief This function sends an 'OK\r' string through the TCU UART
 * That is the reply sent by Xbee module when an AT command is accepted
 */
void digi_at_reply_ok(void)
{
    uint8_t reply[3] = {'O','K','\r'};
    sendFrameToTcu(reply, 3);
}

/**@brief This function sends an 'ERROR\r' string through the TCU UART
 * That is the reply sent by Xbee module when an AT command is not accepted
 */
void digi_at_reply_error(void)
{
    uint8_t reply[6] = {'E','R','R','O','R','\r'};
    sendFrameToTcu(reply, 6);
}

/**@brief This function places the node identifier string into a buffer
 * We have considered that the maximum size of the identifier is 32 characters
 */
int8_t digi_at_read_ni(uint8_t* buffer)
{
    uint8_t i = 0;
    
    for(i=0; i<33; i++)
    {
        if( xbee_parameters.at_ni[i] == 0 )
        {
            break;
        }
        else
        {
            buffer[i] = xbee_parameters.at_ni[i];
        }
    }

    buffer[i] = '\r';
    buffer[i+1] = 0;

    return(i+1);
}


/**@brief This function sends the reply to a read AT command through the TCU UART
 *
 * @param  at_command  Enum value representing an AT command.
 */
void digi_at_reply_read_command(uint8_t at_command)
{
    uint8_t reply[34]; // 32 Characters + '\r' + null
    int8_t reply_size = 0;
    int8_t itemp;
    switch (at_command)
    {
     case AT_VR:
        reply_size = sprintf(reply, "%x\r", xbee_parameters.at_vr);
        break;
     case AT_HV:
        reply_size = sprintf(reply, "%x\r", xbee_parameters.at_hv);
        break;
     case AT_SH:
        reply_size = sprintf(reply, "%x\r", xbee_parameters.at_sh);
        break;
     case AT_SL:
        reply_size = sprintf(reply, "%x\r", xbee_parameters.at_sl);
        break;
     case AT_JV:
        reply_size = sprintf(reply, "%x\r", xbee_parameters.at_jv);
        break;
     case AT_NJ:
        reply_size = sprintf(reply, "%x\r", xbee_parameters.at_nj);
        break;
     case AT_NW:
        reply_size = sprintf(reply, "%x\r", xbee_parameters.at_nw);
        break;
     case AT_ID:
        itemp = sprintf(reply, "%x", (uint32_t)(xbee_parameters.at_id>>32));
        if(itemp > 1)
        {
            reply_size = sprintf(&reply[itemp], "%08x\r", (uint32_t)(xbee_parameters.at_id));
            if(reply_size > 0) reply_size = reply_size+itemp;
        }
        else
        {
            reply_size = sprintf(reply, "%x\r", (uint32_t)(xbee_parameters.at_id));
        }
        break;
     case AT_NI:
        reply_size = digi_at_read_ni(reply);
        break;
     case AT_CE:
        reply_size = sprintf(reply, "%x\r", xbee_parameters.at_ce);
        break;
     case AT_AI:
        reply_size = sprintf(reply, "%x\r", xbee_parameters.at_ai);
        break;
     case AT_CH:
        reply_size = sprintf(reply, "%x\r", xbee_parameters.at_ch);
        break;
     case AT_MY:
        reply_size = sprintf(reply, "%x\r", xbee_parameters.at_my);
        break;
     case AT_ZS:
        reply_size = sprintf(reply, "%x\r", xbee_parameters.at_zs);
        break;
     case AT_BD:
        reply_size = sprintf(reply, "%x\r", xbee_parameters.at_bd);
        break;
     case AT_NB:
        reply_size = sprintf(reply, "%x\r", xbee_parameters.at_nb);
        break;
     default:
        break; //It should never happen
    }

    if(reply_size > 0)
    {
        sendFrameToTcu(reply, reply_size);
    }
    else
    {
        digi_at_reply_error(); // Read command not supported (It should never happen)
    }
}

/**@brief This function reacts to an action AT command and sends the reply through the TCU UART
 *
 * @param  at_command  Enum value representing an AT command.
 *
 */
void digi_at_reply_action_command(uint8_t at_command)
{
    switch (at_command)
    {
     case AT_AC: //Apply changes and leave command mode
        digi_at_reply_ok();
        break;
     case AT_WR: //Write in flash memory and leave command mode
        digi_at_reply_ok();
        break;
     case AT_CN: //Leave command mode
        digi_at_reply_ok();
        break;
     default:
        digi_at_reply_error(); // Command not supported (It should never happen)
        break; //It should never happen
    }
}

/**@brief This function updates the at parameter structure with the new value
 *        sent with an AT write command.
 *        It also generates the "OK" or "ERROR" reply to the command
 *
 * @param  at_command  Enum value representing a write AT command.
 * @param  command_data  Data to be written
 *
 * @retval True if the new value was accepted
 * @retval False if the new value was not accepted (out of range)
 */
bool digi_at_reply_write_command(uint8_t at_command, const char *command_data_string, uint8_t string_size)
{
    bool return_value = false;

    if( at_command == AT_NI ) // The data for that command is a string
    {
        if( string_size <= MAXIMUM_SIZE_NODE_IDENTIFIER )
        {
            uint8_t i;
            for(i = 0; i<string_size; i++)
            {
                xbee_parameters.at_ni[i] = command_data_string[i];
            }
            xbee_parameters.at_ni[i] = '\0'; //End of string
            return_value = true;
        }
    }
    else //The data for the rest of the commands is a number
    {
        uint64_t command_data;
        if( convert_hex_string_to_uint64(command_data_string, string_size, &command_data) )
        {
            switch (at_command)
            {
             case AT_JV:
                if( command_data == 1 ) return_value = true; // Only accepted value with the current firmware
                break;
             case AT_NJ:
                if( command_data == 0xFF ) return_value = true; // Only accepted value with the current firmware
                break;
             case AT_NW: // Valid range is [0, 0x64FF]
                if( command_data < 0x64FF )
                {
                    xbee_parameters.at_nw = command_data;
                    return_value = true; 
                }
                break;
             case AT_ID: // Valid range is [0, 0xFFFFFFFFFFFFFFFF]
                xbee_parameters.at_id = command_data;
                return_value = true; 
                break;
             case AT_CE:
                if( command_data == 0 ) return_value = true; // Only accepted value with the current firmware
                break;
             case AT_ZS:
                if( command_data == 2 ) return_value = true; // Only accepted value with the current firmware (Zigbee Pro stack)
                break;
             case AT_BD:
                if( command_data == 4 ) return_value = true; // Only accepted value with the current firmware (19200 bps)
                break;
             case AT_NB:
                if( command_data == 1 ) return_value = true; // Only accepted value with the current firmware (no parity)
                break;
             default:
                break; //It should never happen
            }
        }         
    }

    if( return_value )
    {
        digi_at_reply_ok();
    }
    else
    {
        digi_at_reply_error();
    }
    return return_value;
}

/**@brief This function analizes a buffer containing the last frame
 *  received through the TCU uart. It decides if it contains a valid
 *  AT command, and the type of command (read, write, action).
 * 
 * @param  input_data  Pointer to buffer containing received frame
 * @param  size_input_data  Size of input data(bytes)
 *
 * @retval Negative value Error code.
 * @retval 0 OK. Command accepted and we should stay in command mode
 * @retval 1 OK. Command accepted and we should leave command mode
 */
int8_t digi_at_analyze_and_reply_to_command(uint8_t *input_data, uint16_t size_input_data)
{
    if( size_input_data < MINIMUM_SIZE_AT_COMMAND )
    {
        digi_at_reply_error();
        return AT_CMD_ERROR_TOO_SHORT;
    }

    if( size_input_data > MAXIMUM_SIZE_AT_COMMAND )
    {
        digi_at_reply_error();
        return AT_CMD_ERROR_TOO_LONG;
    }

    if( input_data[0] >= 'a' ) input_data[0] = input_data[0] - 'a' + 'A'; // If lowcase, convert to upcase
    if( input_data[1] >= 'a' ) input_data[1] = input_data[1] - 'a' + 'A'; // If lowcase, convert to upcase
    if( input_data[2] >= 'a' ) input_data[2] = input_data[2] - 'a' + 'A'; // If lowcase, convert to upcase
    if( input_data[3] >= 'a' ) input_data[3] = input_data[3] - 'a' + 'A'; // If lowcase, convert to upcase

    if( ( input_data[0] != 'A' ) || ( input_data[1] != 'T' ) )
    {
        digi_at_reply_error();
        return AT_CMD_ERROR_WRONG_PREFIX;
    }

    if( size_input_data == 4 ) // Four bytes --> It is a read command or an action command
    {
        if( ( input_data[2] == 'V' ) && ( input_data[3] == 'R' ) ) // ATVR
        {
            digi_at_reply_read_command(AT_VR);
            return AT_CMD_OK_STAY_IN_CMD_MODE;
        }
        if( ( input_data[2] == 'H' ) && ( input_data[3] == 'V' ) ) // ATHV
        {
            digi_at_reply_read_command(AT_HV);
            return AT_CMD_OK_STAY_IN_CMD_MODE;
        }
        if( ( input_data[2] == 'S' ) && ( input_data[3] == 'H' ) ) // ATSH
        {
            digi_at_reply_read_command(AT_SH);
            return AT_CMD_OK_STAY_IN_CMD_MODE;
        }
        if( ( input_data[2] == 'S' ) && ( input_data[3] == 'L' ) ) // ATSL
        {
            digi_at_reply_read_command(AT_SL);
            return AT_CMD_OK_STAY_IN_CMD_MODE;
        }
        if( ( input_data[2] == 'J' ) && ( input_data[3] == 'V' ) ) // ATJV
        {
            digi_at_reply_read_command(AT_JV);
            return AT_CMD_OK_STAY_IN_CMD_MODE;
        }
        if( ( input_data[2] == 'N' ) && ( input_data[3] == 'J' ) ) // ATNJ
        {
            digi_at_reply_read_command(AT_NJ);
            return AT_CMD_OK_STAY_IN_CMD_MODE;
        }
        if( ( input_data[2] == 'N' ) && ( input_data[3] == 'W' ) ) // ATNW
        {
            digi_at_reply_read_command(AT_NW);
            return AT_CMD_OK_STAY_IN_CMD_MODE;
        }
        if( ( input_data[2] == 'I' ) && ( input_data[3] == 'D' ) ) // ATID
        {
            digi_at_reply_read_command(AT_ID);
            return AT_CMD_OK_STAY_IN_CMD_MODE;
        }
        if( ( input_data[2] == 'N' ) && ( input_data[3] == 'I' ) ) // ATNI
        {
            digi_at_reply_read_command(AT_NI);
            return AT_CMD_OK_STAY_IN_CMD_MODE;
        }
        if( ( input_data[2] == 'C' ) && ( input_data[3] == 'E' ) ) // ATCE
        {
            digi_at_reply_read_command(AT_CE);
            return AT_CMD_OK_STAY_IN_CMD_MODE;
        }
        if( ( input_data[2] == 'A' ) && ( input_data[3] == 'I' ) ) // ATAI
        {
            digi_at_reply_read_command(AT_AI);
            return AT_CMD_OK_STAY_IN_CMD_MODE;
        }
        if( ( input_data[2] == 'C' ) && ( input_data[3] == 'H' ) ) // ATCH
        {
            digi_at_reply_read_command(AT_CH);
            return AT_CMD_OK_STAY_IN_CMD_MODE;
        }
        if( ( input_data[2] == 'M' ) && ( input_data[3] == 'Y' ) ) // ATMY
        {
            digi_at_reply_read_command(AT_MY);
            return AT_CMD_OK_STAY_IN_CMD_MODE;
        }
        if( ( input_data[2] == 'Z' ) && ( input_data[3] == 'S' ) ) // ATZS
        {
            digi_at_reply_read_command(AT_ZS);
            return AT_CMD_OK_STAY_IN_CMD_MODE;
        }
        if( ( input_data[2] == 'B' ) && ( input_data[3] == 'D' ) ) // ATBD
        {
            digi_at_reply_read_command(AT_BD);
            return AT_CMD_OK_STAY_IN_CMD_MODE;
        }
        if( ( input_data[2] == 'N' ) && ( input_data[3] == 'B' ) ) // ATNB
        {
            digi_at_reply_read_command(AT_NB);
            return AT_CMD_OK_STAY_IN_CMD_MODE;
        }
        if( ( input_data[2] == 'A' ) && ( input_data[3] == 'C' ) ) // ATAC
        {
            digi_at_reply_action_command(AT_AC);
            return AT_CMD_OK_LEAVE_CMD_MODE;
        }
        if( ( input_data[2] == 'W' ) && ( input_data[3] == 'R' ) ) // ATWR
        {
            digi_at_reply_action_command(AT_WR);
            return AT_CMD_OK_LEAVE_CMD_MODE;
        }
        if( ( input_data[2] == 'C' ) && ( input_data[3] == 'N' ) ) // ATCN
        {
            digi_at_reply_action_command(AT_CN);
            return AT_CMD_OK_LEAVE_CMD_MODE;
        }
        digi_at_reply_error();
        return AT_CMD_ERROR_NOT_SUPPORTED_READ_CMD;
    }
    else // More than 4 characters --> It is a write command
    {
        uint8_t command_data_size = (uint8_t)size_input_data - 4;

        if( ( input_data[2] == 'N' ) && ( input_data[3] == 'I' ) )
        {
            if( digi_at_reply_write_command(AT_NI, &input_data[4], command_data_size) ) return AT_CMD_OK_STAY_IN_CMD_MODE;
            else return AT_CMD_ERROR_WRITE_DATA_NOT_VALID;
        }
        if( ( input_data[2] == 'J' ) && ( input_data[3] == 'V' ) ) // ATJV
        {
            if( digi_at_reply_write_command(AT_JV, &input_data[4], command_data_size) ) return AT_CMD_OK_STAY_IN_CMD_MODE;
            else return AT_CMD_ERROR_WRITE_DATA_NOT_VALID;
        }
        if( ( input_data[2] == 'N' ) && ( input_data[3] == 'J' ) ) // ATNJ
        {
            if( digi_at_reply_write_command(AT_NJ, &input_data[4], command_data_size) ) return AT_CMD_OK_STAY_IN_CMD_MODE;
            else return AT_CMD_ERROR_WRITE_DATA_NOT_VALID;
        }
        if( ( input_data[2] == 'N' ) && ( input_data[3] == 'W' ) ) // ATNW
        {
            if( digi_at_reply_write_command(AT_NW, &input_data[4], command_data_size) ) return AT_CMD_OK_STAY_IN_CMD_MODE;
            else return AT_CMD_ERROR_WRITE_DATA_NOT_VALID;
        }
        if( ( input_data[2] == 'I' ) && ( input_data[3] == 'D' ) ) // ATID
        {
            if( digi_at_reply_write_command(AT_ID, &input_data[4], command_data_size) ) return AT_CMD_OK_STAY_IN_CMD_MODE;
            else return AT_CMD_ERROR_WRITE_DATA_NOT_VALID;
        }
        if( ( input_data[2] == 'C' ) && ( input_data[3] == 'E' ) ) // ATCE
        {
            if( digi_at_reply_write_command(AT_CE, &input_data[4], command_data_size) ) return AT_CMD_OK_STAY_IN_CMD_MODE;
            else return AT_CMD_ERROR_WRITE_DATA_NOT_VALID;
        }
        if( ( input_data[2] == 'Z' ) && ( input_data[3] == 'S' ) ) // ATZS
        {
            if( digi_at_reply_write_command(AT_ZS, &input_data[4], command_data_size) ) return AT_CMD_OK_STAY_IN_CMD_MODE;
            else return AT_CMD_ERROR_WRITE_DATA_NOT_VALID;
        }
        if( ( input_data[2] == 'B' ) && ( input_data[3] == 'D' ) ) // ATBD
        {
            if( digi_at_reply_write_command(AT_BD, &input_data[4], command_data_size) ) return AT_CMD_OK_STAY_IN_CMD_MODE;
            else return AT_CMD_ERROR_WRITE_DATA_NOT_VALID;
        }
        if( ( input_data[2] == 'N' ) && ( input_data[3] == 'B' ) ) // ATNB
        {
            if( digi_at_reply_write_command(AT_NB, &input_data[4], command_data_size) ) return AT_CMD_OK_STAY_IN_CMD_MODE;
            else return AT_CMD_ERROR_WRITE_DATA_NOT_VALID;
        }
        digi_at_reply_error();
        return AT_CMD_ERROR_NOT_SUPPORTED_WRITE_CMD; // It is not a supported write AT command
    }
}

/**@brief This auxiliary function converts an string containing a number in hexadecimal format
 *  in the numeric value
 * 
 * @param  hex_string  Pointer to buffer
 * @param  string_size Size of string
 * @param  output_result Pointer to the variable where the result is stored.
 *
 * @retval True if string contained an hexadecimal number
 * @retval False if the string did not contain an hexadecimal number
 */
bool convert_hex_string_to_uint64(const char *hex_string, uint8_t string_size, uint64_t *output_result)
{
    uint64_t result = 0;
    if( ( hex_string == NULL ) || ( string_size == 0 ) || ( string_size > 16 ) || ( output_result == NULL ) )
    {
        return false; // Verify the input parameters are valid
    }

    result = 0; // Initialize result

    for( uint8_t i = 0; i < string_size; ++i )
    {
        uint8_t current_byte = hex_string[i];
        // Verify if it is an hexadecimal character

        if( ( current_byte >= '0' ) && ( current_byte <= '9' ) )
        {
            current_byte = current_byte - '0';
        }
        else if( ( current_byte >= 'a' ) && ( current_byte <= 'f' ) )
        {
            current_byte = current_byte - 'a' + 10;
        }
        else if( ( current_byte >= 'A' ) && ( current_byte <= 'F' ) )
        {
            current_byte = current_byte - 'A' + 10;
        }
        else
        {
            return false;
        }

        //Add contribution of current byte
        result = ( result << 4 ) + (uint64_t)(current_byte);
    }

    *output_result = result;
    return true;
}

