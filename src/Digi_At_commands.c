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
#include "nvram.h"

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
    xbee_parameters.at_nb = 1;     // Xbee's UART parity
    sprintf(xbee_parameters.at_ni, "NORDIC"); // Node identifier string
}

/**@brief This function initializes the structure used to handle the AT commands
 * used to read/write parameters
 */
void digi_at_init_xbee_parameter_command(void)
{
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
        break;
    }

    if(reply_size > 0)
    {
        sendFrameToTcu(reply, reply_size);
    }
    else
    {
        digi_at_reply_error(); // Read command not supported
    }
}

/**@brief This function analizes a buffer containing the last frame
 *  received through the TCU uart. It decides if it contains a valid
 *  AT command, and the type of command (read, write, action).
 * 
 * @param  input_data  Pointer to buffer containing received frame
 * @param  size_input_data  Size of input data(bytes)
 *
 * @retval Negative value Error code.
 * @retval 0 OK
 */
int8_t digi_at_analyze_and_reply_to_command(uint8_t *input_data, uint16_t size_input_data)
{
    if( input_data[0] >= 'a' ) input_data[0] = input_data[0] - 'a' + 'A'; // If lowcase, convert to upcase
    if( input_data[1] >= 'a' ) input_data[1] = input_data[1] - 'a' + 'A'; // If lowcase, convert to upcase
    if( input_data[2] >= 'a' ) input_data[2] = input_data[2] - 'a' + 'A'; // If lowcase, convert to upcase
    if( input_data[3] >= 'a' ) input_data[3] = input_data[3] - 'a' + 'A'; // If lowcase, convert to upcase

    if( ( size_input_data < 4 ) || ( input_data[0] != 'A' ) || ( input_data[1] != 'T' ) )
    {
        digi_at_reply_error();
        return -2; // It is not an AT command
    }

    if( size_input_data == 4 ) // Read command or action command
    {
        if( ( input_data[2] == 'V' ) && ( input_data[3] == 'R' ) ) // ATVR
        {
            digi_at_reply_read_command(AT_VR);
            return 0;
        }
        if( ( input_data[2] == 'H' ) && ( input_data[3] == 'V' ) ) // ATHV
        {
            digi_at_reply_read_command(AT_HV);
            return 0;
        }
        if( ( input_data[2] == 'S' ) && ( input_data[3] == 'H' ) ) // ATSH
        {
            digi_at_reply_read_command(AT_SH);
            return 0;
        }
        if( ( input_data[2] == 'S' ) && ( input_data[3] == 'L' ) ) // ATSL
        {
            digi_at_reply_read_command(AT_SL);
            return 0;
        }
        if( ( input_data[2] == 'J' ) && ( input_data[3] == 'V' ) ) // ATJV
        {
            digi_at_reply_read_command(AT_JV);
            return 0;
        }
        if( ( input_data[2] == 'N' ) && ( input_data[3] == 'J' ) ) // ATNJ
        {
            digi_at_reply_read_command(AT_NJ);
            return 0;
        }
        if( ( input_data[2] == 'N' ) && ( input_data[3] == 'W' ) ) // ATNW
        {
            digi_at_reply_read_command(AT_NW);
            return 0;
        }
        if( ( input_data[2] == 'I' ) && ( input_data[3] == 'D' ) ) // ATID
        {
            digi_at_reply_read_command(AT_ID);
            return 0;
        }
        if( ( input_data[2] == 'N' ) && ( input_data[3] == 'I' ) ) // ATNI
        {
            digi_at_reply_read_command(AT_NI);
            return 0;
        }
        if( ( input_data[2] == 'C' ) && ( input_data[3] == 'E' ) ) // ATCE
        {
            digi_at_reply_read_command(AT_CE);
            return 0;
        }
        if( ( input_data[2] == 'A' ) && ( input_data[3] == 'I' ) ) // ATAI
        {
            digi_at_reply_read_command(AT_AI);
            return 0;
        }
        if( ( input_data[2] == 'C' ) && ( input_data[3] == 'H' ) ) // ATCH
        {
            digi_at_reply_read_command(AT_CH);
            return 0;
        }
        if( ( input_data[2] == 'M' ) && ( input_data[3] == 'Y' ) ) // ATMY
        {
            digi_at_reply_read_command(AT_MY);
            return 0;
        }
        if( ( input_data[2] == 'Z' ) && ( input_data[3] == 'S' ) ) // ATZS
        {
            digi_at_reply_read_command(AT_ZS);
            return 0;
        }
        if( ( input_data[2] == 'B' ) && ( input_data[3] == 'D' ) ) // ATBD
        {
            digi_at_reply_read_command(AT_BD);
            return 0;
        }
        if( ( input_data[2] == 'N' ) && ( input_data[3] == 'B' ) ) // ATNB
        {
            digi_at_reply_read_command(AT_NB);
            return 0;
        }
        else
        {
            digi_at_reply_error();
            return -3; // It is not a supported AT command
        }
    }
    else
    {
        return -4; // TODO, meter comandos de escritura
    }
}

//-----------------------------------------------------------------------------------------------------------------------
// Continuar con lo siguiente:
//     En principio, para considerar que se ha recibido una trama en modo comando, no hay que esperar a que pasen x ms sin
//     recibir caracter. Lo que hay que esperar es a la recepcion de un '\r' que indicará el fin de trama. 
//     Así que habría que diferenciar en cómo se considera el fin de trama cuando se está en modo Modbus, y como se considera
//     el fin de trama cuando se está en modo comando.
//     De hecho parte del analisis de la recepcion de trama en modo comando se realizará durante la rececpión de la trama
//         Si el primer caracter no es 'A', da igual lo que venga despues, la trama es incorrecta
//         Si el segundo caracter no es 'T', da igual lo que venga despues, la trama es incorrecta
//         Si el tamaño de la trama es menor que 5 (incluyendo el '\r'), la trama es incorrecta
//
//     Podríamos definir una estructura de datos para almacenar comandos AT con los siguientes campos
//         Campo 1: Primer caracter del comando
//         Campo 2: Segundo caracter del comando
//         Campo 3: Tipo de dato (numerico o array de caracteres)
//         Campo 4: Puntero a primer byte de la variable que contiene el dato que se quiere leer
//         Campo 5: Tamaño del dato
//     
//         O, en vez de los campos 3, 4, y 5, podría haber un puntero a la función que se debería ejecutar al recibir ese comando
//
//     Definiriamos un array de elementos de esa estructura. Ese array contiene los comandos soportados
//     Al recibir un comando recorreriamos ese array, hasta encontrar el comando específico
//-----------------------------------------------------------------------------------------------------------------------
