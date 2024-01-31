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
    sprintf(xbee_parameters.at_ni, "NORDIC\r"); // Node identifier string
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

/**@brief This function sends the node identifier string through the TCU UART
 * We have considered that the maximum size of the identifier is 32 characters
 */
void digi_at_reply_read_ni(void)
{
    uint8_t reply[33]; //32 Characters + '\r'
    uint8_t i = 0;
    bool b_null_found = false;
    
    for(i=0; i<33; i++)
    {
        if( (xbee_parameters.at_ni[i] == 0) || (xbee_parameters.at_ni[i] == '\r') )
        {
            b_null_found = true;
            reply[i] = '\r';
            break;
        }
        else
        {
            reply[i] = xbee_parameters.at_ni[i];
        }
    }
    if( b_null_found )
    {
        sendFrameToTcu(reply, i+1);
    }
    else
    {
        reply[0] = '\r';
        sendFrameToTcu(reply, 1);        
    }
}

/**@brief This function sends the reply to a read AT command through the TCU UART
 *
 * @param  at_command  Enum value representing an AT command.
 */
void digi_at_reply_read_command(uint8_t at_command)
{
    uint8_t reply[18]; //16 Characters + \r + null
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
        itemp = sprintf(reply, "%x\r", (uint32_t)(xbee_parameters.at_id>>32));
        if(itemp > 0)
        {
            reply_size = sprintf(&reply[itemp], "%08x\r", (uint32_t)(xbee_parameters.at_id));
            if(reply_size > 0) reply_size = reply_size+itemp;
        }
        break;
     case AT_NI:
        digi_at_reply_read_ni();
        return;       
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
        digi_at_reply_error(); // Read command not supported
        return;      
    }
    if(reply_size > 0) sendFrameToTcu(reply, reply_size);
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
int8_t digi_at_extract_command(uint8_t *input_data, uint16_t size_input_data)
{
    if( ( size_input_data == 3 ) && ( input_data[0] == '+' ) && ( input_data[1] == '+' ) && ( input_data[2] == '+' ) )
    {
        digi_at_reply_ok();
        return 0;
    }

    if( input_data[size_input_data-1] == '\r' )
    {
        if( ( size_input_data < 5 ) || ( input_data[0] != 'A' ) || ( input_data[1] != 'T' ) )
        {
            digi_at_reply_error();
            return -2; // It is not an AT command
        }
        
        if( size_input_data == 5 ) // Read command or action command
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
    else
    {
        return -1;  // Error code. No detected '\r' at the end of the string
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
 