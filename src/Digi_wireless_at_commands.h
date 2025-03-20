/*
 * Copyright (c) 2025 IED
 *
 */

#ifndef DIGI_WIRELESS_AT_COMMANDS_H_
#define DIGI_WIRELESS_AT_COMMANDS_H_

#include "global_defines.h"

#define MAX_SIZE_AT_COMMAND_REPLY MAXIMUM_SIZE_NODE_IDENTIFIER

/* Enumerative with the supported Xbee wireless AT commands used to read parameters */
enum wireless_at_read_cmd_e{
    EXT_READ_AT_AI, // Read Association indication (AI)
    EXT_READ_AT_AR, // Read Aggregation route notification (AR)
    EXT_READ_AT_BD, // Read Serial interface data rate (BD)
    EXT_READ_AT_BH, // Read Broadcast radius (BH)
    EXT_READ_AT_CC, // Read Command sequence character (CC)
    EXT_READ_AT_CE, // Read Coordinator enable (CE) 
    EXT_READ_AT_CH, // Read Operating channel (CH) 
    EXT_READ_AT_CI, // Read Cluster identifier (CI)   
    EXT_READ_AT_CR, // Read PAN conflict threshold (CR) 
    EXT_READ_AT_CT, // Read Command mode timeout (CT) 
    EXT_READ_AT_D0, // Read AD0/DIO0 configuration (D0) 
    EXT_READ_AT_D1, // Read AD1/DIO1 configuration (D1) 
    EXT_READ_AT_D2, // Read AD2/DIO2 configuration (D2) 
    EXT_READ_AT_D3, // Read AD3/DIO3 configuration (D3) 
    EXT_READ_AT_D4, // Read AD4/DIO4 configuration (D4) 
    EXT_READ_AT_D5, // Read DIO5/Assoc configuration (D5) 
    EXT_READ_AT_D6, // Read DIO6 configuration (D6) 
    EXT_READ_AT_D7, // Read DIO7 configuration (D7) 
    EXT_READ_AT_D8, // Read DIO8/SleepRQ configuration (D8) 
    EXT_READ_AT_D9, // Read DIO9/ON_SLEEP configuration (D9) 
    EXT_READ_AT_DB, // Read Received signal strength (DB) 
    EXT_READ_AT_DD, // Read Device type identifier (DD)
    EXT_READ_AT_DE, // Read Destination endpoint (DE)
    EXT_READ_AT_DH, // Read Destination address (DH)
    EXT_READ_AT_DL, // Read Destination address (DL)
    EXT_READ_AT_EA, // Read ACK failures (EA)
    EXT_READ_AT_EE, // Read Encryption enable (EE)
    EXT_READ_AT_EO, // Read Encryption options (EO)
    EXT_READ_AT_GT, // Read Guard times (GT)
    EXT_READ_AT_HV, // Read Hardware version (HV)
    EXT_READ_AT_IC, // Read DIO change detect (IC)
    EXT_READ_AT_ID, // Read Extended PAN identifier (ID)
    EXT_READ_AT_II, // Read Initial PAN identifier (II)
    EXT_READ_AT_IR, // Read I/O sample rate (IR)
    EXT_READ_AT_JN, // Read Join notification (JN)
    EXT_READ_AT_JV, // Read Join verification (JV)
    EXT_READ_AT_KY, // Read Link encryption key (KY)
    EXT_READ_AT_LT, // Read Associate LED blink time (LT)
    EXT_READ_AT_MP, // Read Parent address (MP)
    EXT_READ_AT_MY, // Read Network address (MY)
    EXT_READ_AT_NB, // Read Serial interface parity (NB)
    EXT_READ_AT_NC, // Read Number of remaining children (NC)
    EXT_READ_AT_NH, // Read Maximum hops (NH)
    EXT_READ_AT_NI, // Read Node identifier (NI)
    EXT_READ_AT_NJ, // Read Node join time (NJ)
    EXT_READ_AT_NK, // Read Network encryption key (NK)
    EXT_READ_AT_NP, // Read Maximum RF payload (NP)
    EXT_READ_AT_NT, // Read Node discovery timeout (NT)
    EXT_READ_AT_NW, // Read Network watchdog timeout (NW)
    EXT_READ_AT_OI, // Read PAN identifier (OI)
    EXT_READ_AT_OP, // Read Extended PAN identifier (OP)
    EXT_READ_AT_P2, // Read DIO12/CD configuration (P2)
    EXT_READ_AT_P3, // Read DIO13/DOUT configuration (P3)
    EXT_READ_AT_P4, // Read DIO14/DIN configuration (P4)
    EXT_READ_AT_P5, // Read DIO15 configuration (P5)
    EXT_READ_AT_P6, // Read DIO16 configuration (P6)
    EXT_READ_AT_P7, // Read DIO17 configuration (P7)
    EXT_READ_AT_P8, // Read DIO18 configuration (P8)
    EXT_READ_AT_P9, // Read DIO19 configuration (P9)
    EXT_READ_AT_PD, // Read Pull-up/down direction (PD)   
    EXT_READ_AT_PL, // Read Transmit power level (PL)
    EXT_READ_AT_PO, // Read Read Polling rate (PO)
    EXT_READ_AT_PP, // Read Transmit power at PL4 (PP)
    EXT_READ_AT_PR, // Read Pull-up resistor enable (PR)
    EXT_READ_AT_RO, // Read Packetization timeout (RO)
    EXT_READ_AT_SB, // Read Stop bits (SB)
    EXT_READ_AT_SC, // Read Scan channels (SC)
    EXT_READ_AT_SD, // Read Scan duration (SD)
    EXT_READ_AT_SE, // Read Source endpoint (SE)
    EXT_READ_AT_SM, // Read Sleep mode (SM)
    EXT_READ_AT_SN, // Read Peripheral sleep count (SN)
    EXT_READ_AT_SO, // Read Sleep options (SO)
    EXT_READ_AT_SP, // Read Cyclic sleep period (SP)
    EXT_READ_AT_ST, // Read Time before sleep (ST)
    EXT_READ_AT_TP, // Read Temperature (TP)
    EXT_READ_AT_Vplus, // Read Supply voltage high threshold (V+)
    EXT_READ_AT_VR, // Read Firmware version (VR)
    EXT_READ_AT_WH, // Read Wake host delay (WH)
    EXT_READ_AT_ZS, // Read ZigBee stack profile (ZS)
    EXT_READ_AT_percV, // Read Supply voltage (%V)
    NUMBER_OF_WIRELESS_AT_READ_COMMANDS,
    NO_SUPPORTED_EXT_READ_AT_CMD
};

void digi_wireless_at_init(void);
bool is_a_ping_command(uint8_t* input_data, int16_t size_of_input_data);
bool is_a_digi_read_at_command(uint8_t* input_data, int16_t size_of_input_data);
void digi_wireless_read_at_command_manager(void);
bool digi_wireless_read_at_cmd_reply(void);
bool digi_wireless_ping_reply(void);

#endif /* DIGI_WIRELESS_AT_COMMANDS_H_ */







