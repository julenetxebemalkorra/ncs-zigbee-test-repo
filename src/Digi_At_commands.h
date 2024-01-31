/*
 * Copyright (c) 2024 IED
 *
 */

#ifndef DIGI_AT_COMMANDS_H_
#define DIGI_AT_COMMANDS_H_

/* Enumerative with the supported Xbee AT commands                            */
enum {
    AT_CN, // Exit from command mode
    AT_VR, // Read the Xbee's FW version 
    AT_HV, // Read the Xbee's HW version 
    AT_SH, // Read the high part of the MAC address
    AT_SL, // Read the low part of the MAC address
    AT_JV, // Read / write the "join verification" parameter [JV]
    AT_NJ, // Read / write the "node join time" parameter [NJ]
    AT_NW, // Read / write the "network watchdog" parameter [NW]
    AT_ID, // Read / write the "extended pan id" parameter [ID]
    AT_NI, // Read / write the "node identifier string" parameter [NI]
    AT_CE, // Read / write the "coordinator enabled" parameter [CE]
    AT_AI, // Read the "association indication" parameter [AI]
    AT_CH, // Read the "operation channel" parameter [CH]
    AT_MY, // Read the "short address" parameter [MY]
    AT_ZS, // Read / write the "Zigbee stack profile" parameter [ZS]
    AT_BD, // Read / write the "baud rate" parameter [BD]
    AT_NB, // Read / write the "parity" parameter [BD]
    AT_AC, // Apply changes
    AT_WR, // Write changes in NVM
    AT_NR0 // Reset network (local)
};

/* Xbee parameters                                                            */
struct xbee_parameters_t {
    uint16_t at_vr; // Xbee's FW version (read only)
    uint16_t at_hv; // Xbee's HW version (read only)
    uint32_t at_sh; // High part of the MAC address (read only)
    uint32_t at_sl; // Low part of the MAC address (read only)
    uint8_t at_jv;  // Node join verification parameter
    uint8_t at_nj;  // Node join time parameter
    uint16_t at_nw; // Network watchdog parameter
    uint64_t at_id; // Extended pan id parameter
    uint8_t at_ce;   // Coordinator enabled parameter
    uint8_t at_ai;   // Association indication parameter (read only)
    uint8_t at_ch;   // Operation channel parameter (read only)
    uint16_t at_my;   // Short address parameter (read only)
    uint8_t at_zs;   // Xbee's Zigbee stack profile parameter
    uint8_t at_bd;   // Xbee's UART baud rate parameter
    uint8_t at_nb;   // Xbee's UART parity parameter
    uint8_t at_ni[33];   // Node identifier string parameter
};

#endif /* DIGI_AT_COMMANDS_H_ */







