/*
 * Copyright (c) 2024 IED
 *
 */

#ifndef DIGI_AT_COMMANDS_H_
#define DIGI_AT_COMMANDS_H_

#include "global_defines.h"

#define MINIMUM_SIZE_AT_COMMAND 4
#define MAXIMUM_SIZE_AT_COMMAND 4 + 1 + MAXIMUM_SIZE_LINK_KEY //Write command + blank + size of node identifier
#define STANDARD_SIZE_LINK_KEY 16

#define HARDCODED_ATJV_VALUE 1
#define HARDCODED_ATNJ_VALUE 0xFF

/* Enumerative with the supported Xbee AT commands used to read/write parameters */
enum parameter_at_command_e{
    AT_VR, // Read the Xbee's FW version
    AT_HV, // Read the Xbee's HW version
    AT_SH, // Read the high part of the MAC address
    AT_SL, // Read the low part of the MAC address
    AT_JV, // Read / write the "coordinator join verification" parameter [JV]
    AT_NJ, // Read / write the "node join time" parameter [NJ]
    AT_NW, // Read / write the "network watchdog" parameter [NW]
    AT_ID, // Read / write the "extended pan id" parameter [ID]
    AT_NI, // Read / write the "node identifier string" parameter [NI]
    AT_CE, // Read / write the "coordinator enabled" parameter [CE]
    AT_AI, // Read the "association indication" parameter [AI]
    AT_CH, // Read the "operation channel" parameter [CH]
    AT_MY, // Read the "short address" parameter [MY]
    AT_EE, // Read / write the "encryption enable" parameter [EE]
    AT_EO, // Read / write the "encryption options" parameter [EO]
    AT_KY, // Read / write the "link encryption key" parameter [KY]
    AT_ZS, // Read / write the "Zigbee stack profile" parameter [ZS]
    AT_BD, // Read / write the "baud rate" parameter [BD]
    AT_NB, // Read / write the "parity" parameter [BD]
    AT_AC, // Apply changes and leave command mode
    AT_WR, // Write in flash memory and leave command mode
    AT_CN, // Leave command mode
    AT_NR, // Reset the Xbee
    NUMBER_OF_PARAMETER_AT_COMMANDS
};

/* Enumerative with the error codes generated when analyzing an AT command    */
enum at_command_analysis_error_code_e{
    AT_CMD_ERROR_TOO_SHORT = -6,
    AT_CMD_ERROR_TOO_LONG = -5,
    AT_CMD_ERROR_WRONG_PREFIX = -4,
    AT_CMD_ERROR_NOT_SUPPORTED_READ_CMD = -3,
    AT_CMD_ERROR_WRITE_DATA_NOT_VALID = -2,
    AT_CMD_ERROR_NOT_SUPPORTED_WRITE_CMD = -1,
    AT_CMD_OK_STAY_IN_CMD_MODE = 0,
    AT_CMD_OK_LEAVE_CMD_MODE = 1
};

/* Xbee parameters                                                            */
struct xbee_parameters_t {
    uint16_t at_vr; // Xbee's FW version (read only)
    uint16_t at_hv; // Xbee's HW version (read only)
    uint32_t at_sh; // High part of the MAC address (read only)
    uint32_t at_sl; // Low part of the MAC address (read only)
    uint8_t at_jv;  // Coordinator join verification parameter
    uint8_t at_nj;  // Node join time parameter
    uint16_t at_nw; // Network watchdog parameter
    uint64_t at_id; // Extended pan id parameter
    uint8_t at_ce;   // Coordinator enabled parameter
    uint8_t at_ai;   // Association indication parameter (read only)
    uint8_t at_ch;   // Operation channel parameter (read only)
    uint16_t at_my;   // Short address parameter (read only)
    uint8_t at_ee;   // Encryption enable parameter
    uint8_t at_eo;   // Encryption options parameter
    uint8_t at_ky[MAXIMUM_SIZE_LINK_KEY];   // Link Encryption Key parameter
    uint8_t at_zs;   // Xbee's Zigbee stack profile parameter
    uint8_t at_bd;   // Xbee's UART baud rate parameter
    uint8_t at_nb;   // Xbee's UART parity parameter
    uint8_t at_ni[MAXIMUM_SIZE_NODE_IDENTIFIER + 1];   // Node identifier string parameter (plus one to include the '\0')
};

/* Function prototypes used only internally                                   */
void digi_at_init_xbee_parameters(void);
void digi_at_init_xbee_parameter_command(void);
int8_t digi_at_read_ni(uint8_t* buffer);
int8_t digi_at_read_ky(uint8_t* buffer);
void digi_at_reply_read_command(uint8_t at_command);
void digi_at_reply_action_command(uint8_t at_command);
bool digi_at_reply_write_command(uint8_t at_command, const char *command_data_string, uint8_t string_size);
bool convert_hex_string_to_uint64(const char *hex_string, uint8_t string_size, uint64_t *output_result);
void ascii_to_hex(const char *ascii, uint8_t *hex, size_t hex_len);


/* Function prototypes used externally                                        */
void digi_at_init(void);
void digi_at_reply_ok(void);
void digi_at_reply_error(void);
int8_t digi_at_analyze_and_reply_to_command(uint8_t *input_data, uint16_t size_input_data);
uint64_t digi_at_get_parameter_id(void);
void digi_at_get_parameter_ni(uint8_t *ni);
void digi_at_get_parameter_ky(uint8_t *ky);

#endif /* DIGI_AT_COMMANDS_H_ */







