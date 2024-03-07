/*
 * Copyright (c) 2024 IED
 *
 */

#ifndef DIGI_AT_COMMANDS_H_
#define DIGI_AT_COMMANDS_H_

#define MINIMUM_SIZE_AT_COMMAND 4
#define MAXIMUM_SIZE_AT_COMMAND 20 //Write command with a data of 16 bytes
#define MAXIMUM_SIZE_NODE_IDENTIFIER 16

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
    AT_ZS, // Read / write the "Zigbee stack profile" parameter [ZS]
    AT_BD, // Read / write the "baud rate" parameter [BD]
    AT_NB, // Read / write the "parity" parameter [BD]
    NUMBER_OF_PARAMETER_AT_COMMANDS
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
    uint8_t at_zs;   // Xbee's Zigbee stack profile parameter
    uint8_t at_bd;   // Xbee's UART baud rate parameter
    uint8_t at_nb;   // Xbee's UART parity parameter
    uint8_t at_ni[MAXIMUM_SIZE_NODE_IDENTIFIER + 1];   // Node identifier string parameter (plus one to include the '\0')
};

 struct xbee_parameter_comando_at_t {
    uint8_t first_char;        // First char of AT command
    uint8_t second_char;       // Second char of AT command
    uint8_t *data;             // Pointer to data associated to that command
    uint8_t size_of_data;      // Size of data (number of bytes)
    bool    b_numeric_data;    // If true, data is a numeric value. If false, is a string of characters.
    bool    b_read_only;       // If true, it is a read only parameter. If false, it can be written.
};

/* Function prototypes used only internally                                   */
void digi_at_init_xbee_parameters(void);
void digi_at_init_xbee_parameter_command(void);
int8_t digi_at_read_ni(uint8_t* buffer);
void digi_at_reply_read_command(uint8_t at_command);
bool digi_at_reply_write_command(uint8_t at_command, const char *command_data_string, uint8_t string_size);
bool convert_hex_string_to_uint64(const char *hex_string, uint8_t string_size, uint64_t *output_result);

/* Function prototypes used externally                                        */
void digi_at_init(void);
void digi_at_reply_ok(void);
void digi_at_reply_error(void);
int8_t digi_at_analyze_and_reply_to_command(uint8_t *input_data, uint16_t size_input_data);

#endif /* DIGI_AT_COMMANDS_H_ */







