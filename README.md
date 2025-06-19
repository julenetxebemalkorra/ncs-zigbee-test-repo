# Zigbee Router Firmware for nRF52840

This repository contains the embedded firmware for a custom Zigbee Router based on the Nordic nRF52840 SoC. The project is built on **Zephyr RTOS** and uses the **ZBOSS Zigbee stack** to enable advanced features like:

- Zigbee routing and APS-level communication
- AT command interface (UART and Zigbee)
- FUOTA (Firmware Upgrade Over-The-Air) support using APS transport
- Persistent configuration via Zephyr NVS
- Node discovery handling for XBee/Digi compatibility

---

## 📦 Features

- **UART Interface**: Command and transparent modes compatible with Digi AT syntax
- **Zigbee APS Handling**: Supports custom profile for AT and FOTA clusters
- **Firmware Updates**: Uses `dfu_target_mcuboot` for secure image application
- **State Machines**: For FUOTA and AT command processing
- **Zigbee Network Configuration**: Stored in flash via Zephyr’s NVS APIs

---

## Requirements

- Zephyr RTOS (tested with nRF Connect SDK v2.9.1)
- Nordic nRF52840 SoC
- MCUboot DFU support enabled
- ZBOSS Zigbee stack integration
- VS Code with West & Zephyr tooling configured

## 🗂️ Directory Structure

```text
├── main.c                        # Application entry point
├── zigbee_configuration.c        # Zigbee stack & network config
├── Tcu_Uart.c                    # UART reception/transmission logic
├── Digi_At_commands.c            # AT command handler (UART)
├── Digi_wireless_at_commands.c   # AT command handler (Zigbee APS)
├── Digi_node_discovery.c         # XBee ND response
├── zigbee_aps.c                  # APS layer transmission queue
├── OTA_dfu_target.c              # Low-level DFU write interface
├── Digi_fota.c                   # FUOTA state machine manager
├── nvram.c                       # Persistent flash-backed parameter storage
├── system.c                      # Watchdog, GPIO, timer utilities
├── include/                      # Common headers
└── doc/                          # Doxygen documentation output (ignored in Git)
