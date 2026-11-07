// BridgeHub: the S3's own USB port as a USB host for one serial unit,
// served raw on a TCP port (hub.c).
//
// This file may be distributed under the terms of the GNU GPLv3 license.
#pragma once

/**
 * Installs the USB host and starts the hub's tasks on core 0. Called once,
 * after the network is up.
 */
void hub_start(void);
