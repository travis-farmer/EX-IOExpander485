/*
 *  © 2022 Peter Cole. All rights reserved.
 *  © 2024, Travis Farmer. All rights reserved.
 * 
 *  This is the example configuration file for EX-IOExpander485.
 * 
 *  It is highly recommended to copy this to "config.h" and modify to suit your specific
 *  requirements.
 * 
 *  NOTE: Modifications to this file will be overwritten by future software updates.
 */
#ifndef MYCONFIG_H
#define MYCONFIG_H

/////////////////////////////////////////////////////////////////////////////////////
//  Define RS485 Node
//  Default 1, can be any valid, available node from 0 to 254, but must not be the same
//  for more than one node.
// 
#define RS485_NODE 1

/////////////////////////////////////////////////////////////////////////////////////
//  Uncomment to enable diag output
// 
// #define DIAG

/////////////////////////////////////////////////////////////////////////////////////
//  Delay between dumping the status of the port config if DIAG enabled
// 
#define DIAG_CONFIG_DELAY 5

/////////////////////////////////////////////////////////////////////////////////////
//  Enable test mode - ensure only one test mode is active at one time.
//  This is handy if serial input doesn't work for commands for some reason. 
// 
//  ANALOGUE - equivalent of <T A>
//  INPUT - equivalent of <T I>
//  OUTPUT - equivalent of <T O>
//  PULLUP - equivalent of <T P>
// 
// #define TEST_MODE ANALOGUE_TEST
// #define TEST_MODE INPUT_TEST
// #define TEST_MODE OUTPUT_TEST
// #define TEST_MODE PULLUP_TEST

#endif