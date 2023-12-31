/*
 *  at86rf215-driver: OS-independent driver for the AT86RF215 transceiver
 *
 *  Copyright (C) 2020, Libre Space Foundation <http://libre.space>
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __AT86RF215_BB_H__
#define __AT86RF215_BB_H__

#include <stdint.h>
#include <stdlib.h>
#include "../io_utils/io_utils.h"
#include "../spi/spi.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * AT86RF215 driver error codes
 */
typedef enum {
	AT86RF215_OK = 0,        	//!< No error
	AT86RF215_INVAL_PARAM,   	//!< Invalid parameter
	AT86RF215_NO_INIT,       	//!< The IC has not been initialized
	AT86RF215_NOT_IMPL,      	//!< The requested configuration is not implemented
	AT86RF215_INVAL_VAL,     	//!< Invalid value
	AT86RF215_UNKNOWN_IC,    	//!< The IC is not valid
	AT86RF215_NOT_SUPPORTED, 	//!< The requested configuration/mode is not supported
	AT86RF215_INVAL_CONF,    	//!< The requested configuration is invalid
	AT86RF215_INVAL_CHPM,    	//!< Invalid chip mode for the requested operation
	AT86RF215_SPI_READ_FAILED,  //!< SPI Read Failed
	AT86RF215_SPI_WRITE_FAILED, //!< SPI Write Failed
} at86rf215_error_t;

/**
 * Device Family
 */
typedef enum {
	AT86RF215   = 0x34, //!< Dual band transceiver and I/Q radio
	AT86RF215IQ = 0x35, //!< I/Q radio
	AT86RF215M  = 0x36  //!< Sub-1GHz Transceiver and I/Q radio
} at86rf215_family_t;

/**
 * Radio front-end
 */
typedef enum {
	AT86RF215_RF09 = 0, //!< Sub-1 GHz radio
	AT86RF215_RF24 = 1, //!< 2.4 GHz radio
} at86rf215_radio_t;

/**
 * CLKO pad driver strength
 */
typedef enum {
	AT86RF215_RF_DRVCLKO2 = 0, //!< 2 mA
	AT86RF215_RF_DRVCLKO4,     //!< 4 mA
	AT86RF215_RF_DRVCLKO6,     //!< 6 mA
	AT86RF215_RF_DRVCLKO8      //!< 8 mA
} at86rf215_rf_clko_drv_t;

/**
 * CLKO output selection
 */
typedef enum {
	AT86RF215_RF_CLKO_OFF = 0,
	AT86RF215_RF_CLKO_26_MHZ,
	AT86RF215_RF_CLKO_32_MHZ,
	AT86RF215_RF_CLKO_16_MHZ,
	AT86RF215_RF_CLKO_8_MHZ,
	AT86RF215_RF_CLKO_4_MHZ,
	AT86RF215_RF_CLKO_2_MHZ,
	AT86RF215_RF_CLKO_1_MHZ,
} at86rf215_rf_clko_os_t;

/**
 * External Frontend Control
 */
typedef enum {
	AT86RF215_RF_FEMODE0 = 0,
	AT86RF215_RF_FEMODE1,
	AT86RF215_RF_FEMODE2,
	AT86RF215_RF_FEMODE3,
} at86rf215_external_rffe_control_t;

typedef enum {
	AT86RF215_STATE_RF_TRXOFF     = 0x2,
	AT86RF215_STATE_RF_TXPREP     = 0x3,
	AT86RF215_STATE_RF_TX         = 0x4,
	AT86RF215_STATE_RF_RX         = 0x5,
	AT86RF215_STATE_RF_TRANSITION = 0x6,
	AT86RF215_STATE_RF_RESET      = 0x7
} at86rf215_rf_state_t;

/**
 * Radio command affecting the FSM
 */
typedef enum {
	AT86RF215_CMD_RF_NOP    = 0x0, //!< AT86RF215_CMD_RF_NOP
	AT86RF215_CMD_RF_SLEEP  = 0x1, //!< AT86RF215_CMD_RF_SLEEP
	AT86RF215_CMD_RF_TRXOFF = 0x2, //!< AT86RF215_CMD_RF_TRXOFF
	AT86RF215_CMD_RF_TXPREP = 0x3, //!< AT86RF215_CMD_RF_TXPREP
	AT86RF215_CMD_RF_TX     = 0x4, //!< AT86RF215_CMD_RF_TX
	AT86RF215_CMD_RF_RX     = 0x5, //!< AT86RF215_CMD_RF_RX
	AT86RF215_CMD_RF_RESET  = 0x7  //!< AT86RF215_CMD_RF_RESET
} at86rf215_rf_cmd_t;

/**
 * Chip mode
 */
typedef enum {
	AT86RF215_RF_MODE_BBRF =
	        0x0, //!< RF enabled, baseband (BBC0, BBC1) enabled, I/Q IF disabled
	AT86RF215_RF_MODE_RF =
	        0x1, //!< RF enabled, baseband (BBC0, BBC1) disabled, I/Q IF enabled
	AT86RF215_RF_MODE_BBRF09 =
	        0x4, //!< RF enabled, baseband (BBC0) disabled and (BBC1) enabled, I/Q IF for sub-GHz Transceiver enabled
	AT86RF215_RF_MODE_BBRF24 =
	        0x5 //!< RF enabled, baseband (BBC1) disabled and (BBC0) enabled, I/Q IF for 2.4GHz Transceiver enabled
} at86rf215_chpm_t;

/**
 * Ramp up/down time of the power amplifier
 */
typedef enum {
	AT86RF215_RF_PARAMP4U  = 0x0, //!< 4 us
	AT86RF215_RF_PARAMP8U  = 0x1, //!< 8 us
	AT86RF215_RF_PARAMP16U = 0x2, //!< 16 us
	AT86RF215_RF_PARAMP32U = 0x3, //!< 32 us
} at86rf215_paramp_t;

/**
 * Low pass filter configuration
 */
typedef enum {
	AT86RF215_RF_FLC80KHZ   = 0x0,
	AT86RF215_RF_FLC100KHZ  = 0x1,
	AT86RF215_RF_FLC125KHZ  = 0x2,
	AT86RF215_RF_FLC160KHZ  = 0x3,
	AT86RF215_RF_FLC200KHZ  = 0x4,
	AT86RF215_RF_FLC250KHZ  = 0x5,
	AT86RF215_RF_FLC315KHZ  = 0x6,
	AT86RF215_RF_FLC400KHZ  = 0x7,
	AT86RF215_RF_FLC500KHZ  = 0x8,
	AT86RF215_RF_FLC624KHZ  = 0x9,
	AT86RF215_RF_FLC800KHZ  = 0xA,
	AT86RF215_RF_FLC1000KHZ = 0xB
} at86rf215_lpfcut_t;

typedef enum {
	AT86RF215_RF_BW160KHZ_IF250KHZ   = 0,
	AT86RF215_RF_BW200KHZ_IF250KHZ   = 0x1,
	AT86RF215_RF_BW250KHZ_IF250KHZ   = 0x2,
	AT86RF215_RF_BW320KHZ_IF500KHZ   = 0x3,
	AT86RF215_RF_BW400KHZ_IF500KHZ   = 0x4,
	AT86RF215_RF_BW500KHZ_IF500KHZ   = 0x5,
	AT86RF215_RF_BW630KHZ_IF1000KHZ  = 0x6,
	AT86RF215_RF_BW800KHZ_IF1000KHZ  = 0x7,
	AT86RF215_RF_BW1000KHZ_IF1000KHZ = 0x8,
	AT86RF215_RF_BW1250KHZ_IF2000KHZ = 0x9,
	AT86RF215_RF_BW1600KHZ_IF2000KHZ = 0xA,
	AT86RF215_RF_BW2000KHZ_IF2000KHZ = 0xB
} at86rf215_rx_bw_t;

/**
 * Sampling rate configuration
 */
typedef enum {
	AT86RF215_SR_4000KHZ = 0x1,
	AT86RF215_SR_2000KHZ = 0x2,
	AT86RF215_SR_1333KHZ = 0x3,
	AT86RF215_SR_1000KHZ = 0x4,
	AT86RF215_SR_800KHZ  = 0x5,
	AT86RF215_SR_666KHZ  = 0x6,
	AT86RF215_SR_500KHZ  = 0x8,
	AT86RF215_SR_400KHZ  = 0xA,
} at86rf215_sr_t;

/**
 * Power amplifier current settings
 */
typedef enum {
	AT86RF215_PACUR_22mA_RED = 0x0, //!< current reduction by about 22mA (3dB reduction of max. small signal gain)
	AT86RF215_PACUR_18mA_RED = 0x1, //!< current reduction by about 18mA (2dB reduction of max. small signal gain)
	AT86RF215_PACUR_11mA_RED = 0x2, //!< current reduction by about 11mA (1dB reduction of max. small signal gain)
	AT86RF215_PACUR_NO_RED 	 = 0x3, //!< No power amplifier current reduction (max. transmit small signal gain)
} at86rf215_pacur_t;

/**
 * Channel mode configuration
 */
typedef enum {
	AT86RF215_CM_IEEE 		 = 0, //!< IEEE compliant
	AT86RF215_CM_FINE_RES_04 = 1, //!< Fine resolution (389.5-510.0)MHz with 99.182Hz channel stepping
	AT86RF215_CM_FINE_RES_09 = 2, //!< Fine resolution (779-1020)MHz with 198.364Hz channel stepping
	AT86RF215_CM_FINE_RES_24 = 3, //!< Fine resolution (2400-2483.5)MHz with 396.728Hz channel stepping
} at86rf215_cm_t;

/**
 * PLL loop bandwidth setting
 */
typedef enum {
	AT86RF215_PLL_LBW_DEFAULT = 0, //!< Default
	AT86RF215_PLL_LBW_SMALLER = 1, //!< 15% smaller PLL loop bandwidth
	AT86RF215_PLL_LBW_LARGER  = 2, //!< 15% larger PLL loop bandwidth
} at86rf215_pll_lbw_t;

/**
 * PLL lock status
 */
typedef enum {
	AT86RF215_PLL_UNLOCKED = 0, //!< PLL is not locked
	AT86RF215_PLL_LOCKED   = 1, //!< PLL is locked
} at86rf215_pll_ls_t;

/**
 * Output pads (IRQ, MISO,  FEA09, FEB09, FEA24, FEB24)) driving current strength
 */
typedef enum {
	AT86RF215_RF_DRV2 = 0, //!< 2 mA
	AT86RF215_RF_DRV4 = 1, //!< 4 mA
	AT86RF215_RF_DRV6 = 2, //!< 6 mA
	AT86RF215_RF_DRV8 = 3, //!< 8 mA
} at86rf215_drv_t;

/**
 * LVDS driving current strength
 */
typedef enum {
	AT86RF215_LVDS_DRV1 = 0, //!< 1 mA
	AT86RF215_LVDS_DRV2 = 1, //!< 2 mA
	AT86RF215_LVDS_DRV3 = 2, //!< 3 mA
	AT86RF215_LVDS_DRV4 = 3, //!< 4 mA
} at86rf215_lvds_drv_t;

/**
 * LVDS common mode voltage
 */
typedef enum {
	AT86RF215_LVDS_CMV150 = 0, //!< 150 mV
	AT86RF215_LVDS_CMV200 = 1, //!< 200 mV
	AT86RF215_LVDS_CMV250 = 2, //!< 250 mV
	AT86RF215_LVDS_CMV300 = 3, //!< 300 mV
} at86rf215_lvds_cmv_t;

struct at86rf215_radio_conf {
	at86rf215_cm_t      cm;
	uint32_t            cs;
	uint32_t            base_freq;
	at86rf215_pll_lbw_t lbw;
};

/**
 * Physical layer type of the baseband core
 */
typedef enum {
	AT86RF215_BB_PHYOFF  = 0, //!< BB core is OFF
	AT86RF215_BB_MRFSK   = 1, //!< MR-FSK
	AT86RF215_BB_MROFDM  = 2, //!< MR-OFDM
	AT86RF215_BB_MROQPSK = 3  //!< MR-OQPSK
} at86rf215_phy_t;

/**
 * FCS type
 */
typedef enum {
	AT86RF215_FCS_32 = 0, //!< 32-bit
	AT86RF215_FCS_16 = 1  //!< 16-bit
} at86rf215_fcs_t;

/**
 * FSK 3-dB bandwidth of the shaping filter
 */
typedef enum {
	AT86RF215_FSK_BT_05 = 0, //!< BT = 0.5
	AT86RF215_FSK_BT_10 = 1, //!< BT = 1.0
	AT86RF215_FSK_BT_15 = 2, //!< BT = 1.5
	AT86RF215_FSK_BT_20 = 3, //!< BT = 2.0
} at86rf215_fsk_bt_t;

/**
 * Scaling factor with regard to the modulation index.
 * The effectively applied modulation index results in the modulation index
 * configured with MIDX times the scaling factor.
 */
typedef enum {
	AT86RF215_MIDXS_78  = 0, //!< s = 1.0 - 1/8 (7/8)
	AT86RF215_MIDXS_88  = 1, //!< s = 1.0 (8/8)
	AT86RF215_MIDXS_98  = 2, //!< s = 1.0 + 1/6 (9/8)
	AT86RF215_MIDXS_108 = 3, //!< s = 1.0 + 1/4 (10/8)
} at86rf215_fsk_midxs_t;

/**
 * Modulation index
 */
typedef enum {
	AT86RF215_MIDX_0 = 0, //!< Modulation index = 0.375
	AT86RF215_MIDX_1 = 1, //!< Modulation index = 0.5
	AT86RF215_MIDX_2 = 2, //!< Modulation index = 0.75
	AT86RF215_MIDX_3 = 3, //!< Modulation index = 1.0
	AT86RF215_MIDX_4 = 4, //!< Modulation index = 1.25
	AT86RF215_MIDX_5 = 5, //!< Modulation index = 1.5
	AT86RF215_MIDX_6 = 6, //!< Modulation index = 1.75
	AT86RF215_MIDX_7 = 7, //!< Modulation index = 2.0
} at86rf215_fsk_midx_t;

/**
 * FSK type
 */
typedef enum {
	AT86RF215_2FSK = 0, //!< 2-FSK
	AT86RF215_4FSK = 1, //!< 4-FSK
} at86rf215_fsk_t;

/**
 * FSK symbol rates
 */
typedef enum {
	AT86RF215_FSK_SRATE_50  = 0, //!< 50 kHz
	AT86RF215_FSK_SRATE_100 = 1, //!< 100 kHz
	AT86RF215_FSK_SRATE_150 = 2, //!< 150 kHz
	AT86RF215_FSK_SRATE_200 = 3, //!< 200 kHz
	AT86RF215_FSK_SRATE_300 = 4, //!< 300 kHz
	AT86RF215_FSK_SRATE_400 = 5, //!< 400 kHz
} at86rf215_fsk_srate_t;

/**
 * Receiver override setting.
 * If Receiver Override is enabled during frame reception,
 * the FSK receiver restarts synchronization after a rapid input signal increase.
 * The minimum input signal level is -110dBm.
 */
typedef enum {
	AT86RF215_FSK_RXO_6DB  	   = 0,    //!< Receiver restarted by >6dB stronger frame
	AT86RF215_FSK_RXO_12DB 	   = 1,    //!< Receiver restarted by >12dB stronger frame
	AT86RF215_FSK_RXO_18DB 	   = 2,    //!< Receiver restarted by >18dB stronger frame
	AT86RF215_FSK_RXO_DISABLED = 3,	   //!< Receiver override disabled
} at86rf215_fsk_rxo_t;

/**
 * FSK FEC Scheme
 */
typedef enum {
	AT86RF215_FSK_FEC_NRNSC = 0, //!< non-recursive and non-systematic convolutional cod
	AT86RF215_FSK_FEC_RSC 	= 1, //!< recursive and systematic convolutional code
} at86rf215_fsk_fecs_t;

/**
 * SFD condifuration
 */
typedef enum {
	AT86RF215_SFD_UNCODED_IEEE = 0, //!< Uncoded IEEE mode
	AT86RF215_SFD_UNCODED_RAW  = 1, //!< Uncoded RAW mode
	AT86RF215_SFD_CODED_IEEE   = 2, //!< Coded IEEE mode
	AT86RF215_SFD_CODED_RAW    = 3, //!< Coded RAW mode
} at86rf215_sfd_mode_t;

/**
 *  Energy Measurement
 */
typedef enum{
	AT86RF215_EDD_DTB_2US	= 0, //!< 2 micro seconds
	AT86RF215_EDD_DTB_8US	= 1, //!< 8 micro seconds
	AT86RF215_EDD_DTB_32US	= 2, //!< 32 micro seconds
	AT86RF215_EDD_DTB_128US	= 3, //!< 128 micro seconds
} at86rf215_edd_dtb_t;

/**
 *  TX Modulation and coding scheme
 */
typedef enum{
	AT86RF215_BPSK_1_2_N_4_FREQ_REP	= 0, //!< BPSK, coding rate 1/2, 4 x frequency repetition
	AT86RF215_BPSK_1_2_N_2_FREQ_REP	= 1, //!< BPSK, coding rate 1/2, 2 x frequency repetition
	AT86RF215_QPSK_1_2_N_2_FREQ_REP	= 2, //!< QPSK, coding rate 1/2, 2 x frequency repetition
	AT86RF215_QPSK_1_2				= 3, //!< QPSK, coding rate 1/2
	AT86RF215_QPSK_3_4				= 4, //!< QPSK, coding rate 3/4
	AT86RF215_16_QAM_1_2			= 5, //!< 16-QAM, coding rate 1/2
	AT86RF215_16_QAM_3_4			= 6, //!< 16-QAM, coding rate 3/4
} at86rf215_ofdm_tx_mcs_t;

/**
 *  Receiver Scrambler Seed Status
 */
typedef enum{
	AT86RF215_SSRX_0 = 0, //!< 0b000010111
	AT86RF215_SSRX_1 = 1, //!< 0b101110111
	AT86RF215_SSRX_2 = 2, //!< 0b000011100
	AT86RF215_SSRX_3 = 3, //!< 0b101111100
} at86rf215_ofdm_ssrx_t;

/**
 *  Transmitter Scrambler Seed Configuration
 */
typedef enum{
	AT86RF215_SSTX_0 = 0, //!< 0b000010111
	AT86RF215_SSTX_1 = 1, //!< 0b101110111
	AT86RF215_SSTX_2 = 2, //!< 0b000011100
	AT86RF215_SSTX_3 = 3, //!< 0b101111100
} at86rf215_ofdm_sstx_t;

/**
 *  MR-OFDM Bandwidth Option
 */
typedef enum{
	AT86RF215_OPT_1 = 0, //!< Option 1
	AT86RF215_OPT_2 = 1, //!< Option 2
	AT86RF215_OPT_3 = 2, //!< Option 3
	AT86RF215_OPT_4 = 3, //!< Option 4
} at86rf215_ofdm_opt_t;

typedef enum{
	at86rf215_cs_1 = 0,
	at86rf215_cs_2 = 1,
	at86rf215_cs_both = 2,
} at86rf215_chip_select_t;

/**
 *
 */
struct at86rf215_mrfsk_conf {
	at86rf215_fsk_t       mord;
	at86rf215_fsk_midx_t  midx;
	at86rf215_fsk_midxs_t midxs;
	at86rf215_fsk_bt_t    bt; //!< Bandwidth time product
	at86rf215_fsk_srate_t srate;
	uint8_t               fi              : 1;
	uint16_t              preamble_length : 10;
	at86rf215_fsk_rxo_t   rxo;
	uint8_t               pdtm  : 1;
	uint8_t               rxpto : 1;
	uint8_t               mse   : 1;
	uint8_t               pri   : 1;
	uint8_t               fecie : 1;
	at86rf215_fsk_fecs_t  fecs;
	uint8_t               sfd_threshold      : 4;
	uint8_t               preamble_threshold : 4;
	uint8_t               sfdq               : 1;
	uint8_t               sfd32              : 1;
	uint8_t               rawrbit            : 1;
	at86rf215_sfd_mode_t  csfd1;
	at86rf215_sfd_mode_t  csfd0;
	uint16_t              sfd1;
	uint16_t              sfd0;
	uint8_t               sfd : 1;
	uint8_t               dw  : 1;
	uint8_t               rb2 : 1;
	uint8_t               rb1 : 1;
	uint8_t               dm;
	uint8_t               preemphasis      : 1;
	uint32_t              preemphasis_taps : 24;
};

/**
 *
 */
struct at86rf215_mrofdm_conf {
	at86rf215_ofdm_tx_mcs_t	mcs;
	at86rf215_ofdm_ssrx_t 	ssrx;
	at86rf215_ofdm_sstx_t 	sstx;
	uint8_t 				lfo	: 1;
	uint8_t					poi	: 1;
	at86rf215_ofdm_opt_t  	opt;
	uint8_t					pdt : 3;
	uint8_t					rxo : 1;
};

struct at86rf215_bb_conf {
	uint8_t         ctx    : 1;
	uint8_t         fcsfe  : 1;
	uint8_t         txafcs : 1;
	at86rf215_fcs_t fcst;
	at86rf215_phy_t pt;
	union {
		struct at86rf215_mrfsk_conf 	fsk;
		struct at86rf215_mrofdm_conf	ofdm;
	};
};

/**
 * Configuration parameters for IQ operational mode
 */
struct at86rf215_iq_conf {
	uint8_t              extlb : 1;  /**< Set 1 to enable the external loopback */
	at86rf215_lvds_drv_t drv;        /**< Driving current of the LVDS pins */
	at86rf215_lvds_cmv_t cmv;        /**< LVDS common mode voltage */
	uint8_t              cmv1v2 : 1; /**<
	                   Set one to enable the 1.2V common mode compliant
	                   to the IEEE STD 1596. In this case the cmv field
	                   is not taken into account */
	uint8_t              eec    : 1; /**< Enable/disable embedded control */
	uint8_t        skedrv : 2; /**< Skew allignment I/Q IF driver. Refer to the
	                   datasheet for the exact values of this field.
	                   Default value is 0x2. */
	at86rf215_sr_t tsr;        /**< TX sampling rate of the IQ interface */
	uint8_t        trcut : 3;  /**< TX relative cuttof frequency. Refer to the
	                   datasheet for the exact values */
	at86rf215_sr_t rsr;        /**< RX sampling rate of the IQ interface */
	uint8_t        rrcut : 3;  /**< RX relative cuttof frequency. Refer to the
	                   datasheet for the exact values */
};

/**
 * Operational state of the IC for a specific radio frontend.
 * @note These states are for internal use and do not correspond in any way to
 * the at86rf215_rf_state_t
 */
typedef enum {
	AT86RF215_OP_STATE_IDLE, //!< IDLE
	AT86RF215_OP_STATE_TX,   //!< TX operation is active
	AT86RF215_OP_STATE_RX    //!< RX operation is active
} at86rf215_op_state_t;

typedef struct
{
    int low_ch_i;
    int low_ch_q;
    int hi_ch_i;
    int hi_ch_q;
} at86rf215_cal_st;

typedef struct 
{
    pthread_mutex_t ready_mutex;
    pthread_cond_t ready_cond;
    int ready;
} at86rf215_event_st;

struct at86rf215_radio {
	uint32_t             init;
	at86rf215_cm_t       cm;
	uint8_t              cs_reg;
	uint32_t             cs;
	uint32_t             base_freq;
	at86rf215_op_state_t op_state;
	at86rf215_event_st   trxready;
};

/**
 * Private members of the at86rf215. Should not be accessed directly by the user
 */
struct at86rf215_priv {
	uint32_t                 init;
	uint8_t                  version;
	at86rf215_family_t       family;
	at86rf215_chpm_t         chpm;
	struct at86rf215_radio   radios[2];
	struct at86rf215_bb_conf bbc[2];
};

struct at86rf215 {
	at86rf215_rf_clko_os_t            clko_os;
	at86rf215_rf_clko_drv_t           clk_drv;
	at86rf215_external_rffe_control_t rf_femode_09;
	at86rf215_external_rffe_control_t rf_femode_24;
	uint8_t                           xo_fs   : 1;
	uint8_t                           xo_trim : 4;
	uint8_t                           irqmm   : 1;
	uint8_t                           irqp    : 1;
	at86rf215_drv_t                   pad_drv;
	struct at86rf215_priv             priv;
	at86rf215_cal_st				  cal;
    bool 							  override_cal;
	pthread_t						  irq_tid;
	gpio_interrupt_t*				  irq_data;
	at86rf215_chip_select_t			  chip_select;
	spi_t							  spidev;
};

int ready(const struct at86rf215 *h);

void event_node_init(at86rf215_event_st* ev);

void event_node_close(at86rf215_event_st* ev);

void event_node_wait_ready(at86rf215_event_st* ev);

void event_node_signal_ready(at86rf215_event_st* ev, int ready);

int at86rf215_init(struct at86rf215 *h, spi_t *spidev);

int at86rf215_radio_conf(struct at86rf215 *h, at86rf215_radio_t radio, const struct at86rf215_radio_conf *conf);

int at86rf215_spi_read(const struct at86rf215 *h, uint8_t *out, const uint8_t *in, size_t len);

int at86rf215_spi_write(const struct at86rf215 *h, const uint8_t *in, size_t len);

int at86rf215_reg_read_8(const struct at86rf215 *h, uint8_t *out, uint16_t reg);

int at86rf215_reg_write_8(const struct at86rf215 *h, const uint8_t in, uint16_t reg);

int at86rf215_get_state(const struct at86rf215 *h, at86rf215_rf_state_t *state, at86rf215_radio_t radio);

int at86rf215_set_cmd(struct at86rf215 *h, at86rf215_rf_cmd_t cmd, at86rf215_radio_t radio);

int at86rf215_set_mode(struct at86rf215 *h, at86rf215_chpm_t mode);

int at86rf215_transceiver_reset(struct at86rf215 *h, at86rf215_radio_t radio);

int at86rf215_set_bbc_irq_mask(struct at86rf215 *h, at86rf215_radio_t radio, uint8_t mask);

int at86rf215_set_radio_irq_mask(struct at86rf215 *h, at86rf215_radio_t radio, uint8_t mask);

int at86rf215_set_txcutc(struct at86rf215 *h, at86rf215_radio_t radio, at86rf215_paramp_t paramp, at86rf215_lpfcut_t lpf);

int at86rf215_set_rxdfe(struct at86rf215 *h, at86rf215_radio_t radio, uint8_t rcut, at86rf215_sr_t sr);

int at86rf215_set_txdfe(struct at86rf215 *h, at86rf215_radio_t radio, uint8_t rcut, uint8_t dm, at86rf215_sr_t sr);

int at86rf215_set_pac(struct at86rf215 *h, at86rf215_radio_t radio, at86rf215_pacur_t pacur, uint8_t power);

int at86rf215_set_channel(struct at86rf215 *h, at86rf215_radio_t radio, uint16_t channel);

int at86rf215_set_freq(struct at86rf215 *h, at86rf215_radio_t radio, uint32_t freq);

int at86rf215_get_pll_ls(const struct at86rf215 *h, at86rf215_pll_ls_t *status, at86rf215_radio_t radio);

int at86rf215_set_bw(struct at86rf215 *h, at86rf215_radio_t radio, uint8_t if_inv, uint8_t if_shift, at86rf215_rx_bw_t bw);

int at86rf215_set_edd(struct at86rf215 *h, at86rf215_radio_t radio, uint8_t df, at86rf215_edd_dtb_t dtb);

int at86rf215_get_rssi(struct at86rf215 *h, at86rf215_radio_t radio, float *rssi);

int at86rf215_get_edv(struct at86rf215 *h, at86rf215_radio_t radio, float *edv);

int at86rf215_get_agc_gain(struct at86rf215 *h, at86rf215_radio_t radio, uint8_t *gain);

int at86rf215_irq_callback(int event, unsigned int line_offset, const struct timespec * time, void *data);

int at86rf215_irq_user_callback(const struct at86rf215 *h, uint8_t rf09_irqs, uint8_t rf24_irqs, uint8_t bbc0_irqs, uint8_t bbc1_irqs);

int at86rf215_bb_conf(struct at86rf215 *h, at86rf215_radio_t radio, const struct at86rf215_bb_conf *conf);

int at86rf215_bb_enable(struct at86rf215 *h, at86rf215_radio_t radio, uint8_t en);

int at86rf215_tx_frame(struct at86rf215 *h, at86rf215_radio_t radio, const uint8_t *psdu, size_t len);

int at86rf215_tx_frame_auto_mode(struct at86rf215 *h, at86rf215_radio_t radio, const uint8_t *psdu, size_t len);

int at86rf215_iq_conf(struct at86rf215 *h, at86rf215_radio_t radio, const struct at86rf215_iq_conf *conf);

int at86rf215_get_irq_mask(const struct at86rf215 *h, uint8_t *mask, at86rf215_radio_t radio);

int at86rf215_calibrate_device(struct at86rf215 *h, at86rf215_radio_t radio, int* i, int* q);

void at86rf215_radio_set_tx_iq_calibration(const struct at86rf215 *h, at86rf215_radio_t radio, int cal_i, int cal_q);

void at86rf215_radio_get_tx_iq_calibration(const struct at86rf215 *h, at86rf215_radio_t radio, int *cal_i, int *cal_q);


int evt_at86rf215_reg_read_8(const struct at86rf215 *h, uint8_t *out, uint16_t reg);

int evt_at86rf215_reg_write_8(const struct at86rf215* h, const uint8_t in, uint16_t reg);

#ifdef __cplusplus
}
#endif

#endif /* __AT86RF215_BB_H__ */
