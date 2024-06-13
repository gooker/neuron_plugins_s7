/**
 * NEURON IIoT System for Industry 4.0
 * Copyright (C) 2020-2022 EMQ Technologies Co., Ltd All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 **/
#ifndef _NEU_M_PLUGIN_S7_H_
#define _NEU_M_PLUGIN_S7_H_

#include <stdbool.h>
#include <stdint.h>

#include <neuron.h>
#include "snap7_isotcp.h"
#include "snap7_types.h"

/* LL -> LE 1,2,3,4
   LB -> LE 2,1,4,3
   BB -> BE 3,4,1,2
   BL -> BE 4,3,2,1
   L64 -> 1,2,3,4,5,6,7,8
   B64 -> 8,7,6,5,4,3,2,1
*/

typedef enum s7_action {
    S7_ACTION_DEFAULT        = 0,
    S7_ACTION_HOLD_REG_WRITE = 1
} s7_action_e;


	// u_char  PDUType;     // 0xE0 Connection request
	// 					 // 0xD0 Connection confirm
	// 					 // 0x80 Disconnect request
	// 					 // 0xDC Disconnect confirm
typedef enum s7_tcotp {
    S7_COTP_CO_REQUEST        = 0xE0,
    S7_COTP_CO_CONFIRM        = 0xD0,
    S7_COTP_DC_REQUEST        = 0x80,
    S7_COTP_DC_CONFIRM        = 0xDC,
    S7_COTP_DT_DATA           = 0xF0,
} s7_tcotp_e;

typedef enum s7_function {
    S7_READ_COIL            = 0x1,
    S7_READ_INPUT           = 0x02,
    S7_READ_HOLD_REG        = 0x03,
    S7_READ_INPUT_REG       = 0x04,
    S7_WRITE_S_COIL         = 0x05,
    S7_WRITE_S_HOLD_REG     = 0x06,
    S7_WRITE_M_HOLD_REG     = 0x10,
    S7_WRITE_M_COIL         = 0x0F,
    S7_READ_COIL_ERR        = 0x81,
    S7_READ_INPUT_ERR       = 0x82,
    S7_READ_HOLD_REG_ERR    = 0x83,
    S7_READ_INPUT_REG_ERR   = 0x84,
    S7_WRITE_S_COIL_ERR     = 0x85,
    S7_WRITE_S_HOLD_REG_ERR = 0x86,
    S7_WRITE_M_HOLD_REG_ERR = 0x90,
    S7_WRITE_M_COIL_ERR     = 0x8F,
    S7_DEVICE_ERR           = -2
} s7_function_e;

typedef enum s7_area {
    S7AreaPE   =	0x81,
    S7AreaPA   =	0x82,
    S7AreaMK   =	0x83,
    S7AreaDB   =	0x84,
    S7AreaCT   =	0x1C,
    S7AreaTM   =	0x1D
} s7_area_e;

typedef struct s7_read_item {
    uint16_t      dbnumber;
    s7_area_e     area;
    uint16_t      start_address;
    uint16_t      n_register;
} s7_read_item_t;

typedef struct s7_read_cmd {
    uint8_t       item_num;
    uint8_t       reserve_id;
    s7_read_item_t item[MaxVars];

    UT_array **tags; // s7_point_t ptr;
} s7_read_cmd_t;

// struct S7_TPTK {
//     uint16_t seq;
//     uint16_t protocol;
//     uint16_t len;
// } __attribute__((packed));

struct S7_TPTK {
	uint8_t Version;    // Always 3 for RFC 1006
	uint8_t Reserved;   // 0
	uint8_t HI_Lenght;  // High part of packet lenght (entire frame, payload and TPDU included)
	uint8_t LO_Lenght;  // Low part of packet lenght (entire frame, payload and TPDU included)
} __attribute__((packed));

void s7_header_wrap(neu_protocol_pack_buf_t *buf);
void s7_cotp_con_warap(neu_protocol_pack_buf_t *buf,uint8_t *base);
void s7_s7com_con_warap(neu_protocol_pack_buf_t *buf,uint8_t *base);
void s7_s7com_multiread_warap(neu_protocol_pack_buf_t *buf,uint8_t *base,s7_read_cmd_t *cmd,uint16_t pdu_size);
void s7_s7com_mutilwrite_warap(neu_protocol_pack_buf_t *buf,uint8_t *base,uint16_t dbnumber,
                       enum s7_area area, uint16_t start_address,
                       uint16_t n_reg, uint8_t *bytes,uint16_t pdu_size);

struct s7_code {
    uint8_t slave_id;
    uint8_t function;
} __attribute__((packed));

void s7_code_wrap(neu_protocol_pack_buf_t *buf, uint8_t slave_id,
                      uint8_t function);
int  s7_cotp_unwrap(neu_protocol_unpack_buf_t *buf,
                        struct s7_code *       out_code);
int s7_cotp_co_unwrap(neu_protocol_unpack_buf_t *buf, S7_TCOTP_CO * cotp_co);
int s7_cotp_dt_unwrap(neu_protocol_unpack_buf_t *buf, S7_TCOTP_DT * cotp_dt);
u_char s7_cotp_pdutype_get(neu_protocol_unpack_buf_t *buf);
u_char s7_res_funcode_get(neu_protocol_unpack_buf_t *buf);

int  s7_res_header23_unwrap(neu_protocol_unpack_buf_t *buf, TS7ResHeader23 *ps7res_header);
int  s7_res_nego_param_unwrap(neu_protocol_unpack_buf_t *buf, TResFunNegotiateParams *param);
int  s7_res_read_param_unwrap(neu_protocol_unpack_buf_t *buf, TResFunReadParams *param);
int  s7_res_read_item_unwrap(neu_protocol_unpack_buf_t *buf, TResFunReadItem *param);
int  s7_res_write_item_unwrap(neu_protocol_unpack_buf_t *buf, byte *param,int item_cnt);

struct s7_address {
    uint16_t start_address;
    uint16_t n_reg;
} __attribute__((packed));

void s7_address_wrap(neu_protocol_pack_buf_t *buf, uint16_t start,
                         uint16_t n_register, enum s7_action m_action);
int  s7_address_unwrap(neu_protocol_unpack_buf_t *buf,
                           struct s7_address *    out_address);

struct s7_data {
    uint8_t n_byte;
    uint8_t byte[];
} __attribute__((packed));

void s7_data_wrap(neu_protocol_pack_buf_t *buf, uint8_t n_byte,
                      uint8_t *bytes, enum s7_action action);
int  s7_data_unwrap(neu_protocol_unpack_buf_t *buf,
                        struct s7_data *       out_data);

struct s7_crc {
    uint16_t crc;
} __attribute__((packed));

void s7_crc_set(neu_protocol_pack_buf_t *buf);
void s7_crc_wrap(neu_protocol_pack_buf_t *buf);
int  s7_crc_unwrap(neu_protocol_unpack_buf_t *buf,
                       struct s7_crc *        out_crc);
int s7_stack_BuildControlPDU(TIsoControlPDU *pIsoControlPDU);
int s7_stack_NegotiatePDU(TIsoDataPDU *pIsoDataPDU);
int s7_stack_ReadMultiVars(TIsoDataPDU *pIsoDataPDU,s7_read_cmd_t *cmd,uint16_t pdu_size);
int s7_stack_WriteMultiVars(TIsoDataPDU *pIsoDataPDU,uint16_t dbnumber,
                       enum s7_area area, uint16_t start_address,
                       uint16_t n_reg, uint8_t *bytes, uint16_t pdu_size);

int DataSizeByte(int WordLength);
word GetNextWord();
word SwapWord(word Value);
longword SwapDWord(longword Value);
const char *s7_area_to_str(s7_area_e area);

#endif