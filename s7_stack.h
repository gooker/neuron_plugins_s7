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
#ifndef _NEU_M_PLUGIN_S7_STACK_H_
#define _NEU_M_PLUGIN_S7_STACK_H_

#include <stdint.h>

#include <neuron.h>

#include "s7.h"

typedef int (*s7_stack_send)(void *ctx, uint16_t n_byte, uint8_t *bytes);
typedef int (*s7_stack_value)(void *ctx, uint16_t dbnumber, uint16_t n_byte,
                                  uint8_t *bytes, int error);
typedef int (*s7_stack_write_resp)(void *ctx, void *req, int error);

typedef enum s7_protocol {
    S7_PROTOCOL_TCP = 1,
    S7_PROTOCOL_300 = 2,
} s7_protocol_e;

struct s7_stack {
    void *                  ctx;
    s7_stack_send       send_fn;
    s7_stack_value      value_fn;
    s7_stack_write_resp write_resp;

    s7_protocol_e protocol;
    uint16_t          read_seq;
    uint16_t          write_seq;

    uint8_t *buf;
    uint16_t buf_size;

    bool cotp_is_connected; // COTP connection status
    bool s7com_is_connected; // TPKT connection status
    uint16_t pdu_size;
};

typedef struct s7_stack s7_stack_t;

s7_stack_t *s7_stack_create(void *ctx, s7_protocol_e protocol,
                                    s7_stack_send       send_fn,
                                    s7_stack_value      value_fn,
                                    s7_stack_write_resp write_resp);
void            s7_stack_destroy(s7_stack_t *stack);

int s7_stack_recv(s7_stack_t *stack,neu_protocol_unpack_buf_t *buf);
int s7_stack_Handshake(s7_stack_t *stack);
int  s7_stack_read(s7_stack_t *stack, s7_read_cmd_t *cmd, uint16_t *response_size);
int  s7_stack_write(s7_stack_t *stack, void *req, uint16_t dbnumber,
                        enum s7_area area, uint16_t start_address,
                        uint16_t n_reg, uint8_t *bytes, uint8_t n_byte,
                        uint16_t *response_size, bool response);

#endif