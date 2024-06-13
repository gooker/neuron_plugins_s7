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
#ifndef _NEU_M_PLUGIN_S7_REQ_H_
#define _NEU_M_PLUGIN_S7_REQ_H_

#include <neuron.h>

#include "s7_stack.h"
#include "s7_point.h"

struct s7_group_data {
    UT_array *              tags;
    char *                  group;
    s7_read_cmd_sort_t *cmd_sort;
};

struct s7_write_tags_data {
    UT_array *               tags;
    s7_write_cmd_sort_t *cmd_sort;
};

struct neu_plugin {
    neu_plugin_common_t common;

    neu_conn_t *    conn;
    s7_stack_t *stack;

    void *   plugin_group_data;
    uint16_t cmd_idx;

    neu_event_io_t *tcp_server_io;
    bool            is_server;
    bool            is_serial;
    int             client_fd;
    neu_events_t *  events;

    s7_protocol_e protocol;

    uint16_t interval;
    uint16_t retry_interval;
    uint16_t max_retries;
};

void s7_conn_connected(void *data, int fd);
void s7_conn_disconnected(void *data, int fd);
void s7_tcp_server_listen(void *data, int fd);
void s7_tcp_server_stop(void *data, int fd);
int  s7_tcp_server_io_callback(enum neu_event_io_type type, int fd,
                                   void *usr_data);
int s7_stack_connect(neu_plugin_t *plugin) ;
int64_t s7_stack_datacom(neu_plugin_t *plugin,struct s7_group_data *gd);
int s7_group_sort(neu_plugin_t *plugin,neu_plugin_group_t *group,struct s7_group_data **gd);
int s7_group_timer(neu_plugin_t *plugin, neu_plugin_group_t *group);
int s7_send_msg(void *ctx, uint16_t n_byte, uint8_t *bytes);
int s7_value_handle(void *ctx, uint8_t tag_item_idx, uint16_t n_byte,
                        uint8_t *bytes, int error);
int s7_write(neu_plugin_t *plugin, void *req, neu_datatag_t *tag,
                 neu_value_u value, bool response);
int s7_write_tag(neu_plugin_t *plugin, void *req, neu_datatag_t *tag,
                     neu_value_u value);
int s7_write_tags(neu_plugin_t *plugin, void *req, UT_array *tags);
int s7_write_resp(void *ctx, void *req, int error);

#endif
