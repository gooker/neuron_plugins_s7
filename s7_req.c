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
#include <time.h>

#include "s7_req.h"

static void plugin_group_free(neu_plugin_group_t *pgp);
static int  process_protocol_buf(neu_plugin_t *plugin, uint8_t reserve_id,
                                 uint16_t response_size);

void s7_conn_connected(void *data, int fd)
{
    struct neu_plugin *plugin = (struct neu_plugin *) data;
    (void) fd;

    plugin->common.link_state = NEU_NODE_LINK_STATE_CONNECTED;
}

void s7_conn_disconnected(void *data, int fd)
{
    struct neu_plugin *plugin = (struct neu_plugin *) data;
    (void) fd;

    plugin->common.link_state = NEU_NODE_LINK_STATE_DISCONNECTED;
}

void s7_tcp_server_listen(void *data, int fd)
{
    struct neu_plugin *  plugin = (struct neu_plugin *) data;
    neu_event_io_param_t param  = {
        .cb       = s7_tcp_server_io_callback,
        .fd       = fd,
        .usr_data = (void *) plugin,
    };

    plugin->tcp_server_io = neu_event_add_io(plugin->events, param);
}

void s7_tcp_server_stop(void *data, int fd)
{
    struct neu_plugin *plugin = (struct neu_plugin *) data;
    (void) fd;

    neu_event_del_io(plugin->events, plugin->tcp_server_io);
}

int s7_tcp_server_io_callback(enum neu_event_io_type type, int fd,
                                  void *usr_data)
{
    neu_plugin_t *plugin = (neu_plugin_t *) usr_data;

    switch (type) {
    case NEU_EVENT_IO_READ: {
        int client_fd = neu_conn_tcp_server_accept(plugin->conn);
        if (client_fd > 0) {
            plugin->client_fd = client_fd;
        }

        break;
    }
    case NEU_EVENT_IO_CLOSED:
    case NEU_EVENT_IO_HUP:
        plog_warn(plugin, "tcp server recv: %d, conn closed, fd: %d", type, fd);
        neu_event_del_io(plugin->events, plugin->tcp_server_io);
        neu_conn_disconnect(plugin->conn);
        break;
    }

    return 0;
}

int s7_send_msg(void *ctx, uint16_t n_byte, uint8_t *bytes)
{
    neu_plugin_t *plugin = (neu_plugin_t *) ctx;
    int           ret    = 0;

    plog_send_protocol(plugin, bytes, n_byte);

    if (plugin->is_server) {
        ret = neu_conn_tcp_server_send(plugin->conn, plugin->client_fd, bytes,
                                       n_byte);
    } else {
        ret = neu_conn_send(plugin->conn, bytes, n_byte);
    }

    return ret;
}

int s7_stack_read_retry(neu_plugin_t *plugin, struct s7_group_data *gd,
                            uint16_t i, uint16_t j, uint16_t *response_size)
{
    struct timespec t3 = { .tv_sec = plugin->retry_interval / 1000,
                           .tv_nsec =
                               1000 * 1000 * (plugin->retry_interval % 1000) };
    struct timespec t4 = { 0 };
    nanosleep(&t3, &t4);
    plog_notice(plugin, "Resend read req. Times:%hu", j + 1);
    int ret = s7_stack_read(plugin->stack, &(gd->cmd_sort->cmd[i]), response_size);
    return ret;
}

int s7_stack_connect(neu_plugin_t *plugin)
{
    clock_t start_time = clock();
    while (1)
    {
        int ret = s7_stack_Handshake(plugin->stack);
        if (ret == 0)
        {
            break;
        }
        else
        {
            process_protocol_buf(plugin, 0, 0);
        }
        
        // Check if 3 seconds have passed
        double time_passed_in_seconds = ((double)(clock() - start_time)) / CLOCKS_PER_SEC;
        if (time_passed_in_seconds >= 3.0)
        {
            return -1;
        }
    }
    return 0;
}

//sS7 数据交互
int64_t s7_stack_datacom(neu_plugin_t *plugin,struct s7_group_data *gd)
{
    int64_t                rtt = NEU_METRIC_LAST_RTT_MS_MAX;
    for (uint16_t i = 0; i < gd->cmd_sort->n_cmd; i++) {
        plugin->cmd_idx        = i;
        uint16_t response_size = 0;
        uint64_t read_tms      = neu_time_ms();
        int      ret_buf       = 0;
        int      ret_r         = s7_stack_read(plugin->stack,&(gd->cmd_sort->cmd[i]), &response_size);
        if (ret_r > 0) {
            ret_buf = process_protocol_buf(
                plugin, gd->cmd_sort->cmd[i].reserve_id, response_size);
            if (ret_buf > 0) {
                rtt = neu_time_ms() - read_tms;
            } else if (ret_buf == 0) {
                for (uint16_t j = 0; j < plugin->max_retries; j++) {
                    ret_r = s7_stack_read_retry(plugin, gd, i, j,
                                                    &response_size);
                    if (ret_r > 0) {
                        ret_buf = process_protocol_buf(
                            plugin, gd->cmd_sort->cmd[i].reserve_id,
                            response_size);
                        if (ret_buf > 0) {
                            rtt = neu_time_ms() - read_tms;
                            break;
                        } else if (ret_buf < 0) {
                            if (ret_buf == -1) {
                                s7_value_handle(
                                    plugin, gd->cmd_sort->cmd[i].reserve_id, 0,
                                    NULL,
                                    NEU_ERR_PLUGIN_PROTOCOL_DECODE_FAILURE);
                                plog_error(
                                    plugin,
                                    "modbus message error, skip, %hhu!%hu",
                                    gd->cmd_sort->cmd[i].reserve_id,
                                    gd->cmd_sort->cmd[i].item_num);
                            } else if (ret_buf == -2) {
                                s7_value_handle(
                                    plugin, gd->cmd_sort->cmd[i].reserve_id, 0,
                                    NULL, NEU_ERR_PLUGIN_READ_FAILURE);
                                plog_error(
                                    plugin,
                                    "modbus device response error, skip, "
                                    "%hhu!%hu",
                                    gd->cmd_sort->cmd[i].reserve_id,
                                    gd->cmd_sort->cmd[i].item_num);
                            }
                            rtt = neu_time_ms() - read_tms;
                            break;
                        }
                    } else {
                        s7_value_handle(plugin,
                                            gd->cmd_sort->cmd[i].reserve_id, 0,
                                            NULL, NEU_ERR_PLUGIN_DISCONNECTED);
                        rtt = NEU_METRIC_LAST_RTT_MS_MAX;
                        neu_conn_disconnect(plugin->conn);
                        break;
                    }
                }
                if (ret_r > 0 && ret_buf == 0) {
                    s7_value_handle(plugin, gd->cmd_sort->cmd[i].reserve_id,
                                        0, NULL,
                                        NEU_ERR_PLUGIN_DEVICE_NOT_RESPONSE);
                    plog_warn(plugin,
                              "no modbus response received, skip, %hhu!%hu",
                              gd->cmd_sort->cmd[i].reserve_id,
                              gd->cmd_sort->cmd[i].item_num);
                }
            } else if (ret_buf < 0) {
                if (ret_buf == -1) {
                    s7_value_handle(plugin, gd->cmd_sort->cmd[i].reserve_id,
                                        0, NULL,
                                        NEU_ERR_PLUGIN_PROTOCOL_DECODE_FAILURE);
                    plog_error(plugin, "s7 message error, skip, %hhu!%hu",
                               gd->cmd_sort->cmd[i].reserve_id,
                               gd->cmd_sort->cmd[i].item_num);
                } else if (ret_buf == -2) {
                    s7_value_handle(plugin, gd->cmd_sort->cmd[i].reserve_id,
                                        0, NULL, NEU_ERR_PLUGIN_READ_FAILURE);
                    plog_error(plugin,
                               "s7 device response error, skip, %hhu!%hu",
                               gd->cmd_sort->cmd[i].reserve_id,
                               gd->cmd_sort->cmd[i].item_num);
                }
                rtt = neu_time_ms() - read_tms;
            }
        } else {
            for (uint16_t j = 0; j < plugin->max_retries; j++) {
                ret_r =
                    s7_stack_read_retry(plugin, gd, i, j, &response_size);
                if (ret_r > 0) {
                    ret_buf = process_protocol_buf(
                        plugin, gd->cmd_sort->cmd[i].reserve_id, response_size);
                    if (ret_buf > 0) {
                        rtt = neu_time_ms() - read_tms;
                        break;
                    } else if (ret_buf < 0) {
                        if (ret_buf == -1) {
                            s7_value_handle(
                                plugin, gd->cmd_sort->cmd[i].reserve_id, 0, NULL,
                                NEU_ERR_PLUGIN_PROTOCOL_DECODE_FAILURE);
                            plog_error(plugin,
                                       "modbus message error, skip, %hhu!%hu",
                                       gd->cmd_sort->cmd[i].reserve_id,
                                       gd->cmd_sort->cmd[i].item_num);
                        } else if (ret_buf == -2) {
                            s7_value_handle(
                                plugin, gd->cmd_sort->cmd[i].reserve_id, 0, NULL,
                                NEU_ERR_PLUGIN_READ_FAILURE);
                            plog_error(plugin,
                                       "modbus device response error, skip, "
                                       "%hhu!%hu",
                                       gd->cmd_sort->cmd[i].reserve_id,
                                       gd->cmd_sort->cmd[i].item_num);
                        }
                        rtt = neu_time_ms() - read_tms;
                        break;
                    } else if (ret_buf == 0) {
                        s7_value_handle(
                            plugin, gd->cmd_sort->cmd[i].reserve_id, 0, NULL,
                            NEU_ERR_PLUGIN_DEVICE_NOT_RESPONSE);
                        plog_warn(plugin,
                                  "no modbus response received, skip, %hhu!%hu",
                                  gd->cmd_sort->cmd[i].reserve_id,
                                  gd->cmd_sort->cmd[i].item_num);
                        continue;
                    }
                }
            }
            if (ret_r <= 0) {
                s7_value_handle(plugin, gd->cmd_sort->cmd[i].reserve_id, 0,
                                    NULL, NEU_ERR_PLUGIN_DISCONNECTED);
                rtt = NEU_METRIC_LAST_RTT_MS_MAX;
                neu_conn_disconnect(plugin->conn);
                break;
            }
        }
        if (plugin->interval > 0) {
            struct timespec t1 = { .tv_sec  = plugin->interval / 1000,
                                   .tv_nsec = 1000 * 1000 *
                                       (plugin->interval % 1000) };
            struct timespec t2 = { 0 };
            nanosleep(&t1, &t2);
        }
    }
    return rtt;
}

int s7_group_sort(neu_plugin_t *plugin,neu_plugin_group_t *group,struct s7_group_data **gd)
{
    if (group->user_data == NULL) {
        *gd = calloc(1, sizeof(struct s7_group_data));

        group->user_data  = (*gd);
        group->group_free = plugin_group_free;
        utarray_new((*gd)->tags, &ut_ptr_icd);

        utarray_foreach(group->tags, neu_datatag_t *, tag)
        {
            s7_point_t *p   = calloc(1, sizeof(s7_point_t));
            int             ret = s7_tag_to_point(tag, p);
            if (ret != NEU_ERR_SUCCESS) {
                // plog_error(plugin, "invalid tag: %s, address: %s", tag->name,
                //            tag->address);
            }
            utarray_push_back((*gd)->tags, &p);
        }

        (*gd)->group    = strdup(group->group_name);

        uint16_t max_byte = 0xF0 - 18 - 7; //240-header-tptk&cotp
        if (plugin->stack->pdu_size > 0)
        {
            max_byte    = plugin->stack->pdu_size- 18 - 7; 
        }
        
        (*gd)->cmd_sort = s7_tag_sort((*gd)->tags, max_byte);
    }
    (*gd)                     = (struct s7_group_data *) group->user_data;
    plugin->plugin_group_data = (*gd);
    return 0;
}

int s7_group_timer(neu_plugin_t *plugin, neu_plugin_group_t *group)
{
    neu_conn_state_t               state = { 0 };
    neu_adapter_update_metric_cb_t update_metric =
        plugin->common.adapter_callbacks->update_metric;

    //S7 数据交互之前需要先进行2次握手 成功后才能进行数据交互
    int cnt_ret = s7_stack_connect(plugin);
    if (cnt_ret < 0)
    {
        plog_error(plugin, "s7 stack connect failed");
        return -1;
    }

    //初始化group_data tag sort
    struct s7_group_data *gd  = NULL;
    s7_group_sort(plugin,group,&gd);

    //S7 数据交互
    int64_t rtt = s7_stack_datacom(plugin,gd);

    state = neu_conn_state(plugin->conn);
    update_metric(plugin->common.adapter, NEU_METRIC_SEND_BYTES,
                  state.send_bytes, NULL);
    update_metric(plugin->common.adapter, NEU_METRIC_RECV_BYTES,
                  state.recv_bytes, NULL);
    update_metric(plugin->common.adapter, NEU_METRIC_LAST_RTT_MS, rtt, NULL);
    update_metric(plugin->common.adapter, NEU_METRIC_GROUP_LAST_SEND_MSGS,
                  gd->cmd_sort->n_cmd, group->group_name);
    return 0;
}

int s7_value_handle(void *ctx, uint8_t tag_item_idx, uint16_t n_byte,
                        uint8_t *bytes, int error)
{
    neu_plugin_t *            plugin = (neu_plugin_t *) ctx;
    struct s7_group_data *gd = (struct s7_group_data *) plugin->plugin_group_data;
    if(gd == NULL)
    {
        return 0;
    }

    uint16_t start_address = gd->cmd_sort->cmd[plugin->cmd_idx].item[tag_item_idx].start_address;
    // uint16_t n_register    = gd->cmd_sort->cmd[plugin->cmd_idx].item[tag_item_idx].n_register;

    if (error == NEU_ERR_PLUGIN_DISCONNECTED) {
        neu_dvalue_t dvalue = { 0 };

        dvalue.type      = NEU_TYPE_ERROR;
        dvalue.value.i32 = error;
        plugin->common.adapter_callbacks->driver.update(
            plugin->common.adapter, gd->group, NULL, dvalue);
        return 0;
    } else if (error != NEU_ERR_SUCCESS) {
        utarray_foreach(gd->cmd_sort->cmd[plugin->cmd_idx].tags[tag_item_idx],
                        s7_point_t **, p_tag)
        {
            neu_dvalue_t dvalue = { 0 };
            dvalue.type         = NEU_TYPE_ERROR;
            dvalue.value.i32    = error;
            plugin->common.adapter_callbacks->driver.update(
                plugin->common.adapter, gd->group, (*p_tag)->name, dvalue);
        }
        return 0;
    }

    utarray_foreach(gd->cmd_sort->cmd[plugin->cmd_idx].tags[tag_item_idx], s7_point_t **,
                    p_tag)
    {
        neu_dvalue_t dvalue = { 0 };

        if (n_byte >= ((*p_tag)->start_address - start_address) + (*p_tag)->n_register ) 
        {
            memcpy(dvalue.value.bytes.bytes,
                    bytes +((*p_tag)->start_address - start_address),
                    (*p_tag)->n_register);
            dvalue.value.bytes.length = (*p_tag)->n_register;
        }

        dvalue.type = (*p_tag)->type;
        switch ((*p_tag)->type) {
        case NEU_TYPE_INT8:
        case NEU_TYPE_UINT8:
            dvalue.value.u8 = bytes[0];
            break;
        case NEU_TYPE_UINT16:
        case NEU_TYPE_INT16:
            dvalue.value.u16 = ntohs(dvalue.value.u16);
            break;
        case NEU_TYPE_FLOAT:
        case NEU_TYPE_INT32:
        case NEU_TYPE_UINT32:
            dvalue.value.u32 = ntohl(dvalue.value.u32);
            break;
        case NEU_TYPE_DOUBLE:
        case NEU_TYPE_INT64:
        case NEU_TYPE_UINT64:
            dvalue.value.u64 = neu_ntohll(dvalue.value.u64);
            break;
        case NEU_TYPE_BIT: {
            uint16_t offset = (*p_tag)->start_address - start_address;
            if (n_byte > offset) {
                neu_value8_u u8 = { .value = bytes[offset] };
                dvalue.value.u8 = neu_value8_get_bit(u8, (*p_tag)->option.bit.bit);
            }
            break;
        }
        case NEU_TYPE_STRING: {
            switch ((*p_tag)->option.string.type) {
            case NEU_DATATAG_STRING_TYPE_H:
                break;
            case NEU_DATATAG_STRING_TYPE_L:
                neu_datatag_string_ltoh(dvalue.value.str,
                                        strlen(dvalue.value.str));
                break;
            case NEU_DATATAG_STRING_TYPE_D:
                break;
            case NEU_DATATAG_STRING_TYPE_E:
                break;
            }

            if (!neu_datatag_string_is_utf8(dvalue.value.str,
                                            strlen(dvalue.value.str))) {
                dvalue.value.str[0] = '?';
                dvalue.value.str[1] = 0;
            }
            break;
        }
        default:
            break;
        }

        plugin->common.adapter_callbacks->driver.update(
            plugin->common.adapter, gd->group, (*p_tag)->name, dvalue);
    }
    return 0;
}

int s7_write(neu_plugin_t *plugin, void *req, neu_datatag_t *tag,
                 neu_value_u value, bool response)
{
    s7_point_t point = { 0 };
    int            ret   = s7_tag_to_point(tag, &point);
    assert(ret == 0);
    uint8_t n_byte = 0;

    switch (tag->type) {
    case NEU_TYPE_INT8:
    case NEU_TYPE_UINT8:
        n_byte = sizeof(uint8_t);
        break;
    case NEU_TYPE_UINT16:
    case NEU_TYPE_INT16:
        value.u16 = htons(value.u16);
        n_byte    = sizeof(uint16_t);
        break;
    case NEU_TYPE_FLOAT:
    case NEU_TYPE_UINT32:
    case NEU_TYPE_INT32:
        value.u32 = htonl(value.u32);
        n_byte    = sizeof(uint32_t);
        break;

    case NEU_TYPE_DOUBLE:
    case NEU_TYPE_INT64:
    case NEU_TYPE_UINT64:
        value.u64 = neu_htonll(value.u64);
        n_byte    = sizeof(uint64_t);
        break;
    case NEU_TYPE_BIT: {
        n_byte = sizeof(uint8_t);
        break;
    }
    case NEU_TYPE_STRING: {
        switch (point.option.string.type) {
        case NEU_DATATAG_STRING_TYPE_H:
            break;
        case NEU_DATATAG_STRING_TYPE_L:
            neu_datatag_string_ltoh(value.str, point.option.string.length);
            break;
        case NEU_DATATAG_STRING_TYPE_D:
            break;
        case NEU_DATATAG_STRING_TYPE_E:
            break;
        }
        n_byte = point.option.string.length;
        break;
    }
    case NEU_TYPE_BYTES: {
        n_byte = point.option.bytes.length;
        break;
    }
    default:
        // assert(1 == 0);
        break;
    }

    uint16_t response_size = 0;
    ret                    = s7_stack_write(
        plugin->stack, req, point.dbnumber, point.area, point.start_address,
        point.n_register, value.bytes.bytes, n_byte, &response_size, response);
    if (ret > 0) {
        process_protocol_buf(plugin, point.dbnumber, response_size);
    }

    return ret;
}

int s7_write_tag(neu_plugin_t *plugin, void *req, neu_datatag_t *tag,
                     neu_value_u value)
{
    int ret = 0;
    ret     = s7_write(plugin, req, tag, value, true);
    return ret;
}

int s7_write_tags(neu_plugin_t *plugin, void *req, UT_array *tags)
{
    struct s7_write_tags_data *gtags = NULL;
    int                            ret   = 0;
    int                            rv    = 0;

    gtags = calloc(1, sizeof(struct s7_write_tags_data));

    utarray_new(gtags->tags, &ut_ptr_icd);
    utarray_foreach(tags, neu_plugin_tag_value_t *, tag)
    {
        s7_point_write_t *p = calloc(1, sizeof(s7_point_write_t));
        ret                     = s7_write_tag_to_point(tag, p);
        assert(ret == 0);

        utarray_push_back(gtags->tags, &p);
    }
    gtags->cmd_sort = s7_write_tags_sort(gtags->tags);
    for (uint16_t i = 0; i < gtags->cmd_sort->n_cmd; i++) {
        uint16_t response_size = 0;

        ret = s7_stack_write(
            plugin->stack, req, gtags->cmd_sort->cmd[i].dbnumber,
            gtags->cmd_sort->cmd[i].area, gtags->cmd_sort->cmd[i].start_address,
            gtags->cmd_sort->cmd[i].n_register, gtags->cmd_sort->cmd[i].bytes,
            gtags->cmd_sort->cmd[i].n_byte, &response_size, false);
        if (ret > 0) {
            process_protocol_buf(plugin, gtags->cmd_sort->cmd[i].dbnumber,
                                 response_size);
        }

        if (ret <= 0) {
            rv = 1;
        }
        if (plugin->interval > 0) {
            struct timespec t1 = { .tv_sec  = plugin->interval / 1000,
                                   .tv_nsec = 1000 * 1000 *
                                       (plugin->interval % 1000) };
            struct timespec t2 = { 0 };
            nanosleep(&t1, &t2);
        }
    }

    if (rv == 0) {
        plugin->common.adapter_callbacks->driver.write_response(
            plugin->common.adapter, req, NEU_ERR_SUCCESS);
    } else {
        plugin->common.adapter_callbacks->driver.write_response(
            plugin->common.adapter, req, NEU_ERR_PLUGIN_DISCONNECTED);
    }

    for (uint16_t i = 0; i < gtags->cmd_sort->n_cmd; i++) {
        utarray_free(gtags->cmd_sort->cmd[i].tags);
        free(gtags->cmd_sort->cmd[i].bytes);
    }
    free(gtags->cmd_sort->cmd);
    free(gtags->cmd_sort);
    utarray_foreach(gtags->tags, s7_point_write_t **, tag) { free(*tag); }
    utarray_free(gtags->tags);
    free(gtags);
    return ret;
}

int s7_write_resp(void *ctx, void *req, int error)
{
    neu_plugin_t *plugin = (neu_plugin_t *) ctx;

    plugin->common.adapter_callbacks->driver.write_response(
        plugin->common.adapter, req, error);
    return 0;
}

static void plugin_group_free(neu_plugin_group_t *pgp)
{
    struct s7_group_data *gd = (struct s7_group_data *) pgp->user_data;

    s7_tag_sort_free(gd->cmd_sort);

    utarray_foreach(gd->tags, s7_point_t **, tag) { free(*tag); }

    utarray_free(gd->tags);
    free(gd->group);

    free(gd);
}

static int process_protocol_buf(neu_plugin_t *plugin, uint8_t reserve_id,
                                uint16_t response_size)
{
    uint8_t                   recv_buf[1024];
    neu_protocol_unpack_buf_t pbuf     = { 0 };
    ssize_t                   ret      = 0;

    //nouse
    // printf("reserve_id:%d,response_size:%d\n",reserve_id,response_size);

    if (plugin->protocol == S7_PROTOCOL_TCP) {
        if (plugin->is_server) {
            ret = neu_conn_tcp_server_recv(plugin->conn, plugin->client_fd,
                                           recv_buf,
                                           sizeof(struct S7_TPTK));
        } else {
            ret = neu_conn_recv(plugin->conn, recv_buf,sizeof(struct S7_TPTK));
        }

        if (ret == 0 || ret == -1) {
            return 0;
        }

        if (ret == sizeof(struct S7_TPTK)) {
            struct S7_TPTK *header = (struct S7_TPTK *) recv_buf;
            int                   ret1   = 0;
            //len 等于 header的HI_Lenght和LO_Lenght组合uint16 
            int                   len    = (header->HI_Lenght << 8) | header->LO_Lenght;

            //校验长度
            // if (len != response_size) {
            //     return -1;
            // }

            plog_recv_protocol(plugin, recv_buf, ret);
            if (plugin->is_server) {
                ret1 = neu_conn_tcp_server_recv(
                    plugin->conn, plugin->client_fd,
                    recv_buf + sizeof(struct S7_TPTK),
                    len);
            } else {
                ret1 = neu_conn_recv(plugin->conn,
                                     recv_buf + sizeof(struct S7_TPTK),
                                     len - sizeof(struct S7_TPTK));
            }
            if (ret1 != (len - (int)sizeof(struct S7_TPTK))) {
                return -1;
            }
            ret += ret1;
        } else {
            return -1;
        }

        if (ret > 0) {
            if (ret < 512) {
                plog_recv_protocol(plugin, recv_buf, ret);
            }
            neu_protocol_unpack_buf_init(&pbuf, recv_buf, ret);
            int ret_s = s7_stack_recv(plugin->stack,&pbuf);
            if (ret_s == S7_DEVICE_ERR) {
                ret = ret_s;
            } else {
                // if (ret != response_size) {
                //     ret = -1;
                // } else {
                    ret = ret_s;
                // }
            }
        }
    }
    return ret;
}