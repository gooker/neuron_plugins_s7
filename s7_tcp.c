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
#include <stdlib.h>

#include <neuron.h>

#include "errcodes.h"

#include "s7_point.h"
#include "s7_req.h"
#include "s7_stack.h"

static neu_plugin_t *driver_open(void);

static int driver_close(neu_plugin_t *plugin);
static int driver_init(neu_plugin_t *plugin, bool load);
static int driver_uninit(neu_plugin_t *plugin);
static int driver_start(neu_plugin_t *plugin);
static int driver_stop(neu_plugin_t *plugin);
static int driver_config(neu_plugin_t *plugin, const char *config);
static int driver_request(neu_plugin_t *plugin, neu_reqresp_head_t *head,
                          void *data);

static int driver_tag_validator(const neu_datatag_t *tag);
static int driver_validate_tag(neu_plugin_t *plugin, neu_datatag_t *tag);
static int driver_group_timer(neu_plugin_t *plugin, neu_plugin_group_t *group);
static int driver_write(neu_plugin_t *plugin, void *req, neu_datatag_t *tag,
                        neu_value_u value);
static int driver_write_tags(neu_plugin_t *plugin, void *req, UT_array *tags);

static const neu_plugin_intf_funs_t plugin_intf_funs = {
    .open    = driver_open,
    .close   = driver_close,
    .init    = driver_init,
    .uninit  = driver_uninit,
    .start   = driver_start,
    .stop    = driver_stop,
    .setting = driver_config,
    .request = driver_request,

    .driver.validate_tag  = driver_validate_tag,
    .driver.group_timer   = driver_group_timer,
    .driver.write_tag     = driver_write,
    .driver.tag_validator = driver_tag_validator,
    .driver.write_tags    = driver_write_tags,
    .driver.add_tags      = NULL,
    .driver.load_tags     = NULL,
    .driver.del_tags      = NULL,
};

const neu_plugin_module_t neu_plugin_module = {
    .version     = NEURON_PLUGIN_VER_1_0,
    .schema      = "s7-tcp",
    .module_name = "Siemens S7",
    .module_descr =
        "This plugin is used to connect Siemens PLC using the s7 TCP protocol.",
    .module_descr_zh =
        "该插件用于使用S7协议连接西门子PLC",
    .intf_funs = &plugin_intf_funs,
    .kind      = NEU_PLUGIN_KIND_SYSTEM,
    .type      = NEU_NA_TYPE_DRIVER,
    .display   = true,
    .single    = false,
};

static neu_plugin_t *driver_open(void)
{
    neu_plugin_t *plugin = calloc(1, sizeof(neu_plugin_t));

    neu_plugin_common_init(&plugin->common);

    return plugin;
}

static int driver_close(neu_plugin_t *plugin)
{
    free(plugin);

    return 0;
}

static int driver_init(neu_plugin_t *plugin, bool load)
{
    (void) load;
    plugin->protocol = S7_PROTOCOL_TCP;
    plugin->events   = neu_event_new();
    plugin->stack    = s7_stack_create((void *) plugin, S7_PROTOCOL_TCP,
                                        s7_send_msg, s7_value_handle,
                                        s7_write_resp);

    plog_notice(plugin, "%s init success", plugin->common.name);
    return 0;
}

static int driver_uninit(neu_plugin_t *plugin)
{
    plog_notice(plugin, "%s uninit start", plugin->common.name);
    if (plugin->conn != NULL) {
        neu_conn_destory(plugin->conn);
    }

    if (plugin->stack) {
        s7_stack_destroy(plugin->stack);
    }

    neu_event_close(plugin->events);

    plog_notice(plugin, "%s uninit success", plugin->common.name);

    return 0;
}

static int driver_start(neu_plugin_t *plugin)
{
    neu_conn_start(plugin->conn);
    plog_notice(plugin, "%s start success", plugin->common.name);
    return 0;
}

static int driver_stop(neu_plugin_t *plugin)
{
    neu_conn_stop(plugin->conn);
    plog_notice(plugin, "%s stop success", plugin->common.name);
    return 0;
}

static int driver_config(neu_plugin_t *plugin, const char *config)
{
    int              ret       = 0;
    char *           err_param = NULL;
    neu_json_elem_t  host      = { .name      = "host",
                             .t         = NEU_JSON_STR,
                             .v.val_str = NULL };
    neu_json_elem_t  port      = { .name = "port", .t = NEU_JSON_INT };
    neu_json_elem_t  pdu_size  = { .name = "pdu_size", .t = NEU_JSON_INT };
    neu_json_elem_t  module    = { .name = "module", .t = NEU_JSON_INT };
    neu_json_elem_t  rack      = { .name = "rack", .t = NEU_JSON_INT };
    neu_json_elem_t  slot      = { .name = "slot", .t = NEU_JSON_INT };
    neu_json_elem_t  interval  = { .name = "interval", .t = NEU_JSON_INT };
    neu_conn_param_t param = { 0 };


    ret = neu_parse_param((char *) config, &err_param, 6,&host,  &port, &pdu_size,
                          &module,&rack, &slot);
    if (ret != 0) {
        plog_error(plugin, "config: %s, decode error: %s", config, err_param);
        free(err_param);
        if (host.v.val_str != NULL) {
            free(host.v.val_str);
        }
        return -1;
    }

    param.log              = plugin->common.log;
    plugin->interval       = interval.v.val_int;

    param.type                      = NEU_CONN_TCP_CLIENT;
    param.params.tcp_client.ip      = host.v.val_str;
    param.params.tcp_client.port    = port.v.val_int;
    param.params.tcp_client.timeout = 3000;
    plugin->is_server               = false;

    plog_notice(plugin,
                "config: host: %s, port: %" PRId64 ", module: %" PRId64 "",
                host.v.val_str, port.v.val_int, module.v.val_int);

    if (plugin->conn != NULL) {
        plugin->conn = neu_conn_reconfig(plugin->conn, &param);
    } else {
        plugin->common.link_state = NEU_NODE_LINK_STATE_DISCONNECTED;
        plugin->conn =
            neu_conn_new(&param, (void *) plugin, s7_conn_connected,
                         s7_conn_disconnected);
    }

    free(host.v.val_str);
    return 0;
}

static int driver_request(neu_plugin_t *plugin, neu_reqresp_head_t *head,
                          void *data)
{
    (void) plugin;
    (void) head;
    (void) data;
    return 0;
}

static int driver_tag_validator(const neu_datatag_t *tag)
{
    s7_point_t point = { 0 };
    return s7_tag_to_point(tag, &point);
}

static int driver_validate_tag(neu_plugin_t *plugin, neu_datatag_t *tag)
{
    s7_point_t point = { 0 };

    int ret = s7_tag_to_point(tag, &point);
    if (ret == 0) {
        plog_notice(
            plugin,
            "validate tag success, name: %s, address: %s, type: %d, slave id: "
            "%d, start address: %d, n register: %d, area: %s",
            tag->name, tag->address, tag->type, point.dbnumber,
            point.start_address, point.n_register,
            s7_area_to_str(point.area));
    } else {
        plog_error(
            plugin,
            "validate tag error, name: %s, address: %s, type: %d, slave id: "
            "%d, start address: %d, n register: %d, area: %s",
            tag->name, tag->address, tag->type, point.dbnumber,
            point.start_address, point.n_register,
            s7_area_to_str(point.area));
    }
    
    return ret;
}

static int driver_group_timer(neu_plugin_t *plugin, neu_plugin_group_t *group)
{
    return s7_group_timer(plugin, group);
}

static int driver_write(neu_plugin_t *plugin, void *req, neu_datatag_t *tag,
                        neu_value_u value)
{
    return s7_write_tag(plugin, req, tag, value);
}

static int driver_write_tags(neu_plugin_t *plugin, void *req, UT_array *tags)
{
    return s7_write_tags(plugin, req, tags);
}