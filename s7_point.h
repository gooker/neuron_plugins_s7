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
#ifndef _NEU_PLUGIN_S7_POINT_H_
#define _NEU_PLUGIN_S7_POINT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include <neuron.h>

#include "s7.h"

typedef struct s7_point {
    uint16_t      dbnumber;
    s7_area_e     area;
    uint16_t      start_address;
    uint16_t      n_register;

    neu_type_e                type;
    neu_datatag_addr_option_u option;
    char                      name[NEU_TAG_NAME_LEN];
} s7_point_t;

typedef struct s7_point_write {
    s7_point_t point;
    neu_value_u    value;
} s7_point_write_t;

int s7_tag_to_point(const neu_datatag_t *tag, s7_point_t *point);
int s7_write_tag_to_point(const neu_plugin_tag_value_t *tag,
                              s7_point_write_t *        point);

typedef struct s7_read_cmd_sort {
    uint16_t      n_cmd;
    s7_read_cmd_t *cmd;
} s7_read_cmd_sort_t;

typedef struct s7_write_cmd {
    uint16_t      dbnumber;
    s7_area_e     area;
    uint16_t      start_address;
    uint16_t      n_register;
    uint8_t       n_byte;
    uint8_t *     bytes;

    UT_array *tags;
} s7_write_cmd_t;

typedef struct s7_write_cmd_sort {
    uint16_t            n_cmd;
    s7_write_cmd_t *cmd;
} s7_write_cmd_sort_t;

s7_read_cmd_sort_t * s7_tag_sort(UT_array *tags, uint16_t max_byte);
s7_write_cmd_sort_t *s7_write_tags_sort(UT_array *tags);
void                     s7_tag_sort_free(s7_read_cmd_sort_t *cs);

#ifdef __cplusplus
}
#endif

#endif