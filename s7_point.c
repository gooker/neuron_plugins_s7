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
#include <memory.h>

#include <neuron.h>

#include "s7_point.h"

struct s7_sort_ctx {
    uint16_t start;
    uint16_t end;
};

static __thread uint16_t s7_read_max_byte = 240;

static int  tag_cmp(neu_tag_sort_elem_t *tag1, neu_tag_sort_elem_t *tag2);
static bool tag_sort(neu_tag_sort_t *sort, void *tag, void *tag_to_be_sorted);
static int  tag_cmp_write(neu_tag_sort_elem_t *tag1, neu_tag_sort_elem_t *tag2);
static bool tag_sort_write(neu_tag_sort_t *sort, void *tag,
                           void *tag_to_be_sorted);

int s7_tag_to_point(const neu_datatag_t *tag, s7_point_t *point)
{
    int      ret           = NEU_ERR_SUCCESS;
    uint32_t start_address = 0;
    ret                    = neu_datatag_parse_addr_option(tag, &point->option);
    if (ret != 0) {
        return NEU_ERR_TAG_ADDRESS_FORMAT_INVALID;
    }

    //AREA ADDRESS[.BIT][.LEN]

    int  n    = sscanf(tag->address, "DB%hu.DBW%u", &point->dbnumber,&start_address);
    if (n != 2 || start_address == 0 || point->dbnumber <= 0) {
        return NEU_ERR_TAG_ADDRESS_FORMAT_INVALID;
    }

    point->area = S7AreaDB;
    point->start_address = (uint16_t) start_address;

    point->start_address -= 1;
    point->type = tag->type;

    switch (point->type) {
    case NEU_TYPE_BIT:
        point->n_register = 1;
        break;
    case NEU_TYPE_UINT8:
    case NEU_TYPE_INT8:
        point->n_register = 1;
        break;
    case NEU_TYPE_UINT16:
    case NEU_TYPE_INT16:
        point->n_register = 2;
        break;
    case NEU_TYPE_UINT32:
    case NEU_TYPE_INT32:
    case NEU_TYPE_FLOAT:
        point->n_register = 4;
        break;
    case NEU_TYPE_UINT64:
    case NEU_TYPE_INT64:
    case NEU_TYPE_DOUBLE:
        point->n_register = 8;
        break;
    case NEU_TYPE_STRING:
        if (point->option.string.length > 127) {
            return NEU_ERR_TAG_ADDRESS_FORMAT_INVALID;
        }
        switch (point->option.string.type) {
        case NEU_DATATAG_STRING_TYPE_H:
        case NEU_DATATAG_STRING_TYPE_L:
            point->n_register = point->option.string.length;
            break;
        case NEU_DATATAG_STRING_TYPE_D:
        case NEU_DATATAG_STRING_TYPE_E:
            point->n_register = point->option.string.length;
            break;
        }
        break;
    case NEU_TYPE_BYTES:
        // if (point->option.bytes.length > 128 ||
        //     point->option.bytes.length % 2 == 1) {
        //     return NEU_ERR_TAG_ADDRESS_FORMAT_INVALID;
        // }
        point->n_register = point->option.bytes.length;
        break;
    default:
        return NEU_ERR_TAG_TYPE_NOT_SUPPORT;
    }

    strncpy(point->name, tag->name, sizeof(point->name));
    return ret;
}

int s7_write_tag_to_point(const neu_plugin_tag_value_t *tag,
                              s7_point_write_t *        point)
{
    int ret      = NEU_ERR_SUCCESS;
    ret          = s7_tag_to_point(tag->tag, &point->point);
    point->value = tag->value;
    return ret;
}

int sorts_cmp(const void *a, const void *b) {
    neu_tag_sort_t *sort_a = (neu_tag_sort_t *)a;
    neu_tag_sort_t *sort_b = (neu_tag_sort_t *)b;

    if (sort_a->info.size < sort_b->info.size) return -1;
    if (sort_a->info.size > sort_b->info.size) return 1;
    return 0;
}

s7_read_cmd_sort_t *s7_tag_sort(UT_array *tags, uint16_t max_byte)
{
    s7_read_max_byte          = max_byte;
    neu_tag_sort_result_t *result = neu_tag_sort(tags, tag_sort, tag_cmp);

    //tag_sort再排序,按每个组的大小,由小到大排序
    qsort(result->sorts, result->n_sort, sizeof(neu_tag_sort_t), sorts_cmp);

    s7_read_cmd_sort_t *sort_result = calloc(1, sizeof(s7_read_cmd_sort_t));
    sort_result->cmd   = calloc(result->n_sort, sizeof(s7_read_cmd_t));
    
    for (uint16_t i = 0; i < result->n_sort; i++) {
        sort_result->cmd[i].tags = calloc(MaxVars,sizeof(UT_array*));
        for (size_t j = 0; j < MaxVars; j++) {
            utarray_new(sort_result->cmd[i].tags[j], &ut_ptr_icd);
        }
    }

    int cmd_idx = 0,item_idx= 0,res_len = 0;
    for (uint16_t i = 0; i < result->n_sort; i++) {
        s7_point_t *tag = *(s7_point_t **) utarray_front(result->sorts[i].tags);
        struct s7_sort_ctx *ctx = result->sorts[i].info.context;

        //检测是否超过最大读取长度,超过则重新分配cmd
        res_len += ctx->end - ctx->start;
        if(res_len > s7_read_max_byte){
            cmd_idx++;
            item_idx = 0;
            res_len = ctx->end - ctx->start;
        }

        sort_result->cmd[cmd_idx].item_num = item_idx+1;
        sort_result->cmd[cmd_idx].tags[item_idx] = utarray_clone(result->sorts[i].tags);
        sort_result->cmd[cmd_idx].item[item_idx].dbnumber  = tag->dbnumber;
        sort_result->cmd[cmd_idx].item[item_idx].area     = tag->area;
        sort_result->cmd[cmd_idx].item[item_idx].start_address = tag->start_address;
        sort_result->cmd[cmd_idx].item[item_idx].n_register    = ctx->end - ctx->start;
        item_idx++;

        free(result->sorts[i].info.context);
    }
    sort_result->n_cmd = cmd_idx+1;

    neu_tag_sort_free(result);
    return sort_result;
}

int cal_n_byte(int type, neu_value_u *value, neu_datatag_addr_option_u option)
{
    int n = 0;
    switch (type) {
    case NEU_TYPE_UINT16:
    case NEU_TYPE_INT16:
        n          = sizeof(uint16_t);
        value->u16 = htons(value->u16);
        break;
    case NEU_TYPE_FLOAT:
    case NEU_TYPE_UINT32:
    case NEU_TYPE_INT32:
        n          = sizeof(uint32_t);
        value->u32 = htonl(value->u32);
        break;

    case NEU_TYPE_DOUBLE:
    case NEU_TYPE_INT64:
    case NEU_TYPE_UINT64:
        n          = sizeof(uint64_t);
        value->u64 = neu_htonll(value->u64);
        break;
    case NEU_TYPE_BIT: {
        n = sizeof(uint8_t);
        break;
    }
    case NEU_TYPE_STRING: {
        n = option.string.length;
        switch (option.string.type) {
        case NEU_DATATAG_STRING_TYPE_H:
            break;
        case NEU_DATATAG_STRING_TYPE_L:
            neu_datatag_string_ltoh(value->str, option.string.length);
            break;
        case NEU_DATATAG_STRING_TYPE_D:
            break;
        case NEU_DATATAG_STRING_TYPE_E:
            break;
        }
        break;
    }
    case NEU_TYPE_BYTES: {
        n = option.bytes.length;
        break;
    }
    default:
        // assert(1 == 0);
        break;
    }
    return n;
}

s7_write_cmd_sort_t *s7_write_tags_sort(UT_array *tags)
{
    neu_tag_sort_result_t *result =
        neu_tag_sort(tags, tag_sort_write, tag_cmp_write);

    s7_write_cmd_sort_t *sort_result =
        calloc(1, sizeof(s7_write_cmd_sort_t));
    sort_result->n_cmd = result->n_sort;
    sort_result->cmd   = calloc(result->n_sort, sizeof(s7_write_cmd_t));
    for (uint16_t i = 0; i < result->n_sort; i++) {
        s7_point_write_t *tag =
            *(s7_point_write_t **) utarray_front(result->sorts[i].tags);
        struct s7_sort_ctx *ctx = result->sorts[i].info.context;

        int num_tags              = utarray_len(result->sorts[i].tags);
        sort_result->cmd[i].bytes = calloc(num_tags, sizeof(neu_value_u));
        int      n_byte = 0;
        uint8_t *data_bit = calloc((num_tags + 7) / 8, sizeof(uint8_t));
        // int      k        = 0;
        utarray_foreach(result->sorts[i].tags, s7_point_write_t **, tag_s)
        {
            // if ((*tag_s)->point.area == S7_AREA_COIL) {
            //     n_byte_tag = cal_n_byte((*tag_s)->point.type, &(*tag_s)->value,
            //                             (*tag_s)->point.option);
            //     data_bit[k / 8] += ((*tag_s)->value.i8) << k % 8;
            //     n_byte += n_byte_tag;
            //     k++;
            // } else {
            //     n_byte_tag = cal_n_byte((*tag_s)->point.type, &(*tag_s)->value,
            //                             (*tag_s)->point.option);
            //     memcpy(sort_result->cmd[i].bytes +
            //                2 *
            //                    ((*tag_s)->point.start_address -
            //                     tag->point.start_address),
            //            &((*tag_s)->value), n_byte_tag);
            //     n_byte += n_byte_tag;
            // }
        }
        // if ((*(s7_point_write_t **) utarray_front(result->sorts[i].tags))
        //         ->point.area == S7_AREA_COIL) {
        //     memcpy(sort_result->cmd[i].bytes, data_bit, (k + 7) / 8);
        // }

        sort_result->cmd[i].tags     = utarray_clone(result->sorts[i].tags);
        sort_result->cmd[i].dbnumber = tag->point.dbnumber;
        sort_result->cmd[i].area     = tag->point.area;
        sort_result->cmd[i].start_address = tag->point.start_address;
        sort_result->cmd[i].n_register    = ctx->end - ctx->start;
        sort_result->cmd[i].n_byte        = n_byte;

        free(data_bit);
        free(result->sorts[i].info.context);
    }

    neu_tag_sort_free(result);
    return sort_result;
}

void s7_tag_sort_free(s7_read_cmd_sort_t *cs)
{
    for (uint16_t i = 0; i < cs->n_cmd; i++) {
        for (size_t j = 0; j < MaxVars; j++)
        {
            utarray_free(cs->cmd[i].tags[j]);
        }
        free(cs->cmd[i].tags);
    }

    free(cs->cmd);
    free(cs);
}

static int tag_cmp(neu_tag_sort_elem_t *tag1, neu_tag_sort_elem_t *tag2)
{
    s7_point_t *p_t1 = (s7_point_t *) tag1->tag;
    s7_point_t *p_t2 = (s7_point_t *) tag2->tag;

    if (p_t1->area > p_t2->area) {
        return 1;
    } else if (p_t1->area < p_t2->area) {
        return -1;
    }

    if (p_t1->dbnumber > p_t2->dbnumber) {
        return 1;
    } else if (p_t1->dbnumber < p_t2->dbnumber) {
        return -1;
    }

    if (p_t1->start_address > p_t2->start_address) {
        return 1;
    } else if (p_t1->start_address < p_t2->start_address) {
        return -1;
    }

    if (p_t1->n_register > p_t2->n_register) {
        return 1;
    } else if (p_t1->n_register < p_t2->n_register) {
        return -1;
    }

    return 0;
}

static bool tag_sort(neu_tag_sort_t *sort, void *tag, void *tag_to_be_sorted)
{
    s7_point_t *        t1  = (s7_point_t *) tag;
    s7_point_t *        t2  = (s7_point_t *) tag_to_be_sorted;
    struct s7_sort_ctx *ctx = NULL;

    if (sort->info.context == NULL) {
        sort->info.context = calloc(1, sizeof(struct s7_sort_ctx));
        ctx                = (struct s7_sort_ctx *) sort->info.context;
        ctx->start         = t1->start_address;
        ctx->end           = t1->start_address + t1->n_register;
        return true;
    }

    ctx = (struct s7_sort_ctx *) sort->info.context;

    if (t1->area != t2->area) {
        return false;
    }

    if (t1->dbnumber != t2->dbnumber) {
        return false;
    }

    if (t2->start_address > ctx->end) {
        return false;
    }
    //计算返回数据长度是否超过s7_read_max_byte
    uint16_t res_len   = (ctx->end - ctx->start) + t2->n_register;
    if (res_len >= s7_read_max_byte) {
        return false;
    }

    if (t2->start_address + t2->n_register > ctx->end) {
        ctx->end = t2->start_address + t2->n_register;
    }

    return true;
}

static int tag_cmp_write(neu_tag_sort_elem_t *tag1, neu_tag_sort_elem_t *tag2)
{
    s7_point_write_t *p_t1 = (s7_point_write_t *) tag1->tag;
    s7_point_write_t *p_t2 = (s7_point_write_t *) tag2->tag;

    if (p_t1->point.area > p_t2->point.area) {
        return 1;
    } else if (p_t1->point.area < p_t2->point.area) {
        return -1;
    }

    if (p_t1->point.dbnumber > p_t2->point.dbnumber) {
        return 1;
    } else if (p_t1->point.dbnumber < p_t2->point.dbnumber) {
        return -1;
    }

    if (p_t1->point.start_address > p_t2->point.start_address) {
        return 1;
    } else if (p_t1->point.start_address < p_t2->point.start_address) {
        return -1;
    }

    if (p_t1->point.n_register > p_t2->point.n_register) {
        return 1;
    } else if (p_t1->point.n_register < p_t2->point.n_register) {
        return -1;
    }

    return 0;
}

static bool tag_sort_write(neu_tag_sort_t *sort, void *tag,
                           void *tag_to_be_sorted)
{
    s7_point_write_t *  t1  = (s7_point_write_t *) tag;
    s7_point_write_t *  t2  = (s7_point_write_t *) tag_to_be_sorted;
    struct s7_sort_ctx *ctx = NULL;

    if (sort->info.context == NULL) {
        sort->info.context = calloc(1, sizeof(struct s7_sort_ctx));
        ctx                = (struct s7_sort_ctx *) sort->info.context;
        ctx->start         = t1->point.start_address;
        ctx->end           = t1->point.start_address + t1->point.n_register;
        return true;
    }

    ctx = (struct s7_sort_ctx *) sort->info.context;
    if (t1->point.area != t2->point.area) {
        return false;
    }

    if (t1->point.dbnumber != t2->point.dbnumber) {
        return false;
    }

    if (t2->point.start_address > ctx->end) {
        return false;
    }


    if (t2->point.start_address + t2->point.n_register > ctx->end) {
        ctx->end = t2->point.start_address + t2->point.n_register;
    }

    return true;
}