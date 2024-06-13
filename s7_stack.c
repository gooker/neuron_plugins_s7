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
#include <assert.h>

#include <neuron.h>

#include "s7_req.h"
#include "s7_stack.h"

const byte PduTp_request      = 1;      // family request
const byte PduTp_responseNofield = 2;      // family response
const byte PduTp_response     = 3;      // family response
const byte PduTp_userdata     = 7;      // family user data

// PDU Functions
const byte s7FuncRead    	= 0x04;   // Read area
const byte s7FuncWrite   	= 0x05;   // Write area
const byte s7Negotiate   	= 0xF0;   // Negotiate PDU length

s7_stack_t *s7_stack_create(void *ctx, s7_protocol_e protocol,
                                    s7_stack_send       send_fn,
                                    s7_stack_value      value_fn,
                                    s7_stack_write_resp write_resp)
{
    s7_stack_t *stack = calloc(1, sizeof(s7_stack_t));

    stack->ctx        = ctx;
    stack->send_fn    = send_fn;
    stack->value_fn   = value_fn;
    stack->write_resp = write_resp;
    stack->protocol   = protocol;

    stack->buf_size = 256;
    stack->buf      = calloc(stack->buf_size, 1);

    stack->cotp_is_connected = false;
    stack->s7com_is_connected = false;
    stack->pdu_size = -1;

    return stack;
}

void s7_stack_destroy(s7_stack_t *stack)
{
    free(stack->buf);
    free(stack);
}

//数据包 协议iso on tcp. 以TPKT开头. 0x03 0x00 0x00 0x16
int s7_stack_tpkt_check(neu_protocol_unpack_buf_t *buf)
{
    struct S7_TPTK *s7_tptk = (struct S7_TPTK *) neu_protocol_unpack_buf(buf, sizeof(struct S7_TPTK));
    if(s7_tptk == NULL) {
        return -1;
    }

    //协议iso on tcp 固定0x3
    if(s7_tptk->Version != 0x03) {
        return -2;
    }

    uint16_t len = (s7_tptk->HI_Lenght << 8) | s7_tptk->LO_Lenght;
    if(len != buf->size){
        return -3;
    }
    return 0;
}

int s7_stack_recv(s7_stack_t *stack,neu_protocol_unpack_buf_t *buf)
{
    if(stack->protocol != S7_PROTOCOL_TCP) 
        return -1;

    int ret = s7_stack_tpkt_check(buf);
    if(ret < 0)
    {
        plog_warn((neu_plugin_t *) stack->ctx, "s7 s7_tptk unwrap fail:%d",ret);
        return -1;
    }

    //数据包 COTP,主要使用TCOTP_CO 和 S7_TCOTP_DT
    s7_tcotp_e PDUType = s7_cotp_pdutype_get(buf);
    switch (PDUType) {
        case S7_COTP_CO_CONFIRM:
            {
                S7_TCOTP_CO cotp_co;
                int co_ret = s7_cotp_co_unwrap(buf,&cotp_co);
                if(co_ret == 0){
                    stack->cotp_is_connected = true;
                }else{
                    plog_notice((neu_plugin_t *) stack->ctx,"s7 cotp_co unwrap fail:%d",co_ret);
                    return -1;
                }
                break;
            }
        case S7_COTP_DT_DATA: 
            {
    
                S7_TCOTP_DT cotp_dt;
                int co_ret = s7_cotp_dt_unwrap(buf,&cotp_dt);
                if(co_ret != 0 )
                {
                    plog_notice((neu_plugin_t *) stack->ctx,"s7 cotp_dt unwrap fail:%d",co_ret);
                    return -1;
                }
                
                //解析s7协议header,判断是否有错误
                TS7ResHeader23 s7res_header;
                int header_ret = s7_res_header23_unwrap(buf,&s7res_header);
                if(header_ret != 0)
                {
                    printf("s7 com err:0x%X\n",s7res_header.Error);
                    plog_warn((neu_plugin_t *) stack->ctx,"s7 com err:0x%X",s7res_header.Error);
                    return -1;
                }

                //解析不同应答类型,0x3响应job,0x2简单确认             
                if(s7res_header.PDUType == PduTp_response){
                    byte funcode = s7_res_funcode_get(buf);
                    if(funcode  == s7Negotiate)
                    {
                        TResFunNegotiateParams s7res_param;   
                        s7_res_nego_param_unwrap(buf,&s7res_param);
                        //S7 COM 握手成功
                        stack->s7com_is_connected = true;
                        stack->pdu_size = s7res_param.PDULength;
                        plog_notice((neu_plugin_t *) stack->ctx,"pdu size:%d,ParallelJobs_1:%d,ParallelJobs_2:%d",
                            s7res_param.PDULength,s7res_param.ParallelJobs_1,s7res_param.ParallelJobs_2);
                    }
                    else if(funcode == s7FuncRead)
                    {
                        TResFunReadParams s7res_param;
                        s7_res_read_param_unwrap(buf,&s7res_param);
                        plog_notice((neu_plugin_t *) stack->ctx,"s7 receive func:0x%X data:%d",s7res_param.FunRead,s7res_param.ItemCount);
                        for (size_t i = 0; i < s7res_param.ItemCount; i++)
                        {
                            TResFunReadItem s7res_item;
                            s7_res_read_item_unwrap(buf,&s7res_item);
                            plog_notice((neu_plugin_t *) stack->ctx,"s7 receive err:0x%X func:0x%X,len:%d data[0]:0x%X",
                                s7res_item.ReturnCode,s7res_item.TransportSize,s7res_item.DataLength>>3,s7res_item.Data[0]);
                            
                            int err = NEU_ERR_SUCCESS;
                            if(s7res_item.ReturnCode != 0xFF)
                            {
                                plog_warn((neu_plugin_t *) stack->ctx,"s7 com ReturnCode err:0x%X",s7res_item.ReturnCode);
                                err = NEU_ERR_PLUGIN_READ_FAILURE;
                            }

                            //对tag数据进行赋值
                            struct s7_data data = { 0 };
                            data.n_byte = s7res_item.DataLength>>3;
                            stack->value_fn(stack->ctx, i, data.n_byte, s7res_item.Data, err);
                        }

                    }else if(funcode == s7FuncWrite)
                    {
                        TResFunReadParams s7res_param;
                        s7_res_read_param_unwrap(buf,&s7res_param);

                        byte wret_Data[MaxVars];
                        int w_ret = s7_res_write_item_unwrap(buf,wret_Data,s7res_param.ItemCount);
                        if(w_ret != 0)
                        {
                            plog_warn((neu_plugin_t *) stack->ctx,"s7 res write item unwrap fail:%d",w_ret);
                            return -1;
                        }
                        else
                        {
                            bool w_failed = false;
                            for(size_t i = 0; i < s7res_param.ItemCount; i++)
                            {
                                if(wret_Data[i] != 0xFF)
                                {
                                    plog_warn((neu_plugin_t *) stack->ctx,"s7 res write item fail:%d",wret_Data[i]);
                                    w_failed = true;                              
                                }
                            }
                            if(w_failed)
                                return S7_DEVICE_ERR;
                        }

                    }else
                    {
                        plog_warn((neu_plugin_t *) stack->ctx,"s7 res unknown funcode:0x%X",funcode);
                        return 0;
                    }

                }else if(s7res_header.PDUType == PduTp_responseNofield)
                {
                    //S7 简单确认,暂时直接返回0
                    return 0;
                }else
                {
                    //其他暂时记录
                    plog_warn((neu_plugin_t *) stack->ctx,"s7 com unused s7res_header.PDUType:0x%X",s7res_header.PDUType);
                    return 0;
                }

                break;
            }
        case S7_COTP_DC_CONFIRM:
            {
                return 0;
            }
        case S7_COTP_CO_REQUEST:
        case S7_COTP_DC_REQUEST:
        default:
            return -1;
    }

    return neu_protocol_unpack_buf_used_size(buf);
}

int s7_stack_Handshake(s7_stack_t *stack)
{
    static __thread uint8_t                 buf[1024] = { 0 };
    static __thread neu_protocol_pack_buf_t pbuf    = { 0 };
    int                                     ret     = 0;
    neu_protocol_pack_buf_init(&pbuf, buf, sizeof(buf));

    //判断连接是否成功
    if(!stack->cotp_is_connected) {
        s7_cotp_con_warap(&pbuf,buf);
    }
    if(stack->cotp_is_connected && !stack->s7com_is_connected){
        s7_s7com_con_warap(&pbuf, buf);
    }
    if(stack->cotp_is_connected && stack->s7com_is_connected){
        return 0;
    }

    ret = stack->send_fn(stack->ctx, neu_protocol_pack_buf_used_size(&pbuf),
                         neu_protocol_pack_buf_get(&pbuf));
    if (ret <= 0) {
        stack->value_fn(stack->ctx, 0, 0, NULL, NEU_ERR_PLUGIN_DISCONNECTED);
        return -1;
    }

    return ret;
}

int s7_stack_read(s7_stack_t *stack, s7_read_cmd_t *cmd, uint16_t *response_size)
{
    static __thread uint8_t                 buf[1024] = { 0 };
    static __thread neu_protocol_pack_buf_t pbuf    = { 0 };
    int                                     ret     = 0;
    *response_size                                  = 0;

    neu_protocol_pack_buf_init(&pbuf, buf, sizeof(buf));

    if(stack->cotp_is_connected && stack->s7com_is_connected){
        s7_s7com_multiread_warap(&pbuf, buf,cmd,stack->pdu_size);
    }else
        return -1;

    ret = stack->send_fn(stack->ctx, neu_protocol_pack_buf_used_size(&pbuf),
                         neu_protocol_pack_buf_get(&pbuf));
        *response_size = ret;
    if (ret <= 0) {
        stack->value_fn(stack->ctx, 0, 0, NULL, NEU_ERR_PLUGIN_DISCONNECTED);
        plog_warn((neu_plugin_t *) stack->ctx, "send read req fail, %hhu!%hu",
                  cmd->reserve_id, cmd->item_num);
    }

    return ret;
}

int s7_stack_write(s7_stack_t *stack, void *req, uint16_t dbnumber,
                       enum s7_area area, uint16_t start_address,
                       uint16_t n_reg, uint8_t *bytes, uint8_t n_byte,
                       uint16_t *response_size, bool response)
{
    static __thread neu_protocol_pack_buf_t pbuf     = { 0 };

    memset(stack->buf, 0, stack->buf_size);
    neu_protocol_pack_buf_init(&pbuf, stack->buf, stack->buf_size);

    if(stack->cotp_is_connected && stack->s7com_is_connected){
        s7_s7com_mutilwrite_warap(&pbuf, stack->buf,dbnumber, area, start_address, n_reg, bytes, stack->buf_size);
    }else
        return -1;

    *response_size += sizeof(struct s7_code);

    int ret = stack->send_fn(stack->ctx, neu_protocol_pack_buf_used_size(&pbuf),
                             neu_protocol_pack_buf_get(&pbuf));
    if (ret > 0) {
        if (response) {
            stack->write_resp(stack->ctx, req, NEU_ERR_SUCCESS);
            plog_notice((neu_plugin_t *) stack->ctx, "send write req, %hu!%hu",
                        dbnumber, start_address);
        }
    } else {
        if (response) {
            stack->write_resp(stack->ctx, req, NEU_ERR_PLUGIN_DISCONNECTED);
            plog_warn((neu_plugin_t *) stack->ctx,
                      "send write req fail, %hu!%hu", dbnumber, start_address);
        }
    }
    return ret;
}

