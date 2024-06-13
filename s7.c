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
#include <netinet/in.h>

#include <neuron.h>

#include "s7.h"
#include "snap7_isotcp.h"
// #include "snap7_types.h"

const int S7WLBit     = 0x01;
const int S7WLByte    = 0x02;
const int S7WLChar    = 0x03;
const int S7WLWord    = 0x04;
const int S7WLInt     = 0x05;
const int S7WLDWord   = 0x06;
const int S7WLDInt    = 0x07;
const int S7WLReal    = 0x08;
const int S7WLCounter = 0x1C;
const int S7WLTimer   = 0x1D;


// Result transport size
const byte TS_ResBit   = 0x03;
const byte TS_ResByte  = 0x04;
const byte TS_ResInt   = 0x05;
const byte TS_ResReal  = 0x07;
const byte TS_ResOctet = 0x09;

// PDU Type
const byte PduType_request      = 1;      // family request
const byte PduType_responseNofield = 2;      // family response
const byte PduType_response     = 3;      // family response
const byte PduType_userdata     = 7;      // family user data

// PDU Functions
const byte pduResponse    	= 0x02;   // Response (when error)
const byte pduFuncRead    	= 0x04;   // Read area
const byte pduFuncWrite   	= 0x05;   // Write area
const byte pduNegotiate   	= 0xF0;   // Negotiate PDU length
const byte pduStart         = 0x28;   // CPU start
const byte pduStop          = 0x29;   // CPU stop
const byte pduStartUpload   = 0x1D;   // Start Upload
const byte pduUpload        = 0x1E;   // Upload
const byte pduEndUpload     = 0x1F;   // EndUpload
const byte pduReqDownload   = 0x1A;   // Start Download request
const byte pduDownload      = 0x1B;   // Download request
const byte pduDownloadEnded = 0x1C;   // Download end request
const byte pduControl   	= 0x28;   // Control (insert/delete..)

const longword errIsoMask    	        = 0x000F0000;
const longword errIsoBase               = 0x0000FFFF;

const longword errIsoConnect            = 0x00010000; // Connection error
const longword errIsoDisconnect         = 0x00020000; // Disconnect error
const longword errIsoInvalidPDU         = 0x00030000; // Bad format
const longword errIsoInvalidDataSize    = 0x00040000; // Bad Datasize passed to send/recv : buffer is invalid
const longword errIsoNullPointer    	= 0x00050000; // Null passed as pointer
const longword errIsoShortPacket    	= 0x00060000; // A short packet received
const longword errIsoTooManyFragments   = 0x00070000; // Too many packets without EoT flag
const longword errIsoPduOverflow    	= 0x00080000; // The sum of fragments data exceded maximum packet size
const longword errIsoSendPacket         = 0x00090000; // An error occurred during send
const longword errIsoRecvPacket         = 0x000A0000; // An error occurred during recv
const longword errIsoInvalidParams    	= 0x000B0000; // Invalid TSAP params
const longword errIsoResvd_1    	    = 0x000C0000; // Unassigned
const longword errIsoResvd_2    	    = 0x000D0000; // Unassigned
const longword errIsoResvd_3    	    = 0x000E0000; // Unassigned
const longword errIsoResvd_4    	    = 0x000F0000; // Unassigned

const longword ISO_OPT_TCP_NODELAY   	= 0x00000001; // Disable Nagle algorithm
const longword ISO_OPT_INSIDE_MTU    	= 0x00000002; // Max packet size < MTU ethernet card

// PDU Type consts (Code + Credit)
const byte pdu_type_CR    	= 0xE0;  // Connection request
const byte pdu_type_CC    	= 0xD0;  // Connection confirm
const byte pdu_type_DR    	= 0x80;  // Disconnect request
const byte pdu_type_DC    	= 0xC0;  // Disconnect confirm
const byte pdu_type_DT    	= 0xF0;  // Data transfer

const byte pdu_EoT    		= 0x80;  // End of Trasmission Packet (This packet is complete)

const longword DataHeaderSize  = sizeof(TTPKT)+sizeof(S7_TCOTP_DT);
const longword IsoFrameSize    = IsoPayload_Size+DataHeaderSize;


	// int BuildControlPDU(TIsoControlPDU *pIsoControlPDU);

word SrcTSap = 0x0100;  // Source TSAP
word DstTSap = 0x0100;  // Destination TSAP RemoteTSAP = (0x01<<8)+(Rack*0x20)+Slot;
word SrcRef  = 0x0100;   // Source Reference
word DstRef  = 0x0000;   // Destination Reference
word cntword = 0x0001;   // Counter for PDU sequence

bool LittleEndian = true; // Little Endian by default
int IsoPDUSize = 1024;
int LastIsoError;
int LastTcpError;


void s7_cotp_con_warap(neu_protocol_pack_buf_t *buf,uint8_t *base)
{
    TIsoControlPDU pdu;
    int ret_size = s7_stack_BuildControlPDU(&pdu);
    // printf("cotp connection build size:%d\n",ret_size);
    memcpy(base, &pdu, ret_size);
    buf->size = ret_size;
    buf->offset = 0;
}

void s7_s7com_con_warap(neu_protocol_pack_buf_t *buf,uint8_t *base)
{
    TIsoDataPDU pdu;
    int ret_size = s7_stack_NegotiatePDU(&pdu);
    // printf("cotp s7 comm build size:%d\n",ret_size);
    memcpy(base, &pdu, ret_size);
    buf->size = ret_size;
    buf->offset = 0;
}
void s7_s7com_multiread_warap(neu_protocol_pack_buf_t *buf,uint8_t *base,s7_read_cmd_t *cmd,uint16_t pdu_size)
{
    TIsoDataPDU pdu;
    int ret_size = s7_stack_ReadMultiVars(&pdu,cmd,pdu_size);
    memcpy(base, &pdu, ret_size);
    buf->size = ret_size;
    buf->offset = 0;
}

void s7_s7com_mutilwrite_warap(neu_protocol_pack_buf_t *buf,uint8_t *base,uint16_t dbnumber,
                       enum s7_area area, uint16_t start_address,
                       uint16_t n_reg, uint8_t *bytes,uint16_t pdu_size)
{
    TIsoDataPDU pdu;
    int ret_size = s7_stack_WriteMultiVars(&pdu,dbnumber,area,start_address,n_reg,bytes,pdu_size);
    memcpy(base, &pdu, ret_size);
    buf->size = ret_size;
    buf->offset = 0;
}

static uint16_t calcrc(uint8_t *buf, int len)
{
    uint16_t crc     = 0xffff;
    uint16_t crcpoly = 0xa001;

    for (int i = 0; i < len; i++) {
        uint8_t x = buf[i];
        crc ^= x;
        for (int k = 0; k < 8; k++) {
            uint16_t usepoly = crc & 0x1;
            crc >>= 1;
            if (usepoly)
                crc ^= crcpoly;
        }
    }

    return crc;
}

void s7_header_wrap(neu_protocol_pack_buf_t *buf)
{
    assert(neu_protocol_pack_buf_unused_size(buf) >=
           sizeof(struct S7_TPTK));
    struct S7_TPTK *header =
        (struct S7_TPTK *) neu_protocol_pack_buf(
            buf, sizeof(struct S7_TPTK));

    header->Version     = 0x3;
    header->Reserved    = 0x0;
    header->HI_Lenght   = 0x0;
    header->LO_Lenght   = 0x0;
    // header->len      = htons(neu_protocol_pack_buf_used_size(buf) -
    //                     sizeof(struct S7_TPTK));
}

void s7_code_wrap(neu_protocol_pack_buf_t *buf, uint8_t slave_id,
                      uint8_t function)
{
    assert(neu_protocol_pack_buf_unused_size(buf) >=
           sizeof(struct s7_code));
    struct s7_code *code = (struct s7_code *) neu_protocol_pack_buf(
        buf, sizeof(struct s7_code));

    code->slave_id = slave_id;
    code->function = function;
}

u_char s7_cotp_pdutype_get(neu_protocol_unpack_buf_t *buf)
{
    // uint16_t tcotp_offset = 2;
    return *(buf->base + buf->offset + 2 -1);
}

u_char s7_res_funcode_get(neu_protocol_unpack_buf_t *buf)
{
    // uint16_t 功能码 = 1;
    if(buf->size < (buf->offset + 1))
        return -1;
    return *(buf->base + buf->offset + 1 -1);
}


//解析cotp_co 返回0 正常,返回-1 异常
int s7_cotp_co_unwrap(neu_protocol_unpack_buf_t *buf, S7_TCOTP_CO * cotp_co)
{
    int cotp_co_size = 0x12; //sizeof(S7_TCOTP_CO)
    S7_TCOTP_CO *tmp = (S7_TCOTP_CO *) neu_protocol_unpack_buf(buf, cotp_co_size);
    if (tmp == NULL) {
        return -1;
    }

    if(tmp->PDUType == S7_COTP_CO_CONFIRM)
    {
        *cotp_co = *tmp;
        return 0;
    }else
        return -2;
}

int s7_cotp_dt_unwrap(neu_protocol_unpack_buf_t *buf, S7_TCOTP_DT * cotp_dt)
{
    //将buf->base+buf->offset,0x3 长度数据 memcpy cotp_dt
    S7_TCOTP_DT *tmp = (S7_TCOTP_DT *) neu_protocol_unpack_buf(buf, sizeof(S7_TCOTP_DT));    
    if (tmp == NULL) {
        return -1;
    }
    if(tmp->PDUType == S7_COTP_DT_DATA)
    {
        *cotp_dt = *tmp;
        return 0;
    }else
        return -1;
}

int  s7_res_header23_unwrap(neu_protocol_unpack_buf_t *buf, TS7ResHeader23 *ps7res_header)
{
    TS7ResHeader23 *tmp = (TS7ResHeader23 *) neu_protocol_unpack_buf(buf, sizeof(TS7ResHeader23));    
    if (tmp == NULL) {
        return -1;
    }
    *ps7res_header = *tmp;
    if(ps7res_header->Error != 0)
    {
        ps7res_header->Error = ntohs(ps7res_header->Error);
        return -1;
    }

    return 0;
}

int  s7_res_nego_param_unwrap(neu_protocol_unpack_buf_t *buf, TResFunNegotiateParams *param)
{
    TResFunNegotiateParams *tmp = (TResFunNegotiateParams *) neu_protocol_unpack_buf(buf, sizeof(TResFunNegotiateParams));
    if (tmp == NULL) {
        return -1;
    }

    *param = *tmp;
    //进行字节序转换 word
    param->ParallelJobs_1 = ntohs(param->ParallelJobs_1);
    param->ParallelJobs_2 = ntohs(param->ParallelJobs_2);
    param->PDULength = ntohs(param->PDULength);
    return 0;
}

int  s7_res_read_param_unwrap(neu_protocol_unpack_buf_t *buf, TResFunReadParams *param)
{
    TResFunReadParams *tmp = (TResFunReadParams *) neu_protocol_unpack_buf(buf, sizeof(TResFunReadParams));
    if (tmp == NULL) {
        return -1;
    }

    *param = *tmp;
    return 0;
}

int  s7_res_read_item_unwrap(neu_protocol_unpack_buf_t *buf, TResFunReadItem *param)
{  
    int item_header = 4; //读取数据的每个item的头部大小
    TResFunReadItem *tmp = (TResFunReadItem *) neu_protocol_unpack_buf(buf, item_header);
    if (tmp == NULL) {
        return -1;
    }

    param->DataLength = ntohs(tmp->DataLength);
    param->ReturnCode = tmp->ReturnCode;
    param->TransportSize = tmp->TransportSize;

    int offset = 0;
    if ((param->TransportSize != TS_ResOctet) && (param->TransportSize != TS_ResReal) && (param->TransportSize != TS_ResBit))
        offset += (param->DataLength >> 3);
    else
    {
        //TODO 检查,DataLength是bit数 
        offset += (param->DataLength/8+param->DataLength%8?1:0);
    }
        
    memcpy(param->Data, tmp->Data, offset);
    buf->offset += offset;
        
    return 0;
}

int  s7_res_write_item_unwrap(neu_protocol_unpack_buf_t *buf, byte *param,int item_cnt)
{
    int offset = item_cnt;
    if(buf->size < (buf->offset + offset))
        return -1;
    memcpy(param, buf->base + buf->offset, offset);
    buf->offset += offset;
        
    return 0;
}

int s7_cotp_unwrap(neu_protocol_unpack_buf_t *buf,
                       struct s7_code *       out_code)
{
    struct s7_code *code = (struct s7_code *) neu_protocol_unpack_buf(
        buf, sizeof(struct s7_code));

    if (code == NULL) {
        return 0;
    }

    *out_code = *code;

    return sizeof(struct s7_code);
}

void s7_address_wrap(neu_protocol_pack_buf_t *buf, uint16_t start,
                         uint16_t n_register, enum s7_action action)
{
    if (action == S7_ACTION_HOLD_REG_WRITE && n_register == 1) {
        assert(neu_protocol_pack_buf_unused_size(buf) >= sizeof(start));
        uint16_t *address =
            (uint16_t *) neu_protocol_pack_buf(buf, sizeof(start));

        *address = htons(start);
    } else {
        assert(neu_protocol_pack_buf_unused_size(buf) >=
               sizeof(struct s7_address));
        struct s7_address *address =
            (struct s7_address *) neu_protocol_pack_buf(
                buf, sizeof(struct s7_address));

        address->start_address = htons(start);
        address->n_reg         = htons(n_register);
    }
}

int s7_address_unwrap(neu_protocol_unpack_buf_t *buf,
                          struct s7_address *    out_address)
{
    struct s7_address *address =
        (struct s7_address *) neu_protocol_unpack_buf(
            buf, sizeof(struct s7_address));

    if (address == NULL) {
        return 0;
    }

    *out_address               = *address;
    out_address->start_address = ntohs(out_address->start_address);
    out_address->n_reg         = ntohs(out_address->n_reg);

    return sizeof(struct s7_address);
}

void s7_data_wrap(neu_protocol_pack_buf_t *buf, uint8_t n_byte,
                      uint8_t *bytes, enum s7_action action)
{
    assert(neu_protocol_pack_buf_unused_size(buf) >=
           sizeof(struct s7_data) + n_byte);
    uint8_t *data = neu_protocol_pack_buf(buf, n_byte);

    memcpy(data, bytes, n_byte);

    if (action == S7_ACTION_DEFAULT || n_byte > 2) {
        struct s7_data *mdata =
            (struct s7_data *) neu_protocol_pack_buf(
                buf, sizeof(struct s7_data));

        mdata->n_byte = n_byte;
    }
}

int s7_data_unwrap(neu_protocol_unpack_buf_t *buf,
                       struct s7_data *       out_data)
{
    struct s7_data *mdata = (struct s7_data *) neu_protocol_unpack_buf(
        buf, sizeof(struct s7_data));

    if (mdata == NULL ||
        mdata->n_byte > neu_protocol_unpack_buf_unused_size(buf)) {
        return 0;
    }

    *out_data = *mdata;

    return sizeof(struct s7_data);
}

void s7_crc_set(neu_protocol_pack_buf_t *buf)
{
    uint16_t  crc   = calcrc(neu_protocol_pack_buf_get(buf),
                          neu_protocol_pack_buf_used_size(buf) - 2);
    uint16_t *p_crc = (uint16_t *) neu_protocol_pack_buf_set(
        buf, neu_protocol_pack_buf_used_size(buf) - 2, 2);

    *p_crc = crc;
}

void s7_crc_wrap(neu_protocol_pack_buf_t *buf)
{
    assert(neu_protocol_pack_buf_unused_size(buf) >= sizeof(struct s7_crc));
    struct s7_crc *crc = (struct s7_crc *) neu_protocol_pack_buf(
        buf, sizeof(struct s7_crc));

    crc->crc = 0;
}

int s7_crc_unwrap(neu_protocol_unpack_buf_t *buf,
                      struct s7_crc *        out_crc)
{
    struct s7_crc *crc = (struct s7_crc *) neu_protocol_unpack_buf(
        buf, sizeof(struct s7_crc));

    if (crc == NULL) {
        return 0;
    }

    *out_crc = *crc;

    return sizeof(struct s7_crc);
}

const char *s7_area_to_str(s7_area_e area)
{
    // S7AreaPE   =	0x81,
    // S7AreaPA   =	0x82,
    // S7AreaMK   =	0x83,
    // S7AreaDB   =	0x84,
    // S7AreaCT   =	0x1C,
    // S7AreaTM   =	0x1D
    switch (area) {
        case S7AreaPE:
            return "PE";
        case S7AreaPA:
            return "PA";
        case S7AreaMK:
            return "MK";
        case S7AreaDB:
            return "DB";
        case S7AreaCT:
            return "CT";
        case S7AreaTM:
            return "TM";
        default:
            return "UNKNOWN";
    }
}


//暂定rack 0/slot 1 DstTSap = 0x0100
int s7_stack_BuildControlPDU(TIsoControlPDU *pIsoControlPDU)
{
	int ParLen, IsoLen;
	pIsoControlPDU->COTP.Params.PduSizeCode=0xC0; // code that identifies TPDU size
	pIsoControlPDU->COTP.Params.PduSizeLen =0x01; // 1 byte this field
	pIsoControlPDU->COTP.Params.PduSizeVal =0x0A;
	// Build TSAPs
	pIsoControlPDU->COTP.Params.TSAP[0]=0xC1;   // code that identifies source TSAP
	pIsoControlPDU->COTP.Params.TSAP[1]=2;      // source TSAP Len
	pIsoControlPDU->COTP.Params.TSAP[2]=(SrcTSap>>8) & 0xFF; // HI part
	pIsoControlPDU->COTP.Params.TSAP[3]=SrcTSap & 0xFF; // LO part

    // word RemoteTSAP = (0x01<<8)+(Rack*0x20)+Slot;

	pIsoControlPDU->COTP.Params.TSAP[4]=0xC2; // code that identifies dest TSAP
	pIsoControlPDU->COTP.Params.TSAP[5]=2;    // dest TSAP Len
	pIsoControlPDU->COTP.Params.TSAP[6]=(DstTSap>>8) & 0xFF; // HI part
	pIsoControlPDU->COTP.Params.TSAP[7]=DstTSap & 0xFF; // LO part

	// Params length
	ParLen=11;            // 2 Src TSAP (Code+field Len)      +
						  // 2 Src TSAP len                   +
						  // 2 Dst TSAP (Code+field Len)      +
						  // 2 Src TSAP len                   +
						  // 3 PDU size (Code+field Len+Val)  = 11
	// Telegram length
	IsoLen=sizeof(TTPKT)+ // TPKT Header
			7 +           // COTP Header Size without params
			ParLen;       // COTP params

	pIsoControlPDU->TPKT.Version  =0x3; // RFC 1006
	pIsoControlPDU->TPKT.Reserved =0;
	pIsoControlPDU->TPKT.HI_Lenght=0; // Connection Telegram size cannot exced 255 bytes, so
								  // this field is always 0
	pIsoControlPDU->TPKT.LO_Lenght=IsoLen;

	pIsoControlPDU->COTP.HLength  =ParLen + 6;  // <-- 6 = 7 - 1 (COTP Header size - 1)
	pIsoControlPDU->COTP.PDUType  =pdu_type_CR; // Connection Request
	pIsoControlPDU->COTP.DstRef   =DstRef;      // Destination reference
	pIsoControlPDU->COTP.SrcRef   =SrcRef;      // Source reference
	pIsoControlPDU->COTP.CO_R     =0x00;        // Class + Option : RFC0983 states that it must be always 0x40
											// but for some equipment (S7) must be 0 in disaccord of specifications !!!

                                            
    int pIsoControlPDU_size = pIsoControlPDU->TPKT.HI_Lenght*256+pIsoControlPDU->TPKT.LO_Lenght;
    // printf("pIsoControlPDU pIsoControlPDU_size %d\n", pIsoControlPDU_size);
	return pIsoControlPDU_size;
}

int s7_stack_NegotiatePDU(TIsoDataPDU *pIsoDataPDU)
{
    word PDURequest = 960;
    PReqFunNegotiateParams ReqNegotiate;

    TS7Answer17 DUH_out;
    // Setup Pointers
    ReqNegotiate = (PReqFunNegotiateParams)((pbyte)(&DUH_out) + sizeof(TS7ResHeader17));

    // Header
    DUH_out.Header.P        = 0x32;            // Always $32
    DUH_out.Header.PDUType  = 0x01; // $01
    DUH_out.Header.AB_EX    = 0x0000;          // Always $0000
    DUH_out.Header.Sequence = GetNextWord();   // AutoInc
    DUH_out.Header.ParLen   = SwapWord(sizeof(TReqFunNegotiateParams)); // 8 bytes
    DUH_out.Header.DataLen  = 0x0000;
    // Params
    ReqNegotiate->FunNegotiate = 0xF0;
    ReqNegotiate->Unknown = 0x00;
    ReqNegotiate->ParallelJobs_1 = 0x0100;
    ReqNegotiate->ParallelJobs_2 = 0x0100;
    ReqNegotiate->PDULength = SwapWord(PDURequest);
    int S7pduSize = sizeof( TS7ResHeader17 ) + sizeof( TReqFunNegotiateParams );

    u_int IsoSize =S7pduSize+DataHeaderSize;
	if ((IsoSize>0) && (IsoSize<=IsoFrameSize))
	{
		pIsoDataPDU->TPKT.Version  = isoTcpVersion;
		pIsoDataPDU->TPKT.Reserved = 0;
		pIsoDataPDU->TPKT.HI_Lenght= ((u_short)(IsoSize)>> 8) & 0xFF;
		pIsoDataPDU->TPKT.LO_Lenght= (u_short)(IsoSize) & 0xFF;
		// COPT
		pIsoDataPDU->COTP.HLength   =sizeof(S7_TCOTP_DT)-1;
		pIsoDataPDU->COTP.PDUType   =pdu_type_DT;
		pIsoDataPDU->COTP.EoT_Num   =pdu_EoT;
		// Fill payload

		memcpy(pIsoDataPDU->Payload, &DUH_out, IsoSize);
	}
    return IsoSize;
}

int s7_stack_ReadMultiVars(TIsoDataPDU *pIsoDataPDU,s7_read_cmd_t *cmd,uint16_t pdu_size)
{
    int ItemsCount = cmd->item_num;
    if (ItemsCount>MaxVars)
    	return -1;

    TReqFunReadParams ReqParams;
    word       RPSize    = (word)(2 + ItemsCount * sizeof(TReqFunReadItem));
    
    // Fill Header
    TS7ReqHeader ReqHeader;
    ReqHeader.P=0x32;                    // Always 0x32
    ReqHeader.PDUType=0x01;              // 0x01
    ReqHeader.AB_EX=0x0000;              // Always 0x0000
    ReqHeader.Sequence=GetNextWord();    // AutoInc
    ReqHeader.ParLen=SwapWord(RPSize);   // Request params size
    ReqHeader.DataLen=0x0000;            // No data in output

    // Fill Params
    ReqParams.FunRead=pduFuncRead;      // 0x04
    ReqParams.ItemsCount=ItemsCount;

    int c;
    for (c = 0; c < ItemsCount; c++)
    {
        ReqParams.Items[c].ItemHead[0]=0x12;
        ReqParams.Items[c].ItemHead[1]=0x0A;
        ReqParams.Items[c].ItemHead[2]=0x10;

        ReqParams.Items[c].TransportSize=S7WLByte;
        ReqParams.Items[c].Length=SwapWord(cmd->item[c].n_register);
        ReqParams.Items[c].Area=cmd->item[c].area;
        // TODO目前认为 dbnumber会拆tag
        ReqParams.Items[c].DBNumber=SwapWord(cmd->item[c].dbnumber);

        // Adjusts the offset
        longword   Address = cmd->item[c].start_address*8;

        // Builds the offset
        ReqParams.Items[c].Address[2]=Address & 0x000000FF;
        Address=Address >> 8;
        ReqParams.Items[c].Address[1]=Address & 0x000000FF;
        Address=Address >> 8;
        ReqParams.Items[c].Address[0]=Address & 0x000000FF;
        // Item++;
    };

    int IsoSize = RPSize+sizeof(TS7ReqHeader)+DataHeaderSize;
	if (IsoSize>pdu_size)
		return -2;

	if ((IsoSize>0) && (IsoSize<=(int)IsoFrameSize))
	{
		pIsoDataPDU->TPKT.Version  = isoTcpVersion;
		pIsoDataPDU->TPKT.Reserved = 0;
		pIsoDataPDU->TPKT.HI_Lenght= ((u_short)(IsoSize)>> 8) & 0xFF;
		pIsoDataPDU->TPKT.LO_Lenght= (u_short)(IsoSize) & 0xFF;
		// COPT
		pIsoDataPDU->COTP.HLength   =sizeof(S7_TCOTP_DT)-1;
		pIsoDataPDU->COTP.PDUType   =pdu_type_DT;
		pIsoDataPDU->COTP.EoT_Num   =pdu_EoT;
		// Fill payload

		memcpy(pIsoDataPDU->Payload,(pbyte)&ReqHeader,sizeof(TS7ReqHeader));
		memcpy(pIsoDataPDU->Payload+sizeof(TS7ReqHeader),(pbyte)&ReqParams, RPSize);
	}

    return IsoSize;
}

int s7_stack_WriteMultiVars(TIsoDataPDU *pIsoDataPDU,uint16_t dbnumber,
                       enum s7_area area, uint16_t start_address,
                       uint16_t n_reg, uint8_t *bytes,uint16_t pdu_size)
{
    TS7DataItem Item;
    Item.Area = area;
    Item.DBNumber = dbnumber;
    Item.Start = start_address;
    Item.Amount = n_reg;
    Item.WordLen = S7WLByte;
    Item.pdata = bytes;

    TReqFunWriteParams  ReqParams;
    uintptr_t          Offset;
    longword           Address;
    int                ItemsCount, c;
    word               RPSize; // ReqParams size
    word               Size;   // Write data size
    int                WordSize;

    ItemsCount = 1;

    if (ItemsCount>MaxVars)
    	return -1;

    RPSize    = (word)(2 + ItemsCount * sizeof(TReqFunWriteItem));

    // Fill Header
    TS7ReqHeader ReqHeader;
    ReqHeader.P=0x32;                    // Always 0x32
    ReqHeader.PDUType=PduType_request;              // 0x01
    ReqHeader.AB_EX=0x0000;              // Always 0x0000
    ReqHeader.Sequence=GetNextWord();    // AutoInc
    ReqHeader.ParLen=SwapWord(RPSize);   // Request params size
    ReqHeader.DataLen=0x0000;            // No data in output

    // Fill Params
    ReqParams.FunWrite=pduFuncWrite;      // 0x05
    ReqParams.ItemsCount=ItemsCount;

    Offset=0;

    TReqFunWriteDataItem ReqData[MaxVars];

    for (c = 0; c < ItemsCount; c++)
    {
        // Items Params
        ReqParams.Items[c].ItemHead[0]=0x12;
        ReqParams.Items[c].ItemHead[1]=0x0A;
        ReqParams.Items[c].ItemHead[2]=0x10;

        ReqParams.Items[c].TransportSize=Item.WordLen;
        ReqParams.Items[c].Length=SwapWord(Item.Amount);
        ReqParams.Items[c].Area=Item.Area;

        if (Item.Area==S7AreaDB)
            ReqParams.Items[c].DBNumber=SwapWord(Item.DBNumber);
        else
            ReqParams.Items[c].DBNumber=0x0000;

        // Adjusts the offset
        if ((Item.WordLen==S7WLBit) || (Item.WordLen==S7WLCounter) || (Item.WordLen==S7WLTimer))
        	Address=Item.Start;
        else
        	Address=Item.Start*8;
        // Builds the offset
        ReqParams.Items[c].Address[2]=Address & 0x000000FF;
        Address=Address >> 8;
        ReqParams.Items[c].Address[1]=Address & 0x000000FF;
        Address=Address >> 8;
        ReqParams.Items[c].Address[0]=Address & 0x000000FF;

        // Items Data
        // pbyte P = sizeof(ReqParams)+RPSize;
        // ReqData[c]=(PReqFunWriteDataItem)((pbyte)(P)+Offset);
        ReqData[c].ReturnCode=0x00;

        if (Item.WordLen == S7WLBit) {
            ReqData[c].TransportSize = TS_ResBit;
        } else if (Item.WordLen == S7WLInt || Item.WordLen == S7WLDInt) {
            ReqData[c].TransportSize = TS_ResInt;
        } else if (Item.WordLen == S7WLReal) {
            ReqData[c].TransportSize = TS_ResReal;
        } else if (Item.WordLen == S7WLChar || Item.WordLen == S7WLCounter || Item.WordLen == S7WLTimer) {
            ReqData[c].TransportSize = TS_ResOctet;
        } else {
            ReqData[c].TransportSize = TS_ResByte; // byte/word/dword etc.
        }

        WordSize=DataSizeByte(Item.WordLen);
        Size=Item.Amount * WordSize;

		if ((ReqData[c].TransportSize!=TS_ResOctet) && (ReqData[c].TransportSize!=TS_ResReal) && (ReqData[c].TransportSize!=TS_ResBit))
           ReqData[c].DataLength=SwapWord(Size*8);
        else
           ReqData[c].DataLength=SwapWord(Size);

        memcpy(ReqData[c].Data, Item.pdata, Size);

		if ((Size % 2) != 0 && (ItemsCount - c != 1))
			Size++; // Skip fill byte for Odd frame (except for the last one)

        Offset+=(4+Size); // next item
    };

    ReqHeader.DataLen=SwapWord((word)(Offset));

    int IsoSize=RPSize+sizeof(TS7ReqHeader)+(int)(Offset)+DataHeaderSize;

	if (IsoSize>pdu_size)
		return -2;

	if ((IsoSize>0) && (IsoSize<=(int)IsoFrameSize))
	{
		pIsoDataPDU->TPKT.Version  = isoTcpVersion;
		pIsoDataPDU->TPKT.Reserved = 0;
		pIsoDataPDU->TPKT.HI_Lenght= ((u_short)(IsoSize)>> 8) & 0xFF;
		pIsoDataPDU->TPKT.LO_Lenght= (u_short)(IsoSize) & 0xFF;
		// COPT
		pIsoDataPDU->COTP.HLength   =sizeof(S7_TCOTP_DT)-1;
		pIsoDataPDU->COTP.PDUType   =pdu_type_DT;
		pIsoDataPDU->COTP.EoT_Num   =pdu_EoT;
		// Fill payload

		memcpy(pIsoDataPDU->Payload,(pbyte)&ReqHeader,sizeof(TS7ReqHeader));
		memcpy(pIsoDataPDU->Payload+sizeof(TS7ReqHeader),(pbyte)&ReqParams, RPSize);
        memcpy(pIsoDataPDU->Payload+sizeof(TS7ReqHeader)+RPSize,(pbyte)ReqData, Offset);
	}
    
    return IsoSize;
}

int DataSizeByte(int WordLength)
{
	if (WordLength == S7WLBit) {
        return 1;  // S7 sends 1 byte per bit
    } else if (WordLength == S7WLByte) {
        return 1;
    } else if (WordLength == S7WLChar) {
        return 1;
    } else if (WordLength == S7WLWord) {
        return 2;
    } else if (WordLength == S7WLDWord) {
        return 4;
    } else if (WordLength == S7WLInt) {
        return 2;
    } else if (WordLength == S7WLDInt) {
        return 4;
    } else if (WordLength == S7WLReal) {
        return 4;
    } else if (WordLength == S7WLCounter) {
        return 2;
    } else if (WordLength == S7WLTimer) {
        return 2;
    } else {
        return 0;
    }
}

word GetNextWord()
{
     if (cntword==0xFFFF)
        cntword=0;
     return cntword++;
}

word SwapWord(word Value)
{
	if (LittleEndian)
		return  ((Value >> 8) & 0xFF) | ((Value << 8) & 0xFF00);
	else
	    return Value;
}
//---------------------------------------------------------------------------
longword SwapDWord(longword Value)
{
	if (LittleEndian)
		return (Value >> 24) | ((Value << 8) & 0x00FF0000) | ((Value >> 8) & 0x0000FF00) | (Value << 24);
	else
		return Value;
}

