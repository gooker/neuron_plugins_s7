/*=============================================================================|
|  PROJECT SNAP7                                                         1.3.0 |
|==============================================================================|
|  Copyright (C) 2013, 2015 Davide Nardella                                    |
|  All rights reserved.                                                        |
|==============================================================================|
|  SNAP7 is free software: you can redistribute it and/or modify               |
|  it under the terms of the Lesser GNU General Public License as published by |
|  the Free Software Foundation, either version 3 of the License, or           |
|  (at your option) any later version.                                         |
|                                                                              |
|  It means that you can distribute your commercial software linked with       |
|  SNAP7 without the requirement to distribute the source code of your         |
|  application and without the requirement that your application be itself     |
|  distributed under LGPL.                                                     |
|                                                                              |
|  SNAP7 is distributed in the hope that it will be useful,                    |
|  but WITHOUT ANY WARRANTY; without even the implied warranty of              |
|  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the               |
|  Lesser GNU General Public License for more details.                         |
|                                                                              |
|  You should have received a copy of the GNU General Public License and a     |
|  copy of Lesser GNU General Public License along with Snap7.                 |
|  If not, see  http://www.gnu.org/licenses/                                   |
|=============================================================================*/
#ifndef s7_isotcp_h
#define s7_isotcp_h
//---------------------------------------------------------------------------
// #include "snap7_msgsock.h"
#include "snap7_platform.h"
// #include "snap7_sysutils.h"
#include <stdbool.h>
//---------------------------------------------------------------------------
#pragma pack(1)

#define isoTcpVersion    	3      // RFC 1006
#define isoTcpPort    		102    // RFC 1006
#define isoInvalidHandle        0
#define MaxTSAPLength    	16     // Max Lenght for Src and Dst TSAP
#define MaxIsoFragments         64     // Max fragments
#define IsoPayload_Size    	4096   // Iso telegram Buffer size

#define noError    			0


// TPKT Header - ISO on TCP - RFC 1006 (4 bytes)
typedef struct{
	u_char Version;    // Always 3 for RFC 1006
	u_char Reserved;   // 0
	u_char HI_Lenght;  // High part of packet lenght (entire frame, payload and TPDU included)
	u_char LO_Lenght;  // Low part of packet lenght (entire frame, payload and TPDU included)
} TTPKT;               // Packet length : min 7 max 65535

typedef struct {
	u_char PduSizeCode;
	u_char PduSizeLen;
	u_char PduSizeVal;
	u_char TSAP[245]; // We don't know in advance these fields....
} TCOPT_Params ;

// COTP Header for CONNECTION REQUEST/CONFIRM - DISCONNECT REQUEST/CONFIRM
typedef struct {
	u_char  HLength;     // Header length : initialized to 6 (length without params - 1)
						 // descending classes that add values in params field must update it.
	u_char  PDUType;     // 0xE0 Connection request
						 // 0xD0 Connection confirm
						 // 0x80 Disconnect request
						 // 0xDC Disconnect confirm
	u_short DstRef;      // Destination reference : Always 0x0000
	u_short SrcRef;      // Source reference : Always 0x0000
	u_char  CO_R;
	TCOPT_Params Params;
	
} S7_TCOTP_CO ;
typedef S7_TCOTP_CO *PCOTP_CO;

// COTP Header for DATA EXCHANGE
typedef struct {
	u_char HLength;   // Header length : 3 for this header
	u_char PDUType;   // 0xF0 for this header
	u_char EoT_Num;   // EOT (bit 7) + PDU Number (bits 0..6)
         		  // EOT = 1 -> End of Trasmission Packet (This packet is complete)
			  // PDU Number : Always 0
} S7_TCOTP_DT;
typedef S7_TCOTP_DT *PCOTP_DT;

// Info part of a PDU, only common parts. We use it to check the consistence
// of a telegram regardless of it's nature (control or data).
typedef struct {
	TTPKT TPKT; 	// TPKT Header
			// Common part of any COTP
	u_char HLength; // Header length : 3 for this header
	u_char PDUType; // Pdu type
} TIsoHeaderInfo ;
typedef TIsoHeaderInfo *PIsoHeaderInfo;


typedef struct {
	TTPKT 	 TPKT; // TPKT Header
	S7_TCOTP_CO COTP; // COPT Header for CONNECTION stuffs
} TIsoControlPDU;
typedef TIsoControlPDU *PIsoControlPDU;

typedef u_char TIsoPayload[IsoPayload_Size];

typedef struct {
	TTPKT 	    TPKT; // TPKT Header
	S7_TCOTP_DT    COTP; // COPT Header for DATA EXCHANGE
	TIsoPayload Payload; // Payload
} TIsoDataPDU ;

typedef TIsoDataPDU *PIsoDataPDU;
typedef TIsoPayload *PIsoPayload;

typedef enum {
	pkConnectionRequest,
	pkDisconnectRequest,
	pkEmptyFragment,
	pkInvalidPDU,
	pkUnrecognizedType,
	pkValidData
} TPDUKind ;

#pragma pack()




#endif // s7_isotcp_h
