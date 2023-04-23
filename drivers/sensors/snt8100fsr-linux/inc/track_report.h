/*****************************************************************************
* File: track-report.h
*
* (c) 2016 Sentons Inc. - All Rights Reserved.
*
* All information contained herein is and remains the property of Sentons
* Incorporated and its suppliers if any. The intellectual and technical
* concepts contained herein are proprietary to Sentons Incorporated and its
* suppliers and may be covered by U.S. and Foreign Patents, patents in
* process, and are protected by trade secret or copyright law. Dissemination
* of this information or reproduction of this material is strictly forbidden
* unless prior written permission is obtained from Sentons Incorporated.
*
* SENTONS PROVIDES THIS SOURCE CODE STRICTLY ON AN "AS IS" BASIS,
* WITHOUT ANY WARRANTY WHATSOEVER, AND EXPRESSLY DISCLAIMS ALL
* WARRANTIES, EXPRESS, IMPLIED OR STATUTORY WITH REGARD THERETO, INCLUDING
* THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
* PURPOSE, TITLE OR NON-INFRINGEMENT OF THIRD PARTY RIGHTS. SENTONS SHALL
* NOT BE LIABLE FOR ANY DAMAGES SUFFERED BY YOU AS A RESULT OF USING,
* MODIFYING OR DISTRIBUTING THIS SOFTWARE OR ITS DERIVATIVES.
*
*
*****************************************************************************/
#ifndef TRACKREPORT_H
#define TRACKREPORT_H

/*==========================================================================*/
/* DEFINES                                                                  */
/*==========================================================================*/

/*==========================================================================*/
/* STRUCTURES                                                               */
/*==========================================================================*/

// Positional Track Report
typedef struct __attribute__((__packed__)) track_report {
    uint8_t  trk_id : 5;    // track_id
    uint8_t  bar_id : 3;    // bar_id
    uint8_t  force_lvl;     // force of touch
    uint16_t  center;       // center position, pos0
    uint16_t  bottom;       // bottom position, pos1
    uint16_t  top;          // top position, pos2
} track_report_t, *p_track_report_t;    
// Region Track Report

#define STG_ELMT_MAX                7

// Region bar_id = {6, 7}
#define STG_START_BAR_ID            6
#define IS_STG_TRACK_REPORT(barid)  ((int)((barid)>= STG_START_BAR_ID))

typedef struct __attribute__((__packed__)) stg_track_report {
    uint8_t  rsvd   : 5;
    uint8_t  bar_id : 3;                    // bar_id
    uint8_t  force_lvl[STG_ELMT_MAX];       // force of touch for i'th gauge
} stg_track_report_t, *p_stg_track_report_t;


#define TR_DIAG_MAX_MPATH                 10    // num in PH_0006_LP_Pixie

typedef struct tr_diag_vers_001_s
{
    uint8_t           start_code;       // always 0x00
    uint8_t           vers;             // 0x01 in this case
    uint16_t          mpa[TR_DIAG_MAX_MPATH];     // D1
    uint8_t           d_mp[TR_DIAG_MAX_MPATH];    // D1
    uint8_t           ntm;                        // D1
    uint8_t           gpio;                       // C2
    uint8_t           atc;                        // C2
    uint8_t           rsvd[5];

} tr_diag_vers_001_t, *p_tr_diag_vers_001_t;

typedef struct tr_diag_vers_002_s
{
    uint8_t           start_code;       // always 0x00
    uint8_t           vers;             // 0x02 in this case
    uint16_t          mpa[TR_DIAG_MAX_MPATH];     // D1
    uint8_t           d_mp[TR_DIAG_MAX_MPATH];    // D1
    uint8_t           ntm;                        // D1
    uint8_t           gpio;                       // C2
    uint8_t           atc;                        // C2
    uint8_t           frame_rate;                 // C2
    uint8_t           trig;                       // C2
    uint8_t           rsvd[3];

} tr_diag_vers_002_t, *p_tr_diag_vers_002_t;

/*****************************************************************************
* Gesture Reports may be added to struct bar1d_host_tr_s using the following
* format:
* 
* 1. gesture records will conform to the 8 byte size.
* 2. First gesture record will have first two bytes of 0x00 0x80
* 3. All subsequent gesture records will have first byte >0x80
*
* Gesture Header Record:
*
*    7  6  5  4  3  2  1  0
*  +-----------------------+
*  |          0x00         |
*  +-----------------------+
*  |          0x80         |
*  +-----------------------+
*  |    swipe0_velocity    |
*  +-----------------------+
*  |    swipe1_velocity    |
*  +-----------------------+
*  |          tap          |
*  +-----------------------+
*  |       reserved        |
*  |                       |
*  +--+--+--+--+-----------+
*  |Lo|Sh|Ca|St|EN           |
*  +--+--+--+--+-----------+
*
* L - long squeeze indicator
* S - short squeeze indicator
* Ca - squeeze cancelled indicator
* St - squeeze start indicator
* En - squeeze end indicator
*/

#define GS_RPT_TYPE_HDR               0x80
#define GS_RPT_TYPE_SLIDE             0x81
#define GS_RPT_TYPE_SQUEEZE           0x82

#define GS_MASK(size, type)  ((type) ((1 << (size)) - 1))
#define GS_PMASK(mask, pos)  ((mask) << (pos))
#define GS_GET_FIELD(val, pos, mask) (((val) >> (pos)) & (mask))

#define GS_SLIDER_REC_MAX     4

#define GS_HDR_SQUEEZE_END_POS        3
#define GS_HDR_SQUEEZE_END_SIZE       1
#define GS_HDR_SQUEEZE_END_MASK       GS_MASK(GS_HDR_SQUEEZE_END_SIZE, uint8_t)
#define GS_HDR_SQUEEZE_END_PMASK      GS_PMASK(GS_HDR_SQUEEZE_END_MASK, GS_HDR_SQUEEZE_END_POS)
#define GS_GET_SQUEEZE_END(s)         (GS_GET_FIELD(s, GS_HDR_SQUEEZE_END_POS, GS_HDR_SQUEEZE_END_MASK))

#define GS_HDR_SQUEEZE_START_POS      4
#define GS_HDR_SQUEEZE_START_SIZE     1
#define GS_HDR_SQUEEZE_START_MASK     GS_MASK(GS_HDR_SQUEEZE_START_SIZE, uint8_t)
#define GS_HDR_SQUEEZE_START_PMASK    GS_PMASK(GS_HDR_SQUEEZE_START_MASK, GS_HDR_SQUEEZE_START_POS)
#define GS_GET_SQUEEZE_START(s)       (GS_GET_FIELD(s, GS_HDR_SQUEEZE_START_POS, GS_HDR_SQUEEZE_START_MASK))

#define GS_HDR_SQUEEZE_CANCEL_POS      5
#define GS_HDR_SQUEEZE_CANCEL_SIZE     1
#define GS_HDR_SQUEEZE_CANCEL_MASK     GS_MASK(GS_HDR_SQUEEZE_CANCEL_SIZE, uint8_t)
#define GS_HDR_SQUEEZE_CANCEL_PMASK    GS_PMASK(GS_HDR_SQUEEZE_CANCEL_MASK, GS_HDR_SQUEEZE_CANCEL_POS)
#define GS_GET_SQUEEZE_CANCEL(s)       (GS_GET_FIELD(s, GS_HDR_SQUEEZE_CANCEL_POS, GS_HDR_SQUEEZE_CANCEL_MASK))

#define GS_HDR_SQUEEZE_SHORT_POS      6
#define GS_HDR_SQUEEZE_SHORT_SIZE     1
#define GS_HDR_SQUEEZE_SHORT_MASK     GS_MASK(GS_HDR_SQUEEZE_SHORT_SIZE, uint8_t)
#define GS_HDR_SQUEEZE_SHORT_PMASK    GS_PMASK(GS_HDR_SQUEEZE_SHORT_MASK, GS_HDR_SQUEEZE_SHORT_POS)
#define GS_GET_SQUEEZE_SHORT(s)       (GS_GET_FIELD(s, GS_HDR_SQUEEZE_SHORT_POS, GS_HDR_SQUEEZE_SHORT_MASK))

#define GS_HDR_SQUEEZE_LONG_POS      7
#define GS_HDR_SQUEEZE_LONG_SIZE     1
#define GS_HDR_SQUEEZE_LONG_MASK     GS_MASK(GS_HDR_SQUEEZE_LONG_SIZE, uint8_t)
#define GS_HDR_SQUEEZE_LONG_PMASK    GS_PMASK(GS_HDR_SQUEEZE_LONG_MASK, GS_HDR_SQUEEZE_LONG_POS)
#define GS_GET_SQUEEZE_LONG(s)       (GS_GET_FIELD(s, GS_HDR_SQUEEZE_LONG_POS, GS_HDR_SQUEEZE_LONG_MASK))

#define GS_HDR_TAP_START0_POS        0
#define GS_HDR_TAP_START0_SIZE       1
#define GS_HDR_TAP_START0_MASK       GS_MASK(GS_HDR_TAP_START0_SIZE, uint8_t)
#define GS_HDR_TAP_START0_PMASK      GS_PMASK(GS_HDR_TAP_START0_MASK, GS_HDR_TAP_START0_POS)
#define GS_GET_TAP_START(r, x)       (((r)>>(GS_HDR_TAP_START0_POS+(x))&GS_HDR_TAP_START0_MASK))
#define GS_SET_TAP_START(r, x, v)    ((r)&~(GS_HDR_TAP_START0_MASK<<(GS_HDR_TAP_START0_POS+(x)))) \
										  | (((v)&GS_HDR_TAP_START0_MASK)<<(GS_HDR_TAP_START0_POS+(x))))
	
#define GS_HDR_TAP_STOP0_POS         4
#define GS_HDR_TAP_STOP0_SIZE        1
#define GS_HDR_TAP_STOP0_MASK        GS_MASK(GS_HDR_TAP_STOP0_SIZE, uint8_t)
#define GS_HDR_TAP_STOP0_PMASK       GS_PMASK(GS_HDR_TAP_STOP0_MASK, GS_HDR_TAP_STOP0_POS)
#define GS_GET_TAP_STOP(r, x)        (((r)>>(GS_HDR_TAP_STOP0_POS+(x))&GS_HDR_TAP_STOP0_MASK))
#define GS_SET_TAP_STOP(r, x, v)     ((r)&~(GS_HDR_TAP_STOP0_MASK<<(GS_HDR_TAP_STOP0_POS+(x)))) \
										  | (((v)&GS_HDR_TAP_STOP0_MASK)<<(GS_HDR_TAP_STOP0_POS+(x))))
	
#define GS_TAP_MAX_NUM               4

typedef struct __attribute__((__packed__)) gs_hdr_rec_s 
{
	uint8_t		escape0;
	uint8_t		escape1;
	int8_t		swipe0_velocity;
	int8_t		swipe1_velocity;
	int8_t		swipe2_velocity;
	uint8_t   tap;
	uint8_t		reserved;
	uint8_t		squeeze;
} gs_hdr_rec_t, *p_gs_hdr_rec_t;

/*****************************************************************************
*
* Gesture Slider Records:
*
*    7  6  5  4  3  2  1  0
*  +-----------------------+
*  |          0x81         |
*  +-----------------------+
*  |fid1 | id1 |fid0 | id0 |
*  +-----------------------+
*  |     slider_force0     |
*  +-----------------------+
*  |     slider_force1     |
*  +-----------------------+
*  |      slider_pos0      |
*  |                       |
*  +-----------------------+
*  |      slider_pos1      |
*  |                       |
*  +-----------------------+
*
*/
#define GS_SLIDER_ID0_POS           0
#define GS_SLIDER_ID0_SIZE          2
#define GS_SLIDER_ID0_MASK          GS_MASK(GS_SLIDER_ID0_SIZE, uint8_t)
#define GS_SLIDER_ID0_PMASK         GS_PMASK(GS_SLIDER_ID0_MASK, GS_SLIDER_ID0_POS)
#define GS_GET_SLIDER_ID0(s)        (GS_GET_FIELD(s, GS_SLIDER_ID0_POS, GS_SLIDER_ID0_MASK))

#define GS_SLIDER_FID0_POS          2
#define GS_SLIDER_FID0_SIZE         2
#define GS_SLIDER_FID0_MASK         GS_MASK(GS_SLIDER_FID0_SIZE, uint8_t)
#define GS_SLIDER_FID0_PMASK        GS_PMASK(GS_SLIDER_FID0_MASK, GS_SLIDER_FID0_POS)
#define GS_GET_SLIDER_FID0(s)       (GS_GET_FIELD(s, GS_SLIDER_FID0_POS, GS_SLIDER_FID0_MASK))

#define GS_SLIDER_ID1_POS           4
#define GS_SLIDER_ID1_SIZE          2
#define GS_SLIDER_ID1_MASK          GS_MASK(GS_SLIDER_ID1_SIZE, uint8_t)
#define GS_SLIDER_ID1_PMASK         GS_PMASK(GS_SLIDER_ID1_MASK, GS_SLIDER_ID1_POS)
#define GS_GET_SLIDER_ID1(s)        (GS_GET_FIELD(s, GS_SLIDER_ID1_POS, GS_SLIDER_ID1_MASK))

#define GS_SLIDER_FID1_POS          6
#define GS_SLIDER_FID1_SIZE         2
#define GS_SLIDER_FID1_MASK         GS_MASK(GS_SLIDER_FID1_SIZE, uint8_t)
#define GS_SLIDER_FID1_PMASK        GS_PMASK(GS_SLIDER_FID1_MASK, GS_SLIDER_FID1_POS)
#define GS_GET_SLIDER_FID1(s)       (GS_GET_FIELD(s, GS_SLIDER_FID1_POS, GS_SLIDER_FID1_MASK))

typedef struct __attribute__((__packed__)) gs_slider_rec_s 
{
	uint8_t		escape0;
	uint8_t		slider_finger_id;
	uint8_t		slider_force0;
	uint8_t		slider_force1;
	uint16_t	slider_pos0;
	uint16_t	slider_pos1;
} gs_slider_rec_t, *p_gs_slider_rec_t;

typedef struct __attribute__((__packed__)) gs_rpt_s 
{
  uint16_t        length;
  uint16_t        fr_nr;    // not used
	gs_hdr_rec_t	  hdr;
	gs_slider_rec_t	rec[GS_SLIDER_REC_MAX];

} gs_rpt_t, *p_gs_rpt_t;

/*****************************************************************************
*
* Gesture Squeeze Records:
* (assuming SNT_GS_SQUEEZE_MAX_NUM is 4)
*
*    7  6  5  4  3  2  1  0
*  +-----------------------+
*  |          0x82         |
*  +--+--+--+--+--+--------+
*  |Lo|Sh|Ca|St|En|        |    squeeze[0]
*  +--+--+--+--+--+--------+
*  |Lo|Sh|Ca|St|En|        |    squeeze[1]
*  +--+--+--+--+--+--------+
*  |Lo|Sh|Ca|St|En|        |    squeeze[2]
*  +--+--+--+--+--+--------+
*  |Lo|Sh|Ca|St|En|        |    squeeze[3]
*  +--+--+--+--+--+--------+
*  | Reserved              |
*  +-----------------------+
*  | Reserved              |
*  +-----------------------+
*  | Reserved              |
*  +-----------------------+
*
* Lo - long squeeze indicator
* Sh - short squeeze indicator
* Ca - squeeze cancelled indicator
* St - squeeze start indicator
* En - squeeze end indicator
*/
#define SNT_GS_SQUEEZE_MAX_NUM        2

typedef struct __attribute__((__packed__)) gs_squeeze_rec_s 
{
	uint8_t		escape0;
  uint8_t   squeeze[SNT_GS_SQUEEZE_MAX_NUM];
#if SNT_GS_SQUEEZE_MAX_NUM < 7
	uint8_t		reserved[7-SNT_GS_SQUEEZE_MAX_NUM];  // filler to keep the structure size at 8 bytes
#endif
} gs_squeeze_rec_t, *p_gs_squeeze_rec_t;

#define TR_DIAG_REC_VERS_MAX              0x7f

// Diag report if first byte is 0 and second byte < 0x80
#define TR_DIAG_IS_REPORT(p)  \
          (((p_track_report_t)(p))->bar_id==0 && ((p_track_report_t)(p))->trk_id==0 && ((p_track_report_t)(p))->force_lvl <= TR_DIAG_REC_VERS_MAX)

#define GS_IS_HDR_REPORT(p) (((p_gs_hdr_rec_t)(p))->escape1 == GS_RPT_TYPE_HDR)

// Diag report if first byte is 0 and second byte >= 0x80
#define GS_IS_REPORT(p)       \
         (((p_track_report_t)(p))->bar_id==0 && ((p_track_report_t)(p))->trk_id==0 && ((p_track_report_t)(p))->force_lvl >= GS_RPT_TYPE_HDR)

/*==========================================================================*/
/* PROTOTYPES                                                               */
/*==========================================================================*/

#endif // TRACKREPORT_H
