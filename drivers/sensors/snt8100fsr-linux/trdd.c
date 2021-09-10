/*****************************************************************************
* File: trdd.c
*
* (c) 2018 Sentons Inc. - All Rights Reserved.
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

//
// BUILD LINE FOR THIS TOOL:
//
// gcc -I../inc -o trdd trdd.c
//

/*****************************************************************************
 * INCLUDE FILES
 ****************************************************************************/
#include <unistd.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <getopt.h>
#define SBOOT_SRC
#include <SentonsTFifoReportConfig.h>

/*****************************************************************************
 * MACROS AND DATA STRUCTURES
 ****************************************************************************/

typedef struct input_tr_rec_s
{
    uint32_t ts;
    uint32_t fn;
    uint32_t bar_id;
    uint32_t a1;
    uint32_t a2;
    uint32_t a3;
    uint32_t a4;
    uint32_t a5;
    uint32_t a6;
    uint32_t a7;

} input_tr_rec_t, *p_input_tr_rec_t;


typedef struct event_log_s 
{
    uint8_t     sub_sys;
    uint8_t     evt_id;
    uint16_t    parm1;
    uint16_t    parm2;
    uint16_t    parm3;

} event_log_t, *p_event_log_t;

#define STATE_TR            0
#define STATE_TR_DIAG       1

#define MIN_STRAIN_BAR_ID   6
#define MAX_STRAIN_BAR_ID   7
#define IS_STRAIN_BAR(id)   ((id) <= MAX_STRAIN_BAR_ID && (id) >= MIN_STRAIN_BAR_ID)

#define FTYPE_TR_DIAG_TXT   0
#define FTYPE_DEEP_TRACE    1
#define FTYPE_TR_DIAG_BIN   2
#define FTYPE_EVENT_LOG_BIN 3

#define PRINT_LEVEL_NORMAL  0
#define PRINT_LEVEL_VERBOSE 1
#define PRINT_LEVEL_DEBUG   2

/*****************************************************************************
 * GLOBAL VARIABLES
 ****************************************************************************/

char *in_fname = NULL;
char *out_fname = NULL;
FILE *fin = NULL;
FILE *fout = NULL;
FILE *ferr = NULL;
int state = STATE_TR;
int tr_cur_rec;
int tr_max_rec;
int print_level = PRINT_LEVEL_NORMAL;

uint8_t *p_tr_diag_buf;
uint8_t *p_tr_diag_base;
tr_diag_vers_001_t tr_diag_rec_001;
tr_diag_vers_002_t tr_diag_rec_002;
int ftype = FTYPE_TR_DIAG_TXT;

uint32_t cur_ts = 0;


/*****************************************************************************
 * FUNCTION DECLARATIONS
 ****************************************************************************/


void process_report_rec(p_input_tr_rec_t p_in);
void start_tr_diag(p_input_tr_rec_t p_in);
void process_tr_diag(p_input_tr_rec_t p_in);
void print_strain_tr(p_input_tr_rec_t p_in);
void print_normal_tr(p_input_tr_rec_t p_in);
void print_tr_diag(uint8_t *p_buf);
void print_tr_diag_001(void* p_buf);
void print_tr_diag_002(void* p_buf);
void print_tr_diag_003(void* p_buf);
void print_tr_diag_004(void* p_buf);
void print_tr_diag_005(void* p_buf);
void process_tr_diag_txt_file(void);
void process_tr_diag_bin_file(void);
void process_deep_trace_file(void);
void process_event_log_bin_file(void);


/*****************************************************************************
 * SOURCE CODE
 ****************************************************************************/

/** 
 */
void print_usage() {
    printf("Usage: trdd [-erdh] [-o outfile] [infile]\n");
    printf("       -d: print deep_trace.bin (binary) file\n");
    printf("       -e: print event_log.bin (binary) file\n");
    printf("       -h: print this help text\n");
    printf("       -r: print track_report.bin (binary) file\n");
    printf("       -v: verbose output\n");
    printf("       default: print track_report.log (text) file\n");
}


FILE *open_in_file(char *fname, int ftype)
{
    FILE *fret = NULL;
    if (fname == NULL) {
        fret = stdin;
    } else {
        switch (ftype) {
            case FTYPE_TR_DIAG_TXT: fret = fopen(fname, "r"); break;
            case FTYPE_TR_DIAG_BIN:
            case FTYPE_EVENT_LOG_BIN:
            case FTYPE_DEEP_TRACE : fret = fopen(fname, "rb"); break;
            default: fprintf(ferr,"Unknown ftype (%d)\n", ftype); break;
        }
    }
    return fret;
}

FILE *open_out_file(char *fname, int ftype)
{
    FILE *fret = NULL;

    if (fname == NULL) {
        fret = stdout;
    } else {
        fret = fopen(fname, "w");
    }
    return fret;
}

/*****************************************************************************
 * main()
 *
 * usage: trdd [track_report_file]
 *
 * If track_report_file not supplied, assumes stdin.
 *
 * input is track report logging file from snt8100fsr reference linux driver:
 *
 * echo 1 >/sys/snt8100fsr/log_track_reports
 * echo 0 >/sys/snt8100fsr/log_track_reports
 *
 * This program will parse a track_report log file and reformat the track
 * report diagnostic section.
 *
 * To turn on track report diagnostic logging:
 *
 * echo 0x40 1 >/sys/snt8100fsr/set_reg
 *
 * To Make:
 *
 * cc -o trdd trdd.c
 *
 ****************************************************************************/
int main(int argc, char *argv[])
{
    int option = 0;

    ferr = stderr;
  
     //Specifying the expected options
    while ((option = getopt(argc, argv,"edho:rv")) != -1) {
        switch (option) {
            case 'e': ftype=FTYPE_EVENT_LOG_BIN; break;
            case 'd': ftype=FTYPE_DEEP_TRACE; break;
            case 'o': out_fname = optarg; break;
            case 'r': ftype=FTYPE_TR_DIAG_BIN; break;
            case 'v': print_level = PRINT_LEVEL_VERBOSE; break;
            default : print_usage(); exit(1); break;
        }
    }

    // Open input and output files
    if (optind < argc) {
        in_fname = argv[optind];
    } 
    fin = open_in_file(in_fname, ftype);
    if (fin==NULL) {
        fprintf(ferr, "ERROR! Couldn't open %s\n", in_fname);
        exit(1);
    }
    fout = open_out_file(out_fname, ftype);
    if (fout==NULL) {
        fprintf(ferr, "ERROR! Couldn't open %s\n", out_fname);
        exit(1);
    }
    switch (ftype) {
        case FTYPE_EVENT_LOG_BIN:   process_event_log_bin_file();   break;
        case FTYPE_TR_DIAG_TXT:     process_tr_diag_txt_file();     break;
        case FTYPE_TR_DIAG_BIN:     process_tr_diag_bin_file();     break;
        case FTYPE_DEEP_TRACE :     process_deep_trace_file();      break;
        default: fprintf(ferr,"Unknown ftype (%d)\n", ftype); break;
    }
}

void process_tr_diag_txt_file(void)
{
    int num_token;
    char line[256];
    char *p;
    input_tr_rec_t tr_in;
    int lineno=1;
  
    p = fgets(line, 256, fin);
    while (p) {

        /* snt8100fsr driver adds a \0 after first line. take it out */
        if (line[0]==0) p = line+1; else p = line;

        num_token = sscanf(p,"%u, %u, %u, %u, %u, %u, %u, %u, %u, %u",
                            &tr_in.ts,
                            &tr_in.fn,
                            &tr_in.bar_id,
                            &tr_in.a1,
                            &tr_in.a2,
                            &tr_in.a3,
                            &tr_in.a4,
                            &tr_in.a5,
                            &tr_in.a6,
                            &tr_in.a7);

        if (num_token == 8 || num_token == 10) // piezo or strain report
            process_report_rec(&tr_in);
        else
            fprintf(fout,"%s",line);

        p = fgets(line, 256, fin);
        lineno++;
    }
}


void process_report_rec(p_input_tr_rec_t p_in)
{
    /* add extra \n on new record  */
    if (p_in->ts != cur_ts) {
        fprintf(fout, "\n");
    }

    switch (state) {
        case STATE_TR: {
            if (p_in->bar_id == 0 && p_in->a1 == 0) {
                start_tr_diag(p_in);
            } else if (IS_STRAIN_BAR(p_in->bar_id)) {
                print_strain_tr(p_in);
            } else {
                print_normal_tr(p_in);
            }

        }
        break;

        case STATE_TR_DIAG: {
            process_tr_diag(p_in);
        }
        break;

        default: fprintf(ferr, "ERROR: unknown state %d\n", state); exit(1);
    }

    /* update new record indicator */
    if (p_in->ts != cur_ts)
        cur_ts = p_in->ts;
}

/*****************************************************************************
 * start_tr_diag()
 *
 * Initialize state transition to processing tr_diag section of track report.
 * Key opeation is taking track reports and refactoring their data into the
 * tr_diag structure.
 *
 * conversion of normal tr to bytes in tr_diag:
 *
 * bar_id<<3 | a1&0x3 = byte0
 * a2 = byte1
 * a3 = byte2 | byte3
 * a4 = byte4 | byte5
 * a5 = byte6 | byte7
 *
 * conversion of strain tr to bytes in tr_diag:
 *
 * bar_id << 3 = byte0 (WARNING! MISSING 3lsb0:2)
 * a1 = byte1
 * a2 = byte2
 * a3 = byte3
 * a4 = byte4
 * a5 = byte5
 * a6 = byte6
 * a7 = byte7
 *
 ****************************************************************************/

void start_tr_diag(p_input_tr_rec_t p_in)
{
    if (p_in->ts != cur_ts) {
        fprintf(fout, "%u, %u\n", p_in->ts, p_in->fn);
    }
    state = STATE_TR_DIAG;
    tr_cur_rec = 1;

    // currently 2 versions of tr_diag structure defined.
    if (p_in->a2 == 1) {
        p_tr_diag_base = (uint8_t*) &tr_diag_rec_001;
        p_tr_diag_buf = p_tr_diag_base;
        tr_max_rec = TR_DIAG_REC_VERS_001;
    } else if (p_in->a2 == 2) {
        p_tr_diag_base = (uint8_t*) &tr_diag_rec_002;
        p_tr_diag_buf = p_tr_diag_base;
        tr_max_rec = TR_DIAG_REC_VERS_002;
    } else {
        fprintf(ferr, "ERROR Unkown tr_diag version %d\n", (int)p_in->a2);
        exit(1);
    }
    *p_tr_diag_buf++ = 0; // barid|a1 is 0 by def
    *p_tr_diag_buf++ = p_in->a2;
    *p_tr_diag_buf++ = p_in->a3&0xff;
    *p_tr_diag_buf++ = (p_in->a3>>8)&0xff;
    *p_tr_diag_buf++ = p_in->a4&0xff;
    *p_tr_diag_buf++ = (p_in->a4>>8)&0xff;
    *p_tr_diag_buf++ = p_in->a5&0xff;
    *p_tr_diag_buf++ = (p_in->a5>>8)&0xff;

    if (tr_cur_rec == tr_max_rec) {
        print_tr_diag(p_tr_diag_base);
        state = STATE_TR;
    }
}


/*****************************************************************************
 * process_tr_diag()
 *
 * continue converting track report records into the tr_diag structure.
 *
 ****************************************************************************/
void process_tr_diag(p_input_tr_rec_t p_in)
{
    if (IS_STRAIN_BAR(p_in->bar_id)) {
        fprintf(ferr, "WARNING! Possible loss of data in tr_diag fr_nr=%u\n",p_in->fn);
        *p_tr_diag_buf++ = p_in->bar_id<<3;
        *p_tr_diag_buf++ = p_in->a1;
        *p_tr_diag_buf++ = p_in->a2;
        *p_tr_diag_buf++ = p_in->a3;
        *p_tr_diag_buf++ = p_in->a4;
        *p_tr_diag_buf++ = p_in->a5;
        *p_tr_diag_buf++ = p_in->a6;
        *p_tr_diag_buf++ = p_in->a7;

    } else {
        *p_tr_diag_buf++ = p_in->bar_id<<3 | p_in->a1&0x3;
        *p_tr_diag_buf++ = p_in->a2;
        *p_tr_diag_buf++ = p_in->a3&0xff;
        *p_tr_diag_buf++ = (p_in->a3>>8)&0xff;
        *p_tr_diag_buf++ = p_in->a4&0xff;
        *p_tr_diag_buf++ = (p_in->a4>>8)&0xff;
        *p_tr_diag_buf++ = p_in->a5&0xff;
        *p_tr_diag_buf++ = (p_in->a5>>8)&0xff;
    }
    tr_cur_rec++;
    if (tr_cur_rec == tr_max_rec) {
        print_tr_diag(p_tr_diag_base);
        state = STATE_TR;
    }
}


/*****************************************************************************
 * print_tr_diag()
 ****************************************************************************/
void print_tr_diag(uint8_t* p_buf)
{
    if (p_buf) {
        switch (p_buf[1]) {
            case 1: print_tr_diag_001((void*) p_buf); break;
            case 2: print_tr_diag_002((void*) p_buf); break;
            case 3: print_tr_diag_003((void*) p_buf); break;
            case 4: print_tr_diag_004((void*) p_buf); break;
            case 5: print_tr_diag_005((void*) p_buf); break;
            default: fprintf(stderr, "ERROR! print! Unknown version %u\n", p_buf[1]);
                     exit(1);
        }
    }
}


/*****************************************************************************
 * print_tr_diag_001()
 *
 * formats and prints version 1 tr_diag.
 ****************************************************************************/
void print_tr_diag_001(void* p)
{
  int i;
  p_tr_diag_vers_001_t p_diag = (p_tr_diag_vers_001_t) p;

  fprintf(fout, "    vers=%d, gpio=0x%02x, atc=%d, ntm.stress=%d, ntm.nt=%d\n",
            p_diag->vers,
            p_diag->gpio,
            p_diag->atc,
            p_diag->ntm>>4,
            p_diag->ntm&0xf);
  fprintf(fout, "    mpa : ");
  for (i=0; i < TR_DIAG_MAX_MPATH; i++) {
    fprintf(fout, "0x%02x ",p_diag->mpa[i]);
  }
  fprintf(fout, "\n    d_mp: ");
  for (i=0; i < TR_DIAG_MAX_MPATH; i++) {
    fprintf(fout, "0x%04x ",p_diag->d_mp[i]);
  }
  fprintf(fout,"\n");
}


/*****************************************************************************
 * print_tr_diag_002()
 *
 * formats and prints version 2 tr_diag.
 ****************************************************************************/
void print_tr_diag_002(void* p)
{
  int i;
  p_tr_diag_vers_002_t p_diag = (p_tr_diag_vers_002_t) p;

  fprintf(fout, "    vers=%d, fr=%u, tap=0x%02x, gpio=0x%02x, atc=%d, ntm.stress=%d, ntm.nt=%d\n",
            p_diag->vers,
            p_diag->frame_rate,
            p_diag->trig,
            p_diag->gpio,
            p_diag->atc,
            p_diag->ntm>>4,
            p_diag->ntm&0xf);
  fprintf(fout, "    mpa : ");
  for (i=0; i < TR_DIAG_MAX_MPATH; i++) {
    fprintf(fout, "0x%04x ",p_diag->mpa[i]&0xffff);
  }
  fprintf(fout, "\n    d_mp: ");
  for (i=0; i < TR_DIAG_MAX_MPATH; i++) {
    fprintf(fout, "0x%02x ",p_diag->d_mp[i]&0xff);
  }
  fprintf(fout,"\n");
}

void print_tr_diag_tap_rec(void* p)
{
  p_tr_diag_tap_instance_t p_tap = (p_tr_diag_tap_instance_t) p;

  fprintf(fout, "        state=%d, trk_id=0x%02x, last=%d, peak=%d, floor=%d, window=0x%04x\n",
           p_tap->state,
           p_tap->det_track_id,
           p_tap->last_force,
           p_tap->peak_force,
           p_tap->floor_force,
           p_tap->window);
}

void print_tr_diag_slide_rec(void *p)
{
  p_tr_diag_slide_instance_t p_slide = (p_tr_diag_slide_instance_t) p;

  fprintf(fout, "        state=%d, bar_trk=0x%02x, p0=%d, f0=%d, maxdf=%d, nrep=%d\n",
          p_slide->state,
          p_slide->bar_track,
          p_slide->pos0,
          p_slide->force0,
          p_slide->max_dforce,
          p_slide->nreport);
}

void print_tr_diag_003(void* p)
{
  int i;
  p_tr_diag_tap_t p_dg = (p_tr_diag_tap_t) p;

  fprintf(fout, "    vers=%d, fr=%u, tap=0x%02x, trig=0x%02x\n",
                    p_dg->vers,
                    p_dg->frame_rate,
                    p_dg->trig,
                    p_dg->gpio);
  for (i=0; i < TR_DIAG_MAX_TAP; i++) {
    print_tr_diag_tap_rec(&p_dg->tap[i]);
  }
}


void print_tr_diag_004(void* p)
{
  int i;
  p_tr_diag_slide_t p_dg = (p_tr_diag_slide_t) p;

  fprintf(fout, "    vers=%d, fr=%u, tap=0x%02x, trig=0x%02x\n",
                    p_dg->vers,
                    p_dg->frame_rate,
                    p_dg->trig,
                    p_dg->gpio);
  for (i=0; i < TR_DIAG_MAX_SLIDE; i++) {
    print_tr_diag_slide_rec(&p_dg->slide[i]);
  }
}

void print_tr_diag_005(void* p)
{
  int i;
  p_tr_diag_tap_slide_t p_dg = (p_tr_diag_tap_slide_t) p;
  fprintf(fout, "    vers=%d, fr=%u, tap=0x%02x, trig=0x%02x\n",
                    p_dg->vers,
                    p_dg->frame_rate,
                    p_dg->trig,
                    p_dg->gpio);
  for (i=0; i < TR_DIAG_MAX_TAP; i++) {
    print_tr_diag_tap_rec(&p_dg->tap[i]);
  }
  fprintf(fout, "\n");
  for (i=0; i < TR_DIAG_MAX_SLIDE; i++) {
    print_tr_diag_slide_rec(&p_dg->slide[i]);
  }
}


/*****************************************************************************
 * print_strain_tr()
 *
 ****************************************************************************/
void print_strain_tr(p_input_tr_rec_t p_in)
{
    fprintf(fout, "%u, %u, %u, %u, %u, %u, %u, %u, %u, %u\n",
            p_in->ts,
            p_in->fn,
            p_in->bar_id,
            p_in->a1,
            p_in->a2,
            p_in->a3,
            p_in->a4,
            p_in->a5,
            p_in->a6,
            p_in->a7);
}



/*****************************************************************************
 * print_normal_tr()
 *
 ****************************************************************************/
void print_normal_tr(p_input_tr_rec_t p_in)
{
    fprintf(fout, "%u, %u, %u, %u, %u, %u, %u, %u\n",
            p_in->ts,
            p_in->fn,
            p_in->bar_id,
            p_in->a1,
            p_in->a2,
            p_in->a3,
            p_in->a4,
            p_in->a5);
}

void print_track_rec_bin(int fr_nr, p_bar1d_host_tr_rec_t p_tr_rec, uint32_t ts)
{   
    if (p_tr_rec==NULL) return;

    int bar_id = p_tr_rec->bar_trk_id >> 5 & 0x7;
    int track_id = p_tr_rec->bar_trk_id & 0x1f;
    if (ts)
        fprintf(fout, "%d(%u): ", fr_nr, ts);
    else
        fprintf(fout, "%d: ", fr_nr);
    if (IS_STRAIN_BAR(bar_id)) {
        p_bar1d_host_tr_sprite_rec_t p_sr_rec = (p_bar1d_host_tr_sprite_rec_t) p_tr_rec;
        fprintf(fout,"bar=%u, %u %u %u %u %u %u %u\n",
                bar_id,
                p_sr_rec->force_lvl[0],
                p_sr_rec->force_lvl[1],
                p_sr_rec->force_lvl[2],
                p_sr_rec->force_lvl[3],
                p_sr_rec->force_lvl[4],
                p_sr_rec->force_lvl[5],
                p_sr_rec->force_lvl[6]
                );
    } else {
        fprintf(fout,"bar=%u trk=%u frc=%u p0=%5.3f(%u) p1=%5.3f(%u) p2=%5.3f(%u)\n",
                 bar_id, 
                 track_id, 
                 p_tr_rec->force_lvl, 
                 (float)p_tr_rec->pos0/16.0,p_tr_rec->pos0,
                 (float)p_tr_rec->pos1/16.0,p_tr_rec->pos1,
                 (float)p_tr_rec->pos2/16.0,p_tr_rec->pos2);
    }
}

void print_tr_bin(p_bar1d_host_tr_t p_tr, uint32_t ts)
{
    if (p_tr == NULL) {fprintf(ferr,"NULL tr record\n"); exit(1);}

    int num_rec = (p_tr->length - sizeof(uint16_t))/sizeof(bar1d_host_tr_rec_t); // sub out fr_nr field

    int rec_idx = 0;
    int gs_rec_found = 0;
    int i;
    
    if (print_level >= PRINT_LEVEL_VERBOSE) {
      fprintf(fout, "TFIFO Report Number of Records = %d\n\n", num_rec);
      fprintf(fout, "TFIFO Report Raw Record dump:\n\n");
      for (i=0; i < num_rec; i++) {
        fprintf(fout,"    %2i: 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n", i,
                  ((uint8_t*)&p_tr->tr[i])[0],
                  ((uint8_t*)&p_tr->tr[i])[1],
                  ((uint8_t*)&p_tr->tr[i])[2],
                  ((uint8_t*)&p_tr->tr[i])[3],
                  ((uint8_t*)&p_tr->tr[i])[4],
                  ((uint8_t*)&p_tr->tr[i])[5],
                  ((uint8_t*)&p_tr->tr[i])[6],
                  ((uint8_t*)&p_tr->tr[i])[7]);
      }
      fprintf(fout,"\nTFIFO Report Formated Records:\n");
    }
    fprintf(fout,"\n");

    while (rec_idx < num_rec) {

        // tr_diag section
        if (p_tr->tr[rec_idx].bar_trk_id == 0 && p_tr->tr[rec_idx].force_lvl < 0x80) {
            if (rec_idx == 0) {
                if (ts == 0)
                    fprintf(fout, "%d:\n", p_tr->fr_nr);
                else
                    fprintf(fout, "%d(%u):\n", p_tr->fr_nr, ts);
            }
            print_tr_diag((uint8_t*)&p_tr->tr[rec_idx]);

            break;  // end of records
        } 

        if (gs_rec_found || (p_tr->tr[rec_idx].bar_trk_id == 0 && p_tr->tr[rec_idx].force_lvl >= 0x80)) {
            // gesture section
            if (gs_rec_found == 0) {
                p_gs_hdr_rec_t p = (p_gs_hdr_rec_t) &p_tr->tr[rec_idx];
                if (rec_idx == 0) {
                    if (ts == 0)
                        fprintf(fout, "%d:\n", p_tr->fr_nr);
                    else
                        fprintf(fout, "%d(%u):\n", p_tr->fr_nr, ts);
                }
                fprintf(fout, "    0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n",
                                    ((uint8_t*)p)[0],
                                    ((uint8_t*)p)[1],
                                    ((uint8_t*)p)[2],
                                    ((uint8_t*)p)[3],
                                    ((uint8_t*)p)[4],
                                    ((uint8_t*)p)[5],
                                    ((uint8_t*)p)[6],
                                    ((uint8_t*)p)[7]);
                if (p->squeeze != 0) {
                    // Handle the legacy case of the squeeze still being in the gesture header
                    fprintf(fout, "    Gesture: sw0=%d sw1=%d sw2=%d, sq_st=%u, sq_cs=%u, sq_sh=%u, sq_lg=%u, sq_end=%u\n",
                                p->swipe0_velocity, p->swipe1_velocity, p->swipe2_velocity,
                                GS_GET_SQUEEZE_START(p->squeeze), 
                                GS_GET_SQUEEZE_CANCEL(p->squeeze), 
                                GS_GET_SQUEEZE_SHORT(p->squeeze), 
                                GS_GET_SQUEEZE_LONG(p->squeeze),
                                GS_GET_SQUEEZE_END(p->squeeze));

                } else {
                    // Normal case of no squeeze in the the header
                    fprintf(fout, "    Gesture: sw0=%u sw1=%u sw2=%u\n",
                                p->swipe0_velocity, p->swipe1_velocity, p->swipe1_velocity);
                }
                fprintf(fout, "             tap_start[");
                for (i=0;i<GS_TAP_MAX_NUM;i++)
                  if (GS_GET_TAP_START(p->tap,i)) fprintf(fout,"%d",i);
                fprintf(fout, "] tap_stop[");
                for (i=0;i<GS_TAP_MAX_NUM;i++)
                  if (GS_GET_TAP_STOP(p->tap,i)) fprintf(fout,"%d",i);
                fprintf(fout,"] (0x%x)\n", p->tap);
                gs_rec_found = 1;
            } else {
                // Gesture Report
                // First guess is slider.  If that's wrong then keep trying
                p_gs_slider_rec_t p = (p_gs_slider_rec_t) &p_tr->tr[rec_idx];
                if (p->escape0 == GS_ID_SLIDE) {
                    // We are, indeed, a slider
                uint8_t id0 = GS_GET_SLIDER_ID0(p->slider_finger_id);
                uint8_t id1 = GS_GET_SLIDER_ID1(p->slider_finger_id);
                uint8_t fid0 = GS_GET_SLIDER_FID0(p->slider_finger_id);
                uint8_t fid1 = GS_GET_SLIDER_FID1(p->slider_finger_id);
                fprintf(fout, "    0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n",
                                    ((uint8_t*)p)[0],
                                    ((uint8_t*)p)[1],
                                    ((uint8_t*)p)[2],
                                    ((uint8_t*)p)[3],
                                    ((uint8_t*)p)[4],
                                    ((uint8_t*)p)[5],
                                    ((uint8_t*)p)[6],
                                    ((uint8_t*)p)[7]);
                if (fid0) {
                    fprintf(fout, "    Slider[%u,%u]: frc=%u, pos=%u\n", id0, fid0,
                                    p->slider_force0, p->slider_pos0);
                }
                if (fid1) {
                    fprintf(fout, "    Slider[%u,%u]: frc=%u, pos=%u\n", id1, fid1,
                                    p->slider_force1, p->slider_pos1);
                    }
                } else if (p->escape0 == GS_ID_SQUEEZE) {
                    // We are a Squeeze report
                    p_gs_squeeze_rec_t p_sq = (p_gs_squeeze_rec_t) &p_tr->tr[rec_idx];
                    fprintf(fout, "    Squeeze: (St|Ca|Sh|Lo|End)");
                    for (int sqIdx = 0; sqIdx < SNT_GS_SQUEEZE_MAX_NUM; sqIdx++) {
                        if (p_sq->squeeze[sqIdx] == 0) {
                            fprintf(fout, " #%d=(-----)", sqIdx);
                        } else {
                            fprintf(fout, " #%d=(%d%d%d%d%d)",
                                sqIdx,
                                GS_GET_SQUEEZE_START(p_sq->squeeze[sqIdx]),
                                GS_GET_SQUEEZE_CANCEL(p_sq->squeeze[sqIdx]),
                                GS_GET_SQUEEZE_SHORT(p_sq->squeeze[sqIdx]),
                                GS_GET_SQUEEZE_LONG(p_sq->squeeze[sqIdx]), 
                                GS_GET_SQUEEZE_END(p_sq->squeeze[sqIdx]) );
                        }
                    }
                    fprintf(fout,"\n");
                } else {      
                    // This is not good.  Better to report it than to ignore it
                    fprintf(fout,"    ERROR: Bad Gesture Report id (%d)\n", p->escape0);
                }
            }

        } else {
            // track reports section
            print_track_rec_bin(p_tr->fr_nr, &p_tr->tr[rec_idx], ts);
        }
        rec_idx++;
    }
}



void process_tr_diag_bin_file(void)
{
    uint16_t rec_len;
    uint32_t ts;
    bar1d_host_tr_t tr;
    int ret;
    int tr_i = 0;

    while (42) {
        // read log record length
        ret = fread((void*)&rec_len, sizeof(uint16_t), 1, fin);
        if (ret != 1) {
            if (!feof(fin)) 
                fprintf(ferr, "Failed to read bin track report log rec size %d - EOF\n", ret);
            return;
        }

        // read timestamp
        ret = fread((void*)&ts, sizeof(uint32_t), 1, fin);
        if (ret != 1) {
            fprintf(ferr, "Failed to read bin track report log rec timestamp %d\n", ret);
            return;
        }

        // read tr record length
        ret = fread((void*)&tr.length, sizeof(uint16_t), 1, fin);
        if (ret != 1) {
            fprintf(ferr, "Failed to read bin track report rec size %d - EOF\n", ret);
            return;
        }

        // read fr_nr+tr reports
        ret = fread((void*)&tr.fr_nr, sizeof(uint8_t), tr.length, fin);
        if (ret != tr.length) {
            fprintf(ferr, "Failed to read bin track report rec size %d - EOF\n", ret);
            return;
        }

        if (print_level >= PRINT_LEVEL_VERBOSE) {
          fprintf(fout, "\n**********************************************************\n");
          fprintf(fout, "TFIFO Report record index      = %d\n", tr_i++);
          fprintf(fout, "TFIFO Report length            = %d\n", rec_len);
          fprintf(fout, "TFIFO Report timestamp         = %d\n", ts);
          fprintf(fout, "TFIFO Report Content Length    = %d\n", tr.length);
          fprintf(fout, "TFIFO Report Frame Number      = %d\n", tr.fr_nr);
        }

        // print reports
        print_tr_bin(&tr, ts);
    }
}

void process_event_log_bin_file(void)
{
    uint32_t ev[2];
    int ret;
    uint32_t status;

    ret = fread((void*)&status, sizeof(uint8_t), sizeof(status), fin);
    if (ret != sizeof(status)) {
        if (!feof(fin)) 
            fprintf(ferr, "Failed to read event log status size %d\n", ret);
        return;
    }
    fprintf(fout,"EventLog Status = %d\n", status);

    while (42) {
        // read log record 
        ret = fread((void*)&ev, sizeof(uint8_t), sizeof(ev), fin);
        if (ret != sizeof(ev)) {
            if (!feof(fin)) 
                fprintf(ferr, "Failed to read event log rec size %d\n", ret);
            return;
        }
        fprintf(fout,"0x%02x  0x%02x  0x%04x  0x%04x  0x%04x\n",
                ev[0] >> 24 & 0xff,
                ev[0] >> 16 & 0xff,
                ev[0] & 0xffff,
                ev[1] >> 16 & 0xffff,
                ev[1] & 0xffff);
    }

}




void process_deep_trace_file(void)
{
    int i;
    uint16_t trace_len;
    uint16_t r_idx;
    uint16_t w_idx;
    int ret = fread((void*)&trace_len, sizeof(uint16_t), 1, fin);
    if (ret != 1) {
        fprintf(ferr, "Failed to read deep trace size %d\n", ret);
        return;
    }
    ret = fread((void*)&r_idx, sizeof(uint16_t), 1, fin);
    if (ret != 1) {
        fprintf(ferr, "Failed to read deep trace r_idx size\n");
        return;
    }
    ret = fread((void*)&w_idx, sizeof(uint16_t), 1, fin);
    if (ret != 1) {
        fprintf(ferr, "Failed to read deep trace w_idx size\n");
        return;
    }
    trace_len -= 2*sizeof(uint16_t);

    uint8_t *p_buf = malloc(trace_len);
    if (p_buf==NULL) {
        fprintf(ferr,"Could not malloc deep trace buffer %d\n", trace_len);
        return;
    }

    ret = fread((void*)p_buf, sizeof(uint8_t), trace_len, fin);
    if (ret != trace_len) {
        fprintf(ferr, "Could not read full deep trace file (%d!=%d)\n", ret, trace_len);
        return;
    }

    while (r_idx != w_idx) {
        uint8_t rec_len = p_buf[r_idx] - sizeof(uint8_t);     // sub out len field
        r_idx = (r_idx+1) % trace_len;
        bar1d_host_tr_t tr;
        uint8_t *p_tr_buf = (uint8_t*)&tr;
        for (i=0; i < rec_len; i++) {
            p_tr_buf[i] = p_buf[r_idx];
            r_idx = (r_idx+1) % trace_len;
        }
        print_tr_bin(&tr, 0 /*no ts*/);
    }
    
}


/* EOF */


