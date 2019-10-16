/** ==================================================================
* @file imp3dec.h
*
* @path @path $(codecs_dev)\MP3_Decoder\AlgAPI
*
* @desc This header defines all types, constants, and functions shared by all
*       implementations of the DECODE algorithm.
*
* =====================================================================
* Copyright (c) Texas Instruments Inc 2003, 2004
*
* Use of this software is controlled by the terms and conditions found
* in the license agreement under which this software has been supplied
=====================================================================*/

#ifndef IDECODE_
#define IDECODE_

/***************************************************************
*  INCLUDE FILES
****************************************************************/
#include <ti/xdais/ialg.h>
#include <ti/xdais/dm/iauddec1.h>


/* common to AAC & MP3 */
#define     MP3DEC_NO_ERROR               0
#define     MP3DEC_ERR_SYNC_NOT_FOUND     1
/* specific to Mp3 */
#define     MP3DEC_ERR_NOT_LAYER3		  2 /* Bitstream not layer3 */
#define     MP3DEC_ERR_FREE_FORMAT        3 /* Free format stream error */
#define     MP3DEC_ERR_MAIN_NEGATIVE      4 /* Main data pointer is wrong or not there in the buffer*/
#define     MP3DEC_ERR_JS_BOUND_ERROR     5 /* js_bound bad layer/modext error */
#define     MP3DEC_ERR_INSUFFICIENT_INPUT 6 /* input data for decoding is not sufficient */
#define     MP3DEC_ERR_INVALID_DATA       7 /* invalid data for L1 and L2 */
#define     MP3DEC_WARN_BAD_PCM_DATA      8 /* BAD PCM data for L3 */
#define     MP3DEC_WARN_CHANGE_CH         9 /* change in number of channels */
#define     MP3DEC_WARN_CHANGE_SF         10 /* change in samplerate */
#define     MP3DEC_WARN_CHANGE_BITRATE    11
#define     MP3DEC_ERR_LAYER              12
#define     MP3DEC_ERR_SCALEFACDEC        13
#define     MP3DEC_ERR_HUFFMANDEC         14
#define     MP3DEC_ERR_INVERSEQUANT       15
#define     MP3DEC_ERR_ALIASCANCELLATION  16
#define     MP3DEC_ERR_INVERSEMDCT        17
#define     MP3DEC_ERR_POLYSYNTHESIS      18
#define     MP3DEC_ERR_INVALIDPOINTER     19
#define     MP3DEC_ERR_CRC                20
#define     MP3DEC_ERR_NOSUPPORT          21

/*
 *  ======== IDECODE_Obj ========
 *  This structure must be the first field of all DECODE instance objects.
 */
typedef struct IMP3DEC_Obj {
    struct IMP3DEC_Fxns *fxns;
} IMP3DEC_Obj;

/*
 *  ======== IMP3_Handle ========
 *  This handle is used to reference all DECODE instance objects.
 */
typedef struct IMP3DEC_Obj  *IMP3DEC_Handle;

/*
 *  ======== IMP3_Status ========
 *  This structure defines the parameters that can be changed at runtime
 *  (read/write), and the instance status parameters (read-only).
 */
typedef struct IMP3DEC_Status{
    IAUDDEC1_Status  auddec_status; /* must be followed for audio decoder */
	 /* codec and implementation specific fields*/
	XDAS_Int32     programmedChannelMode;  /*Programmed Channel Mode in the stream */
	XDAS_Int32     bsi[7];                 /* bitstream information*/

 }IMP3DEC_Status;

/*
// ===========================================================================
// IMP3_Cmd
//
// The Cmd enumeration defines the control commands for the MP3
// control method.
*/
typedef IAUDDEC1_Cmd IMP3DEC_Cmd;


/*
// ===========================================================================
// control method commands
*/
#define IMP3DEC_GETSTATUS  XDM_GETSTATUS
#define IMP3DEC_SETPARAMS  XDM_SETPARAMS
#define IMP3DEC_RESET      XDM_RESET
#define IMP3DEC_SETDEFAULT XDM_SETDEFAULT
#define IMP3DEC_FLUSH      XDM_FLUSH
#define IMP3DEC_GETBUFINFO XDM_GETBUFINFO
#define IMP3DEC_GETVERSION XDM_GETVERSION

/*
// ===========================================================================
// IMP3_Params
//
// This structure defines the creation parameters for all MP3 objects
*/
typedef struct IMP3DEC_Params {
    IAUDDEC1_Params auddec_params; /* must be second element of creation params */

} IMP3DEC_Params;

/*
// ===========================================================================
// IMP3_PARAMS
//
// Default parameter values for MP3 instance objects
*/
extern const IMP3DEC_Params IMP3DEC_PARAMS;

/*
// ===========================================================================
// IMP3_Params
//
// This structure defines the run time parameters for all MP3 objects
*/
typedef struct IMP3DEC_DynamicParams {
    IAUDDEC1_DynamicParams auddec_dynamicparams; /* must be second field of all params structures */
} IMP3DEC_DynamicParams;


typedef struct IMP3DEC_InArgs {
    IAUDDEC1_InArgs auddec_inArgs; /* must be second field of all params structures */
} IMP3DEC_InArgs;


typedef struct IMP3DEC_OutArgs{
    IAUDDEC1_OutArgs  auddec_outArgs; /* must be followed for audio decoder */
}IMP3DEC_OutArgs;

/*
 *  ======== IMP3_Fxns ========
 *  This structure defines all of the operations on DECODE objects.
 */
typedef struct IMP3DEC_Fxns{
	IAUDDEC1_Fxns iauddec; /* must be second element of objects */
}IMP3DEC_Fxns;

#endif	/* IDECODE_ */



