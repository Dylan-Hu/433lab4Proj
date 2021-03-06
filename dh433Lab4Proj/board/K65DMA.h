/*******************************************************************************************
 * K65DMAIn.h
 * 02/10/2017 Todd Morton
 ******************************************************************************************/

/*****************************************************************************************************
* Module definition against multiple inclusion
*****************************************************************************************************/
#ifndef  SAMPLE_STREAM_PRESENT
#define  SAMPLE_STREAM_PRESENT

/*****************************************************************************************************
* Definition of sample stream macros/constants
*****************************************************************************************************/

/*****************************************************************************************************
* Definition of global VARIABLES
*****************************************************************************************************/

/*****************************************************************************************************
* Declaration of public functions
*****************************************************************************************************/
void DMA2_DMA18_IRQHandler(void);
void DMAInit(DSP_BLOCK_T *dsp_in_buf, DSP_BLOCK_T *dsp_out_buf);
INT8U DMAInPend(OS_TICK tout, OS_ERR *os_err_ptr);
void DMAStopFull(void);
void DMAStart(void);



#endif
