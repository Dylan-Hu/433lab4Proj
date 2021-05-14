
/*******************************************************************************************
* AppDSP.c
* This is an example of one data processing task that does some real-time digital processing.
*
* 02/10/2017 Todd Morton
* 04/03/2019 Todd Morton
*
* Modified for lab 4 use. Part 2 can be verified using headphones plugged into stereo out.
* Volume will be different, and there will be a buzzing noise,
* but the tone should sound the same. Part 3 data can be verified using expression window.
* 5/14/2021 Dylan Huntsman
*******************************************************************************************/
/******************************************************************************************
* Include files
*******************************************************************************************/
#include "MCUType.h"
#include "app_cfg.h"
#include "os.h"
#include "I2S.h"
#include "TLV320AIC3007.h"
#include "K65TWR_GPIO.h"
#include "AppDSP.h"
#include "K65DMA.h"
/******************************************************************************************
 * Module Defines
 *****************************************************************************************/
#define SINESIZE            DSP_SAMPLES_PER_BLOCK-NUM_TAPS+1
#define SAMPLE_RATE_HZ      48000
#define FREQ_NORM(x)        x*SAMPLE_RATE_HZ/DSP_SAMPLES_PER_BLOCK
/******************************************************************************************
 * Private Variables
 *****************************************************************************************/
static float32_t firCoef32[NUM_TAPS];
static float32_t cosMod[DSP_SAMPLES_PER_BLOCK];
static q31_t firCoeffQ31[NUM_TAPS];
static DSP_BLOCK_T dspInBuffer[DSP_NUM_IN_CHANNELS][DSP_NUM_BLOCKS];
static DSP_BLOCK_T dspOutBuffer[DSP_NUM_OUT_CHANNELS][DSP_NUM_BLOCKS];

static INT8U dspStopReqFlag = 0;
static OS_SEM dspFullStop;
static q31_t firStateQ31[NUM_TAPS+DSP_SAMPLES_PER_BLOCK-1];
static CPU_STK dspTaskStk[APP_CFG_DSP_TASK_STK_SIZE];
static OS_TCB dspTaskTCB;
static DSP_PARAMS_T dspParams;
static const INT8U dspCodeToSize[4] = {16,20,24,32};
static const INT16U dspCodeToRate[11] = {48000,32000,24000,19200,16000,13700,
                                         12000,10700,9600,8700,8000};

static arm_rfft_instance_q31 arm_rfft_sR_q31_len128Mod;
static arm_fir_instance_q31 firSine;
//Single struct to hold all message info
typedef struct{
    q31_t noisyQ[DSP_SAMPLES_PER_BLOCK];    //part 2 unfiltered message
    q31_t modQ[DSP_SAMPLES_PER_BLOCK];      //part 3 unfiltered message
    q31_t filteredQ[DSP_SAMPLES_PER_BLOCK]; //part 2 filtered message
    q31_t FFT[2*DSP_SAMPLES_PER_BLOCK];     //part 3 FFT destination
    q31_t FFTMag[DSP_SAMPLES_PER_BLOCK];    //part 3 FFTmag destination
    q31_t magQ;                             //part 3 FFTmag MAX destination
    uint32_t index;                         //part 3 FFTmag index destination
    float32_t toneF[DSP_SAMPLES_PER_BLOCK]; //original message destination
    float32_t noisyF[DSP_SAMPLES_PER_BLOCK];//part 2 unfiltered message
    float32_t modF[DSP_SAMPLES_PER_BLOCK];  //part 3 unfiltered message
    DSP_BLOCK_T inputQ;                     //variable to pass original message to CODEC
    DSP_BLOCK_T filteredAudio;              //variable to pass filtered message to CODEC
} DSP_AUDIO;

static DSP_AUDIO signalTone;
typedef enum {STARTUP, COMPLETE} FILTERING_STATES;
static FILTERING_STATES FilterState;
/*******************************************************************************************
* Private Function Prototypes
*******************************************************************************************/
static void SineInit(void);
static void FilterInit(void);
static void  dspTask(void *p_arg);
/*******************************************************************************************
* DSPInit()- Initializes all dsp requirements - CODEC,I2S,DMA, and sets initial sample rate
*            and sample size.
*******************************************************************************************/
void DSPInit(void){
    OS_ERR os_err;

    OSTaskCreate(&dspTaskTCB,
                "DSP Task ",
                dspTask,
                (void *) 0,
                APP_CFG_DSP_TASK_PRIO,
                &dspTaskStk[0],
                (APP_CFG_DSP_TASK_STK_SIZE / 10u),
                APP_CFG_DSP_TASK_STK_SIZE,
                0,
                0,
                (void *) 0,
                (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                &os_err);

    OSSemCreate(&dspFullStop, "DMA Stopped", 0, &os_err);
    CODECInit();
    I2SInit(DSP_SSIZE_CODE_32BIT);
    DSPSampleRateSet(CODEC_SRATE_CODE_48K);
    DSPSampleSizeSet(DSP_SSIZE_CODE_32BIT);
    DMAInit(&dspInBuffer[0][0], &dspOutBuffer[0][0]);
    SineInit();
    FilterInit();
    I2S_RX_ENABLE();
    I2S_TX_ENABLE();


}

/*******************************************************************************************
* dspTask - Performs filtering on mixed and modulated signals once, handles audio output
*******************************************************************************************/
static void dspTask(void *p_arg){

    OS_ERR os_err;
    INT8U buffer_index;
    (void)p_arg;
    while(1){

        DB0_TURN_OFF();                             /* Turn off debug bit while waiting */
        buffer_index = DMAInPend(0, &os_err);
        DB0_TURN_ON();
        // DSP code goes here.
        //Actual filtering will only happen once
        switch(FilterState){
        case STARTUP:
            //Part 2 stuff happens here.
            arm_fir_q31(&firSine, (q31_t *) &signalTone.noisyQ[0], &signalTone.filteredAudio.samples[0], DSP_SAMPLES_PER_BLOCK); //Left Channel

            //Part 3 stuff happens here.
            arm_fir_q31(&firSine, (q31_t *) &signalTone.modQ[0], &signalTone.filteredQ[0], DSP_SAMPLES_PER_BLOCK);  //Filter demodulated signal
            arm_rfft_q31(&arm_rfft_sR_q31_len128Mod, &signalTone.filteredQ[0], &signalTone.FFT[0]);
            arm_cmplx_mag_q31(&signalTone.FFT[0], &signalTone.FFTMag[0], DSP_SAMPLES_PER_BLOCK);
            arm_max_q31(&signalTone.FFTMag[0], DSP_SAMPLES_PER_BLOCK, &signalTone.magQ, &signalTone.index);
            signalTone.index=FREQ_NORM(signalTone.index);
            FilterState = COMPLETE;
            break;
        case COMPLETE:
            break;
        default:
            break;
        }
        dspOutBuffer[DSP_LEFT_CH][buffer_index] = signalTone.filteredAudio; //Left Channel
        dspOutBuffer[DSP_RIGHT_CH][buffer_index] = signalTone.inputQ; //Right Channel
        if((buffer_index == 1)&&(dspStopReqFlag == 1)){
            OSSemPost(&dspFullStop,OS_OPT_POST_1,&os_err);
        }
    }
}
/*******************************************************************************************
* SineInit - Creates the original tone at f1 Hz, the mixed signal, and the demodulated signal.
*               Modulation and demodulation happen at the same time in this scenario.
* Parameters: None
* Returns: None
*******************************************************************************************/
static void SineInit(void){
    float32_t f1 = 3750;
    float32_t f2 = 18750;
    INT16U i;
    for(i=0;i<DSP_SAMPLES_PER_BLOCK;i++){
        signalTone.toneF[i]=arm_cos_f32(2*PI*i*f1/SAMPLE_RATE_HZ)/8;
        cosMod[i]=arm_cos_f32(2*PI*i*f2/SAMPLE_RATE_HZ);
        signalTone.modF[i]=signalTone.toneF[i]*cosMod[i]*cosMod[i];
        signalTone.noisyF[i]=signalTone.toneF[i]+cosMod[i]/8;
    }
    //max value of these is 1 (before /8 division) so no overflow should occur
    arm_float_to_q31(&signalTone.toneF[0], &signalTone.inputQ.samples[0], DSP_SAMPLES_PER_BLOCK);
    arm_float_to_q31(&signalTone.noisyF[0], &signalTone.noisyQ[0], DSP_SAMPLES_PER_BLOCK);
    arm_float_to_q31(&signalTone.modF[0], &signalTone.modQ[0], DSP_SAMPLES_PER_BLOCK);
}
/*******************************************************************************************
* FilterInit - Creates the filter coefficients using chassings formula. works with any value
*               of NUM_TAPS. NUM_TAPS must be odd.
* Parameters: None
* Returns: None
*******************************************************************************************/
static void FilterInit(void){
    float32_t fc = 4000;
    float32_t fs = SAMPLE_RATE_HZ;
    INT16U coeff_size = (NUM_TAPS-1)/2;
    INT16U i;
    firCoef32[coeff_size] = 2*fc/fs;
    for(i=1; i<=coeff_size; i++){
        firCoef32[i+coeff_size] = arm_sin_f32(PI*i*firCoef32[coeff_size])/(i*PI);
        firCoef32[coeff_size-i] = firCoef32[i+coeff_size];
    }
    //max value of float coeffs is 1 so no overflow can occur.
    arm_float_to_q31(&firCoef32[0], &firCoeffQ31[0], NUM_TAPS);
    arm_fir_init_q31(&firSine, NUM_TAPS, &firCoeffQ31[0], &firStateQ31[0], DSP_SAMPLES_PER_BLOCK);
    arm_rfft_init_q31(&arm_rfft_sR_q31_len128Mod, DSP_SAMPLES_PER_BLOCK, 0, 1);
}
/*******************************************************************************************
* DSPSampleSizeSet
* To set sample size you must set word size on both the CODEC and I2S
* Note: Does not change DMA or buffer word size which can be changed independently.
*******************************************************************************************/
void DSPSampleSizeSet(INT8U size_code){

    (void)CODECSetSampleSize(size_code);
    I2SWordSizeSet(size_code);
    dspParams.ssize = dspCodeToSize[size_code];

}
/*******************************************************************************************
* DSPSampleSizeGet
* To read current sample size code
*******************************************************************************************/
INT8U DSPSampleSizeGet(void){

    return dspParams.ssize;

}
/*******************************************************************************************
* DSPSampleRateGet
* To read current sample rate code
*******************************************************************************************/
INT16U DSPSampleRateGet(void){

    return dspParams.srate;

}
/*******************************************************************************************
* DSPSampleRateSet
* To set sample rate you set the rate on the CODEC
*******************************************************************************************/
void DSPSampleRateSet(INT8U rate_code){

    (void)CODECSetSampleRate(rate_code);
    dspParams.srate = dspCodeToRate[rate_code];

}
/*******************************************************************************************
* DSPStart
* Enable DMA to fill block with samples
*******************************************************************************************/
void DSPStartReq(void){

    dspStopReqFlag = 0;
    DMAStart();
    CODECEnable();
    CODECSetPage(0x00);
    CODECDefaultConfig();
    CODECHeadphoneOutOn();

}
/*******************************************************************************************
* DSPStop
* Disable DA after input/output buffers are full
*******************************************************************************************/
void DSPStopReq(void){

    dspStopReqFlag = 1;
    DMAStopFull();

}
/****************************************************************************************
 * DSP signal when buffer is full and DMA stopped
 * 04/16/2020 TDM
 ***************************************************************************************/

void DSPStopFullPend(OS_TICK tout, OS_ERR *os_err_ptr){
    OSSemPend(&dspFullStop, tout, OS_OPT_PEND_BLOCKING,(void *)0, os_err_ptr);
}
/****************************************************************************************
 * Return a pointer to the requested buffer
 * 04/16/2020 TDM
 ***************************************************************************************/

INT32S *DSPBufferGet(BUFF_ID_T buff_id){
    INT32S *buf_ptr = (void*)0;
    if(buff_id == LEFT_IN){
        buf_ptr = (INT32S *)&dspInBuffer[DSP_LEFT_CH][0];
    }else if(buff_id == RIGHT_IN){
        buf_ptr = (INT32S *)&dspInBuffer[DSP_RIGHT_CH][0];
    }else if(buff_id == RIGHT_OUT){
        buf_ptr = (INT32S *)&dspOutBuffer[DSP_RIGHT_CH][0];
    }else if(buff_id == LEFT_OUT){
        buf_ptr = (INT32S *)&dspOutBuffer[DSP_LEFT_CH][0];
    }else{
    }
    return buf_ptr;
}


