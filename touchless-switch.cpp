/************************************
 * FDC2214 Touchless Switch
 * Rev 1 - shabaz - April 2017
 *
 * Connections:
 * FRDM-K64F:
 * PTE25 - SDA
 * PTE24 - SCL
 * PTC2 - shut
 * PTC3 - addr
 * PTC4 - intb
 *
 * Optional WS2812 LEDs
 * PTB9 - Data
 ***********************************/

// defines
#include "mbed.h"
#include <ctype.h>
#include "arm_math.h"
#include "arm_const_structs.h"
#include "WS2812.h"
#include "PixelArray.h"
#define K64F

// WS2812 stuff
#define WS2812_BUF 8
#define NUM_COLORS 6
#define NUM_LEDS_PER_COLOR 10

// board-specific config. I only tested K64F
#ifdef K22F
#define INTB_PIN PTB16
#endif
#ifdef K64F
#define INTB_PIN PTC4
#endif

// optional low-pass filtering. Not needed! So set to 1.
#define BSIZE 1

// buffer sizes
#define MSIZE 16
#define LSIZE 32

/*********************************
 * FDC2214 register addresses
 *********************************/
// Converter output data registers (there is a formula to convert to
// actual capacitance values
#define DATA_MSW_CH0 0x00
#define DATA_LSW_CH0 0x01
#define DATA_MSW_CH1 0x02
#define DATA_LSW_CH1 0x03
#define DATA_MSW_CH2 0x04
#define DATA_LSW_CH2 0x05
#define DATA_MSW_CH3 0x06
#define DATA_LSW_CH3 0x07
// reference count registers (affects conversion time)
#define CMD_RCOUNT_CH0 0x08
#define CMD_RCOUNT_CH1 0x09
#define CMD_RCOUNT_CH2 0x0a
#define CMD_RCOUNT_CH3 0x0b
// offset registers (allows for higher-res readings when gain is set to higer than 1)
#define CMD_OFFSET_CH0 0x0c
#define CMD_OFFSET_CH1 0x0d
#define CMD_OFFSET_CH2 0x0e
#define CMD_OFFSET_CH3 0x0f
// oscillator stabilization time registers
#define CMD_SETTLECOUNT_CH0 0x10
#define CMD_SETTLECOUNT_CH1 0x11
#define CMD_SETTLECOUNT_CH2 0x12
#define CMD_SETTLECOUNT_CH3 0x13
// these set up the reference clock division to suit the tank circuit oscillator
// frequency and single- or differential-ended configuration
#define CMD_CLOCK_DIVIDERS_CH0 0x14
#define CMD_CLOCK_DIVIDERS_CH1 0x15
#define CMD_CLOCK_DIVIDERS_CH2 0x16
#define CMD_CLOCK_DIVIDERS_CH3 0x17
// these need to be set so that the oscillation amplitude falls within a certain range
#define CMD_DRIVE_CURRENT_CH0 0x1e
#define CMD_DRIVE_CURRENT_CH1 0x1f
#define CMD_DRIVE_CURRENT_CH2 0x20
#define CMD_DRIVE_CURRENT_CH3 0x21
// CMD_ERROR_CONFIG is used to set up what things you want to be notified about via the interrupt pin
#define CMD_ERROR_CONFIG 0x19
// CMD_MUX_CONFIG defines what channels are scanned, and a deglitch filter bandwidth:
#define CMD_MUX_CONFIG 0x1b
// CMD_RESET_DEV is used to reset the device and to program the gain
#define CMD_RESET_DEV 0x1c
// CMD_CONFIG is used to choose the channel if CMD_MUX_CONFIG is set to not multiplex multiple channels.
// this register is also used to bring the device out of standby mode
#define CMD_CONFIG 0x1a
// device status reporting:
#define CMD_STATUS 0x18
// dummy register value, use to identify end-of-configuration strings in this code
#define CMD_FIN 0xff

// the LED is wired to be switched on when the pin is high
#define LED_ON 0
#define LED_OFF 1

/***********************************
 * global variables
 ***********************************/
// WS2812B stuff 
PixelArray px(WS2812_BUF);
// these suit the K64F board, may need changing for other boards
WS2812 ws(D2, WS2812_BUF, 0, 5, 5, 0);
int colorbuf[NUM_COLORS];
// FDC2214 I2C address
const int fdcaddr = (0x2a<<1); // when addrpin=0
// transform variables
const static arm_cfft_instance_f32 *S;
float samples[LSIZE];
float magnitudes[MSIZE];
// LED object
DigitalOut myled(LED1);
// FDC2214 pins
#ifdef K22F
DigitalOut shut(PTB19);
DigitalOut addrpin(PTB18);
#endif
#ifdef K64F
DigitalOut shut(PTC2);
DigitalOut addrpin(PTC3);
DigitalIn intb(INTB_PIN);
#endif
// debug
Serial pc(USBTX, USBRX);
#ifdef K22F
I2C i2c(PTE0, PTE1);
#endif
#ifdef K64F
I2C i2c(PTE25, PTE24);
#endif
// FDC2214 configuration params have two bytes, msb and lsb here refer to bytes, not bits : )
typedef struct cfg_param_s
{
    char c_addr;
    char c_param_msb;
    char c_param_lsb;
} cfg_param_t;
// Take these settings from TI's eval board PC software in Menu->Registers window
cfg_param_t cfg_arr[]={
    {CMD_RCOUNT_CH0, 0xff, 0xff},
    {CMD_RCOUNT_CH1, 0xff, 0xff},
    {CMD_RCOUNT_CH2, 0xff, 0xff},
    {CMD_RCOUNT_CH3, 0xff, 0xff},
    {CMD_OFFSET_CH0, 0x00, 0x00},
    {CMD_OFFSET_CH1, 0x00, 0x00},
    {CMD_OFFSET_CH2, 0x00, 0x00},
    {CMD_OFFSET_CH3, 0x00, 0x00},
    {CMD_SETTLECOUNT_CH0, 0x04, 0x00},
    {CMD_SETTLECOUNT_CH1, 0x04, 0x00},
    {CMD_SETTLECOUNT_CH2, 0x04, 0x00},
    {CMD_SETTLECOUNT_CH3, 0x04, 0x00},
    {CMD_CLOCK_DIVIDERS_CH0, 0x10, 0x01},
    {CMD_CLOCK_DIVIDERS_CH1, 0x10, 0x01},
    {CMD_CLOCK_DIVIDERS_CH2, 0x10, 0x01},
    {CMD_CLOCK_DIVIDERS_CH3, 0x10, 0x01},
    {CMD_DRIVE_CURRENT_CH0, 0x88, 0x00},
    {CMD_DRIVE_CURRENT_CH1, 0x88, 0x00},
    {CMD_DRIVE_CURRENT_CH2, 0x88, 0x00},
    {CMD_DRIVE_CURRENT_CH3, 0x88, 0x00},
    {CMD_ERROR_CONFIG, 0x00, 0x01},
    {CMD_MUX_CONFIG, 0x82, 0x0c},
    {CMD_CONFIG, 0x1e, 0x01},
    {CMD_FIN, 0xff, 0xff}};

/********************************************
 * classes
 ********************************************/
// data analyzer
class Analyzer {
public:
    Analyzer(void)
    {
        mInh=1;
        mTot=0;
        mCount=0;
        mLCount=0;   
        mActLevel=0;
    }
    // plug in data so that a moving historical window can be processed
    void insert(unsigned int val)
    {
        int i;
        // store raw data
        for (i=1; i<BSIZE; i++)
        {
            mBuf[i-1]=mBuf[i];
        }
        mBuf[BSIZE-1]=val;
        // mTot implements a simple moving average (if required)
        mTot=mTot+val;
        mCount++;
        if (mCount>=BSIZE)
        {
            mCount=0;
            // store the averaged values into another array, to implement a long-term average
            for (i=1; i<LSIZE; i++)
            {
                mLta[i-1]=mLta[i];
            }
            mLta[LSIZE-1]=mTot/BSIZE;
            mTot=0;
            mLCount++;
            // is the long-term array now fully populated?
            if (mLCount>=LSIZE)
            {
                // don't inhibit analysis any longer
                mInh=0;
                // set to zero if you don't want to process so frequently
                mLCount=999; //0;
                // should move this out to a separate function, shouldn't really be in
                // 'insert'
                // we need floats for the transform
                for (i=0; i<LSIZE; i++)
                {
                    samples[i]=(float)mLta[i];
                }
                arm_cfft_f32(S, samples, 0, 1); // < match MSIZE, i.e. half of LSIZE
                arm_cmplx_mag_f32(samples, magnitudes, MSIZE);
                mActLevel=(unsigned int)magnitudes[MSIZE-1];
            }
        }
    }
    // these long-term average and short-term average get functions are not used
    // but left them there for general use of this class
    unsigned int getLta(void)
    {
        int i;
        unsigned int lta=0;
        if (mInh) return 0;
        for (i=0; i<LSIZE; i++)
        {
            lta+=mLta[i];
        }
        lta=lta/LSIZE;
        return(lta);
    }
    unsigned int getSta(void)
    {
        int i;
        unsigned int sta=0;
        if (mInh) return 0;
        for (i=0; i<BSIZE; i++)
        {
            sta+=mBuf[i];
        }
        sta=sta/BSIZE;
        return(sta);
    }        
    unsigned int getActLevel(void)
    {
        return mActLevel;
    }
private:
    unsigned int mBuf[BSIZE];
    unsigned int mLta[LSIZE];
    char mCount;
    char mLCount;
    unsigned int mTot;
    char mInh;
    unsigned int mActLevel;
};

// FDC2214 handler
class Fdc {
public:
    Fdc(void)
    { 
        mEnabled=0;
    }

    void enable()
    {
        mEnabled=1;
    }
    
    void disable()
    {
        mEnabled=0;
    }
    
    void reset()
    {
        char cmd[2];
        // reset FDC2214
        cmd[0]=CMD_RESET_DEV;
        cmd[1]=0x84; // output gain set to 8
        cmd[2]=0x00;
        i2c.write(fdcaddr, cmd, 3);
        wait(0.01);
    }

    void read()
    {
        char cmd[2];
        if (mEnabled)
        {
            // read channel 0
            cmd[0]=DATA_MSW_CH0;
            i2c.write(fdcaddr, cmd, 1, true); // repeated start
            i2c.read(fdcaddr, cmd, 2);
            mMsw[0]=(((unsigned int)cmd[0])<<8) | (((unsigned int)cmd[1]) & 0x00ff);
            cmd[0]=DATA_LSW_CH0;
            i2c.write(fdcaddr, cmd, 1, true); // repeated start
            i2c.read(fdcaddr, cmd, 2);
            mLsw[0]=(((unsigned int)cmd[0])<<8) | (((unsigned int)cmd[1]) & 0x00ff);
        
            // read channel 1 although the result isn't needed for this project
            cmd[0]=DATA_MSW_CH1;
            i2c.write(fdcaddr, cmd, 1, true); // repeated start
            i2c.read(fdcaddr, cmd, 2);
            mMsw[1]=(((unsigned int)cmd[0])<<8) | (((unsigned int)cmd[1]) & 0x00ff);
            cmd[0]=DATA_LSW_CH1;
            i2c.write(fdcaddr, cmd, 1, true); // repeated start
            i2c.read(fdcaddr, cmd, 2);
            mLsw[1]=(((unsigned int)cmd[0])<<8) | (((unsigned int)cmd[1]) & 0x00ff);
            
            // read ch3 and ch4 although we're not using these
            cmd[0]=DATA_MSW_CH2;
            i2c.write(fdcaddr, cmd, 1, true); i2c.read(fdcaddr, cmd, 2);
            cmd[0]=DATA_LSW_CH2;
            i2c.write(fdcaddr, cmd, 1, true); i2c.read(fdcaddr, cmd, 2);
            cmd[0]=DATA_MSW_CH3;
            i2c.write(fdcaddr, cmd, 1, true); i2c.read(fdcaddr, cmd, 2);
            cmd[0]=DATA_LSW_CH3;
            i2c.write(fdcaddr, cmd, 1, true); i2c.read(fdcaddr, cmd, 2);
            
            // read status register to clear interrupt pin
            cmd[0]=CMD_STATUS;
            i2c.write(fdcaddr, cmd, 1, true); // repeated start
            i2c.read(fdcaddr, cmd, 2);
        
        }
        else
        {
            pc.printf("FDC object not enabled yet\n\r");
        }
    }
    // placeholder. We don't require the actual freq in Hz, nor the capacitance in Farads, for this project
    int getfreq(char chan)
    {
        double afreq;
        unsigned int tfreq;
        tfreq=(mMsw[chan]<<16) | (mLsw[chan]&0xffff);
        printf("tfreq: %d\n\r", tfreq);
        // todo
        return(0);
    }
    // retrieve the FDC capacitance to digital conversion data register value (i.e. raw value)
    unsigned int getDat(char chan)
    {
        unsigned int tdat;
        tdat=(mMsw[chan]<<16) | (mLsw[chan]&0xffff);
        return(tdat);
    }

private:
    unsigned char mEnabled;
    //InterruptIn mInterrupt;
    volatile unsigned int mMsw[2];
    volatile unsigned int mLsw[2];
};

// create the objects
Fdc fdc;
Analyzer analyzer;

/*************************************
 * function prototypes
 *************************************/
 
 /************************************
 * functions
 *************************************/
// debugger
void callback()
{
    char c;
    c=pc.getc();
}

// WS2812B stuff, not particularly interesting
// these just light the WS2812B array in red or switch it all off
void
ws_on(void)
{
    for (int i = 0; i < WS2812_BUF; i++) {
        px.Set(i, colorbuf[0]);
    }   
    for (int j=0; j<WS2812_BUF; j++) {
        // px.SetI(pixel position, II value)
        px.SetI(j, 0xff);
    }
    for (int z=WS2812_BUF; z >= 0 ; z--) {
            ws.write_offsets(px.getBuf(),z,z,z);
            wait(0.075);
    }
}

void
ws_off(void)
{
    for (int i = 0; i < WS2812_BUF; i++) {
        px.Set(i, colorbuf[3]);
    }   
    for (int j=0; j<WS2812_BUF; j++) {
        // px.SetI(pixel position, II value)
        px.SetI(j, 0x00);
    }
    for (int z=WS2812_BUF; z >= 0 ; z--) {
            ws.write_offsets(px.getBuf(),z,z,z);
            wait(0.075);
    }
}


// main() runs in its own thread in the OS
int
main(void)
{
    char cmd[10];
    unsigned int dat;
    unsigned int act;
    char evt=0;
    char dstat=0;
    unsigned int oldact=0; // crude output filter. good enough!
        
    myled=LED_OFF; // switch off the LED
    
    int not_finished=1;
    int i=0;
    
    // set up debugger
    pc.baud(115200);
    pc.attach(&callback);
    pc.printf("Hello\n\r");
    
    // WS2812B stuff
    ws.useII(WS2812::PER_PIXEL); // use per-pixel intensity scaling
    // set up the colours we want to draw with
    colorbuf[0]=0x2f0000;
    colorbuf[1]=0x2f2f00;
    colorbuf[2]=0x002f00;
    colorbuf[3]=0x002f2f;
    colorbuf[4]=0x00002f;
    colorbuf[5]=0x2f002f;

    // transform stuff
    S=& arm_cfft_sR_f32_len16;
    
    // handle FDC2214 startup
    shut=1; // put FDC2214 into shutdown
    addrpin=0; // clear I2C address pin
    wait(0.1);
    shut=0; // bring FDC2214 out of shutdown mode
    wait(0.01);
    // reset FDC2214
    fdc.reset();

    // handle FDC2214 configuration
    not_finished=1;
    i=0;
    do
    {
        cmd[0]=cfg_arr[i].c_addr;
        cmd[1]=cfg_arr[i].c_param_msb;
        cmd[2]=cfg_arr[i].c_param_lsb;
        i2c.write(fdcaddr, cmd, 3);
        i++;
        if (cfg_arr[i].c_addr == CMD_FIN)
        {
            not_finished=0;
            break;
        }
    } while(not_finished);
    pc.printf("cfg done\n\r");
    fdc.enable();
    
    // main forever loop
    while(1)
    {
        if (!intb) // conversions ready?
        {
            fdc.read();
            dat=fdc.getDat(0);
            analyzer.insert(dat); // plug in the data
            act=analyzer.getActLevel();
            // test to see if we have sufficient hand-swipe activity.
            // 4000 worked fine but so did 3000. It's not a super critical value
            if ((act>4000) && (oldact>4000))
            {
                //evt implements a one-shot trigger
                if (evt==0)
                {
                    evt=1;
                    // dstat implements a toggle action
                    if (dstat==0)
                    {
                        dstat=1;  
                        myled=LED_ON;
                        ws_on();
                         
                    }
                    else
                    {
                        dstat=0;
                        myled=LED_OFF;
                        ws_off();
                    }
                
                }    
            }
            else if ((act<=4000) && (oldact<=4000))
            {
                //clear the one-shot trigger
                evt=0;
            }
            oldact=act;
        }
    }
    
    return(0); // warning on this line is ok
}

