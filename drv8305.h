#ifndef DRV8305_H
#define DRV8305_H

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "drv83xx_types.h"
#include "../pwm/pwm.h"
#include "../pinctrl/pinctrl.h"
#include "../../SPI_test_config/mcc/mcc_generated_files/spi/spi.h" //need to find a better way of referencing mcc files

// Masks for SPI registers

/********************
---------------------
|Warnings & Watchdog|
---------------------
********************/

#define DRV8305_WWR_OTW_MASK    0x001
#define DRV8305_WWR_TEMP_MASK   0x10E
#define DRV8305_WWR_VCHP_MASK   0x010
#define DRV8305_WWR_VDS_MASK    0x020
#define DRV8305_WWR_VDDO_MASK   0x040
#define DRV8305_WWR_VDDU_MASK   0x080
#define DRV8305_WWR_FAULT_MASK  0x400

/********************
---------------------
|   OV/VDS Faults   |
---------------------
********************/
#define DRV8305_VDSF_VDS        0x3F0
#define DRV8305_VDSF_SOCP       0x007

/********************
---------------------
|     IC Faults     |
---------------------
********************/
#define DRV8305_ICF_VCPH        0x007
#define DRV8305_IC_UVLO         0x070
#define DRV8305_IC_WD           0x200
#define DRV8305_IC_VDD_UVLO     0x200


typedef enum {
    DRV8305_WARNING_ADDR     = 0x1,
    DRV8305_VDSFAULT_ADDR    = 0x2,
    DRV8305_ICFAULT_ADDR     = 0x3,
    DRV8305_VGSFAULT_ADDR    = 0x4,
    DRV8305_HSGATECTRL_ADDR  = 0x5,
    DRV8305_LSGATECTRL_ADDR  = 0x6,
    DRV8305_GATECTRL_ADDR    = 0x7,
    DRV8305_ICCTRL_ADDR      = 0x9,
    DRV8305_SHNTCTRL_ADDR    = 0xA,
    DRV8305_VREGCTRL_ADDR    = 0xB,
    DRV8305_VDSCNTRL_ADDR    = 0xC
} drv8305Addr_t;

/*________________________________________________*/

#if  __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__ //flip the struct order so that the bits member remains readable on the host device 
// this works on ATMEGA328p, need to check alt implementation on a big endien device
typedef struct {

    union {
        struct { // 0x1
            bool OTW        : 1 ;
            bool TEMP_FLAG3 : 1 ;
            bool TEMP_FLAG2 : 1 ;
            bool TEMP_FLAG1 : 1 ;
            bool VCHP_UVFL  : 1 ;
            bool VDS_STATUS : 1 ;
            bool PVDD_OVFL  : 1 ;
            bool PVDD_UVFL  : 1 ;
            bool TEMP_FLAG4 : 1 ;
            int             : 1 ; //RSVD
            bool FAULT      : 1 ;
            int             : 5 ; // N/A
        };

        uint16_t bits;

    } WWR; //warnings and watchdog reset

    union {
        struct { // 0x2
            bool SNS_A_OCP  : 1 ;
            bool SNS_B_OCP  : 1 ;
            bool SNS_C_OCP  : 1 ;
            int             : 2 ; //RSVD
            bool VDS_LC     : 1 ;
            bool VDS_HC     : 1 ;
            bool VDS_LB     : 1 ;
            bool VDS_HB     : 1 ;
            bool VDS_LA     : 1 ;
            bool VDS_HA     : 1 ;
            int             : 5 ; // N/A
        };

        uint16_t bits;

    } VDS; //Overvoltage/VDS    

    union {
        struct { // 0x3
            bool VCPH_OVLO_ABS  : 1 ;
            bool VCPH_OVLO      : 1 ;
            bool VCPH_OVLO2     : 1 ;
            int                 : 1 ; //RSVD
            bool VCP_LSD_UVLO2  : 1 ;
            bool AVDD_UVLO      : 1 ;
            bool VREG_UV        : 1 ;
            int                 : 1 ; //RSVD
            bool OTSD           : 1 ;
            bool WD_FAULT       : 1 ;
            bool PVDD_UVLO2     : 1 ;
            int                 : 5 ; // N/A
        };

        uint16_t bits;

    } IC; //IC

    union {
        struct { // 0x3
            int             : 5 ; //RSVD
            bool VGS_LC     : 1 ;
            bool VGS_HC     : 1 ;
            bool VGS_LB     : 1 ;
            bool VGS_HB     : 1 ;
            bool VGS_LA     : 1 ;
            bool VGS_HA     : 1 ;
            int             : 5 ; // N/A
        };

        uint16_t bits;

    } VGS; //VGS

} drv8305Faults_t;




#else

typedef struct {

    union {
        struct { // 0x1
            int             : 5 ; // N/A
            bool FAULT      : 1 ;
            int             : 1 ; //RSVD
            bool TEMP_FLAG4 : 1 ;
            bool PVDD_UVFL  : 1 ;
            bool PVDD_OVFL  : 1 ;
            bool VDS_STATUS : 1 ;
            bool VCHP_UVFL  : 1 ;
            bool TEMP_FLAG1 : 1 ;
            bool TEMP_FLAG2 : 1 ;
            bool TEMP_FLAG3 : 1 ;
            bool OTW        : 1 ;
        };

        uint16_t bits;

    } WWR; //warnings and watchdog reset

    union {
        struct { // 0x2
            int             : 5 ; // N/A
            bool VDS_HA     : 1 ;
            bool VDS_LA     : 1 ;
            bool VDS_HB     : 1 ;
            bool VDS_LB     : 1 ;
            bool VDS_HC     : 1 ;
            bool VDS_LC     : 1 ;
            int             : 2 ; //RSVD
            bool SNS_C_OCP  : 1 ;
            bool SNS_B_OCP  : 1 ;
            bool SNS_A_OCP  : 1 ;
        };

        uint16_t bits;

    } VDS; //Overvoltage/VDS    

    union {
        struct { // 0x3
            int                 : 5 ; // N/A
            bool PVDD_UVLO2     : 1 ;
            bool WD_FAULT       : 1 ;
            bool OTSD           : 1 ;
            int                 : 1 ; //RSVD
            bool VREG_UV        : 1 ;
            bool AVDD_UVLO      : 1 ;
            bool VCP_LSD_UVLO2  : 1 ;
            int                 : 1 ; //RSVD
            bool VCPH_OVLO2     : 1 ;
            bool VCPH_OVLO      : 1 ;
            bool VCPH_OVLO_ABS  : 1 ;
        };

        uint16_t bits;

    } IC; //IC

    union {
        struct { // 0x3
            int             : 5 ; // N/A
            bool VGS_HA     : 1 ;
            bool VGS_LA     : 1 ;
            bool VGS_HB     : 1 ;
            bool VGS_LB     : 1 ;
            bool VGS_HC     : 1 ;
            bool VGS_LC     : 1 ;
            int             : 5 ; //RSVD
        };

        uint16_t bits;

    } VGS; //VGS

} drv8305Faults_t;


#endif

    // ALL OF THESE SETTINGS STRUCTS NEED TO CHANGE, I SWAPPED THE BIT ORDER INSTEAD OF THE BYTE ORDER

/*________________________________________________*/

typedef enum {

    PKTSRC_220NS,
    PKTSRC_440NS,
    PKTSRC_880NS,
    PKTSRC_1780NS,

} drv8305TDrive_t;

typedef enum {
    PKISNK_20MA,
    PKISNK_30MA,
    PKISNK_40MA,
    PKISNK_50MA,
    PKISNK_60MA,
    PKISNK_70MA,
    PKISNK_80MA,
    PKISNK_250MA,
    PKISNK_500MA,
    PKISNK_750MA,
    PKISNK_1000MA,
    PKISNK_1250MA,
} drv8305IDriveN_t;

typedef enum {
    PKISRC_10MA,
    PKISRC_20MA,
    PKISRC_30MA,
    PKISRC_40MA,
    PKISRC_50MA,
    PKISRC_60MA,
    PKISRC_70MA,
    PKISRC_125MA,
    PKISRC_250MA,
    PKISRC_500MA,
    PKISRC_750MA,
    PKISRC_1000MA,
} drv8305IDrivep_t;


#if  __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__ //flip the struct order so that the bits member remains readable on the host device

typedef union {
    struct { // 0x5 and 0x6
        drv8305IDrivep_t   IDRIVEP : 4;
        drv8305IDriveN_t   IDRIVEN : 4;
        drv8305TDrive_t    TDRIVE  : 2;
        int                        : 6; //RSVD

    };

    uint16_t bits;

} drv8305_XS_GATE_DRIVE_CONTROL_t;

#else

typedef union { 
    struct { // 0x5 and 0x6
        int                        : 6; //RSVD
        drv8305TDrive_t    TDRIVEN : 2;
        drv8305IDriveN_t   IDRIVEN : 4;
        drv8305IDrivep_t   IDRIVEP : 4;

    };

    uint16_t bits;

} drv8305_XS_GATE_DRIVE_CONTROL_t;

#endif


/*________________________________________________*/

typedef enum {
    DIODE,
    ACTIVE
} drv8305CommOption_t;

typedef enum {
    DRV_PWM_INPUT_6,
    DRV_PWM_INPUT_3,
    DRV_PWM_INPUT_1
} drv8305PwmMode_t;

typedef enum {
    DEADTIME_35NS,
    DEADTIME_52NS,
    DEADTIME_88NS,
    DEADTIME_440NS,
    DEADTIME_880NS,
    DEADTIME_1760NS,
    DEADTIME_3520NS,
    DEADTIME_5280NS,
} drv8305DeadTime_t;


typedef enum {
    VDSBLNK_0US,
    VDSBLNK_1_75US,
    VDSBLNK_3_5US,
    VDSBLNK_7US,
} drv8305TBlank_t;

typedef enum {
    VDSGLTCH_0US,
    VDSGLTCH_1_75US,
    VDSGLTCH_3_5US,
    VDSGLTCH_7US,
} drv8305TVds_t;


#if  __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__

typedef union{
    struct { //0x7
        drv8305TVds_t       TVDS        : 2;
        drv8305TBlank_t     TBLANK      : 2;
        drv8305DeadTime_t   DEAD_TIME   : 3;
        drv8305PwmMode_t    PWM_MODE    : 2;
        drv8305CommOption_t COMM_OPTION : 1;
        int                             : 6; //RSVD
    };

    uint16_t bits;
} drv8305_GATE_DRIVE_CONTROL_t; 

#else

typedef union{
    struct { //0x7
        int                             : 6; //RSVD
        drv8305CommOption_t COMM_OPTION : 1;
        drv8305PwmMode_t    PWM_MODE    : 2;
        drv8305DeadTime_t   DEAD_TIME   : 3;
        drv8305TBlank_t     TBLANK      : 2;
        drv8305TVds_t       TVDS        : 2;
    };

    uint16_t bits;
} drv8305_GATE_DRIVE_CONTROL_t; 

#endif

/*________________________________________________*/

typedef enum {
    WD_10MS,
    WD_20MS,
    WD_50MS,
    WD_100MS
} drv8305WdDelay_t;

typedef enum {
    VCPH_UV_4V9,
    VCPH_UV_4V6
} drv8305VCPH_t;

#if  __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__

typedef union{
    struct {  //0x9
        drv8305VCPH_t       SET_VCPH_UV     : 1;
        bool                CLR_FLTS        : 1;
        bool                SLEEP           : 1;
        bool                WD_EN           : 1;
        bool                DIS_SNS_OCP     : 1;
        drv8305WdDelay_t    WD_DLY          : 2;        
        bool                EN_SNS_CLAMP    : 1;
        bool                DIS_GDRV_FAULT  : 1;
        bool                DIS_PVDD_UVLO2  : 1;
        bool                FLIP_OTSD       : 1;        
        int                                 : 5; //RSVD

    };

    uint16_t bits;
} drv8305_IC_OPERATION_t;

#else

typedef union{
    struct {  //0x9
        int                                 : 5; //RSVD
        bool                FLIP_OTSD       : 1;
        bool                DIS_PVDD_UVLO2  : 1;
        bool                DIS_GDRV_FAULT  : 1;
        bool                EN_SNS_CLAMP    : 1;
        drv8305WdDelay_t    WD_DLY          : 2;
        bool                DIS_SNS_OCP     : 1;  
        bool                WD_EN           : 1;
        bool                SLEEP           : 1;
        bool                CLR_FLTS        : 1;
        drv8305VCPH_t       SET_VCPH_UV     : 1;
    };

    uint16_t bits;
} drv8305_IC_OPERATION_t;

#endif
/*________________________________________________*/

typedef enum {
    CS_BLANK_0NS,
    CS_BLANK_500NS,
    CS_BLANK_2500NS,
    CS_BLANK_10000NS,
} drv8305CsBlank_t;

typedef enum {
    CSGAIN_10,
    CSGAIN_20,
    CSGAIN_40,
    CSGAIN_80,
} drv8305CsGain_t;

#if  __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__

typedef union {
    struct {  //0xA
        drv8305CsGain_t GAIN_CS1    : 2;
        drv8305CsGain_t GAIN_CS2    : 2;
        drv8305CsGain_t GAIN_CS3    : 2;
        drv8305CsBlank_t CS_BLANK   : 2;
        bool DC_CAL_CH1             : 1;
        bool DC_CAL_CH2             : 1;
        bool DC_CAL_CH3             : 1;
        int                         : 5; //RSVD
    };

    uint16_t bits;
} drv8305_SHUNT_AMP_CTRL_t;

#else

typedef union {
    struct {  //0xA
        int                         : 5; //RSVD
        bool DC_CAL_CH3             : 1;
        bool DC_CAL_CH2             : 1;
        bool DC_CAL_CH1             : 1;
        drv8305CsBlank_t CS_BLANK   : 2;
        drv8305CsGain_t GAIN_CS3    : 2;
        drv8305CsGain_t GAIN_CS2    : 2;
        drv8305CsGain_t GAIN_CS1    : 2;
    };

    uint16_t bits;
} drv8305_SHUNT_AMP_CTRL_t;

#endif
/*________________________________________________*/

typedef enum {
    SCALE_2 = 1,
    SCALE_4 = 2
} drv8305VrefScale_t;

typedef enum {
    SLEEPDLY_0us,
    SLEEPDLY_10us,
    SLEEPDLY_50us,
    SLEEPDLY_1000us
} drv8305SleepDly_t;

typedef enum {
    VREGUV_9,
    VREGUV_8,
    VREGUV_7
} drv8305VregUv_t;


#if  __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__

typedef union {
    struct  {  //0xB
        drv8305VregUv_t     VREG_UV_LEVEL   : 2;
        bool                DIS_VREG_PWRGD  : 1; 
        drv8305SleepDly_t   SLEEP_DLY       : 2;
        int                                 : 3; //RSVD
        drv8305VrefScale_t  VREF_SCALE      : 2;
        int                                 : 6; //RSVD
    };

    uint16_t bits;
} drv8305_VOLAGE_REG_CTRL_t;

#else

typedef union {
    struct  {  //0xB
        int                                 : 6; //RSVD
        drv8305VrefScale_t  VREF_SCALE      : 2;
        int                                 : 3; //RSVD
        drv8305SleepDly_t   SLEEP_DLY       : 2;
        bool                DIS_VREG_PWRGD  : 1; 
        drv8305VregUv_t     VREG_UV_LEVEL   : 2;
    };

    uint16_t bits;
} drv8305_VOLAGE_REG_CTRL_t;

#endif


/*________________________________________________*/

typedef enum {
    VDSLVL_0V060,
    VDSLVL_0V068,
    VDSLVL_0V076,
    VDSLVL_0V086,
    VDSLVL_0V097,
    VDSLVL_0V109,
    VDSLVL_0V123,
    VDSLVL_0V138,
    VDSLVL_0V155,
    VDSLVL_0V175,
    VDSLVL_0V197,
    VDSLVL_0V222,
    VDSLVL_0V250,
    VDSLVL_0V282,
    VDSLVL_0V317,
    VDSLVL_0V358,
    VDSLVL_0V403,
    VDSLVL_0V454,
    VDSLVL_0V511,
    VDSLVL_0V576,
    VDSLVL_0V648,
    VDSLVL_0V730,
    VDSLVL_0V822,
    VDSLVL_0V926,
    VDSLVL_1V043,
    VDSLVL_1V175,
    VDSLVL_1V324,
    VDSLVL_1V491,
    VDSLVL_1V679,
    VDSLVL_1V892,
    VDSLVL_2V131    
} drv8305VdsLevel_t;

typedef enum {
    VDSPROTECTION_LATCH,
    VDSPROTECTION_REPORT,
    VDSPROTECTION_DISABLED,
} drv8305VdsMode_t;


#if  __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__

typedef union {
    struct  { //0xC
        drv8305VdsMode_t    VDS_MODE    : 3;
        drv8305VdsLevel_t   VDS_LEVEL   : 5;
        int                             : 8; //RSVD
    };

    uint16_t bits;
} drv8305_VDS_SENSE_CTRL_t;

#else

typedef union {
    struct  { //0xC
        int                             : 8; //RSVD
        drv8305VdsLevel_t   VDS_LEVEL   : 5;
        drv8305VdsMode_t    VDS_MODE    : 3;
    };

    uint16_t bits;
} drv8305_VDS_SENSE_CTRL_t;

#endif


/*________________________________________________*/

typedef struct {
    const struct SPI_INTERFACE *spiInterface; // this should change eventually to something more universal than the mcc generated struct
    const pinId8_t              nCS;
}drv8305Comms_t;



typedef struct {
    drv8305_XS_GATE_DRIVE_CONTROL_t highGateCtrl;      
    drv8305_XS_GATE_DRIVE_CONTROL_t lowGateCtrl;
    drv8305_GATE_DRIVE_CONTROL_t    gateCtrl;
    drv8305_IC_OPERATION_t          icCtrl;
    drv8305_SHUNT_AMP_CTRL_t        shntCtrl;
    drv8305_VOLAGE_REG_CTRL_t       vregCtrl;
    drv8305_VDS_SENSE_CTRL_t        vdsCtrl;

} drv8305Settings_t;

const drv8305Settings_t DRV8305_DEFAULT_SETTINGS;

typedef struct {
    union {
        struct {
            pwmCh8_t pwm;
            pinId8_t inla;
            pinId8_t inhb;
            pinId8_t inlb;
            pinId8_t dwell; //
            uint8_t  state;
        } singlePwm;

        struct {
            pwmCh8_t pwm1;
            pwmCh8_t pwm2;
            pwmCh8_t pwm3;
        } triplePWM;

        struct {
            pwmCh8_t pwm1;
            pwmCh8_t pwm2;
            pwmCh8_t pwm3;
            pwmCh8_t pwm4;
            pwmCh8_t pwm5;
            pwmCh8_t pwm6;
        } sixPWM; 
    };
    pinId8_t enGate;
    pinId8_t nFault;
    pinId8_t pwrgd;
    //still need some current amplifier stuff
    pinId8_t wake;

} drv8305Pins_t;

/**
 * @brief device struct for DRV8305 Motor Controller
 * @details 
 * example minimum definition:
 *    drv8305Dev_t motCtrlDev0 = {
 *       .comms.spiInterface = &SPI,
 *       .comms.nCS.port = &PORTB, 
 *       .comms.nCS.pin = 1
 *      };
 */
typedef struct {
    drv8305Settings_t settings;
    drv8305Faults_t faults;
    drv8305Comms_t comms;
    drv8305Pins_t pinCtrl;
    
} drv8305Dev_t;

/********************
---------------------
|  Public Functions |
---------------------
********************/


/**
 * @brief Reads a single SPI register of a DRV8305 Motor Controller
 * 
 * @param addr Register address to be read
 * @param data Data read from register
 * @return drvError_t 
 */
inline drvError_t drv8305RegRead(drv8305Comms_t *spi, drv8305Addr_t addr, uint16_t *data){
    bool timeout = false;
    
    setPin8(spi->nCS, LOW);

    uint16_t buffer = (1<<15) | (addr << 11);

    #if  __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
        buffer = (buffer << 8 ) | (buffer >> 8); 
    #endif

    spi->spiInterface->BufferExchange(&buffer, sizeof(buffer));

    #if  __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
        buffer = (buffer << 8 ) | (buffer >> 8); 
    #endif
    *data = (buffer & 0x7ff);
    

    setPin8(spi->nCS, HIGH);
    
    if (timeout) {
        return DRV_NO_RESP;
    } else {
        return DRV_OK;
    }
    
}

/**
 * @brief Writes to a single SPI register of a DRV8305 Motor Controller
 * 
 * @param addr Register address to be written to
 * @param data Data to be written to register, contains the overwritten value of the register after writing
 * @return drvError_t 
 */
inline drvError_t drv8305RegWrite(drv8305Comms_t *spi, drv8305Addr_t addr, uint16_t *data){
    bool timeout = false;
    setPin8(spi->nCS, LOW);
    
    uint16_t buffer = (0<<15) | (addr << 11) | (*data & 0x7ff);

    #if  __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
        buffer = (buffer << 8 ) | (buffer >> 8); 
    #endif

    spi->spiInterface->BufferExchange(&buffer, sizeof(buffer));
    *data = (buffer & 0x7ff); //data now cointains the overwritten values, is this even useful?

    setPin8(spi->nCS, HIGH);

    if (timeout) {
        return DRV_NO_RESP;
    } else {
        return DRV_OK;
    }
    
}

drvError_t drv8305SetSettings(drv8305Dev_t *);

drvError_t drv8305GetSettings(drv8305Dev_t *);

drvError_t drv8305GetFaults(drv8305Dev_t *);


drvError_t drv8305CCW(drv8305Dev_t*);

drvError_t drv8305CW(drv8305Dev_t*);

drvError_t drv8305Stop(drv8305Dev_t*);

drvError_t drv8305Align(drv8305Dev_t*);
#endif