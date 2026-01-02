#ifndef DRV83XX_TYPES_H
#define DRV83XX_TYPES_H

typedef enum drvError{
    DRV_OK,
    DRV_NO_RESP,
    DRV_UNKNOWN_SETTING
} drvError_t;

typedef enum drvState{
    DRV_STOP,
    DRV_ALIGN,
    DRV_CCW,
    DRV_CW
} drvState_t;


#endif