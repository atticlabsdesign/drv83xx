#ifndef DRV83XX_TYPES_H
#define DRV83XX_TYPES_H

#define DRV_CCW true
#define DRV_CW  false

typedef enum drvError{
    DRV_OK,
    DRV_NO_RESP,
    DRV_UNKNOWN_SETTING
} drvError_t;


#endif