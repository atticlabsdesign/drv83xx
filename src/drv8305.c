#include "../drv8305.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

// extern void SPI_BufferExchange(void * bufferData, size_t bufferSize); //could probably replace this with a function pointer in the struct which the programmer defines with the settings, the functions would all then take the struct and use the function

const drv8305Settings_t DRV8305_DEFAULT_SETTINGS = {
    .highGateCtrl.bits  = 0x344,
    .lowGateCtrl.bits   = 0x344,
    .gateCtrl.bits      = 0x216,
    .icCtrl.bits        = 0x020,
    .shntCtrl.bits      = 0x000,
    .vregCtrl.bits      = 0x10A,
    .vdsCtrl.bits       = 0x0C8, //mine returns 2c8 but the high bit is in the reserved area so doesnt matter
};

/********************
---------------------
|  Private Functions |
---------------------
********************/
uint8_t inline drv8305StateMachine(bool, uint8_t *);


/********************
---------------------
|Function Definitions|
---------------------
********************/


/**
 * @brief Reads a single SPI register of a DRV8305 Motor Controller
 * 
 * @param addr Register address to be read
 * @param data Data read from register
 * @return drvError_t 
 */
drvError_t drv8305RegRead(drv8305Comms_t *spi, drv8305Addr_t addr, uint16_t *data){
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
    
    *data = (buffer & 0x3ff);
    

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
drvError_t drv8305RegWrite(drv8305Comms_t *spi, drv8305Addr_t addr, uint16_t *data){
    bool timeout = false;
    setPin8(spi->nCS, LOW);
    
    uint16_t buffer = (0<<15) | (addr << 11) | (*data & 0x3ff);

    #if  __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
        buffer = (buffer << 8 ) | (buffer >> 8); 
    #endif

    spi->spiInterface->BufferExchange(&buffer, sizeof(buffer));
    *data = (buffer & 0x3ff); //data now cointains the overwritten values, is this even useful?

    setPin8(spi->nCS, HIGH);

    if (timeout) {
        return DRV_NO_RESP;
    } else {
        return DRV_OK;
    }
    
}


drvError_t drv8305GetSettings(drv8305Dev_t *dev){
    drvError_t error = DRV_OK;
    
    error = drv8305RegRead(&dev->comms, DRV8305_HSGATECTRL_ADDR, &dev->settings.highGateCtrl.bits);
    if (error) {
        return error;
    }
    
    error = drv8305RegRead(&dev->comms, DRV8305_LSGATECTRL_ADDR, &dev->settings.lowGateCtrl.bits);
    if (error) {
        return error;
    }
    
    error = drv8305RegRead(&dev->comms, DRV8305_GATECTRL_ADDR, &dev->settings.gateCtrl.bits);
    if (error) {
        return error;
    }
    
    error = drv8305RegRead(&dev->comms, DRV8305_ICCTRL_ADDR, &dev->settings.icCtrl.bits);
    if (error) {
        return error;
    }
    
    error = drv8305RegRead(&dev->comms, DRV8305_SHNTCTRL_ADDR, &dev->settings.shntCtrl.bits);
    if (error) {
        return error;
    }
    
    error = drv8305RegRead(&dev->comms, DRV8305_VREGCTRL_ADDR, &dev->settings.vregCtrl.bits);
    if (error) {
        return error;
    }
    
    error = drv8305RegRead(&dev->comms, DRV8305_VDSCNTRL_ADDR, &dev->settings.vdsCtrl.bits);

    return error;
}

drvError_t drv8305GetFaults(drv8305Dev_t *dev){
    drvError_t error = DRV_OK;
    
    error = drv8305RegRead(&dev->comms, DRV8305_WARNING_ADDR, &dev->faults.WWR.bits);
    if (error) {
        return error;
    }
    
    error = drv8305RegRead(&dev->comms, DRV8305_VDSFAULT_ADDR, &dev->faults.VDS.bits);
    if (error) {
        return error;
    }

    error = drv8305RegRead(&dev->comms, DRV8305_ICFAULT_ADDR, &dev->faults.IC.bits);
    if (error) {
        return error;
    }

    error = drv8305RegRead(&dev->comms, DRV8305_VGSFAULT_ADDR, &dev->faults.VGS.bits);
    if (error) {
        return error;
    }

    return error;
}

drvError_t drv8305SetSettings(drv8305Dev_t *dev){ //TODO: try to figure out a way to avoid unneccessarily writing to registers
    drvError_t error = DRV_OK;          

    uint16_t buffer; //so we dont overwrite the values in the struct

    buffer = dev->settings.highGateCtrl.bits;
    error = drv8305RegWrite(&dev->comms, DRV8305_HSGATECTRL_ADDR, &buffer);
    if (error) {
        return error;
    }

    buffer = dev->settings.lowGateCtrl.bits; 
    error = drv8305RegWrite(&dev->comms, DRV8305_LSGATECTRL_ADDR, &buffer);
    if (error) {
        return error;
    }

    buffer = dev->settings.gateCtrl.bits;
    error = drv8305RegWrite(&dev->comms, DRV8305_GATECTRL_ADDR, &buffer);
    if (error) {
        return error;
    }

    buffer = dev->settings.icCtrl.bits; 
    error = drv8305RegWrite(&dev->comms, DRV8305_ICCTRL_ADDR, &buffer);
    if (error) {
        return error;
    }

    buffer = dev->settings.shntCtrl.bits;
    error = drv8305RegWrite(&dev->comms, DRV8305_SHNTCTRL_ADDR, &buffer);
    if (error) {
        return error;
    }

    buffer = dev->settings.vregCtrl.bits; 
    error = drv8305RegWrite(&dev->comms, DRV8305_VREGCTRL_ADDR, &buffer);
    if (error) {
        return error;
    }

    buffer = dev->settings.vdsCtrl.bits; 
    error = drv8305RegWrite(&dev->comms, DRV8305_VDSCNTRL_ADDR, &buffer);

    return error;

}

drvError_t drv8305Spin(drv8305Dev_t *dev, bool dir){
    drvError_t error = DRV_OK;
    uint8_t state = 0;

    switch (dev->settings.gateCtrl.PWM_MODE) {

        case DRV_PWM_INPUT_1:
            state = drv8305StateMachine(dir,&dev->pinCtrl.singlePwm.state); //TODO: NEED WAY TO CONTROL DWELL TIME

            setPin8(dev->pinCtrl.singlePwm.dwell, (state & 1));
            state >>= 1;
            setPin8(dev->pinCtrl.singlePwm.inlb, (state & 1));
            state >>= 1;
            setPin8(dev->pinCtrl.singlePwm.inhb, (state & 1));
            state >>= 1;
            setPin8(dev->pinCtrl.singlePwm.inla, (state & 1));

        break;

        case  DRV_PWM_INPUT_3:
            
        break;

        case  DRV_PWM_INPUT_6:
            
        break;
        
        default:
        error = DRV_UNKNOWN_SETTING;
    
    }

    return error;
}


drvError_t drv8305Stop(drv8305Dev_t *dev){
    drvError_t error = DRV_OK;
    switch (dev->settings.gateCtrl.PWM_MODE) {

        case DRV_PWM_INPUT_1:
            // all gpio low
            setPin8(dev->pinCtrl.singlePwm.inla, false);
            setPin8(dev->pinCtrl.singlePwm.inhb, false);
            setPin8(dev->pinCtrl.singlePwm.inlb, false);
            setPin8(dev->pinCtrl.singlePwm.dwell, false);
            
        break;

        case  DRV_PWM_INPUT_3:
            //pull all 3 channels low
            pwm8SetDutyCycle(dev->pinCtrl.triplePWM.pwm1,0, true);
            pwm8SetDutyCycle(dev->pinCtrl.triplePWM.pwm1,0, true);
            pwm8SetDutyCycle(dev->pinCtrl.triplePWM.pwm1,0, true);
        break;

        case  DRV_PWM_INPUT_6: //this is super scuffed
            pwm8SetDutyCycle(dev->pinCtrl.sixPWM.pwm1,0, true);
            pwm8SetDutyCycle(dev->pinCtrl.sixPWM.pwm2,0, true);
            pwm8SetDutyCycle(dev->pinCtrl.sixPWM.pwm3,0, true);
            pwm8SetDutyCycle(dev->pinCtrl.sixPWM.pwm4,0, true);
            pwm8SetDutyCycle(dev->pinCtrl.sixPWM.pwm5,0, true);
            pwm8SetDutyCycle(dev->pinCtrl.sixPWM.pwm6,0, true);
        break;
        
        default:
        error = DRV_UNKNOWN_SETTING;
    
    }

    return error;
}

drvError_t drv8305Align(drv8305Dev_t *dev){
    drvError_t error = DRV_OK;
    switch (dev->settings.gateCtrl.PWM_MODE) {

        case DRV_PWM_INPUT_1:
            // 1110
            setPin8(dev->pinCtrl.singlePwm.inla, true);
            setPin8(dev->pinCtrl.singlePwm.inhb, true);
            setPin8(dev->pinCtrl.singlePwm.inlb, true);
            setPin8(dev->pinCtrl.singlePwm.dwell, false);
        break;

        case  DRV_PWM_INPUT_3:
            //pwm first channel, low on other 2
        break;

        case  DRV_PWM_INPUT_6:
            //two pwm, 0101 on the rest
        break;
        
        default:
        error = DRV_UNKNOWN_SETTING;
    
    }

    return error;
}

uint8_t inline drv8305StateMachine(bool CCW, uint8_t *currentState){ //current state needs to be updated outside of this function before calling
    uint8_t STATE_LUT[12] = {0x6, 0x5, 0x4, 0xD, 0xC, 0x9, 0x8, 0xB, 0xA, 0x3, 0x2, 0x7};
    
    if (*currentState == 255 && !CCW) {
        *currentState = 11;
    } else if (*currentState == 12 && CCW) {
        *currentState = 0;
    } 

    return STATE_LUT[*currentState];
}

#ifdef DRV8305_DEBUG_MENUS

#include "../../dbgcli/dbgcli.h"

char drv8305DbgReadOptions[2] = {'h','a'};

const dbgCliCommand_t drv8305DbgRead = {
    .keyword = {"read", "r"},
    .help = "read(aka: r) <addr> to read registers -a to read all",
    .options = drv8305DbgReadOptions,
};

char drv8305DbgWriteOptions[2] = {'h'};

const dbgCliCommand_t drv8305DbgWrite = {
    .keyword = {"write", "w"},
    .help = "read(aka: r) <addr> to read registers -a to read all",
    .options = drv8305DbgWriteOptions,
};

const dbgCliCommand_t drv8305DbgHelp = {
    .keyword = {"read", "r"},
    .help = "read(aka: r) <addr> to read registers -a to read all",
    .options = drv8305DbgWriteOptions,
};

dbgCliCommand_t drv8305DbgCommands[2] = {drv8305DbgRead,drv8305DbgWrite};

const submenu_t drv8305DbgMenu = {
    .description = "This is the DRV8305 motor controller driver Debug Menu type\n Help to see a list of commands or a command followed by -h\n for more info on that command",
    .commands = drv8305DbgCommands,
    .cmdNum = sizeof(drv8305DbgCommands),
};

// ccw <duration> to spin the motor counter clockwise

// cw <duration> to spin the motor counter clockwise

#endif
