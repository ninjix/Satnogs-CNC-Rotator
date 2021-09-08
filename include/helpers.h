#ifndef LIBRARIES_HELPERS_H_
#define LIBRARIES_HELPERS_H_
#include <AccelStepper.h>
#include <globals.h>

/**************************************************************************/
/*!
    @brief    Convert degrees to steps according to step/revolution, rotator
              gear box ratio and microstep
    @param    deg
              Degrees in float format
    @return   Steps for stepper motor driver, int32_t
*/
/**************************************************************************/
int32_t deg2step(float deg)
{
    return (RATIO * SPR * deg / 360);
}

/**************************************************************************/
/*!
    @brief    Convert steps to degrees according to step/revolution, rotator
              gear box ratio and microstep
    @param    step
              Steps in int32_t format
    @return   Degrees in float format
*/
/**************************************************************************/
float step2deg(int32_t step)
{
    return (360.00 * step / (SPR * RATIO));
}


#endif