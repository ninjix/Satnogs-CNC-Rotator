
// #define __DEBUG__

#ifdef __DEBUG__
#define DEBUG(...) Serial.print(__VA_ARGS__)
#else
#define DEBUG(...)
#endif

#include <AccelStepper.h>
#include <Wire.h>
#include <globals.h>
#include <helpers.h>
#include <easycomm.h>
#include <rotator_pins.h>
#include <endstop.h>
#include <watchdog.h>

// Instantiate objects
uint32_t t_run = 0; // run time of uC
wdt_timer wdt;
easycomm comm;

AccelStepper stepper_az(1, M1IN1, M1IN2);
AccelStepper stepper_el(1, M2IN1, M2IN2);
endstop switch_az(SW1, DEFAULT_HOME_STATE);
endstop switch_el(SW2, DEFAULT_HOME_STATE);

enum _rotator_error homing(int32_t seek_az, int32_t seek_el);
int32_t deg2step(float deg);
float step2deg(int32_t step);

void setup()
{
    // Homing switch
    switch_az.init();
    switch_el.init();

    // Serial Communication
    comm.easycomm_init();
    Serial.println("SATNOGS Rotator is ready.");

    // Stepper Motor setup
    stepper_az.setEnablePin(MOTOR_EN);
    stepper_az.setPinsInverted(false, false, true);
    stepper_az.enableOutputs();
    stepper_az.setMaxSpeed(MAX_SPEED);
    stepper_az.setAcceleration(MAX_ACCELERATION);
    stepper_az.setMinPulseWidth(MIN_PULSE_WIDTH);
    stepper_el.setPinsInverted(false, false, true);
    stepper_el.enableOutputs();
    stepper_el.setMaxSpeed(MAX_SPEED);
    stepper_el.setAcceleration(MAX_ACCELERATION);
    stepper_el.setMinPulseWidth(MIN_PULSE_WIDTH);

    // Initialize WDT
    wdt.watchdog_init();
}

void loop()
{
    // Update WDT
    wdt.watchdog_reset();

    // Get end stop status
    rotator.switch_az = switch_az.get_state();
    rotator.switch_el = switch_el.get_state();

    // Run easycomm implementation
    comm.easycomm_proc();

    // Get position of both axis
    control_az.input = step2deg(stepper_az.currentPosition());
    control_el.input = step2deg(stepper_el.currentPosition());

    // Check rotator status
    if (rotator.rotator_status != error)
    {
        if (rotator.homing_flag == false)
        {
            // Check home flag
            rotator.control_mode = position;

            // Homing
            rotator.rotator_error = homing(
                deg2step(-MAX_M1_ANGLE),
                deg2step(-MAX_M2_ANGLE));
            if (rotator.rotator_error == no_error)
            {
                // No error
                rotator.rotator_status = idle;
                rotator.homing_flag = true;
            }
            else
            {
                // Error
                rotator.rotator_status = error;
                rotator.rotator_error = homing_error;
            }
        }
        else
        {
            // Control Loop
            stepper_az.moveTo(deg2step(control_az.setpoint));
            stepper_el.moveTo(deg2step(control_el.setpoint));
            rotator.rotator_status = pointing;

            // Move azimuth and elevation motors
            stepper_az.run();
            stepper_el.run();

            // Idle rotator
            if (stepper_az.distanceToGo() == 0 && stepper_el.distanceToGo() == 0)
            {
                rotator.rotator_status = idle;
            }
        }
    }
    else
    {
        // Error handler, stop motors and disable the motor driver
        stepper_az.stop();
        stepper_az.disableOutputs();
        stepper_el.stop();
        stepper_el.disableOutputs();
        if (rotator.rotator_error != homing_error)
        {
            // Reset error according to error value
            rotator.rotator_error = no_error;
            rotator.rotator_status = idle;
        }
    }
}

/**************************************************************************/
/*!
    @brief    Move both axis with one direction in order to find home position,
              end-stop switches
    @param    seek_az
              Steps to find home position for azimuth axis
    @param    seek_el
              Steps to find home position for elevation axis
    @return   _rotator_error
*/
/**************************************************************************/
enum _rotator_error homing(int32_t seek_az, int32_t seek_el)
{
    bool isHome_az = false;
    bool isHome_el = false;

    // Move motors to "seek" position
    DEBUG("Moving motors to the \"seek\" position.\n");
    stepper_az.moveTo(seek_az);
    stepper_el.moveTo(seek_el);

    // Homing loop
    while (isHome_az == false || isHome_el == false)
    {
        // Update WDT
        wdt.watchdog_reset();
        if (switch_az.get_state() == true && !isHome_az)
        {
            // Find azimuth home
            stepper_az.moveTo(stepper_az.currentPosition());
            DEBUG("Switch AZ detected. Setting isHome_az = true\n");
            isHome_az = true;
        }
        if (switch_el.get_state() == true && !isHome_el)
        {
            // Find elevation home
            stepper_el.moveTo(stepper_el.currentPosition());
            DEBUG("Switch EL detected. Setting isHome_el = true\n");
            isHome_el = true;
        }
        // Check if the rotator goes out of limits or something goes wrong (in mechanical)
        if ((stepper_az.distanceToGo() == 0 && !isHome_az) ||
            (stepper_el.distanceToGo() == 0 && !isHome_el))
        {
            DEBUG("Homing error. Rotator out of limits: AZ=");
            DEBUG(isHome_az);
            DEBUG(" EL=");
            DEBUG(isHome_el);
            DEBUG("\n");
            return homing_error;
        }

        // Move motors to "seek" position
        stepper_az.run();
        stepper_el.run();
    }
    // Delay to Deccelerate and homing, to complete the movements
    uint32_t time = millis();
    while (millis() - time < HOME_DELAY)
    {
        // wdt.watchdog_reset();
        stepper_az.run();
        stepper_el.run();
    }
    
    // Set the home position and reset all critical control variables
    DEBUG("Setting home positions.\n");
    stepper_az.setCurrentPosition(0);
    stepper_el.setCurrentPosition(0);
    control_az.setpoint = 0;
    control_el.setpoint = 0;
    DEBUG("Homing complete.");
    return no_error;
}
