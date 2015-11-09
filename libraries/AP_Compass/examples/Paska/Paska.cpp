/*
 *       Example of APM_Compass library (HMC5843 sensor).
 *       Code by Jordi MuÒoz and Jose Julio. DIYDrones.com
 */

#include <AP_Common/AP_Common.h>
#include <AP_Progmem/AP_Progmem.h>
#include <AP_Param/AP_Param.h>
#include <StorageManager/StorageManager.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL_AVR/AP_HAL_AVR.h>
#include <AP_HAL_PX4/AP_HAL_PX4.h>
#include <AP_HAL_Linux/AP_HAL_Linux.h>
#include <AP_HAL_FLYMAPLE/AP_HAL_FLYMAPLE.h>
#include <AP_HAL_Empty/AP_HAL_Empty.h>
#include <AP_HAL_VRBRAIN/AP_HAL_VRBRAIN.h>

#include <AP_Math/AP_Math.h>    // ArduPilot Mega Vector/Matrix math Library
#include <AP_Declination/AP_Declination.h>
#include <AP_Compass/AP_Compass.h> // Compass Library
#include <AP_Scheduler/AP_Scheduler.h>
#include <DataFlash/DataFlash.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <Filter/Filter.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_AHRS/AP_AHRS.h>
#include "Console.h"

/*
const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

static Compass compass;

uint32_t timer;

void setup() {
    hal.console->println("Paska testiohjelma");

    if (!compass.init()) {
        hal.console->println("compass initialisation failed!");
        while (1) ;
    }
    hal.console->printf("init done - %u compasses detected\n", compass.get_count());

    compass.set_and_save_offsets(0,0,0,0); // set offsets to account for surrounding interference
    compass.set_declination(ToRad(0.0f)); // set local difference between magnetic north and true north

    hal.scheduler->delay(1000);
    timer = hal.scheduler->micros();
}

void loop()
{
    if((hal.scheduler->micros()- timer) > 300000L)
    {
      consoleNoteLn("voi vittu");
      
      timer = hal.scheduler->micros();
    } else {
      hal.scheduler->delay(1);
    }
}

*/
