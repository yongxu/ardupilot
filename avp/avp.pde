#include <math.h>
#include <stdarg.h>
#include <stdio.h>

// Libraries
#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_HAL.h>
#include <AP_Menu.h>
#include <AP_Param.h>
#include <StorageManager.h>
#include <AP_GPS.h>         // ArduPilot GPS library
#include <AP_ADC.h>         // ArduPilot Mega Analog to Digital Converter Library
#include <AP_ADC_AnalogSource.h>
#include <AP_Baro.h>
#include <AP_Compass.h>     // ArduPilot Mega Magnetometer Library
#include <AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <AP_InertialSensor.h> // Inertial Sensor (uncalibated IMU) Library
#include <AP_AHRS.h>         // ArduPilot Mega DCM Library
#include <AP_NavEKF.h>
#include <AP_Mission.h>     // Mission command library
#include <AP_Rally.h>
#include <AP_Terrain.h>
#include <PID.h>            // PID library
#include <RC_Channel.h>     // RC Channel Library
#include <AP_RangeFinder.h> // Range finder library
#include <Filter.h>         // Filter library
#include <Butter.h>         // Filter library - butterworth filter
#include <AP_Buffer.h>      // FIFO buffer library
#include <ModeFilter.h>     // Mode Filter from Filter library
#include <AverageFilter.h>  // Mode Filter from Filter library
#include <AP_Relay.h>       // APM relay
#include <AP_ServoRelayEvents.h>
#include <GCS_MAVLink.h>    // MAVLink GCS definitions
#include <AP_SerialManager.h>   // Serial manager library
#include <AP_Airspeed.h>    // needed for AHRS build
#include <AP_Vehicle.h>     // needed for AHRS build
#include <DataFlash.h>
#include <AP_RCMapper.h>        // RC input mapping library
#include <AP_Scheduler.h>       // main loop scheduler
#include <UARTDriver.h>
#include <stdarg.h>
#include <AP_Navigation.h>
#include <APM_Control.h>
#include <AP_L1_Control.h>
#include <AP_BoardConfig.h>

#include <AP_HAL_AVR.h>
#include <AP_HAL_AVR_SITL.h>
#include <AP_HAL_PX4.h>
#include <AP_HAL_VRBRAIN.h>
#include <AP_HAL_FLYMAPLE.h>
#include <AP_HAL_Linux.h>
#include <AP_HAL_Empty.h>


#include <AP_Notify.h>      // Notify library
#include <AP_BattMonitor.h> // Battery monitor library

#include <AP_Declination.h> // ArduPilot Mega Declination Helper Library
#define HIGH 1
#define LOW 0
const   float radius_of_earth   = 6378100;  // meters
const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

static struct   Location current_loc;

static float heading = 0;

// true if we have a position estimate from AHRS
static bool have_position;

static float    ground_speed = 0;
static float    groundspeed_error;
static float    lateral_acceleration;
////////////////////////////////////////////////////////////////////////////////
// System Timers
////////////////////////////////////////////////////////////////////////////////
// Time in microseconds of start of main control loop.
static uint32_t     fast_loopTimer_us;
// Number of milliseconds used in last main loop cycle
static uint32_t     delta_us_fast_loop;
// The main loop execution time.  Seconds
//This is the time between calls to the DCM algorithm and is the Integration time for the gyros.
static float G_Dt                       = 0.02;
// Counter of main loop executions.  Used for performance monitoring and failsafe processing
static uint16_t         mainLoop_count;



AP_InertialSensor ins;
Compass compass;
AP_GPS gps;
AP_Baro barometer;
AP_SerialManager serial_manager;
// use ekf
AP_AHRS_NavEKF  ahrs(ins, barometer, gps);

AP_HAL::UARTDriver *uart_tk1;
AP_HAL::UARTDriver *uart_arduino;


static void update_compass(void)
{
    if (compass.read()) {
        ahrs.set_compass(&compass);
        // update offsets
        compass.learn_offsets();
        heading = compass.calculate_heading(ahrs.get_dcm_matrix());

        //g_gps->update();
    } else {
        ahrs.set_compass(NULL);
    }
}

static void compass_accumulate(void)
{
    compass.accumulate();
}

// static void update_alt()
// {
//     barometer.update();
// }

static void one_second_loop(void)
{
    // allow orientation change at runtime to aid config
    ahrs.set_orientation();

    static uint8_t counter;

    counter++;

    // save compass offsets once a minute
    if (counter >= 60) {
        compass.save_offsets();
        counter = 0;
    }
}

static void one_hundred_ms_loop(void)
{
    Vector3f drift  = ahrs.get_gyro_drift();
    Vector3f acc    =ahrs.get_ins().get_accel();
    hal.console->printf_P(
            PSTR("r:%4.1f  p:%4.1f y:%4.1f "
                "drift=(%5.1f %5.1f %5.1f) hdg=%.1f\n"),
                    ToDeg(ahrs.roll),
                    ToDeg(ahrs.pitch),
                    ToDeg(ahrs.yaw),
                    ToDeg(drift.x),
                    ToDeg(drift.y),
                    ToDeg(drift.z),
                    compass.use_for_yaw() ? ToDeg(heading) : 0.0);
    hal.console->printf("accel: x:%4.1f  y:%4.1f z:%4.1f \n",acc.x,acc.y,acc.z);

    Vector3f mag=compass.get_field();
    hal.console->printf("compass: x:%4.1f  y:%4.1f z:%4.1f \n",mag.x,mag.y,mag.z);
    hal.console->printf("Mag Declination: %4.4f\n",degrees(compass.get_declination()));


    static uint32_t last_msg_ms;
    gps.update();
    if (last_msg_ms != gps.last_message_time_ms()) {
        last_msg_ms = gps.last_message_time_ms();
        const Location &loc = gps.location();
        hal.console->print("Lat: ");
        print_latlon(hal.console, loc.lat);
        hal.console->print(" Lon: ");
        print_latlon(hal.console, loc.lng);
        hal.console->printf(" Alt: %.2fm GSP: %.2fm/s CoG: %d SAT: %d TIM: %u/%lu STATUS: %u\n",
                            loc.alt * 0.01f,
                            gps.ground_speed(),
                            (int)gps.ground_course_cd() / 100,
                            gps.num_sats(),
                            gps.time_week(),
                            (unsigned long)gps.time_week_ms(),
                            gps.status());
    }

}

static void update_GPS_10Hz(void)
{
// A counter used to count down valid gps fixes to allow the gps estimate to settle
// before recording our home position (and executing a ground start if we booted with an air start)
    static uint8_t  ground_start_count  = 20;
    have_position = ahrs.get_position(current_loc);

    if (have_position && gps.status() >= AP_GPS::GPS_OK_FIX_3D) {

        if (ground_start_count > 1){
            ground_start_count--;

        } else if (ground_start_count == 1) {
            // We countdown N number of good GPS fixes
            // so that the altitude is more accurate
            // -------------------------------------
            if (current_loc.lat == 0) {
                ground_start_count = 20;
            } else {
                compass.set_initial_location(gps.location().lat, gps.location().lng);
                ground_start_count = 0;
            }
        }
        Vector3f velocity;
        if (ahrs.get_velocity_NED(velocity)) {
            ground_speed = pythagorous2(velocity.x, velocity.y);
        } else {
            ground_speed   = gps.ground_speed();
        }
    }
}

// update AHRS system
static void ahrs_update()
{
    ahrs.set_fly_forward(true);

    ahrs.update();

    Vector3f velocity;
    if (ahrs.get_velocity_NED(velocity)) {
        ground_speed = pythagorous2(velocity.x, velocity.y);
    }
}

static void cmd_updates(){
    uint16_t nbytes = uart_tk1->available();
    for (uint16_t i=0; i<nbytes; i++)
    {
        uint8_t c = uart_tk1->read();
        //parse c, one byte at a time
        hal.console->write(c);
    }


    //the test below passed
    /*uint16_t nbytes = hal.console->available();
    for (uint16_t i=0; i<nbytes; i++)
    {
        uint8_t c = hal.console->read();
        //parse c, one byte at a time
        hal.console->write(c);
    }*/
}

static void arduino_updates(){
    uint16_t nbytes = uart_arduino->available();
    for (uint16_t i=0; i<nbytes; i++)
    {
        uint8_t c = uart_arduino->read();
        //parse c, one byte at a time
        hal.console->write(c);
    }
}

static void uart_init()
{
    hal.uartC->begin(115200);
    hal.uartD->begin(57600);
    uart_tk1=hal.uartC;
    uart_arduino=hal.uartD;

    serial_manager.init_console();
    serial_manager.set_blocking_writes_all(false);
}

static AP_Scheduler scheduler;
/*
  scheduler table - all regular tasks are listed here, along with how
  often they should be called (in 20ms units) and the maximum time
  they are expected to take (in microseconds)
 */
static const AP_Scheduler::Task scheduler_tasks[] PROGMEM = {
    { ahrs_update,            1,   6400 },
    { update_compass,         5,   2000 },
    { compass_accumulate,     1,    900 },
    { update_GPS_10Hz,        5,   2500 },
    { one_hundred_ms_loop,     5,  1800 },
    { one_second_loop,        50,  1800 },
    { cmd_updates,            1,   1700 },
    { arduino_updates,        1,   1700 }
};

void setup(void)
{
    //gps.init(NULL, serial_manager);

    ahrs.init();

    ins.init(AP_InertialSensor::COLD_START,
             AP_InertialSensor::RATE_100HZ);

    uart_init();
    barometer.init();

    if( compass.init() ) {
        ahrs.set_compass(&compass);
    } else {
        //hal.console->printf("No compass detected\n");
    }

    ahrs.set_fly_forward(true);
    ahrs.set_vehicle_class(AHRS_VEHICLE_GROUND);

    ahrs.reset();


    scheduler.init(&scheduler_tasks[0], sizeof(scheduler_tasks)/sizeof(scheduler_tasks[0]));

}

void loop(void)
{
    ins.wait_for_sample();

    uint32_t now = hal.scheduler->micros();
    delta_us_fast_loop  = now - fast_loopTimer_us;
    G_Dt                = delta_us_fast_loop * 1.0e-6f;
    fast_loopTimer_us   = now;

    mainLoop_count++;

    // hal.uartC->printf("Hello on UART %s at %.3f seconds\n",
    //              "C", hal.scheduler->millis()*0.001f);

    // hal.uartD->printf("Hello on UART %s at %.3f seconds\n",
    //              "D", hal.scheduler->millis()*0.001f);
    // hal.console->printf("Hello on console at %.3f seconds\n",
    //              hal.scheduler->millis()*0.001f);

    scheduler.tick();
    scheduler.run(19500U);

}

AP_HAL_MAIN();
