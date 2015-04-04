#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

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


void message_head(const char *name,unsigned int &id_counter){
    hal.console->printf("{\n\"name\":\"%s\",\"id\":%d,\"stamp\":%d,\n",
        name,id_counter,hal.scheduler->micros());
    id_counter++;
}
void message_tail(){
    hal.console->printf("\n}#");
}

int parse_avp_cmd(char* cmd ){
    size_t pos=0;
    int argc=0;
    char *argv[20];
    size_t cmd_len=strlen(cmd);
    while (pos<cmd_len) {
        argv[argc]=strtok(cmd+pos, " ");
        pos+=strlen(argv[argc])+1;
        argc++;
    }

    if (!strcmp(argv[0], "arm")) {
        printf("arm!\n");
    }
    if (!strcmp(argv[0], "mov")) {
        printf("mov!\n");
    }

    return 0;
}

static uint16_t radio_last_value[8];
static void read_radio()
{
    bool changed=false;
    for (uint8_t i=0; i<8; i++) {
        uint16_t v = hal.rcin->read(i);
        if (radio_last_value[i] != v) {
            changed = true;
            radio_last_value[i] = v;
        }
    }
    if (changed) {
        for (uint8_t i=0; i<8; i++) {
 //           hal.console->printf("%2u:%04u ", (unsigned)i+1, (unsigned)radio_last_value[i]);
        }
 //       hal.console->println();
    }

}

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
    //gps.update();

    ahrs.set_fly_forward(true);

    ahrs.update();

    Vector3f velocity;
    if (ahrs.get_velocity_NED(velocity)) {
        ground_speed = pythagorous2(velocity.x, velocity.y);
    }
}

static void cmd_updates(){
    static char buf[256];
    static int buf_pos=0;
    uint16_t nbytes = hal.console->available();
    for (uint16_t i=0; i<nbytes; i++)
    {
        uint8_t c = hal.console->read();
        //parse c, one byte at a time
        buf[buf_pos++]=c;
        if(buf_pos==255){
            buf_pos=0;
            return;
        }
        if(c=='\n' || c=='\r'){
            buf[buf_pos]='\0';
            buf_pos=0;
            parse_avp_cmd(buf);
            hal.console->printf("%s\n", buf);
            return;
        }
        //hal.console->write(c);
    }
}

static void uart_init()
{
    serial_manager.init_console();
    hal.uartA->begin(115200,2048,32768);
    hal.uartA->set_blocking_writes(true);

    hal.console->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
}

static AP_Scheduler scheduler;

static const AP_Scheduler::Task scheduler_tasks[] PROGMEM = {
    { read_radio,             1,   1000 },
    { ahrs_update,            1,   9600 },
    { update_compass,         5,   1500 },
    { compass_accumulate,     1,    900 },
    { update_GPS_10Hz,        5,   2500 },
    { one_hundred_ms_loop,     5,  3000 },
//    { cmd_updates,            1,   4000 },
    { one_second_loop,        50,  1800 }
};

void setup(void)
{
    ahrs.init();

    ins.init(AP_InertialSensor::COLD_START,
             AP_InertialSensor::RATE_100HZ);

    uart_init();
    barometer.init();
    gps._type[0]=9;
    gps.init(NULL, serial_manager);

    //gps._type[1]=9;


    if( compass.init() ) {
        ahrs.set_compass(&compass);
    } else {
        //hal.console->printf("No compass detected\n");
    }

    ahrs.set_fly_forward(true);
    ahrs.set_vehicle_class(AHRS_VEHICLE_GROUND);
    ahrs.set_ekf_use(true);
    ahrs.reset();

    for (uint8_t i=0; i<14; i++) {
        hal.rcout->enable_ch(i);
        hal.rcout->write(i, 1500);
    }

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

    Vector3f acc    =ahrs.get_accel_ef_blended();
    Vector3f vel,pos;
    ahrs.get_velocity_NED(vel);
    ahrs.get_relative_position_NED(pos);

    static unsigned int ahrs_id=0;
    message_head("ahrs",ahrs_id);
    hal.console->printf("\"gyro\":{\"r\":%f,\"p\":%f,\"y\":%f,\"yv\":%f,\"hdg\":%f},\n",
                    ahrs.roll,
                    ahrs.pitch,
                    ahrs.yaw,
                    ahrs.get_yaw_rate_earth(),
                    heading);


//    ahrs.get_position();
    hal.console->printf("\"ekf\":\"%s\",\n",ahrs.have_inertial_nav()?"true":"false");
    hal.console->printf("\"vel\":{\"x\":%4.4f, \"y\":%4.4f,\"z\":%4.4f},\n",vel.x,vel.y,vel.z);
    hal.console->printf("\"pos\":{\"x\":%4.4f, \"y\":%4.4f,\"z\":%4.4f},\n",pos.x,pos.y,pos.z);
    hal.console->printf("\"acc\":{\"x\":%4.4f, \"y\":%4.4f,\"z\":%4.4f}",acc.x,acc.y,acc.z);


    // static uint32_t last_msg_ms;
    // if (last_msg_ms != gps.last_message_time_ms()) {
    //     last_msg_ms = gps.last_message_time_ms();
        gps.update();
        const Location &loc = gps.location();
        hal.console->print(",\n\"gps\":{\"lat\": ");
        print_latlon(hal.console, loc.lat);
        hal.console->print(" ,\"lon\": ");
        print_latlon(hal.console, loc.lng);
        hal.console->printf(",\"alt\": %.2f, \"gsp\": %.2f,\"SAT\": %d, \"STATUS\": %u}",
                            loc.alt * 0.01f,
                            ground_speed,
                            gps.num_sats(),
                            gps.status());
//   }

   message_tail();

    scheduler.tick();
    scheduler.run(20000U);

}

AP_HAL_MAIN();
