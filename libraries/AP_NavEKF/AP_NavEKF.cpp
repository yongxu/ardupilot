/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL.h>

#if HAL_CPU_CLASS >= HAL_CPU_CLASS_150

// uncomment this to force the optimisation of this code, note that
// this makes debugging harder
#if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL || CONFIG_HAL_BOARD == HAL_BOARD_LINUX
#pragma GCC optimize("O0")
#else
#pragma GCC optimize("O3")
#endif

#include "AP_NavEKF.h"
#include <AP_AHRS.h>

#include <stdio.h>

extern const AP_HAL::HAL& hal;

#define earthRate 0.000072921f // earth rotation rate (rad/sec)

// constructor
NavEKF::NavEKF(const AP_AHRS *ahrs, AP_Baro &baro) :
    _ahrs(ahrs),
    _baro(baro),
    useAirspeed(true),          // activates fusion of compass data
    useCompass(true),           // activates fusion of airspeed data
    covTimeStepMax(0.07f),      // maximum time (sec) between covariance prediction updates
    covDelAngMax(0.05f),        // maximum delta angle between covariance prediction updates
    TASmsecMax(333),            // maximum allowed interval between airspeed measurement updates
    fuseMeNow(false),            // forces fusion to occur on the IMU frame that data arrives
    staticMode(true)            // staticMode forces position and velocity fusion with zero values

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    ,_perf_UpdateFilter(perf_alloc(PC_ELAPSED, "EKF_UpdateFilter")),
    _perf_CovariancePrediction(perf_alloc(PC_ELAPSED, "EKF_CovariancePrediction")),
    _perf_FuseVelPosNED(perf_alloc(PC_ELAPSED, "EKF_FuseVelPosNED")),
    _perf_FuseMagnetometer(perf_alloc(PC_ELAPSED, "EKF_FuseMagnetometer")),
    _perf_FuseAirspeed(perf_alloc(PC_ELAPSED, "EKF_FuseAirspeed"))
#endif
{
    // Tuning parameters
    _gpsHorizVelNoise       = 0.15f;    // GPS horizontal velocity measurement noise : m/s
    _gpsVertVelNoise        = 0.15f;    // GPS vertical velocity measurement noise : m/s
    _gpsHorizPosNoise       = 0.5f;     // GPS horizontal position measurement noise : m
    _baroAltNoise           = 0.5f;     // Baro height measurement noise : m
    _gpsNEVelVarAccScale    = 0.1f;     // scale factor applied to NE velocity measurement variance due to Vdot
    _gpsDVelVarAccScale     = 0.15f;    // scale factor applied to D velocity measurement variance due to Vdot
    _gpsPosVarAccScale      = 0.1f;     // scale factor applied to position measurement variance due to Vdot
    _magNoise               = 0.05f;    // magnetometer measurement noise : gauss
    _magVarRateScale        = 0.05f;    // scale factor applied to magnetometer variance due to angular rate
    _easNoise               = 1.4f;     // equivalent airspeed measurement noise : m/s
    _windVelProcessNoise    = 0.1f;     // wind velocity states process noise : (m/s^2)
    _wndVarHgtRateScale     = 0.5f;     // scale factor applied to wind states process noise from height rate
    _gyrNoise               = 1.5e-2f;  // gyro process noise : rad/s
    _accNoise               = 0.5f;     // accelerometer process noise : m/s^2
    _gyroBiasNoiseScaler    = 3.0f;     // scale factor applied to gyro bias state process variance when on ground
    _gyroBiasProcessNoise   = 2.0e-6f;  // gyro bias states process noise : rad/s
    _accelBiasProcessNoise	= 1.0e-3f;	// accel bias state process noise : m/s^2
    _magEarthProcessNoise   = 3.0e-4f;  // earth magnetic field states process noise : gauss/sec
    _magBodyProcessNoise    = 3.0e-4f;  // body magnetic field states process noise : gauss/sec
    // When a new measurement is available, the filter states used to process it is taken from a delay
    // buffer. These parameters control how far back in the buffer the states are taken for each measurement
    _msecVelDelay           = 220;      // effective average delay of GPS velocity measurements rel to IMU (msec)
    _msecPosDelay           = 220;      // effective average delay of GPS position measurements rel to (msec)
    _msecHgtDelay           = 60;       // effective average delay of height measurements rel to (msec)
    _msecMagDelay           = 40;       // effective average delay of magnetometer measurements rel to IMU (msec)
    _msecTasDelay           = 240;      // effective average delay of airspeed measurements rel to IMU (msec)
    // These parameters control the way that a single GPS or baro measurement is fused repeatedly to provide
    // a 'smoother' trajectory output
    _msecGpsAvg             = 200;      // average number of msec between GPS measurements
    _msecHgtAvg             = 100;      // average number of msec between height measurements
    _fusionModeGPS          = 1;        // 0 = use 3D velocity, 1 = use 2D velocity, 2 = use no velocity
    // These parameters control the size of the gate that is applied to reject unreasonable measurements
    // The innovation variancea (contained in the EKF4 log message) are multiplied by these scale factors to determine the +-
    // gate thresholds. If the innovation for a measurement (contained in the EKF3 log message) is outside the gate, that measurement
    // will not be used. Set too tight and good data will be rjected and there is a risk that the system could go free inertial for periods
    // Set too loose and bad data that could corrupt the EKF solution could be accepted.
    _gpsVelInnovGate        = 5;        // Number of standard deviations applied to GPS velocity innovation consistency check
    _gpsPosInnovGate        = 10;        // Number of standard deviations applied to GPS position innovation consistency check
    _hgtInnovGate           = 10;        // Number of standard deviations applied to height innovation consistency check
    _magInnovGate           = 5;        // Number of standard deviations applied to magnetometer innovation consistency check
    _tasInnovGate           = 10;        // Number of standard deviations applied to true airspeed innovation consistency check
    // these parameters control how long GPS and height measurements can be locked out if they fail the innovation consistency check
    _gpsRetryTimeUseTAS     = 10000;    // GPS retry time following innovation consistency fail if TAS measurements are used (msec)
    _gpsRetryTimeNoTAS      = 5000;     // GPS retry time following innovation consistency fail if no TAS measurements are used (msec)
    _hgtRetryTimeMode0      = 10000;    // height measurement retry time following innovation consistency fail if GPS fusion mode is = 0 (msec)
    _hgtRetryTimeMode12     = 5000;     // height measurement retry time following innovation consistency fail if GPS fusion mode is > 0 (msec)

    // Misc initial conditions
    hgtRate = 0.0f;
    mag_state.q0 = 1;
    mag_state.DCM.identity();
}

bool NavEKF::healthy(void) const
{
    if (!statesInitialised) {
        return false;
    }
    Quaternion q(states[0],states[1],states[2],states[3]);
    if (q.is_nan()) {
        return false;
    }
    if (isnan(states[4]) || isnan(states[5]) || isnan(states[6])) {
        return false;
    }
    // If measurements have failed innovation consistency checks for long enough to time-out
    // and force fusion then the nav solution can be conidered to be unhealthy
    // This will only be set as a transient
    if (posTimeout || velTimeout || hgtTimeout) {
        return false;
    }

    // all OK
    return true;
}

bool NavEKF::HeightDrifting(void) const
{
    // Set to true if height measurements are failing the innovation consistency check
    return !hgtHealth;
}

bool NavEKF::PositionDrifting(void) const
{
    // Set to true if position measurements are failing the innovation consistency check
    return !posHealth;
}

void NavEKF::SetStaticMode(bool setting) {
        staticMode = setting;
}

void NavEKF::ResetPosition(void)
{
    // read the GPS
    readGpsData();
    readHgtData();
    // write to state vector
    states[7] = posNE[0];
    states[8] = posNE[1];
    states[9] = -hgtMea;
}

void NavEKF::InitialiseFilter(void)
{
    // Set re-used variables to zero
    ZeroVariables();

    // get initial time deltat between IMU measurements (sec)
    dtIMU = _ahrs->get_ins().get_delta_time();

    // Calculate initial filter quaternion states from AHRS solution
    Quaternion initQuat;
    initQuat.from_euler(_ahrs->roll, _ahrs->pitch, _ahrs->yaw);

    // Calculate initial Tbn matrix and rotate Mag measurements into NED
    // to set initial NED magnetic field states
    Matrix3f initTbn;
    initQuat.rotation_matrix(initTbn);
    Vector3f initMagNED;
    Vector3f initMagXYZ;
    if (useCompass)
    {
        readMagData();
        initMagXYZ = magData - magBias;
        initMagNED = initTbn * initMagXYZ;
    }

    // read the GPS height
    readGpsData();

    // read the barometer height
    readHgtData();

    // set onground flag
    OnGroundCheck();

    // write to state vector
    for (uint8_t j=0; j<=3; j++) states[j] = initQuat[j]; // quaternions
    states[4] = velNED[0];
    states[5] = velNED[1];
    states[6] = velNED[2];
    states[7] = posNE[0];
    states[8] = posNE[1];
    states[9] = - _baro.get_altitude();
    for (uint8_t j=10; j<=15; j++) states[j] = 0.0; // dAngBias, dVelBias, windVel
    states[16] = initMagNED.x; // Magnetic Field North
    states[17] = initMagNED.y; // Magnetic Field East
    states[18] = initMagNED.z; // Magnetic Field Down
    for (uint8_t j=19; j<=21; j++) states[j] = magBias[j-19]; // Magnetic Field Bias XYZ

    statesInitialised = true;

    // initialise the covariance matrix
    CovarianceInit(_ahrs->roll, _ahrs->pitch, _ahrs->yaw);

    //Define Earth rotation vector in the NED navigation frame
    calcEarthRateNED(earthRateNED, _ahrs->get_home().lat);

    //Initialise summed variables used by covariance prediction
    summedDelAng.x = 0.0;
    summedDelAng.y = 0.0;
    summedDelAng.z = 0.0;
    summedDelVel.x = 0.0;
    summedDelVel.y = 0.0;
    summedDelVel.z = 0.0;
    dt = 0.0;

    //Initialise IMU pre-processing states
    readIMUData();
}

void NavEKF::InitialiseFilterBootstrap(void)
{
    // Set re-used variables to zero
    ZeroVariables();

    // acceleration vector in XYZ body axes measured by the IMU (m/s^2)
    Vector3f initAccVec;

    // body magnetic field vector with offsets removed
    Vector3f initMagXYZ;

    // Take 50 readings at 20msec intervals and average
    initAccVec.zero();
    initMagXYZ.zero();
    for (uint8_t i=1; i<=50; i++) {
        initAccVec = initAccVec + _ahrs->get_ins().get_accel();
        initMagXYZ = initMagXYZ + _ahrs->get_compass()->get_field() * 0.001f; // convert from Gauss to mGauss
        hal.scheduler->delay(20);
    }
    initMagXYZ = initMagXYZ * 0.02f;

    // Normalise the acceleration vector
    initAccVec.normalize();

    // Calculate initial pitch angle
    float pitch = asinf(initAccVec.x);

    // calculate initial roll angle
    float roll = -asinf(initAccVec.y / cosf(pitch));

    // calculate initial yaw angle
    float yaw;
    Matrix3f Tbn;
    Vector3f initMagVecNED;
    if (useCompass) {
        // calculate rotation matrix from body to NED frame
        Tbn.from_euler(roll, pitch, 0.0f);

        // rotate the magnetic field into NED axesn
        initMagVecNED = Tbn*initMagXYZ;

        // calculate heading of mag field rel to body heading
        float magHeading = atan2f(initMagVecNED.y, initMagVecNED.x);

        // get the magnetic declination
        float magDecAng = _ahrs->get_compass()->get_declination();

        // calculate yaw angle rel to true north
        yaw = magDecAng - magHeading;
    } else {
        yaw = 0.0f;
    }

    // Calculate initial filter quaternion states
    Quaternion initQuat;
    initQuat.from_euler(roll, pitch, yaw);

    // Calculate initial Tbn matrix and rotate Mag measurements into NED
    // to set initial NED magnetic field states
    initQuat.rotation_matrix(Tbn);
    initMagVecNED = Tbn * initMagXYZ;

    //Get the initial compass bias estimates
    Vector3f initMagBias = -_ahrs->get_compass()->get_offsets() * 0.001f;

    // read the GPS
    readGpsData();

    // read the barometer
    readHgtData();

    // set onground flag
    OnGroundCheck();

    // write to state vector
    for (uint8_t j=0; j<=3; j++) states[j] = initQuat[j]; // quaternions
    states[4] = velNED[0];
    states[5] = velNED[1];
    states[6] = velNED[2];
    states[7] = posNE[0];
    states[8] = posNE[1];
    states[9] = - _baro.get_altitude();
    for (uint8_t j=10; j<=15; j++) states[j] = 0.0f; // dAngBias, dVelBias, windVel
    for (uint8_t j=16; j<=18; j++) states[j] = initMagVecNED[j-16]; // Magnetic Field NED
    for (uint8_t j=19; j<=21; j++) states[j] = initMagBias[j-19]; // Magnetic Field Bias XYZ

    statesInitialised = true;

    // initialise the covariance matrix
    CovarianceInit(roll, pitch, yaw);

    //Define Earth rotation vector in the NED navigation frame
    calcEarthRateNED(earthRateNED, _ahrs->get_home().lat);

    //Initialise IMU pre-processing states
    readIMUData();
}

void NavEKF::UpdateFilter()
{
    if (!statesInitialised) {
        return;
    }

    perf_begin(_perf_UpdateFilter);

    // This function will be called at 100Hz
    //
    // Read IMU data and convert to delta angles and velocities
    readIMUData();

    if (dtIMU > 0.2f) {
        // we have stalled for far too long - reset from DCM
        InitialiseFilter();
        perf_end(_perf_UpdateFilter);
        return;
    }

    // Check if on ground
    OnGroundCheck();

    // staticmode is always set to false in the air
    // If we are not using compass, then staticMode is set to true whenever on ground
    // When exiting static mode with no compass, do a forced yaw alignment
    if (!onGround) { // we are flying
        if (!useCompass && staticMode) { // we are exiting static mode
            // align yaw angle with GPS velocity and reset quaternion covariances
            ForceYawAlignment();
            //TODO protection against inflight heading drift if no compass is fitted.
        }
    staticMode = false;
    } else if (!useCompass) { // we are on ground without a compass
        staticMode = true;
    }

    // Run the strapdown INS equations every IMU update
    UpdateStrapdownEquationsNED();

    // store the predicted states for subsequent use by measurement fusion
    StoreStates();

    // sum delta angles and time used by covariance prediction
    summedDelAng = summedDelAng + correctedDelAng;
    summedDelVel = summedDelVel + correctedDelVel;
    dt += dtIMU;

    // perform a covariance prediction if the total delta angle has exceeded the limit
    // or the time limit will be exceeded at the next IMU update
    // Do not predict covariance if magnetometer fusion still needs to be performed
    if (((dt >= (covTimeStepMax - dtIMU)) || (summedDelAng.length() > covDelAngMax))) {
        CovariancePrediction();
        covPredStep = true;
        summedDelAng.zero();
        summedDelVel.zero();
        dt = 0.0;
    } else {
        covPredStep = false;
    }

    // Update states using GPS, altimeter, compass and airspeed observations
    SelectVelPosFusion();
    SelectMagFusion();
    SelectTasFusion();

    perf_end(_perf_UpdateFilter);
}

void NavEKF::SelectVelPosFusion()
{
    // Command fusion of GPS measurements if new ones available
    readGpsData();
    if (newDataGps) {
        fuseVelData = true;
        fusePosData = true;
        // Calculate the scale factor to be applied to the measurement variance to account for
        // the fact we repeat fusion of the same measurement to provide a smoother output
        gpsVarScaler = _msecGpsAvg/(1000.0f*dtIMU);
    }
    // Timeout fusion when data is stale. Needed because we continue to fuse the same
    // measurement until the next one arrives
    if (hal.scheduler->millis() > lastFixTime_ms + _msecGpsAvg + 40) {
        fuseVelData = false;
        fusePosData = false;
    }
    // Command fusion of height measurements if new ones available
    readHgtData();
    if (newDataHgt)
    {
        fuseHgtData = true;
        // Calculate the scale factor to be applied to the measurement variance to account for
        // the fact we repeat fusion of the same measurement to provide a smoother output
        hgtVarScaler = _msecHgtAvg/(1000.0f*dtIMU);
    }
    // Timeout fusion when data is stale. Needed because we continue to fuse the same
    // measurement until the next one arrives
    if (hal.scheduler->millis() > lastHgtUpdate + _msecHgtAvg + 40) {
        fuseHgtData = false;
    }
    //Perform fusion of measurement available
    // If static mode, skip fusion if average rate of change of velocity since last fusion > 10 m/s^2
    // This prevents acceleraton transients from corrupting the attitude during ground handling
    // and rapid launches without a magnetometer
    imuStepsVelFuse += 1;
    accelSumVelFuse = accelSumVelFuse + velDotNED;
    if (fuseVelData || fusePosData || fuseHgtData)
    {
        float avgAccMag = accelSumVelFuse.length() / imuStepsVelFuse;
        if (!staticMode || avgAccMag < 10.0f) {
            FuseVelPosNED();
        }
        imuStepsVelFuse = 0;
        accelSumVelFuse.zero();
    }
    newDataGps = false;
    newDataHgt = false;
}

void NavEKF::SelectMagFusion()
{
    readMagData();
    // Fuse Magnetometer Measurements
    bool dataReady = statesInitialised && useCompass && newDataMag;
    if (dataReady)
    {
        MAGmsecPrev = IMUmsec;
        fuseMagData = true;
    }
    else
    {
        fuseMagData = false;
    }
    // Magnetometer fusion is always called if enabled because its fusion is spread across 3 time steps to reduce peak load
    FuseMagnetometer();

}

void NavEKF::SelectTasFusion()
{
    readAirSpdData();
    // Fuse Airspeed Measurements - hold off if magnetometer fusion has been performed, unless maximum time interval exceeded
    bool dataReady = (statesInitialised && useAirspeed && !onGround  && newDataTas);
    bool timeout = ((IMUmsec - TASmsecPrev) >= TASmsecMax);
    if (dataReady && (!magFusePerformed || timeout || fuseMeNow))
    {
        TASmsecPrev = IMUmsec;
        FuseAirspeed();
    }
}

void NavEKF::UpdateStrapdownEquationsNED()
{
    Vector3f delVelNav;
    float rotationMag;
    float rotScaler;
    Quaternion qUpdated;
    float quatMag;
    float quatMagInv;
    Quaternion deltaQuat;
    const Vector3f gravityNED(0, 0, GRAVITY_MSS);

    // Remove sensor bias errors
    correctedDelAng.x = dAngIMU.x - states[10];
    correctedDelAng.y = dAngIMU.y - states[11];
    correctedDelAng.z = dAngIMU.z - states[12];
    correctedDelVel.x = dVelIMU.x;
    correctedDelVel.y = dVelIMU.y;
    correctedDelVel.z = dVelIMU.z - states[13];

    // Save current measurements
    prevDelAng = correctedDelAng;

    // Apply corrections for earths rotation rate and coning errors
    // % * - and + operators have been overloaded
    correctedDelAng   = correctedDelAng - prevTnb * earthRateNED*dtIMU + (prevDelAng % correctedDelAng) * 8.333333e-2f;

    // Convert the rotation vector to its equivalent quaternion
    rotationMag = correctedDelAng.length();
    if (rotationMag < 1e-12f)
    {
        deltaQuat[0] = 1;
        deltaQuat[1] = 0;
        deltaQuat[2] = 0;
        deltaQuat[3] = 0;
    }
    else
    {
        deltaQuat[0] = cosf(0.5f * rotationMag);
        rotScaler = (sinf(0.5f * rotationMag)) / rotationMag;
        deltaQuat[1] = correctedDelAng.x * rotScaler;
        deltaQuat[2] = correctedDelAng.y * rotScaler;
        deltaQuat[3] = correctedDelAng.z * rotScaler;
    }

    // Update the quaternions by rotating from the previous attitude through
    // the delta angle rotation quaternion
    qUpdated[0] = states[0]*deltaQuat[0] - states[1]*deltaQuat[1] - states[2]*deltaQuat[2] - states[3]*deltaQuat[3];
    qUpdated[1] = states[0]*deltaQuat[1] + states[1]*deltaQuat[0] + states[2]*deltaQuat[3] - states[3]*deltaQuat[2];
    qUpdated[2] = states[0]*deltaQuat[2] + states[2]*deltaQuat[0] + states[3]*deltaQuat[1] - states[1]*deltaQuat[3];
    qUpdated[3] = states[0]*deltaQuat[3] + states[3]*deltaQuat[0] + states[1]*deltaQuat[2] - states[2]*deltaQuat[1];

    // Normalise the quaternions and update the quaternion states
    quatMag = sqrtf(sq(qUpdated[0]) + sq(qUpdated[1]) + sq(qUpdated[2]) + sq(qUpdated[3]));
    if (quatMag > 1e-16f)
    {
        quatMagInv = 1.0f/quatMag;
        states[0] = quatMagInv*qUpdated[0];
        states[1] = quatMagInv*qUpdated[1];
        states[2] = quatMagInv*qUpdated[2];
        states[3] = quatMagInv*qUpdated[3];
    }

    // Calculate the body to nav cosine matrix
    Quaternion q(states[0],states[1],states[2],states[3]);
    Matrix3f Tbn_temp;
    q.rotation_matrix(Tbn_temp);
    prevTnb = Tbn_temp.transposed();

    // transform body delta velocities to delta velocities in the nav frame
    // * and + operators have been overloaded
    delVelNav = Tbn_temp*correctedDelVel + gravityNED*dtIMU;

    // Calculate the rate of change of velocity (used for launch detect and other functions)
    velDotNED = delVelNav / dtIMU ;

    // Calculate a filtered
    velDotNEDfilt = velDotNED * 0.05f + lastVelDotNED * 0.95f;

    // calculate a magnitude of the filtered nav acceleration (required for GPS
    // variance estimation)
    accNavMag = velDotNED.length();

    // If calculating position save previous velocity
    Vector3f lastVelocity;
    lastVelocity.x = states[4];
    lastVelocity.y = states[5];
    lastVelocity.z = states[6];

    // Sum delta velocities to get velocity
    states[4] = states[4] + delVelNav.x;
    states[5] = states[5] + delVelNav.y;
    states[6] = states[6] + delVelNav.z;

    // If calculating postions, do a trapezoidal integration for position
    states[7] = states[7] + 0.5f*(states[4] + lastVelocity[0])*dtIMU;
    states[8] = states[8] + 0.5f*(states[5] + lastVelocity[1])*dtIMU;
    states[9] = states[9] + 0.5f*(states[6] + lastVelocity[2])*dtIMU;

    // Limit states to protect against divergence
    ConstrainStates();
}

void NavEKF::CovariancePrediction()
{
    perf_begin(_perf_CovariancePrediction);
    // scalars
    float windVelSigma;
    float dAngBiasSigma;
    float dVelBiasSigma;
    float magEarthSigma;
    float magBodySigma;
    float daxCov;
    float dayCov;
    float dazCov;
    float dvxCov;
    float dvyCov;
    float dvzCov;
    float dvx;
    float dvy;
    float dvz;
    float dax;
    float day;
    float daz;
    float q0;
    float q1;
    float q2;
    float q3;
    float dax_b;
    float day_b;
    float daz_b;
    float dvz_b;

    // calculate covariance prediction process noise
    // use filtered height rate to increase wind process noise when climbing or descending
    // this allows for wind gradient effects.
    // filter height rate using a 10 second time constant filter
    float alpha = 0.1f * dt;
    hgtRate = hgtRate * (1.0f - alpha) - states[6] * alpha;

    // use filtered height rate to increase wind process noise when climbing or descending
    // this allows for wind gradient effects.
    windVelSigma  = dt * _windVelProcessNoise * (1.0f + _wndVarHgtRateScale * fabsf(hgtRate));
    dAngBiasSigma = dt * _gyroBiasProcessNoise;
    dVelBiasSigma = dt * _accelBiasProcessNoise;
    magEarthSigma = dt * _magEarthProcessNoise;
    magBodySigma  = dt * _magBodyProcessNoise;
    for (uint8_t i= 0; i<=9;  i++) processNoise[i] = 1.0e-9f;
    for (uint8_t i=10; i<=12; i++) processNoise[i] = dAngBiasSigma;
    // scale gyro bias noise when on ground to allow for faster bias estimation
    for (uint8_t i=10; i<=12; i++) {
        processNoise[i] = dAngBiasSigma;
        if (onGround) {
            processNoise[i] *= _gyroBiasNoiseScaler;
        }
    }
	processNoise[13] = dVelBiasSigma;
    for (uint8_t i=14; i<=15; i++) processNoise[i] = windVelSigma;
    for (uint8_t i=16; i<=18; i++) processNoise[i] = magEarthSigma;
    for (uint8_t i=19; i<=21; i++) processNoise[i] = magBodySigma;
    for (uint8_t i= 0; i<=21; i++) processNoise[i] = sq(processNoise[i]);

    // set variables used to calculate covariance growth
    dvx = summedDelVel.x;
    dvy = summedDelVel.y;
    dvz = summedDelVel.z;
    dax = summedDelAng.x;
    day = summedDelAng.y;
    daz = summedDelAng.z;
    q0 = states[0];
    q1 = states[1];
    q2 = states[2];
    q3 = states[3];
    dax_b = states[10];
    day_b = states[11];
    daz_b = states[12];
    dvz_b = states[13];
    daxCov = sq(dt*_gyrNoise);
    dayCov = sq(dt*_gyrNoise);
    dazCov = sq(dt*_gyrNoise);
    dvxCov = sq(dt*_accNoise);
    dvyCov = sq(dt*_accNoise);
    dvzCov = sq(dt*_accNoise);

    // Predicted covariance calculation
	SF[0] = dvz - dvz_b;
	SF[1] = 2*q3*SF[0] + 2*dvx*q1 + 2*dvy*q2;
	SF[2] = 2*dvx*q3 - 2*q1*SF[0] + 2*dvy*q0;
	SF[3] = 2*q2*SF[0] + 2*dvx*q0 - 2*dvy*q3;
	SF[4] = day/2 - day_b/2;
	SF[5] = daz/2 - daz_b/2;
	SF[6] = dax/2 - dax_b/2;
	SF[7] = dax_b/2 - dax/2;
	SF[8] = daz_b/2 - daz/2;
	SF[9] = day_b/2 - day/2;
	SF[10] = 2*q0*SF[0];
	SF[11] = q1/2;
	SF[12] = q2/2;
	SF[13] = q3/2;
	SF[14] = 2*dvy*q1;

	SG[0] = q0/2;
	SG[1] = sq(q3);
	SG[2] = sq(q2);
	SG[3] = sq(q1);
	SG[4] = sq(q0);
	SG[5] = 2*q2*q3;
	SG[6] = 2*q1*q3;
	SG[7] = 2*q1*q2;

	SQ[0] = dvzCov*(SG[5] - 2*q0*q1)*(SG[1] - SG[2] - SG[3] + SG[4]) - dvyCov*(SG[5] + 2*q0*q1)*(SG[1] - SG[2] + SG[3] - SG[4]) + dvxCov*(SG[6] - 2*q0*q2)*(SG[7] + 2*q0*q3);
	SQ[1] = dvzCov*(SG[6] + 2*q0*q2)*(SG[1] - SG[2] - SG[3] + SG[4]) - dvxCov*(SG[6] - 2*q0*q2)*(SG[1] + SG[2] - SG[3] - SG[4]) + dvyCov*(SG[5] + 2*q0*q1)*(SG[7] - 2*q0*q3);
	SQ[2] = dvzCov*(SG[5] - 2*q0*q1)*(SG[6] + 2*q0*q2) - dvyCov*(SG[7] - 2*q0*q3)*(SG[1] - SG[2] + SG[3] - SG[4]) - dvxCov*(SG[7] + 2*q0*q3)*(SG[1] + SG[2] - SG[3] - SG[4]);
	SQ[3] = (dayCov*q1*SG[0])/2 - (dazCov*q1*SG[0])/2 - (daxCov*q2*q3)/4;
	SQ[4] = (dazCov*q2*SG[0])/2 - (daxCov*q2*SG[0])/2 - (dayCov*q1*q3)/4;
	SQ[5] = (daxCov*q3*SG[0])/2 - (dayCov*q3*SG[0])/2 - (dazCov*q1*q2)/4;
	SQ[6] = (daxCov*q1*q2)/4 - (dazCov*q3*SG[0])/2 - (dayCov*q1*q2)/4;
	SQ[7] = (dazCov*q1*q3)/4 - (daxCov*q1*q3)/4 - (dayCov*q2*SG[0])/2;
	SQ[8] = (dayCov*q2*q3)/4 - (daxCov*q1*SG[0])/2 - (dazCov*q2*q3)/4;
	SQ[9] = sq(SG[0]);
	SQ[10] = sq(q1);

	SPP[0] = SF[10] + SF[14] - 2*dvx*q2;
	SPP[1] = 2*q2*SF[0] + 2*dvx*q0 - 2*dvy*q3;
	SPP[2] = 2*dvx*q3 - 2*q1*SF[0] + 2*dvy*q0;
	SPP[3] = 2*q0*q1 - 2*q2*q3;
	SPP[4] = 2*q0*q2 + 2*q1*q3;
	SPP[5] = sq(q0) - sq(q1) - sq(q2) + sq(q3);
	SPP[6] = SF[13];
	SPP[7] = SF[12];

	nextP[0][0] = P[0][0] + P[1][0]*SF[7] + P[2][0]*SF[9] + P[3][0]*SF[8] + P[10][0]*SF[11] + P[11][0]*SPP[7] + P[12][0]*SPP[6] + (daxCov*SQ[10])/4 + SF[7]*(P[0][1] + P[1][1]*SF[7] + P[2][1]*SF[9] + P[3][1]*SF[8] + P[10][1]*SF[11] + P[11][1]*SPP[7] + P[12][1]*SPP[6]) + SF[9]*(P[0][2] + P[1][2]*SF[7] + P[2][2]*SF[9] + P[3][2]*SF[8] + P[10][2]*SF[11] + P[11][2]*SPP[7] + P[12][2]*SPP[6]) + SF[8]*(P[0][3] + P[1][3]*SF[7] + P[2][3]*SF[9] + P[3][3]*SF[8] + P[10][3]*SF[11] + P[11][3]*SPP[7] + P[12][3]*SPP[6]) + SF[11]*(P[0][10] + P[1][10]*SF[7] + P[2][10]*SF[9] + P[3][10]*SF[8] + P[10][10]*SF[11] + P[11][10]*SPP[7] + P[12][10]*SPP[6]) + SPP[7]*(P[0][11] + P[1][11]*SF[7] + P[2][11]*SF[9] + P[3][11]*SF[8] + P[10][11]*SF[11] + P[11][11]*SPP[7] + P[12][11]*SPP[6]) + SPP[6]*(P[0][12] + P[1][12]*SF[7] + P[2][12]*SF[9] + P[3][12]*SF[8] + P[10][12]*SF[11] + P[11][12]*SPP[7] + P[12][12]*SPP[6]) + (dayCov*sq(q2))/4 + (dazCov*sq(q3))/4;
	nextP[0][1] = P[0][1] + SQ[8] + P[1][1]*SF[7] + P[2][1]*SF[9] + P[3][1]*SF[8] + P[10][1]*SF[11] + P[11][1]*SPP[7] + P[12][1]*SPP[6] + SF[6]*(P[0][0] + P[1][0]*SF[7] + P[2][0]*SF[9] + P[3][0]*SF[8] + P[10][0]*SF[11] + P[11][0]*SPP[7] + P[12][0]*SPP[6]) + SF[5]*(P[0][2] + P[1][2]*SF[7] + P[2][2]*SF[9] + P[3][2]*SF[8] + P[10][2]*SF[11] + P[11][2]*SPP[7] + P[12][2]*SPP[6]) + SF[9]*(P[0][3] + P[1][3]*SF[7] + P[2][3]*SF[9] + P[3][3]*SF[8] + P[10][3]*SF[11] + P[11][3]*SPP[7] + P[12][3]*SPP[6]) + SPP[6]*(P[0][11] + P[1][11]*SF[7] + P[2][11]*SF[9] + P[3][11]*SF[8] + P[10][11]*SF[11] + P[11][11]*SPP[7] + P[12][11]*SPP[6]) - SPP[7]*(P[0][12] + P[1][12]*SF[7] + P[2][12]*SF[9] + P[3][12]*SF[8] + P[10][12]*SF[11] + P[11][12]*SPP[7] + P[12][12]*SPP[6]) - (q0*(P[0][10] + P[1][10]*SF[7] + P[2][10]*SF[9] + P[3][10]*SF[8] + P[10][10]*SF[11] + P[11][10]*SPP[7] + P[12][10]*SPP[6]))/2;
	nextP[0][2] = P[0][2] + SQ[7] + P[1][2]*SF[7] + P[2][2]*SF[9] + P[3][2]*SF[8] + P[10][2]*SF[11] + P[11][2]*SPP[7] + P[12][2]*SPP[6] + SF[4]*(P[0][0] + P[1][0]*SF[7] + P[2][0]*SF[9] + P[3][0]*SF[8] + P[10][0]*SF[11] + P[11][0]*SPP[7] + P[12][0]*SPP[6]) + SF[8]*(P[0][1] + P[1][1]*SF[7] + P[2][1]*SF[9] + P[3][1]*SF[8] + P[10][1]*SF[11] + P[11][1]*SPP[7] + P[12][1]*SPP[6]) + SF[6]*(P[0][3] + P[1][3]*SF[7] + P[2][3]*SF[9] + P[3][3]*SF[8] + P[10][3]*SF[11] + P[11][3]*SPP[7] + P[12][3]*SPP[6]) + SF[11]*(P[0][12] + P[1][12]*SF[7] + P[2][12]*SF[9] + P[3][12]*SF[8] + P[10][12]*SF[11] + P[11][12]*SPP[7] + P[12][12]*SPP[6]) - SPP[6]*(P[0][10] + P[1][10]*SF[7] + P[2][10]*SF[9] + P[3][10]*SF[8] + P[10][10]*SF[11] + P[11][10]*SPP[7] + P[12][10]*SPP[6]) - (q0*(P[0][11] + P[1][11]*SF[7] + P[2][11]*SF[9] + P[3][11]*SF[8] + P[10][11]*SF[11] + P[11][11]*SPP[7] + P[12][11]*SPP[6]))/2;
	nextP[0][3] = P[0][3] + SQ[6] + P[1][3]*SF[7] + P[2][3]*SF[9] + P[3][3]*SF[8] + P[10][3]*SF[11] + P[11][3]*SPP[7] + P[12][3]*SPP[6] + SF[5]*(P[0][0] + P[1][0]*SF[7] + P[2][0]*SF[9] + P[3][0]*SF[8] + P[10][0]*SF[11] + P[11][0]*SPP[7] + P[12][0]*SPP[6]) + SF[4]*(P[0][1] + P[1][1]*SF[7] + P[2][1]*SF[9] + P[3][1]*SF[8] + P[10][1]*SF[11] + P[11][1]*SPP[7] + P[12][1]*SPP[6]) + SF[7]*(P[0][2] + P[1][2]*SF[7] + P[2][2]*SF[9] + P[3][2]*SF[8] + P[10][2]*SF[11] + P[11][2]*SPP[7] + P[12][2]*SPP[6]) - SF[11]*(P[0][11] + P[1][11]*SF[7] + P[2][11]*SF[9] + P[3][11]*SF[8] + P[10][11]*SF[11] + P[11][11]*SPP[7] + P[12][11]*SPP[6]) + SPP[7]*(P[0][10] + P[1][10]*SF[7] + P[2][10]*SF[9] + P[3][10]*SF[8] + P[10][10]*SF[11] + P[11][10]*SPP[7] + P[12][10]*SPP[6]) - (q0*(P[0][12] + P[1][12]*SF[7] + P[2][12]*SF[9] + P[3][12]*SF[8] + P[10][12]*SF[11] + P[11][12]*SPP[7] + P[12][12]*SPP[6]))/2;
	nextP[0][4] = P[0][4] + P[1][4]*SF[7] + P[2][4]*SF[9] + P[3][4]*SF[8] + P[10][4]*SF[11] + P[11][4]*SPP[7] + P[12][4]*SPP[6] + SF[3]*(P[0][0] + P[1][0]*SF[7] + P[2][0]*SF[9] + P[3][0]*SF[8] + P[10][0]*SF[11] + P[11][0]*SPP[7] + P[12][0]*SPP[6]) + SF[1]*(P[0][1] + P[1][1]*SF[7] + P[2][1]*SF[9] + P[3][1]*SF[8] + P[10][1]*SF[11] + P[11][1]*SPP[7] + P[12][1]*SPP[6]) + SPP[0]*(P[0][2] + P[1][2]*SF[7] + P[2][2]*SF[9] + P[3][2]*SF[8] + P[10][2]*SF[11] + P[11][2]*SPP[7] + P[12][2]*SPP[6]) - SPP[2]*(P[0][3] + P[1][3]*SF[7] + P[2][3]*SF[9] + P[3][3]*SF[8] + P[10][3]*SF[11] + P[11][3]*SPP[7] + P[12][3]*SPP[6]) - SPP[4]*(P[0][13] + P[1][13]*SF[7] + P[2][13]*SF[9] + P[3][13]*SF[8] + P[10][13]*SF[11] + P[11][13]*SPP[7] + P[12][13]*SPP[6]);
	nextP[0][5] = P[0][5] + P[1][5]*SF[7] + P[2][5]*SF[9] + P[3][5]*SF[8] + P[10][5]*SF[11] + P[11][5]*SPP[7] + P[12][5]*SPP[6] + SF[2]*(P[0][0] + P[1][0]*SF[7] + P[2][0]*SF[9] + P[3][0]*SF[8] + P[10][0]*SF[11] + P[11][0]*SPP[7] + P[12][0]*SPP[6]) + SF[1]*(P[0][2] + P[1][2]*SF[7] + P[2][2]*SF[9] + P[3][2]*SF[8] + P[10][2]*SF[11] + P[11][2]*SPP[7] + P[12][2]*SPP[6]) + SF[3]*(P[0][3] + P[1][3]*SF[7] + P[2][3]*SF[9] + P[3][3]*SF[8] + P[10][3]*SF[11] + P[11][3]*SPP[7] + P[12][3]*SPP[6]) - SPP[0]*(P[0][1] + P[1][1]*SF[7] + P[2][1]*SF[9] + P[3][1]*SF[8] + P[10][1]*SF[11] + P[11][1]*SPP[7] + P[12][1]*SPP[6]) + SPP[3]*(P[0][13] + P[1][13]*SF[7] + P[2][13]*SF[9] + P[3][13]*SF[8] + P[10][13]*SF[11] + P[11][13]*SPP[7] + P[12][13]*SPP[6]);
	nextP[0][6] = P[0][6] + P[1][6]*SF[7] + P[2][6]*SF[9] + P[3][6]*SF[8] + P[10][6]*SF[11] + P[11][6]*SPP[7] + P[12][6]*SPP[6] + SF[2]*(P[0][1] + P[1][1]*SF[7] + P[2][1]*SF[9] + P[3][1]*SF[8] + P[10][1]*SF[11] + P[11][1]*SPP[7] + P[12][1]*SPP[6]) + SF[1]*(P[0][3] + P[1][3]*SF[7] + P[2][3]*SF[9] + P[3][3]*SF[8] + P[10][3]*SF[11] + P[11][3]*SPP[7] + P[12][3]*SPP[6]) + SPP[0]*(P[0][0] + P[1][0]*SF[7] + P[2][0]*SF[9] + P[3][0]*SF[8] + P[10][0]*SF[11] + P[11][0]*SPP[7] + P[12][0]*SPP[6]) - SPP[1]*(P[0][2] + P[1][2]*SF[7] + P[2][2]*SF[9] + P[3][2]*SF[8] + P[10][2]*SF[11] + P[11][2]*SPP[7] + P[12][2]*SPP[6]) - (sq(q0) - sq(q1) - sq(q2) + sq(q3))*(P[0][13] + P[1][13]*SF[7] + P[2][13]*SF[9] + P[3][13]*SF[8] + P[10][13]*SF[11] + P[11][13]*SPP[7] + P[12][13]*SPP[6]);
	nextP[0][7] = P[0][7] + P[1][7]*SF[7] + P[2][7]*SF[9] + P[3][7]*SF[8] + P[10][7]*SF[11] + P[11][7]*SPP[7] + P[12][7]*SPP[6] + dt*(P[0][4] + P[1][4]*SF[7] + P[2][4]*SF[9] + P[3][4]*SF[8] + P[10][4]*SF[11] + P[11][4]*SPP[7] + P[12][4]*SPP[6]);
	nextP[0][8] = P[0][8] + P[1][8]*SF[7] + P[2][8]*SF[9] + P[3][8]*SF[8] + P[10][8]*SF[11] + P[11][8]*SPP[7] + P[12][8]*SPP[6] + dt*(P[0][5] + P[1][5]*SF[7] + P[2][5]*SF[9] + P[3][5]*SF[8] + P[10][5]*SF[11] + P[11][5]*SPP[7] + P[12][5]*SPP[6]);
	nextP[0][9] = P[0][9] + P[1][9]*SF[7] + P[2][9]*SF[9] + P[3][9]*SF[8] + P[10][9]*SF[11] + P[11][9]*SPP[7] + P[12][9]*SPP[6] + dt*(P[0][6] + P[1][6]*SF[7] + P[2][6]*SF[9] + P[3][6]*SF[8] + P[10][6]*SF[11] + P[11][6]*SPP[7] + P[12][6]*SPP[6]);
	nextP[0][10] = P[0][10] + P[1][10]*SF[7] + P[2][10]*SF[9] + P[3][10]*SF[8] + P[10][10]*SF[11] + P[11][10]*SPP[7] + P[12][10]*SPP[6];
	nextP[0][11] = P[0][11] + P[1][11]*SF[7] + P[2][11]*SF[9] + P[3][11]*SF[8] + P[10][11]*SF[11] + P[11][11]*SPP[7] + P[12][11]*SPP[6];
	nextP[0][12] = P[0][12] + P[1][12]*SF[7] + P[2][12]*SF[9] + P[3][12]*SF[8] + P[10][12]*SF[11] + P[11][12]*SPP[7] + P[12][12]*SPP[6];
	nextP[0][13] = P[0][13] + P[1][13]*SF[7] + P[2][13]*SF[9] + P[3][13]*SF[8] + P[10][13]*SF[11] + P[11][13]*SPP[7] + P[12][13]*SPP[6];
	nextP[0][14] = P[0][14] + P[1][14]*SF[7] + P[2][14]*SF[9] + P[3][14]*SF[8] + P[10][14]*SF[11] + P[11][14]*SPP[7] + P[12][14]*SPP[6];
	nextP[0][15] = P[0][15] + P[1][15]*SF[7] + P[2][15]*SF[9] + P[3][15]*SF[8] + P[10][15]*SF[11] + P[11][15]*SPP[7] + P[12][15]*SPP[6];
	nextP[0][16] = P[0][16] + P[1][16]*SF[7] + P[2][16]*SF[9] + P[3][16]*SF[8] + P[10][16]*SF[11] + P[11][16]*SPP[7] + P[12][16]*SPP[6];
	nextP[0][17] = P[0][17] + P[1][17]*SF[7] + P[2][17]*SF[9] + P[3][17]*SF[8] + P[10][17]*SF[11] + P[11][17]*SPP[7] + P[12][17]*SPP[6];
	nextP[0][18] = P[0][18] + P[1][18]*SF[7] + P[2][18]*SF[9] + P[3][18]*SF[8] + P[10][18]*SF[11] + P[11][18]*SPP[7] + P[12][18]*SPP[6];
	nextP[0][19] = P[0][19] + P[1][19]*SF[7] + P[2][19]*SF[9] + P[3][19]*SF[8] + P[10][19]*SF[11] + P[11][19]*SPP[7] + P[12][19]*SPP[6];
	nextP[0][20] = P[0][20] + P[1][20]*SF[7] + P[2][20]*SF[9] + P[3][20]*SF[8] + P[10][20]*SF[11] + P[11][20]*SPP[7] + P[12][20]*SPP[6];
	nextP[0][21] = P[0][21] + P[1][21]*SF[7] + P[2][21]*SF[9] + P[3][21]*SF[8] + P[10][21]*SF[11] + P[11][21]*SPP[7] + P[12][21]*SPP[6];
	nextP[1][0] = P[1][0] + SQ[8] + P[0][0]*SF[6] + P[2][0]*SF[5] + P[3][0]*SF[9] + P[11][0]*SPP[6] - P[12][0]*SPP[7] - (P[10][0]*q0)/2 + SF[7]*(P[1][1] + P[0][1]*SF[6] + P[2][1]*SF[5] + P[3][1]*SF[9] + P[11][1]*SPP[6] - P[12][1]*SPP[7] - (P[10][1]*q0)/2) + SF[9]*(P[1][2] + P[0][2]*SF[6] + P[2][2]*SF[5] + P[3][2]*SF[9] + P[11][2]*SPP[6] - P[12][2]*SPP[7] - (P[10][2]*q0)/2) + SF[8]*(P[1][3] + P[0][3]*SF[6] + P[2][3]*SF[5] + P[3][3]*SF[9] + P[11][3]*SPP[6] - P[12][3]*SPP[7] - (P[10][3]*q0)/2) + SF[11]*(P[1][10] + P[0][10]*SF[6] + P[2][10]*SF[5] + P[3][10]*SF[9] + P[11][10]*SPP[6] - P[12][10]*SPP[7] - (P[10][10]*q0)/2) + SPP[7]*(P[1][11] + P[0][11]*SF[6] + P[2][11]*SF[5] + P[3][11]*SF[9] + P[11][11]*SPP[6] - P[12][11]*SPP[7] - (P[10][11]*q0)/2) + SPP[6]*(P[1][12] + P[0][12]*SF[6] + P[2][12]*SF[5] + P[3][12]*SF[9] + P[11][12]*SPP[6] - P[12][12]*SPP[7] - (P[10][12]*q0)/2);
	nextP[1][1] = P[1][1] + P[0][1]*SF[6] + P[2][1]*SF[5] + P[3][1]*SF[9] + P[11][1]*SPP[6] - P[12][1]*SPP[7] + daxCov*SQ[9] - (P[10][1]*q0)/2 + SF[6]*(P[1][0] + P[0][0]*SF[6] + P[2][0]*SF[5] + P[3][0]*SF[9] + P[11][0]*SPP[6] - P[12][0]*SPP[7] - (P[10][0]*q0)/2) + SF[5]*(P[1][2] + P[0][2]*SF[6] + P[2][2]*SF[5] + P[3][2]*SF[9] + P[11][2]*SPP[6] - P[12][2]*SPP[7] - (P[10][2]*q0)/2) + SF[9]*(P[1][3] + P[0][3]*SF[6] + P[2][3]*SF[5] + P[3][3]*SF[9] + P[11][3]*SPP[6] - P[12][3]*SPP[7] - (P[10][3]*q0)/2) + SPP[6]*(P[1][11] + P[0][11]*SF[6] + P[2][11]*SF[5] + P[3][11]*SF[9] + P[11][11]*SPP[6] - P[12][11]*SPP[7] - (P[10][11]*q0)/2) - SPP[7]*(P[1][12] + P[0][12]*SF[6] + P[2][12]*SF[5] + P[3][12]*SF[9] + P[11][12]*SPP[6] - P[12][12]*SPP[7] - (P[10][12]*q0)/2) + (dayCov*sq(q3))/4 + (dazCov*sq(q2))/4 - (q0*(P[1][10] + P[0][10]*SF[6] + P[2][10]*SF[5] + P[3][10]*SF[9] + P[11][10]*SPP[6] - P[12][10]*SPP[7] - (P[10][10]*q0)/2))/2;
	nextP[1][2] = P[1][2] + SQ[5] + P[0][2]*SF[6] + P[2][2]*SF[5] + P[3][2]*SF[9] + P[11][2]*SPP[6] - P[12][2]*SPP[7] - (P[10][2]*q0)/2 + SF[4]*(P[1][0] + P[0][0]*SF[6] + P[2][0]*SF[5] + P[3][0]*SF[9] + P[11][0]*SPP[6] - P[12][0]*SPP[7] - (P[10][0]*q0)/2) + SF[8]*(P[1][1] + P[0][1]*SF[6] + P[2][1]*SF[5] + P[3][1]*SF[9] + P[11][1]*SPP[6] - P[12][1]*SPP[7] - (P[10][1]*q0)/2) + SF[6]*(P[1][3] + P[0][3]*SF[6] + P[2][3]*SF[5] + P[3][3]*SF[9] + P[11][3]*SPP[6] - P[12][3]*SPP[7] - (P[10][3]*q0)/2) + SF[11]*(P[1][12] + P[0][12]*SF[6] + P[2][12]*SF[5] + P[3][12]*SF[9] + P[11][12]*SPP[6] - P[12][12]*SPP[7] - (P[10][12]*q0)/2) - SPP[6]*(P[1][10] + P[0][10]*SF[6] + P[2][10]*SF[5] + P[3][10]*SF[9] + P[11][10]*SPP[6] - P[12][10]*SPP[7] - (P[10][10]*q0)/2) - (q0*(P[1][11] + P[0][11]*SF[6] + P[2][11]*SF[5] + P[3][11]*SF[9] + P[11][11]*SPP[6] - P[12][11]*SPP[7] - (P[10][11]*q0)/2))/2;
	nextP[1][3] = P[1][3] + SQ[4] + P[0][3]*SF[6] + P[2][3]*SF[5] + P[3][3]*SF[9] + P[11][3]*SPP[6] - P[12][3]*SPP[7] - (P[10][3]*q0)/2 + SF[5]*(P[1][0] + P[0][0]*SF[6] + P[2][0]*SF[5] + P[3][0]*SF[9] + P[11][0]*SPP[6] - P[12][0]*SPP[7] - (P[10][0]*q0)/2) + SF[4]*(P[1][1] + P[0][1]*SF[6] + P[2][1]*SF[5] + P[3][1]*SF[9] + P[11][1]*SPP[6] - P[12][1]*SPP[7] - (P[10][1]*q0)/2) + SF[7]*(P[1][2] + P[0][2]*SF[6] + P[2][2]*SF[5] + P[3][2]*SF[9] + P[11][2]*SPP[6] - P[12][2]*SPP[7] - (P[10][2]*q0)/2) - SF[11]*(P[1][11] + P[0][11]*SF[6] + P[2][11]*SF[5] + P[3][11]*SF[9] + P[11][11]*SPP[6] - P[12][11]*SPP[7] - (P[10][11]*q0)/2) + SPP[7]*(P[1][10] + P[0][10]*SF[6] + P[2][10]*SF[5] + P[3][10]*SF[9] + P[11][10]*SPP[6] - P[12][10]*SPP[7] - (P[10][10]*q0)/2) - (q0*(P[1][12] + P[0][12]*SF[6] + P[2][12]*SF[5] + P[3][12]*SF[9] + P[11][12]*SPP[6] - P[12][12]*SPP[7] - (P[10][12]*q0)/2))/2;
	nextP[1][4] = P[1][4] + P[0][4]*SF[6] + P[2][4]*SF[5] + P[3][4]*SF[9] + P[11][4]*SPP[6] - P[12][4]*SPP[7] - (P[10][4]*q0)/2 + SF[3]*(P[1][0] + P[0][0]*SF[6] + P[2][0]*SF[5] + P[3][0]*SF[9] + P[11][0]*SPP[6] - P[12][0]*SPP[7] - (P[10][0]*q0)/2) + SF[1]*(P[1][1] + P[0][1]*SF[6] + P[2][1]*SF[5] + P[3][1]*SF[9] + P[11][1]*SPP[6] - P[12][1]*SPP[7] - (P[10][1]*q0)/2) + SPP[0]*(P[1][2] + P[0][2]*SF[6] + P[2][2]*SF[5] + P[3][2]*SF[9] + P[11][2]*SPP[6] - P[12][2]*SPP[7] - (P[10][2]*q0)/2) - SPP[2]*(P[1][3] + P[0][3]*SF[6] + P[2][3]*SF[5] + P[3][3]*SF[9] + P[11][3]*SPP[6] - P[12][3]*SPP[7] - (P[10][3]*q0)/2) - SPP[4]*(P[1][13] + P[0][13]*SF[6] + P[2][13]*SF[5] + P[3][13]*SF[9] + P[11][13]*SPP[6] - P[12][13]*SPP[7] - (P[10][13]*q0)/2);
	nextP[1][5] = P[1][5] + P[0][5]*SF[6] + P[2][5]*SF[5] + P[3][5]*SF[9] + P[11][5]*SPP[6] - P[12][5]*SPP[7] - (P[10][5]*q0)/2 + SF[2]*(P[1][0] + P[0][0]*SF[6] + P[2][0]*SF[5] + P[3][0]*SF[9] + P[11][0]*SPP[6] - P[12][0]*SPP[7] - (P[10][0]*q0)/2) + SF[1]*(P[1][2] + P[0][2]*SF[6] + P[2][2]*SF[5] + P[3][2]*SF[9] + P[11][2]*SPP[6] - P[12][2]*SPP[7] - (P[10][2]*q0)/2) + SF[3]*(P[1][3] + P[0][3]*SF[6] + P[2][3]*SF[5] + P[3][3]*SF[9] + P[11][3]*SPP[6] - P[12][3]*SPP[7] - (P[10][3]*q0)/2) - SPP[0]*(P[1][1] + P[0][1]*SF[6] + P[2][1]*SF[5] + P[3][1]*SF[9] + P[11][1]*SPP[6] - P[12][1]*SPP[7] - (P[10][1]*q0)/2) + SPP[3]*(P[1][13] + P[0][13]*SF[6] + P[2][13]*SF[5] + P[3][13]*SF[9] + P[11][13]*SPP[6] - P[12][13]*SPP[7] - (P[10][13]*q0)/2);
	nextP[1][6] = P[1][6] + P[0][6]*SF[6] + P[2][6]*SF[5] + P[3][6]*SF[9] + P[11][6]*SPP[6] - P[12][6]*SPP[7] - (P[10][6]*q0)/2 + SF[2]*(P[1][1] + P[0][1]*SF[6] + P[2][1]*SF[5] + P[3][1]*SF[9] + P[11][1]*SPP[6] - P[12][1]*SPP[7] - (P[10][1]*q0)/2) + SF[1]*(P[1][3] + P[0][3]*SF[6] + P[2][3]*SF[5] + P[3][3]*SF[9] + P[11][3]*SPP[6] - P[12][3]*SPP[7] - (P[10][3]*q0)/2) + SPP[0]*(P[1][0] + P[0][0]*SF[6] + P[2][0]*SF[5] + P[3][0]*SF[9] + P[11][0]*SPP[6] - P[12][0]*SPP[7] - (P[10][0]*q0)/2) - SPP[1]*(P[1][2] + P[0][2]*SF[6] + P[2][2]*SF[5] + P[3][2]*SF[9] + P[11][2]*SPP[6] - P[12][2]*SPP[7] - (P[10][2]*q0)/2) - (sq(q0) - sq(q1) - sq(q2) + sq(q3))*(P[1][13] + P[0][13]*SF[6] + P[2][13]*SF[5] + P[3][13]*SF[9] + P[11][13]*SPP[6] - P[12][13]*SPP[7] - (P[10][13]*q0)/2);
	nextP[1][7] = P[1][7] + P[0][7]*SF[6] + P[2][7]*SF[5] + P[3][7]*SF[9] + P[11][7]*SPP[6] - P[12][7]*SPP[7] - (P[10][7]*q0)/2 + dt*(P[1][4] + P[0][4]*SF[6] + P[2][4]*SF[5] + P[3][4]*SF[9] + P[11][4]*SPP[6] - P[12][4]*SPP[7] - (P[10][4]*q0)/2);
	nextP[1][8] = P[1][8] + P[0][8]*SF[6] + P[2][8]*SF[5] + P[3][8]*SF[9] + P[11][8]*SPP[6] - P[12][8]*SPP[7] - (P[10][8]*q0)/2 + dt*(P[1][5] + P[0][5]*SF[6] + P[2][5]*SF[5] + P[3][5]*SF[9] + P[11][5]*SPP[6] - P[12][5]*SPP[7] - (P[10][5]*q0)/2);
	nextP[1][9] = P[1][9] + P[0][9]*SF[6] + P[2][9]*SF[5] + P[3][9]*SF[9] + P[11][9]*SPP[6] - P[12][9]*SPP[7] - (P[10][9]*q0)/2 + dt*(P[1][6] + P[0][6]*SF[6] + P[2][6]*SF[5] + P[3][6]*SF[9] + P[11][6]*SPP[6] - P[12][6]*SPP[7] - (P[10][6]*q0)/2);
	nextP[1][10] = P[1][10] + P[0][10]*SF[6] + P[2][10]*SF[5] + P[3][10]*SF[9] + P[11][10]*SPP[6] - P[12][10]*SPP[7] - (P[10][10]*q0)/2;
	nextP[1][11] = P[1][11] + P[0][11]*SF[6] + P[2][11]*SF[5] + P[3][11]*SF[9] + P[11][11]*SPP[6] - P[12][11]*SPP[7] - (P[10][11]*q0)/2;
	nextP[1][12] = P[1][12] + P[0][12]*SF[6] + P[2][12]*SF[5] + P[3][12]*SF[9] + P[11][12]*SPP[6] - P[12][12]*SPP[7] - (P[10][12]*q0)/2;
	nextP[1][13] = P[1][13] + P[0][13]*SF[6] + P[2][13]*SF[5] + P[3][13]*SF[9] + P[11][13]*SPP[6] - P[12][13]*SPP[7] - (P[10][13]*q0)/2;
	nextP[1][14] = P[1][14] + P[0][14]*SF[6] + P[2][14]*SF[5] + P[3][14]*SF[9] + P[11][14]*SPP[6] - P[12][14]*SPP[7] - (P[10][14]*q0)/2;
	nextP[1][15] = P[1][15] + P[0][15]*SF[6] + P[2][15]*SF[5] + P[3][15]*SF[9] + P[11][15]*SPP[6] - P[12][15]*SPP[7] - (P[10][15]*q0)/2;
	nextP[1][16] = P[1][16] + P[0][16]*SF[6] + P[2][16]*SF[5] + P[3][16]*SF[9] + P[11][16]*SPP[6] - P[12][16]*SPP[7] - (P[10][16]*q0)/2;
	nextP[1][17] = P[1][17] + P[0][17]*SF[6] + P[2][17]*SF[5] + P[3][17]*SF[9] + P[11][17]*SPP[6] - P[12][17]*SPP[7] - (P[10][17]*q0)/2;
	nextP[1][18] = P[1][18] + P[0][18]*SF[6] + P[2][18]*SF[5] + P[3][18]*SF[9] + P[11][18]*SPP[6] - P[12][18]*SPP[7] - (P[10][18]*q0)/2;
	nextP[1][19] = P[1][19] + P[0][19]*SF[6] + P[2][19]*SF[5] + P[3][19]*SF[9] + P[11][19]*SPP[6] - P[12][19]*SPP[7] - (P[10][19]*q0)/2;
	nextP[1][20] = P[1][20] + P[0][20]*SF[6] + P[2][20]*SF[5] + P[3][20]*SF[9] + P[11][20]*SPP[6] - P[12][20]*SPP[7] - (P[10][20]*q0)/2;
	nextP[1][21] = P[1][21] + P[0][21]*SF[6] + P[2][21]*SF[5] + P[3][21]*SF[9] + P[11][21]*SPP[6] - P[12][21]*SPP[7] - (P[10][21]*q0)/2;
	nextP[2][0] = P[2][0] + SQ[7] + P[0][0]*SF[4] + P[1][0]*SF[8] + P[3][0]*SF[6] + P[12][0]*SF[11] - P[10][0]*SPP[6] - (P[11][0]*q0)/2 + SF[7]*(P[2][1] + P[0][1]*SF[4] + P[1][1]*SF[8] + P[3][1]*SF[6] + P[12][1]*SF[11] - P[10][1]*SPP[6] - (P[11][1]*q0)/2) + SF[9]*(P[2][2] + P[0][2]*SF[4] + P[1][2]*SF[8] + P[3][2]*SF[6] + P[12][2]*SF[11] - P[10][2]*SPP[6] - (P[11][2]*q0)/2) + SF[8]*(P[2][3] + P[0][3]*SF[4] + P[1][3]*SF[8] + P[3][3]*SF[6] + P[12][3]*SF[11] - P[10][3]*SPP[6] - (P[11][3]*q0)/2) + SF[11]*(P[2][10] + P[0][10]*SF[4] + P[1][10]*SF[8] + P[3][10]*SF[6] + P[12][10]*SF[11] - P[10][10]*SPP[6] - (P[11][10]*q0)/2) + SPP[7]*(P[2][11] + P[0][11]*SF[4] + P[1][11]*SF[8] + P[3][11]*SF[6] + P[12][11]*SF[11] - P[10][11]*SPP[6] - (P[11][11]*q0)/2) + SPP[6]*(P[2][12] + P[0][12]*SF[4] + P[1][12]*SF[8] + P[3][12]*SF[6] + P[12][12]*SF[11] - P[10][12]*SPP[6] - (P[11][12]*q0)/2);
	nextP[2][1] = P[2][1] + SQ[5] + P[0][1]*SF[4] + P[1][1]*SF[8] + P[3][1]*SF[6] + P[12][1]*SF[11] - P[10][1]*SPP[6] - (P[11][1]*q0)/2 + SF[6]*(P[2][0] + P[0][0]*SF[4] + P[1][0]*SF[8] + P[3][0]*SF[6] + P[12][0]*SF[11] - P[10][0]*SPP[6] - (P[11][0]*q0)/2) + SF[5]*(P[2][2] + P[0][2]*SF[4] + P[1][2]*SF[8] + P[3][2]*SF[6] + P[12][2]*SF[11] - P[10][2]*SPP[6] - (P[11][2]*q0)/2) + SF[9]*(P[2][3] + P[0][3]*SF[4] + P[1][3]*SF[8] + P[3][3]*SF[6] + P[12][3]*SF[11] - P[10][3]*SPP[6] - (P[11][3]*q0)/2) + SPP[6]*(P[2][11] + P[0][11]*SF[4] + P[1][11]*SF[8] + P[3][11]*SF[6] + P[12][11]*SF[11] - P[10][11]*SPP[6] - (P[11][11]*q0)/2) - SPP[7]*(P[2][12] + P[0][12]*SF[4] + P[1][12]*SF[8] + P[3][12]*SF[6] + P[12][12]*SF[11] - P[10][12]*SPP[6] - (P[11][12]*q0)/2) - (q0*(P[2][10] + P[0][10]*SF[4] + P[1][10]*SF[8] + P[3][10]*SF[6] + P[12][10]*SF[11] - P[10][10]*SPP[6] - (P[11][10]*q0)/2))/2;
	nextP[2][2] = P[2][2] + P[0][2]*SF[4] + P[1][2]*SF[8] + P[3][2]*SF[6] + P[12][2]*SF[11] - P[10][2]*SPP[6] + dayCov*SQ[9] + (dazCov*SQ[10])/4 - (P[11][2]*q0)/2 + SF[4]*(P[2][0] + P[0][0]*SF[4] + P[1][0]*SF[8] + P[3][0]*SF[6] + P[12][0]*SF[11] - P[10][0]*SPP[6] - (P[11][0]*q0)/2) + SF[8]*(P[2][1] + P[0][1]*SF[4] + P[1][1]*SF[8] + P[3][1]*SF[6] + P[12][1]*SF[11] - P[10][1]*SPP[6] - (P[11][1]*q0)/2) + SF[6]*(P[2][3] + P[0][3]*SF[4] + P[1][3]*SF[8] + P[3][3]*SF[6] + P[12][3]*SF[11] - P[10][3]*SPP[6] - (P[11][3]*q0)/2) + SF[11]*(P[2][12] + P[0][12]*SF[4] + P[1][12]*SF[8] + P[3][12]*SF[6] + P[12][12]*SF[11] - P[10][12]*SPP[6] - (P[11][12]*q0)/2) - SPP[6]*(P[2][10] + P[0][10]*SF[4] + P[1][10]*SF[8] + P[3][10]*SF[6] + P[12][10]*SF[11] - P[10][10]*SPP[6] - (P[11][10]*q0)/2) + (daxCov*sq(q3))/4 - (q0*(P[2][11] + P[0][11]*SF[4] + P[1][11]*SF[8] + P[3][11]*SF[6] + P[12][11]*SF[11] - P[10][11]*SPP[6] - (P[11][11]*q0)/2))/2;
	nextP[2][3] = P[2][3] + SQ[3] + P[0][3]*SF[4] + P[1][3]*SF[8] + P[3][3]*SF[6] + P[12][3]*SF[11] - P[10][3]*SPP[6] - (P[11][3]*q0)/2 + SF[5]*(P[2][0] + P[0][0]*SF[4] + P[1][0]*SF[8] + P[3][0]*SF[6] + P[12][0]*SF[11] - P[10][0]*SPP[6] - (P[11][0]*q0)/2) + SF[4]*(P[2][1] + P[0][1]*SF[4] + P[1][1]*SF[8] + P[3][1]*SF[6] + P[12][1]*SF[11] - P[10][1]*SPP[6] - (P[11][1]*q0)/2) + SF[7]*(P[2][2] + P[0][2]*SF[4] + P[1][2]*SF[8] + P[3][2]*SF[6] + P[12][2]*SF[11] - P[10][2]*SPP[6] - (P[11][2]*q0)/2) - SF[11]*(P[2][11] + P[0][11]*SF[4] + P[1][11]*SF[8] + P[3][11]*SF[6] + P[12][11]*SF[11] - P[10][11]*SPP[6] - (P[11][11]*q0)/2) + SPP[7]*(P[2][10] + P[0][10]*SF[4] + P[1][10]*SF[8] + P[3][10]*SF[6] + P[12][10]*SF[11] - P[10][10]*SPP[6] - (P[11][10]*q0)/2) - (q0*(P[2][12] + P[0][12]*SF[4] + P[1][12]*SF[8] + P[3][12]*SF[6] + P[12][12]*SF[11] - P[10][12]*SPP[6] - (P[11][12]*q0)/2))/2;
	nextP[2][4] = P[2][4] + P[0][4]*SF[4] + P[1][4]*SF[8] + P[3][4]*SF[6] + P[12][4]*SF[11] - P[10][4]*SPP[6] - (P[11][4]*q0)/2 + SF[3]*(P[2][0] + P[0][0]*SF[4] + P[1][0]*SF[8] + P[3][0]*SF[6] + P[12][0]*SF[11] - P[10][0]*SPP[6] - (P[11][0]*q0)/2) + SF[1]*(P[2][1] + P[0][1]*SF[4] + P[1][1]*SF[8] + P[3][1]*SF[6] + P[12][1]*SF[11] - P[10][1]*SPP[6] - (P[11][1]*q0)/2) + SPP[0]*(P[2][2] + P[0][2]*SF[4] + P[1][2]*SF[8] + P[3][2]*SF[6] + P[12][2]*SF[11] - P[10][2]*SPP[6] - (P[11][2]*q0)/2) - SPP[2]*(P[2][3] + P[0][3]*SF[4] + P[1][3]*SF[8] + P[3][3]*SF[6] + P[12][3]*SF[11] - P[10][3]*SPP[6] - (P[11][3]*q0)/2) - SPP[4]*(P[2][13] + P[0][13]*SF[4] + P[1][13]*SF[8] + P[3][13]*SF[6] + P[12][13]*SF[11] - P[10][13]*SPP[6] - (P[11][13]*q0)/2);
	nextP[2][5] = P[2][5] + P[0][5]*SF[4] + P[1][5]*SF[8] + P[3][5]*SF[6] + P[12][5]*SF[11] - P[10][5]*SPP[6] - (P[11][5]*q0)/2 + SF[2]*(P[2][0] + P[0][0]*SF[4] + P[1][0]*SF[8] + P[3][0]*SF[6] + P[12][0]*SF[11] - P[10][0]*SPP[6] - (P[11][0]*q0)/2) + SF[1]*(P[2][2] + P[0][2]*SF[4] + P[1][2]*SF[8] + P[3][2]*SF[6] + P[12][2]*SF[11] - P[10][2]*SPP[6] - (P[11][2]*q0)/2) + SF[3]*(P[2][3] + P[0][3]*SF[4] + P[1][3]*SF[8] + P[3][3]*SF[6] + P[12][3]*SF[11] - P[10][3]*SPP[6] - (P[11][3]*q0)/2) - SPP[0]*(P[2][1] + P[0][1]*SF[4] + P[1][1]*SF[8] + P[3][1]*SF[6] + P[12][1]*SF[11] - P[10][1]*SPP[6] - (P[11][1]*q0)/2) + SPP[3]*(P[2][13] + P[0][13]*SF[4] + P[1][13]*SF[8] + P[3][13]*SF[6] + P[12][13]*SF[11] - P[10][13]*SPP[6] - (P[11][13]*q0)/2);
	nextP[2][6] = P[2][6] + P[0][6]*SF[4] + P[1][6]*SF[8] + P[3][6]*SF[6] + P[12][6]*SF[11] - P[10][6]*SPP[6] - (P[11][6]*q0)/2 + SF[2]*(P[2][1] + P[0][1]*SF[4] + P[1][1]*SF[8] + P[3][1]*SF[6] + P[12][1]*SF[11] - P[10][1]*SPP[6] - (P[11][1]*q0)/2) + SF[1]*(P[2][3] + P[0][3]*SF[4] + P[1][3]*SF[8] + P[3][3]*SF[6] + P[12][3]*SF[11] - P[10][3]*SPP[6] - (P[11][3]*q0)/2) + SPP[0]*(P[2][0] + P[0][0]*SF[4] + P[1][0]*SF[8] + P[3][0]*SF[6] + P[12][0]*SF[11] - P[10][0]*SPP[6] - (P[11][0]*q0)/2) - SPP[1]*(P[2][2] + P[0][2]*SF[4] + P[1][2]*SF[8] + P[3][2]*SF[6] + P[12][2]*SF[11] - P[10][2]*SPP[6] - (P[11][2]*q0)/2) - (sq(q0) - sq(q1) - sq(q2) + sq(q3))*(P[2][13] + P[0][13]*SF[4] + P[1][13]*SF[8] + P[3][13]*SF[6] + P[12][13]*SF[11] - P[10][13]*SPP[6] - (P[11][13]*q0)/2);
	nextP[2][7] = P[2][7] + P[0][7]*SF[4] + P[1][7]*SF[8] + P[3][7]*SF[6] + P[12][7]*SF[11] - P[10][7]*SPP[6] - (P[11][7]*q0)/2 + dt*(P[2][4] + P[0][4]*SF[4] + P[1][4]*SF[8] + P[3][4]*SF[6] + P[12][4]*SF[11] - P[10][4]*SPP[6] - (P[11][4]*q0)/2);
	nextP[2][8] = P[2][8] + P[0][8]*SF[4] + P[1][8]*SF[8] + P[3][8]*SF[6] + P[12][8]*SF[11] - P[10][8]*SPP[6] - (P[11][8]*q0)/2 + dt*(P[2][5] + P[0][5]*SF[4] + P[1][5]*SF[8] + P[3][5]*SF[6] + P[12][5]*SF[11] - P[10][5]*SPP[6] - (P[11][5]*q0)/2);
	nextP[2][9] = P[2][9] + P[0][9]*SF[4] + P[1][9]*SF[8] + P[3][9]*SF[6] + P[12][9]*SF[11] - P[10][9]*SPP[6] - (P[11][9]*q0)/2 + dt*(P[2][6] + P[0][6]*SF[4] + P[1][6]*SF[8] + P[3][6]*SF[6] + P[12][6]*SF[11] - P[10][6]*SPP[6] - (P[11][6]*q0)/2);
	nextP[2][10] = P[2][10] + P[0][10]*SF[4] + P[1][10]*SF[8] + P[3][10]*SF[6] + P[12][10]*SF[11] - P[10][10]*SPP[6] - (P[11][10]*q0)/2;
	nextP[2][11] = P[2][11] + P[0][11]*SF[4] + P[1][11]*SF[8] + P[3][11]*SF[6] + P[12][11]*SF[11] - P[10][11]*SPP[6] - (P[11][11]*q0)/2;
	nextP[2][12] = P[2][12] + P[0][12]*SF[4] + P[1][12]*SF[8] + P[3][12]*SF[6] + P[12][12]*SF[11] - P[10][12]*SPP[6] - (P[11][12]*q0)/2;
	nextP[2][13] = P[2][13] + P[0][13]*SF[4] + P[1][13]*SF[8] + P[3][13]*SF[6] + P[12][13]*SF[11] - P[10][13]*SPP[6] - (P[11][13]*q0)/2;
	nextP[2][14] = P[2][14] + P[0][14]*SF[4] + P[1][14]*SF[8] + P[3][14]*SF[6] + P[12][14]*SF[11] - P[10][14]*SPP[6] - (P[11][14]*q0)/2;
	nextP[2][15] = P[2][15] + P[0][15]*SF[4] + P[1][15]*SF[8] + P[3][15]*SF[6] + P[12][15]*SF[11] - P[10][15]*SPP[6] - (P[11][15]*q0)/2;
	nextP[2][16] = P[2][16] + P[0][16]*SF[4] + P[1][16]*SF[8] + P[3][16]*SF[6] + P[12][16]*SF[11] - P[10][16]*SPP[6] - (P[11][16]*q0)/2;
	nextP[2][17] = P[2][17] + P[0][17]*SF[4] + P[1][17]*SF[8] + P[3][17]*SF[6] + P[12][17]*SF[11] - P[10][17]*SPP[6] - (P[11][17]*q0)/2;
	nextP[2][18] = P[2][18] + P[0][18]*SF[4] + P[1][18]*SF[8] + P[3][18]*SF[6] + P[12][18]*SF[11] - P[10][18]*SPP[6] - (P[11][18]*q0)/2;
	nextP[2][19] = P[2][19] + P[0][19]*SF[4] + P[1][19]*SF[8] + P[3][19]*SF[6] + P[12][19]*SF[11] - P[10][19]*SPP[6] - (P[11][19]*q0)/2;
	nextP[2][20] = P[2][20] + P[0][20]*SF[4] + P[1][20]*SF[8] + P[3][20]*SF[6] + P[12][20]*SF[11] - P[10][20]*SPP[6] - (P[11][20]*q0)/2;
	nextP[2][21] = P[2][21] + P[0][21]*SF[4] + P[1][21]*SF[8] + P[3][21]*SF[6] + P[12][21]*SF[11] - P[10][21]*SPP[6] - (P[11][21]*q0)/2;
	nextP[3][0] = P[3][0] + SQ[6] + P[0][0]*SF[5] + P[1][0]*SF[4] + P[2][0]*SF[7] - P[11][0]*SF[11] + P[10][0]*SPP[7] - (P[12][0]*q0)/2 + SF[7]*(P[3][1] + P[0][1]*SF[5] + P[1][1]*SF[4] + P[2][1]*SF[7] - P[11][1]*SF[11] + P[10][1]*SPP[7] - (P[12][1]*q0)/2) + SF[9]*(P[3][2] + P[0][2]*SF[5] + P[1][2]*SF[4] + P[2][2]*SF[7] - P[11][2]*SF[11] + P[10][2]*SPP[7] - (P[12][2]*q0)/2) + SF[8]*(P[3][3] + P[0][3]*SF[5] + P[1][3]*SF[4] + P[2][3]*SF[7] - P[11][3]*SF[11] + P[10][3]*SPP[7] - (P[12][3]*q0)/2) + SF[11]*(P[3][10] + P[0][10]*SF[5] + P[1][10]*SF[4] + P[2][10]*SF[7] - P[11][10]*SF[11] + P[10][10]*SPP[7] - (P[12][10]*q0)/2) + SPP[7]*(P[3][11] + P[0][11]*SF[5] + P[1][11]*SF[4] + P[2][11]*SF[7] - P[11][11]*SF[11] + P[10][11]*SPP[7] - (P[12][11]*q0)/2) + SPP[6]*(P[3][12] + P[0][12]*SF[5] + P[1][12]*SF[4] + P[2][12]*SF[7] - P[11][12]*SF[11] + P[10][12]*SPP[7] - (P[12][12]*q0)/2);
	nextP[3][1] = P[3][1] + SQ[4] + P[0][1]*SF[5] + P[1][1]*SF[4] + P[2][1]*SF[7] - P[11][1]*SF[11] + P[10][1]*SPP[7] - (P[12][1]*q0)/2 + SF[6]*(P[3][0] + P[0][0]*SF[5] + P[1][0]*SF[4] + P[2][0]*SF[7] - P[11][0]*SF[11] + P[10][0]*SPP[7] - (P[12][0]*q0)/2) + SF[5]*(P[3][2] + P[0][2]*SF[5] + P[1][2]*SF[4] + P[2][2]*SF[7] - P[11][2]*SF[11] + P[10][2]*SPP[7] - (P[12][2]*q0)/2) + SF[9]*(P[3][3] + P[0][3]*SF[5] + P[1][3]*SF[4] + P[2][3]*SF[7] - P[11][3]*SF[11] + P[10][3]*SPP[7] - (P[12][3]*q0)/2) + SPP[6]*(P[3][11] + P[0][11]*SF[5] + P[1][11]*SF[4] + P[2][11]*SF[7] - P[11][11]*SF[11] + P[10][11]*SPP[7] - (P[12][11]*q0)/2) - SPP[7]*(P[3][12] + P[0][12]*SF[5] + P[1][12]*SF[4] + P[2][12]*SF[7] - P[11][12]*SF[11] + P[10][12]*SPP[7] - (P[12][12]*q0)/2) - (q0*(P[3][10] + P[0][10]*SF[5] + P[1][10]*SF[4] + P[2][10]*SF[7] - P[11][10]*SF[11] + P[10][10]*SPP[7] - (P[12][10]*q0)/2))/2;
	nextP[3][2] = P[3][2] + SQ[3] + P[0][2]*SF[5] + P[1][2]*SF[4] + P[2][2]*SF[7] - P[11][2]*SF[11] + P[10][2]*SPP[7] - (P[12][2]*q0)/2 + SF[4]*(P[3][0] + P[0][0]*SF[5] + P[1][0]*SF[4] + P[2][0]*SF[7] - P[11][0]*SF[11] + P[10][0]*SPP[7] - (P[12][0]*q0)/2) + SF[8]*(P[3][1] + P[0][1]*SF[5] + P[1][1]*SF[4] + P[2][1]*SF[7] - P[11][1]*SF[11] + P[10][1]*SPP[7] - (P[12][1]*q0)/2) + SF[6]*(P[3][3] + P[0][3]*SF[5] + P[1][3]*SF[4] + P[2][3]*SF[7] - P[11][3]*SF[11] + P[10][3]*SPP[7] - (P[12][3]*q0)/2) + SF[11]*(P[3][12] + P[0][12]*SF[5] + P[1][12]*SF[4] + P[2][12]*SF[7] - P[11][12]*SF[11] + P[10][12]*SPP[7] - (P[12][12]*q0)/2) - SPP[6]*(P[3][10] + P[0][10]*SF[5] + P[1][10]*SF[4] + P[2][10]*SF[7] - P[11][10]*SF[11] + P[10][10]*SPP[7] - (P[12][10]*q0)/2) - (q0*(P[3][11] + P[0][11]*SF[5] + P[1][11]*SF[4] + P[2][11]*SF[7] - P[11][11]*SF[11] + P[10][11]*SPP[7] - (P[12][11]*q0)/2))/2;
	nextP[3][3] = P[3][3] + P[0][3]*SF[5] + P[1][3]*SF[4] + P[2][3]*SF[7] - P[11][3]*SF[11] + P[10][3]*SPP[7] + (dayCov*SQ[10])/4 + dazCov*SQ[9] - (P[12][3]*q0)/2 + SF[5]*(P[3][0] + P[0][0]*SF[5] + P[1][0]*SF[4] + P[2][0]*SF[7] - P[11][0]*SF[11] + P[10][0]*SPP[7] - (P[12][0]*q0)/2) + SF[4]*(P[3][1] + P[0][1]*SF[5] + P[1][1]*SF[4] + P[2][1]*SF[7] - P[11][1]*SF[11] + P[10][1]*SPP[7] - (P[12][1]*q0)/2) + SF[7]*(P[3][2] + P[0][2]*SF[5] + P[1][2]*SF[4] + P[2][2]*SF[7] - P[11][2]*SF[11] + P[10][2]*SPP[7] - (P[12][2]*q0)/2) - SF[11]*(P[3][11] + P[0][11]*SF[5] + P[1][11]*SF[4] + P[2][11]*SF[7] - P[11][11]*SF[11] + P[10][11]*SPP[7] - (P[12][11]*q0)/2) + SPP[7]*(P[3][10] + P[0][10]*SF[5] + P[1][10]*SF[4] + P[2][10]*SF[7] - P[11][10]*SF[11] + P[10][10]*SPP[7] - (P[12][10]*q0)/2) + (daxCov*sq(q2))/4 - (q0*(P[3][12] + P[0][12]*SF[5] + P[1][12]*SF[4] + P[2][12]*SF[7] - P[11][12]*SF[11] + P[10][12]*SPP[7] - (P[12][12]*q0)/2))/2;
	nextP[3][4] = P[3][4] + P[0][4]*SF[5] + P[1][4]*SF[4] + P[2][4]*SF[7] - P[11][4]*SF[11] + P[10][4]*SPP[7] - (P[12][4]*q0)/2 + SF[3]*(P[3][0] + P[0][0]*SF[5] + P[1][0]*SF[4] + P[2][0]*SF[7] - P[11][0]*SF[11] + P[10][0]*SPP[7] - (P[12][0]*q0)/2) + SF[1]*(P[3][1] + P[0][1]*SF[5] + P[1][1]*SF[4] + P[2][1]*SF[7] - P[11][1]*SF[11] + P[10][1]*SPP[7] - (P[12][1]*q0)/2) + SPP[0]*(P[3][2] + P[0][2]*SF[5] + P[1][2]*SF[4] + P[2][2]*SF[7] - P[11][2]*SF[11] + P[10][2]*SPP[7] - (P[12][2]*q0)/2) - SPP[2]*(P[3][3] + P[0][3]*SF[5] + P[1][3]*SF[4] + P[2][3]*SF[7] - P[11][3]*SF[11] + P[10][3]*SPP[7] - (P[12][3]*q0)/2) - SPP[4]*(P[3][13] + P[0][13]*SF[5] + P[1][13]*SF[4] + P[2][13]*SF[7] - P[11][13]*SF[11] + P[10][13]*SPP[7] - (P[12][13]*q0)/2);
	nextP[3][5] = P[3][5] + P[0][5]*SF[5] + P[1][5]*SF[4] + P[2][5]*SF[7] - P[11][5]*SF[11] + P[10][5]*SPP[7] - (P[12][5]*q0)/2 + SF[2]*(P[3][0] + P[0][0]*SF[5] + P[1][0]*SF[4] + P[2][0]*SF[7] - P[11][0]*SF[11] + P[10][0]*SPP[7] - (P[12][0]*q0)/2) + SF[1]*(P[3][2] + P[0][2]*SF[5] + P[1][2]*SF[4] + P[2][2]*SF[7] - P[11][2]*SF[11] + P[10][2]*SPP[7] - (P[12][2]*q0)/2) + SF[3]*(P[3][3] + P[0][3]*SF[5] + P[1][3]*SF[4] + P[2][3]*SF[7] - P[11][3]*SF[11] + P[10][3]*SPP[7] - (P[12][3]*q0)/2) - SPP[0]*(P[3][1] + P[0][1]*SF[5] + P[1][1]*SF[4] + P[2][1]*SF[7] - P[11][1]*SF[11] + P[10][1]*SPP[7] - (P[12][1]*q0)/2) + SPP[3]*(P[3][13] + P[0][13]*SF[5] + P[1][13]*SF[4] + P[2][13]*SF[7] - P[11][13]*SF[11] + P[10][13]*SPP[7] - (P[12][13]*q0)/2);
	nextP[3][6] = P[3][6] + P[0][6]*SF[5] + P[1][6]*SF[4] + P[2][6]*SF[7] - P[11][6]*SF[11] + P[10][6]*SPP[7] - (P[12][6]*q0)/2 + SF[2]*(P[3][1] + P[0][1]*SF[5] + P[1][1]*SF[4] + P[2][1]*SF[7] - P[11][1]*SF[11] + P[10][1]*SPP[7] - (P[12][1]*q0)/2) + SF[1]*(P[3][3] + P[0][3]*SF[5] + P[1][3]*SF[4] + P[2][3]*SF[7] - P[11][3]*SF[11] + P[10][3]*SPP[7] - (P[12][3]*q0)/2) + SPP[0]*(P[3][0] + P[0][0]*SF[5] + P[1][0]*SF[4] + P[2][0]*SF[7] - P[11][0]*SF[11] + P[10][0]*SPP[7] - (P[12][0]*q0)/2) - SPP[1]*(P[3][2] + P[0][2]*SF[5] + P[1][2]*SF[4] + P[2][2]*SF[7] - P[11][2]*SF[11] + P[10][2]*SPP[7] - (P[12][2]*q0)/2) - (sq(q0) - sq(q1) - sq(q2) + sq(q3))*(P[3][13] + P[0][13]*SF[5] + P[1][13]*SF[4] + P[2][13]*SF[7] - P[11][13]*SF[11] + P[10][13]*SPP[7] - (P[12][13]*q0)/2);
	nextP[3][7] = P[3][7] + P[0][7]*SF[5] + P[1][7]*SF[4] + P[2][7]*SF[7] - P[11][7]*SF[11] + P[10][7]*SPP[7] - (P[12][7]*q0)/2 + dt*(P[3][4] + P[0][4]*SF[5] + P[1][4]*SF[4] + P[2][4]*SF[7] - P[11][4]*SF[11] + P[10][4]*SPP[7] - (P[12][4]*q0)/2);
	nextP[3][8] = P[3][8] + P[0][8]*SF[5] + P[1][8]*SF[4] + P[2][8]*SF[7] - P[11][8]*SF[11] + P[10][8]*SPP[7] - (P[12][8]*q0)/2 + dt*(P[3][5] + P[0][5]*SF[5] + P[1][5]*SF[4] + P[2][5]*SF[7] - P[11][5]*SF[11] + P[10][5]*SPP[7] - (P[12][5]*q0)/2);
	nextP[3][9] = P[3][9] + P[0][9]*SF[5] + P[1][9]*SF[4] + P[2][9]*SF[7] - P[11][9]*SF[11] + P[10][9]*SPP[7] - (P[12][9]*q0)/2 + dt*(P[3][6] + P[0][6]*SF[5] + P[1][6]*SF[4] + P[2][6]*SF[7] - P[11][6]*SF[11] + P[10][6]*SPP[7] - (P[12][6]*q0)/2);
	nextP[3][10] = P[3][10] + P[0][10]*SF[5] + P[1][10]*SF[4] + P[2][10]*SF[7] - P[11][10]*SF[11] + P[10][10]*SPP[7] - (P[12][10]*q0)/2;
	nextP[3][11] = P[3][11] + P[0][11]*SF[5] + P[1][11]*SF[4] + P[2][11]*SF[7] - P[11][11]*SF[11] + P[10][11]*SPP[7] - (P[12][11]*q0)/2;
	nextP[3][12] = P[3][12] + P[0][12]*SF[5] + P[1][12]*SF[4] + P[2][12]*SF[7] - P[11][12]*SF[11] + P[10][12]*SPP[7] - (P[12][12]*q0)/2;
	nextP[3][13] = P[3][13] + P[0][13]*SF[5] + P[1][13]*SF[4] + P[2][13]*SF[7] - P[11][13]*SF[11] + P[10][13]*SPP[7] - (P[12][13]*q0)/2;
	nextP[3][14] = P[3][14] + P[0][14]*SF[5] + P[1][14]*SF[4] + P[2][14]*SF[7] - P[11][14]*SF[11] + P[10][14]*SPP[7] - (P[12][14]*q0)/2;
	nextP[3][15] = P[3][15] + P[0][15]*SF[5] + P[1][15]*SF[4] + P[2][15]*SF[7] - P[11][15]*SF[11] + P[10][15]*SPP[7] - (P[12][15]*q0)/2;
	nextP[3][16] = P[3][16] + P[0][16]*SF[5] + P[1][16]*SF[4] + P[2][16]*SF[7] - P[11][16]*SF[11] + P[10][16]*SPP[7] - (P[12][16]*q0)/2;
	nextP[3][17] = P[3][17] + P[0][17]*SF[5] + P[1][17]*SF[4] + P[2][17]*SF[7] - P[11][17]*SF[11] + P[10][17]*SPP[7] - (P[12][17]*q0)/2;
	nextP[3][18] = P[3][18] + P[0][18]*SF[5] + P[1][18]*SF[4] + P[2][18]*SF[7] - P[11][18]*SF[11] + P[10][18]*SPP[7] - (P[12][18]*q0)/2;
	nextP[3][19] = P[3][19] + P[0][19]*SF[5] + P[1][19]*SF[4] + P[2][19]*SF[7] - P[11][19]*SF[11] + P[10][19]*SPP[7] - (P[12][19]*q0)/2;
	nextP[3][20] = P[3][20] + P[0][20]*SF[5] + P[1][20]*SF[4] + P[2][20]*SF[7] - P[11][20]*SF[11] + P[10][20]*SPP[7] - (P[12][20]*q0)/2;
	nextP[3][21] = P[3][21] + P[0][21]*SF[5] + P[1][21]*SF[4] + P[2][21]*SF[7] - P[11][21]*SF[11] + P[10][21]*SPP[7] - (P[12][21]*q0)/2;
	nextP[4][0] = P[4][0] + P[0][0]*SF[3] + P[1][0]*SF[1] + P[2][0]*SPP[0] - P[3][0]*SPP[2] - P[13][0]*SPP[4] + SF[7]*(P[4][1] + P[0][1]*SF[3] + P[1][1]*SF[1] + P[2][1]*SPP[0] - P[3][1]*SPP[2] - P[13][1]*SPP[4]) + SF[9]*(P[4][2] + P[0][2]*SF[3] + P[1][2]*SF[1] + P[2][2]*SPP[0] - P[3][2]*SPP[2] - P[13][2]*SPP[4]) + SF[8]*(P[4][3] + P[0][3]*SF[3] + P[1][3]*SF[1] + P[2][3]*SPP[0] - P[3][3]*SPP[2] - P[13][3]*SPP[4]) + SF[11]*(P[4][10] + P[0][10]*SF[3] + P[1][10]*SF[1] + P[2][10]*SPP[0] - P[3][10]*SPP[2] - P[13][10]*SPP[4]) + SPP[7]*(P[4][11] + P[0][11]*SF[3] + P[1][11]*SF[1] + P[2][11]*SPP[0] - P[3][11]*SPP[2] - P[13][11]*SPP[4]) + SPP[6]*(P[4][12] + P[0][12]*SF[3] + P[1][12]*SF[1] + P[2][12]*SPP[0] - P[3][12]*SPP[2] - P[13][12]*SPP[4]);
	nextP[4][1] = P[4][1] + P[0][1]*SF[3] + P[1][1]*SF[1] + P[2][1]*SPP[0] - P[3][1]*SPP[2] - P[13][1]*SPP[4] + SF[6]*(P[4][0] + P[0][0]*SF[3] + P[1][0]*SF[1] + P[2][0]*SPP[0] - P[3][0]*SPP[2] - P[13][0]*SPP[4]) + SF[5]*(P[4][2] + P[0][2]*SF[3] + P[1][2]*SF[1] + P[2][2]*SPP[0] - P[3][2]*SPP[2] - P[13][2]*SPP[4]) + SF[9]*(P[4][3] + P[0][3]*SF[3] + P[1][3]*SF[1] + P[2][3]*SPP[0] - P[3][3]*SPP[2] - P[13][3]*SPP[4]) + SPP[6]*(P[4][11] + P[0][11]*SF[3] + P[1][11]*SF[1] + P[2][11]*SPP[0] - P[3][11]*SPP[2] - P[13][11]*SPP[4]) - SPP[7]*(P[4][12] + P[0][12]*SF[3] + P[1][12]*SF[1] + P[2][12]*SPP[0] - P[3][12]*SPP[2] - P[13][12]*SPP[4]) - (q0*(P[4][10] + P[0][10]*SF[3] + P[1][10]*SF[1] + P[2][10]*SPP[0] - P[3][10]*SPP[2] - P[13][10]*SPP[4]))/2;
	nextP[4][2] = P[4][2] + P[0][2]*SF[3] + P[1][2]*SF[1] + P[2][2]*SPP[0] - P[3][2]*SPP[2] - P[13][2]*SPP[4] + SF[4]*(P[4][0] + P[0][0]*SF[3] + P[1][0]*SF[1] + P[2][0]*SPP[0] - P[3][0]*SPP[2] - P[13][0]*SPP[4]) + SF[8]*(P[4][1] + P[0][1]*SF[3] + P[1][1]*SF[1] + P[2][1]*SPP[0] - P[3][1]*SPP[2] - P[13][1]*SPP[4]) + SF[6]*(P[4][3] + P[0][3]*SF[3] + P[1][3]*SF[1] + P[2][3]*SPP[0] - P[3][3]*SPP[2] - P[13][3]*SPP[4]) + SF[11]*(P[4][12] + P[0][12]*SF[3] + P[1][12]*SF[1] + P[2][12]*SPP[0] - P[3][12]*SPP[2] - P[13][12]*SPP[4]) - SPP[6]*(P[4][10] + P[0][10]*SF[3] + P[1][10]*SF[1] + P[2][10]*SPP[0] - P[3][10]*SPP[2] - P[13][10]*SPP[4]) - (q0*(P[4][11] + P[0][11]*SF[3] + P[1][11]*SF[1] + P[2][11]*SPP[0] - P[3][11]*SPP[2] - P[13][11]*SPP[4]))/2;
	nextP[4][3] = P[4][3] + P[0][3]*SF[3] + P[1][3]*SF[1] + P[2][3]*SPP[0] - P[3][3]*SPP[2] - P[13][3]*SPP[4] + SF[5]*(P[4][0] + P[0][0]*SF[3] + P[1][0]*SF[1] + P[2][0]*SPP[0] - P[3][0]*SPP[2] - P[13][0]*SPP[4]) + SF[4]*(P[4][1] + P[0][1]*SF[3] + P[1][1]*SF[1] + P[2][1]*SPP[0] - P[3][1]*SPP[2] - P[13][1]*SPP[4]) + SF[7]*(P[4][2] + P[0][2]*SF[3] + P[1][2]*SF[1] + P[2][2]*SPP[0] - P[3][2]*SPP[2] - P[13][2]*SPP[4]) - SF[11]*(P[4][11] + P[0][11]*SF[3] + P[1][11]*SF[1] + P[2][11]*SPP[0] - P[3][11]*SPP[2] - P[13][11]*SPP[4]) + SPP[7]*(P[4][10] + P[0][10]*SF[3] + P[1][10]*SF[1] + P[2][10]*SPP[0] - P[3][10]*SPP[2] - P[13][10]*SPP[4]) - (q0*(P[4][12] + P[0][12]*SF[3] + P[1][12]*SF[1] + P[2][12]*SPP[0] - P[3][12]*SPP[2] - P[13][12]*SPP[4]))/2;
	nextP[4][4] = P[4][4] + P[0][4]*SF[3] + P[1][4]*SF[1] + P[2][4]*SPP[0] - P[3][4]*SPP[2] - P[13][4]*SPP[4] + dvyCov*sq(SG[7] - 2*q0*q3) + dvzCov*sq(SG[6] + 2*q0*q2) + SF[3]*(P[4][0] + P[0][0]*SF[3] + P[1][0]*SF[1] + P[2][0]*SPP[0] - P[3][0]*SPP[2] - P[13][0]*SPP[4]) + SF[1]*(P[4][1] + P[0][1]*SF[3] + P[1][1]*SF[1] + P[2][1]*SPP[0] - P[3][1]*SPP[2] - P[13][1]*SPP[4]) + SPP[0]*(P[4][2] + P[0][2]*SF[3] + P[1][2]*SF[1] + P[2][2]*SPP[0] - P[3][2]*SPP[2] - P[13][2]*SPP[4]) - SPP[2]*(P[4][3] + P[0][3]*SF[3] + P[1][3]*SF[1] + P[2][3]*SPP[0] - P[3][3]*SPP[2] - P[13][3]*SPP[4]) - SPP[4]*(P[4][13] + P[0][13]*SF[3] + P[1][13]*SF[1] + P[2][13]*SPP[0] - P[3][13]*SPP[2] - P[13][13]*SPP[4]) + dvxCov*sq(SG[1] + SG[2] - SG[3] - SG[4]);
	nextP[4][5] = P[4][5] + SQ[2] + P[0][5]*SF[3] + P[1][5]*SF[1] + P[2][5]*SPP[0] - P[3][5]*SPP[2] - P[13][5]*SPP[4] + SF[2]*(P[4][0] + P[0][0]*SF[3] + P[1][0]*SF[1] + P[2][0]*SPP[0] - P[3][0]*SPP[2] - P[13][0]*SPP[4]) + SF[1]*(P[4][2] + P[0][2]*SF[3] + P[1][2]*SF[1] + P[2][2]*SPP[0] - P[3][2]*SPP[2] - P[13][2]*SPP[4]) + SF[3]*(P[4][3] + P[0][3]*SF[3] + P[1][3]*SF[1] + P[2][3]*SPP[0] - P[3][3]*SPP[2] - P[13][3]*SPP[4]) - SPP[0]*(P[4][1] + P[0][1]*SF[3] + P[1][1]*SF[1] + P[2][1]*SPP[0] - P[3][1]*SPP[2] - P[13][1]*SPP[4]) + SPP[3]*(P[4][13] + P[0][13]*SF[3] + P[1][13]*SF[1] + P[2][13]*SPP[0] - P[3][13]*SPP[2] - P[13][13]*SPP[4]);
	nextP[4][6] = P[4][6] + SQ[1] + P[0][6]*SF[3] + P[1][6]*SF[1] + P[2][6]*SPP[0] - P[3][6]*SPP[2] - P[13][6]*SPP[4] + SF[2]*(P[4][1] + P[0][1]*SF[3] + P[1][1]*SF[1] + P[2][1]*SPP[0] - P[3][1]*SPP[2] - P[13][1]*SPP[4]) + SF[1]*(P[4][3] + P[0][3]*SF[3] + P[1][3]*SF[1] + P[2][3]*SPP[0] - P[3][3]*SPP[2] - P[13][3]*SPP[4]) + SPP[0]*(P[4][0] + P[0][0]*SF[3] + P[1][0]*SF[1] + P[2][0]*SPP[0] - P[3][0]*SPP[2] - P[13][0]*SPP[4]) - SPP[1]*(P[4][2] + P[0][2]*SF[3] + P[1][2]*SF[1] + P[2][2]*SPP[0] - P[3][2]*SPP[2] - P[13][2]*SPP[4]) - (sq(q0) - sq(q1) - sq(q2) + sq(q3))*(P[4][13] + P[0][13]*SF[3] + P[1][13]*SF[1] + P[2][13]*SPP[0] - P[3][13]*SPP[2] - P[13][13]*SPP[4]);
	nextP[4][7] = P[4][7] + P[0][7]*SF[3] + P[1][7]*SF[1] + P[2][7]*SPP[0] - P[3][7]*SPP[2] - P[13][7]*SPP[4] + dt*(P[4][4] + P[0][4]*SF[3] + P[1][4]*SF[1] + P[2][4]*SPP[0] - P[3][4]*SPP[2] - P[13][4]*SPP[4]);
	nextP[4][8] = P[4][8] + P[0][8]*SF[3] + P[1][8]*SF[1] + P[2][8]*SPP[0] - P[3][8]*SPP[2] - P[13][8]*SPP[4] + dt*(P[4][5] + P[0][5]*SF[3] + P[1][5]*SF[1] + P[2][5]*SPP[0] - P[3][5]*SPP[2] - P[13][5]*SPP[4]);
	nextP[4][9] = P[4][9] + P[0][9]*SF[3] + P[1][9]*SF[1] + P[2][9]*SPP[0] - P[3][9]*SPP[2] - P[13][9]*SPP[4] + dt*(P[4][6] + P[0][6]*SF[3] + P[1][6]*SF[1] + P[2][6]*SPP[0] - P[3][6]*SPP[2] - P[13][6]*SPP[4]);
	nextP[4][10] = P[4][10] + P[0][10]*SF[3] + P[1][10]*SF[1] + P[2][10]*SPP[0] - P[3][10]*SPP[2] - P[13][10]*SPP[4];
	nextP[4][11] = P[4][11] + P[0][11]*SF[3] + P[1][11]*SF[1] + P[2][11]*SPP[0] - P[3][11]*SPP[2] - P[13][11]*SPP[4];
	nextP[4][12] = P[4][12] + P[0][12]*SF[3] + P[1][12]*SF[1] + P[2][12]*SPP[0] - P[3][12]*SPP[2] - P[13][12]*SPP[4];
	nextP[4][13] = P[4][13] + P[0][13]*SF[3] + P[1][13]*SF[1] + P[2][13]*SPP[0] - P[3][13]*SPP[2] - P[13][13]*SPP[4];
	nextP[4][14] = P[4][14] + P[0][14]*SF[3] + P[1][14]*SF[1] + P[2][14]*SPP[0] - P[3][14]*SPP[2] - P[13][14]*SPP[4];
	nextP[4][15] = P[4][15] + P[0][15]*SF[3] + P[1][15]*SF[1] + P[2][15]*SPP[0] - P[3][15]*SPP[2] - P[13][15]*SPP[4];
	nextP[4][16] = P[4][16] + P[0][16]*SF[3] + P[1][16]*SF[1] + P[2][16]*SPP[0] - P[3][16]*SPP[2] - P[13][16]*SPP[4];
	nextP[4][17] = P[4][17] + P[0][17]*SF[3] + P[1][17]*SF[1] + P[2][17]*SPP[0] - P[3][17]*SPP[2] - P[13][17]*SPP[4];
	nextP[4][18] = P[4][18] + P[0][18]*SF[3] + P[1][18]*SF[1] + P[2][18]*SPP[0] - P[3][18]*SPP[2] - P[13][18]*SPP[4];
	nextP[4][19] = P[4][19] + P[0][19]*SF[3] + P[1][19]*SF[1] + P[2][19]*SPP[0] - P[3][19]*SPP[2] - P[13][19]*SPP[4];
	nextP[4][20] = P[4][20] + P[0][20]*SF[3] + P[1][20]*SF[1] + P[2][20]*SPP[0] - P[3][20]*SPP[2] - P[13][20]*SPP[4];
	nextP[4][21] = P[4][21] + P[0][21]*SF[3] + P[1][21]*SF[1] + P[2][21]*SPP[0] - P[3][21]*SPP[2] - P[13][21]*SPP[4];
	nextP[5][0] = P[5][0] + P[0][0]*SF[2] + P[2][0]*SF[1] + P[3][0]*SF[3] - P[1][0]*SPP[0] + P[13][0]*SPP[3] + SF[7]*(P[5][1] + P[0][1]*SF[2] + P[2][1]*SF[1] + P[3][1]*SF[3] - P[1][1]*SPP[0] + P[13][1]*SPP[3]) + SF[9]*(P[5][2] + P[0][2]*SF[2] + P[2][2]*SF[1] + P[3][2]*SF[3] - P[1][2]*SPP[0] + P[13][2]*SPP[3]) + SF[8]*(P[5][3] + P[0][3]*SF[2] + P[2][3]*SF[1] + P[3][3]*SF[3] - P[1][3]*SPP[0] + P[13][3]*SPP[3]) + SF[11]*(P[5][10] + P[0][10]*SF[2] + P[2][10]*SF[1] + P[3][10]*SF[3] - P[1][10]*SPP[0] + P[13][10]*SPP[3]) + SPP[7]*(P[5][11] + P[0][11]*SF[2] + P[2][11]*SF[1] + P[3][11]*SF[3] - P[1][11]*SPP[0] + P[13][11]*SPP[3]) + SPP[6]*(P[5][12] + P[0][12]*SF[2] + P[2][12]*SF[1] + P[3][12]*SF[3] - P[1][12]*SPP[0] + P[13][12]*SPP[3]);
	nextP[5][1] = P[5][1] + P[0][1]*SF[2] + P[2][1]*SF[1] + P[3][1]*SF[3] - P[1][1]*SPP[0] + P[13][1]*SPP[3] + SF[6]*(P[5][0] + P[0][0]*SF[2] + P[2][0]*SF[1] + P[3][0]*SF[3] - P[1][0]*SPP[0] + P[13][0]*SPP[3]) + SF[5]*(P[5][2] + P[0][2]*SF[2] + P[2][2]*SF[1] + P[3][2]*SF[3] - P[1][2]*SPP[0] + P[13][2]*SPP[3]) + SF[9]*(P[5][3] + P[0][3]*SF[2] + P[2][3]*SF[1] + P[3][3]*SF[3] - P[1][3]*SPP[0] + P[13][3]*SPP[3]) + SPP[6]*(P[5][11] + P[0][11]*SF[2] + P[2][11]*SF[1] + P[3][11]*SF[3] - P[1][11]*SPP[0] + P[13][11]*SPP[3]) - SPP[7]*(P[5][12] + P[0][12]*SF[2] + P[2][12]*SF[1] + P[3][12]*SF[3] - P[1][12]*SPP[0] + P[13][12]*SPP[3]) - (q0*(P[5][10] + P[0][10]*SF[2] + P[2][10]*SF[1] + P[3][10]*SF[3] - P[1][10]*SPP[0] + P[13][10]*SPP[3]))/2;
	nextP[5][2] = P[5][2] + P[0][2]*SF[2] + P[2][2]*SF[1] + P[3][2]*SF[3] - P[1][2]*SPP[0] + P[13][2]*SPP[3] + SF[4]*(P[5][0] + P[0][0]*SF[2] + P[2][0]*SF[1] + P[3][0]*SF[3] - P[1][0]*SPP[0] + P[13][0]*SPP[3]) + SF[8]*(P[5][1] + P[0][1]*SF[2] + P[2][1]*SF[1] + P[3][1]*SF[3] - P[1][1]*SPP[0] + P[13][1]*SPP[3]) + SF[6]*(P[5][3] + P[0][3]*SF[2] + P[2][3]*SF[1] + P[3][3]*SF[3] - P[1][3]*SPP[0] + P[13][3]*SPP[3]) + SF[11]*(P[5][12] + P[0][12]*SF[2] + P[2][12]*SF[1] + P[3][12]*SF[3] - P[1][12]*SPP[0] + P[13][12]*SPP[3]) - SPP[6]*(P[5][10] + P[0][10]*SF[2] + P[2][10]*SF[1] + P[3][10]*SF[3] - P[1][10]*SPP[0] + P[13][10]*SPP[3]) - (q0*(P[5][11] + P[0][11]*SF[2] + P[2][11]*SF[1] + P[3][11]*SF[3] - P[1][11]*SPP[0] + P[13][11]*SPP[3]))/2;
	nextP[5][3] = P[5][3] + P[0][3]*SF[2] + P[2][3]*SF[1] + P[3][3]*SF[3] - P[1][3]*SPP[0] + P[13][3]*SPP[3] + SF[5]*(P[5][0] + P[0][0]*SF[2] + P[2][0]*SF[1] + P[3][0]*SF[3] - P[1][0]*SPP[0] + P[13][0]*SPP[3]) + SF[4]*(P[5][1] + P[0][1]*SF[2] + P[2][1]*SF[1] + P[3][1]*SF[3] - P[1][1]*SPP[0] + P[13][1]*SPP[3]) + SF[7]*(P[5][2] + P[0][2]*SF[2] + P[2][2]*SF[1] + P[3][2]*SF[3] - P[1][2]*SPP[0] + P[13][2]*SPP[3]) - SF[11]*(P[5][11] + P[0][11]*SF[2] + P[2][11]*SF[1] + P[3][11]*SF[3] - P[1][11]*SPP[0] + P[13][11]*SPP[3]) + SPP[7]*(P[5][10] + P[0][10]*SF[2] + P[2][10]*SF[1] + P[3][10]*SF[3] - P[1][10]*SPP[0] + P[13][10]*SPP[3]) - (q0*(P[5][12] + P[0][12]*SF[2] + P[2][12]*SF[1] + P[3][12]*SF[3] - P[1][12]*SPP[0] + P[13][12]*SPP[3]))/2;
	nextP[5][4] = P[5][4] + SQ[2] + P[0][4]*SF[2] + P[2][4]*SF[1] + P[3][4]*SF[3] - P[1][4]*SPP[0] + P[13][4]*SPP[3] + SF[3]*(P[5][0] + P[0][0]*SF[2] + P[2][0]*SF[1] + P[3][0]*SF[3] - P[1][0]*SPP[0] + P[13][0]*SPP[3]) + SF[1]*(P[5][1] + P[0][1]*SF[2] + P[2][1]*SF[1] + P[3][1]*SF[3] - P[1][1]*SPP[0] + P[13][1]*SPP[3]) + SPP[0]*(P[5][2] + P[0][2]*SF[2] + P[2][2]*SF[1] + P[3][2]*SF[3] - P[1][2]*SPP[0] + P[13][2]*SPP[3]) - SPP[2]*(P[5][3] + P[0][3]*SF[2] + P[2][3]*SF[1] + P[3][3]*SF[3] - P[1][3]*SPP[0] + P[13][3]*SPP[3]) - SPP[4]*(P[5][13] + P[0][13]*SF[2] + P[2][13]*SF[1] + P[3][13]*SF[3] - P[1][13]*SPP[0] + P[13][13]*SPP[3]);
	nextP[5][5] = P[5][5] + P[0][5]*SF[2] + P[2][5]*SF[1] + P[3][5]*SF[3] - P[1][5]*SPP[0] + P[13][5]*SPP[3] + dvxCov*sq(SG[7] + 2*q0*q3) + dvzCov*sq(SG[5] - 2*q0*q1) + SF[2]*(P[5][0] + P[0][0]*SF[2] + P[2][0]*SF[1] + P[3][0]*SF[3] - P[1][0]*SPP[0] + P[13][0]*SPP[3]) + SF[1]*(P[5][2] + P[0][2]*SF[2] + P[2][2]*SF[1] + P[3][2]*SF[3] - P[1][2]*SPP[0] + P[13][2]*SPP[3]) + SF[3]*(P[5][3] + P[0][3]*SF[2] + P[2][3]*SF[1] + P[3][3]*SF[3] - P[1][3]*SPP[0] + P[13][3]*SPP[3]) - SPP[0]*(P[5][1] + P[0][1]*SF[2] + P[2][1]*SF[1] + P[3][1]*SF[3] - P[1][1]*SPP[0] + P[13][1]*SPP[3]) + SPP[3]*(P[5][13] + P[0][13]*SF[2] + P[2][13]*SF[1] + P[3][13]*SF[3] - P[1][13]*SPP[0] + P[13][13]*SPP[3]) + dvyCov*sq(SG[1] - SG[2] + SG[3] - SG[4]);
	nextP[5][6] = P[5][6] + SQ[0] + P[0][6]*SF[2] + P[2][6]*SF[1] + P[3][6]*SF[3] - P[1][6]*SPP[0] + P[13][6]*SPP[3] + SF[2]*(P[5][1] + P[0][1]*SF[2] + P[2][1]*SF[1] + P[3][1]*SF[3] - P[1][1]*SPP[0] + P[13][1]*SPP[3]) + SF[1]*(P[5][3] + P[0][3]*SF[2] + P[2][3]*SF[1] + P[3][3]*SF[3] - P[1][3]*SPP[0] + P[13][3]*SPP[3]) + SPP[0]*(P[5][0] + P[0][0]*SF[2] + P[2][0]*SF[1] + P[3][0]*SF[3] - P[1][0]*SPP[0] + P[13][0]*SPP[3]) - SPP[1]*(P[5][2] + P[0][2]*SF[2] + P[2][2]*SF[1] + P[3][2]*SF[3] - P[1][2]*SPP[0] + P[13][2]*SPP[3]) - (sq(q0) - sq(q1) - sq(q2) + sq(q3))*(P[5][13] + P[0][13]*SF[2] + P[2][13]*SF[1] + P[3][13]*SF[3] - P[1][13]*SPP[0] + P[13][13]*SPP[3]);
	nextP[5][7] = P[5][7] + P[0][7]*SF[2] + P[2][7]*SF[1] + P[3][7]*SF[3] - P[1][7]*SPP[0] + P[13][7]*SPP[3] + dt*(P[5][4] + P[0][4]*SF[2] + P[2][4]*SF[1] + P[3][4]*SF[3] - P[1][4]*SPP[0] + P[13][4]*SPP[3]);
	nextP[5][8] = P[5][8] + P[0][8]*SF[2] + P[2][8]*SF[1] + P[3][8]*SF[3] - P[1][8]*SPP[0] + P[13][8]*SPP[3] + dt*(P[5][5] + P[0][5]*SF[2] + P[2][5]*SF[1] + P[3][5]*SF[3] - P[1][5]*SPP[0] + P[13][5]*SPP[3]);
	nextP[5][9] = P[5][9] + P[0][9]*SF[2] + P[2][9]*SF[1] + P[3][9]*SF[3] - P[1][9]*SPP[0] + P[13][9]*SPP[3] + dt*(P[5][6] + P[0][6]*SF[2] + P[2][6]*SF[1] + P[3][6]*SF[3] - P[1][6]*SPP[0] + P[13][6]*SPP[3]);
	nextP[5][10] = P[5][10] + P[0][10]*SF[2] + P[2][10]*SF[1] + P[3][10]*SF[3] - P[1][10]*SPP[0] + P[13][10]*SPP[3];
	nextP[5][11] = P[5][11] + P[0][11]*SF[2] + P[2][11]*SF[1] + P[3][11]*SF[3] - P[1][11]*SPP[0] + P[13][11]*SPP[3];
	nextP[5][12] = P[5][12] + P[0][12]*SF[2] + P[2][12]*SF[1] + P[3][12]*SF[3] - P[1][12]*SPP[0] + P[13][12]*SPP[3];
	nextP[5][13] = P[5][13] + P[0][13]*SF[2] + P[2][13]*SF[1] + P[3][13]*SF[3] - P[1][13]*SPP[0] + P[13][13]*SPP[3];
	nextP[5][14] = P[5][14] + P[0][14]*SF[2] + P[2][14]*SF[1] + P[3][14]*SF[3] - P[1][14]*SPP[0] + P[13][14]*SPP[3];
	nextP[5][15] = P[5][15] + P[0][15]*SF[2] + P[2][15]*SF[1] + P[3][15]*SF[3] - P[1][15]*SPP[0] + P[13][15]*SPP[3];
	nextP[5][16] = P[5][16] + P[0][16]*SF[2] + P[2][16]*SF[1] + P[3][16]*SF[3] - P[1][16]*SPP[0] + P[13][16]*SPP[3];
	nextP[5][17] = P[5][17] + P[0][17]*SF[2] + P[2][17]*SF[1] + P[3][17]*SF[3] - P[1][17]*SPP[0] + P[13][17]*SPP[3];
	nextP[5][18] = P[5][18] + P[0][18]*SF[2] + P[2][18]*SF[1] + P[3][18]*SF[3] - P[1][18]*SPP[0] + P[13][18]*SPP[3];
	nextP[5][19] = P[5][19] + P[0][19]*SF[2] + P[2][19]*SF[1] + P[3][19]*SF[3] - P[1][19]*SPP[0] + P[13][19]*SPP[3];
	nextP[5][20] = P[5][20] + P[0][20]*SF[2] + P[2][20]*SF[1] + P[3][20]*SF[3] - P[1][20]*SPP[0] + P[13][20]*SPP[3];
	nextP[5][21] = P[5][21] + P[0][21]*SF[2] + P[2][21]*SF[1] + P[3][21]*SF[3] - P[1][21]*SPP[0] + P[13][21]*SPP[3];
	nextP[6][0] = P[6][0] + P[1][0]*SF[2] + P[3][0]*SF[1] + P[0][0]*SPP[0] - P[2][0]*SPP[1] - P[13][0]*(sq(q0) - sq(q1) - sq(q2) + sq(q3)) + SF[7]*(P[6][1] + P[1][1]*SF[2] + P[3][1]*SF[1] + P[0][1]*SPP[0] - P[2][1]*SPP[1] - P[13][1]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) + SF[9]*(P[6][2] + P[1][2]*SF[2] + P[3][2]*SF[1] + P[0][2]*SPP[0] - P[2][2]*SPP[1] - P[13][2]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) + SF[8]*(P[6][3] + P[1][3]*SF[2] + P[3][3]*SF[1] + P[0][3]*SPP[0] - P[2][3]*SPP[1] - P[13][3]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) + SF[11]*(P[6][10] + P[1][10]*SF[2] + P[3][10]*SF[1] + P[0][10]*SPP[0] - P[2][10]*SPP[1] - P[13][10]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) + SPP[7]*(P[6][11] + P[1][11]*SF[2] + P[3][11]*SF[1] + P[0][11]*SPP[0] - P[2][11]*SPP[1] - P[13][11]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) + SPP[6]*(P[6][12] + P[1][12]*SF[2] + P[3][12]*SF[1] + P[0][12]*SPP[0] - P[2][12]*SPP[1] - P[13][12]*(sq(q0) - sq(q1) - sq(q2) + sq(q3)));
	nextP[6][1] = P[6][1] + P[1][1]*SF[2] + P[3][1]*SF[1] + P[0][1]*SPP[0] - P[2][1]*SPP[1] - P[13][1]*(sq(q0) - sq(q1) - sq(q2) + sq(q3)) + SF[6]*(P[6][0] + P[1][0]*SF[2] + P[3][0]*SF[1] + P[0][0]*SPP[0] - P[2][0]*SPP[1] - P[13][0]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) + SF[5]*(P[6][2] + P[1][2]*SF[2] + P[3][2]*SF[1] + P[0][2]*SPP[0] - P[2][2]*SPP[1] - P[13][2]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) + SF[9]*(P[6][3] + P[1][3]*SF[2] + P[3][3]*SF[1] + P[0][3]*SPP[0] - P[2][3]*SPP[1] - P[13][3]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) + SPP[6]*(P[6][11] + P[1][11]*SF[2] + P[3][11]*SF[1] + P[0][11]*SPP[0] - P[2][11]*SPP[1] - P[13][11]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) - SPP[7]*(P[6][12] + P[1][12]*SF[2] + P[3][12]*SF[1] + P[0][12]*SPP[0] - P[2][12]*SPP[1] - P[13][12]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) - (q0*(P[6][10] + P[1][10]*SF[2] + P[3][10]*SF[1] + P[0][10]*SPP[0] - P[2][10]*SPP[1] - P[13][10]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))))/2;
	nextP[6][2] = P[6][2] + P[1][2]*SF[2] + P[3][2]*SF[1] + P[0][2]*SPP[0] - P[2][2]*SPP[1] - P[13][2]*(sq(q0) - sq(q1) - sq(q2) + sq(q3)) + SF[4]*(P[6][0] + P[1][0]*SF[2] + P[3][0]*SF[1] + P[0][0]*SPP[0] - P[2][0]*SPP[1] - P[13][0]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) + SF[8]*(P[6][1] + P[1][1]*SF[2] + P[3][1]*SF[1] + P[0][1]*SPP[0] - P[2][1]*SPP[1] - P[13][1]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) + SF[6]*(P[6][3] + P[1][3]*SF[2] + P[3][3]*SF[1] + P[0][3]*SPP[0] - P[2][3]*SPP[1] - P[13][3]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) + SF[11]*(P[6][12] + P[1][12]*SF[2] + P[3][12]*SF[1] + P[0][12]*SPP[0] - P[2][12]*SPP[1] - P[13][12]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) - SPP[6]*(P[6][10] + P[1][10]*SF[2] + P[3][10]*SF[1] + P[0][10]*SPP[0] - P[2][10]*SPP[1] - P[13][10]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) - (q0*(P[6][11] + P[1][11]*SF[2] + P[3][11]*SF[1] + P[0][11]*SPP[0] - P[2][11]*SPP[1] - P[13][11]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))))/2;
	nextP[6][3] = P[6][3] + P[1][3]*SF[2] + P[3][3]*SF[1] + P[0][3]*SPP[0] - P[2][3]*SPP[1] - P[13][3]*(sq(q0) - sq(q1) - sq(q2) + sq(q3)) + SF[5]*(P[6][0] + P[1][0]*SF[2] + P[3][0]*SF[1] + P[0][0]*SPP[0] - P[2][0]*SPP[1] - P[13][0]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) + SF[4]*(P[6][1] + P[1][1]*SF[2] + P[3][1]*SF[1] + P[0][1]*SPP[0] - P[2][1]*SPP[1] - P[13][1]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) + SF[7]*(P[6][2] + P[1][2]*SF[2] + P[3][2]*SF[1] + P[0][2]*SPP[0] - P[2][2]*SPP[1] - P[13][2]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) - SF[11]*(P[6][11] + P[1][11]*SF[2] + P[3][11]*SF[1] + P[0][11]*SPP[0] - P[2][11]*SPP[1] - P[13][11]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) + SPP[7]*(P[6][10] + P[1][10]*SF[2] + P[3][10]*SF[1] + P[0][10]*SPP[0] - P[2][10]*SPP[1] - P[13][10]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) - (q0*(P[6][12] + P[1][12]*SF[2] + P[3][12]*SF[1] + P[0][12]*SPP[0] - P[2][12]*SPP[1] - P[13][12]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))))/2;
	nextP[6][4] = P[6][4] + SQ[1] + P[1][4]*SF[2] + P[3][4]*SF[1] + P[0][4]*SPP[0] - P[2][4]*SPP[1] - P[13][4]*(sq(q0) - sq(q1) - sq(q2) + sq(q3)) + SF[3]*(P[6][0] + P[1][0]*SF[2] + P[3][0]*SF[1] + P[0][0]*SPP[0] - P[2][0]*SPP[1] - P[13][0]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) + SF[1]*(P[6][1] + P[1][1]*SF[2] + P[3][1]*SF[1] + P[0][1]*SPP[0] - P[2][1]*SPP[1] - P[13][1]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) + SPP[0]*(P[6][2] + P[1][2]*SF[2] + P[3][2]*SF[1] + P[0][2]*SPP[0] - P[2][2]*SPP[1] - P[13][2]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) - SPP[2]*(P[6][3] + P[1][3]*SF[2] + P[3][3]*SF[1] + P[0][3]*SPP[0] - P[2][3]*SPP[1] - P[13][3]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) - SPP[4]*(P[6][13] + P[1][13]*SF[2] + P[3][13]*SF[1] + P[0][13]*SPP[0] - P[2][13]*SPP[1] - P[13][13]*(sq(q0) - sq(q1) - sq(q2) + sq(q3)));
	nextP[6][5] = P[6][5] + SQ[0] + P[1][5]*SF[2] + P[3][5]*SF[1] + P[0][5]*SPP[0] - P[2][5]*SPP[1] - P[13][5]*(sq(q0) - sq(q1) - sq(q2) + sq(q3)) + SF[2]*(P[6][0] + P[1][0]*SF[2] + P[3][0]*SF[1] + P[0][0]*SPP[0] - P[2][0]*SPP[1] - P[13][0]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) + SF[1]*(P[6][2] + P[1][2]*SF[2] + P[3][2]*SF[1] + P[0][2]*SPP[0] - P[2][2]*SPP[1] - P[13][2]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) + SF[3]*(P[6][3] + P[1][3]*SF[2] + P[3][3]*SF[1] + P[0][3]*SPP[0] - P[2][3]*SPP[1] - P[13][3]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) - SPP[0]*(P[6][1] + P[1][1]*SF[2] + P[3][1]*SF[1] + P[0][1]*SPP[0] - P[2][1]*SPP[1] - P[13][1]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) + SPP[3]*(P[6][13] + P[1][13]*SF[2] + P[3][13]*SF[1] + P[0][13]*SPP[0] - P[2][13]*SPP[1] - P[13][13]*(sq(q0) - sq(q1) - sq(q2) + sq(q3)));
	nextP[6][6] = P[6][6] + P[1][6]*SF[2] + P[3][6]*SF[1] + P[0][6]*SPP[0] - P[2][6]*SPP[1] - P[13][6]*(sq(q0) - sq(q1) - sq(q2) + sq(q3)) + dvxCov*sq(SG[6] - 2*q0*q2) + dvyCov*sq(SG[5] + 2*q0*q1) - SPP[5]*(P[6][13] + P[1][13]*SF[2] + P[3][13]*SF[1] + P[0][13]*SPP[0] - P[2][13]*SPP[1] - P[13][13]*SPP[5]) + SF[2]*(P[6][1] + P[1][1]*SF[2] + P[3][1]*SF[1] + P[0][1]*SPP[0] - P[2][1]*SPP[1] - P[13][1]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) + SF[1]*(P[6][3] + P[1][3]*SF[2] + P[3][3]*SF[1] + P[0][3]*SPP[0] - P[2][3]*SPP[1] - P[13][3]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) + SPP[0]*(P[6][0] + P[1][0]*SF[2] + P[3][0]*SF[1] + P[0][0]*SPP[0] - P[2][0]*SPP[1] - P[13][0]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) - SPP[1]*(P[6][2] + P[1][2]*SF[2] + P[3][2]*SF[1] + P[0][2]*SPP[0] - P[2][2]*SPP[1] - P[13][2]*(sq(q0) - sq(q1) - sq(q2) + sq(q3))) + dvzCov*sq(SG[1] - SG[2] - SG[3] + SG[4]);
	nextP[6][7] = P[6][7] + P[1][7]*SF[2] + P[3][7]*SF[1] + P[0][7]*SPP[0] - P[2][7]*SPP[1] - P[13][7]*SPP[5] + dt*(P[6][4] + P[1][4]*SF[2] + P[3][4]*SF[1] + P[0][4]*SPP[0] - P[2][4]*SPP[1] - P[13][4]*SPP[5]);
	nextP[6][8] = P[6][8] + P[1][8]*SF[2] + P[3][8]*SF[1] + P[0][8]*SPP[0] - P[2][8]*SPP[1] - P[13][8]*SPP[5] + dt*(P[6][5] + P[1][5]*SF[2] + P[3][5]*SF[1] + P[0][5]*SPP[0] - P[2][5]*SPP[1] - P[13][5]*SPP[5]);
	nextP[6][9] = P[6][9] + P[1][9]*SF[2] + P[3][9]*SF[1] + P[0][9]*SPP[0] - P[2][9]*SPP[1] - P[13][9]*SPP[5] + dt*(P[6][6] + P[1][6]*SF[2] + P[3][6]*SF[1] + P[0][6]*SPP[0] - P[2][6]*SPP[1] - P[13][6]*SPP[5]);
	nextP[6][10] = P[6][10] + P[1][10]*SF[2] + P[3][10]*SF[1] + P[0][10]*SPP[0] - P[2][10]*SPP[1] - P[13][10]*SPP[5];
	nextP[6][11] = P[6][11] + P[1][11]*SF[2] + P[3][11]*SF[1] + P[0][11]*SPP[0] - P[2][11]*SPP[1] - P[13][11]*SPP[5];
	nextP[6][12] = P[6][12] + P[1][12]*SF[2] + P[3][12]*SF[1] + P[0][12]*SPP[0] - P[2][12]*SPP[1] - P[13][12]*SPP[5];
	nextP[6][13] = P[6][13] + P[1][13]*SF[2] + P[3][13]*SF[1] + P[0][13]*SPP[0] - P[2][13]*SPP[1] - P[13][13]*SPP[5];
	nextP[6][14] = P[6][14] + P[1][14]*SF[2] + P[3][14]*SF[1] + P[0][14]*SPP[0] - P[2][14]*SPP[1] - P[13][14]*SPP[5];
	nextP[6][15] = P[6][15] + P[1][15]*SF[2] + P[3][15]*SF[1] + P[0][15]*SPP[0] - P[2][15]*SPP[1] - P[13][15]*SPP[5];
	nextP[6][16] = P[6][16] + P[1][16]*SF[2] + P[3][16]*SF[1] + P[0][16]*SPP[0] - P[2][16]*SPP[1] - P[13][16]*SPP[5];
	nextP[6][17] = P[6][17] + P[1][17]*SF[2] + P[3][17]*SF[1] + P[0][17]*SPP[0] - P[2][17]*SPP[1] - P[13][17]*SPP[5];
	nextP[6][18] = P[6][18] + P[1][18]*SF[2] + P[3][18]*SF[1] + P[0][18]*SPP[0] - P[2][18]*SPP[1] - P[13][18]*SPP[5];
	nextP[6][19] = P[6][19] + P[1][19]*SF[2] + P[3][19]*SF[1] + P[0][19]*SPP[0] - P[2][19]*SPP[1] - P[13][19]*SPP[5];
	nextP[6][20] = P[6][20] + P[1][20]*SF[2] + P[3][20]*SF[1] + P[0][20]*SPP[0] - P[2][20]*SPP[1] - P[13][20]*SPP[5];
	nextP[6][21] = P[6][21] + P[1][21]*SF[2] + P[3][21]*SF[1] + P[0][21]*SPP[0] - P[2][21]*SPP[1] - P[13][21]*SPP[5];
	nextP[7][0] = P[7][0] + P[4][0]*dt + SF[7]*(P[7][1] + P[4][1]*dt) + SF[9]*(P[7][2] + P[4][2]*dt) + SF[8]*(P[7][3] + P[4][3]*dt) + SF[11]*(P[7][10] + P[4][10]*dt) + SPP[7]*(P[7][11] + P[4][11]*dt) + SPP[6]*(P[7][12] + P[4][12]*dt);
	nextP[7][1] = P[7][1] + P[4][1]*dt + SF[6]*(P[7][0] + P[4][0]*dt) + SF[5]*(P[7][2] + P[4][2]*dt) + SF[9]*(P[7][3] + P[4][3]*dt) + SPP[6]*(P[7][11] + P[4][11]*dt) - SPP[7]*(P[7][12] + P[4][12]*dt) - (q0*(P[7][10] + P[4][10]*dt))/2;
	nextP[7][2] = P[7][2] + P[4][2]*dt + SF[4]*(P[7][0] + P[4][0]*dt) + SF[8]*(P[7][1] + P[4][1]*dt) + SF[6]*(P[7][3] + P[4][3]*dt) + SF[11]*(P[7][12] + P[4][12]*dt) - SPP[6]*(P[7][10] + P[4][10]*dt) - (q0*(P[7][11] + P[4][11]*dt))/2;
	nextP[7][3] = P[7][3] + P[4][3]*dt + SF[5]*(P[7][0] + P[4][0]*dt) + SF[4]*(P[7][1] + P[4][1]*dt) + SF[7]*(P[7][2] + P[4][2]*dt) - SF[11]*(P[7][11] + P[4][11]*dt) + SPP[7]*(P[7][10] + P[4][10]*dt) - (q0*(P[7][12] + P[4][12]*dt))/2;
	nextP[7][4] = P[7][4] + P[4][4]*dt + SF[1]*(P[7][1] + P[4][1]*dt) + SF[3]*(P[7][0] + P[4][0]*dt) + SPP[0]*(P[7][2] + P[4][2]*dt) - SPP[2]*(P[7][3] + P[4][3]*dt) - SPP[4]*(P[7][13] + P[4][13]*dt);
	nextP[7][5] = P[7][5] + P[4][5]*dt + SF[2]*(P[7][0] + P[4][0]*dt) + SF[1]*(P[7][2] + P[4][2]*dt) + SF[3]*(P[7][3] + P[4][3]*dt) - SPP[0]*(P[7][1] + P[4][1]*dt) + SPP[3]*(P[7][13] + P[4][13]*dt);
	nextP[7][6] = P[7][6] + P[4][6]*dt + SF[2]*(P[7][1] + P[4][1]*dt) + SF[1]*(P[7][3] + P[4][3]*dt) + SPP[0]*(P[7][0] + P[4][0]*dt) - SPP[1]*(P[7][2] + P[4][2]*dt) - SPP[5]*(P[7][13] + P[4][13]*dt);
	nextP[7][7] = P[7][7] + P[4][7]*dt + dt*(P[7][4] + P[4][4]*dt);
	nextP[7][8] = P[7][8] + P[4][8]*dt + dt*(P[7][5] + P[4][5]*dt);
	nextP[7][9] = P[7][9] + P[4][9]*dt + dt*(P[7][6] + P[4][6]*dt);
	nextP[7][10] = P[7][10] + P[4][10]*dt;
	nextP[7][11] = P[7][11] + P[4][11]*dt;
	nextP[7][12] = P[7][12] + P[4][12]*dt;
	nextP[7][13] = P[7][13] + P[4][13]*dt;
	nextP[7][14] = P[7][14] + P[4][14]*dt;
	nextP[7][15] = P[7][15] + P[4][15]*dt;
	nextP[7][16] = P[7][16] + P[4][16]*dt;
	nextP[7][17] = P[7][17] + P[4][17]*dt;
	nextP[7][18] = P[7][18] + P[4][18]*dt;
	nextP[7][19] = P[7][19] + P[4][19]*dt;
	nextP[7][20] = P[7][20] + P[4][20]*dt;
	nextP[7][21] = P[7][21] + P[4][21]*dt;
	nextP[8][0] = P[8][0] + P[5][0]*dt + SF[7]*(P[8][1] + P[5][1]*dt) + SF[9]*(P[8][2] + P[5][2]*dt) + SF[8]*(P[8][3] + P[5][3]*dt) + SF[11]*(P[8][10] + P[5][10]*dt) + SPP[7]*(P[8][11] + P[5][11]*dt) + SPP[6]*(P[8][12] + P[5][12]*dt);
	nextP[8][1] = P[8][1] + P[5][1]*dt + SF[6]*(P[8][0] + P[5][0]*dt) + SF[5]*(P[8][2] + P[5][2]*dt) + SF[9]*(P[8][3] + P[5][3]*dt) + SPP[6]*(P[8][11] + P[5][11]*dt) - SPP[7]*(P[8][12] + P[5][12]*dt) - (q0*(P[8][10] + P[5][10]*dt))/2;
	nextP[8][2] = P[8][2] + P[5][2]*dt + SF[4]*(P[8][0] + P[5][0]*dt) + SF[8]*(P[8][1] + P[5][1]*dt) + SF[6]*(P[8][3] + P[5][3]*dt) + SF[11]*(P[8][12] + P[5][12]*dt) - SPP[6]*(P[8][10] + P[5][10]*dt) - (q0*(P[8][11] + P[5][11]*dt))/2;
	nextP[8][3] = P[8][3] + P[5][3]*dt + SF[5]*(P[8][0] + P[5][0]*dt) + SF[4]*(P[8][1] + P[5][1]*dt) + SF[7]*(P[8][2] + P[5][2]*dt) - SF[11]*(P[8][11] + P[5][11]*dt) + SPP[7]*(P[8][10] + P[5][10]*dt) - (q0*(P[8][12] + P[5][12]*dt))/2;
	nextP[8][4] = P[8][4] + P[5][4]*dt + SF[1]*(P[8][1] + P[5][1]*dt) + SF[3]*(P[8][0] + P[5][0]*dt) + SPP[0]*(P[8][2] + P[5][2]*dt) - SPP[2]*(P[8][3] + P[5][3]*dt) - SPP[4]*(P[8][13] + P[5][13]*dt);
	nextP[8][5] = P[8][5] + P[5][5]*dt + SF[2]*(P[8][0] + P[5][0]*dt) + SF[1]*(P[8][2] + P[5][2]*dt) + SF[3]*(P[8][3] + P[5][3]*dt) - SPP[0]*(P[8][1] + P[5][1]*dt) + SPP[3]*(P[8][13] + P[5][13]*dt);
	nextP[8][6] = P[8][6] + P[5][6]*dt + SF[2]*(P[8][1] + P[5][1]*dt) + SF[1]*(P[8][3] + P[5][3]*dt) + SPP[0]*(P[8][0] + P[5][0]*dt) - SPP[1]*(P[8][2] + P[5][2]*dt) - SPP[5]*(P[8][13] + P[5][13]*dt);
	nextP[8][7] = P[8][7] + P[5][7]*dt + dt*(P[8][4] + P[5][4]*dt);
	nextP[8][8] = P[8][8] + P[5][8]*dt + dt*(P[8][5] + P[5][5]*dt);
	nextP[8][9] = P[8][9] + P[5][9]*dt + dt*(P[8][6] + P[5][6]*dt);
	nextP[8][10] = P[8][10] + P[5][10]*dt;
	nextP[8][11] = P[8][11] + P[5][11]*dt;
	nextP[8][12] = P[8][12] + P[5][12]*dt;
	nextP[8][13] = P[8][13] + P[5][13]*dt;
	nextP[8][14] = P[8][14] + P[5][14]*dt;
	nextP[8][15] = P[8][15] + P[5][15]*dt;
	nextP[8][16] = P[8][16] + P[5][16]*dt;
	nextP[8][17] = P[8][17] + P[5][17]*dt;
	nextP[8][18] = P[8][18] + P[5][18]*dt;
	nextP[8][19] = P[8][19] + P[5][19]*dt;
	nextP[8][20] = P[8][20] + P[5][20]*dt;
	nextP[8][21] = P[8][21] + P[5][21]*dt;
	nextP[9][0] = P[9][0] + P[6][0]*dt + SF[7]*(P[9][1] + P[6][1]*dt) + SF[9]*(P[9][2] + P[6][2]*dt) + SF[8]*(P[9][3] + P[6][3]*dt) + SF[11]*(P[9][10] + P[6][10]*dt) + SPP[7]*(P[9][11] + P[6][11]*dt) + SPP[6]*(P[9][12] + P[6][12]*dt);
	nextP[9][1] = P[9][1] + P[6][1]*dt + SF[6]*(P[9][0] + P[6][0]*dt) + SF[5]*(P[9][2] + P[6][2]*dt) + SF[9]*(P[9][3] + P[6][3]*dt) + SPP[6]*(P[9][11] + P[6][11]*dt) - SPP[7]*(P[9][12] + P[6][12]*dt) - (q0*(P[9][10] + P[6][10]*dt))/2;
	nextP[9][2] = P[9][2] + P[6][2]*dt + SF[4]*(P[9][0] + P[6][0]*dt) + SF[8]*(P[9][1] + P[6][1]*dt) + SF[6]*(P[9][3] + P[6][3]*dt) + SF[11]*(P[9][12] + P[6][12]*dt) - SPP[6]*(P[9][10] + P[6][10]*dt) - (q0*(P[9][11] + P[6][11]*dt))/2;
	nextP[9][3] = P[9][3] + P[6][3]*dt + SF[5]*(P[9][0] + P[6][0]*dt) + SF[4]*(P[9][1] + P[6][1]*dt) + SF[7]*(P[9][2] + P[6][2]*dt) - SF[11]*(P[9][11] + P[6][11]*dt) + SPP[7]*(P[9][10] + P[6][10]*dt) - (q0*(P[9][12] + P[6][12]*dt))/2;
	nextP[9][4] = P[9][4] + P[6][4]*dt + SF[1]*(P[9][1] + P[6][1]*dt) + SF[3]*(P[9][0] + P[6][0]*dt) + SPP[0]*(P[9][2] + P[6][2]*dt) - SPP[2]*(P[9][3] + P[6][3]*dt) - SPP[4]*(P[9][13] + P[6][13]*dt);
	nextP[9][5] = P[9][5] + P[6][5]*dt + SF[2]*(P[9][0] + P[6][0]*dt) + SF[1]*(P[9][2] + P[6][2]*dt) + SF[3]*(P[9][3] + P[6][3]*dt) - SPP[0]*(P[9][1] + P[6][1]*dt) + SPP[3]*(P[9][13] + P[6][13]*dt);
	nextP[9][6] = P[9][6] + P[6][6]*dt + SF[2]*(P[9][1] + P[6][1]*dt) + SF[1]*(P[9][3] + P[6][3]*dt) + SPP[0]*(P[9][0] + P[6][0]*dt) - SPP[1]*(P[9][2] + P[6][2]*dt) - SPP[5]*(P[9][13] + P[6][13]*dt);
	nextP[9][7] = P[9][7] + P[6][7]*dt + dt*(P[9][4] + P[6][4]*dt);
	nextP[9][8] = P[9][8] + P[6][8]*dt + dt*(P[9][5] + P[6][5]*dt);
	nextP[9][9] = P[9][9] + P[6][9]*dt + dt*(P[9][6] + P[6][6]*dt);
	nextP[9][10] = P[9][10] + P[6][10]*dt;
	nextP[9][11] = P[9][11] + P[6][11]*dt;
	nextP[9][12] = P[9][12] + P[6][12]*dt;
	nextP[9][13] = P[9][13] + P[6][13]*dt;
	nextP[9][14] = P[9][14] + P[6][14]*dt;
	nextP[9][15] = P[9][15] + P[6][15]*dt;
	nextP[9][16] = P[9][16] + P[6][16]*dt;
	nextP[9][17] = P[9][17] + P[6][17]*dt;
	nextP[9][18] = P[9][18] + P[6][18]*dt;
	nextP[9][19] = P[9][19] + P[6][19]*dt;
	nextP[9][20] = P[9][20] + P[6][20]*dt;
	nextP[9][21] = P[9][21] + P[6][21]*dt;
	nextP[10][0] = P[10][0] + P[10][1]*SF[7] + P[10][2]*SF[9] + P[10][3]*SF[8] + P[10][10]*SF[11] + P[10][11]*SPP[7] + P[10][12]*SPP[6];
	nextP[10][1] = P[10][1] + P[10][0]*SF[6] + P[10][2]*SF[5] + P[10][3]*SF[9] + P[10][11]*SPP[6] - P[10][12]*SPP[7] - (P[10][10]*q0)/2;
	nextP[10][2] = P[10][2] + P[10][0]*SF[4] + P[10][1]*SF[8] + P[10][3]*SF[6] + P[10][12]*SF[11] - P[10][10]*SPP[6] - (P[10][11]*q0)/2;
	nextP[10][3] = P[10][3] + P[10][0]*SF[5] + P[10][1]*SF[4] + P[10][2]*SF[7] - P[10][11]*SF[11] + P[10][10]*SPP[7] - (P[10][12]*q0)/2;
	nextP[10][4] = P[10][4] + P[10][1]*SF[1] + P[10][0]*SF[3] + P[10][2]*SPP[0] - P[10][3]*SPP[2] - P[10][13]*SPP[4];
	nextP[10][5] = P[10][5] + P[10][0]*SF[2] + P[10][2]*SF[1] + P[10][3]*SF[3] - P[10][1]*SPP[0] + P[10][13]*SPP[3];
	nextP[10][6] = P[10][6] + P[10][1]*SF[2] + P[10][3]*SF[1] + P[10][0]*SPP[0] - P[10][2]*SPP[1] - P[10][13]*SPP[5];
	nextP[10][7] = P[10][7] + P[10][4]*dt;
	nextP[10][8] = P[10][8] + P[10][5]*dt;
	nextP[10][9] = P[10][9] + P[10][6]*dt;
	nextP[10][10] = P[10][10];
	nextP[10][11] = P[10][11];
	nextP[10][12] = P[10][12];
	nextP[10][13] = P[10][13];
	nextP[10][14] = P[10][14];
	nextP[10][15] = P[10][15];
	nextP[10][16] = P[10][16];
	nextP[10][17] = P[10][17];
	nextP[10][18] = P[10][18];
	nextP[10][19] = P[10][19];
	nextP[10][20] = P[10][20];
	nextP[10][21] = P[10][21];
	nextP[11][0] = P[11][0] + P[11][1]*SF[7] + P[11][2]*SF[9] + P[11][3]*SF[8] + P[11][10]*SF[11] + P[11][11]*SPP[7] + P[11][12]*SPP[6];
	nextP[11][1] = P[11][1] + P[11][0]*SF[6] + P[11][2]*SF[5] + P[11][3]*SF[9] + P[11][11]*SPP[6] - P[11][12]*SPP[7] - (P[11][10]*q0)/2;
	nextP[11][2] = P[11][2] + P[11][0]*SF[4] + P[11][1]*SF[8] + P[11][3]*SF[6] + P[11][12]*SF[11] - P[11][10]*SPP[6] - (P[11][11]*q0)/2;
	nextP[11][3] = P[11][3] + P[11][0]*SF[5] + P[11][1]*SF[4] + P[11][2]*SF[7] - P[11][11]*SF[11] + P[11][10]*SPP[7] - (P[11][12]*q0)/2;
	nextP[11][4] = P[11][4] + P[11][1]*SF[1] + P[11][0]*SF[3] + P[11][2]*SPP[0] - P[11][3]*SPP[2] - P[11][13]*SPP[4];
	nextP[11][5] = P[11][5] + P[11][0]*SF[2] + P[11][2]*SF[1] + P[11][3]*SF[3] - P[11][1]*SPP[0] + P[11][13]*SPP[3];
	nextP[11][6] = P[11][6] + P[11][1]*SF[2] + P[11][3]*SF[1] + P[11][0]*SPP[0] - P[11][2]*SPP[1] - P[11][13]*SPP[5];
	nextP[11][7] = P[11][7] + P[11][4]*dt;
	nextP[11][8] = P[11][8] + P[11][5]*dt;
	nextP[11][9] = P[11][9] + P[11][6]*dt;
	nextP[11][10] = P[11][10];
	nextP[11][11] = P[11][11];
	nextP[11][12] = P[11][12];
	nextP[11][13] = P[11][13];
	nextP[11][14] = P[11][14];
	nextP[11][15] = P[11][15];
	nextP[11][16] = P[11][16];
	nextP[11][17] = P[11][17];
	nextP[11][18] = P[11][18];
	nextP[11][19] = P[11][19];
	nextP[11][20] = P[11][20];
	nextP[11][21] = P[11][21];
	nextP[12][0] = P[12][0] + P[12][1]*SF[7] + P[12][2]*SF[9] + P[12][3]*SF[8] + P[12][10]*SF[11] + P[12][11]*SPP[7] + P[12][12]*SPP[6];
	nextP[12][1] = P[12][1] + P[12][0]*SF[6] + P[12][2]*SF[5] + P[12][3]*SF[9] + P[12][11]*SPP[6] - P[12][12]*SPP[7] - (P[12][10]*q0)/2;
	nextP[12][2] = P[12][2] + P[12][0]*SF[4] + P[12][1]*SF[8] + P[12][3]*SF[6] + P[12][12]*SF[11] - P[12][10]*SPP[6] - (P[12][11]*q0)/2;
	nextP[12][3] = P[12][3] + P[12][0]*SF[5] + P[12][1]*SF[4] + P[12][2]*SF[7] - P[12][11]*SF[11] + P[12][10]*SPP[7] - (P[12][12]*q0)/2;
	nextP[12][4] = P[12][4] + P[12][1]*SF[1] + P[12][0]*SF[3] + P[12][2]*SPP[0] - P[12][3]*SPP[2] - P[12][13]*SPP[4];
	nextP[12][5] = P[12][5] + P[12][0]*SF[2] + P[12][2]*SF[1] + P[12][3]*SF[3] - P[12][1]*SPP[0] + P[12][13]*SPP[3];
	nextP[12][6] = P[12][6] + P[12][1]*SF[2] + P[12][3]*SF[1] + P[12][0]*SPP[0] - P[12][2]*SPP[1] - P[12][13]*SPP[5];
	nextP[12][7] = P[12][7] + P[12][4]*dt;
	nextP[12][8] = P[12][8] + P[12][5]*dt;
	nextP[12][9] = P[12][9] + P[12][6]*dt;
	nextP[12][10] = P[12][10];
	nextP[12][11] = P[12][11];
	nextP[12][12] = P[12][12];
	nextP[12][13] = P[12][13];
	nextP[12][14] = P[12][14];
	nextP[12][15] = P[12][15];
	nextP[12][16] = P[12][16];
	nextP[12][17] = P[12][17];
	nextP[12][18] = P[12][18];
	nextP[12][19] = P[12][19];
	nextP[12][20] = P[12][20];
	nextP[12][21] = P[12][21];
	nextP[13][0] = P[13][0] + P[13][1]*SF[7] + P[13][2]*SF[9] + P[13][3]*SF[8] + P[13][10]*SF[11] + P[13][11]*SPP[7] + P[13][12]*SPP[6];
	nextP[13][1] = P[13][1] + P[13][0]*SF[6] + P[13][2]*SF[5] + P[13][3]*SF[9] + P[13][11]*SPP[6] - P[13][12]*SPP[7] - (P[13][10]*q0)/2;
	nextP[13][2] = P[13][2] + P[13][0]*SF[4] + P[13][1]*SF[8] + P[13][3]*SF[6] + P[13][12]*SF[11] - P[13][10]*SPP[6] - (P[13][11]*q0)/2;
	nextP[13][3] = P[13][3] + P[13][0]*SF[5] + P[13][1]*SF[4] + P[13][2]*SF[7] - P[13][11]*SF[11] + P[13][10]*SPP[7] - (P[13][12]*q0)/2;
	nextP[13][4] = P[13][4] + P[13][1]*SF[1] + P[13][0]*SF[3] + P[13][2]*SPP[0] - P[13][3]*SPP[2] - P[13][13]*SPP[4];
	nextP[13][5] = P[13][5] + P[13][0]*SF[2] + P[13][2]*SF[1] + P[13][3]*SF[3] - P[13][1]*SPP[0] + P[13][13]*SPP[3];
	nextP[13][6] = P[13][6] + P[13][1]*SF[2] + P[13][3]*SF[1] + P[13][0]*SPP[0] - P[13][2]*SPP[1] - P[13][13]*SPP[5];
	nextP[13][7] = P[13][7] + P[13][4]*dt;
	nextP[13][8] = P[13][8] + P[13][5]*dt;
	nextP[13][9] = P[13][9] + P[13][6]*dt;
	nextP[13][10] = P[13][10];
	nextP[13][11] = P[13][11];
	nextP[13][12] = P[13][12];
	nextP[13][13] = P[13][13];
	nextP[13][14] = P[13][14];
	nextP[13][15] = P[13][15];
	nextP[13][16] = P[13][16];
	nextP[13][17] = P[13][17];
	nextP[13][18] = P[13][18];
	nextP[13][19] = P[13][19];
	nextP[13][20] = P[13][20];
	nextP[13][21] = P[13][21];
	nextP[14][0] = P[14][0] + P[14][1]*SF[7] + P[14][2]*SF[9] + P[14][3]*SF[8] + P[14][10]*SF[11] + P[14][11]*SPP[7] + P[14][12]*SPP[6];
	nextP[14][1] = P[14][1] + P[14][0]*SF[6] + P[14][2]*SF[5] + P[14][3]*SF[9] + P[14][11]*SPP[6] - P[14][12]*SPP[7] - (P[14][10]*q0)/2;
	nextP[14][2] = P[14][2] + P[14][0]*SF[4] + P[14][1]*SF[8] + P[14][3]*SF[6] + P[14][12]*SF[11] - P[14][10]*SPP[6] - (P[14][11]*q0)/2;
	nextP[14][3] = P[14][3] + P[14][0]*SF[5] + P[14][1]*SF[4] + P[14][2]*SF[7] - P[14][11]*SF[11] + P[14][10]*SPP[7] - (P[14][12]*q0)/2;
	nextP[14][4] = P[14][4] + P[14][1]*SF[1] + P[14][0]*SF[3] + P[14][2]*SPP[0] - P[14][3]*SPP[2] - P[14][13]*SPP[4];
	nextP[14][5] = P[14][5] + P[14][0]*SF[2] + P[14][2]*SF[1] + P[14][3]*SF[3] - P[14][1]*SPP[0] + P[14][13]*SPP[3];
	nextP[14][6] = P[14][6] + P[14][1]*SF[2] + P[14][3]*SF[1] + P[14][0]*SPP[0] - P[14][2]*SPP[1] - P[14][13]*SPP[5];
	nextP[14][7] = P[14][7] + P[14][4]*dt;
	nextP[14][8] = P[14][8] + P[14][5]*dt;
	nextP[14][9] = P[14][9] + P[14][6]*dt;
	nextP[14][10] = P[14][10];
	nextP[14][11] = P[14][11];
	nextP[14][12] = P[14][12];
	nextP[14][13] = P[14][13];
	nextP[14][14] = P[14][14];
	nextP[14][15] = P[14][15];
	nextP[14][16] = P[14][16];
	nextP[14][17] = P[14][17];
	nextP[14][18] = P[14][18];
	nextP[14][19] = P[14][19];
	nextP[14][20] = P[14][20];
	nextP[14][21] = P[14][21];
	nextP[15][0] = P[15][0] + P[15][1]*SF[7] + P[15][2]*SF[9] + P[15][3]*SF[8] + P[15][10]*SF[11] + P[15][11]*SPP[7] + P[15][12]*SPP[6];
	nextP[15][1] = P[15][1] + P[15][0]*SF[6] + P[15][2]*SF[5] + P[15][3]*SF[9] + P[15][11]*SPP[6] - P[15][12]*SPP[7] - (P[15][10]*q0)/2;
	nextP[15][2] = P[15][2] + P[15][0]*SF[4] + P[15][1]*SF[8] + P[15][3]*SF[6] + P[15][12]*SF[11] - P[15][10]*SPP[6] - (P[15][11]*q0)/2;
	nextP[15][3] = P[15][3] + P[15][0]*SF[5] + P[15][1]*SF[4] + P[15][2]*SF[7] - P[15][11]*SF[11] + P[15][10]*SPP[7] - (P[15][12]*q0)/2;
	nextP[15][4] = P[15][4] + P[15][1]*SF[1] + P[15][0]*SF[3] + P[15][2]*SPP[0] - P[15][3]*SPP[2] - P[15][13]*SPP[4];
	nextP[15][5] = P[15][5] + P[15][0]*SF[2] + P[15][2]*SF[1] + P[15][3]*SF[3] - P[15][1]*SPP[0] + P[15][13]*SPP[3];
	nextP[15][6] = P[15][6] + P[15][1]*SF[2] + P[15][3]*SF[1] + P[15][0]*SPP[0] - P[15][2]*SPP[1] - P[15][13]*SPP[5];
	nextP[15][7] = P[15][7] + P[15][4]*dt;
	nextP[15][8] = P[15][8] + P[15][5]*dt;
	nextP[15][9] = P[15][9] + P[15][6]*dt;
	nextP[15][10] = P[15][10];
	nextP[15][11] = P[15][11];
	nextP[15][12] = P[15][12];
	nextP[15][13] = P[15][13];
	nextP[15][14] = P[15][14];
	nextP[15][15] = P[15][15];
	nextP[15][16] = P[15][16];
	nextP[15][17] = P[15][17];
	nextP[15][18] = P[15][18];
	nextP[15][19] = P[15][19];
	nextP[15][20] = P[15][20];
	nextP[15][21] = P[15][21];
	nextP[16][0] = P[16][0] + P[16][1]*SF[7] + P[16][2]*SF[9] + P[16][3]*SF[8] + P[16][10]*SF[11] + P[16][11]*SPP[7] + P[16][12]*SPP[6];
	nextP[16][1] = P[16][1] + P[16][0]*SF[6] + P[16][2]*SF[5] + P[16][3]*SF[9] + P[16][11]*SPP[6] - P[16][12]*SPP[7] - (P[16][10]*q0)/2;
	nextP[16][2] = P[16][2] + P[16][0]*SF[4] + P[16][1]*SF[8] + P[16][3]*SF[6] + P[16][12]*SF[11] - P[16][10]*SPP[6] - (P[16][11]*q0)/2;
	nextP[16][3] = P[16][3] + P[16][0]*SF[5] + P[16][1]*SF[4] + P[16][2]*SF[7] - P[16][11]*SF[11] + P[16][10]*SPP[7] - (P[16][12]*q0)/2;
	nextP[16][4] = P[16][4] + P[16][1]*SF[1] + P[16][0]*SF[3] + P[16][2]*SPP[0] - P[16][3]*SPP[2] - P[16][13]*SPP[4];
	nextP[16][5] = P[16][5] + P[16][0]*SF[2] + P[16][2]*SF[1] + P[16][3]*SF[3] - P[16][1]*SPP[0] + P[16][13]*SPP[3];
	nextP[16][6] = P[16][6] + P[16][1]*SF[2] + P[16][3]*SF[1] + P[16][0]*SPP[0] - P[16][2]*SPP[1] - P[16][13]*SPP[5];
	nextP[16][7] = P[16][7] + P[16][4]*dt;
	nextP[16][8] = P[16][8] + P[16][5]*dt;
	nextP[16][9] = P[16][9] + P[16][6]*dt;
	nextP[16][10] = P[16][10];
	nextP[16][11] = P[16][11];
	nextP[16][12] = P[16][12];
	nextP[16][13] = P[16][13];
	nextP[16][14] = P[16][14];
	nextP[16][15] = P[16][15];
	nextP[16][16] = P[16][16];
	nextP[16][17] = P[16][17];
	nextP[16][18] = P[16][18];
	nextP[16][19] = P[16][19];
	nextP[16][20] = P[16][20];
	nextP[16][21] = P[16][21];
	nextP[17][0] = P[17][0] + P[17][1]*SF[7] + P[17][2]*SF[9] + P[17][3]*SF[8] + P[17][10]*SF[11] + P[17][11]*SPP[7] + P[17][12]*SPP[6];
	nextP[17][1] = P[17][1] + P[17][0]*SF[6] + P[17][2]*SF[5] + P[17][3]*SF[9] + P[17][11]*SPP[6] - P[17][12]*SPP[7] - (P[17][10]*q0)/2;
	nextP[17][2] = P[17][2] + P[17][0]*SF[4] + P[17][1]*SF[8] + P[17][3]*SF[6] + P[17][12]*SF[11] - P[17][10]*SPP[6] - (P[17][11]*q0)/2;
	nextP[17][3] = P[17][3] + P[17][0]*SF[5] + P[17][1]*SF[4] + P[17][2]*SF[7] - P[17][11]*SF[11] + P[17][10]*SPP[7] - (P[17][12]*q0)/2;
	nextP[17][4] = P[17][4] + P[17][1]*SF[1] + P[17][0]*SF[3] + P[17][2]*SPP[0] - P[17][3]*SPP[2] - P[17][13]*SPP[4];
	nextP[17][5] = P[17][5] + P[17][0]*SF[2] + P[17][2]*SF[1] + P[17][3]*SF[3] - P[17][1]*SPP[0] + P[17][13]*SPP[3];
	nextP[17][6] = P[17][6] + P[17][1]*SF[2] + P[17][3]*SF[1] + P[17][0]*SPP[0] - P[17][2]*SPP[1] - P[17][13]*SPP[5];
	nextP[17][7] = P[17][7] + P[17][4]*dt;
	nextP[17][8] = P[17][8] + P[17][5]*dt;
	nextP[17][9] = P[17][9] + P[17][6]*dt;
	nextP[17][10] = P[17][10];
	nextP[17][11] = P[17][11];
	nextP[17][12] = P[17][12];
	nextP[17][13] = P[17][13];
	nextP[17][14] = P[17][14];
	nextP[17][15] = P[17][15];
	nextP[17][16] = P[17][16];
	nextP[17][17] = P[17][17];
	nextP[17][18] = P[17][18];
	nextP[17][19] = P[17][19];
	nextP[17][20] = P[17][20];
	nextP[17][21] = P[17][21];
	nextP[18][0] = P[18][0] + P[18][1]*SF[7] + P[18][2]*SF[9] + P[18][3]*SF[8] + P[18][10]*SF[11] + P[18][11]*SPP[7] + P[18][12]*SPP[6];
	nextP[18][1] = P[18][1] + P[18][0]*SF[6] + P[18][2]*SF[5] + P[18][3]*SF[9] + P[18][11]*SPP[6] - P[18][12]*SPP[7] - (P[18][10]*q0)/2;
	nextP[18][2] = P[18][2] + P[18][0]*SF[4] + P[18][1]*SF[8] + P[18][3]*SF[6] + P[18][12]*SF[11] - P[18][10]*SPP[6] - (P[18][11]*q0)/2;
	nextP[18][3] = P[18][3] + P[18][0]*SF[5] + P[18][1]*SF[4] + P[18][2]*SF[7] - P[18][11]*SF[11] + P[18][10]*SPP[7] - (P[18][12]*q0)/2;
	nextP[18][4] = P[18][4] + P[18][1]*SF[1] + P[18][0]*SF[3] + P[18][2]*SPP[0] - P[18][3]*SPP[2] - P[18][13]*SPP[4];
	nextP[18][5] = P[18][5] + P[18][0]*SF[2] + P[18][2]*SF[1] + P[18][3]*SF[3] - P[18][1]*SPP[0] + P[18][13]*SPP[3];
	nextP[18][6] = P[18][6] + P[18][1]*SF[2] + P[18][3]*SF[1] + P[18][0]*SPP[0] - P[18][2]*SPP[1] - P[18][13]*SPP[5];
	nextP[18][7] = P[18][7] + P[18][4]*dt;
	nextP[18][8] = P[18][8] + P[18][5]*dt;
	nextP[18][9] = P[18][9] + P[18][6]*dt;
	nextP[18][10] = P[18][10];
	nextP[18][11] = P[18][11];
	nextP[18][12] = P[18][12];
	nextP[18][13] = P[18][13];
	nextP[18][14] = P[18][14];
	nextP[18][15] = P[18][15];
	nextP[18][16] = P[18][16];
	nextP[18][17] = P[18][17];
	nextP[18][18] = P[18][18];
	nextP[18][19] = P[18][19];
	nextP[18][20] = P[18][20];
	nextP[18][21] = P[18][21];
	nextP[19][0] = P[19][0] + P[19][1]*SF[7] + P[19][2]*SF[9] + P[19][3]*SF[8] + P[19][10]*SF[11] + P[19][11]*SPP[7] + P[19][12]*SPP[6];
	nextP[19][1] = P[19][1] + P[19][0]*SF[6] + P[19][2]*SF[5] + P[19][3]*SF[9] + P[19][11]*SPP[6] - P[19][12]*SPP[7] - (P[19][10]*q0)/2;
	nextP[19][2] = P[19][2] + P[19][0]*SF[4] + P[19][1]*SF[8] + P[19][3]*SF[6] + P[19][12]*SF[11] - P[19][10]*SPP[6] - (P[19][11]*q0)/2;
	nextP[19][3] = P[19][3] + P[19][0]*SF[5] + P[19][1]*SF[4] + P[19][2]*SF[7] - P[19][11]*SF[11] + P[19][10]*SPP[7] - (P[19][12]*q0)/2;
	nextP[19][4] = P[19][4] + P[19][1]*SF[1] + P[19][0]*SF[3] + P[19][2]*SPP[0] - P[19][3]*SPP[2] - P[19][13]*SPP[4];
	nextP[19][5] = P[19][5] + P[19][0]*SF[2] + P[19][2]*SF[1] + P[19][3]*SF[3] - P[19][1]*SPP[0] + P[19][13]*SPP[3];
	nextP[19][6] = P[19][6] + P[19][1]*SF[2] + P[19][3]*SF[1] + P[19][0]*SPP[0] - P[19][2]*SPP[1] - P[19][13]*SPP[5];
	nextP[19][7] = P[19][7] + P[19][4]*dt;
	nextP[19][8] = P[19][8] + P[19][5]*dt;
	nextP[19][9] = P[19][9] + P[19][6]*dt;
	nextP[19][10] = P[19][10];
	nextP[19][11] = P[19][11];
	nextP[19][12] = P[19][12];
	nextP[19][13] = P[19][13];
	nextP[19][14] = P[19][14];
	nextP[19][15] = P[19][15];
	nextP[19][16] = P[19][16];
	nextP[19][17] = P[19][17];
	nextP[19][18] = P[19][18];
	nextP[19][19] = P[19][19];
	nextP[19][20] = P[19][20];
	nextP[19][21] = P[19][21];
	nextP[20][0] = P[20][0] + P[20][1]*SF[7] + P[20][2]*SF[9] + P[20][3]*SF[8] + P[20][10]*SF[11] + P[20][11]*SPP[7] + P[20][12]*SPP[6];
	nextP[20][1] = P[20][1] + P[20][0]*SF[6] + P[20][2]*SF[5] + P[20][3]*SF[9] + P[20][11]*SPP[6] - P[20][12]*SPP[7] - (P[20][10]*q0)/2;
	nextP[20][2] = P[20][2] + P[20][0]*SF[4] + P[20][1]*SF[8] + P[20][3]*SF[6] + P[20][12]*SF[11] - P[20][10]*SPP[6] - (P[20][11]*q0)/2;
	nextP[20][3] = P[20][3] + P[20][0]*SF[5] + P[20][1]*SF[4] + P[20][2]*SF[7] - P[20][11]*SF[11] + P[20][10]*SPP[7] - (P[20][12]*q0)/2;
	nextP[20][4] = P[20][4] + P[20][1]*SF[1] + P[20][0]*SF[3] + P[20][2]*SPP[0] - P[20][3]*SPP[2] - P[20][13]*SPP[4];
	nextP[20][5] = P[20][5] + P[20][0]*SF[2] + P[20][2]*SF[1] + P[20][3]*SF[3] - P[20][1]*SPP[0] + P[20][13]*SPP[3];
	nextP[20][6] = P[20][6] + P[20][1]*SF[2] + P[20][3]*SF[1] + P[20][0]*SPP[0] - P[20][2]*SPP[1] - P[20][13]*SPP[5];
	nextP[20][7] = P[20][7] + P[20][4]*dt;
	nextP[20][8] = P[20][8] + P[20][5]*dt;
	nextP[20][9] = P[20][9] + P[20][6]*dt;
	nextP[20][10] = P[20][10];
	nextP[20][11] = P[20][11];
	nextP[20][12] = P[20][12];
	nextP[20][13] = P[20][13];
	nextP[20][14] = P[20][14];
	nextP[20][15] = P[20][15];
	nextP[20][16] = P[20][16];
	nextP[20][17] = P[20][17];
	nextP[20][18] = P[20][18];
	nextP[20][19] = P[20][19];
	nextP[20][20] = P[20][20];
	nextP[20][21] = P[20][21];
	nextP[21][0] = P[21][0] + P[21][1]*SF[7] + P[21][2]*SF[9] + P[21][3]*SF[8] + P[21][10]*SF[11] + P[21][11]*SPP[7] + P[21][12]*SPP[6];
	nextP[21][1] = P[21][1] + P[21][0]*SF[6] + P[21][2]*SF[5] + P[21][3]*SF[9] + P[21][11]*SPP[6] - P[21][12]*SPP[7] - (P[21][10]*q0)/2;
	nextP[21][2] = P[21][2] + P[21][0]*SF[4] + P[21][1]*SF[8] + P[21][3]*SF[6] + P[21][12]*SF[11] - P[21][10]*SPP[6] - (P[21][11]*q0)/2;
	nextP[21][3] = P[21][3] + P[21][0]*SF[5] + P[21][1]*SF[4] + P[21][2]*SF[7] - P[21][11]*SF[11] + P[21][10]*SPP[7] - (P[21][12]*q0)/2;
	nextP[21][4] = P[21][4] + P[21][1]*SF[1] + P[21][0]*SF[3] + P[21][2]*SPP[0] - P[21][3]*SPP[2] - P[21][13]*SPP[4];
	nextP[21][5] = P[21][5] + P[21][0]*SF[2] + P[21][2]*SF[1] + P[21][3]*SF[3] - P[21][1]*SPP[0] + P[21][13]*SPP[3];
	nextP[21][6] = P[21][6] + P[21][1]*SF[2] + P[21][3]*SF[1] + P[21][0]*SPP[0] - P[21][2]*SPP[1] - P[21][13]*SPP[5];
	nextP[21][7] = P[21][7] + P[21][4]*dt;
	nextP[21][8] = P[21][8] + P[21][5]*dt;
	nextP[21][9] = P[21][9] + P[21][6]*dt;
	nextP[21][10] = P[21][10];
	nextP[21][11] = P[21][11];
	nextP[21][12] = P[21][12];
	nextP[21][13] = P[21][13];
	nextP[21][14] = P[21][14];
	nextP[21][15] = P[21][15];
	nextP[21][16] = P[21][16];
	nextP[21][17] = P[21][17];
	nextP[21][18] = P[21][18];
	nextP[21][19] = P[21][19];
	nextP[21][20] = P[21][20];
	nextP[21][21] = P[21][21];

    for (uint8_t i=0; i<= 21; i++)
    {
        nextP[i][i] = nextP[i][i] + processNoise[i];
    }

    // If on ground or no compasss fitted, inhibit magnetic field state updates by
    // setting the corresponding covariance terms to zero
    if (onGround || !useCompass)
    {
        zeroRows(nextP,16,21);
        zeroCols(nextP,16,21);
    }

    // If on ground or not using airspeed sensing, inhibit wind velocity
    // covariance growth.
    if (onGround || !useAirspeed)
    {
        zeroRows(nextP,14,15);
        zeroCols(nextP,14,15);
    }

    // If the total position variance exceeds 1E6 (1000m), then stop covariance
    // growth by setting the predicted to the previous values
    // This prevent an ill conditioned matrix from occurring for long periods
    // without GPS
    if ((P[7][7] + P[8][8]) > 1e6f)
    {
        for (uint8_t i=7; i<=8; i++)
        {
            for (uint8_t j=0; j<=21; j++)
            {
                nextP[i][j] = P[i][j];
                nextP[j][i] = P[j][i];
            }
        }
    }

    // Copy to output whilst forcing symmetry to prevent ill-conditioning
    // of the matrix
    for (uint8_t i=0; i<=21; i++) P[i][i] = nextP[i][i];
    for (uint8_t i=1; i<=21; i++)
    {
        for (uint8_t j=0; j<=i-1; j++)
        {
            P[i][j] = 0.5f*(nextP[i][j] + nextP[j][i]);
            P[j][i] = P[i][j];
        }
    }

    ConstrainVariances();

    perf_end(_perf_CovariancePrediction);
}

void NavEKF::FuseVelPosNED()
{
    perf_begin(_perf_FuseVelPosNED);

    // health is set bad until test passed
    velHealth = false;
    posHealth = false;
    hgtHealth = false;

    // declare variables used to check measurement errors
    Vector3f velInnov;
    Vector2 posInnov;
    float hgtInnov = 0;

    // declare variables used to control access to arrays
    bool fuseData[6] = {false,false,false,false,false,false};
    uint8_t stateIndex;
    uint8_t obsIndex;
    uint8_t indexLimit; // used to prevent access to wind and magnetic field states and variances when on ground

    // declare variables used by state and covariance update calculations
    float NEvelErr;
    float DvelErr;
    float posErr;
    Vector6 R_OBS;
    Vector6 observation;
    float SK;

    // Perform sequential fusion of GPS measurements. This assumes that the
    // errors in the different velocity and position components are
    // uncorrelated which is not true, however in the absence of covariance
    // data from the GPS receiver it is the only assumption we can make
    // so we might as well take advantage of the computational efficiencies
    // associated with sequential fusion
    if (fuseVelData || fusePosData || fuseHgtData)
    {
        // set the GPS data timeout depending on whether airspeed data is present
        uint32_t gpsRetryTime;
        if (useAirspeed) gpsRetryTime = _gpsRetryTimeUseTAS;
        else gpsRetryTime = _gpsRetryTimeNoTAS;

        // Form the observation vector
        for (uint8_t i=0; i<=2; i++) observation[i] = velNED[i];
        for (uint8_t i=3; i<=4; i++) observation[i] = posNE[i-3];
        observation[5] = -hgtMea;

        // zero observations if in static mode (used for pre-arm and bench testing)
        if (staticMode) {
            for (uint8_t i=0; i<=5; i++) observation[i] = 0.0f;
        }

        // additional error in GPS velocity caused by manoeuvring
        NEvelErr = _gpsNEVelVarAccScale * accNavMag;
        DvelErr  = _gpsDVelVarAccScale  * accNavMag;

        // additional error in GPS position caused by manoeuvring
        posErr = _gpsPosVarAccScale*accNavMag;

        // Estimate the GPS Velocity, GPS horiz position and height measurement variances.
        R_OBS[0] = gpsVarScaler*(sq(_gpsHorizVelNoise) + sq(NEvelErr));
        R_OBS[1] = R_OBS[0];
        R_OBS[2] = gpsVarScaler*(sq(_gpsVertVelNoise) + sq(DvelErr));
        R_OBS[3] = gpsVarScaler*(sq(_gpsHorizPosNoise) + sq(posErr));
        R_OBS[4] = R_OBS[3];
        R_OBS[5] = hgtVarScaler*sq(_baroAltNoise);

        // calculate innovations and check GPS data validity using an innovation consistency check
        if (fuseVelData)
        {
            // test velocity measurements
            uint8_t imax = 2;
            if (_fusionModeGPS == 1) imax = 1;
            for (uint8_t i = 0; i<=imax; i++)
            {
                velInnov[i] = statesAtVelTime[i+4] - observation[i];
                stateIndex = 4 + i;
                varInnovVelPos[i] = P[stateIndex][stateIndex] + R_OBS[i];
            }
            // apply an innovation consistency threshold test
            velHealth = ((sq(velInnov[0]) + sq(velInnov[1]) + sq(velInnov[2])) < (sq(_gpsVelInnovGate) * (varInnovVelPos[0] + varInnovVelPos[1] + varInnovVelPos[2])));
            velTimeout = (hal.scheduler->millis() - velFailTime) > gpsRetryTime;
            if (velHealth || velTimeout)
            {
                velHealth = true;
                velFailTime = hal.scheduler->millis();
            }
            else
            {
                velHealth = false;
            }
        }
        if (fusePosData)
        {
            // test horizontal position measurements
            posInnov[0] = statesAtPosTime[7] - observation[3];
            posInnov[1] = statesAtPosTime[8] - observation[4];
            varInnovVelPos[3] = P[7][7] + R_OBS[3];
            varInnovVelPos[4] = P[8][8] + R_OBS[4];
            // apply an innovation consistency threshold test
            posHealth = ((sq(posInnov[0]) + sq(posInnov[1])) < (sq(_gpsPosInnovGate) * (varInnovVelPos[3] + varInnovVelPos[4])));
            posTimeout = (hal.scheduler->millis() - posFailTime) > gpsRetryTime;
            if (posHealth || posTimeout)
            {
                posHealth = true;
                posFailTime = hal.scheduler->millis();
            }
            else
            {
                posHealth = false;
            }
        }
        // test height measurements
        if (fuseHgtData)
        {
            // set the height data timeout depending on whether vertical velocity data is being used
            uint32_t hgtRetryTime;
            if (_fusionModeGPS == 0) hgtRetryTime = _hgtRetryTimeMode0;
            else hgtRetryTime = _hgtRetryTimeMode12;
            // calculate height innovations
            hgtInnov = statesAtHgtTime[9] - observation[5];
            varInnovVelPos[5] = P[9][9] + R_OBS[5];
            // apply an innovation consistency threshold test
            hgtHealth = (sq(hgtInnov) < (sq(_hgtInnovGate) * varInnovVelPos[5]));
            hgtTimeout = (hal.scheduler->millis() - hgtFailTime) > hgtRetryTime;
            if (hgtHealth || hgtTimeout)
            {
                hgtHealth = true;
                hgtFailTime = hal.scheduler->millis();
            }
            else
            {
                hgtHealth = false;
            }
        }
        // Set range for sequential fusion of velocity and position measurements depending
        // on which data is available and its health
        if ((fuseVelData && _fusionModeGPS == 0 && velHealth) || staticMode)
        {
            fuseData[0] = true;
            fuseData[1] = true;
            fuseData[2] = true;
        }
        if (fuseVelData && _fusionModeGPS == 1 && velHealth)
        {
            fuseData[0] = true;
            fuseData[1] = true;
        }
        if ((fusePosData && _fusionModeGPS <= 2 && posHealth) || staticMode)
        {
            fuseData[3] = true;
            fuseData[4] = true;
        }
        if ((fuseHgtData && hgtHealth) || staticMode)
        {
            fuseData[5] = true;
        }
        // Limit access to first 13 states when on ground.
        if (!onGround)
        {
            indexLimit = 21;
        }
        else
        {
            indexLimit = 13;
        }
        // Fuse measurements sequentially
        for (obsIndex=0; obsIndex<=5; obsIndex++)
        {
            if (fuseData[obsIndex])
            {
                stateIndex = 4 + obsIndex;
                // Calculate the measurement innovation, using states from a
                // different time coordinate if fusing height data
                if (obsIndex <= 2)
                {
                    innovVelPos[obsIndex] = statesAtVelTime[stateIndex] - observation[obsIndex];
                }
                else if (obsIndex == 3 || obsIndex == 4)
                {
                    innovVelPos[obsIndex] = statesAtPosTime[stateIndex] - observation[obsIndex];
                }
                else
                {
                    innovVelPos[obsIndex] = statesAtHgtTime[stateIndex] - observation[obsIndex];
                }
                // Calculate the Kalman Gain
                // Calculate innovation variances - also used for data logging
                varInnovVelPos[obsIndex] = P[stateIndex][stateIndex] + R_OBS[obsIndex];
                SK = 1.0f/varInnovVelPos[obsIndex];
                for (uint8_t i= 0; i<=indexLimit; i++)
                {
                    Kfusion[i] = P[i][stateIndex]*SK;
                }
                // Calculate state corrections and re-normalise the quaternions
                for (uint8_t i = 0; i<=indexLimit; i++)
                {
                    states[i] = states[i] - Kfusion[i] * innovVelPos[obsIndex];
                }
                Quaternion q(states[0], states[1], states[2], states[3]);
                q.normalize();
                for (uint8_t i = 0; i<=3; i++) {
                    states[i] = q[i];
                }

                // Update the covariance - take advantage of direct observation of a
                // single state at index = stateIndex to reduce computations
                // Optimised implementation of standard equation P = (I - K*H)*P;
                for (uint8_t i= 0; i<=indexLimit; i++)
                {
                    for (uint8_t j= 0; j<=indexLimit; j++)
                    {
                        KHP[i][j] = Kfusion[i] * P[stateIndex][j];
                    }
                }
                for (uint8_t i= 0; i<=indexLimit; i++)
                {
                    for (uint8_t j= 0; j<=indexLimit; j++)
                    {
                        P[i][j] = P[i][j] - KHP[i][j];
                    }
                }
            }
        }
    }

    // force the covariance matrix to me symmetrical and limit the variances to prevent
    // ill-condiioning.
    ForceSymmetry();
    ConstrainVariances();

    perf_end(_perf_FuseVelPosNED);
}

void NavEKF::FuseMagnetometer()
{
    perf_begin(_perf_FuseMagnetometer);
    ftype &q0 = mag_state.q0;
    ftype &q1 = mag_state.q1;
    ftype &q2 = mag_state.q2;
    ftype &q3 = mag_state.q3;
    ftype &magN = mag_state.magN;
    ftype &magE = mag_state.magE;
    ftype &magD = mag_state.magD;
    ftype &magXbias = mag_state.magXbias;
    ftype &magYbias = mag_state.magYbias;
    ftype &magZbias = mag_state.magZbias;
    uint8_t &obsIndex = mag_state.obsIndex;
    Matrix3f &DCM = mag_state.DCM;
    Vector3f &MagPred = mag_state.MagPred;
    ftype &R_MAG = mag_state.R_MAG;
    ftype *SH_MAG = &mag_state.SH_MAG[0];
    Vector22 H_MAG;
    Vector6 SK_MX;
    Vector6 SK_MY;
    Vector6 SK_MZ;
    uint8_t indexLimit; // used to prevent access to wind and magnetic field states and variances when on ground

    // Perform sequential fusion of Magnetometer measurements.
    // This assumes that the errors in the different components are
    // uncorrelated which is not true, however in the absence of covariance
    // data fit is the only assumption we can make
    // so we might as well take advantage of the computational efficiencies
    // associated with sequential fusion
    if (fuseMagData || obsIndex == 1 || obsIndex == 2)
    {
        // Prevent access last 11 states when on ground.
        if (!onGround)
        {
            indexLimit = 21;
        }
        else
        {
            indexLimit = 13;
        }
        // Calculate observation jacobians and Kalman gains
        if (fuseMagData)
        {
            // Copy required states to local variable names
            q0       = statesAtMagMeasTime[0];
            q1       = statesAtMagMeasTime[1];
            q2       = statesAtMagMeasTime[2];
            q3       = statesAtMagMeasTime[3];
            magN     = statesAtMagMeasTime[16];
            magE     = statesAtMagMeasTime[17];
            magD     = statesAtMagMeasTime[18];
            magXbias = statesAtMagMeasTime[19];
            magYbias = statesAtMagMeasTime[20];
            magZbias = statesAtMagMeasTime[21];

            // rotate predicted earth components into body axes and calculate
            // predicted measurements
            DCM[0][0] = q0*q0 + q1*q1 - q2*q2 - q3*q3;
            DCM[0][1] = 2*(q1*q2 + q0*q3);
            DCM[0][2] = 2*(q1*q3-q0*q2);
            DCM[1][0] = 2*(q1*q2 - q0*q3);
            DCM[1][1] = q0*q0 - q1*q1 + q2*q2 - q3*q3;
            DCM[1][2] = 2*(q2*q3 + q0*q1);
            DCM[2][0] = 2*(q1*q3 + q0*q2);
            DCM[2][1] = 2*(q2*q3 - q0*q1);
            DCM[2][2] = q0*q0 - q1*q1 - q2*q2 + q3*q3;
            MagPred[0] = DCM[0][0]*magN + DCM[0][1]*magE  + DCM[0][2]*magD + magXbias;
            MagPred[1] = DCM[1][0]*magN + DCM[1][1]*magE  + DCM[1][2]*magD + magYbias;
            MagPred[2] = DCM[2][0]*magN + DCM[2][1]*magE  + DCM[2][2]*magD + magZbias;

            // scale magnetometer observation error with total angular rate
            R_MAG = sq(_magNoise) + sq(_magVarRateScale*dAngIMU.length() / dtIMU);

            // Calculate observation jacobians
			SH_MAG[0] = 2*magD*q3 + 2*magE*q2 + 2*magN*q1;
			SH_MAG[1] = 2*magD*q0 - 2*magE*q1 + 2*magN*q2;
			SH_MAG[2] = 2*magD*q1 + 2*magE*q0 - 2*magN*q3;
			SH_MAG[3] = sq(q3);
			SH_MAG[4] = sq(q2);
			SH_MAG[5] = sq(q1);
			SH_MAG[6] = sq(q0);
			SH_MAG[7] = 2*magN*q0;
			SH_MAG[8] = 2*magE*q3;
            for (uint8_t i=0; i<=21; i++) H_MAG[i] = 0;
			H_MAG[0] = SH_MAG[7] + SH_MAG[8] - 2*magD*q2;
			H_MAG[1] = SH_MAG[0];
			H_MAG[2] = 2*magE*q1 - 2*magD*q0 - 2*magN*q2;
			H_MAG[3] = SH_MAG[2];
			H_MAG[16] = SH_MAG[5] - SH_MAG[4] - SH_MAG[3] + SH_MAG[6];
			H_MAG[17] = 2*q0*q3 + 2*q1*q2;
			H_MAG[18] = 2*q1*q3 - 2*q0*q2;
			H_MAG[19] = 1;

            // Calculate Kalman gain
			SK_MX[0] = 1/(P[19][19] + R_MAG + P[1][19]*SH_MAG[0] + P[3][19]*SH_MAG[2] - P[16][19]*(SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6]) - (2*magD*q0 - 2*magE*q1 + 2*magN*q2)*(P[19][2] + P[1][2]*SH_MAG[0] + P[3][2]*SH_MAG[2] - P[16][2]*(SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6]) + P[17][2]*(2*q0*q3 + 2*q1*q2) - P[18][2]*(2*q0*q2 - 2*q1*q3) - P[2][2]*(2*magD*q0 - 2*magE*q1 + 2*magN*q2) + P[0][2]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) + (SH_MAG[7] + SH_MAG[8] - 2*magD*q2)*(P[19][0] + P[1][0]*SH_MAG[0] + P[3][0]*SH_MAG[2] - P[16][0]*(SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6]) + P[17][0]*(2*q0*q3 + 2*q1*q2) - P[18][0]*(2*q0*q2 - 2*q1*q3) - P[2][0]*(2*magD*q0 - 2*magE*q1 + 2*magN*q2) + P[0][0]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) + SH_MAG[0]*(P[19][1] + P[1][1]*SH_MAG[0] + P[3][1]*SH_MAG[2] - P[16][1]*(SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6]) + P[17][1]*(2*q0*q3 + 2*q1*q2) - P[18][1]*(2*q0*q2 - 2*q1*q3) - P[2][1]*(2*magD*q0 - 2*magE*q1 + 2*magN*q2) + P[0][1]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) + SH_MAG[2]*(P[19][3] + P[1][3]*SH_MAG[0] + P[3][3]*SH_MAG[2] - P[16][3]*(SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6]) + P[17][3]*(2*q0*q3 + 2*q1*q2) - P[18][3]*(2*q0*q2 - 2*q1*q3) - P[2][3]*(2*magD*q0 - 2*magE*q1 + 2*magN*q2) + P[0][3]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) - (SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6])*(P[19][16] + P[1][16]*SH_MAG[0] + P[3][16]*SH_MAG[2] - P[16][16]*(SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6]) + P[17][16]*(2*q0*q3 + 2*q1*q2) - P[18][16]*(2*q0*q2 - 2*q1*q3) - P[2][16]*(2*magD*q0 - 2*magE*q1 + 2*magN*q2) + P[0][16]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) + P[17][19]*(2*q0*q3 + 2*q1*q2) - P[18][19]*(2*q0*q2 - 2*q1*q3) - P[2][19]*(2*magD*q0 - 2*magE*q1 + 2*magN*q2) + (2*q0*q3 + 2*q1*q2)*(P[19][17] + P[1][17]*SH_MAG[0] + P[3][17]*SH_MAG[2] - P[16][17]*(SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6]) + P[17][17]*(2*q0*q3 + 2*q1*q2) - P[18][17]*(2*q0*q2 - 2*q1*q3) - P[2][17]*(2*magD*q0 - 2*magE*q1 + 2*magN*q2) + P[0][17]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) - (2*q0*q2 - 2*q1*q3)*(P[19][18] + P[1][18]*SH_MAG[0] + P[3][18]*SH_MAG[2] - P[16][18]*(SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6]) + P[17][18]*(2*q0*q3 + 2*q1*q2) - P[18][18]*(2*q0*q2 - 2*q1*q3) - P[2][18]*(2*magD*q0 - 2*magE*q1 + 2*magN*q2) + P[0][18]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) + P[0][19]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2));
			SK_MX[1] = SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6];
			SK_MX[2] = 2*magD*q0 - 2*magE*q1 + 2*magN*q2;
			SK_MX[3] = SH_MAG[7] + SH_MAG[8] - 2*magD*q2;
			SK_MX[4] = 2*q0*q2 - 2*q1*q3;
			SK_MX[5] = 2*q0*q3 + 2*q1*q2;
			Kfusion[0] = SK_MX[0]*(P[0][19] + P[0][1]*SH_MAG[0] + P[0][3]*SH_MAG[2] + P[0][0]*SK_MX[3] - P[0][2]*SK_MX[2] - P[0][16]*SK_MX[1] + P[0][17]*SK_MX[5] - P[0][18]*SK_MX[4]);
			Kfusion[1] = SK_MX[0]*(P[1][19] + P[1][1]*SH_MAG[0] + P[1][3]*SH_MAG[2] + P[1][0]*SK_MX[3] - P[1][2]*SK_MX[2] - P[1][16]*SK_MX[1] + P[1][17]*SK_MX[5] - P[1][18]*SK_MX[4]);
			Kfusion[2] = SK_MX[0]*(P[2][19] + P[2][1]*SH_MAG[0] + P[2][3]*SH_MAG[2] + P[2][0]*SK_MX[3] - P[2][2]*SK_MX[2] - P[2][16]*SK_MX[1] + P[2][17]*SK_MX[5] - P[2][18]*SK_MX[4]);
			Kfusion[3] = SK_MX[0]*(P[3][19] + P[3][1]*SH_MAG[0] + P[3][3]*SH_MAG[2] + P[3][0]*SK_MX[3] - P[3][2]*SK_MX[2] - P[3][16]*SK_MX[1] + P[3][17]*SK_MX[5] - P[3][18]*SK_MX[4]);
			Kfusion[4] = SK_MX[0]*(P[4][19] + P[4][1]*SH_MAG[0] + P[4][3]*SH_MAG[2] + P[4][0]*SK_MX[3] - P[4][2]*SK_MX[2] - P[4][16]*SK_MX[1] + P[4][17]*SK_MX[5] - P[4][18]*SK_MX[4]);
			Kfusion[5] = SK_MX[0]*(P[5][19] + P[5][1]*SH_MAG[0] + P[5][3]*SH_MAG[2] + P[5][0]*SK_MX[3] - P[5][2]*SK_MX[2] - P[5][16]*SK_MX[1] + P[5][17]*SK_MX[5] - P[5][18]*SK_MX[4]);
			Kfusion[6] = SK_MX[0]*(P[6][19] + P[6][1]*SH_MAG[0] + P[6][3]*SH_MAG[2] + P[6][0]*SK_MX[3] - P[6][2]*SK_MX[2] - P[6][16]*SK_MX[1] + P[6][17]*SK_MX[5] - P[6][18]*SK_MX[4]);
			Kfusion[7] = SK_MX[0]*(P[7][19] + P[7][1]*SH_MAG[0] + P[7][3]*SH_MAG[2] + P[7][0]*SK_MX[3] - P[7][2]*SK_MX[2] - P[7][16]*SK_MX[1] + P[7][17]*SK_MX[5] - P[7][18]*SK_MX[4]);
			Kfusion[8] = SK_MX[0]*(P[8][19] + P[8][1]*SH_MAG[0] + P[8][3]*SH_MAG[2] + P[8][0]*SK_MX[3] - P[8][2]*SK_MX[2] - P[8][16]*SK_MX[1] + P[8][17]*SK_MX[5] - P[8][18]*SK_MX[4]);
			Kfusion[9] = SK_MX[0]*(P[9][19] + P[9][1]*SH_MAG[0] + P[9][3]*SH_MAG[2] + P[9][0]*SK_MX[3] - P[9][2]*SK_MX[2] - P[9][16]*SK_MX[1] + P[9][17]*SK_MX[5] - P[9][18]*SK_MX[4]);
			Kfusion[10] = SK_MX[0]*(P[10][19] + P[10][1]*SH_MAG[0] + P[10][3]*SH_MAG[2] + P[10][0]*SK_MX[3] - P[10][2]*SK_MX[2] - P[10][16]*SK_MX[1] + P[10][17]*SK_MX[5] - P[10][18]*SK_MX[4]);
			Kfusion[11] = SK_MX[0]*(P[11][19] + P[11][1]*SH_MAG[0] + P[11][3]*SH_MAG[2] + P[11][0]*SK_MX[3] - P[11][2]*SK_MX[2] - P[11][16]*SK_MX[1] + P[11][17]*SK_MX[5] - P[11][18]*SK_MX[4]);
			Kfusion[12] = SK_MX[0]*(P[12][19] + P[12][1]*SH_MAG[0] + P[12][3]*SH_MAG[2] + P[12][0]*SK_MX[3] - P[12][2]*SK_MX[2] - P[12][16]*SK_MX[1] + P[12][17]*SK_MX[5] - P[12][18]*SK_MX[4]);
			Kfusion[13] = SK_MX[0]*(P[13][19] + P[13][1]*SH_MAG[0] + P[13][3]*SH_MAG[2] + P[13][0]*SK_MX[3] - P[13][2]*SK_MX[2] - P[13][16]*SK_MX[1] + P[13][17]*SK_MX[5] - P[13][18]*SK_MX[4]);
			Kfusion[14] = SK_MX[0]*(P[14][19] + P[14][1]*SH_MAG[0] + P[14][3]*SH_MAG[2] + P[14][0]*SK_MX[3] - P[14][2]*SK_MX[2] - P[14][16]*SK_MX[1] + P[14][17]*SK_MX[5] - P[14][18]*SK_MX[4]);
			Kfusion[15] = SK_MX[0]*(P[15][19] + P[15][1]*SH_MAG[0] + P[15][3]*SH_MAG[2] + P[15][0]*SK_MX[3] - P[15][2]*SK_MX[2] - P[15][16]*SK_MX[1] + P[15][17]*SK_MX[5] - P[15][18]*SK_MX[4]);
			Kfusion[16] = SK_MX[0]*(P[16][19] + P[16][1]*SH_MAG[0] + P[16][3]*SH_MAG[2] + P[16][0]*SK_MX[3] - P[16][2]*SK_MX[2] - P[16][16]*SK_MX[1] + P[16][17]*SK_MX[5] - P[16][18]*SK_MX[4]);
			Kfusion[17] = SK_MX[0]*(P[17][19] + P[17][1]*SH_MAG[0] + P[17][3]*SH_MAG[2] + P[17][0]*SK_MX[3] - P[17][2]*SK_MX[2] - P[17][16]*SK_MX[1] + P[17][17]*SK_MX[5] - P[17][18]*SK_MX[4]);
			Kfusion[18] = SK_MX[0]*(P[18][19] + P[18][1]*SH_MAG[0] + P[18][3]*SH_MAG[2] + P[18][0]*SK_MX[3] - P[18][2]*SK_MX[2] - P[18][16]*SK_MX[1] + P[18][17]*SK_MX[5] - P[18][18]*SK_MX[4]);
			Kfusion[19] = SK_MX[0]*(P[19][19] + P[19][1]*SH_MAG[0] + P[19][3]*SH_MAG[2] + P[19][0]*SK_MX[3] - P[19][2]*SK_MX[2] - P[19][16]*SK_MX[1] + P[19][17]*SK_MX[5] - P[19][18]*SK_MX[4]);
			Kfusion[20] = SK_MX[0]*(P[20][19] + P[20][1]*SH_MAG[0] + P[20][3]*SH_MAG[2] + P[20][0]*SK_MX[3] - P[20][2]*SK_MX[2] - P[20][16]*SK_MX[1] + P[20][17]*SK_MX[5] - P[20][18]*SK_MX[4]);
			Kfusion[21] = SK_MX[0]*(P[21][19] + P[21][1]*SH_MAG[0] + P[21][3]*SH_MAG[2] + P[21][0]*SK_MX[3] - P[21][2]*SK_MX[2] - P[21][16]*SK_MX[1] + P[21][17]*SK_MX[5] - P[21][18]*SK_MX[4]);

			// Calculate the observation innovation variance
            varInnovMag[0] = 1.0f/SK_MX[0];

            // reset the observation index to 0 (we start by fusing the X
            // measurement)
            obsIndex = 0;
            // set flags to indicate to other processes that fusion has been perfomred and is required on the next time step
            magFusePerformed = true;
            magFuseRequired = true;
        }
        else if (obsIndex == 1) // we are now fusing the Y measurement
        {
            // Calculate observation jacobians
            for (uint8_t i=0; i<=21; i++) H_MAG[i] = 0;
			H_MAG[0] = SH_MAG[2];
			H_MAG[1] = SH_MAG[1];
			H_MAG[2] = SH_MAG[0];
			H_MAG[3] = 2*magD*q2 - SH_MAG[8] - SH_MAG[7];
			H_MAG[16] = 2*q1*q2 - 2*q0*q3;
			H_MAG[17] = SH_MAG[4] - SH_MAG[3] - SH_MAG[5] + SH_MAG[6];
			H_MAG[18] = 2*q0*q1 + 2*q2*q3;
			H_MAG[20] = 1;

            // Calculate Kalman gain
			SK_MY[0] = 1/(P[20][20] + R_MAG + P[0][20]*SH_MAG[2] + P[1][20]*SH_MAG[1] + P[2][20]*SH_MAG[0] - P[17][20]*(SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6]) - (2*q0*q3 - 2*q1*q2)*(P[20][16] + P[0][16]*SH_MAG[2] + P[1][16]*SH_MAG[1] + P[2][16]*SH_MAG[0] - P[17][16]*(SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6]) - P[16][16]*(2*q0*q3 - 2*q1*q2) + P[18][16]*(2*q0*q1 + 2*q2*q3) - P[3][16]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) + (2*q0*q1 + 2*q2*q3)*(P[20][18] + P[0][18]*SH_MAG[2] + P[1][18]*SH_MAG[1] + P[2][18]*SH_MAG[0] - P[17][18]*(SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6]) - P[16][18]*(2*q0*q3 - 2*q1*q2) + P[18][18]*(2*q0*q1 + 2*q2*q3) - P[3][18]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) - (SH_MAG[7] + SH_MAG[8] - 2*magD*q2)*(P[20][3] + P[0][3]*SH_MAG[2] + P[1][3]*SH_MAG[1] + P[2][3]*SH_MAG[0] - P[17][3]*(SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6]) - P[16][3]*(2*q0*q3 - 2*q1*q2) + P[18][3]*(2*q0*q1 + 2*q2*q3) - P[3][3]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) - P[16][20]*(2*q0*q3 - 2*q1*q2) + P[18][20]*(2*q0*q1 + 2*q2*q3) + SH_MAG[2]*(P[20][0] + P[0][0]*SH_MAG[2] + P[1][0]*SH_MAG[1] + P[2][0]*SH_MAG[0] - P[17][0]*(SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6]) - P[16][0]*(2*q0*q3 - 2*q1*q2) + P[18][0]*(2*q0*q1 + 2*q2*q3) - P[3][0]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) + SH_MAG[1]*(P[20][1] + P[0][1]*SH_MAG[2] + P[1][1]*SH_MAG[1] + P[2][1]*SH_MAG[0] - P[17][1]*(SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6]) - P[16][1]*(2*q0*q3 - 2*q1*q2) + P[18][1]*(2*q0*q1 + 2*q2*q3) - P[3][1]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) + SH_MAG[0]*(P[20][2] + P[0][2]*SH_MAG[2] + P[1][2]*SH_MAG[1] + P[2][2]*SH_MAG[0] - P[17][2]*(SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6]) - P[16][2]*(2*q0*q3 - 2*q1*q2) + P[18][2]*(2*q0*q1 + 2*q2*q3) - P[3][2]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) - (SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6])*(P[20][17] + P[0][17]*SH_MAG[2] + P[1][17]*SH_MAG[1] + P[2][17]*SH_MAG[0] - P[17][17]*(SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6]) - P[16][17]*(2*q0*q3 - 2*q1*q2) + P[18][17]*(2*q0*q1 + 2*q2*q3) - P[3][17]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) - P[3][20]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2));
			SK_MY[1] = SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6];
			SK_MY[2] = SH_MAG[7] + SH_MAG[8] - 2*magD*q2;
			SK_MY[3] = 2*q0*q3 - 2*q1*q2;
			SK_MY[4] = 2*q0*q1 + 2*q2*q3;
			Kfusion[0] = SK_MY[0]*(P[0][20] + P[0][0]*SH_MAG[2] + P[0][1]*SH_MAG[1] + P[0][2]*SH_MAG[0] - P[0][3]*SK_MY[2] - P[0][17]*SK_MY[1] - P[0][16]*SK_MY[3] + P[0][18]*SK_MY[4]);
			Kfusion[1] = SK_MY[0]*(P[1][20] + P[1][0]*SH_MAG[2] + P[1][1]*SH_MAG[1] + P[1][2]*SH_MAG[0] - P[1][3]*SK_MY[2] - P[1][17]*SK_MY[1] - P[1][16]*SK_MY[3] + P[1][18]*SK_MY[4]);
			Kfusion[2] = SK_MY[0]*(P[2][20] + P[2][0]*SH_MAG[2] + P[2][1]*SH_MAG[1] + P[2][2]*SH_MAG[0] - P[2][3]*SK_MY[2] - P[2][17]*SK_MY[1] - P[2][16]*SK_MY[3] + P[2][18]*SK_MY[4]);
			Kfusion[3] = SK_MY[0]*(P[3][20] + P[3][0]*SH_MAG[2] + P[3][1]*SH_MAG[1] + P[3][2]*SH_MAG[0] - P[3][3]*SK_MY[2] - P[3][17]*SK_MY[1] - P[3][16]*SK_MY[3] + P[3][18]*SK_MY[4]);
			Kfusion[4] = SK_MY[0]*(P[4][20] + P[4][0]*SH_MAG[2] + P[4][1]*SH_MAG[1] + P[4][2]*SH_MAG[0] - P[4][3]*SK_MY[2] - P[4][17]*SK_MY[1] - P[4][16]*SK_MY[3] + P[4][18]*SK_MY[4]);
			Kfusion[5] = SK_MY[0]*(P[5][20] + P[5][0]*SH_MAG[2] + P[5][1]*SH_MAG[1] + P[5][2]*SH_MAG[0] - P[5][3]*SK_MY[2] - P[5][17]*SK_MY[1] - P[5][16]*SK_MY[3] + P[5][18]*SK_MY[4]);
			Kfusion[6] = SK_MY[0]*(P[6][20] + P[6][0]*SH_MAG[2] + P[6][1]*SH_MAG[1] + P[6][2]*SH_MAG[0] - P[6][3]*SK_MY[2] - P[6][17]*SK_MY[1] - P[6][16]*SK_MY[3] + P[6][18]*SK_MY[4]);
			Kfusion[7] = SK_MY[0]*(P[7][20] + P[7][0]*SH_MAG[2] + P[7][1]*SH_MAG[1] + P[7][2]*SH_MAG[0] - P[7][3]*SK_MY[2] - P[7][17]*SK_MY[1] - P[7][16]*SK_MY[3] + P[7][18]*SK_MY[4]);
			Kfusion[8] = SK_MY[0]*(P[8][20] + P[8][0]*SH_MAG[2] + P[8][1]*SH_MAG[1] + P[8][2]*SH_MAG[0] - P[8][3]*SK_MY[2] - P[8][17]*SK_MY[1] - P[8][16]*SK_MY[3] + P[8][18]*SK_MY[4]);
			Kfusion[9] = SK_MY[0]*(P[9][20] + P[9][0]*SH_MAG[2] + P[9][1]*SH_MAG[1] + P[9][2]*SH_MAG[0] - P[9][3]*SK_MY[2] - P[9][17]*SK_MY[1] - P[9][16]*SK_MY[3] + P[9][18]*SK_MY[4]);
			Kfusion[10] = SK_MY[0]*(P[10][20] + P[10][0]*SH_MAG[2] + P[10][1]*SH_MAG[1] + P[10][2]*SH_MAG[0] - P[10][3]*SK_MY[2] - P[10][17]*SK_MY[1] - P[10][16]*SK_MY[3] + P[10][18]*SK_MY[4]);
			Kfusion[11] = SK_MY[0]*(P[11][20] + P[11][0]*SH_MAG[2] + P[11][1]*SH_MAG[1] + P[11][2]*SH_MAG[0] - P[11][3]*SK_MY[2] - P[11][17]*SK_MY[1] - P[11][16]*SK_MY[3] + P[11][18]*SK_MY[4]);
			Kfusion[12] = SK_MY[0]*(P[12][20] + P[12][0]*SH_MAG[2] + P[12][1]*SH_MAG[1] + P[12][2]*SH_MAG[0] - P[12][3]*SK_MY[2] - P[12][17]*SK_MY[1] - P[12][16]*SK_MY[3] + P[12][18]*SK_MY[4]);
			Kfusion[13] = SK_MY[0]*(P[13][20] + P[13][0]*SH_MAG[2] + P[13][1]*SH_MAG[1] + P[13][2]*SH_MAG[0] - P[13][3]*SK_MY[2] - P[13][17]*SK_MY[1] - P[13][16]*SK_MY[3] + P[13][18]*SK_MY[4]);
			Kfusion[14] = SK_MY[0]*(P[14][20] + P[14][0]*SH_MAG[2] + P[14][1]*SH_MAG[1] + P[14][2]*SH_MAG[0] - P[14][3]*SK_MY[2] - P[14][17]*SK_MY[1] - P[14][16]*SK_MY[3] + P[14][18]*SK_MY[4]);
			Kfusion[15] = SK_MY[0]*(P[15][20] + P[15][0]*SH_MAG[2] + P[15][1]*SH_MAG[1] + P[15][2]*SH_MAG[0] - P[15][3]*SK_MY[2] - P[15][17]*SK_MY[1] - P[15][16]*SK_MY[3] + P[15][18]*SK_MY[4]);
			Kfusion[16] = SK_MY[0]*(P[16][20] + P[16][0]*SH_MAG[2] + P[16][1]*SH_MAG[1] + P[16][2]*SH_MAG[0] - P[16][3]*SK_MY[2] - P[16][17]*SK_MY[1] - P[16][16]*SK_MY[3] + P[16][18]*SK_MY[4]);
			Kfusion[17] = SK_MY[0]*(P[17][20] + P[17][0]*SH_MAG[2] + P[17][1]*SH_MAG[1] + P[17][2]*SH_MAG[0] - P[17][3]*SK_MY[2] - P[17][17]*SK_MY[1] - P[17][16]*SK_MY[3] + P[17][18]*SK_MY[4]);
			Kfusion[18] = SK_MY[0]*(P[18][20] + P[18][0]*SH_MAG[2] + P[18][1]*SH_MAG[1] + P[18][2]*SH_MAG[0] - P[18][3]*SK_MY[2] - P[18][17]*SK_MY[1] - P[18][16]*SK_MY[3] + P[18][18]*SK_MY[4]);
			Kfusion[19] = SK_MY[0]*(P[19][20] + P[19][0]*SH_MAG[2] + P[19][1]*SH_MAG[1] + P[19][2]*SH_MAG[0] - P[19][3]*SK_MY[2] - P[19][17]*SK_MY[1] - P[19][16]*SK_MY[3] + P[19][18]*SK_MY[4]);
			Kfusion[20] = SK_MY[0]*(P[20][20] + P[20][0]*SH_MAG[2] + P[20][1]*SH_MAG[1] + P[20][2]*SH_MAG[0] - P[20][3]*SK_MY[2] - P[20][17]*SK_MY[1] - P[20][16]*SK_MY[3] + P[20][18]*SK_MY[4]);
			Kfusion[21] = SK_MY[0]*(P[21][20] + P[21][0]*SH_MAG[2] + P[21][1]*SH_MAG[1] + P[21][2]*SH_MAG[0] - P[21][3]*SK_MY[2] - P[21][17]*SK_MY[1] - P[21][16]*SK_MY[3] + P[21][18]*SK_MY[4]);

			// Calculate the observation innovation variance
            varInnovMag[1] = 1.0f/SK_MY[0];

            // set flags to indicate to other processes that fusion has been perfomred and is  required on the next time step
            magFusePerformed = true;
            magFuseRequired = true;
        }
        else if (obsIndex == 2) // we are now fusing the Z measurement
        {
            // Calculate observation jacobians
            for (uint8_t i=0; i<=21; i++) H_MAG[i] = 0;
			H_MAG[0] = SH_MAG[1];
			H_MAG[1] = 2*magN*q3 - 2*magE*q0 - 2*magD*q1;
			H_MAG[2] = SH_MAG[7] + SH_MAG[8] - 2*magD*q2;
			H_MAG[3] = SH_MAG[0];
			H_MAG[16] = 2*q0*q2 + 2*q1*q3;
			H_MAG[17] = 2*q2*q3 - 2*q0*q1;
			H_MAG[18] = SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6];
			H_MAG[21] = 1;

            // Calculate Kalman gain
			SK_MZ[0] = 1/(P[21][21] + R_MAG + P[0][21]*SH_MAG[1] + P[3][21]*SH_MAG[0] + P[18][21]*(SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6]) - (2*magD*q1 + 2*magE*q0 - 2*magN*q3)*(P[21][1] + P[0][1]*SH_MAG[1] + P[3][1]*SH_MAG[0] + P[18][1]*(SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6]) + P[16][1]*(2*q0*q2 + 2*q1*q3) - P[17][1]*(2*q0*q1 - 2*q2*q3) - P[1][1]*(2*magD*q1 + 2*magE*q0 - 2*magN*q3) + P[2][1]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) + (SH_MAG[7] + SH_MAG[8] - 2*magD*q2)*(P[21][2] + P[0][2]*SH_MAG[1] + P[3][2]*SH_MAG[0] + P[18][2]*(SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6]) + P[16][2]*(2*q0*q2 + 2*q1*q3) - P[17][2]*(2*q0*q1 - 2*q2*q3) - P[1][2]*(2*magD*q1 + 2*magE*q0 - 2*magN*q3) + P[2][2]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) + SH_MAG[1]*(P[21][0] + P[0][0]*SH_MAG[1] + P[3][0]*SH_MAG[0] + P[18][0]*(SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6]) + P[16][0]*(2*q0*q2 + 2*q1*q3) - P[17][0]*(2*q0*q1 - 2*q2*q3) - P[1][0]*(2*magD*q1 + 2*magE*q0 - 2*magN*q3) + P[2][0]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) + SH_MAG[0]*(P[21][3] + P[0][3]*SH_MAG[1] + P[3][3]*SH_MAG[0] + P[18][3]*(SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6]) + P[16][3]*(2*q0*q2 + 2*q1*q3) - P[17][3]*(2*q0*q1 - 2*q2*q3) - P[1][3]*(2*magD*q1 + 2*magE*q0 - 2*magN*q3) + P[2][3]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) + (SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6])*(P[21][18] + P[0][18]*SH_MAG[1] + P[3][18]*SH_MAG[0] + P[18][18]*(SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6]) + P[16][18]*(2*q0*q2 + 2*q1*q3) - P[17][18]*(2*q0*q1 - 2*q2*q3) - P[1][18]*(2*magD*q1 + 2*magE*q0 - 2*magN*q3) + P[2][18]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) + P[16][21]*(2*q0*q2 + 2*q1*q3) - P[17][21]*(2*q0*q1 - 2*q2*q3) - P[1][21]*(2*magD*q1 + 2*magE*q0 - 2*magN*q3) + (2*q0*q2 + 2*q1*q3)*(P[21][16] + P[0][16]*SH_MAG[1] + P[3][16]*SH_MAG[0] + P[18][16]*(SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6]) + P[16][16]*(2*q0*q2 + 2*q1*q3) - P[17][16]*(2*q0*q1 - 2*q2*q3) - P[1][16]*(2*magD*q1 + 2*magE*q0 - 2*magN*q3) + P[2][16]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) - (2*q0*q1 - 2*q2*q3)*(P[21][17] + P[0][17]*SH_MAG[1] + P[3][17]*SH_MAG[0] + P[18][17]*(SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6]) + P[16][17]*(2*q0*q2 + 2*q1*q3) - P[17][17]*(2*q0*q1 - 2*q2*q3) - P[1][17]*(2*magD*q1 + 2*magE*q0 - 2*magN*q3) + P[2][17]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) + P[2][21]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2));
			SK_MZ[1] = SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6];
			SK_MZ[2] = 2*magD*q1 + 2*magE*q0 - 2*magN*q3;
			SK_MZ[3] = SH_MAG[7] + SH_MAG[8] - 2*magD*q2;
			SK_MZ[4] = 2*q0*q1 - 2*q2*q3;
			SK_MZ[5] = 2*q0*q2 + 2*q1*q3;
			Kfusion[0] = SK_MZ[0]*(P[0][21] + P[0][0]*SH_MAG[1] + P[0][3]*SH_MAG[0] - P[0][1]*SK_MZ[2] + P[0][2]*SK_MZ[3] + P[0][18]*SK_MZ[1] + P[0][16]*SK_MZ[5] - P[0][17]*SK_MZ[4]);
			Kfusion[1] = SK_MZ[0]*(P[1][21] + P[1][0]*SH_MAG[1] + P[1][3]*SH_MAG[0] - P[1][1]*SK_MZ[2] + P[1][2]*SK_MZ[3] + P[1][18]*SK_MZ[1] + P[1][16]*SK_MZ[5] - P[1][17]*SK_MZ[4]);
			Kfusion[2] = SK_MZ[0]*(P[2][21] + P[2][0]*SH_MAG[1] + P[2][3]*SH_MAG[0] - P[2][1]*SK_MZ[2] + P[2][2]*SK_MZ[3] + P[2][18]*SK_MZ[1] + P[2][16]*SK_MZ[5] - P[2][17]*SK_MZ[4]);
			Kfusion[3] = SK_MZ[0]*(P[3][21] + P[3][0]*SH_MAG[1] + P[3][3]*SH_MAG[0] - P[3][1]*SK_MZ[2] + P[3][2]*SK_MZ[3] + P[3][18]*SK_MZ[1] + P[3][16]*SK_MZ[5] - P[3][17]*SK_MZ[4]);
			Kfusion[4] = SK_MZ[0]*(P[4][21] + P[4][0]*SH_MAG[1] + P[4][3]*SH_MAG[0] - P[4][1]*SK_MZ[2] + P[4][2]*SK_MZ[3] + P[4][18]*SK_MZ[1] + P[4][16]*SK_MZ[5] - P[4][17]*SK_MZ[4]);
			Kfusion[5] = SK_MZ[0]*(P[5][21] + P[5][0]*SH_MAG[1] + P[5][3]*SH_MAG[0] - P[5][1]*SK_MZ[2] + P[5][2]*SK_MZ[3] + P[5][18]*SK_MZ[1] + P[5][16]*SK_MZ[5] - P[5][17]*SK_MZ[4]);
			Kfusion[6] = SK_MZ[0]*(P[6][21] + P[6][0]*SH_MAG[1] + P[6][3]*SH_MAG[0] - P[6][1]*SK_MZ[2] + P[6][2]*SK_MZ[3] + P[6][18]*SK_MZ[1] + P[6][16]*SK_MZ[5] - P[6][17]*SK_MZ[4]);
			Kfusion[7] = SK_MZ[0]*(P[7][21] + P[7][0]*SH_MAG[1] + P[7][3]*SH_MAG[0] - P[7][1]*SK_MZ[2] + P[7][2]*SK_MZ[3] + P[7][18]*SK_MZ[1] + P[7][16]*SK_MZ[5] - P[7][17]*SK_MZ[4]);
			Kfusion[8] = SK_MZ[0]*(P[8][21] + P[8][0]*SH_MAG[1] + P[8][3]*SH_MAG[0] - P[8][1]*SK_MZ[2] + P[8][2]*SK_MZ[3] + P[8][18]*SK_MZ[1] + P[8][16]*SK_MZ[5] - P[8][17]*SK_MZ[4]);
			Kfusion[9] = SK_MZ[0]*(P[9][21] + P[9][0]*SH_MAG[1] + P[9][3]*SH_MAG[0] - P[9][1]*SK_MZ[2] + P[9][2]*SK_MZ[3] + P[9][18]*SK_MZ[1] + P[9][16]*SK_MZ[5] - P[9][17]*SK_MZ[4]);
			Kfusion[10] = SK_MZ[0]*(P[10][21] + P[10][0]*SH_MAG[1] + P[10][3]*SH_MAG[0] - P[10][1]*SK_MZ[2] + P[10][2]*SK_MZ[3] + P[10][18]*SK_MZ[1] + P[10][16]*SK_MZ[5] - P[10][17]*SK_MZ[4]);
			Kfusion[11] = SK_MZ[0]*(P[11][21] + P[11][0]*SH_MAG[1] + P[11][3]*SH_MAG[0] - P[11][1]*SK_MZ[2] + P[11][2]*SK_MZ[3] + P[11][18]*SK_MZ[1] + P[11][16]*SK_MZ[5] - P[11][17]*SK_MZ[4]);
			Kfusion[12] = SK_MZ[0]*(P[12][21] + P[12][0]*SH_MAG[1] + P[12][3]*SH_MAG[0] - P[12][1]*SK_MZ[2] + P[12][2]*SK_MZ[3] + P[12][18]*SK_MZ[1] + P[12][16]*SK_MZ[5] - P[12][17]*SK_MZ[4]);
			Kfusion[13] = SK_MZ[0]*(P[13][21] + P[13][0]*SH_MAG[1] + P[13][3]*SH_MAG[0] - P[13][1]*SK_MZ[2] + P[13][2]*SK_MZ[3] + P[13][18]*SK_MZ[1] + P[13][16]*SK_MZ[5] - P[13][17]*SK_MZ[4]);
			Kfusion[14] = SK_MZ[0]*(P[14][21] + P[14][0]*SH_MAG[1] + P[14][3]*SH_MAG[0] - P[14][1]*SK_MZ[2] + P[14][2]*SK_MZ[3] + P[14][18]*SK_MZ[1] + P[14][16]*SK_MZ[5] - P[14][17]*SK_MZ[4]);
			Kfusion[15] = SK_MZ[0]*(P[15][21] + P[15][0]*SH_MAG[1] + P[15][3]*SH_MAG[0] - P[15][1]*SK_MZ[2] + P[15][2]*SK_MZ[3] + P[15][18]*SK_MZ[1] + P[15][16]*SK_MZ[5] - P[15][17]*SK_MZ[4]);
			Kfusion[16] = SK_MZ[0]*(P[16][21] + P[16][0]*SH_MAG[1] + P[16][3]*SH_MAG[0] - P[16][1]*SK_MZ[2] + P[16][2]*SK_MZ[3] + P[16][18]*SK_MZ[1] + P[16][16]*SK_MZ[5] - P[16][17]*SK_MZ[4]);
			Kfusion[17] = SK_MZ[0]*(P[17][21] + P[17][0]*SH_MAG[1] + P[17][3]*SH_MAG[0] - P[17][1]*SK_MZ[2] + P[17][2]*SK_MZ[3] + P[17][18]*SK_MZ[1] + P[17][16]*SK_MZ[5] - P[17][17]*SK_MZ[4]);
			Kfusion[18] = SK_MZ[0]*(P[18][21] + P[18][0]*SH_MAG[1] + P[18][3]*SH_MAG[0] - P[18][1]*SK_MZ[2] + P[18][2]*SK_MZ[3] + P[18][18]*SK_MZ[1] + P[18][16]*SK_MZ[5] - P[18][17]*SK_MZ[4]);
			Kfusion[19] = SK_MZ[0]*(P[19][21] + P[19][0]*SH_MAG[1] + P[19][3]*SH_MAG[0] - P[19][1]*SK_MZ[2] + P[19][2]*SK_MZ[3] + P[19][18]*SK_MZ[1] + P[19][16]*SK_MZ[5] - P[19][17]*SK_MZ[4]);
			Kfusion[20] = SK_MZ[0]*(P[20][21] + P[20][0]*SH_MAG[1] + P[20][3]*SH_MAG[0] - P[20][1]*SK_MZ[2] + P[20][2]*SK_MZ[3] + P[20][18]*SK_MZ[1] + P[20][16]*SK_MZ[5] - P[20][17]*SK_MZ[4]);
			Kfusion[21] = SK_MZ[0]*(P[21][21] + P[21][0]*SH_MAG[1] + P[21][3]*SH_MAG[0] - P[21][1]*SK_MZ[2] + P[21][2]*SK_MZ[3] + P[21][18]*SK_MZ[1] + P[21][16]*SK_MZ[5] - P[21][17]*SK_MZ[4]);

			// Calculate the observation innovation variance
            varInnovMag[2] = 1.0f/SK_MZ[0];

            // set flags to indicate to other processes that fusion has been perfomred and is not required on the next time step
            magFusePerformed = true;
            magFuseRequired = false;
        }
        // Calculate the measurement innovation
        innovMag[obsIndex] = MagPred[obsIndex] - magData[obsIndex];
        // Apply and innovation consistency check
        if ((innovMag[obsIndex]*innovMag[obsIndex]/varInnovMag[obsIndex]) < sq(_magInnovGate))
        {
            // correct the state vector
            for (uint8_t j= 0; j<=indexLimit; j++)
            {
                states[j] = states[j] - Kfusion[j] * innovMag[obsIndex];
            }
            // normalise the quaternion states
            float quatMag = sqrtf(states[0]*states[0] + states[1]*states[1] + states[2]*states[2] + states[3]*states[3]);
            if (quatMag > 1e-12f)
            {
                for (uint8_t j= 0; j<=3; j++)
                {
                    float quatMagInv = 1.0f/quatMag;
                    states[j] = states[j] * quatMagInv;
                }
            }
            // correct the covariance P = (I - K*H)*P
            // take advantage of the empty columns in KH to reduce the
            // number of operations
            for (uint8_t i = 0; i<=indexLimit; i++)
            {
                for (uint8_t j = 0; j<=3; j++)
                {
                    KH[i][j] = Kfusion[i] * H_MAG[j];
                }
                for (uint8_t j = 4; j<=15; j++) KH[i][j] = 0.0f;
                if (!onGround)
                {
                    for (uint8_t j = 16; j<=21; j++)
                    {
                        KH[i][j] = Kfusion[i] * H_MAG[j];
                    }
                }
                else
                {
                    for (uint8_t j = 16; j<=21; j++)
                    {
                        KH[i][j] = 0.0f;
                    }
                }
            }
            for (uint8_t i = 0; i<=indexLimit; i++)
            {
                for (uint8_t j = 0; j<=indexLimit; j++)
                {
                    KHP[i][j] = 0;
                    for (uint8_t k = 0; k<=3; k++)
                    {
                        KHP[i][j] = KHP[i][j] + KH[i][k] * P[k][j];
                    }
                    if (!onGround)
                    {
                        for (uint8_t k = 16; k<=21; k++)
                        {
                            KHP[i][j] = KHP[i][j] + KH[i][k] * P[k][j];
                        }
                    }
                }
            }
            for (uint8_t i = 0; i<=indexLimit; i++)
            {
                for (uint8_t j = 0; j<=indexLimit; j++)
                {
                    P[i][j] = P[i][j] - KHP[i][j];
                }
            }
        }
        obsIndex = obsIndex + 1;
    }
    else
    {
        // set flags to indicate to other processes that fusion has not been performed and is not required on the next time step
    magFusePerformed = false;
    magFuseRequired = false;
    }

    // force the covariance matrix to me symmetrical and limit the variances to prevent
    // ill-condiioning.
    ForceSymmetry();
    ConstrainVariances();

    perf_end(_perf_FuseMagnetometer);
}

void NavEKF::FuseAirspeed()
{
    perf_begin(_perf_FuseAirspeed);
    float vn;
    float ve;
    float vd;
    float vwn;
    float vwe;
    float EAS2TAS = _ahrs->get_EAS2TAS();
    const float R_TAS = sq(_easNoise * constrain_float(EAS2TAS, 0.9f, 10.0f));
    Vector3f SH_TAS;
    float SK_TAS;
    Vector22 H_TAS;
    float VtasPred;

    // Copy required states to local variable names
    vn = statesAtVtasMeasTime[4];
    ve = statesAtVtasMeasTime[5];
    vd = statesAtVtasMeasTime[6];
    vwn = statesAtVtasMeasTime[14];
    vwe = statesAtVtasMeasTime[15];

    // Calculate the predicted airspeed
    VtasPred = sqrtf((ve - vwe)*(ve - vwe) + (vn - vwn)*(vn - vwn) + vd*vd);
    // Perform fusion of True Airspeed measurement
    if (VtasPred > 1.0f)
    {
        // Calculate observation jacobians
		SH_TAS[0] = 1.0f/(sqrt(sq(ve - vwe) + sq(vn - vwn) + sq(vd)));
		SH_TAS[1] = (SH_TAS[0]*(2*ve - 2*vwe))/2;
		SH_TAS[2] = (SH_TAS[0]*(2*vn - 2*vwn))/2;
        for (uint8_t i=0; i<=21; i++) H_TAS[i] = 0.0f;
		H_TAS[4] = SH_TAS[2];
		H_TAS[5] = SH_TAS[1];
		H_TAS[6] = vd*SH_TAS[0];
		H_TAS[14] = -SH_TAS[2];
		H_TAS[15] = -SH_TAS[1];

        // Calculate Kalman gains
		SK_TAS = 1.0f/(R_TAS + SH_TAS[2]*(P[4][4]*SH_TAS[2] + P[5][4]*SH_TAS[1] - P[14][4]*SH_TAS[2] - P[15][4]*SH_TAS[1] + P[6][4]*vd*SH_TAS[0]) + SH_TAS[1]*(P[4][5]*SH_TAS[2] + P[5][5]*SH_TAS[1] - P[14][5]*SH_TAS[2] - P[15][5]*SH_TAS[1] + P[6][5]*vd*SH_TAS[0]) - SH_TAS[2]*(P[4][14]*SH_TAS[2] + P[5][14]*SH_TAS[1] - P[14][14]*SH_TAS[2] - P[15][14]*SH_TAS[1] + P[6][14]*vd*SH_TAS[0]) - SH_TAS[1]*(P[4][15]*SH_TAS[2] + P[5][15]*SH_TAS[1] - P[14][15]*SH_TAS[2] - P[15][15]*SH_TAS[1] + P[6][15]*vd*SH_TAS[0]) + vd*SH_TAS[0]*(P[4][6]*SH_TAS[2] + P[5][6]*SH_TAS[1] - P[14][6]*SH_TAS[2] - P[15][6]*SH_TAS[1] + P[6][6]*vd*SH_TAS[0]));
		Kfusion[0] = SK_TAS*(P[0][4]*SH_TAS[2] - P[0][14]*SH_TAS[2] + P[0][5]*SH_TAS[1] - P[0][15]*SH_TAS[1] + P[0][6]*vd*SH_TAS[0]);
		Kfusion[1] = SK_TAS*(P[1][4]*SH_TAS[2] - P[1][14]*SH_TAS[2] + P[1][5]*SH_TAS[1] - P[1][15]*SH_TAS[1] + P[1][6]*vd*SH_TAS[0]);
		Kfusion[2] = SK_TAS*(P[2][4]*SH_TAS[2] - P[2][14]*SH_TAS[2] + P[2][5]*SH_TAS[1] - P[2][15]*SH_TAS[1] + P[2][6]*vd*SH_TAS[0]);
		Kfusion[3] = SK_TAS*(P[3][4]*SH_TAS[2] - P[3][14]*SH_TAS[2] + P[3][5]*SH_TAS[1] - P[3][15]*SH_TAS[1] + P[3][6]*vd*SH_TAS[0]);
		Kfusion[4] = SK_TAS*(P[4][4]*SH_TAS[2] - P[4][14]*SH_TAS[2] + P[4][5]*SH_TAS[1] - P[4][15]*SH_TAS[1] + P[4][6]*vd*SH_TAS[0]);
		Kfusion[5] = SK_TAS*(P[5][4]*SH_TAS[2] - P[5][14]*SH_TAS[2] + P[5][5]*SH_TAS[1] - P[5][15]*SH_TAS[1] + P[5][6]*vd*SH_TAS[0]);
		Kfusion[6] = SK_TAS*(P[6][4]*SH_TAS[2] - P[6][14]*SH_TAS[2] + P[6][5]*SH_TAS[1] - P[6][15]*SH_TAS[1] + P[6][6]*vd*SH_TAS[0]);
		Kfusion[7] = SK_TAS*(P[7][4]*SH_TAS[2] - P[7][14]*SH_TAS[2] + P[7][5]*SH_TAS[1] - P[7][15]*SH_TAS[1] + P[7][6]*vd*SH_TAS[0]);
		Kfusion[8] = SK_TAS*(P[8][4]*SH_TAS[2] - P[8][14]*SH_TAS[2] + P[8][5]*SH_TAS[1] - P[8][15]*SH_TAS[1] + P[8][6]*vd*SH_TAS[0]);
		Kfusion[9] = SK_TAS*(P[9][4]*SH_TAS[2] - P[9][14]*SH_TAS[2] + P[9][5]*SH_TAS[1] - P[9][15]*SH_TAS[1] + P[9][6]*vd*SH_TAS[0]);
		Kfusion[10] = SK_TAS*(P[10][4]*SH_TAS[2] - P[10][14]*SH_TAS[2] + P[10][5]*SH_TAS[1] - P[10][15]*SH_TAS[1] + P[10][6]*vd*SH_TAS[0]);
		Kfusion[11] = SK_TAS*(P[11][4]*SH_TAS[2] - P[11][14]*SH_TAS[2] + P[11][5]*SH_TAS[1] - P[11][15]*SH_TAS[1] + P[11][6]*vd*SH_TAS[0]);
		Kfusion[12] = SK_TAS*(P[12][4]*SH_TAS[2] - P[12][14]*SH_TAS[2] + P[12][5]*SH_TAS[1] - P[12][15]*SH_TAS[1] + P[12][6]*vd*SH_TAS[0]);
		Kfusion[13] = SK_TAS*(P[13][4]*SH_TAS[2] - P[13][14]*SH_TAS[2] + P[13][5]*SH_TAS[1] - P[13][15]*SH_TAS[1] + P[13][6]*vd*SH_TAS[0]);
		Kfusion[14] = SK_TAS*(P[14][4]*SH_TAS[2] - P[14][14]*SH_TAS[2] + P[14][5]*SH_TAS[1] - P[14][15]*SH_TAS[1] + P[14][6]*vd*SH_TAS[0]);
		Kfusion[15] = SK_TAS*(P[15][4]*SH_TAS[2] - P[15][14]*SH_TAS[2] + P[15][5]*SH_TAS[1] - P[15][15]*SH_TAS[1] + P[15][6]*vd*SH_TAS[0]);
		Kfusion[16] = SK_TAS*(P[16][4]*SH_TAS[2] - P[16][14]*SH_TAS[2] + P[16][5]*SH_TAS[1] - P[16][15]*SH_TAS[1] + P[16][6]*vd*SH_TAS[0]);
		Kfusion[17] = SK_TAS*(P[17][4]*SH_TAS[2] - P[17][14]*SH_TAS[2] + P[17][5]*SH_TAS[1] - P[17][15]*SH_TAS[1] + P[17][6]*vd*SH_TAS[0]);
		Kfusion[18] = SK_TAS*(P[18][4]*SH_TAS[2] - P[18][14]*SH_TAS[2] + P[18][5]*SH_TAS[1] - P[18][15]*SH_TAS[1] + P[18][6]*vd*SH_TAS[0]);
		Kfusion[19] = SK_TAS*(P[19][4]*SH_TAS[2] - P[19][14]*SH_TAS[2] + P[19][5]*SH_TAS[1] - P[19][15]*SH_TAS[1] + P[19][6]*vd*SH_TAS[0]);
		Kfusion[20] = SK_TAS*(P[20][4]*SH_TAS[2] - P[20][14]*SH_TAS[2] + P[20][5]*SH_TAS[1] - P[20][15]*SH_TAS[1] + P[20][6]*vd*SH_TAS[0]);
		Kfusion[21] = SK_TAS*(P[21][4]*SH_TAS[2] - P[21][14]*SH_TAS[2] + P[21][5]*SH_TAS[1] - P[21][15]*SH_TAS[1] + P[21][6]*vd*SH_TAS[0]);

		// Calculate measurement innovation variance
        varInnovVtas = 1.0f/SK_TAS;

        // Calculate measurement innovation
        innovVtas = VtasPred - VtasMeas;

        // Aplly an innovation consistency check
        if ((innovVtas*innovVtas*SK_TAS) < sq(_tasInnovGate))
        {
            // correct the state vector
            for (uint8_t j=0; j<=21; j++)
            {
                states[j] = states[j] - Kfusion[j] * innovVtas;
            }

            Quaternion q(states[0], states[1], states[2], states[3]);
            q.normalize();
            for (uint8_t i = 0; i<=3; i++) {
                states[i] = q[i];
            }
            // correct the covariance P = (I - K*H)*P
            // take advantage of the empty columns in H to reduce the
            // number of operations
            for (uint8_t i = 0; i<=21; i++)
            {
                for (uint8_t j = 0; j<=3; j++) KH[i][j] = 0.0;
                for (uint8_t j = 4; j<=6; j++)
                {
                    KH[i][j] = Kfusion[i] * H_TAS[j];
                }
                for (uint8_t j = 7; j<=13; j++) KH[i][j] = 0.0;
                for (uint8_t j = 14; j<=15; j++)
                {
                    KH[i][j] = Kfusion[i] * H_TAS[j];
                }
                for (uint8_t j = 16; j<=21; j++) KH[i][j] = 0.0;
            }
            for (uint8_t i = 0; i<=21; i++)
            {
                for (uint8_t j = 0; j<=21; j++)
                {
                    KHP[i][j] = 0;
                    for (uint8_t k = 4; k<=6; k++)
                    {
                        KHP[i][j] = KHP[i][j] + KH[i][k] * P[k][j];
                    }
                    for (uint8_t k = 14; k<=15; k++)
                    {
                        KHP[i][j] = KHP[i][j] + KH[i][k] * P[k][j];
                    }
                }
            }
            for (uint8_t i = 0; i<=21; i++)
            {
                for (uint8_t j = 0; j<=21; j++)
                {
                    P[i][j] = P[i][j] - KHP[i][j];
                }
            }
        }
    }

    // force the covariance matrix to me symmetrical and limit the variances to prevent
    // ill-condiioning.
    ForceSymmetry();
    ConstrainVariances();

    perf_end(_perf_FuseAirspeed);
}

void NavEKF::zeroRows(Matrix22 &covMat, uint8_t first, uint8_t last)
{
    uint8_t row;
    for (row=first; row<=last; row++)
    {
        memset(&covMat[row][0], 0, sizeof(covMat[0][0])*22);
    }
}

void NavEKF::zeroCols(Matrix22 &covMat, uint8_t first, uint8_t last)
{
    uint8_t row;
    for (row=0; row<=21; row++)
    {
        memset(&covMat[row][first], 0, sizeof(covMat[0][0])*(1+last-first));
    }
}

// Store states in a history array along with time stamp
void NavEKF::StoreStates()
{
    if (storeIndex > 49) storeIndex = 0;
    for (uint8_t i=0; i<=21; i++) storedStates[i][storeIndex] = states[i];
    statetimeStamp[storeIndex] = hal.scheduler->millis();
    storeIndex = storeIndex + 1;
}

// Output the state vector stored at the time that best matches that specified by msec
void NavEKF::RecallStates(Vector22 &statesForFusion, uint32_t msec)
{
    uint32_t timeDelta;
    uint32_t bestTimeDelta = 200;
    uint8_t bestStoreIndex = 0;
    for (uint8_t i=0; i<=49; i++)
    {
        timeDelta = msec - statetimeStamp[i];
        if (timeDelta < bestTimeDelta)
        {
            bestStoreIndex = i;
            bestTimeDelta = timeDelta;
        }
    }
    if (bestTimeDelta < 200) // only output stored state if < 200 msec retrieval error
    {
        for (uint8_t i=0; i<=21; i++) {
            statesForFusion[i] = storedStates[i][bestStoreIndex];
        }
    }
    else // otherwise output current state
    {
        for (uint8_t i=0; i<=21; i++) {
            statesForFusion[i] = states[i];
        }
    }
}

void NavEKF::quat2Tbn(Matrix3f &Tbn, const Quaternion &quat) const
{
    // Calculate the body to nav cosine matrix
    quat.rotation_matrix(Tbn);
}

void NavEKF::getEulerAngles(Vector3f &euler) const
{
    Quaternion q(states[0], states[1], states[2], states[3]);
    q.to_euler(&euler.x, &euler.y, &euler.z);
}

void NavEKF::getVelNED(Vector3f &vel) const
{
    vel.x = states[4];
    vel.y = states[5];
    vel.z = states[6];
}

bool NavEKF::getPosNED(Vector3f &pos) const
{
    pos.x = states[7];
    pos.y = states[8];
    pos.z = states[9];
    return true;
}

void NavEKF::getGyroBias(Vector3f &gyroBias) const
{
    gyroBias.x = states[10] / dtIMU;
    gyroBias.y = states[11] / dtIMU;
    gyroBias.z = states[12] / dtIMU;
}

void NavEKF::getAccelBias(Vector3f &accelBias) const
{
    accelBias.x = 0.0f;
    accelBias.y = 0.0f;
    accelBias.z = states[13] / dtIMU;
}

void NavEKF::getWind(Vector3f &wind) const
{
    wind.x = states[14];
    wind.y = states[15];
    wind.z = 0.0f; // curently don't estimate this
}

void NavEKF::getMagNED(Vector3f &magNED) const
{
    magNED.x = states[16]*1000.0f;
    magNED.y = states[17]*1000.0f;
    magNED.z = states[18]*1000.0f;
}

void NavEKF::getMagXYZ(Vector3f &magXYZ) const
{
    magXYZ.x = states[19]*1000.0f;
    magXYZ.y = states[20]*1000.0f;
    magXYZ.z = states[21]*1000.0f;
}

bool NavEKF::getLLH(struct Location &loc) const
{
    loc.lat = _ahrs->get_home().lat;
    loc.lng = _ahrs->get_home().lng;
    loc.alt = _ahrs->get_home().alt - states[9]*100;
    location_offset(loc, states[7], states[8]);
    return true;
}

void NavEKF::OnGroundCheck()
{
    const AP_Airspeed *airspeed = _ahrs->get_airspeed();
    uint8_t lowAirSpd = (!airspeed || !airspeed->use() || airspeed->get_airspeed() * airspeed->get_EAS2TAS() < 8.0f);
    uint8_t lowGndSpd = (uint8_t)((sq(velNED[0]) + sq(velNED[1]) + sq(velNED[2])) < 4.0f);
    uint8_t lowHgt = (uint8_t)(fabsf(hgtMea < 15.0f));
    // Go with a majority vote from three criteria
    onGround = ((lowAirSpd + lowGndSpd + lowHgt) >= 2);
}

void NavEKF::CovarianceInit(float roll, float pitch, float yaw)
{
    //TODO better maths for initial quaternion covariances
    // that uses roll, pitch and yaw
    // zero the matrix
    for (uint8_t i=1; i<=21; i++)
    {
        for (uint8_t j=0; j<=21; j++)
        {
            P[i][j] = 0.0f;
        }
    }
	// Quaternions
    P[0][0]   = 1.0e-9f;
    P[1][1]   = 0.25f*sq(radians(1.0f));
    P[2][2]   = 0.25f*sq(radians(1.0f));
    P[3][3]   = 0.25f*sq(radians(1.0f));
	// Velocities
    P[4][4]   = sq(0.7f);
    P[5][5]   = P[4][4];
    P[6][6]   = sq(0.7f);
	// Positions
    P[7][7]   = sq(15.0f);
    P[8][8]   = P[7][7];
    P[9][9]   = sq(5.0f);
	// Delta angle biases
    P[10][10] = sq(radians(0.1f * dtIMU));
    P[11][11] = P[10][10];
    P[12][12] = P[10][10];
	//Z delta velocity Bbias
	P[13][13] = sq(radians(0.5f * dtIMU));
	// Wind velocities
    P[14][14] = sq(8.0f);
    P[15][15]  = P[14][14];
	// NED magnetic field
    P[16][16] = sq(0.02f);
    P[17][17] = P[16][15];
    P[18][18] = P[16][16];
	// XYZ magnetic field
    P[19][19] = sq(0.02f);
    P[20][20] = P[19][19];
    P[21][21] = P[19][19];
}

void NavEKF::ForceSymmetry()
{
    // Force symmetry on the covariance matrix to prevent ill-conditioning
    // of the matrix which would cause the filter to blow-up
    for (uint8_t i=1; i<=21; i++)
    {
        for (uint8_t j=0; j<=i-1; j++)
        {
            float temp = 0.5f*(P[i][j] + P[j][i]);
            P[i][j] = temp;
            P[j][i] = temp;
        }
    }
}

void NavEKF::ConstrainVariances()
{
    // Constrain variances to be within set limits
    for (uint8_t i=0; i<=3; i++) P[i][i] = constrain_float(P[i][i],0.0f,1.0f);
    for (uint8_t i=4; i<=6; i++) P[i][i] = constrain_float(P[i][i],0.0f,1.0e3f);
    for (uint8_t i=7; i<=9; i++) P[i][i] = constrain_float(P[i][i],0.0f,1.0e5f);
    for (uint8_t i=10; i<=12; i++) P[i][i] = constrain_float(P[i][i],0.0f,sq(0.175 * dtIMU));
	P[13][13] = constrain_float(P[13][13],0.0f,sq(10.0 * dtIMU));
    for (uint8_t i=14; i<=15; i++) P[i][i] = constrain_float(P[i][i],0.0f,1.0e3f);
    for (uint8_t i=16; i<=21; i++) P[i][i] = constrain_float(P[i][i],0.0f,1.0f);
}

void NavEKF::ConstrainStates()
{
    // Constrain states to be within set limits
    // Quaternions
    for (uint8_t i=0; i<=3; i++) states[i] = constrain_float(states[i],-1.0f,1.0f);
    // velocity limit 500 m/sec (could set this based on some multiple of max airspeed * EAS2TAS)
    for (uint8_t i=4; i<=6; i++) states[i] = constrain_float(states[i],-5.0e2f,5.0e2f);
    // position limit 1000 km
    //TODO apply circular limit
    for (uint8_t i=7; i<=8; i++) states[i] = constrain_float(states[i],-1.0e6f,1.0e6f);
    // height limit covers home alt on everest through to home alt at SL and ballon drop
    states[9] = constrain_float(states[9],-1.0e4f,4.0e4f);
    // gyro bias limit ~6 deg/sec (this needs to be set based on manufacturers specs)
    for (uint8_t i=10; i<=12; i++) states[i] = constrain_float(states[i],-0.1f*dtIMU,0.1f*dtIMU);
    // Z accel bias limit 0.5 m/s^2	(this neeeds to be set based on manufacturers specs)
    states[13] = constrain_float(states[13],-0.5f*dtIMU,0.5f*dtIMU);
    // Wind Limit 100 m/s (should be based on some multiple of max airspeed * EAS2TAS)
    //TODO apply circular limit
    for (uint8_t i=14; i<=15; i++) states[i] = constrain_float(states[i],-100.0f,100.0f);
    // Earth Field limit 1000 mGauss
    for (uint8_t i=16; i<=18; i++) states[i] = constrain_float(states[i],-1.0f,1.0f);
    // Body Field limit 500 mGauss
    for (uint8_t i=19; i<=21; i++) states[i] = constrain_float(states[i],-0.5f,0.5f);
}

void NavEKF::readIMUData()
{
    Vector3f angRate;     // angular rate vector in XYZ body axes measured by the IMU (rad/s)
    Vector3f accel;       // acceleration vector in XYZ body axes measured by the IMU (m/s^2)

    IMUmsec     = hal.scheduler->millis();
    dtIMU       = _ahrs->get_ins().get_delta_time();
    angRate     = _ahrs->get_ins().get_gyro();
    accel       = _ahrs->get_ins().get_accel();

    // trapezoidal integration
    dAngIMU     = (angRate + lastAngRate) * dtIMU * 0.5f;
    lastAngRate = angRate;
    dVelIMU     = (accel + lastAccel) * dtIMU * 0.5f;
    lastAccel   = accel;
}

void NavEKF::readGpsData()
{
    if ((_ahrs->get_gps()->last_message_time_ms() != lastFixTime_ms) &&
            (_ahrs->get_gps()->status() >= GPS::GPS_OK_FIX_3D))
    {
        lastFixTime_ms = _ahrs->get_gps()->last_message_time_ms();
        newDataGps = true;
        RecallStates(statesAtVelTime, (IMUmsec - _msecVelDelay));
        RecallStates(statesAtPosTime, (IMUmsec - _msecPosDelay));
        velNED[0] = _ahrs->get_gps()->velocity_north(); // (rad)
        velNED[1] = _ahrs->get_gps()->velocity_east(); // (m/s)
        velNED[2] = _ahrs->get_gps()->velocity_down(); // (m/s)
        //::printf("GPSVEL=(%.2f,%.2f,%.2f)\n", velNED[0], velNED[1], velNED[2]);

        // Convert GPS measurements to Pos NE
        struct Location gpsloc;
        gpsloc.lat = _ahrs->get_gps()->latitude;
        gpsloc.lng = _ahrs->get_gps()->longitude;
        Vector2f posdiff = location_diff(_ahrs->get_home(), gpsloc);

        posNE[0] = posdiff.x;
        posNE[1] = posdiff.y;
    }
}

void NavEKF::readHgtData()
{
    if (_baro.get_last_update() != lastHgtUpdate) {
        lastHgtUpdate = _baro.get_last_update();
        hgtMea = _baro.get_altitude();
        newDataHgt = true;
        // recall states from compass measurement time
        RecallStates(statesAtHgtTime, (IMUmsec - _msecHgtDelay));
    } else {
        newDataHgt = false;
    }
}

void NavEKF::readMagData()
{
    // scale compass data to improve numerical conditioning
    if (_ahrs->get_compass()->last_update != lastMagUpdate) {
        lastMagUpdate = _ahrs->get_compass()->last_update;

        magBias = -_ahrs->get_compass()->get_offsets() * 0.001f;
        magData = _ahrs->get_compass()->get_field() * 0.001f + magBias;

        // Recall states from compass measurement time
        RecallStates(statesAtMagMeasTime, (IMUmsec - _msecMagDelay));
        newDataMag = true;
    } else {
        newDataMag = false;
    }
}

void NavEKF::readAirSpdData()
{
    const AP_Airspeed *aspeed = _ahrs->get_airspeed();
    if (aspeed &&
        aspeed->use() &&
        aspeed->last_update_ms() != lastAirspeedUpdate) {
        VtasMeas = aspeed->get_airspeed() * aspeed->get_EAS2TAS();
        lastAirspeedUpdate = aspeed->last_update_ms();
        newDataTas = true;
        RecallStates(statesAtVtasMeasTime, (IMUmsec - _msecTasDelay));
    } else {
        newDataTas = false;
    }
}

void NavEKF::calcEarthRateNED(Vector3f &omega, int32_t latitude) const
{
    float lat_rad = radians(latitude*1.0e-7f);
    omega.x  = earthRate*cosf(lat_rad);
    omega.y  = 0;
    omega.z  = -earthRate*sinf(lat_rad);
}

void NavEKF::ForceYawAlignment()
{
    if ((sq(velNED[0]) + sq(velNED[1])) > 16.0f) {
        float roll;
        float pitch;
        float yaw;
        // get quaternion from existing filter states and calculate roll, pitch and yaw angles
        Quaternion initQuat;
        Quaternion newQuat;
        for (uint8_t i=0; i<=3; i++) initQuat[i] = states[i];
        initQuat.to_euler(&roll, &pitch, &yaw);
        // modify yaw angle from GPS ground course
        yaw = atan2f(velNED[1],velNED[0]);
        // Calculate new filter quaternion states from Euler angles
        newQuat.from_euler(roll, pitch, yaw);
        for (uint8_t i=0; i<=3; i++) states[i] = newQuat[i];
        // set the velocity states
        if (_fusionModeGPS < 2) {
            states[4] = velNED[0];
            states[5] = velNED[1];
        }
        // Reinitialise the quaternion, velocity and position covariances
        // zero the matrix entries
        zeroRows(P,0,9);
        zeroCols(P,0,9);
        // set quaternion variances
        // TODO - maths that sets them based on different roll, yaw and pitch uncertainties
        P[0][0]   = 0.25f*sq(radians(1.0f));
        P[1][1]   = P[0][0];
        P[2][2]   = P[0][0];
        P[3][3]   = P[0][0];
        // set velocty and position state variances
        // we could have a big error coming out of static mode due to GPS lag
        P[4][4]   = 400.0f; // assume 20 m/s
        P[5][5]   = P[4][4];
        P[6][6]   = P[4][4];
        P[7][7]   = 400.0f; // assume 20 m
        P[8][8]   = P[7][7];
        P[9][9]   = P[7][7];
    }
}

void NavEKF::getRotationBodyToNED(Matrix3f &mat) const
{
    Quaternion q(states[0], states[1], states[2], states[3]);
    q.rotation_matrix(mat);
}

void  NavEKF::getInnovations(Vector3f &velInnov, Vector3f &posInnov, Vector3f &magInnov, float &tasInnov) const
{
    velInnov.x = innovVelPos[0];
    velInnov.y = innovVelPos[1];
    velInnov.z = innovVelPos[2];
    posInnov.x = innovVelPos[3];
    posInnov.y = innovVelPos[4];
    posInnov.z = innovVelPos[5];
    magInnov.x = 1e3*innovMag[0]; // Convert back to sensor units
    magInnov.y = 1e3*innovMag[1]; // Convert back to sensor units
    magInnov.z = 1e3*innovMag[2]; // Convert back to sensor units
    tasInnov   = innovVtas;
}

void  NavEKF::getVariances(Vector3f &velVar, Vector3f &posVar, Vector3f &magVar, float &tasVar) const
{
    velVar.x = varInnovVelPos[0];
    velVar.y = varInnovVelPos[1];
    velVar.z = varInnovVelPos[2];
    posVar.x = varInnovVelPos[3];
    posVar.y = varInnovVelPos[4];
    posVar.z = varInnovVelPos[5];
    magVar.x = 1e6*varInnovMag[0]; // Convert back to sensor units
    magVar.y = 1e6*varInnovMag[1]; // Convert back to sensor units
    magVar.z = 1e6*varInnovMag[2]; // Convert back to sensor units
    tasVar   = varInnovVtas;
}

void NavEKF::ZeroVariables()
{
    velTimeout = false;
    posTimeout = false;
    hgtTimeout = false;
    lastFixTime_ms = 0;
    lastMagUpdate = 0;
    lastAirspeedUpdate = 0;
    velFailTime = 0;
    posFailTime = 0;
    hgtFailTime = 0;
    storeIndex = 0;
    TASmsecPrev = 0;
    MAGmsecPrev = 0;
    HGTmsecPrev = 0;
    lastMagUpdate = 0;
    lastAirspeedUpdate = 0;
    lastHgtUpdate = 0;
	lastFixTime_ms = 0;
    dtIMU = 0;
    dt = 0;
	prevDelAng.zero();
    lastAngRate.zero();
    lastAccel.zero();
    lastVelDotNED.zero();
    lastAngRate.zero();
    lastAccel.zero();
    summedDelAng.zero();
    summedDelVel.zero();
    prevTnb.zero();
    memset(&P[0][0], 0, sizeof(P));
    memset(&nextP[0][0], 0, sizeof(nextP));
    memset(&processNoise[0], 0, sizeof(processNoise));
    memset(&storedStates[0][0], 0, sizeof(storedStates));
    memset(&statetimeStamp[0], 0, sizeof(statetimeStamp));
}

#endif // HAL_CPU_CLASS