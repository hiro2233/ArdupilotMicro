/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include <AP_HAL.h>
#include <AP_Progmem.h>
#include "Compass.h"
#include <AP_Vehicle.h>

extern AP_HAL::HAL& hal;

#if APM_BUILD_TYPE(APM_BUILD_ArduCopter)
#define COMPASS_LEARN_DEFAULT 0
#else
#define COMPASS_LEARN_DEFAULT 1
#endif


// Default constructor.
// Note that the Vector/Matrix constructors already implicitly zero
// their values.
//
Compass::Compass(void) :
    _backend_count(0),
    _compass_count(0),
    _board_orientation(ROTATION_NONE),
    _null_init_done(false),
    _thr_or_curr(0.0f),
    _hil_mode(false)
{
    //AP_Param::setup_object_defaults(this, var_info);
    for (uint8_t i=0; i<COMPASS_MAX_BACKEND; i++) {
        _backends[i] = NULL;
        _state[i].last_update_usec = 0;
    }    

#if COMPASS_MAX_INSTANCES > 1
    // default device ids to zero.  init() method will overwrite with the actual device ids
    for (uint8_t i=0; i<COMPASS_MAX_INSTANCES; i++) {
        _state[i].dev_id = 0;
    }
#endif
}

// Default init method
//
bool
Compass::init()
{
    if (_compass_count == 0) {
        // detect available backends. Only called once
        _detect_backends();
    }
    if (_compass_count != 0) {
        // get initial health status
        hal.scheduler->delay(100);
        read();
    }
    return true;
}

//  Register a new compass instance
//
uint8_t Compass::register_compass(void)
{
    if (_compass_count == COMPASS_MAX_INSTANCES) {
        hal.scheduler->panic(PSTR("Too many compass instances"));
    }
    return _compass_count++;
}

/*
  try to load a backend
 */
void 
Compass::_add_backend(AP_Compass_Backend *(detect)(Compass &))
{
    if (_backend_count == COMPASS_MAX_BACKEND) {
        hal.scheduler->panic(PSTR("Too many compass backends"));
    }
    _backends[_backend_count] = detect(*this);
    if (_backends[_backend_count] != NULL) {
        _backend_count++;
    }
}

/*
  detect available backends for this board
 */
void 
Compass::_detect_backends(void)
{
    if (_hil_mode) {
        _add_backend(AP_Compass_HIL::detect);
        return;
    }

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX && CONFIG_HAL_BOARD_SUBTYPE != HAL_BOARD_SUBTYPE_LINUX_NONE && CONFIG_HAL_BOARD_SUBTYPE != HAL_BOARD_SUBTYPE_LINUX_BEBOP
    _add_backend(AP_Compass_HMC5843::detect);
    _add_backend(AP_Compass_AK8963::detect_mpu9250);
#elif HAL_COMPASS_DEFAULT == HAL_COMPASS_HIL
    _add_backend(AP_Compass_HIL::detect);
#elif HAL_COMPASS_DEFAULT == HAL_COMPASS_HMC5843
    _add_backend(AP_Compass_HMC5843::detect);
#elif  HAL_COMPASS_DEFAULT == HAL_COMPASS_AK8963_I2C && HAL_INS_AK8963_I2C_BUS == 1
    _add_backend(AP_Compass_AK8963::detect_i2c1);
#elif HAL_COMPASS_DEFAULT == HAL_COMPASS_PX4 || HAL_COMPASS_DEFAULT == HAL_COMPASS_VRBRAIN
    _add_backend(AP_Compass_PX4::detect);
#else
    #error Unrecognised HAL_COMPASS_TYPE setting
#endif

    if (_backend_count == 0 ||
        _compass_count == 0) {
        hal.console->println_P(PSTR("No Compass backends available"));
    }
}

void 
Compass::accumulate(void)
{    
    for (uint8_t i=0; i< _backend_count; i++) {
        // call accumulate on each of the backend
        _backends[i]->accumulate();
    }
}

bool 
Compass::read(void)
{
    for (uint8_t i=0; i< _backend_count; i++) {
        // call read on each of the backend. This call updates field[i]
        _backends[i]->read();
    }    
    for (uint8_t i=0; i < COMPASS_MAX_INSTANCES; i++) {
        _state[i].healthy = (hal.scheduler->millis() - _state[i].last_update_ms < 500);
    }
    return healthy();
}

void
Compass::set_offsets(uint8_t i, const Vector3f &offsets)
{
    // sanity check compass instance provided
    if (i < COMPASS_MAX_INSTANCES) {
        _state[i].offset.set(offsets);
    }
}

void
Compass::set_and_save_offsets(uint8_t i, const Vector3f &offsets)
{
    // sanity check compass instance provided
    if (i < COMPASS_MAX_INSTANCES) {
        _state[i].offset.set(offsets);
        save_offsets(i);
    }
}

void
Compass::save_offsets(uint8_t i)
{
    _state[i].offset.save();  // save offsets
#if COMPASS_MAX_INSTANCES > 1
    _state[i].dev_id.save();  // save device id corresponding to these offsets
#endif
}

void
Compass::save_offsets(void)
{
    for (uint8_t i=0; i<COMPASS_MAX_INSTANCES; i++) {
        save_offsets(i);
    }
}

void
Compass::set_motor_compensation(uint8_t i, const Vector3f &motor_comp_factor)
{
    _state[i].motor_compensation.set(motor_comp_factor);
}

void
Compass::save_motor_compensation()
{
    _motor_comp_type.save();
    for (uint8_t k=0; k<COMPASS_MAX_INSTANCES; k++) {
        _state[k].motor_compensation.save();
    }
}

void
Compass::set_initial_location(int32_t latitude, int32_t longitude)
{
    // if automatic declination is configured, then compute
    // the declination based on the initial GPS fix
    if (_auto_declination) {
        // Set the declination based on the lat/lng from GPS
        _declination.set(radians(
                AP_Declination::get_declination(
                    (float)latitude / 10000000,
                    (float)longitude / 10000000)));
    }
}

/// return true if the compass should be used for yaw calculations
bool
Compass::use_for_yaw(void) const
{
    uint8_t prim = get_primary();
    return healthy(prim) && use_for_yaw(prim);
}

/// return true if the specified compass can be used for yaw calculations
bool
Compass::use_for_yaw(uint8_t i) const
{
    return _state[i].use_for_yaw;
}

void
Compass::set_declination(float radians, bool save_to_eeprom)
{
    if (save_to_eeprom) {
        _declination.set_and_save(radians);
    }else{
        _declination.set(radians);
    }
}

float
Compass::get_declination() const
{
    return _declination.get();
}

/*
  calculate a compass heading given the attitude from DCM and the mag vector
 */
float
Compass::calculate_heading(const Matrix3f &dcm_matrix) const
{
    float cos_pitch_sq = 1.0f-(dcm_matrix.c.x*dcm_matrix.c.x);

    // Tilt compensated magnetic field Y component:
    const Vector3f &field = get_field();

    float headY = field.y * dcm_matrix.c.z - field.z * dcm_matrix.c.y;

    // Tilt compensated magnetic field X component:
    float headX = field.x * cos_pitch_sq - dcm_matrix.c.x * (field.y * dcm_matrix.c.y + field.z * dcm_matrix.c.z);

    // magnetic heading
    // 6/4/11 - added constrain to keep bad values from ruining DCM Yaw - Jason S.
    float heading = constrain_float(atan2f(-headY,headX), -3.15f, 3.15f);

    // Declination correction (if supplied)
    if( fabsf(_declination) > 0.0f )
    {
        heading = heading + _declination;
        if (heading > PI)    // Angle normalization (-180 deg, 180 deg)
            heading -= (2.0f * PI);
        else if (heading < -PI)
            heading += (2.0f * PI);
    }

    return heading;
}

/// Returns True if the compasses have been configured (i.e. offsets saved)
///
/// @returns                    True if compass has been configured
///
bool Compass::configured(uint8_t i)
{
    // exit immediately if instance is beyond the number of compasses we have available
    if (i > get_count()) {
        return false;
    }

    // exit immediately if all offsets are zero
    if (is_zero(get_offsets(i).length())) {
        return false;
    }

#if COMPASS_MAX_INSTANCES > 1
    // backup detected dev_id
    int32_t dev_id_orig = _state[i].dev_id;

    // load dev_id from eeprom
    _state[i].dev_id.load();

    // if different then the device has not been configured
    if (_state[i].dev_id != dev_id_orig) {
        // restore device id
        _state[i].dev_id = dev_id_orig;
        // return failure
        return false;
    }
#endif

    // if we got here then it must be configured
    return true;
}

bool Compass::configured(void)
{
    bool all_configured = true;
    for(uint8_t i=0; i<get_count(); i++) {
        all_configured = all_configured && configured(i);
    }
    return all_configured;
}

// Update raw magnetometer values from HIL data
//
void Compass::setHIL(uint8_t instance, float roll, float pitch, float yaw)
{
    Matrix3f R;

    // create a rotation matrix for the given attitude
    R.from_euler(roll, pitch, yaw);

    if (!is_equal(_hil.last_declination,get_declination())) {
        _setup_earth_field();
        _hil.last_declination = get_declination();
    }

    // convert the earth frame magnetic vector to body frame, and
    // apply the offsets
    _hil.field[instance] = R.mul_transpose(_hil.Bearth);

    // apply default board orientation for this compass type. This is
    // a noop on most boards
    _hil.field[instance].rotate(MAG_BOARD_ORIENTATION);

    // add user selectable orientation
    _hil.field[instance].rotate((enum Rotation)_state[0].orientation.get());

    if (!_state[0].external) {
        // and add in AHRS_ORIENTATION setting if not an external compass
        _hil.field[instance].rotate(_board_orientation);
    }
    _hil.healthy[instance] = true;
}

// Update raw magnetometer values from HIL mag vector
//
void Compass::setHIL(uint8_t instance, const Vector3f &mag)
{
    _hil.field[instance] = mag;
    _hil.healthy[instance] = true;
    _state[instance].last_update_usec = hal.scheduler->micros();
}

const Vector3f& Compass::getHIL(uint8_t instance) const 
{
    return _hil.field[instance];
}

// setup _Bearth
void Compass::_setup_earth_field(void)
{
    // assume a earth field strength of 400
    _hil.Bearth(400, 0, 0);
    
    // rotate _Bearth for inclination and declination. -66 degrees
    // is the inclination in Canberra, Australia
    Matrix3f R;
    R.from_euler(0, ToRad(66), get_declination());
    _hil.Bearth = R * _hil.Bearth;
}

/*
  set the type of motor compensation to use
 */
void Compass::motor_compensation_type(const uint8_t comp_type)
{
    if (_motor_comp_type <= AP_COMPASS_MOT_COMP_CURRENT && _motor_comp_type != (int8_t)comp_type) {
        _motor_comp_type = (int8_t)comp_type;
        _thr_or_curr = 0;                               // set current current or throttle to zero
        for (uint8_t i=0; i<COMPASS_MAX_INSTANCES; i++) {
            set_motor_compensation(i, Vector3f(0,0,0)); // clear out invalid compensation vectors
        }
    }
}
