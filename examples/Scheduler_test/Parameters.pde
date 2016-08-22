/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Scheduler_test.h"

#define GSCALAR(v, name, def) { schedtest.g.v.vtype, name, Parameters::k_param_ ## v, &schedtest.g.v, {def_value : def} }
#define ASCALAR(v, name, def) { schedtest.aparm.v.vtype, name, Parameters::k_param_ ## v, (const void *)&schedtest.aparm.v, {def_value : def} }
#define GGROUP(v, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## v, &schedtest.g.v, {group_info : class::var_info} }
#define GOBJECT(v, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## v, (const void *)&schedtest.v, {group_info : class::var_info} }
#define GOBJECTN(v, pname, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## pname, (const void *)&schedtest.v, {group_info : class::var_info} }

const AP_Param::Info SchedTest::var_info[] PROGMEM = {
    // @Param: FORMAT_VERSION
    // @DisplayName: Eeprom format version number
    // @Description: This value is incremented when changes are made to the eeprom format
    // @User: Advanced
    GSCALAR(format_version,         "FORMAT_VERSION", 0),

    // @Param: SYSID_SW_TYPE
    // @DisplayName: Software Type
    // @Description: This is used by the ground station to recognise the software type (eg ArduPlane vs ArduCopter)
    // @Values: 0:ArduPlane,4:AntennaTracker,10:Copter,20:Rover
    // @User: Advanced
    // @ReadOnly: True
    GSCALAR(software_type,          "SYSID_SW_TYPE",  Parameters::k_software_type),

    GSCALAR(timeleds,          "TIMELEDS",      1000),

    AP_VAREND
};


void SchedTest::load_parameters(void)
{

    if (!AP_Param::check_var_info()) {
        cliSerial->printf_P(PSTR("Bad var table\n"));
        hal.scheduler->panic(PSTR("Bad var table"));
    }

    if (!g.format_version.load() ||
        g.format_version != Parameters::k_format_version) {

        // erase all parameters
        hal.console->printf_P(PSTR("Firmware change: erasing EEPROM...\n"));
        AP_Param::erase_all();

        // save the current format version
        g.format_version.set_and_save(Parameters::k_format_version);
        hal.console->println_P(PSTR("done."));
    }

    uint32_t before = hal.scheduler->micros();
    // Load all auto-loaded EEPROM variables
    AP_Param::load_all();
    hal.console->printf_P(PSTR("load_all took %luus\n"), (unsigned long)(hal.scheduler->micros() - before));

}
