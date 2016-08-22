#pragma once

#define APM_BUILD_DIRECTORY APM_BUILD_ArduCopter

#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
#include <AP_Scheduler.h>
#include <AP_Menu.h>

#include "Parameters.h"

#define OUTPUT 1
#define LED_PIN 9
#define DELAY_MAIN_LOOP 10

typedef struct __pins
{
    uint8_t pin = 0;
    bool sw_blink = false;
} pins_t;

class SchedTest {
public:
    SchedTest();
        
    void setup();
    void loop();

    int8_t menu_auto(uint8_t argc, const Menu::arg *argv);
    int8_t menu_test(uint8_t argc, const Menu::arg *argv);
    int8_t menu_set(uint8_t argc, const Menu::arg *argv);
    int8_t menu_show(uint8_t argc, const Menu::arg *argv);

private:

    Parameters g;
    AP_Param param_loader;
    static const AP_Param::Info var_info[];

    AP_Scheduler scheduler;
    pins_t pins[20];

    static const AP_Scheduler::Task scheduler_tasks[];
    AP_HAL::BetterStream* cliSerial;

    void ins_update(void);
    void one_hz_print(void);
    void five_second_call(void);
    void print_blanks(int16_t num);
    void ckeck_input_menu(void);
    void load_parameters(void);
};
