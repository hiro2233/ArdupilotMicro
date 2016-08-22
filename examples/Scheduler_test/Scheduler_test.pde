// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

//
// Simple test for the AP_Scheduler interface
//

#include "Scheduler_test.h"

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

SchedTest::SchedTest():
   param_loader(var_info)
{
   g.timeleds.load();
}

//pins_t SchedTest::pins[20];
static SchedTest schedtest;

#define MENU_FUNC(func) FUNCTOR_BIND(&schedtest, &SchedTest::func, int8_t, uint8_t, const Menu::arg *)
#define SCHED_TASK(func) FUNCTOR_BIND(&schedtest, &SchedTest::func, void)

/*
  scheduler table - all regular tasks are listed here, along with how
  often they should be called (in DELAY_MAIN_LOOP ms units) and the maximum time
  they are expected to take (in microseconds)
 */

const AP_Scheduler::Task SchedTest::scheduler_tasks[] PROGMEM = {
    { SCHED_TASK(ins_update),             1,   1000 },
    { SCHED_TASK(one_hz_print),         100,   1000 },
    { SCHED_TASK(five_second_call),     500,   1800 },
};

void SchedTest::print_blanks(int16_t num)
{
    while(num > 0) {
        num--;
        cliSerial->println("");
    }
}

int8_t SchedTest::menu_test(uint8_t argc, const Menu::arg *argv)
{
    uint8_t len = strnlen(argv[1].str, 16);
    uint8_t pin = (uint8_t)argv[1].i;
    uint8_t HIGH_LOW = (uint8_t)argv[2].i;

    hal.gpio->pinMode(pin, OUTPUT);
    hal.gpio->write(pin, HIGH_LOW);
    return 0;
}

int8_t SchedTest::menu_auto(uint8_t argc, const Menu::arg *argv)
{
  uint8_t len = strnlen(argv[2].str, 16);
  uint8_t led_pin = (uint8_t)argv[1].i;
  pins[led_pin].pin = led_pin;

  if (!strncmp("start", argv[2].str, len)) {
      hal.gpio->pinMode(pins[led_pin].pin, OUTPUT);
      pins[led_pin].sw_blink = true;
      hal.console->printf_P(PSTR("Blink started!\n"));
      return 0;
  }

  if (!strncmp("stop", argv[2].str, len)) {
      hal.gpio->pinMode(pins[led_pin].pin, OUTPUT);
      pins[led_pin].sw_blink = false;
      hal.console->printf_P(PSTR("Blink stoped!\n"));
      return 0;
  }

  return 0;
}

int8_t SchedTest::menu_set(uint8_t argc, const Menu::arg *argv)
{
    int8_t value_int8;
    int16_t value_int16;

    AP_Param *param;
    enum ap_var_type p_type;

    if(argc!=3)
    {
        cliSerial->printf_P(PSTR("Invalid command. Usage: set <name> <value>\n"));
        return 0;
    }

    param = AP_Param::find(argv[1].str, &p_type);
    if(!param)
    {
        cliSerial->printf_P(PSTR("Param not found: %s\n"), argv[1].str);
        return 0;
    }

    switch(p_type)
    {
        case AP_PARAM_INT8:
            value_int8 = (int8_t)(argv[2].i);
            if(argv[2].i!=value_int8)
            {
                cliSerial->printf_P(PSTR("Value out of range for type INT8\n"));
                return 0;
            }
            ((AP_Int8*)param)->set_and_save(value_int8);
            break;
        case AP_PARAM_INT16:
            value_int16 = (int16_t)(argv[2].i);
            if(argv[2].i!=value_int16)
            {
                cliSerial->printf_P(PSTR("Value out of range for type INT16\n"));
                return 0;
            }
            ((AP_Int16*)param)->set_and_save(value_int16);
            break;

        case AP_PARAM_INT32:
            ((AP_Int32*)param)->set_and_save(argv[2].i);
            break;
        case AP_PARAM_FLOAT:
            ((AP_Float*)param)->set_and_save(argv[2].f);
            break;
        default:
            cliSerial->printf_P(PSTR("Cannot set parameter of type %d.\n"), p_type);
            break;
    }

    return 0;
}

int8_t SchedTest::menu_show(uint8_t argc, const Menu::arg *argv)
{
    AP_Param *param;
    ap_var_type type;

    //If a parameter name is given as an argument to show, print only that parameter
    if(argc>=2)
    {

        param=AP_Param::find(argv[1].str, &type);

        if(!param)
        {
            cliSerial->printf_P(PSTR("Parameter not found: '%s'\n"), argv[1].str);
            return 0;
        }
        AP_Param::show(param, argv[1].str, type, hal.console);
        return 0;
    }

    print_blanks(2);

    AP_Param::show_all(cliSerial);

    return(0);
}

const struct Menu::command top_menu_commands[] PROGMEM = {
  {"blink",               MENU_FUNC(menu_auto)},
  {"pin",            MENU_FUNC(menu_test)},
  {"set",            MENU_FUNC(menu_set)},
  {"show",            MENU_FUNC(menu_show)},
};

MENU(top, "menu", top_menu_commands);


void SchedTest::ckeck_input_menu(void)
{
    if (top.check_input()){
       hal.console->printf_P(PSTR("Five seconds: %lu\n"),(unsigned long)hal.scheduler->millis());
    }
}

/*
  update inertial sensor, reading data 
 */
void SchedTest::ins_update(void)
{
    //ins.update();
}

/*
  print something once a second
 */
void SchedTest::one_hz_print(void)
{
    uint16_t timeleds = g.timeleds;
    static uint32_t lasttime = 0;
    uint32_t now = hal.scheduler->millis();

    if ((now - lasttime) >= timeleds) {
        lasttime = hal.scheduler->millis();
        for ( uint8_t i = 0; i < 20; i++) {
            if (pins[i].sw_blink) {
                hal.gpio->toggle(pins[i].pin);
            }
        }
    }
}

/*
  print something every 5 seconds
 */
void SchedTest::five_second_call(void)
{
    //hal.console->printf("five_seconds: t=%lu ins_counter=%u\n", (unsigned long)hal.scheduler->millis(), ins_counter);
}

void SchedTest::setup(void)
{
    cliSerial = hal.console;
    AP_Param::setup_sketch_defaults();
    load_parameters();
    //ins.init(AP_InertialSensor::COLD_START, AP_InertialSensor::RATE_50HZ);
    scheduler.init(&scheduler_tasks[0], ARRAY_SIZE(scheduler_tasks));
    hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&SchedTest::ckeck_input_menu, void));
    hal.console->printf_P(PSTR("System running!\nType help.\n"));

}

void SchedTest::loop(void)
{
    // wait for an INS sample
    //ins.wait_for_sample();
    hal.scheduler->delay(DELAY_MAIN_LOOP);
    // tell the scheduler one tick has passed
    scheduler.tick();
    // run all tasks that fit in 20ms
    scheduler.run(20000);
}

/*
  compatibility with old pde style build
 */
void setup(void);
void loop(void);

void setup(void)
{
    schedtest.setup();
}
void loop(void)
{
    schedtest.loop();
}
AP_HAL_MAIN();
