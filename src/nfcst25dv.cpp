/*************************************************************************************************/
// File: nfcst25dv.cpp
// Description: NFC Dynamic Tags ST25DV ease library.
// Created on: 18 may. 2020
// Last modified date: 20 may. 2020
// Version: 1.0.0
/*************************************************************************************************/

/* Libraries */

#include "nfcst25dv.h"
//#include "hal_functions.h"

#if defined(ARDUINO) // Arduino Framework
    #include "Arduino.h"
#endif

/*************************************************************************************************/

/* HALs */

#if defined(ARDUINO) // Arduino Framework
    #define _DEFAULT_GPIO_SDA (SDA)
    #define _DEFAULT_GPIO_SCL (SCL)

    #define _LOW (LOW)
    #define _HIGH (HIGH)
    #define _OUTPUT (OUTPUT)
    #define _INPUT (INPUT)
    #define _INPUT_PULLUP (INPUT_PULLUP)
    #define _INPUT_PULLDOWN (INPUT_PULLDOWN)

    #define _print(x) do { Serial.print(x); } while(0)
    #define _println(x) do { Serial.println(x); } while(0)
    #define _printf(...) do { Serial.printf(__VA_ARGS__); } while(0)

    #define _pinMode(x, y) do { pinMode(x, y); } while(0)
    #define _digitalWrite(x, y) do { digitalWrite(x, y); } while(0)
    #define _digitalRead(x) (digitalRead(x))

    #define _millis() millis()
    #define _delay(x) delay(x)
    #define _yield() yield()

    #define _i2c_begin(x, y, z) do { Wire.begin(x, y, z); } while(0)
#elif defined(ESP_IDF) // ESP32 ESPIDF Framework
    #define _DEFAULT_GPIO_SDA (21)
    #define _DEFAULT_GPIO_SCL (22)

    #define _LOW (0)
    #define _HIGH (1)
    #define _OUTPUT (GPIO_MODE_OUTPUT)
    #define _INPUT (GPIO_MODE_INPUT)
    #define _INPUT_PULLUP (PULLUP)
    #define _INPUT_PULLDOWN (PULLDOWN)

    #define _print(x) do { printf("%s", x); } while(0)
    #define _println(x) do { printf("%s\n", x); } while(0)
    #define _printf(...) do { printf(__VA_ARGS__); } while(0)

    #define _pinMode(x, y) do \
    { \
        if(mode == _INPUT_PULLUP) \
            gpio_pad_pullup((gpio_num_t)x); \
        else if(mode == _INPUT_PULLDOWN) \
            gpio_pad_pulldown((gpio_num_t)x); \
        else \
            gpio_pad_select_gpio((gpio_num_t)x); \
        gpio_set_direction((gpio_num_t)x, y); \
    } while (0)
    #define _digitalWrite(x, y) do { gpio_set_level((gpio_num_t)x, y); } while(0)
    #define _digitalRead(x) (gpio_get_level((gpio_num_t)x))

    #define _millis() (esp_timer_get_time()/1000)
    #define _delay(x) do { vTaskDelay(x/portTICK_PERIOD_MS); } while(0)
    #define _yield() do { taskYIELD(); } while(0)

    #define _i2c_begin(x, y, z) do { Wire.begin(x, y, z); } while(0)
#elif defined(SAMD20) // SAMD20 ASF Framework
    #define _DEFAULT_GPIO_SDA (PINMUX_PA08C_SERCOM0_PAD0)
    #define _DEFAULT_GPIO_SCL (PINMUX_PA09C_SERCOM0_PAD1)

    #define _LOW false
    #define _HIGH true
    #define _OUTPUT PORT_PIN_DIR_OUTPUT
    #define _INPUT PORT_PIN_DIR_INPUT
    #define _INPUT_PULLUP PORT_PIN_PULL_UP
    #define _INPUT_PULLDOWN PORT_PIN_PULL_DOWN

    #define _print(x)
    #define _println(x)
    #define _printf(...)

    #define _pinMode(x, y) do \
    { \
        struct port_config config_port_pin; \
        port_get_config_defaults(&config_port_pin); \
        if(y == _OUTPUT) \
            config_port_pin.direction = PORT_PIN_DIR_OUTPUT; \
        else if(y == _INPUT) \
            config_port_pin.direction  = PORT_PIN_DIR_INPUT; \
        else if(y == _INPUT_PULLUP) \
        { \
            config_port_pin.direction  = PORT_PIN_DIR_INPUT; \
            config_port_pin.input_pull = PORT_PIN_PULL_UP; \
        } \
        else if(y == _INPUT_PULLDOWN) \
        { \
            config_port_pin.direction  = PORT_PIN_DIR_INPUT; \
            config_port_pin.input_pull = PORT_PIN_PULL_DOWN; \
        } \
        port_pin_set_config(x, &config_port_pin); \
    } while (0)
    #define _digitalWrite(x, y) do { port_pin_set_output_level(x, y); } while(0)
    #define _digitalRead(x) (port_pin_get_output_level(x))

    #define _millis() (systickCount) // External global systickCount variable must exists 
    #define _delay(x) do \
    { \
        uint32_t t0 = systickCount; \
        while( (systickCount - t0)  <= ms) \
            wdt_reset_count();
        \
    } while(0)
    #define _yield() wdt_reset_count();

    #define _i2c_begin(a, x, y, z) do \ // a == global i2c_master_module element
    { \
        struct i2c_master_config config_i2c_master; \
        i2c_master_get_config_defaults(&config_i2c_master); \
        config_i2c_master.buffer_timeout = 10000; \
        i2c_master_init(&a, SERCOM2, &config_i2c_master); \
        i2c_master_enable(&a); \
    } while (0)
#else
    #define _DEFAULT_GPIO_SDA (-1)
    #define _DEFAULT_GPIO_SCL (-1)

    #define _LOW
    #define _HIGH
    #define _OUTPUT
    #define _INPUT
    #define _INPUT_PULLUP
    #define _INPUT_PULLDOWN

    #define _print(x)
    #define _println(x)
    #define _printf(...)

    #define _pinMode(x, y)
    #define _digitalWrite(x, y)
    #define _digitalRead(x) (0)

    #define _millis() (0)
    #define _delay(x)
    #define _yield()

    #define _i2c_begin(x, y, z)
#endif

/*************************************************************************************************/

/* External Functions Prototypes */

extern uint32_t hal_millis(void);
extern int32_t i2c_init(void);
extern int32_t i2c_deinit(void);
extern int32_t i2c_isready(uint16_t device_addr, const uint32_t retries);
extern int32_t i2c_writereg(uint16_t i2c_device_addr, uint16_t reg_addr, uint8_t* data,
        uint16_t length);
extern int32_t i2c_readreg(uint16_t i2c_device_addr, uint16_t reg_addr, uint8_t* data,
        uint16_t length);

/*************************************************************************************************/

/* Constructor */

// NFCST25DV Constructor
NFCST25DV::NFCST25DV(const int8_t gpio_eh, const int8_t gpio_gpo, const int8_t gpio_lpd)
{
    // Assign GPIO variables
    _gpio_eh  = gpio_eh;
    _gpio_gpo = gpio_gpo;
    _gpio_lpd = gpio_lpd;

    _nfc_tag_drv = NULL;

    _lpd_signal = _HIGH;
}

/*************************************************************************************************/

/* Public Methods */

// Initialize GPIOs and I2C
int32_t NFCST25DV::init(void)
{
    int32_t rc = 0;

    // Setup EH pin
    //if(_gpio_eh != UNINITIALIZE)
    //    _pinMode(_gpio_eh, INPUT);

    // Setup GPO (General Purpouse Output) pin
    if(_gpio_gpo != UNINITIALIZE)
        _pinMode(_gpio_gpo, _INPUT);
    
    // Setup LPD (Low Power Down) mode Pin to wake-up
    if(_gpio_lpd != UNINITIALIZE)
    {
        _lpd_signal = _LOW;
        _digitalWrite(_gpio_lpd, _lpd_signal);
        _pinMode(_gpio_lpd, _OUTPUT);
    }

    // Wait 5ms to ensure NFC Tag voltage is stable and module is ready
    _delay(5);

    // Assign HAL I2C Functions
    ST25DV_IO_t i2c_interface;
    i2c_interface.Init    = (ST25DV_Init_Func)i2c_init;
    i2c_interface.DeInit  = (ST25DV_DeInit_Func)i2c_deinit;
    i2c_interface.IsReady = (ST25DV_IsReady_Func)i2c_isready;
    i2c_interface.Read    = (ST25DV_Read_Func)i2c_readreg;
    i2c_interface.Write   = (ST25DV_Write_Func)i2c_writereg;
    i2c_interface.GetTick = (ST25DV_GetTick_Func)hal_millis;

    // Asociate I2C interface into NFC element
    rc = ST25DV_RegisterBusIO(&_std25dv_element, &i2c_interface);
    if(rc != NFCTAG_OK)
    {
        _println("[NFCST25DV] Error: Can't Register I2C interface.");
        _print("Code: "); _println(rc); _println("");
        return rc;
    }

    // Initialize NFC Driver
    _nfc_tag_drv = (NFCTAG_DrvTypeDef *)(void *)&St25Dv_Drv;
    if(_nfc_tag_drv->Init != NULL)
    {
        rc = _nfc_tag_drv->Init(&_std25dv_element);
        if(rc != NFCTAG_OK)
        {
            _nfc_tag_drv = NULL;
            _println("[NFCST25DV] Error: Can't initialize DRV.");
            _print("Code: "); _println(rc); _println("");
            return rc;
        }
    }
    else
    {
        _nfc_tag_drv = NULL;
        _println("[NFCST25DV] Error: DRV init fail.");
        return NFCTAG_ERROR;
    }

    return NFCTAG_OK;
}

// Read ST25DV GPO signal value
uint8_t NFCST25DV::get_gpo_value(void)
{
    return _digitalRead(_gpio_gpo);
}

// Check if ST25DV is in Low Power Mode (depends on LPD signal status)
bool NFCST25DV::is_in_low_power_mode(void)
{
    return (_lpd_signal == HIGH);
}

// Set ST25DV to enter/exit Low Power Mode
void NFCST25DV::low_power_mode(const bool enable)
{
    if(enable)
        _lpd_signal = HIGH;
    else
        _lpd_signal = _LOW;

    _digitalWrite(_gpio_lpd, _lpd_signal);
}

// Read IC Reference of the ST25DV
uint8_t NFCST25DV::get_tag_ic_ref(void)
{
    uint8_t nfctagid = 0;

    if(_nfc_tag_drv->ReadID == NULL)
        return NFCTAG_ERROR;

    _nfc_tag_drv->ReadID(&_std25dv_element, &nfctagid);

    return nfctagid;
}

// Reads the ST25DV IC Revision
int32_t NFCST25DV::get_ic_rev(uint8_t* const ic_revision)
{
    return ST25DV_ReadICRev(&_std25dv_element, ic_revision);
}

// Reads the ST25DV Tag UID
int32_t NFCST25DV::read_tag_uid(ST25DV_UID* const uid)
{
    return ST25DV_ReadUID(&_std25dv_element, uid);
}

// Get NFC Tag memory size
uint32_t NFCST25DV::read_tag_size(void)
{
    ST25DV_MEM_SIZE mem_size;

    ST25DV_ReadMemSize(&_std25dv_element, &mem_size);

    return ((mem_size.BlockSize+1) * (mem_size.Mem_Size+1));
}

// Read data from ST25DV register
int32_t NFCST25DV::read_data(const uint16_t addr, uint8_t* data, const uint16_t length)
{
    if(_nfc_tag_drv->ReadData == NULL)
        return NFCTAG_ERROR;

    return _nfc_tag_drv->ReadData(&_std25dv_element, data, addr, length);
}

// Write data to ST25DV register
int32_t NFCST25DV::write_data(const uint16_t addr, uint8_t* data, const uint16_t length)
{
    if(_nfc_tag_drv->WriteData == NULL )
        return NFCTAG_ERROR;

    return _nfc_tag_drv->WriteData(&_std25dv_element, data, addr, length);
}

// Configure NFC Tag interrupt
int32_t NFCST25DV::set_interrupt_status(const uint16_t int_config)
{
    if ( _nfc_tag_drv->ConfigIT == NULL )
        return NFCTAG_ERROR;

    return _nfc_tag_drv->ConfigIT(&_std25dv_element, int_config);
}

// Get NFC Tag interrupt configuration
int32_t NFCST25DV::get_interrupt_status(uint16_t* const int_config)
{
    if (_nfc_tag_drv->GetITStatus == NULL )
        return NFCTAG_ERROR;

    return _nfc_tag_drv->GetITStatus(&_std25dv_element, int_config);
}

/**
  * @brief  Reads the ST25DV Interrupt Time duration for the GPO pulses.
  * @param  int_time Pointer used to return the coefficient for the GPO Pulse duration (Pulse duration = 302,06 us - ITtime * 512 / fc).
  * @return int32_t enum status.
  */
int32_t NFCST25DV::read_interrupt_pulse(ST25DV_PULSE_DURATION* const int_time)
{
    return ST25DV_ReadITPulse(&_std25dv_element, int_time);
}

/**
  * @brief    Configures the ST25DV Interrupt Time duration for the GPO pulse.
  * @details  Needs the I2C Password presentation to be effective.
  * @param    int_time Coefficient for the Pulse duration to be written (Pulse duration = 302,06 us - ITtime * 512 / fc)
  * @retval   int32_t enum status.
  */
int32_t NFC04A1_NFCTAG_WriteITPulse(const ST25DV_PULSE_DURATION int_time)
{
    return ST25DV_WriteITPulse(&_std25dv_element, int_time);
}

/*************************************************************************************************/

/* Private Methods */

