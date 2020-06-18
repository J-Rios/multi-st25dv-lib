/*************************************************************************************************/
// File: nfcst25dv.cpp
// Description: NFC Dynamic Tags ST25DV ease library.
// Created on: 18 may. 2020
// Last modified date: 24 may. 2020
// Version: 1.0.0
/*************************************************************************************************/

/* Libraries */

#include "nfcst25dv.h"

/*************************************************************************************************/

/* HALs */

#if defined(ARDUINO) // Arduino Framework
    #include "Arduino.h"

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

    #define _delay(x) delay(x)
#elif defined(ESP_IDF) // ESP32 ESPIDF Framework
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

    #define _delay(x) do { vTaskDelay(x/portTICK_PERIOD_MS); } while(0)

#elif defined(SAMD20_ASF) || defined(SAML21_ASF) // SAM0 ASF Framework
    #include "asf.h"

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
        else if(y == (port_pin_dir)(_INPUT_PULLUP)) \
        { \
            config_port_pin.direction  = PORT_PIN_DIR_INPUT; \
            config_port_pin.input_pull = PORT_PIN_PULL_UP; \
        } \
        else if(y == (port_pin_dir)(_INPUT_PULLDOWN)) \
        { \
            config_port_pin.direction  = PORT_PIN_DIR_INPUT; \
            config_port_pin.input_pull = PORT_PIN_PULL_DOWN; \
        } \
        port_pin_set_config(x, &config_port_pin); \
    } while (0)
    #define _digitalWrite(x, y) do { port_pin_set_output_level(x, y); } while(0)
    #define _digitalRead(x) (port_pin_get_output_level(x))

    extern volatile uint32_t systickCount;
    #define _delay(x) do \
    { \
        uint32_t t0 = systickCount; \
        while((systickCount - t0)  <= x) \
            wdt_reset_count(); \
        \
    } while(0)
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

/**
  * @brief  NFCST25DV Constructor.
  */
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

/**
  * @brief  Initialize GPIOs and I2C.
  * @retval NFCTAG enum status.
  */
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

/**
  * @brief  Read ST25DV GPO signal value.
  * @retval Actual GPO signal value.
  */
uint8_t NFCST25DV::get_gpo_value(void)
{
    return _digitalRead(_gpio_gpo);
}

/**
  * @brief  Check if ST25DV is in Low Power Mode (depends on LPD signal status).
  * @retval Tag in Low Power Mode.
  */
bool NFCST25DV::is_in_low_power_mode(void)
{
    return (_lpd_signal == HIGH);
}

/**
  * @brief  Set ST25DV to enter/exit Low Power Mode.
  * @param  enable enable or disable low power mode.
  */
void NFCST25DV::low_power_mode(const bool enable)
{
    if(enable)
        _lpd_signal = HIGH;
    else
        _lpd_signal = _LOW;

    _digitalWrite(_gpio_lpd, _lpd_signal);
}

/**
  * @brief  Check if the nfctag is available.
  * @param  retries Number of trials.
  * @retval NFCTAG enum status.
  */
int32_t NFCST25DV::is_tag_ready(const uint32_t retries)
{
    if(_nfc_tag_drv->IsReady == NULL)
        return NFCTAG_ERROR;

    return _nfc_tag_drv->IsReady(&_std25dv_element, retries);
}

/**
  * @brief  Read IC Reference of the ST25DV.
  * @retval Tag IC Reference or -1 if error.
  */
uint8_t NFCST25DV::get_tag_ic_ref(void)
{
    uint8_t nfctagid = 0;

    if(_nfc_tag_drv->ReadID == NULL)
        return NFCTAG_ERROR;

    _nfc_tag_drv->ReadID(&_std25dv_element, &nfctagid);

    return nfctagid;
}

/**
  * @brief  Reads the ST25DV IC Revision.
  * @param  ic_revision Pointer to get Tag IC revision.
  * @retval NFCTAG enum status.
  */
int32_t NFCST25DV::get_tag_ic_rev(uint8_t* const ic_revision)
{
    return ST25DV_ReadICRev(&_std25dv_element, ic_revision);
}

/**
  * @brief  Reads the ST25DV Tag UID.
  * @param  uid Pointer to get Tag UID value.
  * @retval NFCTAG enum status.
  */
int32_t NFCST25DV::read_tag_uid(ST25DV_UID* const uid)
{
    return ST25DV_ReadUID(&_std25dv_element, uid);
}

/**
  * @brief  Get NFC Tag memory size.
  * @retval NFCTAG enum status.
  */
uint32_t NFCST25DV::read_tag_size(void)
{
    ST25DV_MEM_SIZE mem_size;
    if(ST25DV_ReadMemSize(&_std25dv_element, &mem_size) != NFCTAG_OK)
        return 0;
    return ((mem_size.BlockSize+1) * (mem_size.Mem_Size+1));
}

/**
  * @brief  Read data from ST25DV register.
  * @param  addr Tag data address.
  * @param  data Pointer to get read data.
  * @param  length Length of data to read.
  * @retval NFCTAG enum status.
  */
int32_t NFCST25DV::read_data(const uint16_t addr, uint8_t* data, const uint16_t length)
{
    if(_nfc_tag_drv->ReadData == NULL)
        return NFCTAG_ERROR;

    return _nfc_tag_drv->ReadData(&_std25dv_element, data, addr, length);
}

/**
  * @brief  Write data to ST25DV register.
  * @param  addr Tag data address.
  * @param  data Data to write.
  * @param  length Length of data to write.
  * @retval NFCTAG enum status.
  */
int32_t NFCST25DV::write_data(const uint16_t addr, uint8_t* data, const uint16_t length)
{
    if(_nfc_tag_drv->WriteData == NULL)
        return NFCTAG_ERROR;

    return _nfc_tag_drv->WriteData(&_std25dv_element, data, addr, length);
}

/**
  * @brief  Configure NFC Tag interrupt.
  * @param  int_config Interrupt configuration to set (0x01 => RF BUSY; 0x02 => WIP).
  * @retval NFCTAG enum status.
  */
int32_t NFCST25DV::set_interrupt_status(const uint16_t int_config)
{
    if(_nfc_tag_drv->ConfigIT == NULL)
        return NFCTAG_ERROR;

    return _nfc_tag_drv->ConfigIT(&_std25dv_element, int_config);
}

/**
  * @brief  Get NFC Tag interrupt configuration.
  * @param  int_config Pointer to store current interrupt configuration (0x01 => RF BUSY;
  * 0x02 => WIP).
  * @retval NFCTAG enum status.
  */
int32_t NFCST25DV::get_interrupt_status(uint16_t* const int_config)
{
    if(_nfc_tag_drv->GetITStatus == NULL)
        return NFCTAG_ERROR;

    return _nfc_tag_drv->GetITStatus(&_std25dv_element, int_config);
}

/**
  * @brief  Reads ST25DV Interrupt Time duration for the GPO pulses.
  * @param  int_time Pointer used to return the coefficient for the GPO Pulse duration
  * (Pulse duration = 302,06 us - ITtime * 512 / fc).
  * @retval NFCTAG enum status.
  */
int32_t NFCST25DV::read_interrupt_pulse(ST25DV_PULSE_DURATION* const int_time)
{
    return ST25DV_ReadITPulse(&_std25dv_element, int_time);
}

/**
  * @brief   Configures the ST25DV Interrupt Time duration for the GPO pulse.
  * @details Needs the I2C Password presentation to be effective.
  * @param   int_time Coefficient for the Pulse duration to be written 
  * (Pulse duration = 302,06 us - ITtime * 512 / fc)
  * @retval  NFCTAG enum status.
  */
int32_t NFCST25DV::set_interrupt_pulse(const ST25DV_PULSE_DURATION int_time)
{
    return ST25DV_WriteITPulse(&_std25dv_element, int_time);
}

/**
  * @brief  Reads the ST25DV DSFID.
  * @param  ptr_dsfid Pointer used to return the ST25DV DSFID value.
  * @return int32_t enum status.
  */
int32_t NFCST25DV::read_dsfid(uint8_t* const ptr_dsfid)
{
    return ST25DV_ReadDSFID(&_std25dv_element, ptr_dsfid);
}

/**
  * @brief  Reads the ST25DV DSFID RF Lock state.
  * @param  ptr_lock_dsfid Pointer on a ST25DV_LOCK_STATUS used to return the DSFID lock state.
  * @return int32_t enum status.
  */
int32_t NFCST25DV::read_dsfid_rf_protection(ST25DV_LOCK_STATUS* const ptr_lock_dsfid)
{
    return ST25DV_ReadDsfidRFProtection(&_std25dv_element, ptr_lock_dsfid);
}

/**
  * @brief  Reads the ST25DV AFI.
  * @param  ptr_afi Pointer used to return the ST25DV AFI value.
  * @return int32_t enum status.
  */
int32_t NFCST25DV::read_afi(uint8_t* const ptr_afi)
{
    return ST25DV_ReadAFI(&_std25dv_element, ptr_afi);
}

/**
  * @brief  Reads the AFI RF Lock state.
  * @param  ptr_lock_afi Pointer on a ST25DV_LOCK_STATUS used to return the ASFID lock state.
  * @return int32_t enum status.
  */
int32_t NFCST25DV::read_afi_rf_protection(ST25DV_LOCK_STATUS* const ptr_lock_afi)
{
    return ST25DV_ReadAfiRFProtection(&_std25dv_element, ptr_lock_afi);
}

/**
  * @brief  Reads the I2C Protected Area state.
  * @param  ptr_prot_zone Pointer on a ST25DV_I2C_PROT_ZONE structure used to return the
  * Protected Area state.
  * @return int32_t enum status.
  */
int32_t NFCST25DV::read_i2c_protect_zone(ST25DV_I2C_PROT_ZONE* const ptr_prot_zone)
{
    return ST25DV_ReadI2CProtectZone(&_std25dv_element, ptr_prot_zone);
}

/**
  * @brief    Sets the I2C write-protected state to an EEPROM Area.
  * @details  Needs the I2C Password presentation to be effective.
  * @param    zone                ST25DV_PROTECTION_ZONE value coresponding to the area to protect.
  * @param    rw_protection ST25DV_PROTECTION_CONF value corresponding to the protection to be set.
  * @return   int32_t enum status.
  */
int32_t NFCST25DV::write_i2c_protect_zone(const ST25DV_PROTECTION_ZONE zone, 
        const ST25DV_PROTECTION_CONF rw_protection)
{
    return ST25DV_WriteI2CProtectZonex(&_std25dv_element, zone, rw_protection);
}

/**
  * @brief  Reads the CCFile protection state.
  * @param  ptr_lock_ccfile Pointer on a ST25DV_LOCK_CCFILE value corresponding to the lock
  * state of the CCFile.
  * @return int32_t enum status.
  */
int32_t NFCST25DV::read_lock_ccfile(ST25DV_LOCK_CCFILE* const ptr_lock_ccfile)
{
    return ST25DV_ReadLockCCFile(&_std25dv_element, ptr_lock_ccfile);
}

/**
  * @brief  Locks the CCFile to prevent any RF write access.
  * @details  Needs the I2C Password presentation to be effective.
  * @param  num_block_ccfile ST25DV_CCFILE_BLOCK value corresponding to the number of blocks
  * to be locked.
  * @param  lock_ccfile ST25DV_LOCK_CCFILE value corresponding to the lock state to apply on
  * the CCFile.
  * @return int32_t enum status.
  */
int32_t NFCST25DV::write_lock_ccfile(const ST25DV_CCFILE_BLOCK num_block_ccfile, 
        const ST25DV_LOCK_STATUS lock_ccfile)
{
    return ST25DV_WriteLockCCFile(&_std25dv_element, num_block_ccfile, lock_ccfile);
}

/**
  * @brief  Reads the Cfg registers protection.
  * @param  ptr_lock_cfg Pointer on a ST25DV_LOCK_STATUS value corresponding to the Cfg
  * registers lock state.
  * @return int32_t enum status.
  */
int32_t NFCST25DV::read_lock_cfg(ST25DV_LOCK_STATUS* const ptr_lock_cfg)
{
    return ST25DV_ReadLockCFG(&_std25dv_element, ptr_lock_cfg);
}

/**
  * @brief  Lock/Unlock the Cfg registers, to prevent any RF write access.
  * @details  Needs the I2C Password presentation to be effective.
  * @param  lock_cfg ST25DV_LOCK_STATUS value corresponding to the lock state to be written.
  * @return int32_t enum status.
  */
int32_t NFCST25DV::write_lock_cfg(const ST25DV_LOCK_STATUS lock_cfg)
{
    return ST25DV_WriteLockCFG(&_std25dv_element, lock_cfg);
}

/**
  * @brief  Presents I2C password, to authorize the I2C writes to protected areas.
  * @param  password Password value on 32bits
  * @return int32_t enum status.
  */
int32_t NFCST25DV::present_i2c_password(const ST25DV_PASSWD password)
{
    return ST25DV_PresentI2CPassword(&_std25dv_element, password);
}

/**
  * @brief  Writes a new I2C password.
  * @details  Needs the I2C Password presentation to be effective.
  * @param  password New I2C PassWord value on 32bits.
  * @return int32_t enum status.
  */
int32_t NFCST25DV::write_i2c_password(const ST25DV_PASSWD password)
{
    return ST25DV_WriteI2CPassword(&_std25dv_element, password);
}

/**
  * @brief  Reads the RF Zone Security Status (defining the allowed RF accesses).
  * @param  zone        ST25DV_PROTECTION_ZONE value coresponding to the protected area.
  * @param  ptr_rf_prot_zone Pointer on a ST25DV_RF_PROT_ZONE value corresponding to the
  * area protection state.
  * @return int32_t enum status.
  */
int32_t NFCST25DV::read_rf_zone_security_status(const ST25DV_PROTECTION_ZONE zone,
        ST25DV_RF_PROT_ZONE* const ptr_rf_prot_zone)
{
    return ST25DV_ReadRFZxSS(&_std25dv_element, zone, ptr_rf_prot_zone);
}

/**
  * @brief  Writes the RF Zone Security Status (defining the allowed RF accesses)
  * @details  Needs the I2C Password presentation to be effective.
  * @param  zone ST25DV_PROTECTION_ZONE value corresponding to the area on which to set 
  * the RF protection.
  * @param  rf_prot_zone  Pointer on a ST25DV_RF_PROT_ZONE value defininf the protection
  * to be set on the area.
  * @return int32_t enum status.
  */
int32_t NFCST25DV::write_rf_zone_security_status(const ST25DV_PROTECTION_ZONE zone, 
        const ST25DV_RF_PROT_ZONE rf_prot_zone)
{
    return ST25DV_WriteRFZxSS(&_std25dv_element, zone, rf_prot_zone);
}

/**
  * @brief  Reads the value of the an area end address.
  * @param  end_zone ST25DV_END_ZONE value corresponding to an area end address.
  * @param  ptr_end_zone_value Pointer used to return the end address of the area.
  * @return int32_t enum status.
  */
int32_t NFCST25DV::read_end_zonex(const ST25DV_END_ZONE end_zone,
        uint8_t* const ptr_end_zone_value)
{
    return ST25DV_ReadEndZonex(&_std25dv_element, end_zone, ptr_end_zone_value);
}

/**
  * @brief    Sets the end address of an area.
  * @details  Needs the I2C Password presentation to be effective.
  * @note     The ST25DV answers a NACK when setting the EndZone2 & EndZone3 to same value than
  *           repectively EndZone1 & EndZone2.\n
  *           These NACKs are ok.
  * @param  end_zone ST25DV_END_ZONE value corresponding to an area.
  * @param  end_zone_value   End zone value to be written.
  * @return int32_t enum status.
  */
int32_t NFCST25DV::write_end_zonex(const ST25DV_END_ZONE end_zone, const uint8_t end_zone_value)
{
    return ST25DV_WriteEndZonex(&_std25dv_element, end_zone, end_zone_value);
}

/**
  * @brief  Initializes the end address of the ST25DV areas with their default values 
  * (end of memory).
  * @details  Needs the I2C Password presentation to be effective..
  *           The ST25DV answers a NACK when setting the EndZone2 & EndZone3 to same value than 
  *           repectively EndZone1 & EndZone2.
  *           These NACKs are ok.
  * @return int32_t enum status.
  */
int32_t NFCST25DV::init_end_zone(void)
{
    return ST25DV_InitEndZone(&_std25dv_element);
}

/**
  * @brief  Creates user areas with defined lengths.
  * @details  Needs the I2C Password presentation to be effective.
  * @param  zone_1_length Length of area1 in bytes (32 to 8192, 0x20 to 0x2000)
  * @param  zone_2_length Length of area2 in bytes (0 to 8128, 0x00 to 0x1FC0)
  * @param  zone_3_length Length of area3 in bytes (0 to 8064, 0x00 to 0x1F80)
  * @param  zone_4_length Length of area4 in bytes (0 to 8000, 0x00 to 0x1F40)
  * @return int32_t enum status.
  */
int32_t NFCST25DV::create_user_zone(uint16_t zone_1_length, uint16_t zone_2_length,
        uint16_t zone_3_length, uint16_t zone_4_length)
{
    return ST25DV_CreateUserZone(&_std25dv_element, zone_1_length, zone_2_length, zone_3_length,
            zone_4_length);
}

/**
  * @brief  Reads the Energy harvesting mode.
  * @param  ptr_eh_mode Pointer on a ST25DV_EH_MODE_STATUS value corresponding to the Energy 
  * Harvesting state.
  * @return int32_t enum status.
  */
int32_t NFCST25DV::read_eh_mode(ST25DV_EH_MODE_STATUS* const ptr_eh_mode)
{
    return ST25DV_ReadEHMode(&_std25dv_element, ptr_eh_mode);
}

/**
  * @brief  Sets the Energy harvesting mode.
  * @details  Needs the I2C Password presentation to be effective.
  * @param  eh_mode ST25DV_EH_MODE_STATUS value for the Energy harvesting mode to be set.
  * @return int32_t enum status.
  */
int32_t NFCST25DV::write_eh_mode(const ST25DV_EH_MODE_STATUS eh_mode)
{
    return ST25DV_WriteEHMode(&_std25dv_element, eh_mode);
}

/**
  * @brief  Reads the RF Management configuration.
  * @param  ptr_rf_config Pointer on a ST25DV_RF_MNGT structure used to return the RF Management 
  * configuration.
  * @return int32_t enum status.
  */
int32_t NFCST25DV::read_rf_config(ST25DV_RF_MNGT* const ptr_rf_config)
{
    return ST25DV_ReadRFMngt(&_std25dv_element, ptr_rf_config);
}

/**
  * @brief  Sets the RF Management configuration.
  * @details  Needs the I2C Password presentation to be effective.
  * @param  rf_config Value of the RF Management configuration to be written.
  * @return int32_t enum status.
  */
int32_t NFCST25DV::write_rf_config(const uint8_t rf_config)
{
    return ST25DV_WriteRFMngt(&_std25dv_element, rf_config);
}

/**
  * @brief  Reads the RFDisable register information.
  * @param  ptr_rf_disable Pointer on a ST25DV_EN_STATUS value corresponding to the 
  * RF Disable status.
  * @return int32_t enum status.
  */
int32_t NFCST25DV::read_rf_disable(ST25DV_EN_STATUS* const ptr_rf_disable)
{
    return ST25DV_GetRFDisable(&_std25dv_element, ptr_rf_disable);
}

/**
  * @brief  Sets the RF Disable configuration.
  * @details  Needs the I2C Password presentation to be effective.
  * @return int32_t enum status.
  */
int32_t NFCST25DV::write_rf_disable(void)
{
    return ST25DV_SetRFDisable(&_std25dv_element);
}

/**
  * @brief  Resets the RF Disable configuration
  * @details  Needs the I2C Password presentation to be effective.
  * @return int32_t enum status.
  */
int32_t NFCST25DV::reset_rf_disable(void)
{
    return ST25DV_ResetRFDisable(&_std25dv_element);
}

/**
  * @brief  Reads the RFSleep register information.
  * @param  ptr_rf_sleep Pointer on a ST25DV_EN_STATUS value corresponding to the RF Sleep status.
  * @return int32_t enum status.
  */
int32_t NFCST25DV::read_rf_sleep(ST25DV_EN_STATUS* const ptr_rf_sleep)
{
    return ST25DV_GetRFSleep(&_std25dv_element, ptr_rf_sleep);
}

/**
  * @brief  Sets the RF Sleep configuration.
  * @details  Needs the I2C Password presentation to be effective.
  * @return int32_t enum status.
  */
int32_t NFCST25DV::write_rf_sleep(void)
{
    return ST25DV_SetRFSleep(&_std25dv_element);
}

/**
  * @brief  Resets the RF Sleep configuration.
  * @details  Needs the I2C Password presentation to be effective.
  * @return int32_t enum status.
  */
int32_t NFCST25DV::reset_rf_sleep(void)
{
    return ST25DV_ResetRFSleep(&_std25dv_element);
}

/**
  * @brief  Reads the Mailbox mode.
  * @param  ptr_mb_mode Pointer on a ST25DV_EH_MODE_STATUS value used to return the Mailbox mode.
  * @return int32_t enum status.
  */
int32_t NFCST25DV::read_mailbox_mode(ST25DV_EN_STATUS* const ptr_mb_mode)
{
    return ST25DV_ReadMBMode(&_std25dv_element, ptr_mb_mode);
}

/**
  * @brief  Sets the Mailbox mode.
  * @details  Needs the I2C Password presentation to be effective.
  * @param  mb_mode ST25DV_EN_STATUS value corresponding to the Mailbox mode to be set.
  * @return int32_t enum status.
  */
int32_t NFCST25DV::write_mailbox_mode(const ST25DV_EN_STATUS mb_mode)
{
    return ST25DV_WriteMBMode(&_std25dv_element, mb_mode);
}

/**
  * @brief  Reads the Mailbox watchdog duration coefficient.
  * @param  ptr_wdg_delay Pointer on a uint8_t used to return the watchdog duration coefficient.
  * @return int32_t enum status.
  */
int32_t NFCST25DV::read_mailbox_wd_duration_coef(uint8_t* const ptr_wdg_delay)
{
    return ST25DV_ReadMBWDG(&_std25dv_element, ptr_wdg_delay);
}

/**
  * @brief  Writes the Mailbox watchdog coefficient delay
  * @details  Needs the I2C Password presentation to be effective.
  * @param  wdg_delay Watchdog duration coefficient to be written
  * (Watch dog duration = MB_WDG*30 ms +/- 6%).
  * @return int32_t enum status.
  */
int32_t NFCST25DV::write_mailbox_wd_duration_coef(const uint8_t wdg_delay)
{
    return ST25DV_WriteMBWDG(&_std25dv_element, wdg_delay);
}

/**
  * @brief  Reads N bytes of data from the Mailbox, starting at the specified byte offset.
  * @param  ptr_data   Pointer on the buffer used to return the read data.
  * @param  offset  Offset in the Mailbox memory, byte number to start the read.
  * @param  num_byte  Number of bytes to be read.
  * @return int32_t enum status.
  */
int32_t NFCST25DV::read_mailbox_data(uint8_t* const ptr_data, const uint16_t offset,
        const uint16_t num_byte)
{
    return ST25DV_ReadMailboxData(&_std25dv_element, ptr_data, offset, num_byte);
}

/**
  * @brief  Writes N bytes of data in the Mailbox, starting from first Mailbox Address.
  * @param  ptr_data   Pointer to the buffer containing the data to be written.
  * @param  num_byte  Number of bytes to be written.
  * @return int32_t enum status.
  */
int32_t NFCST25DV::write_mailbox_data(const uint8_t* const ptr_data,  const uint16_t num_byte)
{
    return ST25DV_WriteMailboxData(&_std25dv_element, ptr_data, num_byte);
}

/**
  * @brief  Reads N bytes from the mailbox registers, starting at the specified I2C address.
  * @param  ptr_data   Pointer on the buffer used to return the data.
  * @param  reg_addr I2C memory address to be read.
  * @param  num_byte  Number of bytes to be read.
  * @return int32_t enum status.
  */
int32_t NFCST25DV::read_mailbox_register(uint8_t* const ptr_data, const uint16_t reg_addr,
        const uint16_t num_byte)
{
    return ST25DV_ReadMailboxRegister(&_std25dv_element, ptr_data, reg_addr, num_byte);
}

/**
  * @brief  Writes N bytes to the specified mailbox register.
  * @param  ptr_data Pointer on the data to be written.
  * @param  reg_addr I2C register address to be written.
  * @param  num_byte Number of bytes to be written.
  * @return int32_t enum status.
  */
int32_t NFCST25DV::write_mailbox_register(const uint8_t* const ptr_data,
        const uint16_t reg_addr, const uint16_t num_byte)
{
    return ST25DV_WriteMailboxRegister(&_std25dv_element, ptr_data, reg_addr, num_byte);
}

/**
  * @brief  Reads the status of the security session open register.
  * @param  ptr_session Pointer on a ST25DV_I2CSSO_STATUS value used to return the session status.
  * @return int32_t enum status.
  */
int32_t NFCST25DV::read_i2c_security_session_dyn(ST25DV_I2CSSO_STATUS* const ptr_session)
{
    return ST25DV_ReadI2CSecuritySession_Dyn(&_std25dv_element, ptr_session);
}

/**
  * @brief  Reads the IT status register from the ST25DV.
  * @param  ptr_it_status Pointer on uint8_t, used to return the IT status, such as:
  *                       - RFUSERSTATE = 0x01
  *                       - RFBUSY = 0x02
  *                       - RFINTERRUPT = 0x04
  *                       - FIELDFALLING = 0x08
  *                       - FIELDRISING = 0x10
  *                       - RFPUTMSG = 0x20
  *                       - RFGETMSG = 0x40
  *                       - RFWRITE = 0x80
  *
  * @return int32_t enum status.
  */
int32_t NFCST25DV::read_it_status_dyn(uint8_t* const ptr_it_status)
{
    return ST25DV_ReadITSTStatus_Dyn(&_std25dv_element, ptr_it_status);
}

/**
  * @brief  Read value of dynamic GPO register configuration.
  * @param  ptr_gpo_config ST25DV_GPO pointer of the dynamic GPO configuration to store.
  * @retval NFCTAG enum status.
  */
int32_t NFCST25DV::read_gpo_dyn(uint8_t* ptr_gpo_config)
{
    return ST25DV_ReadGPO_Dyn(&_std25dv_element, ptr_gpo_config);
}

/**
  * @brief  Get dynamique GPO enable status
  * @param  ptr_gpo_en ST25DV_EN_STATUS pointer of the GPO enable status to store.
  * @retval NFCTAG enum status
  */
int32_t NFCST25DV::get_gpo_en_dyn(ST25DV_EN_STATUS* const ptr_gpo_en)
{
    return ST25DV_GetGPO_en_Dyn(&_std25dv_element, ptr_gpo_en);
}

/**
  * @brief  Set dynamique GPO enable configuration.
  * @param  None No parameters.
  * @retval NFCTAG enum status.
  */
int32_t NFCST25DV::set_gpo_en_dyn(void)
{
    return ST25DV_SetGPO_en_Dyn(&_std25dv_element);
}

/**
  * @brief  Reset dynamique GPO enable configuration.
  * @param  None No parameters.
  * @retval NFCTAG enum status.
  */
int32_t NFCST25DV::reset_gpo_en_dyn(void)
{
    return ST25DV_ResetGPO_en_Dyn(&_std25dv_element);
}

/**
  * @brief  Read value of dynamic EH Ctrl register configuration
  * @param  ptr_eh_ctrl ST25DV_EH_CTRL pointer of the dynamic EH Ctrl configuration to store.
  * @retval NFCTAG enum status
  */
int32_t NFCST25DV::read_eh_ctrl_dyn(ST25DV_EH_CTRL* const ptr_eh_ctrl)
{
    return ST25DV_ReadEHCtrl_Dyn(&_std25dv_element, ptr_eh_ctrl);
}

/**
  * @brief  Reads the Energy Harvesting dynamic status.
  * @param  ptr_eh_val Pointer on a ST25DV_EN_STATUS value used to return the Energy Harvesting
  * dynamic status.
  * @return int32_t enum status.
  */
int32_t NFCST25DV::get_eh_en_mode_dyn(ST25DV_EN_STATUS* const ptr_eh_val)
{
    return ST25DV_GetEHENMode_Dyn(&_std25dv_element, ptr_eh_val);
}

/**
  * @brief  Dynamically sets the Energy Harvesting mode.
  * @return int32_t enum status.
  */
int32_t NFCST25DV::set_eh_en_mode_dyn(void)
{
    return ST25DV_SetEHENMode_Dyn(&_std25dv_element);
}

/**
  * @brief  Dynamically unsets the Energy Harvesting mode.
  * @return int32_t enum status.
  */
int32_t NFCST25DV::reset_eh_en_mode_dyn(void)
{
    return ST25DV_ResetEHENMode_Dyn(&_std25dv_element);
}

/**
  * @brief  Reads the EH_ON status from the EH_CTRL_DYN register.
  * @param  ptr_eh_on Pointer on a ST25DV_EN_STATUS value used to return the EHON status.
  * @return int32_t enum status.
  */
int32_t NFCST25DV::get_eh_on_dyn(ST25DV_EN_STATUS * const ptr_eh_on)
{
    return ST25DV_GetEHON_Dyn(&_std25dv_element, ptr_eh_on);
}

/**
  * @brief  Checks if RF Field is present in front of the ST25DV.
  * @param  ptr_rf_field Pointer on a ST25DV_FIELD_STATUS value used to return the field presence.
  * @return int32_t enum status.
  */
int32_t NFCST25DV::get_rf_field_dyn(ST25DV_FIELD_STATUS * const ptr_rf_field)
{
    return ST25DV_GetRFField_Dyn(&_std25dv_element, ptr_rf_field);
}

/**
  * @brief  Check if VCC is supplying the ST25DV.
  * @param  ptr_vcc ST25DV_VCC_STATUS pointer of the VCC status to store.
  * @retval NFCTAG enum status.
  */
int32_t NFCST25DV::get_vcc_dyn(ST25DV_VCC_STATUS* const ptr_vcc)
{
    return ST25DV_GetVCC_Dyn(&_std25dv_element, ptr_vcc);
}

/**
  * @brief  Read value of dynamic RF Management configuration
  * @param  ptr_rf_mngt ST25DV_RF_MNGT pointer of the dynamic RF Management configuration to store.
  * @retval NFCTAG enum status
  */
int32_t NFCST25DV::read_rf_mngt_cfg_dyn(ST25DV_RF_MNGT* const ptr_rf_mngt)
{
    return ST25DV_ReadRFMngt_Dyn(&_std25dv_element, ptr_rf_mngt);
}

/**
  * @brief  Writes a value to the RF Management dynamic register.
  * @param  rf_mngt Value to be written to the RF Management dynamic register.
  * @return int32_t enum status.
  */
int32_t NFCST25DV::write_rf_mngt_cfg_Dyn(const uint8_t rf_mngt)
{
    return ST25DV_WriteRFMngt_Dyn(&_std25dv_element, rf_mngt);
}

/**
  * @brief  Reads the RFDisable dynamic register information.
  * @param  ptr_rf_disable Pointer on a ST25DV_EN_STATUS value used to return the RF Disable state.
  * @return int32_t enum status.
  */
int32_t NFCST25DV::get_rf_disable_dyn(ST25DV_EN_STATUS* const ptr_rf_disable)
{
    return ST25DV_GetRFDisable_Dyn(&_std25dv_element, ptr_rf_disable);
}

/**
  * @brief  Sets the RF Disable dynamic configuration.
  * @return int32_t enum status.
  */
int32_t NFCST25DV::set_rf_disable_dyn(void)
{
    return ST25DV_SetRFDisable_Dyn(&_std25dv_element);
}

/**
  * @brief  Unsets the RF Disable dynamic configuration.
  * @return int32_t enum status.
  */
int32_t NFCST25DV::reset_rf_disable_dyn(void)
{
    return ST25DV_ResetRFDisable_Dyn(&_std25dv_element);
}

/**
  * @brief  Reads the RFSleep dynamic register information.
  * @param  ptr_rf_sleep Pointer on a ST25DV_EN_STATUS values used to return the RF Sleep state.
  * @return int32_t enum status.
  */
int32_t NFCST25DV::get_rf_sleep_dyn(ST25DV_EN_STATUS* const ptr_rf_sleep)
{
    return ST25DV_GetRFSleep_Dyn(&_std25dv_element, ptr_rf_sleep);
}

/**
  * @brief  Sets the RF Sleep dynamic configuration.
  * @return int32_t enum status.
  */
int32_t NFCST25DV::set_rf_sleep_dyn(void)
{
    return ST25DV_SetRFSleep_Dyn(&_std25dv_element);
}

/**
  * @brief  Unsets the RF Sleep dynamic configuration.
  * @return int32_t enum status.
  */
int32_t NFCST25DV::reset_rf_sleep_dyn(void)
{
    return ST25DV_ResetRFSleep_Dyn(&_std25dv_element);
}

/**
  * @brief  Reads the Mailbox ctrl dynamic register.
  * @param  ptr_ctrl_status Pointer on a ST25DV_MB_CTRL_DYN_STATUS structure used to return
  * the dynamic Mailbox ctrl information.
  * @return int32_t enum status.
  */
int32_t NFCST25DV::read_mailbox_ctrl_dyn(ST25DV_MB_CTRL_DYN_STATUS* const ptr_ctrl_status)
{
    return ST25DV_ReadMBCtrl_Dyn(&_std25dv_element, ptr_ctrl_status);
}

/**
  * @brief  Reads the Mailbox Enable dynamic configuration.
  * @param  ptr_mb_en Pointer on a ST25DV_EN_STATUS used to return the Mailbox Enable config.
  * @return int32_t enum status.
  */
int32_t NFCST25DV::get_mailbox_en_dyn(ST25DV_EN_STATUS* const ptr_mb_en)
{
    return ST25DV_GetMBEN_Dyn(&_std25dv_element, ptr_mb_en);
}

/**
  * @brief  Sets the Mailbox Enable dynamic configuration.
  * @return int32_t enum status.
  */
int32_t NFCST25DV::set_mailbox_en_dyn(void)
{
    return ST25DV_SetMBEN_Dyn(&_std25dv_element);
}

/**
  * @brief  Unsets the Mailbox Enable dynamic configuration.
  * @return int32_t enum status.
  */
int32_t NFCST25DV::reset_mailbox_en_dyn(void)
{
    return ST25DV_ResetMBEN_Dyn(&_std25dv_element);
}

/**
  * @brief  Reads the number of bytes currently available in FTM Mailbox.
  * @param  ptr_mb_length Pointer on a uint8_t used to return the Mailbox message length.
  * @return int32_t enum status.
  */
int32_t NFCST25DV::read_mailbox_length_dyn(uint8_t* const ptr_mb_length)
{
    int32_t rc = ST25DV_ReadMBLength_Dyn(&_std25dv_element, ptr_mb_length);
    if(rc == NFCTAG_OK)
        *ptr_mb_length = *ptr_mb_length + 1;
    return rc;
}

/**
  * @brief  Open I2C Security Session to allow static system config registers modification.
  * @param  passwd_msb ST25DV 64 bits password MSB to open I2C Security Session.
  * @param  passwd_lsb ST25DV 64 bits password LSB to open I2C Security Session.
  * @return int32_t enum status.
  */
int32_t NFCST25DV::open_i2c_security_session(const uint32_t passwd_msb,const uint32_t passwd_lsb)
{
    int32_t rc = NFCTAG_OK;
    ST25DV_PASSWD password;
    ST25DV_I2CSSO_STATUS i2c_session_status;

    // Check if I2C Security Session is already open
    rc = read_i2c_security_session_dyn(&i2c_session_status);
    if(rc == NFCTAG_OK)
    {
        if(i2c_session_status == ST25DV_SESSION_OPEN)
        {
            _println("[NFCST25DV] Info: Security Session already open.");
            return NFCTAG_OK;
        }
    }

    // If session is not open, present Password to open it
    password.LsbPasswd = passwd_lsb;
    password.MsbPasswd = passwd_msb;
    rc = present_i2c_password(password);
    if(rc != NFCTAG_OK)
    {
        _println("[NFCST25DV] Error: Can't present I2C Password to open Security Session.");
        return rc;
    }

    // Check open result
    rc = read_i2c_security_session_dyn(&i2c_session_status);
    if(rc != NFCTAG_OK)
    {
        _println("[NFCST25DV] Error: Can't read I2C Security Session status.");
        return rc;
    }
    if(i2c_session_status != ST25DV_SESSION_OPEN)
        return NFCTAG_ERROR;

    return NFCTAG_OK;
}

/**
  * @brief  Close I2C Security Session.
  * @return int32_t enum status.
  */
int32_t NFCST25DV::close_i2c_security_session(void)
{
    int32_t rc = NFCTAG_OK;
    ST25DV_PASSWD password;
    ST25DV_I2CSSO_STATUS i2c_session_status;

    // Check if I2C Security Session is already open
    rc = read_i2c_security_session_dyn(&i2c_session_status);
    if(rc == NFCTAG_OK)
    {
        if(i2c_session_status == ST25DV_SESSION_CLOSED)
        {
            _println("[NFCST25DV] Info: Security Session already closed.");
            return NFCTAG_OK;
        }
    }

    // Present an invalid Password to force close it without POR
    password.LsbPasswd = RESERVED_PASSWD;
    password.MsbPasswd = RESERVED_PASSWD;
    if(present_i2c_password(password) != NFCTAG_OK)
    {
        _println("[NFCST25DV] Error: Can't present I2C Password to close Security Session.");
        return NFCTAG_ERROR;
    }

    // Check close result
    rc = read_i2c_security_session_dyn(&i2c_session_status);
    if(rc != NFCTAG_OK)
    {
        _println("[NFCST25DV] Warning: Can't read if I2C Security Session is closed.");
        return NFCTAG_ERROR;
    }
    if(i2c_session_status != ST25DV_SESSION_CLOSED)
        return NFCTAG_ERROR;
    return NFCTAG_OK;
}

/**
  * @brief  Authorize Fast Transfer Mode usage (set MB_MODE).
  * @param  lock_status Lock or unlock FTM (0 - lock; 1 - unlock).
  * @return int32_t enum status.
  */
int32_t NFCST25DV::ftm_lock_unlock_use(const ST25DV_EN_STATUS lock_status)
{
    ST25DV_EN_STATUS mb_mode_status;

    // Check MB_MODE Register enable/disable status
    if(read_mailbox_mode(&mb_mode_status) != NFCTAG_OK)
    {
        _println("[NFCST25DV] Error: Can't Read mb_mode Register.");
        return NFCTAG_ERROR;
    }

    // Check if it is already locked/unlocked
    if(mb_mode_status == lock_status)
    {
        if(lock_status == ST25DV_ENABLE)
            _println("[NFCST25DV] Info: FTM already unlocked.");
        else
            _println("[NFCST25DV] Info: FTM already locked.");
        return NFCTAG_OK;
    }

    // Enable MB_MODE Register
    return write_mailbox_mode(lock_status);
}

/**
  * @brief  Enable/Disable Fast Transfer Mode (set MB_EN).
  * @param  enable Enable or disable (0 - Disable; 1 - Enable).
  * @return int32_t enum status.
  */
int32_t NFCST25DV::ftm_enable_disable(const uint8_t enable)
{
    if(enable)
        return set_mailbox_en_dyn();
    else
        return reset_mailbox_en_dyn();
}

/**
  * @brief  Disable Fast Transfer Mode (set MB_EN).
  * @return int32_t enum status.
  */
int32_t NFCST25DV::ftm_disable(void)
{
    //if(ftm_status() == ST25DV_DISABLE)
    //{
    //    _println("FTM already disabled.");
    //    return NFCTAG_OK;
    //}
    return ftm_enable_disable(0);
}

/**
  * @brief  Enable Fast Transfer Mode (set MB_EN).
  * @return int32_t enum status.
  */
int32_t NFCST25DV::ftm_enable(void)
{
    //if(ftm_status() == ST25DV_ENABLE)
    //{
    //    _println("FTM already enabled.");
    //    return NFCTAG_OK;
    //}
    return ftm_enable_disable(1);
}

/**
  * @brief  Check if Fast Transfer Mode is enabled or disabled (get MB_EN).
  * @return int32_t enum status.
  */
int32_t NFCST25DV::ftm_status(void)
{
    ST25DV_EN_STATUS ftm_status;

    if(get_mailbox_en_dyn(&ftm_status) != NFCTAG_OK)
    {
        _println("[NFCST25DV] Error: Can't Read mb_en Register.");
        return NFCTAG_ERROR;
    }
    return ftm_status;
}
