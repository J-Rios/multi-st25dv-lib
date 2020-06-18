/*************************************************************************************************/
// File: ftm_fw_upgrade.h
// Description: Firmware upgrade through ST25DV Fast Transfer Mode functions.
// Created on: 09 jun. 2020
// Last modified date: 17 jun. 2020
// Version: 1.0.0
/*************************************************************************************************/

#include "ftm_fw_upgrade.h"

/*************************************************************************************************/

/* HALs */

#if defined(ARDUINO) // Arduino Framework
    #include "HardwareSerial.h"
    #define _print(x) do { Serial.print(x); } while(0)
    #define _println(x) do { Serial.println(x); } while(0)
    #define _printf(...) do { Serial.printf(__VA_ARGS__); } while(0)
    #define _delay(x) do { delay(x); } while(0)
#elif defined(ESP_IDF) // ESP32 ESPIDF Framework
    #define _print(x) do { printf("%s", x); } while(0)
    #define _println(x) do { printf("%s\n", x); } while(0)
    #define _printf(...) do { printf(__VA_ARGS__); } while(0)
    #define _delay(x) do { vTaskDelay(x/portTICK_PERIOD_MS); } while(0)
#elif defined(SAMD20_ASF) || defined(SAML21_ASF) // SAM0 ASF Framework
    #include "asf.h"
    //#include "debug.h"

    #define _print(x)    //do { debug.print(x); } while(0)
    #define _println(x)  //do { debug.println(x); } while(0)
    #define _printf(...) //do { debug.printf(__VA_ARGS__); } while(0)
    extern volatile uint32_t systickCount;
    #define _delay(x) do \
    { \
        uint32_t t0 = systickCount; \
        while( (systickCount - t0)  <= x) \
            wdt_reset_count(); \
        \
    } while(0)
#else
    #define _print(x)
    #define _println(x)
    #define _printf(...)
    #define _delay(x)
#endif

/*************************************************************************************************/

/* File Scope Global Variables */

/**
  * @brief    Pointer to NFCST25DV object declared outside this scope.
  * @details  Will be set by calling ftm_setup() function. 
  */
NFCST25DV* _nfc = (NFCST25DV*)NULL;

/*************************************************************************************************/

/* File Scope Private Functions */

static bool open_st25dv_security_session(const uint32_t passwd_msb, const uint32_t passwd_lsb);
static bool close_st25dv_security_session(void);
static bool unlock_fast_transfer_mode(void);
#if 0
static bool lock_fast_transfer_mode(void);
#endif
static bool setup_gpo_events(void);
static bool setup_gpo_pulse_time(const ST25DV_PULSE_DURATION int_time);
static bool gpo_events_check(IT_GPO_STATUS* const gpo);

/*************************************************************************************************/

/**
  * @brief  Setup all necessary registers to allow FTM transactions.
  * @param  nfc Pointer to an external NFC object to be used in current file functions.
  * @param  passwd_msb I2C Security Session password MSB needed to setup FTM.
  * @param  passwd_lsb I2C Security Session password LSB needed to setup FTM.
  * @return Setup ok true/false.
  */
bool ftm_setup(NFCST25DV* nfc, const uint32_t passwd_msb, const uint32_t passwd_lsb)
{
    _nfc = nfc;
    open_st25dv_security_session(passwd_msb, passwd_lsb);
    unlock_fast_transfer_mode();
    setup_gpo_events();
    setup_gpo_pulse_time(ST25DV_302_US);
    _nfc->write_mailbox_wd_duration_coef(0);
    close_st25dv_security_session();
    return true;
}

/**
  * @brief  Enable FTM.
  * @return true/false.
  */
bool ftm_on(void)
{
    if(_nfc == NULL)
        return false;

    if(_nfc->ftm_status() != ST25DV_ENABLE)
    {
        if(_nfc->ftm_enable() != NFCTAG_OK)
        {
            _println("Error: Can't enable Fast Transfer Mode.");
            return false;
        }
        _println("Fast Transfer Mode enabled.");
    }
    else
    {
        if(ftm_clear_mb())
            _println("Fast Transfer Mode enabled.");
    }
    
    return true;
}

/**
  * @brief  Disable FTM.
  * @return true/false.
  */
bool ftm_off(void)
{
    if(_nfc == NULL)
        return false;

    if(_nfc->ftm_status() != ST25DV_DISABLE)
    {
        if(_nfc->ftm_disable() != NFCTAG_OK)
        {
            _println("Error: Can't disable Fast Transfer Mode.");
            return false;
        }
    }
    _println("Fast Transfer Mode disabled.");
    return true;
}

/**
  * @brief  Clear FTM Mailbox (turn off and then on, the FTM).
  * @return true/false.
  */
bool ftm_clear_mb(void)
{
    if(_nfc == NULL)
        return false;

    if(_nfc->ftm_disable() != NFCTAG_OK)
    {
        _println("Error: Can't clear mailbox (disable FTM).");
        return false;
    }
    if(_nfc->ftm_enable() != NFCTAG_OK)
    {
        _println("Error: Can't clear mailbox (enable FTM).");
        return false;
    }

    return true;
}

/**
  * @brief  Check and shows if FTM is on/off.
  * @return FTM Status: ST25DV_ENABLE/ST25DV_DISABLE.
  */
int8_t ftm_get_status(void)
{
    if(_nfc == NULL)
        return -1;

    int32_t fts_status = _nfc->ftm_status();

    if(fts_status == ST25DV_ENABLE)
    {
        _println("Fast Transfer Mode is enabled.");
        return ST25DV_ENABLE;
    }
    else if(fts_status == ST25DV_DISABLE)
    {
        _println("Fast Transfer Mode is disabled.");
        return ST25DV_DISABLE;
    }
    else
    {
        _println("Can't get current FTM status.");
        return -2;
    }
}

/**
  * @brief  Check if a GPO event has fired.
  * @return true/false.
  */
bool gpo_event_fire(IT_GPO_STATUS* const gpo)
{
    static uint8_t gpo_event_on = 255;

    if(_nfc->get_gpo_value() == HIGH)
    {
        if(gpo_event_on != HIGH)
        {
            _println("GPO HIGH");
            gpo_event_on = HIGH;
            return gpo_events_check(gpo);
        }
    }
    else
    {
        if(gpo_event_on != LOW)
        {
            _println("GPO LOW");
            gpo_event_on = LOW;
        }
    }

    return false;
}

/**
  * @brief  Shows through Serial current GPO events.
  * @param  gpo GPO events structure to check.
  */
void shows_gpo_event(const IT_GPO_STATUS gpo)
{
    if((gpo.Rfuser == 0) && (gpo.RfActivity == 0) && (gpo.RfInterrupt == 0) && (gpo.FieldOff == 0)
    && (gpo.FieldOn == 0) && (gpo.MsgInMailbox == 0) && (gpo.MailboxMsgRead == 0)
    && (gpo.WriteInEEPROM == 0))
        return;

    _println("\nGPO events detected:");
    if(gpo.Rfuser == 1)
        _println("  RF_USER");
    if(gpo.RfActivity == 1)
        _println("  RF_ACTIVITY");
    if(gpo.RfInterrupt == 1)
        _println("  RF_INTERRUPT");
    if(gpo.FieldOff == 1)
        _println("  RF_FIELD_OFF");
    if(gpo.FieldOn == 1)
        _println("  RF_FIELD_ON");
    if(gpo.MsgInMailbox == 1)
        _println("  RF_MB_WRITE");
    if(gpo.MailboxMsgRead == 1)
        _println("  RF_MB_READ");
    if(gpo.WriteInEEPROM == 1)
        _println("  RF_EEPROM_WRITE");
    _println("");
}

/**
  * @brief  Write a message into FTM Mailbox.
  * @param  ptr_data Pointer to data to write.
  * @param  num_bytes Number of bytes to write.
  * @return NFCTAG_StatusTypeDef status.
  */
int32_t ftm_write_msg(const uint8_t* const ptr_data, const uint16_t num_bytes)
{
    int32_t ret = NFCTAG_OK;
    ST25DV_MB_CTRL_DYN_STATUS data = {0};

    /* Check if Mailbox is available */
    ret = _nfc->read_mailbox_ctrl_dyn(&data);
    if(ret != NFCTAG_OK)
    {
        _delay(20);
        return ret;
    }

    /* If available, write data */
    if((data.HostPutMsg == 0) && (data.RfPutMsg == 0))
        ret = _nfc->write_mailbox_data(ptr_data, num_bytes);
    else 
    {
        _delay(20);
        return NFCTAG_BUSY;
    }

    return ret;
}

/**
  * @brief  Read a message from FTM Mailbox.
  * @param  ptr_data Pointer to read data.
  * @param  num_bytes Number of read bytes.
  * @return NFCTAG_StatusTypeDef status.
  */
int32_t ftm_read_msg(uint8_t* const ptr_data, uint8_t* const num_bytes)
{
    int32_t ret = NFCTAG_OK;

    // Read length of message
    ret = _nfc->read_mailbox_length_dyn(num_bytes);
    if(ret != NFCTAG_OK)
    {
        _println("Error: Can't get number of bytes in Mailbox buffer.");
        ftm_clear_mb();
        return ret;
    }

    // Read all data in Mailbox
    return ftm_read_data(ptr_data, 0, *num_bytes);
}

/**
  * @brief  Reads data of Mailbox Message.
  * @param  ptr_data Pointer to data read.
  * @param  offset Offset in Mailbox to start reading.
  * @param  num_bytes Number of bytes to read.
  * @return NFCTAG_StatusTypeDef status.
  */
int32_t ftm_read_data(uint8_t* const ptr_data, const uint8_t offset, const uint16_t num_bytes)
{
    int32_t ret = NFCTAG_OK;
    
    ret = _nfc->read_mailbox_data(ptr_data, offset, num_bytes);
    if(ret != NFCTAG_OK)
    {
        _println("Error: Can't read from the Mailbox buffer.");
        ftm_clear_mb();
    }

    return ret;
}

/**
  * @brief  Prepares frame and send message to FTM Mailbox.
  * @param  msg_header Pointer to structure containing frame header info.
  * @param  num_retries Number of attempts.
  * @retval 1 Message was written to Mailbox.
  * @retval 0 Message was not written to Mailbox.
  */
bool ftm_send_msg(MB_HEADER_T* const msg_header, const uint8_t num_retries)
{
    uint8_t msg_buffer[256];
    uint8_t header_size = 0;
    uint32_t cnt = 0;
    int32_t ret;

    /* Write header to data frame */
    msg_add_header(msg_buffer, msg_header);

    /* Check type of frame chained or not */
    if(msg_header->chaining == MB_CHAINED)
        header_size = MB_CH_DATA;
    else
        header_size = MB_DATA;

    /* Copy data to buffer after header */
    for(cnt = 0; cnt < msg_header->framelength; cnt++)
        msg_buffer[cnt + header_size] = *(msg_header->pData + cnt);

    /* Compute total frame size */
    msg_header->framesize = header_size + msg_header->framelength;

    /* Write message to Mailbox with nbretry tentative*/
    cnt = num_retries;
    do
    {
        ret = ftm_write_msg(msg_buffer, msg_header->framesize);
        if(ret != NFCTAG_OK)
            cnt--;
    }while((ret != NFCTAG_OK) && (cnt > 0));

    if(cnt == 0)
        return false;

    return true;
}

/**
  * @brief  Prepares header message into buffer to send to Mailbox.
  * @param  ptr_data Pointer the the message buffer to send.
  * @param  msg_header Pointer to the header information structure.
  */
void msg_add_header(uint8_t* const ptr_data, const MB_HEADER_T* const msg_header)
{
    ptr_data[MB_FCTCODE] = msg_header->fctcode;
    ptr_data[MB_CMDRESP] = msg_header->cmdresp;
    ptr_data[MB_ERROR] = msg_header->error;
    ptr_data[MB_CHAINING] = msg_header->chaining;
    if( msg_header->chaining == MB_NOTCHAINED )
        ptr_data[MB_LENGTH] = msg_header->framelength;
    else
    {
        ptr_data[MB_CH_FULLLENGTH - 3] = (msg_header->fulllength >> 24) & 0xFF;
        ptr_data[MB_CH_FULLLENGTH - 2] = (msg_header->fulllength >> 16) & 0xFF;
        ptr_data[MB_CH_FULLLENGTH - 1] = (msg_header->fulllength >> 8) & 0xFF;
        ptr_data[MB_CH_FULLLENGTH] = msg_header->fulllength & 0xFF;
        ptr_data[MB_CH_TOTALCHUNK - 1] = (msg_header->totalchunk >> 8) & 0xFF;
        ptr_data[MB_CH_TOTALCHUNK] = msg_header->totalchunk & 0xFF;
        ptr_data[MB_CH_NBCHUNK - 1] = (msg_header->chunknb >> 8) & 0xFF;
        ptr_data[MB_CH_NBCHUNK] = msg_header->chunknb & 0xFF;
        ptr_data[MB_CH_LENGTH] = msg_header->framelength;
    }
}

/**
  * @brief  Extracts global information from header in Fast transfer mode protocol.
  * @param  ptr_data Pointer to the mailbox frame message read.
  * @param  msg_header Pointer to structure for storing global header info.
  */
void msg_decode_header(const uint8_t* const ptr_data, MB_HEADER_T* const msg_header)
{
    msg_header->fctcode = ptr_data[MB_FCTCODE];
    msg_header->cmdresp = ptr_data[MB_CMDRESP];
    msg_header->error = ptr_data[MB_ERROR];
    msg_header->chaining = ptr_data[MB_CHAINING];
    if( msg_header->chaining == MB_NOTCHAINED )
    {
        /* If simple message header is 5-byte long */
        msg_header->framelength = ptr_data[MB_LENGTH];
        msg_header->framesize = MB_DATA + msg_header->framelength;
    }
    else
    {
        /* If simple message header is 13-byte long */
        msg_header->fulllength = ptr_data[MB_CH_FULLLENGTH - 3] & 0xFF;
        msg_header->fulllength = (msg_header->fulllength << 8) | ptr_data[MB_CH_FULLLENGTH - 2];
        msg_header->fulllength = (msg_header->fulllength << 8) | ptr_data[MB_CH_FULLLENGTH - 1];
        msg_header->fulllength = (msg_header->fulllength << 8) | ptr_data[MB_CH_FULLLENGTH];
        msg_header->totalchunk = ptr_data[MB_CH_TOTALCHUNK - 1] & 0xFF;
        msg_header->totalchunk = (msg_header->totalchunk << 8) | ptr_data[MB_CH_TOTALCHUNK];
        msg_header->chunknb = ptr_data[MB_CH_NBCHUNK - 1] & 0xFF;
        msg_header->chunknb = (msg_header->chunknb << 8) | ptr_data[MB_CH_NBCHUNK];
        msg_header->framelength = ptr_data[MB_CH_LENGTH];
        msg_header->framesize = MB_CH_LENGTH + msg_header->framelength;
    }
}

bool check_crc(uint32_t* data, const uint32_t data_length)
{
    uint32_t crc = 0U;

    for(uint32_t i = 0; i < data_length; i++)
        crc = data[i];

    return crc;
}

/*************************************************************************************************/

/* File Scope Private Functions */

/**
  * @brief  Try to open ST25DV I2C Security Session.
  * @return true/false.
  */
static bool open_st25dv_security_session(const uint32_t passwd_msb, const uint32_t passwd_lsb)
{
    if(_nfc == NULL)
        return false;

    if(_nfc->open_i2c_security_session(passwd_msb, passwd_lsb) != NFCTAG_OK)
    {
        _println("Error: Can't open I2C Security Session.");
        return false;
    }
    _println("I2C Security Session open.");
    return true;
}

/**
  * @brief  Try to close ST25DV I2C Security Session.
  * @return true/false.
  */
static bool close_st25dv_security_session(void)
{
    if(_nfc == NULL)
        return false;

    if(_nfc->close_i2c_security_session() != NFCTAG_OK)
    {
        _println("Error: Can't check if I2C Security Session was closed.");
        return false;
    }
    _println("I2C Security Session closed.");
    return true;
}

/**
  * @brief  Try to unlock ST25DV FTM functionality.
  * @return true/false.
  */
static bool unlock_fast_transfer_mode(void)
{
    if(_nfc == NULL)
        return false;

    if(_nfc->ftm_lock_unlock_use(ST25DV_ENABLE) != NFCTAG_OK)
    {
        // I2C Security Session must be open to lock/unlock functionality
        _println("Error: Can't unlock Fast Transfer Mode functionality.");
        return false;
    }
    _println("Fast Transfer Mode functionality unlocked.");
    return true;
}

#if 0
/**
  * @brief  Try to close ST25DV FTM functionality.
  * @return true/false.
  */
static bool lock_fast_transfer_mode(void)
{
    if(_nfc == NULL)
        return false;

    if(_nfc->ftm_lock_unlock_use(ST25DV_DISABLE) != NFCTAG_OK)
    {
        // I2C Security Session must be open to lock/unlock functionality
        _println("Error: Can't lock Fast Transfer Mode.");
        return false;
    }
    _println("Fast Transfer Mode functionality locked.");
    return true;
}
#endif

/**
  * @brief  Setup necessary GPO events for FTM transactions.
  * @return Setup ok true/false.
  */
static bool setup_gpo_events(void)
{
    if(_nfc == NULL)
        return false;

    int32_t gpo_cfg = 0;

    //gpo_cfg = (gpo_cfg | ST25DV_GPO_RFUSERSTATE_MASK);
    //gpo_cfg = (gpo_cfg | ST25DV_GPO_RFACTIVITY_MASK);
    //gpo_cfg = (gpo_cfg | ST25DV_GPO_RFINTERRUPT_MASK);
    gpo_cfg = (gpo_cfg | ST25DV_GPO_FIELDCHANGE_MASK);
    gpo_cfg = (gpo_cfg | ST25DV_GPO_RFPUTMSG_MASK);
    gpo_cfg = (gpo_cfg | ST25DV_GPO_RFGETMSG_MASK);
    //gpo_cfg = (gpo_cfg | ST25DV_GPO_RFWRITE_MASK);
    gpo_cfg = (gpo_cfg | ST25DV_GPO_ENABLE_MASK);

    if(_nfc->set_interrupt_status(gpo_cfg) != NFCTAG_OK)
    {
        _println("Can't setup GPO events configuration.");
        return false;
    }
    _println("GPO events configured.");
    return true;
}

/**
  * @brief  Setup GPO pulse width time to allow FTM transactions.
  * @param  int_time ST25DV_PULSE_DURATION value (pulse time us).
  * @return Setup ok true/false.
  */
static bool setup_gpo_pulse_time(const ST25DV_PULSE_DURATION int_time)
{
    if(_nfc == NULL)
        return false;

    if(_nfc->set_interrupt_pulse(int_time) != NFCTAG_OK)
    {
        _println("Can't set GPO pulse time.");
        return false;
    }
    return true;
}

/**
  * @brief  Check and get current GPO events.
  * @param  gpo GPO events structure to get.
  * @return GPO event check success/fail (true/false).
  */
static bool gpo_events_check(IT_GPO_STATUS* const gpo)
{
    if(_nfc == NULL)
        return false;

    uint8_t itstatus;

    if(_nfc->read_it_status_dyn(&itstatus) != NFCTAG_OK)
    {
        _println("Can't get GPO events fired.");
        return false;
    }

    if((itstatus & ST25DV_ITSTS_DYN_RFUSERSTATE_MASK) == ST25DV_ITSTS_DYN_RFUSERSTATE_MASK)
        gpo->Rfuser = 1;
    if((itstatus & ST25DV_ITSTS_DYN_RFACTIVITY_MASK) == ST25DV_ITSTS_DYN_RFACTIVITY_MASK)
        gpo->RfActivity = 1;
    if((itstatus & ST25DV_ITSTS_DYN_RFINTERRUPT_MASK) == ST25DV_ITSTS_DYN_RFINTERRUPT_MASK)
        gpo->RfInterrupt = 1;
    if((itstatus & ST25DV_ITSTS_DYN_FIELDFALLING_MASK) == ST25DV_ITSTS_DYN_FIELDFALLING_MASK)
        gpo->FieldOff = 1;
    if((itstatus & ST25DV_ITSTS_DYN_FIELDRISING_MASK) == ST25DV_ITSTS_DYN_FIELDRISING_MASK)
        gpo->FieldOn = 1;
    if((itstatus & ST25DV_ITSTS_DYN_RFPUTMSG_MASK) == ST25DV_ITSTS_DYN_RFPUTMSG_MASK)
        gpo->MsgInMailbox = 1;
    if((itstatus & ST25DV_ITSTS_DYN_RFGETMSG_MASK) == ST25DV_ITSTS_DYN_RFGETMSG_MASK)
        gpo->MailboxMsgRead = 1;
    if((itstatus & ST25DV_ITSTS_DYN_RFWRITE_MASK) == ST25DV_ITSTS_DYN_RFWRITE_MASK)
        gpo->WriteInEEPROM = 1;

    return true;
}
