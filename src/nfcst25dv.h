/*************************************************************************************************/
// File: nfcst25dv.h
// Description: NFC Dynamic Tags ST25DV ease library.
// Created on: 18 may. 2020
// Last modified date: 24 may. 2020
// Version: 1.0.0
// Notes: I2C must be initialized outside this library.
/*************************************************************************************************/

/* Include Guard */
#ifndef NFCST25DV_H_
#define NFCST25DV_H_

/* C++ compiler compatibility */
#ifdef __cplusplus
extern "C" {
#endif

/*************************************************************************************************/

/* Libraries */

#include "utility/ST25DV/st25dv.h"

#include <stdint.h>

/*************************************************************************************************/

/* Constants */

// Uninitialized elements value
const int8_t UNINITIALIZE = -1;

/*************************************************************************************************/

class NFCST25DV
{
    public:
        // Public Methods
        NFCST25DV(const int8_t gpio_eh=UNINITIALIZE, const int8_t gpio_gpo=UNINITIALIZE,
                const int8_t gpio_lpd=UNINITIALIZE);
        int32_t init(void);
        uint8_t get_gpo_value(void);
        bool is_in_low_power_mode(void);
        void low_power_mode(const bool enable);
        int32_t is_tag_ready(const uint32_t retries);
        uint8_t get_tag_ic_ref(void);
        int32_t get_tag_ic_rev(uint8_t* const ic_revision);
        int32_t read_tag_uid(ST25DV_UID* const uid);
        uint32_t read_tag_size(void);
        int32_t read_data(const uint16_t addr, uint8_t* data, const uint16_t length);
        int32_t write_data(const uint16_t addr, uint8_t* data, const uint16_t length);
        int32_t get_interrupt_status(uint16_t* const int_config);
        int32_t set_interrupt_status(const uint16_t int_config);
        int32_t read_interrupt_pulse(ST25DV_PULSE_DURATION* const int_time);
        int32_t set_interrupt_pulse(const ST25DV_PULSE_DURATION int_time);
        int32_t read_dsfid(uint8_t* const ptr_dsfid);
        int32_t read_dsfid_rf_protection(ST25DV_LOCK_STATUS* const ptr_lock_dsfid);
        int32_t read_afi(uint8_t* const ptr_afi);
        int32_t read_afi_rf_protection(ST25DV_LOCK_STATUS* const ptr_lock_afi);
        int32_t read_i2c_protect_zone(ST25DV_I2C_PROT_ZONE* const ptr_prot_zone);
        int32_t write_i2c_protect_zone(const ST25DV_PROTECTION_ZONE zone, 
                const ST25DV_PROTECTION_CONF rw_protection);
        int32_t read_lock_ccfile(ST25DV_LOCK_CCFILE* const ptr_lock_ccfile);
        int32_t write_lock_ccfile(const ST25DV_CCFILE_BLOCK num_block_ccfile, 
                const ST25DV_LOCK_STATUS lock_ccfile);
        int32_t read_lock_cfg(ST25DV_LOCK_STATUS* const ptr_lock_cfg);
        int32_t write_lock_cfg(const ST25DV_LOCK_STATUS lock_cfg);
        int32_t present_i2c_password(const ST25DV_PASSWD passWord);
        int32_t write_i2c_password(const ST25DV_PASSWD passWord);
        int32_t read_rf_zone_security_status(const ST25DV_PROTECTION_ZONE zone,
                ST25DV_RF_PROT_ZONE* const ptr_rf_prot_zone);
        int32_t write_rf_zone_security_status(const ST25DV_PROTECTION_ZONE zone, 
        const ST25DV_RF_PROT_ZONE rf_prot_zone);
        int32_t read_end_zonex(const ST25DV_END_ZONE end_zone,
        uint8_t* const ptr_end_zone_value);
        int32_t write_end_zonex(const ST25DV_END_ZONE end_zone, const uint8_t end_zone_value);
        int32_t init_end_zone(void);
        int32_t create_user_zone(uint16_t zone_1_length, uint16_t zone_2_length,
                uint16_t zone_3_length, uint16_t zone_4_length);
        int32_t read_eh_mode(ST25DV_EH_MODE_STATUS* const ptr_eh_mode);
        int32_t write_eh_mode(const ST25DV_EH_MODE_STATUS eh_mode);
        int32_t read_rf_config(ST25DV_RF_MNGT* const ptr_rf_config);
        int32_t write_rf_config(const uint8_t rf_config);
        int32_t read_rf_disable(ST25DV_EN_STATUS* const ptr_rf_disable);
        int32_t write_rf_disable(void);
        int32_t reset_rf_disable(void);
        int32_t read_rf_sleep(ST25DV_EN_STATUS* const ptr_rf_sleep);
        int32_t write_rf_sleep(void);
        int32_t reset_rf_sleep(void);
        int32_t read_mailbox_mode(ST25DV_EN_STATUS* const ptr_mb_mode);
        int32_t write_mailbox_mode(const ST25DV_EN_STATUS mb_mode);
        int32_t read_mailbox_wd_duration_coef(uint8_t* const ptr_wdg_delay);
        int32_t write_mailbox_wd_duration_coef(const uint8_t wdg_delay);
        int32_t read_mailbox_data(uint8_t* const ptr_data, const uint16_t offset,
                const uint16_t num_byte);
        int32_t write_mailbox_data(const uint8_t* const ptr_data,  const uint16_t num_byte);
        int32_t read_mailbox_register(uint8_t* const ptr_data, const uint16_t reg_addr,
                const uint16_t num_byte);
        int32_t write_mailbox_register(const uint8_t* const ptr_data,
                const uint16_t reg_addr, const uint16_t num_byte);
        int32_t read_i2c_security_session_dyn(ST25DV_I2CSSO_STATUS* const ptr_session);
        int32_t read_it_status_dyn(uint8_t* const ptr_it_status);
        int32_t read_gpo_dyn(uint8_t* ptr_gpo_config);
        int32_t get_gpo_en_dyn(ST25DV_EN_STATUS* const ptr_gpo_en);
        int32_t set_gpo_en_dyn(void);
        int32_t reset_gpo_en_dyn(void);
        int32_t read_eh_ctrl_dyn(ST25DV_EH_CTRL* const ptr_eh_ctrl);
        int32_t get_eh_en_mode_dyn(ST25DV_EN_STATUS* const ptr_eh_val);
        int32_t set_eh_en_mode_dyn(void);
        int32_t reset_eh_en_mode_dyn(void);
        int32_t get_eh_on_dyn(ST25DV_EN_STATUS * const ptr_eh_on);
        int32_t get_rf_field_dyn(ST25DV_FIELD_STATUS * const ptr_rf_field);
        int32_t get_vcc_dyn(ST25DV_VCC_STATUS* const ptr_vcc);
        int32_t read_rf_mngt_cfg_dyn(ST25DV_RF_MNGT* const ptr_rf_mngt);
        int32_t write_rf_mngt_cfg_Dyn(const uint8_t rf_mngt);
        int32_t get_rf_disable_dyn(ST25DV_EN_STATUS* const ptr_rf_disable);
        int32_t set_rf_disable_dyn(void);
        int32_t reset_rf_disable_dyn(void);
        int32_t get_rf_sleep_dyn(ST25DV_EN_STATUS* const ptr_rf_sleep);
        int32_t set_rf_sleep_dyn(void);
        int32_t reset_rf_sleep_dyn(void);
        int32_t read_mailbox_ctrl_dyn(ST25DV_MB_CTRL_DYN_STATUS* const ptr_ctrl_status);
        int32_t get_mailbox_en_dyn(ST25DV_EN_STATUS* const ptr_mb_en);
        int32_t set_mailbox_en_dyn(void);
        int32_t reset_mailbox_en_dyn(void);
        int32_t read_mailbox_length_dyn(uint8_t* const ptr_mb_length);

    private:
        int8_t _gpio_eh, _gpio_gpo, _gpio_lpd;
        ST25DV_Object_t _std25dv_element;
        NFCTAG_DrvTypeDef* _nfc_tag_drv;
        uint8_t _lpd_signal;
};

/*************************************************************************************************/

#ifdef __cplusplus
}
#endif  // extern "C"

#endif // NFCST25DV_H_
