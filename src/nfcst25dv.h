/*************************************************************************************************/
// File: nfcst25dv.h
// Description: NFC Dynamic Tags ST25DV ease library.
// Created on: 18 may. 2020
// Last modified date: 20 may. 2020
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

/* Defines */

#if !defined(NFCTAG_OK)
    #define NFCTAG_OK      (0)
#endif
#if !defined(NFCTAG_ERROR)
    #define NFCTAG_ERROR   (-1)
#endif
#if !defined(NFCTAG_BUSY)
    #define NFCTAG_BUSY    (-2)
#endif
#if !defined(NFCTAG_TIMEOUT)
    #define NFCTAG_TIMEOUT (-3)
#endif
#if !defined(NFCTAG_NACK)
    #define NFCTAG_NACK    (-102)
#endif

#define NFCTAG_4K_SIZE            ((uint32_t) 0x200)
#define NFCTAG_16K_SIZE           ((uint32_t) 0x800)
#define NFCTAG_64K_SIZE           ((uint32_t) 0x2000)

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
        uint8_t get_tag_ic_ref(void);
        int32_t get_ic_rev(uint8_t* const ic_revision);
        int32_t read_tag_uid(ST25DV_UID* const uid);
        uint32_t read_tag_size(void);
        int32_t read_data(const uint16_t addr, uint8_t* data, const uint16_t length);
        int32_t write_data(const uint16_t addr, uint8_t* data, const uint16_t length);
        int32_t get_interrupt_status(uint16_t* const int_config);
        int32_t set_interrupt_status(const uint16_t int_config);

    private:
        // Private Attributes
        int8_t _gpio_eh, _gpio_gpo, _gpio_lpd;
        ST25DV_Object_t _std25dv_element;
        NFCTAG_DrvTypeDef* _nfc_tag_drv;
        uint8_t _lpd_signal;

        // Private Methods

};

/*************************************************************************************************/

#ifdef __cplusplus
}
#endif  // extern "C"

#endif // NFCST25DV_H_
