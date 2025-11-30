#include <Arduino.h>
#include <SPI.h>
#include "lr1121_hal.h"
#include "lr1121_modem_hal.h"
#include "lr1121_modem_hal_context.h"
#include "lr1121_modem_modem_types.h"
#include "lr1121_modem_modem.h"

#define LR1121_HAL_WAIT_ON_BUSY_TIMEOUT_MS 5000
#define LR1121_HAL_WAIT_ON_BUSY_DELAY_US 10
#define LR1121_MODEM_WAKEUP_PULSE_DURATION_US 100
#define LR1121_MODEM_RESET_TIMEOUT_MS 10000
#define LR1121_MODEM_RESET_PULSE_DURATION_US 1000

// TODO Refactor modem methods to use hal_reset, wakeup etc. 
/*!
 * Helper function to write data over SPI
 *
 * @param [in] data Pointer to the data buffer
 * @param [in] data_length Length of the data to be sent
 */
void spi_write_data(const uint8_t *data, const uint16_t data_length) {
    for (uint16_t i = 0; i < data_length; i++) {
        SPI.transfer(data[i]);
    }
}

/*!
 * Helper function to read data over SPI with dummy bytes
 *
 * @param [out] data Pointer to the data buffer
 * @param [in] data_length Length of the data to be read
 * @param [in] dummy_byte Dummy byte to be sent while reading
 */
void spi_read_data_with_dummy_byte(uint8_t *data, const uint16_t data_length, const uint8_t dummy_byte) {
    for (uint16_t i = 0; i < data_length; i++) {
        data[i] = SPI.transfer(dummy_byte);
    }
}

/*!
 * Helper function to wait for busy line to reach expected state within timeout.
 *
 * @param [in] context Radio implementation parameters
 * @param [in] timeout_ms Timeout in milliseconds
 * @param [in] expected_state Expected state of the busy pin (HIGH or LOW)
 * 
 * @returns Operation status
 */
static lr1121_hal_status_t lr1121_hal_wait_on_busy(const void* context, uint32_t timeout_ms, bool expected_state) {
    const lr1121_modem_hal_context_t* ctx = (const lr1121_modem_hal_context_t*)context;

    uint32_t timeout = millis() + timeout_ms;
    while(digitalRead(ctx->busy_pin) != expected_state) {
        delayMicroseconds(LR1121_HAL_WAIT_ON_BUSY_DELAY_US);
        // Check if timeout occurred
        if (millis() >= timeout) {
            return LR1121_HAL_STATUS_ERROR;
        }
    }   
    return LR1121_HAL_STATUS_OK;
}   

/*!
 * Helper function to wait for busy line to reach expected state within timeout.
 *
 * @param [in] context Radio implementation parameters
 * @param [in] timeout_ms Timeout in milliseconds
 * @param [in] expected_state Expected state of the busy pin (HIGH or LOW)
 * 
 * @returns Operation status
 */
static lr1121_modem_hal_status_t lr1121_modem_hal_wait_on_busy(const void* context, uint32_t timeout_ms, bool expected_state) {

    lr1121_hal_status_t status = lr1121_hal_wait_on_busy(context, timeout_ms, expected_state);

    if (status == LR1121_HAL_STATUS_OK) {
        return LR1121_MODEM_HAL_STATUS_OK;
    } else {
        return LR1121_MODEM_HAL_STATUS_ERROR;
    }
}

/*!
 * Wake the radio up.
 *
 * @param [in] context Radio implementation parameters
 * @returns Operation status
 */
lr1121_modem_hal_status_t lr1121_modem_hal_wakeup(const void* context) {

    const lr1121_modem_hal_context_t* ctx = (const lr1121_modem_hal_context_t*)context;
    
    // Wait untill busy = HIGH to wake up the chip
    if ((lr1121_modem_hal_wait_on_busy(context, LR1121_HAL_WAIT_ON_BUSY_TIMEOUT_MS, HIGH) == LR1121_MODEM_HAL_STATUS_OK) &&
        (lr1121_hal_wakeup(context) == LR1121_HAL_STATUS_OK)) {
        return LR1121_MODEM_HAL_STATUS_OK;
    } // TODO error handling
    return LR1121_MODEM_HAL_STATUS_BUSY_TIMEOUT;
}

/*!
 * Radio data transfer - write
 *
 * @param [in] context          Radio implementation parameters
 * @param [in] command          Pointer to the buffer to be transmitted
 * @param [in] command_length   Buffer size to be transmitted
 * @param [in] data             Pointer to the buffer to be transmitted
 * @param [in] data_length      Buffer size to be transmitted
 *
 * @returns Operation status
 */
lr1121_modem_hal_status_t lr1121_modem_hal_write( const void* context, const uint8_t* command,
                                                  const uint16_t command_length, const uint8_t* data,
                                                  const uint16_t data_length ) {

    const lr1121_modem_hal_context_t* ctx = (const lr1121_modem_hal_context_t*)context;   

    if (lr1121_modem_hal_wakeup(context) == LR1121_MODEM_HAL_STATUS_OK) {

        uint8_t crc = 0;
        uint8_t crc_received = 0;

        lr1121_modem_hal_status_t status;

        // Select chip
        digitalWrite(ctx->cs_pin, LOW);

        // Compute CRC
        crc = lr1121_modem_compute_crc(0xFF, command, command_length);
        crc = lr1121_modem_compute_crc(crc, data, data_length);

        // Send command, data & CRC
        spi_write_data(command, command_length);
        spi_write_data(data, data_length);
        spi_write_data(&crc, 1);

        // Deselect chip
        digitalWrite(ctx->cs_pin, HIGH);

        // Wait for busy = HIGH
        if (lr1121_modem_hal_wait_on_busy(context, LR1121_HAL_WAIT_ON_BUSY_TIMEOUT_MS, HIGH) != LR1121_MODEM_HAL_STATUS_OK) {
            return LR1121_MODEM_HAL_STATUS_BUSY_TIMEOUT;
        }

        // Send dummy byte to retrieve return code & CRC

        // Select chip
        digitalWrite(ctx->cs_pin, LOW);

        // Send dummy bytes
        spi_read_data_with_dummy_byte((uint8_t*)&status, 1, 0x00);
        spi_read_data_with_dummy_byte(&crc_received, 1, 0x00);
    
        // Deselect chip
        digitalWrite(ctx->cs_pin, HIGH);

        // Compute response CRC
        crc = lr1121_modem_compute_crc(0xFF, (uint8_t*)&status, 1);

        if (crc != crc_received) {
            return LR1121_MODEM_HAL_STATUS_BAD_FRAME;
        }

        // Wait for busy = LOW
        if (lr1121_modem_hal_wait_on_busy(context, LR1121_HAL_WAIT_ON_BUSY_TIMEOUT_MS, LOW) != LR1121_MODEM_HAL_STATUS_OK) {
            return LR1121_MODEM_HAL_STATUS_BUSY_TIMEOUT;
        }
        return status;
    }

    return LR1121_MODEM_HAL_STATUS_BUSY_TIMEOUT;
}

/*!
 * Radio data transfer - write without wait the return code - this API is dedicated to the functions which reset the
 * Modem-E
 *
 * @param [in] context          Radio implementation parameters
 * @param [in] command          Pointer to the buffer to be transmitted
 * @param [in] command_length   Buffer size to be transmitted
 * @param [in] data             Pointer to the buffer to be transmitted
 * @param [in] data_length      Buffer size to be transmitted
 *
 * @returns Operation status
 */
lr1121_modem_hal_status_t lr1121_modem_hal_write_without_rc( const void* context, const uint8_t* command,
                                                             const uint16_t command_length, const uint8_t* data,
                                                             const uint16_t data_length ) {

    const lr1121_modem_hal_context_t* ctx = (const lr1121_modem_hal_context_t*)context;

    if (lr1121_modem_hal_wakeup(context) == LR1121_MODEM_HAL_STATUS_OK) {
        uint8_t crc = 0;
        lr1121_modem_hal_status_t status = LR1121_MODEM_HAL_STATUS_OK;

        // Select chip
        digitalWrite(ctx->cs_pin, LOW);

        // Compute CRC
        crc = lr1121_modem_compute_crc(0xFF, command, command_length);
        crc = lr1121_modem_compute_crc(crc, data, data_length);

        // Send command, data & CRC
        spi_write_data(command, command_length);
        spi_write_data(data, data_length);
        spi_write_data(&crc, 1);

        // Deselect chip
        digitalWrite(ctx->cs_pin, HIGH);

        return status;
    }

    return LR1121_MODEM_HAL_STATUS_BUSY_TIMEOUT;
}

/*!
 * Radio data transfer - read
 *
 * @param [in] context          Radio implementation parameters
 * @param [in] command          Pointer to the buffer to be transmitted
 * @param [in] command_length   Buffer size to be transmitted
 * @param [out] data            Pointer to the buffer to be received
 * @param [in] data_length      Buffer size to be received
 *
 * @returns Operation status
 */
lr1121_modem_hal_status_t lr1121_modem_hal_read(const void* context, const uint8_t* command,
                                                const uint16_t command_length, uint8_t* data,
                                                const uint16_t data_length) {

    const lr1121_modem_hal_context_t* ctx = (const lr1121_modem_hal_context_t*)context;
    
    if (lr1121_modem_hal_wakeup(context) == LR1121_MODEM_HAL_STATUS_OK) {   
        uint8_t crc = 0;
        uint8_t crc_received = 0;
        lr1121_modem_hal_status_t status;
        
        // Select chip
        digitalWrite(ctx->cs_pin, LOW);

        // Compute CRC
        crc = lr1121_modem_compute_crc(0xFF, command, command_length);

        // Send command & CRC
        spi_write_data(command, command_length);
        spi_write_data(&crc, 1);

        // Deselect chip
        digitalWrite(ctx->cs_pin, HIGH);
       
        // Wait for busy = HIGH
        if (lr1121_modem_hal_wait_on_busy(context, LR1121_HAL_WAIT_ON_BUSY_TIMEOUT_MS, HIGH) != LR1121_MODEM_HAL_STATUS_OK) {
            return LR1121_MODEM_HAL_STATUS_BUSY_TIMEOUT;
        }
        
        // Send dummy byte to retrieve RC & CRC
        // Select chip
        digitalWrite(ctx->cs_pin, LOW);

        // Read response code
        spi_read_data_with_dummy_byte((uint8_t*)&status, 1, 0x00);
        if (status == LR1121_MODEM_HAL_STATUS_OK) {
            spi_read_data_with_dummy_byte(data, data_length, 0x00);
        } else {
            return LR1121_MODEM_HAL_STATUS_ERROR;
        }
        // Read CRC
        spi_read_data_with_dummy_byte((uint8_t*)&crc_received, 1, 0x00);

        // Deselect chip
        digitalWrite(ctx->cs_pin, HIGH);

        // Compute response CRC
        crc = lr1121_modem_compute_crc(0xFF, (uint8_t*)&status, 1);
        if (status == LR1121_MODEM_HAL_STATUS_OK) {
            crc = lr1121_modem_compute_crc(crc, data, data_length);
        } else {
            return LR1121_MODEM_HAL_STATUS_ERROR;
        }

        // Compare CRCs
        if (crc != crc_received) {
            return LR1121_MODEM_HAL_STATUS_BAD_FRAME;
        }

        // Wait for busy = LOW
        if (lr1121_modem_hal_wait_on_busy(context, LR1121_HAL_WAIT_ON_BUSY_TIMEOUT_MS, LOW) != LR1121_MODEM_HAL_STATUS_OK) {
            return LR1121_MODEM_HAL_STATUS_BUSY_TIMEOUT;
        }

        return status;
    }

    return LR1121_MODEM_HAL_STATUS_BUSY_TIMEOUT;
}

/*!
 * Reset the radio
 *
 * @remark Must be implemented by the upper layer
 *
 * @param [in] context Radio implementation parameters
 *
 * @returns Operation status
 */
lr1121_modem_hal_status_t lr1121_modem_hal_reset( const void* context ) {

    uint32_t timeout = millis() + LR1121_MODEM_RESET_TIMEOUT_MS;

    // Reset the chip
    lr1121_hal_reset(context);
    // TODO delay?

    while (millis() < timeout) {
        // Wait for the reset event
        lr1121_modem_event_fields_t* event_fields;
        lr1121_modem_response_code_t rc = lr1121_modem_get_event(context, event_fields);

        if (rc == LR1121_MODEM_RESPONSE_CODE_OK && event_fields->event_type == LR1121_MODEM_LORAWAN_EVENT_RESET) {
            Serial.println("Reset event received");
            return LR1121_MODEM_HAL_STATUS_OK;
        }
        // TODO delayMicroseconds?
    }

    return LR1121_MODEM_HAL_STATUS_ERROR;
}

/*!
 * Switch the radio in DFU (Device Firmware Update) mode
 *
 * @param [in] context Radio implementation parameters
 */
void lr1121_modem_hal_enter_dfu( const void* context ) {

    const lr1121_modem_hal_context_t* ctx = (const lr1121_modem_hal_context_t*)context;

    // set busy pin mode to output and force busy pin to low
    pinMode(ctx->busy_pin, OUTPUT);
    digitalWrite(ctx->busy_pin, LOW);

    // reset the chip
    lr1121_hal_reset(context);

    // wait > 100ms
    delay(200);

    // Reinitialize busy pin to input
    pinMode(ctx->busy_pin, INPUT);
}

// ------ Methods implementing lr1121_hal.h API ------

/*!
 * Radio data transfer - write
 *
 * @remark Must be implemented by the upper layer
 *
 * @param [in] context          Radio implementation parameters
 * @param [in] command          Pointer to the buffer to be transmitted
 * @param [in] command_length   Buffer size to be transmitted
 * @param [in] data             Pointer to the buffer to be transmitted
 * @param [in] data_length      Buffer size to be transmitted
 *
 * @returns Operation status
 */
lr1121_hal_status_t lr1121_hal_write(const void* context, const uint8_t* command, const uint16_t command_length,
                                      const uint8_t* data, const uint16_t data_length) {

  const lr1121_modem_hal_context_t* ctx = (const lr1121_modem_hal_context_t*)context;
  
  if (lr1121_hal_wakeup(context) == LR1121_HAL_STATUS_OK) {
    // Select chip
    digitalWrite(ctx->cs_pin, LOW);

    // Send command & data
    spi_write_data(command, command_length);
    spi_write_data(data, data_length);

    // Deselect chip
    digitalWrite(ctx->cs_pin, HIGH);

    return lr1121_hal_wait_on_busy(context, LR1121_HAL_WAIT_ON_BUSY_TIMEOUT_MS, LOW);
  }
  return LR1121_HAL_STATUS_ERROR;
}

/*!
 * Radio data transfer - read
 *
 * @remark Must be implemented by the upper layer
 *
 * @param [in] context          Radio implementation parameters
 * @param [in] command          Pointer to the buffer to be transmitted
 * @param [in] command_length   Buffer size to be transmitted
 * @param [out] data            Pointer to the buffer to be received
 * @param [in] data_length      Buffer size to be received
 *
 * @returns Operation status
 */
lr1121_hal_status_t lr1121_hal_read(const void* context, const uint8_t* command, const uint16_t command_length,
                                     uint8_t* data, const uint16_t data_length) {
  const lr1121_modem_hal_context_t* ctx = (const lr1121_modem_hal_context_t*)context;
  
  if (lr1121_hal_wakeup(context) == LR1121_HAL_STATUS_OK) {

    // Select chip
    digitalWrite(ctx->cs_pin, LOW);

    // Send command
    spi_write_data(command, command_length);

    // Deselect chip
    digitalWrite(ctx->cs_pin, HIGH);

    if (lr1121_hal_wait_on_busy(context, LR1121_HAL_WAIT_ON_BUSY_TIMEOUT_MS, LOW) != LR1121_HAL_STATUS_OK) {
        return LR1121_HAL_STATUS_ERROR;
    }
    
    // Select chip
    digitalWrite(ctx->cs_pin, LOW);
    
    // Send dummy byte to read data
    spi_write_data(0x00, 1);
    spi_read_data_with_dummy_byte(data, data_length, 0x00);

    // Deselect chip
    digitalWrite(ctx->cs_pin, HIGH);

    return lr1121_hal_wait_on_busy(context, LR1121_HAL_WAIT_ON_BUSY_TIMEOUT_MS, LOW);
  }
  return LR1121_HAL_STATUS_ERROR;
}

/*!
 * @brief  Direct read from the SPI bus
 *
 * @remark Unlike @ref lr1121_hal_read, this is a simple direct SPI bus SS/read/nSS operation. While reading the
 * response data, the implementation of this function must ensure that only zero bytes (NOP) are written to the SPI bus.
 *
 * @remark Formerly, that function depended on a lr1121_hal_write_read API function, which required bidirectional SPI
 * communication. Given that all other radio functionality can be implemented with unidirectional SPI, it has been
 * decided to make this HAL API change to simplify implementation requirements.
 *
 * @remark Only required by the @ref lr1121_bootloader_get_status command
 *
 * @param [in]  context      Radio implementation parameters
 * @param [out] data         Pointer to the buffer to be received
 * @param [in]  data_length  Buffer size to be received
 *
 * @returns Operation status
 */
lr1121_hal_status_t lr1121_hal_direct_read(const void* context, uint8_t* data, const uint16_t data_length) {
  
  // TODO: Implement direct read functionality for bootloader.
  return LR1121_HAL_STATUS_ERROR;
}

/*!
 * @brief Reset the radio
 *
 * @remark Must be implemented by the upper layer
 *
 * @param [in] context Radio implementation parameters
 *
 * @returns Operation status
 */
lr1121_hal_status_t lr1121_hal_reset(const void* context) {

  const lr1121_modem_hal_context_t* ctx = (const lr1121_modem_hal_context_t*)context;
  
  // Reset the radio by toggling the reset pin low
  digitalWrite(ctx->reset_pin, LOW);
  delayMicroseconds(LR1121_MODEM_RESET_PULSE_DURATION_US);  
  digitalWrite(ctx->reset_pin, HIGH);
  
  return LR1121_HAL_STATUS_OK;
}

/*!
 * @brief Wake the radio up.
 *
 * @remark Must be implemented by the upper layer
 *
 * @param [in] context Radio implementation parameters
 *
 * @returns Operation status
 */
lr1121_hal_status_t lr1121_hal_wakeup(const void* context) {
  const lr1121_modem_hal_context_t* ctx = (const lr1121_modem_hal_context_t*)context;
  
  // Wakeup radio by toggling CS pin
  digitalWrite(ctx->cs_pin, LOW);
  delayMicroseconds(LR1121_MODEM_WAKEUP_PULSE_DURATION_US); // Ensure CS is low for at least 100us 
  digitalWrite(ctx->cs_pin, HIGH);

  return lr1121_hal_wait_on_busy(context, LR1121_HAL_WAIT_ON_BUSY_TIMEOUT_MS, LOW);
}

