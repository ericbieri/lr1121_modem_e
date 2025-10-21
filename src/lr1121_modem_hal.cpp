#include <Arduino.h>
#include "lr1121_modem_hal.h"
#include "lr1121_modem_hal_context.h"
#include "lr1121_modem_modem_types.h"
#include "lr1121_modem_modem.h"

#define LR1121_MODEM_RESET_TIMEOUT 3000

// Helper function to wait for busy == HIGH  
static lr1121_modem_hal_status_t lr1121_modem_hal_wait_on_busy( const void* context, uint32_t timeout_ms )
{
    const lr1121_modem_hal_context_t* ctx = (const lr1121_modem_hal_context_t*)context;
    uint32_t timeout = millis() + timeout_ms;
    while(digitalRead(ctx->busy_pin) == LOW) {
        delayMicroseconds(10);
        // Check if timeout occurred
        if (millis() >= timeout) {
            return LR1121_MODEM_HAL_STATUS_ERROR;
        }
    }   
    return LR1121_MODEM_HAL_STATUS_OK;
}

// Helper function to wait for busy == LOW  
static lr1121_modem_hal_status_t lr1121_modem_hal_wait_on_unbusy( const void* context, uint32_t timeout_ms )
{
    const lr1121_modem_hal_context_t* ctx = (const lr1121_modem_hal_context_t*)context;
    uint32_t timeout = millis() + timeout_ms;

    while(digitalRead(ctx->busy_pin) == HIGH) {
        delayMicroseconds(10);
        // Check if timeout occurred
        if (millis() >= timeout) {
            return LR1121_MODEM_HAL_STATUS_ERROR;
        }
    }   
    return LR1121_MODEM_HAL_STATUS_OK;
}

/*!
 * Wake the radio up.
 *
 * @param [in] context Radio implementation parameters
 * @returns Operation status
 */
lr1121_modem_hal_status_t lr1121_modem_hal_wakeup(const void* context) {

    const lr1121_modem_hal_context_t* ctx = (const lr1121_modem_hal_context_t*)context;
    
    // if busy = HIGH
    if (lr1121_modem_hal_wait_on_busy( context, 10000 ) == LR1121_MODEM_HAL_STATUS_OK) {
        // Wakeup radio by toggling CS pin
        digitalWrite(ctx->cs_pin, LOW);
        delayMicroseconds(100); // Ensure CS is low for at least 100us 
        digitalWrite(ctx->cs_pin, HIGH);
    } else {
        return LR1121_MODEM_HAL_STATUS_BUSY_TIMEOUT;
    }
    
    // Wait for busy = LOW
    return lr1121_modem_hal_wait_on_unbusy( context, 1000 );
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

        // Send command
        for (uint16_t i = 0; i < command_length; i++) {
            ctx->spi->transfer(command[i]);
        }
        // Send data
        for (uint16_t i = 0; i < data_length; i++) {
            ctx->spi->transfer(data[i]);
        }

        // Compute and send CRC
        crc = lr1121_modem_compute_crc(0xFF, command, command_length);
        crc = lr1121_modem_compute_crc(crc, data, data_length);

        // Send CRC
        ctx->spi->transfer(crc);

        // Deselect chip
        digitalWrite(ctx->cs_pin, HIGH);

        // Wait for busy = HIGH up to 1000 ms
        if (lr1121_modem_hal_wait_on_busy(context, 1000) != LR1121_MODEM_HAL_STATUS_OK) {
            return LR1121_MODEM_HAL_STATUS_BUSY_TIMEOUT;
        }

        // Send dummy byte to retrieve return code & CRC

        // Select chip
        digitalWrite(ctx->cs_pin, LOW);

        // Send dummy bytes
        status = (lr1121_modem_hal_status_t) ctx->spi->transfer(0x00);
        crc_received = (lr1121_modem_hal_status_t) ctx->spi->transfer(0x00);
        
        // Compute response CRC
        crc = lr1121_modem_compute_crc(0xFF, (uint8_t*)&status, 1);
        
        // Deselect chip
        digitalWrite(ctx->cs_pin, HIGH);

        if (crc != crc_received) {
            // Change the response code
            status = LR1121_MODEM_HAL_STATUS_BAD_FRAME;
        }

        // Wait for busy = LOW up to 1000 ms
        if (lr1121_modem_hal_wait_on_unbusy(context, 1000) != LR1121_MODEM_HAL_STATUS_OK) {
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

        // Send command
        for (uint16_t i = 0; i < command_length; i++) {
            ctx->spi->transfer(command[i]);
        }

        // Send data
        for (uint16_t i = 0; i < data_length; i++) {
            ctx->spi->transfer(data[i]);
        }
        // Compute and send CRC
        crc = lr1121_modem_compute_crc(0xFF, command, command_length);
        crc = lr1121_modem_compute_crc(crc, data, data_length);
        // Send CRC
        ctx->spi->transfer(crc);

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
lr1121_modem_hal_status_t lr1121_modem_hal_read( const void* context, const uint8_t* command,
                                                 const uint16_t command_length, uint8_t* data,
                                                 const uint16_t data_length ) {

    const lr1121_modem_hal_context_t* ctx = (const lr1121_modem_hal_context_t*)context;
    
    if (lr1121_modem_hal_wakeup(context) == LR1121_MODEM_HAL_STATUS_OK) {   
        uint8_t crc = 0;
        uint8_t crc_received = 0;
        lr1121_modem_hal_status_t status;

        // Select chip
        digitalWrite(ctx->cs_pin, LOW);

        // Send command
        for (uint16_t i = 0; i < command_length; i++) {
            ctx->spi->transfer(command[i]);
        }

        // Compute and send CRC
        crc = lr1121_modem_compute_crc(0xFF, command, command_length);
        ctx->spi->transfer(crc);

        // Deselect chip
        digitalWrite(ctx->cs_pin, HIGH);
       
        // Wait for busy = HIGH up to 1000 ms
        if (lr1121_modem_hal_wait_on_busy(context, 1000) != LR1121_MODEM_HAL_STATUS_OK) {
            return LR1121_MODEM_HAL_STATUS_BUSY_TIMEOUT;
        }
        
        // Send dummy byte to retrieve RC & CRC
        // Select chip
        digitalWrite(ctx->cs_pin, LOW);

        // Read response code
        status = (lr1121_modem_hal_status_t) ctx->spi->transfer(0);
        if (status == LR1121_MODEM_HAL_STATUS_OK) {
            for (uint16_t i = 0; i < data_length; i++) {
                data[i] = ctx->spi->transfer(0);
            }
        }
        crc_received = ctx->spi->transfer(0);

        // Deselect chip
        digitalWrite(ctx->cs_pin, HIGH);

        // Wait for busy = LOW up to 1000 ms
        if (lr1121_modem_hal_wait_on_unbusy(context, 1000) != LR1121_MODEM_HAL_STATUS_OK) {
            return LR1121_MODEM_HAL_STATUS_BUSY_TIMEOUT;
        }

        // Compute response CRC
        crc = lr1121_modem_compute_crc(0xFF, (uint8_t*)&status, 1);
        if (status == LR1121_MODEM_HAL_STATUS_OK) {
            crc = lr1121_modem_compute_crc(crc, data, data_length);
        }

        if (crc != crc_received) {
            // Change the response code
            status = LR1121_MODEM_HAL_STATUS_BAD_FRAME;
        }
        return status;
    }

    return LR1121_MODEM_HAL_STATUS_BUSY_TIMEOUT;
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
    digitalWrite(ctx->reset_pin, LOW);
    delay(1);
    digitalWrite(ctx->reset_pin, HIGH);

    // wait 250ms
    delay(250);

    // Reinitialize busy pin to input
    pinMode(ctx->busy_pin, INPUT);
}
