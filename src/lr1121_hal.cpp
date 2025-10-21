#include "lr1121_hal.h"
#include "lr1121_modem_hal_context.h"
#include <Arduino.h>

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
  
  // Assert CS pin low
  digitalWrite(ctx->cs_pin, LOW);
  
  // Send command bytes
  for (uint16_t i = 0; i < command_length; i++) {
    ctx->spi->transfer(command[i]);
  }
  
  // Send data bytes
  for (uint16_t i = 0; i < data_length; i++) {
    ctx->spi->transfer(data[i]);
  }
  
  // Deassert CS pin high
  digitalWrite(ctx->cs_pin, HIGH);
  
  return LR1121_HAL_STATUS_OK;
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
  
  // Assert CS pin low
  digitalWrite(ctx->cs_pin, LOW);
  
  // Send command bytes
  for (uint16_t i = 0; i < command_length; i++) {
    ctx->spi->transfer(command[i]);
  }
  
  // Read data bytes
  for (uint16_t i = 0; i < data_length; i++) {
    data[i] = ctx->spi->transfer(0x00);  // Send dummy byte and read response
  }
  
  // Deassert CS pin high
  digitalWrite(ctx->cs_pin, HIGH);
  
  return LR1121_HAL_STATUS_OK;
}

/*!
 * @brief  Radio data transfer - write & read in single operation
 *
 * @remark Must be implemented by the upper layer
 * @remark Only required by lr1121_system_get_status command
 *
 * @param [in] context          Radio implementation parameters
 * @param [in] command          Pointer to the buffer to be transmitted
 * @param [out] data            Pointer to the buffer to be received
 * @param [in] data_length      Buffer size to be received
 *
 * @returns Operation status
 */
lr1121_hal_status_t lr1121_hal_write_read(const void* context, const uint8_t* command, uint8_t* data,
                                           const uint16_t data_length) {
  const lr1121_modem_hal_context_t* ctx = (const lr1121_modem_hal_context_t*)context;
  
  // Assert CS pin low
  digitalWrite(ctx->cs_pin, LOW);
  
  // Send command byte and simultaneously read response
  for (uint16_t i = 0; i < data_length; i++) {
    data[i] = ctx->spi->transfer(command[i]);
  }
  
  // Deassert CS pin high
  digitalWrite(ctx->cs_pin, HIGH);
  
  return LR1121_HAL_STATUS_OK;
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
  const lr1121_modem_hal_context_t* ctx = (const lr1121_modem_hal_context_t*)context;
  
  // Assert CS pin low
  digitalWrite(ctx->cs_pin, LOW);
  
  // Read data bytes by sending NOP (0x00)
  for (uint16_t i = 0; i < data_length; i++) {
    data[i] = ctx->spi->transfer(0x00);
  }
  
  // Deassert CS pin high
  digitalWrite(ctx->cs_pin, HIGH);
  
  return LR1121_HAL_STATUS_OK;
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
  
  // Configure reset pin as output
  pinMode(ctx->reset_pin, OUTPUT);
  
  // Pull reset pin low
  digitalWrite(ctx->reset_pin, LOW);
  delay(10);  // Hold reset for 10ms
  
  // Release reset pin (high)
  digitalWrite(ctx->reset_pin, HIGH);
  delay(50);  // Wait for radio to boot up
  
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
  
  // Assert CS pin low to wake up the radio
  digitalWrite(ctx->cs_pin, LOW);
  
  // Wait for radio to wake up - check busy pin
  uint32_t timeout = millis() + 1000;  // 1 second timeout
  pinMode(ctx->busy_pin, INPUT);
  
  while (digitalRead(ctx->busy_pin) == HIGH && millis() < timeout) {
    delayMicroseconds(100);
  }
  
  // Deassert CS pin high
  digitalWrite(ctx->cs_pin, HIGH);
  
  // Check if timeout occurred
  if (millis() >= timeout) {
    return LR1121_HAL_STATUS_ERROR;
  }
  
  return LR1121_HAL_STATUS_OK;
}
