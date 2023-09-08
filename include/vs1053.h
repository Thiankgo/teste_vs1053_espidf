/**
 * This is a driver library for VS1053 MP3 Codec Breakout
 * (Ogg Vorbis / MP3 / AAC / WMA / FLAC / MIDI Audio Codec Chip).
 * Adapted for Espressif ESP32 boards.
 *
 * version 1.0.1
 *
 * Licensed under GNU GPLv3 <http://gplv3.fsf.org/>
 * Copyright 2018
 *
 * @authors baldram, edzelf, MagicCube, maniacbug
 * @authors nopnop2002
 *
 * Development log:
 *  - 2011: initial VS1053 Arduino library
 *          originally written by J. Coliz (github: @maniacbug),
 *  - 2016: refactored and integrated into Esp-radio sketch
 *          by Ed Smallenburg (github: @edzelf)
 *  - 2017: refactored to use as PlatformIO library
 *          by Marcin Szalomski (github: @baldram | twitter: @baldram)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License or later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef MAIN_VS1053_H_
#define MAIN_VS1053_H_

#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <math.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG "VS1053"
#define _DEBUG_ 0

// SCI Register
#define VS_WRITE_COMMAND 0x02
#define VS_READ_COMMAND 0x03
#define SCI_MODE 0x00
#define SCI_STATUS 0x01
#define SCI_BASS 0x02
#define SCI_CLOCKF 0x03
#define SCI_DECODE_TIME 0x04
#define SCI_AUDATA 0x05
#define SCI_WRAM 0x06
#define SCI_WRAMADDR 0x07
#define SCI_HDAT0 0x08
#define SCI_HDAT1 0x09
#define SCI_AIADDR 0x0a
#define SCI_VOL 0x0b
#define SCI_AICTRL0 0x0c
#define SCI_AICTRL1 0x0d
#define SCI_AICTRL2 0x0e
#define SCI_AICTRL3 0x0f
#define SCI_num_registers 0x0f

// SCI_MODE bits
#define SM_SDINEW 11  // Bitnumber in SCI_MODE always on
#define SM_RESET 2    // Bitnumber in SCI_MODE soft reset
#define SM_CANCEL 3   // Bitnumber in SCI_MODE cancel song
#define SM_TESTS 5    // Bitnumber in SCI_MODE for tests
#define SM_LINE1 14   // Bitnumber in SCI_MODE for Line input

#define LOW 0
#define HIGH 1
#define VS1053_CHUNK_SIZE 32
#define _BV(bit) (1 << (bit))

class VS1053_t {
 private:
  uint8_t sclk_pin;
  uint8_t mosi_pin;
  uint8_t miso_pin;
  uint8_t cs_pin;
  uint8_t dcs_pin;
  uint8_t dreq_pin;
  uint8_t reset_pin;
  uint8_t curvol;       // Current volume setting 0..100%
  uint8_t endFillByte;  // Byte to send when stopping song
  uint8_t chipVersion;  // Version of hardware
  spi_device_handle_t SPIHandleLow;
  spi_device_handle_t SPIHandleFast;

  // Private
  void delay(int ms);
  void await_data_request(void);
  bool current_data_request(void);
  void control_mode_on(void);
  void control_mode_off(void);
  void data_mode_on(void);
  void data_mode_off(void);
  uint16_t read_register( uint8_t _reg);
  bool write_register(uint8_t _reg, uint16_t _value);
  bool sdi_send_buffer(uint8_t *data, size_t len);
  bool sdi_send_fillers( size_t length);
  void wram_write( uint16_t address, uint16_t data);
  uint16_t wram_read( uint16_t address);

 public:
  // public
  VS1053_t(uint8_t _sclk_pin, uint8_t _mosi_pin, uint8_t _miso_pin, uint8_t _cs_pin, uint8_t _dcs_pin, uint8_t _dreq_pin, uint8_t _reset_pin);
	void begin(void);
  void startSong(void);                             // Prepare to start playing. Call this each
                                                             // time a new song starts.
  void playChunk( uint8_t *data, size_t len);  // Play a chunk of data.  Copies the data to
                                                             // the chip.  Blocks until complete.
  void stopSong(void);                              // Finish playing a song. Call this after
                                                             // the last playChunk call.
  void setVolume( uint8_t vol);                // Set the player volume.Level from 0-100,
                                                             // higher is louder.
  void setTone( uint8_t *rtone);               // Set the player baas/treble, 4 nibbles for
                                                             // treble gain/freq and bass gain/freq
  uint8_t getVolume(void);                          // Get the currenet volume setting.
                                                             // higher is louder.
  // void printDetails(VS1053_t * dev, char *header);            // Print configuration details to serial output.
  void softReset(void);            // Do a soft reset
  bool testComm( bool fast);  // Test communication with module
  void switchToMp3Mode(void);
  bool isChipConnected(void);
  uint16_t getDecodedTime(void);  // Provides SCI_DECODE_TIME register value

  void clearDecodedTime(void);  // Clears SCI_DECODE_TIME register (sets 0x00)
  uint8_t getHardwareVersion(void);
};

#endif /* MAIN_VS1053_H_ */
