// I2Cdev library collection - MPU6050 I2C device class
// Based on InvenSense MPU-6050 register map document rev. 2.0, 5/19/2011 (RM-MPU-6000A-00)
// 8/24/2011 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//  2021-09-27 - split implementations out of header files, finally
//  2019-07-08 - Added Auto Calibration routine
//     ... - ongoing debug release

// NOTE: THIS IS ONLY A PARIAL RELEASE. THIS DEVICE CLASS IS CURRENTLY UNDERGOING ACTIVE
// DEVELOPMENT AND IS STILL MISSING SOME IMPORTANT FEATURES. PLEASE KEEP THIS IN MIND IF
// YOU DECIDE TO USE THIS PARTICULAR CODE FOR ANYTHING.

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#include "MPU6050.hpp"

/** Specific address constructor.
 * @param address I2C address, uses default I2C address if none is specified
 * @see MPU6050_DEFAULT_ADDRESS
 * @see MPU6050_ADDRESS_AD0_LOW
 * @see MPU6050_ADDRESS_AD0_HIGH
 */
MPU6050_Base::MPU6050_Base(uint8_t address, void *wireObj):devAddr(address), wireObj(wireObj) {
}

/** Power on and prepare for general usage.
 * This will activate the device and take it out of sleep mode (which must be done
 * after start-up). This function also sets both the accelerometer and the gyroscope
 * to their most sensitive settings, namely +/- 2g and +/- 250 degrees/sec, and sets
 * the clock source to use the X Gyro for reference, which is slightly better than
 * the default internal clock source.
 */
void MPU6050_Base::initialize() {
    setClockSource(MPU6050_CLOCK_PLL_XGYRO);
    setFullScaleGyroRange(MPU6050_GYRO_FS_250);
    setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    setSleepEnabled(false); // thanks to Jack Elston for pointing this one out!
}

/** Set full-scale accelerometer range.
 * @param range New full-scale accelerometer range setting
 * @see getFullScaleAccelRange()
 */
void MPU6050_Base::setFullScaleAccelRange(uint8_t range) {
    I2Cdev::writeBits(devAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range, wireObj);
}

/** Verify the I2C connection.
 * Make sure the device is connected and responds as expected.
 * @return True if connection is valid, false otherwise
 */
bool MPU6050_Base::testConnection() {
    return getDeviceID() == 0x34;
}

/** Set sleep mode status.
 * @param enabled New sleep mode enabled status
 * @see getSleepEnabled()
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_SLEEP_BIT
 */
void MPU6050_Base::setSleepEnabled(bool enabled) {
    I2Cdev::writeBit(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled, wireObj);
}

// BANK_SEL register
void MPU6050_Base::setMemoryBank(uint8_t bank, bool prefetchEnabled, bool userBank) {
    bank &= 0x1F;
    if (userBank) bank |= 0x20;
    if (prefetchEnabled) bank |= 0x40;
    I2Cdev::writeByte(devAddr, MPU6050_RA_BANK_SEL, bank, wireObj);
}

// MEM_START_ADDR register
void MPU6050_Base::setMemoryStartAddress(uint8_t address) {
    I2Cdev::writeByte(devAddr, MPU6050_RA_MEM_START_ADDR, address, wireObj);
}

// MEM_R_W register
uint8_t MPU6050_Base::readMemoryByte() {
    I2Cdev::readByte(devAddr, MPU6050_RA_MEM_R_W, buffer, I2Cdev::readTimeout, wireObj);
    return buffer[0];
}

/** Set the I2C address of the specified slave (0-3).
 * @param num Slave number (0-3)
 * @param address New address for specified slave
 * @see getSlaveAddress()
 * @see MPU6050_RA_I2C_SLV0_ADDR
 */
void MPU6050_Base::setSlaveAddress(uint8_t num, uint8_t address) {
    if (num > 3) return;
    I2Cdev::writeByte(devAddr, MPU6050_RA_I2C_SLV0_ADDR + num*3, address, wireObj);
}

/** Set I2C Master Mode enabled status.
 * @param enabled New I2C Master Mode enabled status
 * @see getI2CMasterModeEnabled()
 * @see MPU6050_RA_USER_CTRL
 * @see MPU6050_USERCTRL_I2C_MST_EN_BIT
 */
void MPU6050_Base::setI2CMasterModeEnabled(bool enabled) {
    I2Cdev::writeBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT, enabled, wireObj);
}

/** Reset the I2C Master.
 * This bit resets the I2C Master when set to 1 while I2C_MST_EN equals 0.
 * This bit automatically clears to 0 after the reset has been triggered.
 * @see MPU6050_RA_USER_CTRL
 * @see MPU6050_USERCTRL_I2C_MST_RESET_BIT
 */
void MPU6050_Base::resetI2CMaster() {
    I2Cdev::writeBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_RESET_BIT, true, wireObj);
}

/** Set clock source setting.
 * An internal 8MHz oscillator, gyroscope based clock, or external sources can
 * be selected as the MPU-60X0 clock source. When the internal 8 MHz oscillator
 * or an external source is chosen as the clock source, the MPU-60X0 can operate
 * in low power modes with the gyroscopes disabled.
 *
 * Upon power up, the MPU-60X0 clock source defaults to the internal oscillator.
 * However, it is highly recommended that the device be configured to use one of
 * the gyroscopes (or an external clock source) as the clock reference for
 * improved stability. The clock source can be selected according to the following table:
 *
 * <pre>
 * CLK_SEL | Clock Source
 * --------+--------------------------------------
 * 0       | Internal oscillator
 * 1       | PLL with X Gyro reference
 * 2       | PLL with Y Gyro reference
 * 3       | PLL with Z Gyro reference
 * 4       | PLL with external 32.768kHz reference
 * 5       | PLL with external 19.2MHz reference
 * 6       | Reserved
 * 7       | Stops the clock and keeps the timing generator in reset
 * </pre>
 *
 * @param source New clock source setting
 * @see getClockSource()
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_CLKSEL_BIT
 * @see MPU6050_PWR1_CLKSEL_LENGTH
 */
void MPU6050_Base::setClockSource(uint8_t source) {
    I2Cdev::writeBits(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source, wireObj);
}

/** Set full interrupt enabled status.
 * Full register byte for all interrupts, for quick reading. Each bit should be
 * set 0 for disabled, 1 for enabled.
 * @param enabled New interrupt enabled status
 * @see getIntFreefallEnabled()
 * @see MPU6050_RA_INT_ENABLE
 * @see MPU6050_INTERRUPT_FF_BIT
 **/
void MPU6050_Base::setIntEnabled(uint8_t enabled) {
    I2Cdev::writeByte(devAddr, MPU6050_RA_INT_ENABLE, enabled, wireObj);
}

/** Set gyroscope sample rate divider.
 * @param rate New sample rate divider
 * @see getRate()
 * @see MPU6050_RA_SMPLRT_DIV
 */
void MPU6050_Base::setRate(uint8_t rate) {
    I2Cdev::writeByte(devAddr, MPU6050_RA_SMPLRT_DIV, rate, wireObj);
}

/** Set external FSYNC configuration.
 * @see getExternalFrameSync()
 * @see MPU6050_RA_CONFIG
 * @param sync New FSYNC configuration value
 */
void MPU6050_Base::setExternalFrameSync(uint8_t sync) {
    I2Cdev::writeBits(devAddr, MPU6050_RA_CONFIG, MPU6050_CFG_EXT_SYNC_SET_BIT, MPU6050_CFG_EXT_SYNC_SET_LENGTH, sync, wireObj);
}

/** Set digital low-pass filter configuration.
 * @param mode New DLFP configuration setting
 * @see getDLPFBandwidth()
 * @see MPU6050_DLPF_BW_256
 * @see MPU6050_RA_CONFIG
 * @see MPU6050_CFG_DLPF_CFG_BIT
 * @see MPU6050_CFG_DLPF_CFG_LENGTH
 */
void MPU6050_Base::setDLPFMode(uint8_t mode) {
    I2Cdev::writeBits(devAddr, MPU6050_RA_CONFIG, MPU6050_CFG_DLPF_CFG_BIT, MPU6050_CFG_DLPF_CFG_LENGTH, mode, wireObj);
}

/** Set full-scale gyroscope range.
 * @param range New full-scale gyroscope range value
 * @see getFullScaleRange()
 * @see MPU6050_GYRO_FS_250
 * @see MPU6050_RA_GYRO_CONFIG
 * @see MPU6050_GCONFIG_FS_SEL_BIT
 * @see MPU6050_GCONFIG_FS_SEL_LENGTH
 */
void MPU6050_Base::setFullScaleGyroRange(uint8_t range) {
    I2Cdev::writeBits(devAddr, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range, wireObj);
}

bool MPU6050_Base::writeProgMemoryBlock(const uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address, bool verify) {
    return writeMemoryBlock(data, dataSize, bank, address, verify, false);
}

bool MPU6050_Base::writeMemoryBlock(const uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address, bool verify, bool useProgMem) {
    setMemoryBank(bank);
    setMemoryStartAddress(address);
    uint8_t chunkSize;
    uint8_t *verifyBuffer=0;
    uint8_t *progBuffer=0;
    uint16_t i;
    //uint8_t j;
    //if (verify) verifyBuffer = (uint8_t *)malloc(MPU6050_DMP_MEMORY_CHUNK_SIZE);
    //if (useProgMem) progBuffer = (uint8_t *)malloc(MPU6050_DMP_MEMORY_CHUNK_SIZE);
    for (i = 0; i < dataSize;) {
        // determine correct chunk size according to bank position and data size
        chunkSize = MPU6050_DMP_MEMORY_CHUNK_SIZE;
        // make sure we don't go past the data size
        if (i + chunkSize > dataSize) chunkSize = dataSize - i;
        // make sure this chunk doesn't go past the bank boundary (256 bytes)
        if (chunkSize > 256 - address) chunkSize = 256 - address;
        if (useProgMem) {
            // write the chunk of data as specified
            //for (j = 0; j < chunkSize; j++) progBuffer[j] = pgm_read_byte(data + i + j);
        } else {
            // write the chunk of data as specified
            progBuffer = (uint8_t *)data + i;
        }
        I2Cdev::writeBytes(devAddr, MPU6050_RA_MEM_R_W, chunkSize, progBuffer, wireObj);
        // verify data if needed
        /*if (verify && verifyBuffer) {
            setMemoryBank(bank);
            setMemoryStartAddress(address);
            I2Cdev::readBytes(devAddr, MPU6050_RA_MEM_R_W, chunkSize, verifyBuffer, I2Cdev::readTimeout, wireObj);
            if (memcmp(progBuffer, verifyBuffer, chunkSize) != 0) {
                free(verifyBuffer);
                if (useProgMem) free(progBuffer);
                return false; // uh oh.
            }
        }*/

        // increase byte index by [chunkSize]
        i += chunkSize;

        // uint8_t automatically wraps to 0 at 256
        address += chunkSize;

        // if we aren't done, update bank (if necessary) and address
        if (i < dataSize) {
            if (address == 0) bank++;
            setMemoryBank(bank);
            setMemoryStartAddress(address);
        }
    }
    if (verify) free(verifyBuffer);
    if (useProgMem) free(progBuffer);
    return true;
}

// DMP_CFG_1 register
void MPU6050_Base::setDMPConfig1(uint8_t config) {
    I2Cdev::writeByte(devAddr, MPU6050_RA_DMP_CFG_1, config, wireObj);
}

void MPU6050_Base::setDMPConfig2(uint8_t config) {
    I2Cdev::writeByte(devAddr, MPU6050_RA_DMP_CFG_2, config, wireObj);
}

// DMP_CFG_2 register
uint8_t MPU6050_Base::getDMPConfig2() {
    I2Cdev::readByte(devAddr, MPU6050_RA_DMP_CFG_2, buffer, I2Cdev::readTimeout, wireObj);
    return buffer[0];
}

void MPU6050_Base::setOTPBankValid(bool enabled) {
    I2Cdev::writeBit(devAddr, MPU6050_RA_XG_OFFS_TC, MPU6050_TC_OTP_BNK_VLD_BIT, enabled, wireObj);
}

/** Set motion detection event acceleration threshold.
 * @param threshold New motion detection acceleration threshold value (LSB = 2mg)
 * @see getMotionDetectionThreshold()
 * @see MPU6050_RA_MOT_THR
 */
void MPU6050_Base::setMotionDetectionThreshold(uint8_t threshold) {
    I2Cdev::writeByte(devAddr, MPU6050_RA_MOT_THR, threshold, wireObj);
}

/** Set zero motion detection event acceleration threshold.
 * @param threshold New zero motion detection acceleration threshold value (LSB = 2mg)
 * @see getZeroMotionDetectionThreshold()
 * @see MPU6050_RA_ZRMOT_THR
 */
void MPU6050_Base::setZeroMotionDetectionThreshold(uint8_t threshold) {
    I2Cdev::writeByte(devAddr, MPU6050_RA_ZRMOT_THR, threshold, wireObj);
}

/** Set motion detection event duration threshold.
 * @param duration New motion detection duration threshold value (LSB = 1ms)
 * @see getMotionDetectionDuration()
 * @see MPU6050_RA_MOT_DUR
 */
void MPU6050_Base::setMotionDetectionDuration(uint8_t duration) {
    I2Cdev::writeByte(devAddr, MPU6050_RA_MOT_DUR, duration, wireObj);
}

/** Set zero motion detection event duration threshold.
 * @param duration New zero motion detection duration threshold value (LSB = 1ms)
 * @see getZeroMotionDetectionDuration()
 * @see MPU6050_RA_ZRMOT_DUR
 */
void MPU6050_Base::setZeroMotionDetectionDuration(uint8_t duration) {
    I2Cdev::writeByte(devAddr, MPU6050_RA_ZRMOT_DUR, duration, wireObj);
}

/** Set FIFO enabled status.
 * @param enabled New FIFO enabled status
 * @see getFIFOEnabled()
 * @see MPU6050_RA_USER_CTRL
 * @see MPU6050_USERCTRL_FIFO_EN_BIT
 */
void MPU6050_Base::setFIFOEnabled(bool enabled) {
    I2Cdev::writeBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_FIFO_EN_BIT, enabled, wireObj);
}

void MPU6050_Base::setDMPEnabled(bool enabled) {
    I2Cdev::writeBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_DMP_EN_BIT, enabled, wireObj);
}

/** Reset the FIFO.
 * This bit resets the FIFO buffer when set to 1 while FIFO_EN equals 0. This
 * bit automatically clears to 0 after the reset has been triggered.
 * @see MPU6050_RA_USER_CTRL
 * @see MPU6050_USERCTRL_FIFO_RESET_BIT
 */
void MPU6050_Base::resetFIFO() {
    I2Cdev::writeBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_FIFO_RESET_BIT, true, wireObj);
}

void MPU6050_Base::resetDMP() {
    I2Cdev::writeBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_DMP_RESET_BIT, true, wireObj);
}

// PWR_MGMT_1 register
/** Trigger a full device reset.
 * A small delay of ~50ms may be desirable after triggering a reset.
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_DEVICE_RESET_BIT
 */
void MPU6050_Base::reset() {
    I2Cdev::writeBit(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_DEVICE_RESET_BIT, true, wireObj);
}

/** Get full set of interrupt status bits.
 * These bits clear to 0 after the register has been read. Very useful
 * for getting multiple INT statuses, since each single bit read clears
 * all of them because it has to read the whole byte.
 * @return Current interrupt status
 * @see MPU6050_RA_INT_STATUS
 */
uint8_t MPU6050_Base::getIntStatus() {
    I2Cdev::readByte(devAddr, MPU6050_RA_INT_STATUS, buffer, I2Cdev::readTimeout, wireObj);
    return buffer[0];
}

/** Get timeout to get a packet from FIFO buffer.
 * @return Current timeout to get a packet from FIFO buffer
 * @see MPU6050_FIFO_DEFAULT_TIMEOUT
 */
uint32_t MPU6050_Base::getFIFOTimeout() {
	return fifoTimeout;
}

// FIFO_COUNT* registers
/** Get current FIFO buffer size.
 * This value indicates the number of bytes stored in the FIFO buffer. This
 * number is in turn the number of bytes that can be read from the FIFO buffer
 * and it is directly proportional to the number of samples available given the
 * set of sensor data bound to be stored in the FIFO (register 35 and 36).
 * @return Current FIFO buffer size
 */
uint16_t MPU6050_Base::getFIFOCount() {
    I2Cdev::readBytes(devAddr, MPU6050_RA_FIFO_COUNTH, 2, buffer, I2Cdev::readTimeout, wireObj);
    return (((uint16_t)buffer[0]) << 8) | buffer[1];
}

/** Get latest byte from FIFO buffer no matter how much time has passed.
 * ===                  GetCurrentFIFOPacket                    ===
 * ================================================================
 * Returns 1) when nothing special was done
 *         2) when recovering from overflow
 *         0) when no valid data is available
 * ================================================================ */
 int8_t MPU6050_Base::GetCurrentFIFOPacket(uint8_t *data, uint8_t length) { // overflow proof
     int16_t fifoC;
     uint32_t BreakTimer = HAL_GetTick();
     // This section of code is for when we allowed more than 1 packet to be acquired
     bool packetReceived = false;
     do {
         if ((fifoC = getFIFOCount())  > length) {
             if (fifoC > 200) { // if you waited to get the FIFO buffer to > 200 bytes it will take longer to get the last packet in the FIFO Buffer than it will take to  reset the buffer and wait for the next to arrive
                 resetFIFO();   // Fixes any overflow corruption
                 fifoC = 0;
                 while (!(fifoC = getFIFOCount()) && ((HAL_GetTick() - BreakTimer) <= (getFIFOTimeout()))); // Get Next New Packet
                 } else { //We have more than 1 packet but less than 200 bytes of data in the FIFO Buffer
                 uint8_t Trash[I2CDEVLIB_WIRE_BUFFER_LENGTH];
                 while ((fifoC = getFIFOCount()) > length) {  // Test each time just in case the MPU is writing to the FIFO Buffer
                     fifoC = fifoC - length; // Save the last packet
                     uint16_t  RemoveBytes;
                     while (fifoC) { // fifo count will reach zero so this is safe
                         RemoveBytes = (fifoC < I2CDEVLIB_WIRE_BUFFER_LENGTH) ? fifoC : I2CDEVLIB_WIRE_BUFFER_LENGTH; // Buffer Length is different than the packet length this will efficiently clear the buffer
                         getFIFOBytes(Trash, (uint8_t)RemoveBytes);
                         fifoC -= RemoveBytes;
                     }
                 }
             }
         }
         if (!fifoC) return 0; // Called too early no data or we timed out after FIFO Reset
         // We have 1 packet
         packetReceived = fifoC == length;
         if (!packetReceived && (HAL_GetTick() - BreakTimer) > (getFIFOTimeout())) return 0;
     } while (!packetReceived);
     getFIFOBytes(data, length); //Get 1 packet
     return 1;
}

 void MPU6050_Base::getFIFOBytes(uint8_t *data, uint8_t length) {
     if(length > 0){
         I2Cdev::readBytes(devAddr, MPU6050_RA_FIFO_R_W, length, data, I2Cdev::readTimeout, wireObj);
     } else {
     	*data = 0;
     }
 }

// XG_OFFS_TC register
uint8_t MPU6050_Base::getOTPBankValid() {
    I2Cdev::readBit(devAddr, MPU6050_RA_XG_OFFS_TC, MPU6050_TC_OTP_BNK_VLD_BIT, buffer, I2Cdev::readTimeout, wireObj);
    return buffer[0];
}
