#include "MPU6050.h"

// Public methods

MPU6050::MPU6050(PinName sda, PinName scl):
    i2c_(sda, scl),
    gyro_scale_(GFS_250DPS),
    accel_scale_(AFS_2G),
    gyro_bias_(),
    accel_bias_(),
    last_update_(0),
    first_update_(0),
    now_(0),
    delta_t_(0.0f),
    count_(0),
    q()
{
    t_.start();

    q[0] = 1.0f;
    q[1] = 0.0f;
    q[2] = 0.0f;
    q[3] = 0.0f;

    // Coonfigure imu
    i2c_.frequency(400000); // use fast (400 kHz) I2C

    // Read WHO_AM_I register for MPU-6050
    uint8_t whoami = readByte(MPU6050_ADDRESS, WHO_AM_I_MPU6050);
    if (whoami == 0x68) // WHO_AM_I should always be 0x68
    {
        // Start by performing self test and reporting values
        selfTestMPU6050();

        if(self_test_[0] < 1.0f && self_test_[1] < 1.0f && self_test_[2] < 1.0f && \
           self_test_[3] < 1.0f && self_test_[4] < 1.0f && self_test_[5] < 1.0f)
        {
             // Reset registers to default in preparation for device calibration
            resetMPU6050();

            // Calibrate gyro and accelerometers, load biases in bias registers
            calibrateMPU6050();

            // Initialize device for active mode read of acclerometer, gyroscope,
            // and temperature
            initMPU6050();
        }
        else
        {
            //Hase::debug("IMU did not pass the self-test");
        }
    }
    else
    {
        //Hase::debug("Could not connect to the IMU: %#x", whoami);
        while(1); // Loop forever if communication doesn't happen
    }
}

void MPU6050::readDataMPU6050()
{
    if(readByte(MPU6050_ADDRESS, INT_STATUS) & 0x01) {  // check if data ready interrupt
        readAccelerometerData();  // Read the x/y/z adc values
        readAccelerometerResolution();

        // Now we'll calculate the accleration value into actual g's
        ax = (float)accel_count_[0]*accel_res_ - accel_bias_[0];  // get actual g value, this depends on scale being set
        ay = (float)accel_count_[1]*accel_res_ - accel_bias_[1];
        az = (float)accel_count_[2]*accel_res_ - accel_bias_[2];

        readGyroscopeData();  // Read the x/y/z adc values
        readGyroscopeResolution();


        // Calculate the gyro value into actual degrees per second
        gx = (float)gyro_count_[0]*gyro_res_; // - gyro_bias_[0];  // get actual gyro value, this depends on scale being set
        gy = (float)gyro_count_[1]*gyro_res_; // - gyro_bias_[1];
        gz = (float)gyro_count_[2]*gyro_res_; // - gyro_bias_[2];

        gx_rad = gx*PI/180.0f;
        gy_rad = gy*PI/180.0f;
        gz_rad = gz*PI/180.0f;

        readTemperatureData();
        temperature = (temp_count_) / 340. + 36.53; // Temperature in degrees Centigrade
    }

    now_ = t_.read_us();
    delta_t_ = (float)((now_ - last_update_)/1000000.0f) ; // set integration time by time elapsed since last filter update
    last_update_ = now_;

    if(last_update_ - first_update_ > 10000000.0f) {
     beta_ = 0.04;  // decrease filter gain after stabilized
     zeta_ = 0.015; // increasey bias drift gain after stabilized
    }

    MadgwickQuaternionUpdate();

    // Serial print and/or display at 0.5 s rate independent of data rates
    //delta_t_ = t_.read_ms() - count_;
    //count_ = t_.read_ms();
}

// Private methods

void MPU6050::readGyroscopeResolution() {
    switch (gyro_scale_)
    {
        // Possible gyro scales (and their register bit settings) are:
        // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
        case GFS_250DPS:
            gyro_res_ = 250.0/32768.0;
            break;
        case GFS_500DPS:
            gyro_res_ = 500.0/32768.0;
            break;
        case GFS_1000DPS:
            gyro_res_ = 1000.0/32768.0;
            break;
        case GFS_2000DPS:
            gyro_res_ = 2000.0/32768.0;
            break;
    }
}

void MPU6050::readAccelerometerResolution() {
    switch (accel_scale_)
    {
        // Possible accelerometer scales (and their register bit settings) are:
        // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
        case AFS_2G:
              accel_res_ = 2.0/32768.0;
              break;
        case AFS_4G:
              accel_res_ = 4.0/32768.0;
              break;
        case AFS_8G:
              accel_res_ = 8.0/32768.0;
              break;
        case AFS_16G:
              accel_res_ = 16.0/32768.0;
              break;
    }
}


void MPU6050::readAccelerometerData()
{
    uint8_t rawData[6];  // x/y/z accel register data stored here

    // Read the six raw data registers into data array
    readBytes(MPU6050_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);

    // Turn the MSB and LSB into a signed 16-bit value
    accel_count_[0] = (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]);
    accel_count_[1] = (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);
    accel_count_[2] = (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);
}

void MPU6050::readGyroscopeData()
{
    uint8_t rawData[6];  // x/y/z gyro register data stored here

    // Read the six raw data registers sequentially into data array
    readBytes(MPU6050_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);

    // Turn the MSB and LSB into a signed 16-bit value
    gyro_count_[0] = (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]);
    gyro_count_[1] = (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);
    gyro_count_[2] = (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);
}

void  MPU6050::readTemperatureData()
{
    uint8_t rawData[2];  // x/y/z gyro register data stored here

    // Read the two raw data registers sequentially into data array
    readBytes(MPU6050_ADDRESS, TEMP_OUT_H, 2, &rawData[0]);

    // Turn the MSB and LSB into a 16-bit value
    temp_count_ = (int16_t)(((int16_t)rawData[0]) << 8 | rawData[1]);
}

void MPU6050::LowPowerAccelOnly()
{

    /* The sensor has a high-pass filter necessary to invoke to allow the sensor motion detection algorithms work properly
    Motion detection occurs on free-fall (acceleration below a threshold for some time for all axes), motion (acceleration
    above a threshold for some time on at least one axis), and zero-motion toggle (acceleration on each axis less than a
    threshold for some time sets this flag, motion above the threshold turns it off). The high-pass filter takes gravity out
    consideration for these threshold evaluations; otherwise, the flags would be set all the time! */

    uint8_t c = readByte(MPU6050_ADDRESS, PWR_MGMT_1);
    writeByte(MPU6050_ADDRESS, PWR_MGMT_1, c & ~0x30); // Clear sleep and cycle bits [5:6]
    writeByte(MPU6050_ADDRESS, PWR_MGMT_1, c |  0x30); // Set sleep and cycle bits [5:6] to zero to make sure accelerometer is running

    c = readByte(MPU6050_ADDRESS, PWR_MGMT_2);
    writeByte(MPU6050_ADDRESS, PWR_MGMT_2, c & ~0x38); // Clear standby XA, YA, and ZA bits [3:5]
    writeByte(MPU6050_ADDRESS, PWR_MGMT_2, c |  0x00); // Set XA, YA, and ZA bits [3:5] to zero to make sure accelerometer is running

    c = readByte(MPU6050_ADDRESS, ACCEL_CONFIG);
    writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, c & ~0x07); // Clear high-pass filter bits [2:0]
    // Set high-pass filter to 0) reset (disable), 1) 5 Hz, 2) 2.5 Hz, 3) 1.25 Hz, 4) 0.63 Hz, or 7) Hold
    writeByte(MPU6050_ADDRESS, ACCEL_CONFIG,  c | 0x00);  // Set ACCEL_HPF to 0; reset mode disbaling high-pass filter

    c = readByte(MPU6050_ADDRESS, CONFIG);
    writeByte(MPU6050_ADDRESS, CONFIG, c & ~0x07); // Clear low-pass filter bits [2:0]
    writeByte(MPU6050_ADDRESS, CONFIG, c |  0x00);  // Set DLPD_CFG to 0; 260 Hz bandwidth, 1 kHz rate

    c = readByte(MPU6050_ADDRESS, INT_ENABLE);
    writeByte(MPU6050_ADDRESS, INT_ENABLE, c & ~0xFF);  // Clear all interrupts
    writeByte(MPU6050_ADDRESS, INT_ENABLE, 0x40);  // Enable motion threshold (bits 5) interrupt only

    // Motion detection interrupt requires the absolute value of any axis to lie above the detection threshold
    // for at least the counter duration
    writeByte(MPU6050_ADDRESS, MOT_THR, 0x80); // Set motion detection to 0.256 g; LSB = 2 mg
    writeByte(MPU6050_ADDRESS, MOT_DUR, 0x01); // Set motion detect duration to 1  ms; LSB is 1 ms @ 1 kHz rate

    wait(0.1);  // Add delay for accumulation of samples

    c = readByte(MPU6050_ADDRESS, ACCEL_CONFIG);
    writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, c & ~0x07); // Clear high-pass filter bits [2:0]
    writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, c |  0x07);  // Set ACCEL_HPF to 7; hold the initial accleration value as a referance

    c = readByte(MPU6050_ADDRESS, PWR_MGMT_2);
    writeByte(MPU6050_ADDRESS, PWR_MGMT_2, c & ~0xC7); // Clear standby XA, YA, and ZA bits [3:5] and LP_WAKE_CTRL bits [6:7]
    writeByte(MPU6050_ADDRESS, PWR_MGMT_2, c |  0x47); // Set wakeup frequency to 5 Hz, and disable XG, YG, and ZG gyros (bits [0:2])

    c = readByte(MPU6050_ADDRESS, PWR_MGMT_1);
    writeByte(MPU6050_ADDRESS, PWR_MGMT_1, c & ~0x20); // Clear sleep and cycle bit 5
    writeByte(MPU6050_ADDRESS, PWR_MGMT_1, c |  0x20); // Set cycle bit 5 to begin low power accelerometer motion interrupts

}

void MPU6050::resetMPU6050() {
    // Reset device
    writeByte(MPU6050_ADDRESS, PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
    wait(0.1);
}

void MPU6050::initMPU6050()
{
    // Initialize MPU6050 device
    // wake up device
    writeByte(MPU6050_ADDRESS, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors
    wait(0.1); // Delay 100 ms for PLL to get established on x-axis gyro; should check for PLL ready interrupt

    // get stable time source
    writeByte(MPU6050_ADDRESS, PWR_MGMT_1, 0x01);  // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001

    // Configure Gyro and Accelerometer
    // Disable FSYNC and set accelerometer and gyro bandwidth to 44 and 42 Hz, respectively;
    // DLPF_CFG = bits 2:0 = 010; this sets the sample rate at 1 kHz for both
    // Maximum delay is 4.9 ms which is just over a 200 Hz maximum rate
    writeByte(MPU6050_ADDRESS, CONFIG, 0x03);

    // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
    writeByte(MPU6050_ADDRESS, SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; the same rate set in CONFIG above

    // Set gyroscope full scale range
    // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
    uint8_t c =  readByte(MPU6050_ADDRESS, GYRO_CONFIG);
    writeByte(MPU6050_ADDRESS, GYRO_CONFIG, c & ~0xE0); // Clear self-test bits [7:5]
    writeByte(MPU6050_ADDRESS, GYRO_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
    writeByte(MPU6050_ADDRESS, GYRO_CONFIG, c | gyro_scale_ << 3); // Set full scale range for the gyro

    // Set accelerometer configuration
    c =  readByte(MPU6050_ADDRESS, ACCEL_CONFIG);
    writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, c & ~0xE0); // Clear self-test bits [7:5]
    writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
    writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, c | accel_scale_ << 3); // Set full scale range for the accelerometer

    // Configure Interrupts and Bypass Enable
    // Set interrupt pin active high, push-pull, and clear on read of INT_STATUS, enable I2C_BYPASS_EN so additional chips
    // can join the I2C bus and all can be controlled by the Arduino as master
    writeByte(MPU6050_ADDRESS, INT_PIN_CFG, 0x22);
    writeByte(MPU6050_ADDRESS, INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
}

void MPU6050::calibrateMPU6050()
{
    uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
    uint16_t ii, packet_count, fifo_count;
    int32_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

    // Reset device, reset all registers, clear gyro and accelerometer bias registers
    writeByte(MPU6050_ADDRESS, PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
    wait(0.1);

    // Get stable time source
    // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
    writeByte(MPU6050_ADDRESS, PWR_MGMT_1, 0x01);
    writeByte(MPU6050_ADDRESS, PWR_MGMT_2, 0x00);
    wait(0.2);

    // Configure device for bias calculation
    writeByte(MPU6050_ADDRESS, INT_ENABLE, 0x00);   // Disable all interrupts
    writeByte(MPU6050_ADDRESS, FIFO_EN, 0x00);      // Disable FIFO
    writeByte(MPU6050_ADDRESS, PWR_MGMT_1, 0x00);   // Turn on internal clock source
    writeByte(MPU6050_ADDRESS, I2C_MST_CTRL, 0x00); // Disable I2C master
    writeByte(MPU6050_ADDRESS, USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
    writeByte(MPU6050_ADDRESS, USER_CTRL, 0x0C);    // Reset FIFO and DMP
    wait(0.015);

    // Configure MPU6050 gyro and accelerometer for bias calculation
    writeByte(MPU6050_ADDRESS, CONFIG, 0x01);      // Set low-pass filter to 188 Hz
    writeByte(MPU6050_ADDRESS, SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
    writeByte(MPU6050_ADDRESS, GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
    writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

    uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
    uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

    // Configure FIFO to capture accelerometer and gyro data for bias calculation
    writeByte(MPU6050_ADDRESS, USER_CTRL, 0x40);   // Enable FIFO
    writeByte(MPU6050_ADDRESS, FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 1024 bytes in MPU-6050)
    wait(0.08); // accumulate 80 samples in 80 milliseconds = 960 bytes

    // At end of sample accumulation, turn off FIFO sensor read
    writeByte(MPU6050_ADDRESS, FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
    readBytes(MPU6050_ADDRESS, FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
    fifo_count = ((uint16_t)data[0] << 8) | data[1];
    packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging

    for (ii = 0; ii < packet_count; ii++) {
        int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
        readBytes(MPU6050_ADDRESS, FIFO_R_W, 12, &data[0]); // read data for averaging

        // Form signed 16-bit integer for each sample in FIFO
        accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;
        accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
        accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;
        gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
        gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
        gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;

        // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
        accel_bias[0] += (int32_t) accel_temp[0];
        accel_bias[1] += (int32_t) accel_temp[1];
        accel_bias[2] += (int32_t) accel_temp[2];
        gyro_bias[0]  += (int32_t) gyro_temp[0];
        gyro_bias[1]  += (int32_t) gyro_temp[1];
        gyro_bias[2]  += (int32_t) gyro_temp[2];

    }

    // Normalize sums to get average count biases
    accel_bias[0] /= (int32_t) packet_count;
    accel_bias[1] /= (int32_t) packet_count;
    accel_bias[2] /= (int32_t) packet_count;
    gyro_bias[0]  /= (int32_t) packet_count;
    gyro_bias[1]  /= (int32_t) packet_count;
    gyro_bias[2]  /= (int32_t) packet_count;

    if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
    else {accel_bias[2] += (int32_t) accelsensitivity;}

    // Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
    data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
    data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
    data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
    data[3] = (-gyro_bias[1]/4)       & 0xFF;
    data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
    data[5] = (-gyro_bias[2]/4)       & 0xFF;

    // Push gyro biases to hardware registers
    writeByte(MPU6050_ADDRESS, XG_OFFS_USRH, data[0]);
    writeByte(MPU6050_ADDRESS, XG_OFFS_USRL, data[1]);
    writeByte(MPU6050_ADDRESS, YG_OFFS_USRH, data[2]);
    writeByte(MPU6050_ADDRESS, YG_OFFS_USRL, data[3]);
    writeByte(MPU6050_ADDRESS, ZG_OFFS_USRH, data[4]);
    writeByte(MPU6050_ADDRESS, ZG_OFFS_USRL, data[5]);

    // Construct gyro bias in deg/s for later manual subtraction
    gyro_bias_[0] = (float) gyro_bias[0]/(float) gyrosensitivity;
    gyro_bias_[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
    gyro_bias_[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

    // Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
    // factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
    // non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
    // compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
    // the accelerometer biases calculated above must be divided by 8.

    int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases

    // Read factory accelerometer trim values
    readBytes(MPU6050_ADDRESS, XA_OFFSET_H, 2, &data[0]);
    accel_bias_reg[0] = (int16_t) ((int16_t)data[0] << 8) | data[1];
    readBytes(MPU6050_ADDRESS, YA_OFFSET_H, 2, &data[0]);
    accel_bias_reg[1] = (int16_t) ((int16_t)data[0] << 8) | data[1];
    readBytes(MPU6050_ADDRESS, ZA_OFFSET_H, 2, &data[0]);
    accel_bias_reg[2] = (int16_t) ((int16_t)data[0] << 8) | data[1];

    uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
    uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis

    for(ii = 0; ii < 3; ii++) {
        if(accel_bias_reg[ii] & mask) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
    }

    // Construct total accelerometer bias, including calculated average accelerometer bias from above
    accel_bias_reg[0] -= (accel_bias[0]/8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
    accel_bias_reg[1] -= (accel_bias[1]/8);
    accel_bias_reg[2] -= (accel_bias[2]/8);

    data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
    data[1] = (accel_bias_reg[0])      & 0xFF;
    data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
    data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
    data[3] = (accel_bias_reg[1])      & 0xFF;
    data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
    data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
    data[5] = (accel_bias_reg[2])      & 0xFF;
    data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

    // Push accelerometer biases to hardware registers
    // writeByte(MPU6050_ADDRESS, XA_OFFSET_H, data[0]);
    // writeByte(MPU6050_ADDRESS, XA_OFFSET_L_TC, data[1]);
    // writeByte(MPU6050_ADDRESS, YA_OFFSET_H, data[2]);
    // writeByte(MPU6050_ADDRESS, YA_OFFSET_L_TC, data[3]);
    // writeByte(MPU6050_ADDRESS, ZA_OFFSET_H, data[4]);
    // writeByte(MPU6050_ADDRESS, ZA_OFFSET_L_TC, data[5]);

    // Output scaled accelerometer biases for manual subtraction in the main program
    accel_bias_[0] = (float)accel_bias[0]/(float)accelsensitivity;
    accel_bias_[1] = (float)accel_bias[1]/(float)accelsensitivity;
    accel_bias_[2] = (float)accel_bias[2]/(float)accelsensitivity;
}

void MPU6050::selfTestMPU6050()
{
    uint8_t rawData[4] = {0, 0, 0, 0};
    float factoryTrim[6];

    // Configure the accelerometer for self-test
    writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, 0xF0); // Enable self test on all three axes and set accelerometer range to +/- 8 g
    writeByte(MPU6050_ADDRESS, GYRO_CONFIG,  0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s

    wait(0.25);  // Delay a while to let the device execute the self-test

    rawData[0] = readByte(MPU6050_ADDRESS, SELF_TEST_X); // X-axis self-test results
    rawData[1] = readByte(MPU6050_ADDRESS, SELF_TEST_Y); // Y-axis self-test results
    rawData[2] = readByte(MPU6050_ADDRESS, SELF_TEST_Z); // Z-axis self-test results
    rawData[3] = readByte(MPU6050_ADDRESS, SELF_TEST_A); // Mixed-axis self-test results

    // Extract the acceleration test results first
    self_test_[0] = (rawData[0] >> 3) | (rawData[3] & 0x30) >> 4 ; // XA_TEST result is a five-bit unsigned integer
    self_test_[1] = (rawData[1] >> 3) | (rawData[3] & 0x0C) >> 4 ; // YA_TEST result is a five-bit unsigned integer
    self_test_[2] = (rawData[2] >> 3) | (rawData[3] & 0x03) >> 4 ; // ZA_TEST result is a five-bit unsigned integer

    // Extract the gyration test results first
    self_test_[3] = rawData[0]  & 0x1F ; // XG_TEST result is a five-bit unsigned integer
    self_test_[4] = rawData[1]  & 0x1F ; // YG_TEST result is a five-bit unsigned integer
    self_test_[5] = rawData[2]  & 0x1F ; // ZG_TEST result is a five-bit unsigned integer

    // Process results to allow final comparison with factory set values
    factoryTrim[0] = (4096.0f*0.34f)*(pow( (0.92f/0.34f) , ((self_test_[0] - 1.0f)/30.0f))); // FT[Xa] factory trim calculation
    factoryTrim[1] = (4096.0f*0.34f)*(pow( (0.92f/0.34f) , ((self_test_[1] - 1.0f)/30.0f))); // FT[Ya] factory trim calculation
    factoryTrim[2] = (4096.0f*0.34f)*(pow( (0.92f/0.34f) , ((self_test_[2] - 1.0f)/30.0f))); // FT[Za] factory trim calculation
    factoryTrim[3] =  ( 25.0f*131.0f)*(pow( 1.046f , (self_test_[3] - 1.0f) ));             // FT[Xg] factory trim calculation
    factoryTrim[4] =  (-25.0f*131.0f)*(pow( 1.046f , (self_test_[4] - 1.0f) ));             // FT[Yg] factory trim calculation
    factoryTrim[5] =  ( 25.0f*131.0f)*(pow( 1.046f , (self_test_[5] - 1.0f) ));             // FT[Zg] factory trim calculation

    //  Output self-test results and factory trim calculation if desired
    //  Serial.println(self_test_[0]); Serial.println(self_test_[1]); Serial.println(self_test_[2]);
    //  Serial.println(self_test_[3]); Serial.println(self_test_[4]); Serial.println(self_test_[5]);
    //  Serial.println(factoryTrim[0]); Serial.println(factoryTrim[1]); Serial.println(factoryTrim[2]);
    //  Serial.println(factoryTrim[3]); Serial.println(factoryTrim[4]); Serial.println(factoryTrim[5]);

    // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
    // To get to percent, must multiply by 100 and subtract result from 100
   for (int i = 0; i < 6; i++) {
        self_test_[i] = 100.0f + 100.0f*(self_test_[i] - factoryTrim[i])/factoryTrim[i]; // Report percent differences
   }
}

void MPU6050::MadgwickQuaternionUpdate()
{
    float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];           // short name local variable for readability
    float norm;                                                 // vector norm
    float f1, f2, f3;                                           // objective funcyion elements
    float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33;   // objective function Jacobian elements
    float qDot1, qDot2, qDot3, qDot4;
    float hatDot1, hatDot2, hatDot3, hatDot4;
    float gerrx, gerry, gerrz, gbiasx, gbiasy, gbiasz;          // gyro bias error

    // Auxiliary variables to avoid repeated arithmetic
    float _halfq1 = 0.5f * q1;
    float _halfq2 = 0.5f * q2;
    float _halfq3 = 0.5f * q3;
    float _halfq4 = 0.5f * q4;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;
    float _2q4 = 2.0f * q4;
    //float _2q1q3 = 2.0f * q1 * q3;
    //float _2q3q4 = 2.0f * q3 * q4;

    // Normalise accelerometer measurement
    norm = sqrt(ax * ax + ay * ay + az * az);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f/norm;
    ax *= norm;
    ay *= norm;
    az *= norm;

    // Compute the objective function and Jacobian
    f1 = _2q2 * q4 - _2q1 * q3 - ax;
    f2 = _2q1 * q2 + _2q3 * q4 - ay;
    f3 = 1.0f - _2q2 * q2 - _2q3 * q3 - az;
    J_11or24 = _2q3;
    J_12or23 = _2q4;
    J_13or22 = _2q1;
    J_14or21 = _2q2;
    J_32 = 2.0f * J_14or21;
    J_33 = 2.0f * J_11or24;

    // Compute the gradient (matrix multiplication)
    hatDot1 = J_14or21 * f2 - J_11or24 * f1;
    hatDot2 = J_12or23 * f1 + J_13or22 * f2 - J_32 * f3;
    hatDot3 = J_12or23 * f2 - J_33 *f3 - J_13or22 * f1;
    hatDot4 = J_14or21 * f1 + J_11or24 * f2;

    // Normalize the gradient
    norm = sqrt(hatDot1 * hatDot1 + hatDot2 * hatDot2 + hatDot3 * hatDot3 + hatDot4 * hatDot4);
    hatDot1 /= norm;
    hatDot2 /= norm;
    hatDot3 /= norm;
    hatDot4 /= norm;

    // Compute estimated gyroscope biases
    gerrx = _2q1 * hatDot2 - _2q2 * hatDot1 - _2q3 * hatDot4 + _2q4 * hatDot3;
    gerry = _2q1 * hatDot3 + _2q2 * hatDot4 - _2q3 * hatDot1 - _2q4 * hatDot2;
    gerrz = _2q1 * hatDot4 - _2q2 * hatDot3 + _2q3 * hatDot2 - _2q4 * hatDot1;

    // Compute and remove gyroscope biases
    gbiasx += gerrx * delta_t_ * zeta_;
    gbiasy += gerry * delta_t_ * zeta_;
    gbiasz += gerrz * delta_t_ * zeta_;
    //gx -= gbiasx;
    //gy -= gbiasy;
    //gz -= gbiasz;

    // Compute the quaternion derivative
    qDot1 = -_halfq2 * gx_rad - _halfq3 * gy_rad - _halfq4 * gz_rad;
    qDot2 =  _halfq1 * gx_rad + _halfq3 * gz_rad - _halfq4 * gy_rad;
    qDot3 =  _halfq1 * gy_rad - _halfq2 * gz_rad + _halfq4 * gx_rad;
    qDot4 =  _halfq1 * gz_rad + _halfq2 * gy_rad - _halfq3 * gx_rad;

    // Compute then integrate estimated quaternion derivative
    q1 += (qDot1 -(beta_ * hatDot1)) * delta_t_;
    q2 += (qDot2 -(beta_ * hatDot2)) * delta_t_;
    q3 += (qDot3 -(beta_ * hatDot3)) * delta_t_;
    q4 += (qDot4 -(beta_ * hatDot4)) * delta_t_;

    // Normalize the quaternion
    norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
    norm = 1.0f/norm;
    q[0] = q1 * norm;
    q[1] = q2 * norm;
    q[2] = q3 * norm;
    q[3] = q4 * norm;
}

void MPU6050::writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
   char data_write[2];
   data_write[0] = subAddress;
   data_write[1] = data;
   i2c_.write(address, data_write, 2, 0);
}

char MPU6050::readByte(uint8_t address, uint8_t subAddress)
{
    char data[1]; // `data` will store the register data
    char data_write[1];
    data_write[0] = subAddress;
    i2c_.write(address, data_write, 1, 1); // no stop
    i2c_.read(address, data, 1, 0);
    return data[0];
}

void MPU6050::readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{
    char data[14];
    char data_write[1];
    data_write[0] = subAddress;
    i2c_.write(address, data_write, 1, 1); // no stop
    i2c_.read(address, data, count, 0);
    for(int ii = 0; ii < count; ii++) {
     dest[ii] = data[ii];
    }
}

