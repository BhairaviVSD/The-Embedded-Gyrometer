




/*
 *
 * Project: The Embedded Gyrometer – The Need for Speed
 * 
 * Description: This project aims to create a wearable speedometer 
 * that calculates velocity using a gyroscopic sensor (L3GD20), without GPS.
 * It measures 3-axis angular velocities, converts them to linear forward velocity, 
 * and calculates the distance traveled solely using the gyroscope data.
 * 
 * Hardware: STM32F429 Discovery Board, L3GD20 Gyroscope
 * 
 * Software: PlatformIO environment
 * 
 * Key Features: 
 *  - Interfacing with STM32F429 and L3GD20
 *  - Sampling angular velocities every 0.5 seconds
 *  - Recording 20 seconds of data for distance calculations on input digital switch
 *  - Displaying Distance travelled on the DISCO_F429ZI LCD board 
 * 
 *  Project Group: Team 45
 *  - Bhairavi Sawantdesai: bvs9764
 *  - Pranay Gupta: pvg2013
 *  - Divya Agarwal: da3257  
 * 
 */





#include "mbed.h"
#include "drivers/LCD_DISCO_F429ZI.h"

#define OUT_X_L 0x28

// Register fields(bits): data_rate(2), Bandwidth(2), Power_down(1), Zen(1), Yen(1), Xen(1)
#define CTRL_REG1 0x20
// Configuration: 200Hz ODR,50Hz cutoff, Power on, Z on, Y on, X on
#define CTRL_REG1_CONFIG 0b01'10'1'1'1'1

// Register fields(bits): reserved(1), endian-ness(1), Full scale sel(2), reserved(1), self-test(2), SPI mode(1)
#define CTRL_REG4 0x23
// Configuration: reserved, little endian, 500 dps, reserved, disabled, 4-wire mode
#define CTRL_REG4_CONFIG 0b0'0'01'0'00'0

// Register fields(bits): I1_Int1 (1), I1_Boot(1), H_Lactive(1), PP_OD(1), I2_DRDY(1), I2_WTM(1), I2_ORun(1), I2_Empty(1)
#define CTRL_REG3 0x22
// Configuration: Int1 disabled, Boot status disabled, active high interrupts, push-pull, enable Int2 data ready, disable fifo interrupts                 
#define CTRL_REG3_CONFIG 0b0'0'0'0'1'000

#define SPI_FLAG 1
#define DATA_READY_FLAG 2

#define SCALING_FACTOR (17.5f * 0.017453292519943295769236907684886f / 1000.0f)     // π/180 = 0.017453292519943295769236907684886

// Smoothing factor for Low Pass Filter
#define FILTER_COEFFICIENT 0.1f 

// Measure digital input of button
DigitalIn inp(PA_0); 
int val;

// Interrupt Initialization
InterruptIn int2(PA_2, PullDown);

// SPI Initialization 
SPI spi(PF_9, PF_8, PF_7, PC_1, use_gpio_ssel); // spi(mosi, miso, sclk, cs)
uint8_t write_buf[32];
uint8_t read_buf[32];

// EventFlags object declaration
EventFlags flags;

// LCD
LCD_DISCO_F429ZI lcd;

float gx , gy , gz ; 
float angular_velocities[40];
float filtered_av[40];
float linear_velocities[40];

void convertToRadians(float* angular_velocities);
void lowPassFilter(float* angular_velocities, float* filtered_av);
void convertToLinearVelocity(float leg_length, float* filtered_av, float* linear_velocities);
float calcuateDistance(float* linear_velocity);

// SPI callback function
void spi_cb(int event) {
    flags.set(SPI_FLAG);
}

// Data Ready callback function
void data_cb() {
    flags.set(DATA_READY_FLAG);
}

void convertToRadians(float* angularVelocity){
    int16_t raw_gx, raw_gy, raw_gz;
    float gx, gy, gz;

    for (int i = 0; i < 40; ++i) {

        // Get raw data
        flags.wait_all(DATA_READY_FLAG);
        write_buf[0] = OUT_X_L | 0x80 | 0x40;

        spi.transfer(write_buf, 7, read_buf, 7, spi_cb);
        flags.wait_all(SPI_FLAG);

        // Process raw data
        raw_gx = (((uint16_t)read_buf[2]) << 8) | ((uint16_t)read_buf[1]);
        raw_gy = (((uint16_t)read_buf[4]) << 8) | ((uint16_t)read_buf[3]);
        raw_gz = (((uint16_t)read_buf[6]) << 8) | ((uint16_t)read_buf[5]);

        // Multiplying by Scaling factor to convert raw values 
        gx = ((float)raw_gx) * SCALING_FACTOR;
        gy = ((float)raw_gy) * SCALING_FACTOR;
        gz = ((float)raw_gz) * SCALING_FACTOR;

        // Combined angular velocity
        float combinedAngVelocity = sqrt(gx * gx + gy * gy + gz * gz);
        angularVelocity[i] = combinedAngVelocity;

        printf(">av_raw:%4.5f|g\n", angularVelocity[i]);

        // Delay of 0.5 sec for the next data collection cycle
        ThisThread::sleep_for(500ms);
    }
}

void lowPassFilter(float* angularVelocity , float* filtered_av){
    float filtered_av_init = 0.0f;
    for (int i = 0; i < 40; ++i) {
        // Low Pass Filter 
        // y[n]=αx[n]+(1−α)y[n−1]
        // y[n] is the output at time n
        // x[n] is the input at time 
        // α is the smoothing factor, related to the cutoff frequency
        filtered_av_init= FILTER_COEFFICIENT * angularVelocity[i] + (1 - FILTER_COEFFICIENT) * filtered_av_init;
        filtered_av[i] = filtered_av_init;
        printf(">av_lowPassFilter: %4.5f|g\n", filtered_av[i]);
    }
}

void convertToLinearVelocity(float leg_length, float* filtered_av, float* linear_velocities) {
    for (int i = 0; i < 40; ++i) {
        // linear velocity = radius of the circular path * angular velocity (in radians per second)
        linear_velocities[i] = leg_length * filtered_av[i];
        printf(">lv: %4.5f|g\n", linear_velocities[i]);
    }
}

float calcuateDistance(float* linear_velocity){
  int size = 40;
  float time_interval = 0.5;
  float distance = 0.0;
    for (int i = 0; i < size; ++i) {
        // Assuming constant linear velocity during the time interval
        distance += (linear_velocity[i] * time_interval);
        printf(">distance: %4.5f|g\n", distance);
    }
    printf(">Total distance covered: %4.5f|g cms \n", distance);
    return distance;
}

int main(){
    // Setting up SPI Interface
    int2.rise(&data_cb);

    // SPI format and frequency
    spi.format(8, 3);
    spi.frequency(1'000'000);

    // Write to control registers for SPI transfer
    write_buf[0] = CTRL_REG1;
    write_buf[1] = CTRL_REG1_CONFIG;
    spi.transfer(write_buf, 2, read_buf, 2, spi_cb);
    flags.wait_all(SPI_FLAG);

    write_buf[0] = CTRL_REG4;
    write_buf[1] = CTRL_REG4_CONFIG;
    spi.transfer(write_buf, 2, read_buf, 2, spi_cb);
    flags.wait_all(SPI_FLAG);

    write_buf[0] = CTRL_REG3;
    write_buf[1] = CTRL_REG3_CONFIG;
    spi.transfer(write_buf, 2, read_buf, 2, spi_cb);
    flags.wait_all(SPI_FLAG);
    // SPI Setup Completed

    write_buf[1] = 0xFF;

    // (Polling for/Setting) the data ready flag
    if (!(flags.get() & DATA_READY_FLAG) && (int2.read() == 1)) {
        flags.set(DATA_READY_FLAG);
    }

    while(1){
        val = inp.read();
        // Check for input of digital button
        if(val == 1){
            // Getting raw angular velocity in x, y and z directions and converting to combined angular velocity in radians/sec
            convertToRadians(angular_velocities);
            // Using a Low Pass Filter on combined angular velocity to smooth out high-frequency components and introduce a small delay
            lowPassFilter(angular_velocities, filtered_av);
            // Converting combined angular velocity to linear velocity  
            convertToLinearVelocity(50.0f , filtered_av ,linear_velocities); // leg_length = 50 cm
            // Calculating Total Distance covered
            float total_distance = calcuateDistance(linear_velocities); // distance in cms
            total_distance = total_distance/100; // distance in metres
            
            // Display the final distance covered on the LCD screen
            char distanceString[20]; 
            snprintf(distanceString, sizeof(distanceString), "%.2f m", total_distance);
            lcd.DisplayStringAt(0, 50, (uint8_t *)"Total Distance = ", LEFT_MODE);
            lcd.DisplayStringAt(0, 70, (uint8_t *)distanceString, LEFT_MODE);
        }
    }
    return 0;
}
