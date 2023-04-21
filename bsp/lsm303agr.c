#include "lsm303agr.h"
#include <math.h>

struct LSM303AGR_acc_data acc_data;
struct LSM303AGR_mag_data mag_data;

/**
  * Initialize the IMU sequentially in the following order
  *     1. initalize the i2c connection
  *     2. initialize the accelerometer @see accInit()
  *     3. initialize the magnetometer @see magInit()
  * @input:
  *         accel_mode      = @see lsm303_accel_mode
  *         accel_range     = @see lsm303_accel_range
  *         accel_data_rate = @see lsm303_accel_data_rate
  *         mag_power_mode  = @see lsm303_mag_power_modes
  *         mag_sys_mode    = @see lsm303_mag_sys_modes
  *         mag_dataRate    = @see lsm303_mag_data_rate
  * @output:
  *         None
*/
void IMUinit(int accel_mode, int accel_range, int accel_data_rate, 
             int mag_power_mode, int mag_sys_mode, int mag_dataRate){
    /* initialize the i2c bus*/
    i2c_init(SCL_PIN, SDA_PIN);
    
    /* Initialize the accelerometer*/
    accInit(accel_mode, accel_range, accel_data_rate);

    /* Initalize the magnetometer*/
    magInit(mag_power_mode, mag_sys_mode, mag_dataRate);

}

/**
  * Initialize the accelerometer by waking up the sensor using control 
  * registers and set up the appropriate modes for the required use case
  * @input:
  *         mode = @see lsm303_accel_mode
  *         range = @see lsm303_accel_range
  *         data_rate = @see lsm303_accel_data_rate
  * @output:
  *         None 
*/
void accInit(int mode, int range, int data_rate){
    acc_data.mode = mode;
    acc_data.range = range;
    acc_data.rate = data_rate;

    uint8_t ctrl = 0x00;
    uint8_t status_A = 0x00, status_FLAG=0x00;

    /* Filter mode normal*/
    COMPASSACCELERO_IO_Write(ACC_I2C_ADDRESS, LSM303AGR_CTRL_REG2_A, 0x00);
    delay_ms(1); // recommended

    /*Interrupts disables*/
    COMPASSACCELERO_IO_Write(ACC_I2C_ADDRESS, LSM303AGR_CTRL_REG3_A, 0x00);
    delay_ms(1); // recommended

    /*Power up the accelerometer and setup the data rate*/
    uint8_t mode_val_Lpen, mode_val_HR;

    switch (mode){
        case LSM303_ACCEL_MODE_LOW_POWER:
            mode_val_Lpen = 1;
            mode_val_HR = 0;
            break;
        case LSM303_ACCEL_MODE_NORMAL:
            mode_val_Lpen = 0;
            mode_val_HR = 0;
            break;
        case LSM303_ACCEL_MODE_HIGH_RESOLUTION:
            mode_val_Lpen = 0;
            mode_val_HR = 1;
            break;
        default:
            mode_val_Lpen = 0;
            mode_val_HR = 0;
            break;
    }

    /* mode setup & full range setup*/
    ctrl = (uint8_t) (0x01); // continuous update (0X00) / waith for both msb&LSB
    ctrl <<= 3;
    switch (range){
        case LSM303_ACCEL_RANGE_16G:
            ctrl |= (uint8_t) (LSM303_ACCEL_RANGE_16G);
            break;
        case LSM303_ACCEL_RANGE_8G:
            ctrl |= (uint8_t) (LSM303_ACCEL_RANGE_8G);
            break;
        case LSM303_ACCEL_RANGE_4G:
            ctrl |= (uint8_t) (LSM303_ACCEL_RANGE_4G);
            break;
        case LSM303_ACCEL_RANGE_2G:
            ctrl |= (uint8_t) (LSM303_ACCEL_RANGE_2G);
            break;
    }

    mode_val_HR = (uint8_t) (mode_val_HR << 3); // self test disabled and spi disabled
    ctrl = (ctrl << 4) | mode_val_HR;
    COMPASSACCELERO_IO_Write(ACC_I2C_ADDRESS, LSM303AGR_CTRL_REG4_A, ctrl);
    delay_ms(1);
    
    /* Enable IMU*/
    switch (data_rate){
            case LSM303_ACCEL_DATARATE_0HZ :
                ctrl = (uint8_t) (LSM303_ACCEL_DATARATE_0HZ);
                break;
            case LSM303_ACCEL_DATARATE_1HZ :
                ctrl = (uint8_t) (LSM303_ACCEL_DATARATE_1HZ);
                break;
            case LSM303_ACCEL_DATARATE_10HZ :
                ctrl = (uint8_t) (LSM303_ACCEL_DATARATE_10HZ);
                break;
            case LSM303_ACCEL_DATARATE_25HZ :
                ctrl = (uint8_t) (LSM303_ACCEL_DATARATE_25HZ);
                break;
            case LSM303_ACCEL_DATARATE_50HZ :
                ctrl = (uint8_t) (LSM303_ACCEL_DATARATE_50HZ);
                break;
            case LSM303_ACCEL_DATARATE_100HZ :
                ctrl = (uint8_t) (LSM303_ACCEL_DATARATE_100HZ);
                break;
            case LSM303_ACCEL_DATARATE_200HZ :
                ctrl = (uint8_t) (LSM303_ACCEL_DATARATE_200HZ);
                break;
            case LSM303_ACCEL_DATARATE_400HZ :
                ctrl = (uint8_t) (LSM303_ACCEL_DATARATE_400HZ);
                break;
            case LSM303_ACCEL_DATARATE_1600HZ :
                ctrl = (uint8_t) (LSM303_ACCEL_DATARATE_1600HZ);
                break;
            case LSM303_ACCEL_DATARATE_1300HZ :
                ctrl = (uint8_t) (LSM303_ACCEL_DATARATE_1300HZ);
                break;
    }

    mode_val_Lpen = (mode_val_Lpen << 3) | LSM303AGR_ACCEL_ENABLE;

    ctrl = (ctrl << 4) | mode_val_Lpen;

    COMPASSACCELERO_IO_Write(ACC_I2C_ADDRESS, LSM303AGR_CTRL_REG1_A, ctrl);

    delay_ms(1); // recommended
    
    /* Wait for ZYXDA BIT to be ready mode*/
    while (!status_FLAG) {
        status_A = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303AGR_STATUS_REG_A);
        status_A = status_A >> 3;
        status_FLAG = status_A & 0x01;
     delay_ms(10);
    }
    // printf("Accelerometer is ready!!!\n");
}

/**
 * Initialize the Magnetometer by waking up the sensor using control 
 * registers and set up the appropriate modes for the required use case
 * @input:
 *          power_mode = @see lsm303_mag_power_modes
 *          sys_mode = @see  lsm303_mag_sys_modes
 *          dataRate = @see  lsm303_mag_data_rate
 * @ouput:
 *          None
 */
void magInit(int power_mode, int sys_mode, int dataRate){
    uint8_t ctrl = 0x00; 
    uint8_t status_A = 0x00, status_FLAG=0x00;

    mag_data.dataRate = dataRate;
    mag_data.sys_mode = sys_mode;
    mag_data.power_mode = power_mode;

    /* Temperature compensation power mode, datarate, system mode*/
    ctrl = 0x08;
    switch (power_mode){
    case LSM303_MAG_HIGH_RESOLUTION_MODE:
        ctrl |= LSM303_MAG_HIGH_RESOLUTION_MODE;
        break;
    case LSM303_MAG_LOW_POWER_MODE:
        ctrl |= LSM303_MAG_LOW_POWER_MODE;
        break;
    }
    ctrl <<= 4; // temperature compensation and corresponding power mode
    uint8_t mode;

    switch (dataRate) {
    case LSM303_MAG_DATARATE_100HZ:
        mode = LSM303_MAG_DATARATE_100HZ;
        break;
    case LSM303_MAG_DATARATE_50HZ:
        mode = LSM303_MAG_DATARATE_50HZ;
        break;
    case LSM303_MAG_DATARATE_20HZ:
        mode = LSM303_MAG_DATARATE_20HZ;
        break;
    case LSM303_MAG_DATARATE_10HZ:
        mode = LSM303_MAG_DATARATE_10HZ;
        break;
    default:
        mode = LSM303_MAG_DATARATE_20HZ;
        break;
    }

    mode <<= 2; // datarate bit

    switch (sys_mode)
    {
    case LSM303_MAG_SYSMODE_CONTINUOUS:
        mode |= LSM303_MAG_SYSMODE_CONTINUOUS;
        break;
    case LSM303_MAG_SYSMODE_SINGLE:
        mode |= LSM303_MAG_SYSMODE_SINGLE;
        break;
    case LSM303_MAG_SYSMODE_IDLE_1:
        mode |= LSM303_MAG_SYSMODE_IDLE_1;
        break;
    case LSM303_MAG_SYSMODE_IDLE_2:
        mode |= LSM303_MAG_SYSMODE_IDLE_2;
        break;
    default:
        mode |= LSM303_MAG_SYSMODE_CONTINUOUS;
        break;
    }
    ctrl |= mode;
    COMPASSACCELERO_IO_Write(MAG_I2C_ADDRESS, LSM303AGR_CFG_REG_A_M, ctrl);
    delay_ms(10);

    /* Offset calibration enable */
    ctrl = 0x02;
    COMPASSACCELERO_IO_Write(MAG_I2C_ADDRESS, LSM303AGR_CFG_REG_B_M, ctrl);
    delay_ms(10);

    /* disable spi, select bdu to avoid incorrect data read*/
    ctrl = 0x10;
    COMPASSACCELERO_IO_Write(MAG_I2C_ADDRESS, LSM303AGR_CFG_REG_C_M, ctrl);
    delay_ms(10);

    while (!status_FLAG){
        status_A = COMPASSACCELERO_IO_Read(MAG_I2C_ADDRESS, LSM303AGR_STATUS_REG_M);
        status_A = status_A >> 3; 
        status_FLAG = status_A & 0x01; // reading zyxda bit
        delay_ms(10);
    }
    // printf("Magnetometer is Ready!!!\n");

}

/**
 * The function reads the values of the accelerometer and convert it into 
 * appropriate units (m/s^2)
 * 
 * Hint: values are passed by reference
 * @input: 
 *          data = float 3 x 1 array
 * @output:
 *          None
 */
void accReadXYZ(float* data){

    int16_t rawData[3];
    uint8_t ctrlX[2] = {0, 0};
    int16_t buffer[6];

    /* Read control registers */
    ctrlX[0] = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303AGR_CTRL_REG4_A);
    ctrlX[1] = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303AGR_CTRL_REG5_A);

    /* Read raw data*/
    buffer[0] = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303AGR_OUT_X_L_A); 
    buffer[1] = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303AGR_OUT_X_H_A);
    buffer[2] = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303AGR_OUT_Y_L_A);
    buffer[3] = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303AGR_OUT_Y_H_A);
    buffer[4] = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303AGR_OUT_Z_L_A);
    buffer[5] = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303AGR_OUT_Z_H_A);

    float lsb = getACCLSB();
    uint8_t shift = getACCShift();
    /* Check in the control register4 the data alignment*/
    if(!(ctrlX[0] & LSM303AGR_BLE_MSB)) {
        for(int i=0; i<3; i++){
            int16_t tmp = buffer[2*i+1];
            tmp <<= 8;
            tmp |= buffer[2*i];
            rawData[i] = tmp;
        }
    }
    else { /* Big Endian Mode */
        for(int i=0; i<3; i++) {
            int16_t tmp = buffer[2*i];
            tmp <<= 8;
            tmp |= buffer[2*i+1];
            rawData[i] = tmp;
        }
    }

    /* get the converted accelerometer data*/

    for (int i = 0; i < 3; i++){
        data[i] = (float) ((rawData[i] >> shift)*lsb*EARTH_GRAVITY);
        // data[i] = rawData[i];
    }

}

/**
 * The function get the accelerometer sensitivity of as per the data sheet
 * @input:
 *          None
 * @output:
 *          lsb = sensitivity
*/

float getACCLSB(){
    float lsb = 0.0;
    if (acc_data.mode == LSM303_ACCEL_MODE_NORMAL){
        switch (acc_data.range){
            case LSM303_ACCEL_RANGE_16G:
                lsb = 0.0469;
                break;
            case LSM303_ACCEL_RANGE_8G:
                lsb = 0.01563;
                break;
            case LSM303_ACCEL_RANGE_4G:
                lsb = 0.00782;
                break;
            case LSM303_ACCEL_RANGE_2G:
                lsb = 0.0039;
                break;
        }
    } else if (acc_data.mode == LSM303_ACCEL_MODE_LOW_POWER){
        switch (acc_data.range){
            case LSM303_ACCEL_RANGE_16G:
                lsb = 0.18758;
                break;
            case LSM303_ACCEL_RANGE_8G:
                lsb = 0.06252;
                break;
            case LSM303_ACCEL_RANGE_4G:
                lsb = 0.03126;
                break;
            case LSM303_ACCEL_RANGE_2G:
                lsb = 0.01563;
                break;
        }
    } else if (acc_data.mode == LSM303_ACCEL_MODE_HIGH_RESOLUTION){
        switch (acc_data.range){
            case LSM303_ACCEL_RANGE_16G:
                lsb = 0.01172;
                break;
            case LSM303_ACCEL_RANGE_8G:
                lsb = 0.0039;
                break;
            case LSM303_ACCEL_RANGE_4G:
                lsb = 0.00195;
                break;
            case LSM303_ACCEL_RANGE_2G:
                lsb = 0.00098;
                break;
        }
    }
    return lsb;
}


/**
 * The function get the amount of the shift required to compute the 
 * correct accelerometer values
 * @input:
 *          None
 * @output:
 *          shift = amount of shift
*/
uint8_t getACCShift(){
    uint8_t shift = 0;
    switch(acc_data.mode){
        case LSM303_ACCEL_MODE_NORMAL:
            shift = 6;
            break;
        case LSM303_ACCEL_MODE_LOW_POWER:
            shift = 8;
            break;
        case LSM303_ACCEL_MODE_HIGH_RESOLUTION:
            shift = 4;
            break;
    }
    return shift;
}

/**
 * The function reads the values of the magnetometer and convert it into 
 * appropriate units (mill Gauss)
 * 
 * Hint: values are passed by reference
 * @input: 
 *          data = float 3 x 1 array
 * @output:
 *          None
 */
void magReadXYZ(float* data){
    int16_t rawData[3];
    uint8_t ctrlx;
    int16_t buffer[6];
    float sensitivity = 1.50; // defined in datasheet

    /* Read the magnetometer control register content */
    ctrlx = COMPASSACCELERO_IO_Read(MAG_I2C_ADDRESS, LSM303AGR_CFG_REG_C_M);
    
    /* Read output register X, Y & Z magnetometer */
    buffer[0] = COMPASSACCELERO_IO_Read(MAG_I2C_ADDRESS, LSM303AGR_OUTX_L_REG_M); 
    buffer[1] = COMPASSACCELERO_IO_Read(MAG_I2C_ADDRESS, LSM303AGR_OUTX_H_REG_M);
    buffer[2] = COMPASSACCELERO_IO_Read(MAG_I2C_ADDRESS, LSM303AGR_OUTY_L_REG_M);
    buffer[3] = COMPASSACCELERO_IO_Read(MAG_I2C_ADDRESS, LSM303AGR_OUTY_H_REG_M);
    buffer[4] = COMPASSACCELERO_IO_Read(MAG_I2C_ADDRESS, LSM303AGR_OUTZ_L_REG_M);
    buffer[5] = COMPASSACCELERO_IO_Read(MAG_I2C_ADDRESS, LSM303AGR_OUTZ_H_REG_M);

    /* Check in the control register4 the data alignment*/
  
    if(!(ctrlx & LSM303AGR_BLE_MSB_MAG)) 
    {
        for(int i=0; i<3; i++){
          int16_t tmp = buffer[2*i+1]; 
          tmp <<= 8;
          tmp |= buffer[2*i];
          rawData[i]= tmp;
        }
    }
    else { /* Big Endian Mode */
        for(int i=0; i<3; i++){
          int16_t tmp = buffer[2*i];
          tmp <<= 8;
          tmp |= buffer[2*i+1];
          tmp >>= 6;
          rawData[i] = tmp;
        }
    }

    /* Normal mode */
    /* Switch the sensitivity value set in the CRTL4 */
    
    /* Obtain the mg value for the three axis */
    for(int i=0; i<3; i++) {
        float temp = (float) (rawData[i]);
        data[i]=( temp * sensitivity);
    }

}


/**
  * The function writes the values to given register address at device address
  * @input:
  *         DeviceAddr = i2c address of the device you are writing into
  *         RegisterAddr = address of the register you are writing into
  *         Value = values you are appending to the register
  * @output:        
  *         None
  */
void COMPASSACCELERO_IO_Write(uint16_t DeviceAddr, uint8_t RegisterAddr, uint8_t Value)
{
    i2c_write_reg(DeviceAddr, RegisterAddr, Value);
}

/**
  * The function reads the values to given register address at device address
  * @input:
  *         DeviceAddr = i2c address of the device you are reading into
  *         RegisterAddr = address of the register you are reading into
  * @output:        
  *         Value = values you are getting from the register
  */
uint8_t COMPASSACCELERO_IO_Read(uint16_t DeviceAddr, uint8_t RegisterAddr)
{
    return i2c_read_reg(DeviceAddr, RegisterAddr);
}

// private tests
void accSelfTest(){
    /**
     * initialize sensor, turn on, enable x/y/z
    */
    printf("Started self test\n");
    uint8_t ctrl = 0x00;
    COMPASSACCELERO_IO_Write(ACC_I2C_ADDRESS, LSM303AGR_CTRL_REG2_A, ctrl);
    printf("Started self test-> reg1\n");
    delay_ms(1);
    COMPASSACCELERO_IO_Write(ACC_I2C_ADDRESS, LSM303AGR_CTRL_REG3_A, ctrl);
    delay_ms(1);
    ctrl = 0x81;
    printf("Started self test-> reg2\n");
    COMPASSACCELERO_IO_Write(ACC_I2C_ADDRESS, LSM303AGR_CTRL_REG4_A, ctrl);
    delay_ms(1);
    ctrl = 0x57;
    printf("Started self test-> reg3\n");
    COMPASSACCELERO_IO_Write(ACC_I2C_ADDRESS, LSM303AGR_CTRL_REG1_A, ctrl);

    /**
     * power up and wait for 90ms for stable output
    */
    delay_ms(90);
    printf("Powered on\n");
    // read zyxda bit in status register
    uint8_t status_A =  (0x00);
    uint8_t STATUS_FLAG =  (0x00);
    status_A = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS,LSM303AGR_STATUS_REG_A);
    STATUS_FLAG = status_A >> 3;
    printf("--status_A = %d",status_A);
    printf(" --status_Flag = %d\n",STATUS_FLAG);


    // read data dn discard
    int16_t buffer[6];

    buffer[0] = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303AGR_OUT_X_L_A); 
    buffer[1] = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303AGR_OUT_X_H_A);
    buffer[2] = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303AGR_OUT_Y_L_A);
    buffer[3] = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303AGR_OUT_Y_H_A);
    buffer[4] = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303AGR_OUT_Z_L_A);
    buffer[5] = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303AGR_OUT_Z_H_A);

    // read and discard
    status_A = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS,LSM303AGR_STATUS_REG_A);
    STATUS_FLAG = status_A >> 3;
    printf("read&discard --status_A = %d",status_A);
    printf(" --status_Flag = %d\n",STATUS_FLAG);

    uint8_t ctrlx[2] = {0,0};
    ctrlx[0] = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303AGR_CTRL_REG4_A);
    ctrlx[1] = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303AGR_CTRL_REG5_A);


    // read output registers for five times
    int16_t OUT_NOST[3] = {0, 0, 0};
    STATUS_FLAG = 0x00;
    printf("----no ST----\n");
    for (int iter = 5; iter > 0 ; iter--){
        while (! STATUS_FLAG){
            status_A = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS,LSM303AGR_STATUS_REG_A);
            STATUS_FLAG = status_A >> 3;
            printf("--status_A = %d",status_A);
            printf(" --status_Flag = %d\n",STATUS_FLAG);
        }
        buffer[0] = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303AGR_OUT_X_L_A); 
        buffer[1] = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303AGR_OUT_X_H_A);
        buffer[2] = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303AGR_OUT_Y_L_A);
        buffer[3] = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303AGR_OUT_Y_H_A);
        buffer[4] = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303AGR_OUT_Z_L_A);
        buffer[5] = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303AGR_OUT_Z_H_A);
        if(!(ctrlx[0] & LSM303AGR_BLE_MSB)) {
            for(int i=0; i<3; i++) {
                OUT_NOST[i]+=((int16_t)((uint16_t)buffer[2*i+1] << 8) + buffer[2*i]);
            }
        }
        else {
            for(int i=0; i<3; i++){
                OUT_NOST[i]+=((int16_t)((uint16_t)buffer[2*i] << 8) + buffer[2*i+1]);
            }
        }
        STATUS_FLAG = 0x00;
    }
    // average the value
    printf("---avg-value--\n");
    for (int i = 0; i < 3; i++){
        OUT_NOST[i] /= 5;
        printf("OUT_NOST[%d] = %d ",i, OUT_NOST[i]);
    }
    printf("\n");

    // enable self-test
    printf("----self test enabled\n");
    ctrl = 0x85;
    COMPASSACCELERO_IO_Write(ACC_I2C_ADDRESS, LSM303AGR_CTRL_REG4_A, ctrl);

    // same as above
    delay_ms(90);
    status_A = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS,LSM303AGR_STATUS_REG_A);
    STATUS_FLAG = status_A >> 3;
    printf("status_A = %d",status_A);
    printf(" status_Flag = %d\n",STATUS_FLAG);

    buffer[0] = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303AGR_OUT_X_L_A); 
    buffer[1] = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303AGR_OUT_X_H_A);
    buffer[2] = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303AGR_OUT_Y_L_A);
    buffer[3] = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303AGR_OUT_Y_H_A);
    buffer[4] = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303AGR_OUT_Z_L_A);
    buffer[5] = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303AGR_OUT_Z_H_A);

    status_A = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS,LSM303AGR_STATUS_REG_A);
    STATUS_FLAG = status_A >> 3;
    printf("status_A = %d",status_A);
    printf(" status_Flag = %d\n",STATUS_FLAG);


    // read for 5 times
    int16_t OUT_ST[3] = {0, 0, 0};
    STATUS_FLAG = 0x00;
    printf("---read five times ----\n");
    for (int iter = 5; iter > 0 ; iter--){
        while (! STATUS_FLAG){
            status_A = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS,LSM303AGR_STATUS_REG_A);
            STATUS_FLAG = status_A >> 3;
            printf("status_A = %d",status_A);
            printf(" status_Flag = %d\n",STATUS_FLAG);
        }
        
        buffer[0] = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303AGR_OUT_X_L_A); 
        buffer[1] = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303AGR_OUT_X_H_A);
        buffer[2] = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303AGR_OUT_Y_L_A);
        buffer[3] = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303AGR_OUT_Y_H_A);
        buffer[4] = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303AGR_OUT_Z_L_A);
        buffer[5] = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303AGR_OUT_Z_H_A);
        if(!(ctrlx[0] & LSM303AGR_BLE_MSB)) {
            for(int i=0; i<3; i++) {
                OUT_ST[i]+=((int16_t)((uint16_t)buffer[2*i+1] << 8) + buffer[2*i]);
            }
        }
        else /* Big Endian Mode */{
            for(int i=0; i<3; i++){
                OUT_ST[i]+=((int16_t)((uint16_t)buffer[2*i] << 8) + buffer[2*i+1]);
            }
        }
        STATUS_FLAG = 0x00;
    }

    // average it out
    printf("---avg---\n");
    for (int i = 0; i < 3; i++){
        OUT_ST[i] /= 5;
        printf("OUT_ST[%d] = %d", i, OUT_ST[i]);
    }
    printf("\n");

    // find the absolute
    int16_t abs_val[3];
    printf("abs values = \n");
    for (int i = 0; i < 3; i++) {
        abs_val[i] = fabs(OUT_NOST[i] - OUT_ST[i]);
        printf("ABS[%d] = %d ", i, abs_val[i]);
    }
    printf("\n");

}

