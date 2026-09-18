#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <stdbool.h>

#define I2C0_NODE DT_NODELABEL(mysensor)

#define BME680_CHIP_ID_REG 0xD0
#define BME680_CHIP_ID     0x61

LOG_MODULE_REGISTER(i2c_temp_read);  

static const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(I2C0_NODE);

#define CHECK_ERROR_READ(reg_addr, ret) \
        do { \
                if(ret != 0) \
                { \
                        LOG_ERR("Failed to read from I2C device address %x at reg. %x \n\r", \
                                dev_i2c.addr, reg_addr); \
                        return ret; \
                } \
        } while(0)
        
#define CHECK_ERROR_WRITE(reg_addr, ret) \
        do { \
                if(ret != 0) \
                { \
                        LOG_ERR("Failed to write from I2C device address %x to reg. %x \n\r", \
                                dev_i2c.addr, reg_addr); \
                        return ret; \
                } \
        } while(0)

static inline void set_byte(uint32_t* target, uint8_t byte, int8_t offset)
{
        uint32_t temp; 

        if(offset < 0) {
                temp = byte >> -offset; 
        } else {
                temp = byte << offset; 
        }

        *target |= temp; 
}
static int enable_temp_only(uint8_t oversampling)
{
        if (oversampling > 0b111) {
                oversampling = 1; 
        }

        uint8_t config[2] = {0x74, (oversampling << 5 | 1)}; 
        int ret = 0; 

        // 0x74 = ctrl_meas
        // Bits 7:5 = osrs_t (temp oversampling)
        // Bits 4:2 = osrs_p
        // Bits 1:0 = mode

        ret = i2c_write_dt(&dev_i2c, config, sizeof(config));
        CHECK_ERROR_WRITE(config[0], ret);

        return 0;  
}

static int get_temp_params(uint16_t* par_t1, uint16_t* par_t2, uint8_t* par_t3)
{
        int ret;
        uint8_t reg_addr; 
        uint8_t data_pool[2] = {0};
        *par_t1 = 0; *par_t2 = 0; *par_t3 = 0;
        
        reg_addr = 0xE9; 
        ret = i2c_burst_read_dt(&dev_i2c, reg_addr, data_pool, sizeof(data_pool));
        CHECK_ERROR_READ(reg_addr, ret); 

        *par_t1 = ( (uint16_t)data_pool[1] << 8 ) | (uint16_t)data_pool[0]; 

        reg_addr = 0x8A; 
        ret = i2c_burst_read_dt(&dev_i2c, reg_addr, data_pool, sizeof(data_pool));
        CHECK_ERROR_READ(reg_addr, ret);

        *par_t2 = ( (uint16_t)data_pool[1] << 8 ) | (uint16_t)data_pool[0]; 

        reg_addr = 0x8C;
        ret = i2c_reg_read_byte_dt(&dev_i2c, reg_addr, par_t3);
        CHECK_ERROR_READ(reg_addr, ret); 

        return 0;
}

static int check_for_new_data() 
{
        uint8_t reg_addr = 0x1D;
        uint8_t meas_status_0 = 0; 
        int ret = 0;

        do {
                ret = i2c_reg_read_byte_dt(&dev_i2c, reg_addr, &meas_status_0);
                CHECK_ERROR_READ(reg_addr, ret);

        } while (!(meas_status_0 & (1 << 7))); 

        return 0;
}

static int get_raw_adc(uint32_t* adc)
{
        // reg[0] = temp_msb[7:0]
        // reg[1] = temp_lsb[7:0]
        // reg[2] = temp_xlsb[7:4]

        uint8_t bytes[3] = {0};
        uint8_t regs[3] = {0x22, 0x23, 0x24}; 
        int ret;

        *adc = 0; 

        for (int i = 0; i < 3; i++) {
                uint8_t reg = regs[i];
                ret = i2c_reg_read_byte_dt(&dev_i2c, reg, &bytes[i]);
                CHECK_ERROR_READ(reg, ret);

                if (reg == 0x24) {
                        bytes[i] &= 0xF0; 
                }
        }

        set_byte(adc, bytes[0], 12); 
        set_byte(adc, bytes[1], 4);
        set_byte(adc, bytes[2], -4);

        return 0;
}

static inline int32_t raw_adc_to_celcius(uint32_t temp_adc, int16_t par_t1,
                                        int16_t par_t2, int8_t par_t3)
{
        int32_t var1 = ((int32_t)temp_adc >> 3) - ((int32_t)par_t1 << 1);
        int32_t var2 = (var1 * (int32_t)par_t2) >> 11;
        int32_t var3 = ((((var1 >> 1) * (var1 >> 1)) >> 12) * ((int32_t)par_t3 << 4)) >> 14;
        int32_t t_fine = var2 + var3;
        int32_t temp_comp = ((t_fine * 5) + 128) >> 8;

        return temp_comp;
}

static int check_chip_id(void)
{
        uint8_t chip_id;
        int ret;

        ret = i2c_reg_read_byte_dt(&dev_i2c, BME680_CHIP_ID_REG, &chip_id);
        CHECK_ERROR_READ(BME680_CHIP_ID_REG, ret);

        LOG_INF("Chip id: 0x%02x\n\r", chip_id);

        if (chip_id != BME680_CHIP_ID) {
                LOG_ERR("Unexpected chip id: 0x%02x (expected 0x%02x)\n\r",
                        chip_id, BME680_CHIP_ID);
                return -1;
        }

        return 0;
}

int main(void)
{
        uint16_t par_t1;
        uint16_t par_t2;
        uint8_t par_t3;
        uint32_t adc;
        uint8_t oversamp = 0b11; // 4x
        int32_t temp_read;
        int ret;

        /* waits for sensor to power on */
        k_msleep(10);


        if (!device_is_ready(dev_i2c.bus)) {
                LOG_ERR("I2C bus %s is not ready!\n\r",dev_i2c.bus->name);
                return 0;
        }

        ret = check_chip_id();
        if (ret != 0) {
                LOG_ERR("Chip id check failed: %d\n\r", ret);
                return 0;
        }

        LOG_INF("Começando medições...\n\r");
        while(1) {
                ret = enable_temp_only(oversamp);
                if (ret != 0) {
                        LOG_ERR("Could not enable temparature readings: %d\n\r", ret);
                        return 0;
                }

                ret = get_temp_params(&par_t1, &par_t2, &par_t3);
                if (ret != 0) {
                        LOG_ERR("Could not get temperature parameters: %d\n\r", ret);
                        return 0;
                }

                ret = check_for_new_data();
                if (ret != 0) {
                        LOG_ERR("New data check failed: %d\n\r", ret);
                        return 0;
                }

                LOG_INF("New data acquired\n\r");

                ret = get_raw_adc(&adc);
                if (ret != 0) {
                        LOG_ERR("Could not get raw adc data: %d\n\r", ret);
                        return 0;
                }

                temp_read = raw_adc_to_celcius(adc, par_t1,
                                                par_t2, par_t3);

                LOG_INF("Temperature: %0.2f C\n\r", temp_read / 100.0);

                k_msleep(1000);
        }

        return 0;
}
