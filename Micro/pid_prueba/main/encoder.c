#include <encoder.h>

#define MAV_SIZE    5

// Functions
void i2c_master_init(void);
void i2c_device_init(void);
esp_err_t i2c_master_read_register(uint8_t reg_addr, uint8_t *data);
static void shift_mav_filter();

// Handlers
i2c_master_bus_handle_t bus_handle;
i2c_master_dev_handle_t device_handle;

// Data
uint16_t mavBuffer[MAV_SIZE]={0};
float sumMAV=0;

void encoder_init(void){
    i2c_master_init();
    i2c_device_init();
}

void i2c_master_init(void){
    i2c_master_bus_config_t i2c_master_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = 1,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_master_config, &bus_handle));
}

void i2c_device_init(void){
    i2c_device_config_t i2c_device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = 0x36,
        .scl_speed_hz = 200000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &i2c_device_config, &device_handle));   
}

esp_err_t i2c_master_read_register(uint8_t reg_addr, uint8_t *data) {
    return i2c_master_transmit_receive(device_handle, &reg_addr, 1, data, 1, -1);
}

// Función para leer la posición del AS5600
int read_as5600_position(void) {
    uint8_t angle_high = 0, angle_low = 0;
    int angle = 0;
    int angle_degrees = 0;
    // Leer los registros de ángulo alto y bajo
    esp_err_t err_high = i2c_master_read_register(0x0C, &angle_high);
    esp_err_t err_low = i2c_master_read_register(0x0D, &angle_low);
    sumMAV = 0;
    if (err_high == ESP_OK && err_low == ESP_OK) {
        // Combina los dos bytes en un valor de 12 bits
        angle = ((angle_high << 8) | angle_low) & 0x0FFF;
        
        // Convierte a grados (0-360)
        angle_degrees = (angle * 360.0) / 4096.0;
        //mavBuffer[0] = angle;
        //for(int j=0; j < MAV_SIZE; j++){
        //    sumMAV += mavBuffer[j];
        //}
        //shift_mav_filter();
        //angle = sumMAV/(MAV_SIZE);
        ESP_LOGI("AS5600", "Position: %d degrees", angle_degrees);
    } else {
        ESP_LOGE("AS5600", "Failed to read position");
    }
    return angle_degrees;
}

static void shift_mav_filter(){
    for (int i = (MAV_SIZE-1); i > 0; i--) {
        mavBuffer[i] = mavBuffer[i-1];
    }
}