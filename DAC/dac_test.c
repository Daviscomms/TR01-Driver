#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

#define MCP4725_ADDRESS             0x60
#define MCP4726_CMD_WRITEDACEEPROM  0x60
#define MCP4726_CMD_WRITEDAC        0x40

void mcp4725SetVoltage( int fd, uint16_t output, bool writeEEPROM )
{
    struct i2c_msg msg;
    struct i2c_rdwr_ioctl_data data;
    uint8_t buf[3];
    int ret;
    
    if (writeEEPROM)
    {
        buf[0] = MCP4726_CMD_WRITEDACEEPROM;
    }
    else
    {
        buf[0] = MCP4726_CMD_WRITEDAC;
    }
    buf[1] = (output / 16);
    buf[2] = (output % 16) << 4;

    msg.addr = (__u16)MCP4725_ADDRESS;
    msg.flags = 0;
    msg.len = 3;
    msg.buf = buf;

    data.msgs = &msg;
    data.nmsgs = 1;

    ret = ioctl(fd, I2C_RDWR, &data);
    if (ret < 0)
    {
        printf("Error: unable to write data\n");
    }
}

void mcp4725ReadConfig( int fd, uint8_t *status, uint16_t *value )
{
    struct i2c_msg msg;
    struct i2c_rdwr_ioctl_data data;
    uint8_t reg_value[3];
    int ret;

    msg.addr = (__u16)MCP4725_ADDRESS;
    msg.flags = I2C_M_RD;
    msg.len = 3;
    msg.buf = reg_value;

    data.msgs = &msg;
    data.nmsgs = 1;

    ret = ioctl(fd, I2C_RDWR, &data);
    if (ret < 0)
    {
        printf("ERROR: unable to read data\n");
        return;
    }

    *status = reg_value[0];
    *value = (reg_value[1] << 4) | (reg_value[2] >> 4);
}

int main(void)
{
    // Init /dev/i2c-2
    int fd = open("/dev/i2c-2", O_RDWR);
    if (fd < 0)
    {
        printf("ERROR: open(%d) failed.\n", fd);
    }

    // Set voltage
    mcp4725SetVoltage(fd, 0x05, false);

    // Read Configuration
    uint8_t status;
    uint16_t value;
    mcp4725ReadConfig(fd, &status, &value);
    printf("Status = %d, Value = %d\n", status, value);
    
    close(fd);
    
    return 0;
}
