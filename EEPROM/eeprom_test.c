#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <string.h>

int main(void)
{
    int file;
    char filename[] = "/dev/i2c-2";  // Replace X with the actual bus number
    int device_addr = 0x50;         // Replace with EEPROM's address
    uint8_t data[512];

    memset(&data, 0, sizeof(data));

    file = open(filename, O_RDWR);
    if (file < 0) {
        perror("Failed to open the I2C bus");
        return 1;
    }

    if (ioctl(file, I2C_SLAVE, device_addr) < 0) {
        perror("Failed to set I2C address");
        return 1;
    }

    // Read from the EEPROM
    if (read(file, data, sizeof(data)) != sizeof(data)) {
        perror("Failed to read from the EEPROM");
        close(file);
        return 1;
    }

    // Process the data (e.g., print it)
    for (int i = 0; i < sizeof(data); i++) {
        printf("Data at address %d: 0x%02X\n", i, data[i]);
    }

    // Optionally, write data to the EEPROM
    /*
    memset(&data, 1, sizeof(data));
    if (write(file, data, sizeof(data)) != sizeof(data)) {
        perror("Failed to write to the EEPROM");
        close(file);
        return 1;
    }
    */

    close(file);
    return 0;
}
