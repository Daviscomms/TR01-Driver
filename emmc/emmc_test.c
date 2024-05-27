#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>

int main(void)
{
    // Device file for emmc
    int fd = open("/dev/mmcblk2p1", O_RDWR);
    if (fd == -1)
    {
        perror("Failed to open emmc.\n");
        return -1;
    }

    // Create the directory /mnt/emmc/ if it doesn't exist
    if (mkdir("/mnt/emmc", 0755) == -1)
    {
        perror("Failed to create directory /mnt/emmc/ .\n");
        close(fd);
        return -1;
    }

    // Open or Create a file within the filesystem
    int file_fd = open("/mnt/emmc/test.txt", O_CREAT | O_RDWR, 0644);
    if (file_fd == -1)
    {
        perror("Failed to open or create a file in emmc filesystem.\n");
        close(fd);
        return -1;
    }

    // Perform write to emmc
    char data[] = "Test emmc";
    ssize_t bytesWritten  = write(file_fd, data, sizeof(data));
    if (bytesWritten == -1)
    {
        perror("Failed to write data into emmc.\n");
        close(file_fd);
        close(fd);
        return -1;
    }

    // Perform read from emmc
    char buffer[1024];
    lseek(file_fd, 0, SEEK_SET);
    ssize_t bytesRead = read(file_fd, buffer, sizeof(buffer));
    if (bytesRead == -1)
    {
        perror("Failed to read data from emmc.\n");
        close(file_fd);
        close(fd);
        return -1;
    }
    else
    {
        printf("data read from emmc = %s\n", buffer);
    }

    close(file_fd);
    close(fd);
    return 0;
}