#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <net/if.h>
#include <arpa/inet.h>
#include <sys/ioctl.h>

int main() {
    int sockfd;
    struct ifreq ifr;
    char interface_name[] = "eth0"; // Replace with your network interface name

    // Create a socket
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) == -1) {
        perror("socket");
        exit(EXIT_FAILURE);
    }

    // Set up the ifreq struct for interface status check
    memset(&ifr, 0, sizeof(struct ifreq));
    strncpy(ifr.ifr_name, interface_name, IFNAMSIZ - 1);

    // Check if the interface is up
    if (ioctl(sockfd, SIOCGIFFLAGS, &ifr) == -1) {
        perror("ioctl");
        close(sockfd);
        exit(EXIT_FAILURE);
    }

    if (ifr.ifr_flags & IFF_UP) {
        printf("Interface %s is up and running.\n", interface_name);
    } else {
        printf("Interface %s is down.\n", interface_name);
    }

    close(sockfd);
    return 0;
}
