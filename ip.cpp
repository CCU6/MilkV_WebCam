#include <iostream>
#include <cstdio>
#include <sys/types.h>
#include <ifaddrs.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <string.h>
int main() {
    struct ifaddrs *interfaces = nullptr;
    struct ifaddrs *addr = nullptr;
    int result = getifaddrs(&interfaces);
    if (result == 0) {
        for (addr = interfaces; addr != nullptr; addr = addr->ifa_next) {
            if (addr->ifa_addr && addr->ifa_addr->sa_family == AF_INET && strcmp(addr->ifa_name,"eth0") == 0) { // IPv4
                struct sockaddr_in *ipAddr = reinterpret_cast<struct sockaddr_in *>(addr->ifa_addr);
                printf("%3d.%3d.%3d.%3d\n",  inet_ntoa(ipAddr->sin_addr));
            }
        }
        freeifaddrs(interfaces);
    } else {
        std::cerr << "getifaddrs failed." << std::endl;
    }
    return 0;
}