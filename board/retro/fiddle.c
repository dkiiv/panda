#include <stdio.h>

int packet;
int c1;
int c2;

c1 = ~0b01000000;
c2 = 0b00100000;

int main() {

    packet = 0b01010101;
    packet = packet & ~0b01000000;
    packet = packet | 0b00100000;
    //       00110101
    printf("%d\n", packet);
}

main();

