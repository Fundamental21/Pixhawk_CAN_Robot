#include <stdio.h>
#include <stdint.h>

int main() {
    // 使用与你的代码相同的union结构
    union {
        float f;
        uint8_t bytes[4];
    } position_union;
    
    // 将45.1赋值给float
    position_union.f = 45.1f;
    
    printf("Float value: %.6f\n", position_union.f);
    printf("4-byte representation (little-endian):\n");
    printf("Byte 0 (LSB): 0x%02X (%d)\n", position_union.bytes[0], position_union.bytes[0]);
    printf("Byte 1:       0x%02X (%d)\n", position_union.bytes[1], position_union.bytes[1]);
    printf("Byte 2:       0x%02X (%d)\n", position_union.bytes[2], position_union.bytes[2]);
    printf("Byte 3 (MSB): 0x%02X (%d)\n", position_union.bytes[3], position_union.bytes[3]);
    
    printf("\nCAN frame data would be:\n");
    printf("data[1] = 0x%02X  // Position byte 0 (LSB)\n", position_union.bytes[0]);
    printf("data[2] = 0x%02X  // Position byte 1\n", position_union.bytes[1]);
    printf("data[3] = 0x%02X  // Position byte 2\n", position_union.bytes[2]);
    printf("data[4] = 0x%02X  // Position byte 3 (MSB)\n", position_union.bytes[3]);
    
    printf("\nHex array: [0x%02X, 0x%02X, 0x%02X, 0x%02X]\n", 
           position_union.bytes[0], position_union.bytes[1], 
           position_union.bytes[2], position_union.bytes[3]);
    
    return 0;
} 