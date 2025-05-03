#include "ulp_riscv.h"
#include "ulp_riscv_utils.h"
#include "esp_attr.h"

volatile uint32_t data = 0;

void main(void) {
    while (1) {
        data += 1;
        
        // Wait before checking again (approx 100ms @ 8MHz)
        ulp_riscv_delay_cycles(8000000); // 8MHz * 0.1s = 800,000 cycles
    }
}