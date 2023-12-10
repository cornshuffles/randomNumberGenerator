/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the Empty PSoC4  Application
*              for ModusToolbox.
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2020-2021, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

#include "cy_pdl.h"
#include "cybsp.h"

#define DELAY 1    // MS delay after driving a sseg

typedef struct {
	GPIO_PRT_Type *A_port;
	uint32_t A_pin;
	GPIO_PRT_Type *B_port;
	uint32_t B_pin;
	GPIO_PRT_Type *C_port;
	uint32_t C_pin;
	GPIO_PRT_Type *D_port;
	uint32_t D_pin;
	GPIO_PRT_Type *E_port;
	uint32_t E_pin;
	GPIO_PRT_Type *F_port;
	uint32_t F_pin;
	GPIO_PRT_Type *G_port;
	uint32_t G_pin;
} sseg;

sseg rand_sseg = {SSEG_A_PORT, SSEG_A_PIN,  SSEG_B_PORT, SSEG_B_PIN,  SSEG_C_PORT, SSEG_C_PIN,  SSEG_D_PORT,
				  SSEG_D_PIN,  SSEG_E_PORT, SSEG_E_PIN,  SSEG_F_PORT, SSEG_F_PIN,  SSEG_G_PORT, SSEG_G_PIN};

cy_stc_gpio_prt_config_t off_port_config = {
.dr = 0x000000ffu,           // All segs off
.intrCfg = 0x00000000u,      // No Interrupts
.pc = 0x001b6db6u,           // Seg A-G strong drive
.pc2 = 0x000000ffu,          // Disable input buffers
.sio = 0x00000000u,          // Reserved
.selActive = 0x00000000u,    // HSIOM - software controlled
};

cy_stc_gpio_prt_config_t zero_port_config = {
.dr = 0x00000040u,           // Seg A,B,C,D,E,F=0
.intrCfg = 0x00000000u,      // No Interrupts
.pc = 0x001b6db6u,           // Seg A-G strong drive
.pc2 = 0x000000ffu,          // Disable input buffers
.sio = 0x00000000u,          // Reserved
.selActive = 0x00000000u,    // HSIOM - software controlled
};

cy_stc_gpio_prt_config_t one_port_config = {
.dr = 0x00000079u,           // Seg B,C=0
.intrCfg = 0x00000000u,      // No Interrupts
.pc = 0x001b6db6u,           // Seg A-G strong drive
.pc2 = 0x000000ffu,          // Disable input buffers
.sio = 0x00000000u,          // Reserved
.selActive = 0x00000000u,    // HSIOM - software controlled
};

cy_stc_gpio_prt_config_t two_port_config = {
.dr = 0x000000a4u,           // Seg A,B,D,E,G=0
.intrCfg = 0x00000000u,      // No Interrupts
.pc = 0x001b6db6u,           // Seg A-G strong drive
.pc2 = 0x000000ffu,          // Disable input buffers
.sio = 0x00000000u,          // Reserved
.selActive = 0x00000000u,    // HSIOM - software controlled
};

cy_stc_gpio_prt_config_t three_port_config = {
.dr = 0x00000030u,           // Seg A,B,C,D,G=0
.intrCfg = 0x00000000u,      // No Interrupts
.pc = 0x001b6db6u,           // Seg A-G strong drive
.pc2 = 0x000000ffu,          // Disable input buffers
.sio = 0x00000000u,          // Reserved
.selActive = 0x00000000u,    // HSIOM - software controlled
};

cy_stc_gpio_prt_config_t four_port_config = {
.dr = 0x00000019u,           // Seg B,C,F,G=0
.intrCfg = 0x00000000u,      // No Interrupts
.pc = 0x001b6db6u,           // Seg A-G strong drive
.pc2 = 0x000000ffu,          // Disable input buffers
.sio = 0x00000000u,          // Reserved
.selActive = 0x00000000u,    // HSIOM - software controlled
};

cy_stc_gpio_prt_config_t five_port_config = {
.dr = 0x00000012u,           // Seg A,C,D,F,G=0
.intrCfg = 0x00000000u,      // No Interrupts
.pc = 0x001b6db6u,           // Seg A-G strong drive
.pc2 = 0x000000ffu,          // Disable input buffers
.sio = 0x00000000u,          // Reserved
.selActive = 0x00000000u,    // HSIOM - software controlled
};

cy_stc_gpio_prt_config_t six_port_config = {
.dr = 0x00000002u,           // Seg A,C,D,E,F,G=0
.intrCfg = 0x00000000u,      // No Interrupts
.pc = 0x001b6db6u,           // Seg A-G strong drive
.pc2 = 0x000000ffu,          // Disable input buffers
.sio = 0x00000000u,          // Reserved
.selActive = 0x00000000u,    // HSIOM - software controlled
};

cy_stc_gpio_prt_config_t seven_port_config = {
.dr = 0x00000078u,           // Seg A,C,D,E,F,G=0
.intrCfg = 0x00000000u,      // No Interrupts
.pc = 0x001b6db6u,           // Seg A-G strong drive
.pc2 = 0x000000ffu,          // Disable input buffers
.sio = 0x00000000u,          // Reserved
.selActive = 0x00000000u,    // HSIOM - software controlled
};

cy_stc_gpio_prt_config_t eight_port_config = {
.dr = 0x00000000u,           // Seg A,B,C,D,E,F,G=0
.intrCfg = 0x00000000u,      // No Interrupts
.pc = 0x001b6db6u,           // Seg A-G strong drive
.pc2 = 0x000000ffu,          // Disable input buffers
.sio = 0x00000000u,          // Reserved
.selActive = 0x00000000u,    // HSIOM - software controlled
};

cy_stc_gpio_prt_config_t nine_port_config = {
.dr = 0x00000018u,           // Seg A,B,C,F,G=0
.intrCfg = 0x00000000u,      // No Interrupts
.pc = 0x001b6db6u,           // Seg A-G strong drive
.pc2 = 0x000000ffu,          // Disable input buffers
.sio = 0x00000000u,          // Reserved
.selActive = 0x00000000u,    // HSIOM - software controlled
};

static inline void driveSSEG(sseg *SSEG, uint32_t num) {
	switch(num) {
		case 0: Cy_GPIO_Port_Init(SSEG->A_port, &zero_port_config); break;
		case 1: Cy_GPIO_Port_Init(SSEG->A_port, &one_port_config); break;
		case 2: Cy_GPIO_Port_Init(SSEG->A_port, &two_port_config); break;
		case 3: Cy_GPIO_Port_Init(SSEG->A_port, &three_port_config); break;
		case 4: Cy_GPIO_Port_Init(SSEG->A_port, &four_port_config); break;
		case 5: Cy_GPIO_Port_Init(SSEG->A_port, &five_port_config); break;
		case 6: Cy_GPIO_Port_Init(SSEG->A_port, &six_port_config); break;
		case 7: Cy_GPIO_Port_Init(SSEG->A_port, &seven_port_config); break;
		case 8: Cy_GPIO_Port_Init(SSEG->A_port, &eight_port_config); break;
		case 9: Cy_GPIO_Port_Init(SSEG->A_port, &nine_port_config); break;
		default: Cy_GPIO_Port_Init(SSEG->A_port, &off_port_config); break;
	}
}

volatile uint32 randNo = 0;
volatile uint32 randNo_huns = 10;
volatile uint32 randNo_tens = 10;
volatile uint32 randNo_ones = 10;

static inline void drawNum(sseg *a_sseg) {
	// Write hundreds
	Cy_GPIO_Write(SSEG_ONES_PORT, SSEG_ONES_PIN, 0);
	driveSSEG(a_sseg, randNo_huns);
	Cy_GPIO_Write(SSEG_HUNS_PORT, SSEG_HUNS_PIN, 1);
	Cy_SysLib_Delay(DELAY);

	// Write tens
	Cy_GPIO_Write(SSEG_HUNS_PORT, SSEG_HUNS_PIN, 0);
	driveSSEG(a_sseg, randNo_tens);
	Cy_GPIO_Write(SSEG_TENS_PORT, SSEG_TENS_PIN, 1);
	Cy_SysLib_Delay(DELAY);

	// Write ones
	Cy_GPIO_Write(SSEG_TENS_PORT, SSEG_TENS_PIN, 0);
	driveSSEG(a_sseg, randNo_ones);
	Cy_GPIO_Write(SSEG_ONES_PORT, SSEG_ONES_PIN, 1);
	Cy_SysLib_Delay(DELAY);
}

void geigerTick_ISR() {
	randNo = Cy_TCPWM_Counter_GetCapture(RAND_COUNTER_HW, RAND_COUNTER_NUM);
	randNo_huns = randNo / 100;
	randNo_tens = (randNo % 100) / 10;
	randNo_ones = randNo % 10;
	Cy_TCPWM_ClearInterrupt(RAND_COUNTER_HW, RAND_COUNTER_NUM, CY_TCPWM_INT_ON_CC);    // Clear the interrupt
}

int main(void) {
	cy_rslt_t result;

	/* Initialize the device and board peripherals */
	result = cybsp_init();
	if(result != CY_RSLT_SUCCESS) {
		CY_ASSERT(0);
	}

	/* Enable global interrupts */
	__enable_irq();

	cy_stc_sysint_t geigerIntrCfg = {
	RAND_COUNTER_IRQ,    // interrupt source
	3UL                  // priority
	};

	Cy_TCPWM_Counter_Init(RAND_COUNTER_HW, RAND_COUNTER_NUM, &RAND_COUNTER_config);    // Initialize counter
	Cy_TCPWM_Counter_Enable(RAND_COUNTER_HW, RAND_COUNTER_NUM);                        // Enable counter
	Cy_TCPWM_TriggerStart(RAND_COUNTER_HW, RAND_COUNTER_MASK);                         // Start counter

	Cy_GPIO_SetFilter(GEIGER_COUNTER_TRIGGER_PORT, GEIGER_COUNTER_TRIGGER_PIN);    // 50ns glitch filter on geiger counter trigger in
	Cy_SysInt_Init(&geigerIntrCfg, &geigerTick_ISR);                               // Initialize geiger interrupt vector
	NVIC_EnableIRQ(geigerIntrCfg.intrSrc);                                         // Enable geiger interrupt source

	for(;;) {
		drawNum(&rand_sseg);
	}
}

/* [] END OF FILE */
