#include "driverlib.h"
#include "device.h"

void initDualADC(void);

void main(void) {
    Device_init();
    Device_initGPIO();
    Interrupt_initModule();
    Interrupt_initVectorTable();

    initDualADC();

    // Enable Global Interrupts
    EALLOW;
    EDIS;
    Interrupt_enableMaster();

    while(1) {
        // Hardware is automatically sampling Current and Voltage
    }
}

void initDualADC(void) {
    // --- 1. CLOCK & MODE SETUP (Module A: Voltage | Module B: Current) ---
    ADC_setPrescaler(ADCA_BASE, ADC_CLK_DIV_2_0);
    ADC_setPrescaler(ADCB_BASE, ADC_CLK_DIV_2_0);
    
    ADC_setMode(ADCA_BASE, ADC_RESOLUTION_12BIT, ADC_SIGNAL_MODE_SINGLE_ENDED);
    ADC_setMode(ADCB_BASE, ADC_RESOLUTION_12BIT, ADC_SIGNAL_MODE_SINGLE_ENDED);

    // --- 2. POWER UP ---
    ADC_enableConverter(ADCA_BASE);
    ADC_enableConverter(ADCB_BASE);
    DEVICE_DELAY_US(1000); // Allow analog circuitry to stabilize

    // --- 3. SOC CONFIGURATION (Instrumentation Logic) ---
    
    // SOC0 on ADC A: Measures Bus Voltage (Channel A0)
    ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_EPWM1_SOCA, 
                 ADC_CH_ADCIN0, 15);

    // SOC0 on ADC B: Measures Phase Current (Channel B0)
    // Linked to the SAME ePWM trigger for synchronization
    ADC_setupSOC(ADCB_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_EPWM1_SOCA, 
                 ADC_CH_ADCIN0, 15);

    // --- 4. INTERRUPT & POST-PROCESSING ---
    
    // We only need one interrupt to signal that BOTH conversions are done
    // because they are triggered at the same time and have the same duration.
    ADC_setInterruptSource(ADCB_BASE, ADC_INT_NUMBER1, ADC_SOC_NUMBER0);
    ADC_enableInterrupt(ADCB_BASE, ADC_INT_NUMBER1);
    ADC_clearInterruptStatus(ADCB_BASE, ADC_INT_NUMBER1);
}
