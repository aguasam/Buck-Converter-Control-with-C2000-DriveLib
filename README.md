# Buck Converter Control with C2000™ DriveLib

This repository contains the implementation of a DC-DC Buck converter control system using the **Texas Instruments TMS320F280049C** DSP (LaunchPad LAUNCHXL-F280049C). The primary goal is to demonstrate the configuration and operation of **ePWM** and **ADC** peripherals using the **DriveLib** software library.

## 📌 Overview
The project is divided into two main source files:
1.  **Open Loop (`malha_aberta.c`):** Used for hardware validation, sensor calibration, and PWM modulation testing with a fixed or manually adjustable duty cycle via the Code Composer Studio (CCS) Expressions window.
2.  **Closed Loop (`malha_fechada.c`):** Implementation of a real-time discrete Proportional-Integral (PI) controller for output voltage regulation.

## 🛠️ Technical Specifications
* **MCU:** TMS320F280049C (C2000™ 32-bit Real-Time MCU)
* **System Clock (SYSCLK):** 100 MHz
* **PWM Switching Frequency:** 40 kHz
* **ADC Sampling Frequency:** 40 kHz (Synchronized with PWM)
* **Counting Mode:** Up-Down Count (Symmetric/Triangular)
* **Control Strategy:** Discrete PI Compensator ($Z$-Domain)

---

## 🔧 Peripheral Configuration (DriveLib)

### 1. ePWM Submodule
To achieve a **40 kHz** frequency in **Up-Down** mode, the period value ($TBPRD$) is calculated based on the 100 MHz clock:
$$TBPRD = \frac{f_{sysclk}}{2 \cdot f_{pwm}} = \frac{100 \text{ MHz}}{2 \cdot 40 \text{ kHz}} = 1250$$

Key DriveLib functions used for `EPWM1` configuration:
* `EPWM_setTimeBasePeriod(EPWM1_BASE, 1250)`: Sets the switching frequency.
* `EPWM_setCounterMode(EPWM1_BASE, EPWM_COUNTER_MODE_UP_DOWN)`: Ensures pulse symmetry to reduce harmonic distortion.
* `EPWM_setActionQualifierAction()`: Configures the GPIO output logic (Set/Clear).
* `EPWM_setADCTriggerSource(EPWM1_BASE, EPWM_SOC_A, EPWM_SOC_TBCTR_PERIOD)`: Configures the PWM to trigger the ADC conversion exactly at the period peak, avoiding switching noise.

### 2. ADC Submodule
The system uses **ADCA** (Channel 0) to monitor the feedback voltage. The configuration follows the guidelines from the `ADC.pdf` manual:
* `ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN0, 15)`: Configures SOC0 to sample channel A0, triggered by ePWM1, with a sample-and-hold window of 15 cycles.
* `ADC_setInterruptSource(ADCA_BASE, ADC_INT_NUMBER1, ADC_SOC_NUMBER0)`: Links the end of SOC0 conversion to the main system interrupt.

---

## 📈 Experimental Results
The oscilloscope captures (files `F0000TEK.BMP` through `F0004TEK.BMP`) illustrate the system behavior:

* **PWM Signal:** 40 kHz square wave with variable duty cycle.
* **Step Response:** The closed-loop system adjusting the output voltage to match the `V_REF_TARGET` setpoint.
* **Control Timing:** GPIO 30 is toggled at the start and end of the ISR (`adca1_ISR`). Observing this pin on the scope allows for CPU load measurement.

---

## 💻 Control ISR Structure
In `malha_fechada.c`, the control law is executed within the ADC interrupt service routine:

```c
__interrupt void adca1_ISR(void)
{
    // --- START MARKER ---
    GPIO_writePin(30, 1); // Set pin HIGH to measure CPU usage

    // 1. ADC READ AND CONVERSION
    // 'f' suffix ensures 32-bit float calculation (optimized for FPU)
    adc_raw = ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER0);
    v_out_real = (float)adc_raw * (3.3f / 4095.0f); 

    // 2. ERROR CALCULATION
    e_k = V_REF_TARGET - v_out_real;

    // 3. DISCRETE PI CONTROL LAW
    u_k = u_k1 + (B0_pi * e_k) + (B1_pi * e_k1);

    // 4. SATURATION (Anti-Windup) AND PWM UPDATE
    if(u_k > 0.95f) u_k = 0.95f; 
    if(u_k < 0.05f) u_k = 0.05f;
    
    EPWM_setCounterCompareValue(EPWM1_BASE, EPWM_COUNTER_COMPARE_A, (uint16_t)(u_k * 1250.0f));

    // 5. STATE UPDATE AND FLAG CLEARING
    e_k1 = e_k;
    u_k1 = u_k;
    ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
    
    GPIO_writePin(30, 0); // End of processing
}
