#include "driverlib.h"
#include "device.h"

// ============================================================================
//                          DEFINIÇÕES E CONSTANTES
// ============================================================================

// --- PROTÓTIPOS DAS FUNÇÕES ---
void initGPIO(void);
void initPWM(void);
void initADC(void);
void initPIE(void);
__interrupt void adca1_ISR(void);

// Tensão de Saída Alvo do Buck (No pino do DSP)
const float V_REF_TARGET = 1.8409f; 

// Período do PWM: (100MHz / (2 * 40kHz)) = 1250 contagens no modo Up-Down
const float PWM_PERIOD_COUNTS = 1250.0f; 

// --- GANHOS DO CONTROLADOR PI (Discretizado por ZoH) ---
const float B0_pi = 0.04f; 
const float B1_pi = -0.03775f;

// ============================================================================
//                  VARIÁVEIS DE DEBUG PARA O CCS E ESTADOS DO PI
// ============================================================================

volatile float u_k        = 0.0f; // Esforço de controle (Duty Cycle de 0 a 1)
volatile float v_out_real = 0.0f; // Tensão atual lida e convertida para Volts
volatile uint16_t adc_raw = 0;    // Valor bruto do ADC (0 a 4095)

// Estados passados 
static float u_k1 = 0.0f; // u[k-1]
static float e_k  = 0.0f; // e[k]
static float e_k1 = 0.0f; // e[k-1]


// ============================================================================
//                                  MAIN
// ============================================================================
void main(void)
{
    Device_init();
    Interrupt_initModule();
    Interrupt_initVectorTable();
    
    initGPIO();
    initADC();
    initPIE(); 
    initPWM(); 
    
    EINT;
    ERTM;
    
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);
    
    EPWM_clearTripZoneFlag(EPWM1_BASE, EPWM_TZ_INTERRUPT | EPWM_TZ_FLAG_OST);
    ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);
    EPWM_clearEventTriggerInterruptFlag(EPWM1_BASE);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);

    while(1) 
    {
        // O controle roda inteiramente dentro da ISR
    }
}

// ============================================================================
//                               CONFIGURAÇÕES
// ============================================================================

void initGPIO(void)
{
    // Configura o pino EPWM1A
    GPIO_setPinConfig(GPIO_0_EPWM1A);
    GPIO_setPadConfig(0, GPIO_PIN_TYPE_STD);
    
    // --- Pino de Debug (GPIO 30) ---
    GPIO_setPinConfig(GPIO_30_GPIO30);
    GPIO_setDirectionMode(30, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(30, GPIO_PIN_TYPE_STD);
    GPIO_writePin(30, 0); // Inicia em BAIXO para o teste no osciloscópio
    
    // Habilita a chave analógica no GPIO23 (onde está o ADCINA5)
    GPIO_setAnalogMode(23, GPIO_ANALOG_ENABLED);
}

void initPWM(void)
{
    EPWM_setTimeBasePeriod(EPWM1_BASE, 1250);
    EPWM_setTimeBaseCounterMode(EPWM1_BASE, EPWM_COUNTER_MODE_UP_DOWN);
    EPWM_setClockPrescaler(EPWM1_BASE, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);
    
    EPWM_setCounterCompareValue(EPWM1_BASE, EPWM_COUNTER_COMPARE_A, 0);
    
    // Lógica do Duty Cycle
    EPWM_setActionQualifierAction(EPWM1_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(EPWM1_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
    
    // Gatilho do ADC no ZERO do contador
    EPWM_setADCTriggerSource(EPWM1_BASE, EPWM_SOC_A, EPWM_SOC_TBCTR_ZERO);
    EPWM_enableADCTrigger(EPWM1_BASE, EPWM_SOC_A);
    EPWM_setADCTriggerEventPrescale(EPWM1_BASE, EPWM_SOC_A, 1); 
}

void initADC(void)
{
    EALLOW;
    ADC_setVREF(ADCA_BASE, ADC_REFERENCE_INTERNAL, ADC_REFERENCE_3_3V);
    ADC_setPrescaler(ADCA_BASE, ADC_CLK_DIV_4_0);
    ADC_enableConverter(ADCA_BASE);
    EDIS;
    DEVICE_DELAY_US(1000); 
    
    // Lendo o ADCINA5 (Pino 23) via Trigger do EPWM1
    ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN5, 20); 

    ADC_setInterruptSource(ADCA_BASE, ADC_INT_NUMBER1, ADC_SOC_NUMBER0); 
    ADC_enableInterrupt(ADCA_BASE, ADC_INT_NUMBER1);
    ADC_setInterruptPulseMode(ADCA_BASE, ADC_PULSE_END_OF_CONV);
}

void initPIE(void)
{
    Interrupt_register(INT_ADCA1, &adca1_ISR);
    Interrupt_enable(INT_ADCA1); 
}

// ============================================================================
//                          MALHA DE CONTROLE PI FECHADA
// ============================================================================

__interrupt void adca1_ISR(void)
{
    // -----------------------------------------------------------------
    // MARCADOR DE INÍCIO: Coloca o pino em ALTO assim que a CPU entra
    // -----------------------------------------------------------------
    
    GPIO_writePin(30, 1); 

    // 1. LEITURA E CONVERSÃO
    // O uso do 'f' garante cálculo em 32-bits float (muito mais rápido na FPU)
    adc_raw = ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER0);
    v_out_real = (float)adc_raw * (3.3f / 4095.0f);

    
    
    
    // 2. CÁLCULO DO ERRO
    e_k = V_REF_TARGET - v_out_real;

    

    // 3. PI DISCRETO
    u_k = u_k1 + (B0_pi * e_k) + (B1_pi * e_k1);

    // 4. SATURAÇÃO (Anti-Windup)
    if(u_k > 0.95f) u_k = 0.95f; 
    if(u_k < 0.0f)  u_k = 0.0f;  

    // 5. ATUALIZAÇÃO DO PWM 
    uint16_t duty_counts = (uint16_t)(u_k * PWM_PERIOD_COUNTS);
    EPWM_setCounterCompareValue(EPWM1_BASE, EPWM_COUNTER_COMPARE_A, duty_counts);

    // 6. ATUALIZAÇÃO DOS ESTADOS
    u_k1 = u_k;
    e_k1 = e_k;

    // LIMPEZA DE FLAGS DA INTERRUPÇÃO
    ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);

    GPIO_writePin(30, 0); 
    
    // -----------------------------------------------------------------
    // MARCADOR DE FIM: Coloca o pino em BAIXO após todos os cálculos
    // -----------------------------------------------------------------
    
}