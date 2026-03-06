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

// --- PARÂMETROS DO SISTEMA FÍSICO ---
// Escala do ADC: 
// 1. Converte valor de 12-bits (0-4095) para tensão no pino (0-3.3V)
// 2. Multiplica pela proporção da instrumentação (12V reais = 1.8409V lidos)
const float ADC_TO_VOLTS = (3.3f / 4095.0f) * (12.0f / 1.8409f); 

// Período do PWM: (100MHz / (2 * 40kHz)) = 1250 contagens no modo Up-Down
const float PWM_PERIOD_COUNTS = 1250.0f; 

// ============================================================================
//                  VARIÁVEIS DE DEBUG PARA O CCS (Malha Aberta)
// ============================================================================
// O uso do "volatile" impede que o compilador otimize essas variáveis, 
// permitindo a leitura e escrita em tempo real via aba "Expressions" no CCS.

volatile float manual_duty = 0.50f;   // Inicializa com 50% de Duty Cycle
volatile uint16_t adc_raw  = 0;       // Valor bruto lido do pino ADCIN5 (0 a 4095)
volatile float v_out_real  = 0.0f;    // Tensão convertida para Volts reais

// ============================================================================
//                                  MAIN
// ============================================================================
void main(void)
{
    // 1. Inicialização Básica do C2000
    Device_init();
    Interrupt_initModule();
    Interrupt_initVectorTable();
    
    // 2. Configuração dos Periféricos
    initGPIO();
    initADC();
    initPIE(); 
    initPWM(); // PWM por último para garantir estado seguro
    
    // 3. Habilitação de Interrupções Globais
    EINT;
    ERTM;
    
    // 4. Sincronização do Clock do PWM (Libera os contadores)
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);
    
    // 5. Limpeza de flags de segurança
    EPWM_clearTripZoneFlag(EPWM1_BASE, EPWM_TZ_INTERRUPT | EPWM_TZ_FLAG_OST);
    ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);
    EPWM_clearEventTriggerInterruptFlag(EPWM1_BASE);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);

    // Loop Infinito
    while(1) 
    {
        // O controle (malha aberta) roda inteiramente dentro da ISR do ADC
    }
}

// ============================================================================
//                               CONFIGURAÇÕES
// ============================================================================

void initGPIO(void)
{
    // Configura o pino EPWM1A para comandar a chave do Buck
    GPIO_setPinConfig(GPIO_0_EPWM1A);
    GPIO_setPadConfig(0, GPIO_PIN_TYPE_STD);
    
    // Pino de Debug para medir tempo de execução da ISR
    GPIO_setPinConfig(GPIO_30_GPIO30);
    GPIO_setDirectionMode(30, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(30, GPIO_PIN_TYPE_STD);
    GPIO_writePin(30, 1); 
}

void initPWM(void)
{
    // Base de tempo: 1250 contagens subindo e descendo = 40 kHz
    EPWM_setTimeBasePeriod(EPWM1_BASE, 1250);
    EPWM_setTimeBaseCounterMode(EPWM1_BASE, EPWM_COUNTER_MODE_UP_DOWN);
    EPWM_setClockPrescaler(EPWM1_BASE, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);
    
    // Partida segura: Duty Cycle = 0% (Chave do Buck aberta)
    EPWM_setCounterCompareValue(EPWM1_BASE, EPWM_COUNTER_COMPARE_A, 0);
    
    // --- LÓGICA DO DUTY CYCLE (Direta) ---
    // Subindo e cruza CMPA -> Vai para LOW
    EPWM_setActionQualifierAction(EPWM1_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    // Descendo e cruza CMPA -> Vai para HIGH
    EPWM_setActionQualifierAction(EPWM1_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
    
    // Gatilho para o ADC exatamente no ZERO do contador (centro do pulso HIGH)
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
    DEVICE_DELAY_US(1000); // Aguarda conversor ligar
    
    // --- ADC NO CANAL 5 (Pino 68) ---
    // O hardware agora vai ler o ADCIN5 a cada gatilho do ePWM1
    ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN5, 20); 

    // Gera interrupção quando a conversão acabar
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
//                     ISR: LEITURA E ATUALIZAÇÃO (MALHA ABERTA)
// ============================================================================

__interrupt void adca1_ISR(void)
{
    GPIO_writePin(30, 0); // Início do processamento (Sinal p/ Osciloscópio)

    // 1. LEITURA DA TENSÃO
    // Salva direto nas variáveis globais (visualizáveis no CCS)
    adc_raw = ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER0);
    v_out_real = (float)adc_raw * ADC_TO_VOLTS;

    // 2. PROTEÇÃO DE SEGURANÇA (Saturação)
    // Impede a inserção de valores inválidos ou perigosos via CCS
    if(manual_duty > 0.95f) manual_duty = 0.95f; // Máximo de 95%
    if(manual_duty < 0.0f)  manual_duty = 0.0f;  // Mínimo de 0%

    // 3. ATUALIZAÇÃO DO PWM
    // Converte a fração (0.0 a 1.0) para contagens do registrador CMPA
    uint16_t duty_counts = (uint16_t)(manual_duty * PWM_PERIOD_COUNTS);
    EPWM_setCounterCompareValue(EPWM1_BASE, EPWM_COUNTER_COMPARE_A, duty_counts);

    // 4. LIMPEZA DE FLAGS
    ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
    
    GPIO_writePin(30, 1); // Fim do processamento
}