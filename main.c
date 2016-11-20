#include "stm32f10x_conf.h"

uint8_t program_mode = 0;
uint32_t fan_speed_start_address = 0x800F000; // starting from 60KB

__IO uint16_t Mainboard_Duty_Cycle = 0;
__IO uint16_t TIM3_IC2_capture_1 = 0;
__IO uint16_t TIM3_IC2_capture_2 = 0;
__IO uint16_t TIM3_IC2_capture_number = 0;
__IO uint16_t TIM3_IC2_capture_diff = 0;
__IO uint16_t TIM3_IC2_Freq = 0;
__IO uint16_t fan_speed_list[100];
__IO uint16_t fan_test_step = 100;
__IO uint16_t fan_test_sample = 0;
__IO uint16_t fan_test_speed = 0;
__IO uint16_t fake_fan_speed = 0;

int16_t fan_alive_timeout = 8;
__IO int16_t fan_10_alive = 8;
__IO int16_t fan_11_alive = 8;
__IO int16_t fan_12_alive = 8;
__IO int16_t fan_13_alive = 8;
__IO int16_t fan_14_alive = 8;
__IO int16_t fan_15_alive = 8;

void write_to_flash(void)
{
    uint8_t i;
    uint32_t start_address = fan_speed_start_address;

    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
    FLASH_ErasePage(fan_speed_start_address);

    for(i = 0; i < 100; i++)
    {
        FLASH_ProgramHalfWord(start_address, fan_speed_list[i]);
        start_address = start_address + 2;
    }

    FLASH_Lock();
}


void erase_flash(void)
{
    uint8_t i;
    for(i = 0; i < 100; i++)
    {
        fan_speed_list[i] = 0xffff;
    }
    write_to_flash();
}


void read_from_flash(void)
{
    uint8_t i;
    uint32_t start_address = fan_speed_start_address;

    for(i = 0; i < 100; i++)
    {
        fan_speed_list[i] = *(uint16_t *)start_address;
        start_address = start_address + 2;
    }
}


void EXTI15_10_IRQHandler(void)
{
    uint32_t p = EXTI->PR;
    EXTI->PR = p; // reset, Reference manual "10.3.6 Pending register (EXTI_PR)"

    if (p & (1 << 10)) { // EXTI Line 10
            fan_10_alive = fan_alive_timeout;
    }

    if (p & (1 << 11)) { // EXTI Line 11
            fan_11_alive = fan_alive_timeout;
    }

    if (p & (1 << 12)) { // EXTI Line 12
            fan_12_alive = fan_alive_timeout;
    }

    if (p & (1 << 13)) { // EXTI Line 13
            fan_13_alive = fan_alive_timeout;
    }

    if (p & (1 << 14)) { // EXTI Line 14
            fan_14_alive = fan_alive_timeout;
    }

    if (p & (1 << 15)) { // EXTI Line 15
            fan_15_alive = fan_alive_timeout;
    }
}


uint8_t check_fan_alive()
{
    if(fan_10_alive){
        fan_10_alive--;
    }
    if(fan_11_alive){
        fan_11_alive--;
    }
    if(fan_12_alive){
        fan_12_alive--;
    }
    if(fan_13_alive){
        fan_13_alive--;
    }
    if(fan_14_alive){
        fan_14_alive--;
    }
    if(fan_15_alive){
        fan_15_alive--;
    }

    if(fan_10_alive && fan_11_alive && fan_12_alive && fan_13_alive && fan_14_alive && fan_15_alive){
        return (uint8_t)0;
    } else {
        return (uint8_t)1;
    }
}


void TIM3_IRQHandler(void)
{
    TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);

    if(TIM3_IC2_capture_number == 0)
    {
        /* Get the Input Capture value */
        TIM3_IC2_capture_1 = TIM_GetCapture2(TIM3);
        TIM3_IC2_capture_number = 1;
    }
    else if(TIM3_IC2_capture_number == 1)
    {
        /* Get the Input Capture value */
        TIM3_IC2_capture_2 = TIM_GetCapture2(TIM3);

        /* Capture computation */
        if (TIM3_IC2_capture_2 > TIM3_IC2_capture_1)
        {
            TIM3_IC2_capture_diff = (TIM3_IC2_capture_2 - TIM3_IC2_capture_1);
        }
        else
        {
            TIM3_IC2_capture_diff = ((0xFFFF - TIM3_IC2_capture_1) + TIM3_IC2_capture_2);
        }

        /* Frequency computation: SYSCLK / TIM_Prescaler */
        TIM3_IC2_Freq = (SystemCoreClock / (1099-1)) / TIM3_IC2_capture_diff;

        TIM3_IC2_capture_number = 0;
    }
}


void config_fan_speed_detection(void)
{
    /* TIM3 clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    /* GPIOA and GPIOB clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    /* Enable the TIM3 global Interrupt */
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* Configure the GPIO ports */
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    TIM_ICInitTypeDef  TIM_ICInitStructure;
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = 0x0;
    TIM_ICInit(TIM3, &TIM_ICInitStructure);

    TIM_PrescalerConfig(TIM3, 1099-1, TIM_PSCReloadMode_Immediate); // 65,5kHz

    /* Enable the CC2 Interrupt Request */
    TIM_ITConfig(TIM3, TIM_IT_CC2, ENABLE);

    /* TIM enable counter */
    TIM_Cmd(TIM3, ENABLE);
}

__IO uint16_t _CCR1;
__IO uint16_t _CCR2;

void TIM2_IRQHandler(void)
{
    TIM2->SR = (uint16_t)~TIM_IT_CC2;

    _CCR1 = TIM2->CCR1; // Puls width
    _CCR2 = TIM2->CCR2; // Frequency

    if(_CCR2 > 0 && _CCR1 > 0 && _CCR2 >= _CCR1)
    {
        Mainboard_Duty_Cycle = (_CCR1 * 100) / _CCR2;
    }
}


uint16_t puls_width_calculator(uint16_t period, uint16_t duty_cycle)
{
    if(duty_cycle > 100){
        duty_cycle = 100;
    }
    return (uint16_t)(period * 256 / 100 * duty_cycle / 256);
}


/* PWM signal erzeugen */
void config_fan_test_generator(uint16_t duty_cycle)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    /* GPIOx clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    TIM_TimeBaseInitTypeDef TIM_TimeBase_InitStructure;
    TIM_TimeBase_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBase_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBase_InitStructure.TIM_Period = 2880-1; // 25kHz
    TIM_TimeBase_InitStructure.TIM_Prescaler = 0;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBase_InitStructure);

    TIM_OCInitTypeDef TIM_OC_InitStructure;
    TIM_OC_InitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OC_InitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
    TIM_OC_InitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set;
    TIM_OC_InitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC_InitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
    TIM_OC_InitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OC_InitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
    TIM_OC_InitStructure.TIM_Pulse = puls_width_calculator(2880, duty_cycle);
    TIM_OC1Init(TIM2, &TIM_OC_InitStructure);

    TIM_Cmd(TIM2, ENABLE);
}


/* PWM signal erneuern */
void fan_test_generator_update(uint16_t duty_cycle)
{
    TIM_OCInitTypeDef TIM_OC_InitStructure;
    TIM_OC_InitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OC_InitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
    TIM_OC_InitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set;
    TIM_OC_InitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC_InitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
    TIM_OC_InitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OC_InitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
    TIM_OC_InitStructure.TIM_Pulse = puls_width_calculator(2880, duty_cycle);
    TIM_OC1Init(TIM2, &TIM_OC_InitStructure);
}


/* PWM input capture mode */
void config_mainboard_pwm_detection(void)
{
    /* TIMx clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    /* GPIOx clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    /* Configure the GPIO ports */
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* Enable the TIMx global Interrupt */
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* TIMx configuration / PWM input capture */
    TIM_ICInitTypeDef  TIM_ICInitStructure;
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = 4; // input filter, ohne erfolgt PWM messung nicht zuverlässig wenn alle lüfter laufen

    TIM_PWMIConfig(TIM2, &TIM_ICInitStructure);

    /* Select the TIMx Input Trigger: TI2FP2 */
    TIM_SelectInputTrigger(TIM2, TIM_TS_TI2FP2);

    /* Select the slave Mode: Reset Mode */
    TIM_SelectSlaveMode(TIM2, TIM_SlaveMode_Reset);

    /* Enable the Master/Slave Mode */
    TIM_SelectMasterSlaveMode(TIM2, TIM_MasterSlaveMode_Enable);

    /* Enable the CC2 Interrupt Request */
    TIM_ITConfig(TIM2, TIM_IT_CC2, ENABLE);

    /* TIM enable counter */
    TIM_Cmd(TIM2, ENABLE);
}


/* drehzahlsimmulator */
void config_fake_fan()
{
    /* GPIOx clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_TIM1, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* TIM1 Main Output Enable */
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
}


void fake_fan_update()
{
    TIM_TimeBaseInitTypeDef TIM_TimeBase_InitStructure;
    TIM_TimeBase_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBase_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    if(fake_fan_speed > 1)
    {
        TIM_TimeBase_InitStructure.TIM_Period = (65514/fake_fan_speed)-1;
    }
    else
    {
        TIM_TimeBase_InitStructure.TIM_Period = 65514-1;
    }
    TIM_TimeBase_InitStructure.TIM_Prescaler = 1099-1;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBase_InitStructure);

    TIM_OCInitTypeDef TIM_OC_InitStructure;
    TIM_OC_InitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OC_InitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
    TIM_OC_InitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set;
    TIM_OC_InitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC_InitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
    TIM_OC_InitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OC_InitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
    TIM_OC_InitStructure.TIM_Pulse = (TIM_TimeBase_InitStructure.TIM_Period+1)/2;
    TIM_OC1Init(TIM1, &TIM_OC_InitStructure);
}


/* PWM signal erzeugen */
void config_fan_generator(uint16_t duty_cycle)
{
    /* TIM3 clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    /* GPIOx clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    TIM_TimeBaseInitTypeDef TIM_TimeBase_InitStructure;
    TIM_TimeBase_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBase_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBase_InitStructure.TIM_Period = 2880-1; // 25kHz
    TIM_TimeBase_InitStructure.TIM_Prescaler = 0;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBase_InitStructure);

    TIM_OCInitTypeDef TIM_OC_InitStructure;
    TIM_OC_InitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OC_InitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
    TIM_OC_InitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set;
    TIM_OC_InitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC_InitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
    TIM_OC_InitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OC_InitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
    TIM_OC_InitStructure.TIM_Pulse = puls_width_calculator(2880, duty_cycle);
    TIM_OC1Init(TIM3, &TIM_OC_InitStructure); // PA6
    TIM_OC2Init(TIM3, &TIM_OC_InitStructure); // PA7
    TIM_OC3Init(TIM3, &TIM_OC_InitStructure); // PB0

    TIM_OC_InitStructure.TIM_Pulse = puls_width_calculator(2880, (duty_cycle+33)); // +33% mehr als die anderen, äußerer lüfter
    TIM_OC4Init(TIM3, &TIM_OC_InitStructure); // PB1

    TIM_Cmd(TIM3, ENABLE);
}


void fan_generator_update(uint16_t duty_cycle)
{
    TIM_OCInitTypeDef TIM_OC_InitStructure;
    TIM_OC_InitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OC_InitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
    TIM_OC_InitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set;
    TIM_OC_InitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC_InitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
    TIM_OC_InitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OC_InitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
    TIM_OC_InitStructure.TIM_Pulse = puls_width_calculator(2880, duty_cycle);
    TIM_OC1Init(TIM3, &TIM_OC_InitStructure); // PA6
    TIM_OC2Init(TIM3, &TIM_OC_InitStructure); // PA7
    TIM_OC3Init(TIM3, &TIM_OC_InitStructure); // PB0

    TIM_OC_InitStructure.TIM_Pulse = puls_width_calculator(2880, (duty_cycle+33)); // +33% mehr als die anderen, äußerer lüfter
    TIM_OC4Init(TIM3, &TIM_OC_InitStructure); // PB1
}


void TIM4_IRQHandler(void)
{
    TIM_ClearITPendingBit(TIM4, TIM_IT_Update);

    if(program_mode == 2)
    {
        if(TIM3_IC2_Freq > 0)
        {
            fan_test_sample++;
            fan_test_speed = fan_test_speed + TIM3_IC2_Freq;

            if(fan_test_sample == 4)
            {
                fan_speed_list[(fan_test_step-1)] = fan_test_speed / 4;

                if(fan_test_step == 0)
                {
                    TIM_Cmd(TIM4, DISABLE);

                    write_to_flash();

                    fan_test_generator_update(100);
                }
                else
                {
                    fan_test_generator_update(fan_test_step);

                    fan_test_sample = 0;
                    fan_test_speed = 0;
                    fan_test_step--;
                }
            }

            TIM3_IC2_Freq = 0;
        }
    }
    else if(program_mode == 0)
    {
        uint16_t duty_cycle;

        if(Mainboard_Duty_Cycle > 0)
        {
            if(Mainboard_Duty_Cycle > 30)
            {
                duty_cycle = Mainboard_Duty_Cycle;
            }
            else
            {
                duty_cycle = 30; // Minnimum!
            }
        }
        else
        {
            duty_cycle = 100;
        }

        if(TIM3->CCR3 != puls_width_calculator(2880, duty_cycle))
        {
            fan_generator_update(duty_cycle);
        }

        if(check_fan_alive() == 0)
        {
            uint16_t speed_update;

            if(Mainboard_Duty_Cycle > 0)
            {
                speed_update = fan_speed_list[(Mainboard_Duty_Cycle-1)];
            }
            else
            {
                speed_update = fan_speed_list[0]/2; // PWM Signal rauslocken
            }

            if(speed_update != fake_fan_speed)
            {
                fake_fan_speed = speed_update;
                fake_fan_update();
                TIM_Cmd(TIM1, ENABLE);
            }
        }
        else
        {
            TIM_Cmd(TIM1, DISABLE); // Lüfterstillstand / Defekt signalisieren
            fake_fan_speed = 0;
            GPIO_WriteBit(GPIOB, GPIO_Pin_5, Bit_SET); // enable fail led
        }

        Mainboard_Duty_Cycle = 0;
    }
}


void config_fake_fan_update(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    TIM_TimeBaseInitTypeDef TIM_TimeBase_InitStructure;
    TIM_TimeBase_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBase_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBase_InitStructure.TIM_Prescaler = 59; // 50ms
    TIM_TimeBase_InitStructure.TIM_Period = 59999;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBase_InitStructure);

    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);

    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
    NVIC_Init(&NVIC_InitStructure);

    TIM_Cmd(TIM4, ENABLE);
}


void config_fan_test_generator_update(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    TIM_TimeBaseInitTypeDef TIM_TimeBase_InitStructure;
    TIM_TimeBase_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBase_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBase_InitStructure.TIM_Prescaler = 143; // 125ms
    TIM_TimeBase_InitStructure.TIM_Period = 62499;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBase_InitStructure);

    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);

    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
    NVIC_Init(&NVIC_InitStructure);

    TIM_Cmd(TIM4, ENABLE);
}


void config_fan_alive_detection(void)
{
    /* GPIOB clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

    /* Configure the GPIO Pins. */
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* Fan fail LED */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource10);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource11);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource12);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource13);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource14);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource15);

    EXTI_InitTypeDef EXTI_InitStructure;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_InitStructure.EXTI_Line = EXTI_Line10 | EXTI_Line11 | EXTI_Line12 | EXTI_Line13 | EXTI_Line14 | EXTI_Line15;
    EXTI_Init(&EXTI_InitStructure);

    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
    NVIC_Init(&NVIC_InitStructure);
}


void detect_mode(void)
{
    /* GPIOx clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    /* Configure the GPIO ports */
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    if(!GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_4))
    {
        program_mode = 2;
    }
    else if(!GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_5))
    {
        program_mode = 1;
    }
    else
    {
        program_mode = 0;
    }
}


int main(void)
{
    detect_mode();

    if(program_mode == 2) /* PA4 + PA2 */
    {
        /* PA0 */
        config_fan_test_generator(100);

        /* PA7 */
        config_fan_speed_detection();

        config_fan_test_generator_update();
    }
    else if(program_mode == 1) /* PA5 + PA3 */
    {
        erase_flash();
    }
    else
    {
        /* PA1 */
        config_mainboard_pwm_detection();

        /* PB0, PB1, PA6, PA7 */
        config_fan_generator(100);

        /* PB5 (fail LED), PB10, PB11,  PB12, PB13, PB14, PB15 */
        config_fan_alive_detection();

        read_from_flash();

        /* PA8 */
        config_fake_fan();

        config_fake_fan_update();
    }

    while (1)
    {
    }
}
