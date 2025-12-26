#include "stm32f10x.h"

unsigned char data1 = 0;//全局变量data1
unsigned char data2 = 0;//全局变量data1

unsigned int sysTimeFlag = 0;
unsigned int key_data = 0;
unsigned int key_num = 0;
unsigned int key_back_data = 0;
unsigned int key_back_num = 110;

void delay_ms(uint64_t xms) {
    SysTick->LOAD = xms * 72000; // SysTick计数器加载值设定为xus * 72
    SysTick->VAL = 0x00; // 清除SysTick计数器
    SysTick->CTRL = 0x00000005; // 启动SysTick计时
    while(!(SysTick->CTRL & 0x00010000)); // 等待SysTick计时完成
    SysTick->CTRL = 0x00000004; // 关闭SysTick
}

void sendstr(const char* arr) {
    int j = 0;
    while (arr[j] != '\0') {
        while ((USART1->SR & 0x0040) == 0);
        // while ((USART3->SR & 0x0040) == 0);
        USART1->DR = arr[j]; // 依次发送字符串内的字符
        // USART3->DR = arr[j]; // 依次发送字符串内的字符
        j++;
    }
}

void wifiSet(void) {
    sendstr("+++");
    delay_ms(100);
    sendstr("AT+WMODE=STA"); // 设置WiFi工作模式为STA（Station）
    delay_ms(10000);
    sendstr("AT+WSTA=temp,12345678"); // 设置WiFi STA的SSID为temp，密码为12345678
    delay_ms(100);
    sendstr("AT+SOCK=TCPC,192.168.4.1,8080"); // 设置TCP客户端连接到服务器
    delay_ms(100);
    sendstr("AT+RESET"); // 重启WiFi模块
}

void sysclock() {
    RCC->CR |= RCC_CR_HSEON; // 启动HSE
    while (!(RCC->CR & RCC_CR_HSERDY)); // 等待HSE就绪
    RCC->CFGR &= ~RCC_CFGR_PLLMULL; // 清除PLL倍频设置
    RCC->CFGR |= RCC_CFGR_PLLMULL9; // 设置PLL倍频为x9
    RCC->CFGR |= RCC_CFGR_PLLSRC; // 选择HSI作为PLL输入源
    RCC->CR |= RCC_CR_PLLON; // 启动PLL
    while (!(RCC->CR & RCC_CR_PLLRDY)); // 等待PLL锁定
    FLASH->ACR |= FLASH_ACR_LATENCY_2; // 设置FLASH延时为2个等待状态，因为FLASH跟不上CPU速度，所有读取FLASH需要等待
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1; // AHB不分频（72MHz）
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2; // APB1分频2（36MHz）
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV2; // APB2不分频（36MHz）

    RCC->CFGR |= RCC_CFGR_SW_PLL; // 选择PLL作为系统时钟
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL); // 等待切换完成
}

void usart1Set(void) {
    // USART1引脚时钟使能
    RCC->APB2ENR |= 0x00000004; // 使能GPIOA的时钟	
    // USART1引脚设定
    GPIOA->CRH = 0x444444b4; // 设定PA9（USART1的发送引脚）的CNF设定为推挽式输出引脚，模式MODE设定为输出模式/速率50M；设定PA10（USART1的接收引脚）的CNF设定为浮空输入引脚，模式MODE设定为输入模式

    // USART1时钟使能
    RCC->APB2ENR |= 1 << 14; // 使能USART1的时钟
    // USART1参数设定
    USART1->BRR = 0x00000138; // 波特率设定为115200，USARTDIV = 36M/16/115200 = 19.53 , DIV_Fraction = 16*0.53 = 8.48 The nearest real number is 8，DIV_Fraction = 0x8，DIV_Mantissa = 19 = 0x13
    USART1->CR2 &= 0xcfff; // 1位停止位
    USART1->CR1 &= 0xebff; // 字节宽度为8位，无奇偶校验
    USART1->CR3 &= 0xfdff; // 无硬件数据流控制
    USART1->CR1 |= 0x000c; // 允许USART1接收和发送数据

    // USART1中断参数设定
    USART1->CR1 |= 0x0020; // 允许USART1接收非空的中断

    // USART1使能
    USART1->CR1 |= 0x2000; //USART1使能
}

void usart3Set(void) {
    //USART3引脚时钟使能
    RCC->APB2ENR |= 0x00000008; //使能GPIOB的时钟	
    //USART3引脚设定
    GPIOB->CRH = 0x44444b44; //设定PB10（USART3的发送引脚）的CNF设定为推挽式输出引脚，模式MODE设定为输出模式/速率50M；设定PB11（USART3的接收引脚）的CNF设定为浮空输入引脚，模式MODE设定为输入模式

    //USART3时钟使能
    RCC->APB1ENR |= 0x00040000; //使能USART3的时钟
    //USART3参数设定
    USART3->BRR = 0x00000138; //波特率设定为115200，USARTDIV = 36M/16/115200 = 19.53 , DIV_Fraction = 16*0.53 = 8.48 The nearest real number is 8，DIV_Fraction = 0x8，DIV_Mantissa = 19 = 0x13
    USART3->CR2 &= 0xcfff; //1位停止位
    USART3->CR1 &= 0xebff; //字节宽度为8位，无奇偶校验
    USART3->CR3 &= 0xfdff; //无硬件数据流控制
    USART3->CR1 |= 0x000c; //允许USART3接收和发送数据

    //USART3中断参数设定
    USART3->CR1 |= 0x0020; //允许USART3接收非空的中断
    //USART3->CR1 |= 0x0080; //允许USART3发送为空的中断

    //USART3使能
    USART3->CR1 |= 0x2000; //USART3使能	
}

void gpioSet(void) {
	RCC->APB2ENR |= 0x00000020; //使能GPIOD的时钟
	
	GPIOD->CRL = 0x41111444; //设定GPIOD的3、4、5、6引脚CNF设定为推挽式输出引脚，模式MODE设定为输出模式/速率10M，所有引脚上电默认值CNF是01: Floating input MODE是00: Input mode (reset state) = 0x4
	GPIOD->ODR = 0; //设定引脚的输出值为0，即低电平，熄灭LED灯
}

void nvicSet(void) {
    SCB->AIRCR = 0x05FA0500; // 优先级分组为Group: 4 Sub: 4（抢占优先级2位，响应优先级2位）

    NVIC->IP[37] = 0x30; // USART1（USART1的中断号为39）的 抢占优先级：0 响应优先级：3
    NVIC->ISER[37 >> 0x05] = 0x00000020; // USART1的中断使能，0 - 31中断源由ISER0控制; 32 - 63中断源由ISER1控制，因此可以通过右移5位决定本中断使用ISER0,还是ISER1
    NVIC->IP[39] = 0x30; // USART3（USART3的中断号为39）的 抢占优先级：0 响应优先级：3
    NVIC->ISER[39 >> 0x05] = 0x00000080; //USART3的中断使能，0 - 31中断源由ISER0控制; 32 - 63中断源由ISER1控制，因此可以通过右移5位决定本中断使用ISER0,还是ISER1
    NVIC->IP[28] = 0x80; //TIM2（TIM2中断号为28）优先级设定对应的位置IPRx: x = 28/4 = 7 (即IPR7), IPR_Ny: y = 28%4 = 0 (即IPR_N0)；设定的数值为 抢占优先级：2 响应优先级：0
    NVIC->ISER[28 >> 0x05] = 0x10000000; //TIM2的中断使能（TIM2中断号为28，对应ISER0的第28位），0 - 31中断源由ISER0控制; 32 - 63中断源由ISER1控制，因此可以通过右移5位决定本中断使用ISER0,还是ISER1
}

void USART1_IRQHandler(void) {
    if ((USART1->SR & 0x0020) != 0) { // 读取USART1 SR寄存器的RXNE位，0：无接收数据 1：有接收数据
        data2 = USART1->DR; //获取当前接收数据
        if (data2 >= 10) {
            USART3->DR = data2;
        } else {
            if (data2 == 0x02) GPIOD->ODR = 0b00001000;
            else if (data2 == 0x03) GPIOD->ODR = 0b00010000;
            else if (data2 == 0x04) GPIOD->ODR = 0b00100000;
            else if (data2 == 0x07) GPIOD->ODR = 0b01000000;
        }
    }
}

void USART3_IRQHandler(void) {
    if ((USART3->SR & 0x0020) != 0) { // 读取USART1 SR寄存器的RXNE位，0：无接收数据 1：有接收数据
        data1 = USART3->DR; //获取当前接收数据
		USART1->DR = data1;
    }
}

void tim2Set() {
    RCC->APB1ENR |= 0x00000001;
    TIM2->PSC = 3599;
    TIM2->ARR = 199;
    TIM2->CR1 &= 0xffef; //设定TIM2的DIR为0，即向上计数模式

    TIM2->SR &= 0xfffe; //TIM2的更新中断标志位UIF清零
    TIM2->DIER |= 0x0001; //TIM2的更新中断使能位UIE设置为1，允许TIM2中断

    TIM2->CR1 &= 0xfffe; //TIM2的计数使能位设为0，停止TIM2计数
}

void TIM2_IRQHandler(void) {
    TIM2->SR &= 0xfffe; //TIM2的更新中断标志位UIF清零
    sysTimeFlag = 1;
    if (key_back_num <= 101) key_back_num++;
}


int main(void) {
    sysclock();
    usart1Set();
    usart3Set();
    gpioSet();
    tim2Set();
    nvicSet(); // 系统中断的设定
    wifiSet(); // WiFi模块的初始化设定
    TIM2->CR1 |= 0x0001; //TIM2的计数使能位设为1，启动TIM2计数

    while (1) { // 每20ms执行一次
        while ((USART1->SR & 0x0040) == 0);
        if (sysTimeFlag == 1) { // 读取PB4-PB7
            sysTimeFlag = 0;
            key_data = GPIOB->IDR & 0x00f0;
            if ((key_data ^ 0x00f0) != 0) { // key_data != 0xF0 按键有变化
                key_num++; // 消抖计数器+1
                if (key_num >= 2 && (key_back_data != key_data || key_back_num >= 25)) { // 连续2次检测到稳定状态
                    if (key_data == 0x00e0) USART1->DR = 0x02; // key2按下
                    else if (key_data == 0x00d0) USART1->DR = 0x03; // key3
                    else if (key_data == 0x00b0) USART1->DR = 0x04; 
                    else if (key_data == 0x0070) USART1->DR = 0x07;
                    key_back_data = key_data; // 保存当前按键
                    key_back_num = 0; // 重置防连按计时
                    key_num = 0; // 重置消抖计数器
                }
            }
        }
    }
}
