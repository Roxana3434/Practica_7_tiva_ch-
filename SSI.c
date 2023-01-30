
#include "lib/include.h"
extern void Configurar_SSI2(void)
{
    //se activa el Modulo 2 SSI2
    SYSCTL->RCGCSSI |= (1<<2); 

    ////Habilita GPIO Puerto D
    SYSCTL->RCGCGPIO |= (1<<3);

    //selector es salida = 1
    GPIOD_AHB->DIR |= (0<<3) | (1<<2) | (1<<1) | (0<<0); 

    // Pines usados con función alternativa (SSI) pag. 770
    //                 SCLK     CS    MOSI   MISO
    GPIOD_AHB->AFSEL = (1<<3)|(1<<2)|(1<<1)|(1<<0); //antes del 4 al 7

    // Combinanando pag. 787 y 1808  colocar valor de 15
    GPIOD_AHB->PCTL = (GPIOD_AHB->PCTL&0xFFFF0000) | 0x0000FFFF; 
    //GPIOD_AHB->PCTL &= 0xFFFF0000;
    //GPIOD_AHB->PCTL |= 0x0000FFFF; // tabla p.688

    //Se establecen como pines digitales
    //                 MISO    MOSI    CS    SCLK
    GPIOD_AHB->DEN |= (1<<0)|(1<<1)|(1<<2)|(1<<3);
    
    //registrar CS
    GPIOD_AHB->DATA |= (1<<5); 

    //GPIOB->PUR |= (0<<7)|(0<<6)|(0<<5)|(0<<4);
    //GPIOB->PDR |= (0<<7)|(0<<6)|(0<<5)|(0<<4);
    //GPIOB->AMSEL |= (0<<7)|(0<<6)|(0<<5)|(0<<4);
    
    //SSE=0 deshabilitar modulo
    SSI2->CR1 = (0<<1); // El CR1 indica los bits mandar, y config. la velocidad del reloj de transmision

    //Se indica el modo de trabajo pag. 1248 (Maestro o Esclavo)
    SSI2->CR1 = (0<<2); //MS = 0 modo maestro

    //Se indica el trabajo con el system clock = 50MHz, pag 1386
    SSI2->CC = (0x0<<0); 

    //Pag. 1242 config. CPSR 
    //SSInClk = SysClk / (CPSDVSR * (1 + SCR))  
    //2 500 000 = 50 000 000/(2*(1+SCR)) 
    // SCR = (SysClk/SSInClk*CPSDVSR) - 1 = 9  -> CPSDVSR y SSInClk son al azar
    // SCR = (50 000 000/2 500 000*2) - 1 = 9
    SSI2->CPSR =0x2; // 2.5 MHZ
   
    //Se indica el numero de datos a mandar pag. 1369
    SSI2->CR0 = (0x9<<8)| (0x0<<6) | (0x1<<4) | (0xF<<0); //Se colocan los 12 correspondientes al DAC 
    // SSI2->CR0 = (0x9<<8) | 0x07; // datos de 8 bits, (0x9<<8) es fijo, para cualquier # de datos que se quiera mandar 
    
    //SSE=1 habilitar modoulo p.961 (0x02)
    SSI2->CR1 |= (1<<1); 
}

extern void SPI_write(uint16_t value) 
{
    SSI2->DR = value;
    while ((SSI2->SR & (1<<0)) == 0);
    
    
}

extern void DAC_Output(unsigned int valueDAC)
{
    char temp;
    GPIOD_AHB->DATA &= ~(1<<3); // CS = 0 se niega
    
    // envío high byte (los primeros 8 bits)
    temp = (valueDAC >> 8) & 0x0F;      // [11...8] a [3...0]
    temp |= 0x30;
    SPI_write(temp);
    
    // envío low byte
    temp = valueDAC;
    SPI_write(temp);        // mando los ultimos 8 bits

    GPIOD_AHB->DATA |= (1<<3);  //CS = 1

}

/*extern void SPI_write(uint8_t data)
{
    while (SSI2->SR & 0x2)
    {
        SSI2->DR = (uint16_t)data;
    }
    
}

extern void SPI_write_data(uint8_t reg, uint8_t data)
{
    GPIOD_AHB->DATA &= ~(1<<3); // CS = 0 se niega
    SPI_write(reg & ~0x80); //escribir registro + MSB igualado a cero
    SPI_write(data);
    GPIOD_AHB->DATA |= (1<<3); //CS = 1
}

extern uint8_t SPI_read(void)
{
    uint8_t data = 0;
    while ((SSI2->SR & 0x10) == 0x10); // espera por busy bit
    data = SSI2->DR;
    return data;
}

extern uint8_t SPI_read_data(uint8_t reg)
{
    uint8_t data = 0;
    GPIOD_AHB->DATA &= ~(1<<3); // CS = 0
    SPI_write(reg | 0x80); // escribe registro + MSB
    SPI_write(0x00); //escribir dato para generar señal de reloj
    data = SPI_read(); //leer dato
    GPIOD_AHB->DATA |= (1<<3); //CS = 1
    return data;
}*/
