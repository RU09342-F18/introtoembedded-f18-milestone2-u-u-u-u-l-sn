
#include <msp430.h>
#include <math.h>

int current_TEMP = 0;
float currTemp = 1.0;
int newTemp = 100;
float tempDiff = 0.0;
float ADC_Voltage = 0;
float ADC_Temp = 0;
float perBit = 3.3/4096;

void outputSetup(void)
{
    //output for PWM
    P1DIR |= BIT2;                              // Sets pin 1.2 to the output direction
    P1SEL |= BIT2;                              // BIT2 = TA0.1 output
    P1OUT &= ~BIT2;                             // Turn off

}


void PWMSetup(void)
{
    TA0CTL = TASSEL_2 | MC_1 | TACLR;           // SMCLK set to UP mode, clear TAR
    TA0CCR0 = 255;                              // PWM Period

    //TA0CCR1 = 25;                               // TA0 duty cycle is ~10%
    TA0CCR1 = 0;                                // TA0 duty cycle is 100%
    TA0CCTL1 = OUTMOD_7;                        // Reset/Set
}


void UARTSetup(void)
{
    P4SEL |= BIT4 | BIT5;                       // BIT4 = TXD output || BIT5 = RXD input
    //P3SEL |= BIT4 | BIT5;                       // BIT3 = TXD output || BIT5 = RXD input
    UCA1CTL1 |= UCSWRST;                        // State Machine Reset + Small Clock Initialization
    UCA1CTL1 |= UCSSEL_2;                       // Sets USCI Clock Source to SMCLK
    UCA1BR0 = 104;                              // Setting the Baud Rate to be 9600
    UCA1BR1 = 0;                                // ^
    UCA1MCTL |= UCBRS_1 + UCBRF_0;
    UCA1CTL1 &= ~UCSWRST;                      // Initialize USCI State Machine
    UCA1IE |= UCRXIE;                           // Enable USCI_A0 RX interrupt
}


void ADCSetup(void)
{

  ADC12CTL0 = ADC12SHT02 + ADC12ON;         // Sampling time, ADC12 on
  ADC12CTL1 = ADC12SHP;                     // Use sampling timer
  ADC12IE = 0x01;                           // Enable interrupt
  ADC12CTL0 |= ADC12ENC;
  P6SEL |= 0x01;                            // P6.0 ADC option select
  P6DIR &= ~BIT0;                           // P6.0 input
  P1DIR |= 0x01;                            // P1.0 output
}


void main(void)
{
    WDTCTL = WDTPW | WDTHOLD;                   // Stop Watchdog Timer
    outputSetup();
    PWMSetup();
    UARTSetup();
    ADCSetup();

    __bis_SR_register(GIE);                 // LPM0, ADC12_ISR will force exit
    while (1)
    {
      ADC12CTL0 |= ADC12SC;                   // Start sampling/conversion
      __no_operation();                       // For debugger
    }
}



#pragma vector=USCI_A1_VECTOR
__interrupt void USCI_A1_ISR(void)
{

    if (UCA1IFG & UCRXIFG) {
        UCA1IFG &= ~UCRXIFG;                    // Clear the RX interrupt flag
        newTemp = (int)UCA1RXBUF;                    // Read Data from UART
    }

}



#pragma vector = ADC12_VECTOR
__interrupt void ADC12_ISR(void)
{
  switch(__even_in_range(ADC12IV,34))
  {
  case  6:                                      // Vector  6:  ADC12IFG0
      ADC_Voltage = ADC12MEM0 * perBit;
          ADC_Temp = (ADC_Voltage - .424)* 160;
          current_TEMP = (int)ADC_Temp;
      tempDiff = current_TEMP - newTemp;

      if (tempDiff > 1 || tempDiff < -1)
      {
          if (current_TEMP > newTemp)
          {
              if (TA0CCR1 < 255)
              {
                  TA0CCR1 += 5;                 //increments PWM cycle by constant
              }
          }
          else if (current_TEMP < newTemp)
          {
              if (TA0CCR1 > 0)
              {
                  TA0CCR1 -= 5;                 //decrements PWM cycle by constant
              }
          }
      }
      while (!(UCA0IFG&UCTXIFG));               // USCI_A0 TX buffer ready?
      UCA1IFG &= ~UCTXIFG;                      // Clear the TX interrupt flag
      UCA1TXBUF = (int)current_TEMP & 0x0FF;        //Transmit currTemp
  default: break;
  }
}
