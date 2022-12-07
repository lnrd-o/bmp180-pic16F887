#include <16F887.h>
#device ADC=10

#FUSES NOWDT,NOBROWNOUT,NOLVP   

#use delay(crystal=20000000)
#use RS232(baud=9600,xmit=PIN_C6,rcv=PIN_C7,PARITY=N, BITS =8, STOP=1, TIMEOUT=1000, ERRORS)   //Inicialización del puerto serie en el uC
#use i2c(master, FAST,sda=PIN_C4, scl=PIN_C3)   

#include"bmp180.h"



void main()
{  
   BMP180Calibracion();
   while(1){
      float temperaturaSensor = BMP180Temperatura();
      printf("%.1g\n\r",temperaturaSensor);
      delay_ms(1000);

   }
} 