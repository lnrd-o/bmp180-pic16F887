//************************************************ 
//  BMP180 Barometric Pressure Sensor 
// 
//  - Hoja de datos: https://cdn-shop.adafruit.com/valorsheets/BST-BMP180-DS000-09.pdf
// 
//Comando a usar:
//#use i2c(master, FAST,sda=PIN_C4, scl=PIN_C3)

#include <math.h> 

const int8 OVS_S = 3; // 3.3.1 Hardware pressure sampling accuracy modes (0 -> ultra low power, 1 -> standard, 2 -> high resolution, 3-> ultra hi-resolution) [P.12]

#define BMP180_DIRECCION 0xEE          // Direccion I2C del BMP180
#define CORRECTION_DE_PRESION   70.6      //presión del lugar en donde estas en mBar    


// Coeficientes para calibracion [P13]
signed int16 ac1; 
signed int16 ac2; 
signed int16 ac3; 
int16 ac4; 
int16 ac5; 
int16 ac6; 
signed int16 b1; 
signed int16 b2; 
signed int16 mb; 
signed int16 mc; 
signed int16 md; 

// factores de multiplicación flotantes 
float _c3; 
float _c4; 
float _b1; 
float _c5; 
float _c6; 
float _mc; 
float _md; 

// constantes complejas
float _x0; 
float _x1; 
float _x2; 
float _y0; 
float _y1; 
float _y2; 
float _p0; 
float _p1; 
float _p2; 

float _s;     
float _Temp;  

 
int8 BMP180LeerByte(int8 direccion) { 
   int8 valor; 
   i2c_start(); 
   i2c_write(BMP180_DIRECCION); 
   i2c_write(direccion); 
   i2c_start(); 
   i2c_write(BMP180_DIRECCION | 0x01 ); 
   valor=i2c_read(0); 
   i2c_stop(); 
   return(valor); 
} 


int16 BMP180LeerEntero(int8 direccion) 
{ 
int8 msb, lsb; 
int16 temp; 

   i2c_start(); 
   i2c_write(BMP180_DIRECCION); 
   i2c_write(direccion); 
   i2c_start(); 
   i2c_write(BMP180_DIRECCION | 0x01 ); 
   msb = i2c_read(); 
   lsb = i2c_read(0); 
   i2c_stop(); 
   temp = make16(msb, lsb); 
   return(temp); 
} 


//---------------------------------------------- 
void BMP180EscribirByte(int8 direccion, int8 valor) 
//---------------------------------------------- 
{ 
   i2c_start(); 
   i2c_write(BMP180_DIRECCION); 
   i2c_write(direccion); 
   i2c_write(valor); 
   i2c_stop(); 
} 


//---------------------------------------------- 
void BMP180Calibracion() 
//---------------------------------------------- 
{ 
   // Leer los valores del EEPROM 
   ac1 = BMP180LeerEntero(0xAA); 
   ac2 = BMP180LeerEntero(0xAC); 
   ac3 = BMP180LeerEntero(0xAE); 
   ac4 = BMP180LeerEntero(0xB0); 
   ac5 = BMP180LeerEntero(0xB2); 
   ac6 = BMP180LeerEntero(0xB4); 
   b1  = BMP180LeerEntero(0xB6); 
   b2  = BMP180LeerEntero(0xB8); 
   mb  = BMP180LeerEntero(0xBA); 
   mc  = BMP180LeerEntero(0xBC); 
   md  = BMP180LeerEntero(0xBE); 

    // calcular los valores
   _c3 = 0.0048828125 * ac3;            // 160 * (-15)^2 * ac3; 
   _c4 = 0.000000030517578125 * ac4;    // 1E-3 * (-15)^2 * ac4; 
   _c5 = 0.00000019073486328125 * ac5;  // ((-15)^2/160) * ac5; 
   _c6 = (float)ac6; 
   _b1 = 0.00002384185791015625 * b1;   // 25600 * (-30)^2 * b1; 
   _mc = 0.08 * mc;                     // ((11)^2 / 25600) * mc; 
   _md = (float)md / 160; 
    
   // calcula constantes de los polinomios 
   _x0 = (float)ac1; 
   _x1 = 0.01953125 * ac2;             // 160 * (-13)^2 * ac2; 
   _x2 = 0.000762939453125 * b2;       // 25600 * (-25)^2 * b2; 
   _y0 = _c4 * 32768;                  //_c4 * (15)^2; 
   _y1 = _c4 * _c3; 
   _y2 = _c4 * _b1; 
   _p0 = 2.364375;  
   _p1 = 0.992984; 
   _p2 = 0.000004421;    
} 


// Leer la primer temperatura invalidada 
//---------------------------------------------- 
int16 BMP180LeerTempPrimera() 
//---------------------------------------------- 
{ 
int16 ut; 
  
  BMP180EscribirByte(0xF4, 0x2E); 
  delay_ms(5); 
  ut = BMP180LeerEntero(0xF6); 
  return((float)ut); 
} 


// Leer el primer valor de presion invalido
//---------------------------------------------- 
int32 BMP180LeerPresionInvalida() 
//---------------------------------------------- 
{ 
int8 msb, lsb, xlsb; 
float p; 
  
  BMP180EscribirByte(0xF4, (0x34 + (OVS_S<<6)) ); 
   
  switch (OVS_S) 
  { 
     case 0: delay_ms(5);  break; 
     case 1: delay_ms(8);  break; 
     case 2: delay_ms(14); break; 
     case 3: delay_ms(26); break; 
  }    
  
   msb  = BMP180LeerByte(0xF6); 
   lsb  = BMP180LeerByte(0xF7); 
   xlsb = BMP180LeerByte(0xF8); 
   p = (256*msb) + lsb + (xlsb/256); 
   return(p); 
} 


//---------------------------------------------- 
float BMP180dameTemperatura(float _tu) 
//---------------------------------------------- 
{ 
float alpha, T; 

   alpha = _c5 * (_tu - _c6); 
   T = alpha + (_mc/(alpha + _md)); 
   _s = T - 25; 
   return(T); 
}    


//---------------------------------------------- 
float BMP180damePresion(float _pu) 
//---------------------------------------------- 
{ 
float x, y, z; 
float P; 

   x = _x2*_s*_s + _x1*_s + _x0; 
   y = _y2*_s*_s + _y1*_s + _y0; 
   z = ((float)_pu - x) / y; 
   P = _p2*z*z + _p1*z + _p0; 
   P += CORRECTION_DE_PRESION; 
   return(P); 
} 


//---------------------------------------------- 
float BMP180Presion(boolean getTemp) 
//---------------------------------------------- 
{ 
   if (getTemp) 
      _Temp = BMP180dameTemperatura(BMP180LeerTempPrimera());  // creates _s required for pressure calculation 
   return(BMP180damePresion(BMP180LeerPresionInvalida())); 
} 


//---------------------------------------------- 
float BMP180Temperatura(void) 
//---------------------------------------------- 
{ 
   _Temp = BMP180dameTemperatura(BMP180LeerTempPrimera()); 
   return(_Temp); 
} 
