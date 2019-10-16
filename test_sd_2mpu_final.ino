#include <SD.h>
#include <Wire.h>

 
//Direccion I2C de la IMU
#define MPU1 0x68
#define MPU2 0x69
 
//Ratios de conversion
#define A_R 4096
#define G_R 65.5
 
//MPU-6050
struct MPU
{
int16_t Ax;
int16_t Ay;
int16_t Az;
//int16_t Gx;
int16_t Gy;
int16_t Gz;
};
struct OFFSET
{
float Ax;
float Ay;
float Az;
float Gx;
float Gy;
float Gz;
};
  
OFFSET OffsetMuslo;
OFFSET OffsetPierna;
  
boolean flag = false;

const int button1 = 2;
const int button2 = 3;
const int CS = 4;
const int led1 = 5;
const int led2 = 6;
unsigned int lastInt1, lastInt2;
unsigned long timer1, timer2, timer3;

File DAT;
File CSV;

void setup() 
{
  pinMode(button1,INPUT);
  pinMode(button2,INPUT);
  pinMode(CS,OUTPUT);
  pinMode(led1,OUTPUT);
  pinMode(led2,OUTPUT);
  digitalWrite(led1,LOW);
  digitalWrite(led2,LOW);
  
  Wire.begin();
  TWBR = 12;

  Wire.beginTransmission(MPU1);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  Wire.beginTransmission(MPU1);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission(true);
  Wire.beginTransmission(MPU1);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission(true);

  delay(10);

  Wire.beginTransmission(MPU2);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  Wire.beginTransmission(MPU2);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission(true);
  Wire.beginTransmission(MPU2);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission(true);
  
   Serial.begin(9600); 
  
  if(!SD.begin(4))
  {
    Serial.println("No se pudo inicializar");
    return;
  }
  
  Serial.println("Inicializacion exitosa");
  
  if(SD.exists("datalog.dat"))
  {
    SD.remove("datalog.dat");
  }
  DAT = SD.open("datalog.dat", FILE_WRITE);

  if(SD.exists("datalog.csv"))
  {
    SD.remove("datalog.csv");
  }
}
  
void loop() 
{
  int buttonState1 = digitalRead(button1);
  if(buttonState1 == HIGH && flag == false)
  {
    if(millis() > lastInt1 + 150)
    {
      //Serial.println("Calibrando Muslo y Pierna...");
      digitalWrite(led1,HIGH);
      
      calibracion();

      //Serial.println("Calibracion terminada");
      
      lastInt1 = millis();
      flag = true;

      digitalWrite(led1,LOW);

      delay(500);
    }
  }
   if(buttonState1 == HIGH && flag == true)
  {
    if(millis() > lastInt1 + 150)
    {
     digitalWrite(led1,HIGH);
     delay(500);
     digitalWrite(led1,LOW);
     
     funcionWrite();
     lastInt1 = millis();
     flag = false;

     int buttonState3;
     while(1)
     {
     digitalWrite(led1,HIGH);
     delay(500);
     digitalWrite(led1,LOW);
     delay(500);
     buttonState3 = digitalRead(button2);
     if(buttonState3 == HIGH) break;
     }
    }
  }
  int buttonState2 = digitalRead(button2);
  if(buttonState2 == HIGH)
  {
    if(millis() > lastInt2 + 150)
    {
    digitalWrite(led2,HIGH);
    delay(500);
    digitalWrite(led2,LOW);
     
    funcionRead();
    lastInt2 = millis();

    while(1)
    {
    digitalWrite(led2,HIGH);
    delay(500);
    digitalWrite(led2,LOW);
    delay(500);
    buttonState1 = digitalRead(button1);
    if(buttonState1 == HIGH) break;
    }
    }
  }
}

void calibracion()
{
  MPU Muslo;
  MPU Pierna;
  for(int i = 0; i < 5000; i++)
  {
    Wire.beginTransmission(MPU1);
    Wire.write(0x3B); //Pedir el registro 0x3B - corresponde al AcX
    Wire.endTransmission(false);
    Wire.requestFrom(MPU1,6,true); //A partir del 0x3B, se piden 6 registros
    OffsetMuslo.Ax += Wire.read()<<8|Wire.read();
    OffsetMuslo.Ay += Wire.read()<<8|Wire.read(); 
    OffsetMuslo.Az += Wire.read()<<8|Wire.read(); 
    Wire.endTransmission(true);

    Wire.beginTransmission(MPU1);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU1,6,true);
    OffsetMuslo.Gx += Wire.read()<<8|Wire.read();
    OffsetMuslo.Gy += Wire.read()<<8|Wire.read();
    OffsetMuslo.Gz += Wire.read()<<8|Wire.read();
    Wire.endTransmission(true);

    Wire.beginTransmission(MPU2);
    Wire.write(0x3B); //Pedir el registro 0x3B - corresponde al AcX
    Wire.endTransmission(false);
    Wire.requestFrom(MPU2,6,true); //A partir del 0x3B, se piden 6 registros
    OffsetPierna.Ax += Wire.read()<<8|Wire.read();
    OffsetPierna.Ay += Wire.read()<<8|Wire.read(); 
    OffsetPierna.Az += Wire.read()<<8|Wire.read(); 
    Wire.endTransmission(true);

    Wire.beginTransmission(MPU2);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU2,6,true);
    OffsetPierna.Gx += Wire.read()<<8|Wire.read();
    OffsetPierna.Gy += Wire.read()<<8|Wire.read();
    OffsetPierna.Gz += Wire.read()<<8|Wire.read();
    Wire.endTransmission(true);
  }
  OffsetMuslo.Ax /= 5000.0;
  OffsetMuslo.Ay /= 5000.0;
  OffsetMuslo.Az /= 5000.0;
  OffsetMuslo.Gx /= 5000.0;
  OffsetMuslo.Gy /= 5000.0;
  OffsetMuslo.Gz /= 5000.0;

  OffsetPierna.Ax /= 5000.0;
  OffsetPierna.Ay /= 5000.0;
  OffsetPierna.Az /= 5000.0;
  OffsetPierna.Gx /= 5000.0;
  OffsetPierna.Gy /= 5000.0;
  OffsetPierna.Gz /= 5000.0;

  Muslo.Ax = (int16_t)OffsetMuslo.Ax;
  Muslo.Ay = (int16_t)OffsetMuslo.Ay; 
  Muslo.Az = (int16_t)OffsetMuslo.Az;
  //Muslo.Gx = (int16_t)OffsetMuslo.Gx;
  Muslo.Gy = (int16_t)OffsetMuslo.Gy;
  Muslo.Gz = (int16_t)OffsetMuslo.Gz;
  
  Pierna.Ax = (int16_t)OffsetPierna.Ax;
  Pierna.Ay = (int16_t)OffsetPierna.Ay; 
  Pierna.Az = (int16_t)OffsetPierna.Az;
  //Pierna.Gx = (int16_t)OffsetPierna.Gx;
  Pierna.Gy = (int16_t)OffsetPierna.Gy;
  Pierna.Gz = (int16_t)OffsetPierna.Gz;

  DAT.write((const uint8_t*)&Muslo, sizeof(Muslo));
  DAT.write((const uint8_t*)&Pierna, sizeof(Pierna));
}

void funcionWrite()
{
  timer1 = 0;
  timer2 = 0;
  
  MPU Muslo;
  MPU Pierna;

  timer3 = millis();
  
  for(int j = 0 ;j < 1000; j++)
  { 
    timer1 = micros();
    
    Wire.beginTransmission(MPU1);
    Wire.write(0x3B); //Pedir el registro 0x3B - corresponde al AcX
    Wire.endTransmission(false);
    Wire.requestFrom(MPU1,6,true); //A partir del 0x3B, se piden 6 registros
    Muslo.Ax = Wire.read()<<8|Wire.read(); Muslo.Ax -= (int16_t)OffsetMuslo.Ax; Muslo.Ax -= A_R;
    Muslo.Ay = Wire.read()<<8|Wire.read(); Muslo.Ay -= (int16_t)OffsetMuslo.Ay; 
    Muslo.Az = Wire.read()<<8|Wire.read(); Muslo.Az -= (int16_t)OffsetMuslo.Az; 
    Wire.endTransmission(true);
      
    Wire.beginTransmission(MPU1);
    Wire.write(0x45);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU1,6,true);
    //Muslo.Gx = Wire.read()<<8|Wire.read(); Muslo.Gx -= (int16_t)OffsetMuslo.Gx;
    Muslo.Gy = Wire.read()<<8|Wire.read(); Muslo.Gy -= (int16_t)OffsetMuslo.Gy;
    Muslo.Gz = Wire.read()<<8|Wire.read(); Muslo.Gz -= (int16_t)OffsetMuslo.Gz;
    Wire.endTransmission(true);

    Wire.beginTransmission(MPU2);
    Wire.write(0x3B); //Pedir el registro 0x3B - corresponde al AcX
    Wire.endTransmission(false);
    Wire.requestFrom(MPU2,6,true); //A partir del 0x3B, se piden 6 registros
    Pierna.Ax = Wire.read()<<8|Wire.read(); Pierna.Ax -= (int16_t)OffsetPierna.Ax; Pierna.Ax -= A_R;
    Pierna.Ay = Wire.read()<<8|Wire.read(); Pierna.Ay -= (int16_t)OffsetPierna.Ay; 
    Pierna.Az = Wire.read()<<8|Wire.read(); Pierna.Az -= (int16_t)OffsetPierna.Az; 
    Wire.endTransmission(true);

    Wire.beginTransmission(MPU2);
    Wire.write(0x45);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU2,6,true);
    //Pierna.Gx = Wire.read()<<8|Wire.read(); Pierna.Gx -= (int16_t)OffsetPierna.Gx;
    Pierna.Gy = Wire.read()<<8|Wire.read(); Pierna.Gy -= (int16_t)OffsetPierna.Gy;
    Pierna.Gz = Wire.read()<<8|Wire.read(); Pierna.Gz -= (int16_t)OffsetPierna.Gz;
    Wire.endTransmission(true);
    
    DAT.write((const uint8_t*)&Muslo, sizeof(Muslo));
    DAT.write((const uint8_t*)&Pierna, sizeof(Pierna));
    
    while(micros() - timer1 < 36000)
    { 
    }  
    
    if(micros() - timer1 > 37000)
    {
    //Serial.print("Error de Downsampling en la muestra numero: ");
    //Serial.println(j);
    //Serial.println(micros() - timer1);
    }
  }
  DAT.close(); 
  Serial.println("Fin");
  }


void funcionRead()
{
  DAT = SD.open("datalog.dat", FILE_READ);//abrimos  el archivo 
  CSV = SD.open("datalog.csv", FILE_WRITE);//abrimos  el archivo
  
    while (DAT.available()) 
    {
      MPU Muslo;
      MPU Pierna;
      
      DAT.read((const uint8_t*)&Muslo, sizeof(Muslo));
      DAT.read((const uint8_t*)&Pierna, sizeof(Pierna));
      
      CSV.print(Muslo.Ax);
      CSV.print(",");
      CSV.print(Muslo.Ay);
      CSV.print(",");
      CSV.print(Muslo.Az);
      CSV.print(",");
      //CSV.print(Muslo.Gx);
      //CSV.print(",");
      CSV.print(Muslo.Gy);
      CSV.print(",");
      CSV.print(Muslo.Gz);
      CSV.print(",");
      CSV.print(Pierna.Ax);
      CSV.print(",");
      CSV.print(Pierna.Ay);
      CSV.print(",");
      CSV.print(Pierna.Az);
      CSV.print(",");
      //CSV.print(Pierna.Gx);
      //CSV.print(",");
      CSV.print(Pierna.Gy);
      CSV.print(",");
      CSV.println(Pierna.Gz);
      
    }
    DAT.close();
    CSV.close();
    /*
    CSV = SD.open("datalog.csv", FILE_READ);//abrimos  el archivo
    
    Serial.println("datalog.csv:");
    while (CSV.available()) 
    {
      Serial.write(CSV.read());
    }
    
    DAT.close();
    CSV.close();
    Serial.println("Fin");
   */
}






