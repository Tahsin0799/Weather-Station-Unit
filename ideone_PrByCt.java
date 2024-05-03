#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <TimerOne.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>


float lat = 23.7810367, lon = 90.378193;

SoftwareSerial gpsSerial(4, 3); //rx,tx

TinyGPS gps; // create gps object




#define    MPU9250_ADDRESS            0x68
#define    MAG_ADDRESS                0x0C
#define    GYRO_FULL_SCALE_250_DPS    0x00
#define    GYRO_FULL_SCALE_500_DPS    0x08
#define    GYRO_FULL_SCALE_1000_DPS   0x10
#define    GYRO_FULL_SCALE_2000_DPS   0x18
#define    ACC_FULL_SCALE_2_G        0x00
#define    ACC_FULL_SCALE_4_G        0x08
#define    ACC_FULL_SCALE_8_G        0x10
#define    ACC_FULL_SCALE_16_G       0x18
#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10
#define SEALEVELPRESSURE_HPA (1013.25)


Adafruit_BME280 bme;
unsigned long delayTime;

#include <LoRa.h>


String PARAMETER;


void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.endTransmission();
  Wire.requestFrom(Address, Nbytes);
  uint8_t index = 0;
  while (Wire.available())
    Data[index++] = Wire.read();
}

void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.write(Data);
  Wire.endTransmission();
}

// Initial time
long int ti;
volatile bool intFlag = false;







void setup()
{
  Serial.begin(9600);
  //bme check
  bool status;
  // default settings
  // (you can also pass in a Wire library object like &Wire2)

  status = bme.begin(0x76);//0x0C

  if (!status)
  {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
  Serial.println("-- Default Test --");
  delayTime = 1000;
  Serial.println();
  //lora check
  while (!Serial);
  Serial.println("LoRa Sender");

  if (!LoRa.begin(433E6))   // or 915E6, the MHz speed of your module
  {
    Serial.println("Starting LoRa failed!");
    while (1);
  }

  //gps

  

  gpsSerial.begin(9600); // connect gps sensor


  //mpu

  Wire.begin();
  I2CwriteByte(MPU9250_ADDRESS, 29, 0x06);
  I2CwriteByte(MPU9250_ADDRESS, 26, 0x06);
  I2CwriteByte(MPU9250_ADDRESS, 27, GYRO_FULL_SCALE_1000_DPS);
  I2CwriteByte(MPU9250_ADDRESS, 28, ACC_FULL_SCALE_4_G);
  I2CwriteByte(MPU9250_ADDRESS, 0x37, 0x02);
  I2CwriteByte(MAG_ADDRESS, 0x0A, 0x16);
  pinMode(13, OUTPUT);
  Timer1.initialize(10000);         // initialize timer1, and set a 1/2 second period
  Timer1.attachInterrupt(callback);  // attaches callback() as a timer overflow interrupt
  ti = millis();
}

long int cpt = 0;
void callback()
{
  intFlag = true;
  digitalWrite(13, digitalRead(13) ^ 1);
}

void loop()
{
  int count = 0;
  String PARAMETER = "";

  LoRa.beginPacket();


  PARAMETER += "t " + String (bme.readTemperature()) + " "; //String(13);
  count++;


  /*Serial.print("Temperature = ");
    Serial.print(1.8 * bme.readTemperature() + 32);
    Serial.print(" *F");*/

  PARAMETER += "p " + String ( bme.readPressure() / 100.0F) + " ";
  count++;

  PARAMETER += "a " + String (bme.readAltitude(SEALEVELPRESSURE_HPA)) + " ";
  count++;

  PARAMETER += "h " + String ( bme.readHumidity()) + " ";
  count++;



  while (!intFlag);
  intFlag = false;
  //    Serial.print (millis() - ti, DEC);
  //    Serial.print ("\t");
  // Read accelerometer and gyroscope
  uint8_t Buf[14];
  I2Cread(MPU9250_ADDRESS, 0x3B, 14, Buf);
  // Create 16 bits values from 8 bits data
  // Accelerometer
  int16_t ax = -(Buf[0] << 8 | Buf[1]);
  int16_t ay = -(Buf[2] << 8 | Buf[3]);
  int16_t az = Buf[4] << 8 | Buf[5];
  // Gyroscope
  int16_t gx = -(Buf[8] << 8 | Buf[9]);
  int16_t gy = -(Buf[10] << 8 | Buf[11]);
  int16_t gz = Buf[12] << 8 | Buf[13];
  // Display values

  // Accelerometer
 
  PARAMETER += "ax " + String ( ax, DEC) + " ";
  count++;

  PARAMETER += "ay " + String ( ay, DEC) + " ";
  count++;
 
  PARAMETER += "az " + String ( az, DEC) + " ";
  count++;

  // Gyroscope
  
  PARAMETER += "gx " + String ( gx, DEC) + " " ;
  count++;

  PARAMETER += "gy " + String ( gy, DEC) + " ";
  count++;

  PARAMETER += "gz " + String ( gz, DEC) + " ";
  count++;



  // :::  Magnetometer ::
  // Read register Status 1 and wait for the DRDY: Data Ready
  uint8_t ST1;
  do
  {
    I2Cread(MAG_ADDRESS, 0x02, 1, &ST1);
  }
  while (!(ST1 & 0x01));
  uint8_t Mag[7];
  I2Cread(MAG_ADDRESS, 0x03, 7, Mag);
  int16_t mx = -(Mag[3] << 8 | Mag[2]);
  int16_t my = -(Mag[1] << 8 | Mag[0]);
  int16_t mz = -(Mag[5] << 8 | Mag[4]);
  // Magnetometer

  PARAMETER += "mx " + String ( mx + 200, DEC) + " " ;
  count++;

  PARAMETER += "my " + String ( my - 70, DEC) + " " ;
  count++;

  PARAMETER += "mz " + String ( mz - 700, DEC) + " " ;
  count++;
  


//gps data
    String latitude = String(lat, 6);
    String longitude = String(lon, 6);
    String ggps="latitud "+latitude +"N longitude "  + longitude+ "E" ;

  while (gpsSerial.available()) { // check for gps data
    if (gps.encode(gpsSerial.read())) // encode gps data
    {
      gps.f_get_position(&lat, &lon); // get latitude and longitude

    latitude = String(lat, 6);
    longitude = String(lon, 6);
    ggps="latitud"+latitude +"N ccc"  + longitude+ "E" ;
    

    }
  }
 



//


  if (count == 13) {
      //PARAMETER+=ggps;
    Serial.println(PARAMETER);
    LoRa.print(PARAMETER);
    LoRa.print("\n");
  }
  LoRa.endPacket();

  //delay(1000);

}