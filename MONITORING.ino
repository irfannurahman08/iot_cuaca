/*
 * SD Card port
 ** MOSI - pin 11
 ** MISO - pin 12
 ** CLK - pin 13
 ** CS - pin 4 (for MKRZero SD: SDCARD_SS_PIN)
*/
#include <LiquidCrystal_I2C.h>
#include <DS3231.h> //mengincludekan library DS3231
#include <Wire.h>
#include <DHT.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include "RTClib.h"
#include <SD.h>

File myFile;

Adafruit_BMP280 bmp; // I2C


LiquidCrystal_I2C lcd(0x27, 16, 2); // GANTI 0x3F Ke 0x27 kalau LCD ga muncul

DHT dht(5, DHT11); //Pin, Jenis DHT

const int sensor_tanah = A1;
const int sensor_hujan = A2;

RTC_DS3231 rtc;
DateTime now;
// Inisialisasi struktur waktu

String jam, menit, detik, hari, bulan, tahun;
//Pin interrupt https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/

// Untuk Angin Gunakan pin D2 pada Arduino
//================================================
volatile byte rpmcount; // count signals
volatile unsigned long last_micros;
unsigned long timeold;
unsigned long timemeasure = 25.00; // seconds
int timetoSleep = 1;               // minutes
unsigned long sleepTime = 15;      // minutes
unsigned long timeNow;
int GPIO_pulse = 2; // Arduino = D2
float rpm, rps;     // frequencies
float radius = 0.1; // meters - measure of the lenght of each the anemometer wing
float velocity_kmh; // km/h
float velocity_ms;  //m/s
float omega = 0;    // rad/s
float calibration_value = 2.0;
//================================================

// Untuk Curah Hujan Gunakan pin D3 pada Arduino
//================================================
const int pin_interrupt = 3; // Arduino = D2
long int jumlah_tip = 0;
long int temp_jumlah_tip = 0;
float curah_hujan = 0.00;
float curah_hujan_per_menit = 0.00;
float curah_hujan_per_jam = 0.00;
float curah_hujan_per_hari = 0.00;
float curah_hujan_hari_ini = 0.00;
float temp_curah_hujan_per_menit = 0.00;
float temp_curah_hujan_per_jam = 0.00;
float temp_curah_hujan_per_hari = 0.00;
float milimeter_per_tip = 0.70;
String cuaca = "Berawan";
volatile boolean flag = false;
//================================================
//custom karakter
 
void hitung_curah_hujan()
{
  flag = true;
}

void rpm_anemometer()
{
  if (long(micros() - last_micros) >= 5000)
  { // time to debounce measures
    rpmcount++;
    last_micros = micros();
  }
}



void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  //====================== SETUP Curah Hujan ================================================
  pinMode(pin_interrupt, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pin_interrupt), hitung_curah_hujan, FALLING); // Akan menghitung tip jika pin berlogika dari HIGH ke LOW
  //====================== END SETUP Curah Hujan ================================================

  //====================== SETUP Angin ================================================
  pinMode(GPIO_pulse, INPUT_PULLUP);
  digitalWrite(GPIO_pulse, LOW);

  detachInterrupt(digitalPinToInterrupt(GPIO_pulse));                         // force to initiate Interrupt on zero
  attachInterrupt(digitalPinToInterrupt(GPIO_pulse), rpm_anemometer, RISING); //Initialize the intterrupt pin
  rpmcount = 0;
  rpm = 0;
  timeold = 0;
  timeNow = 0;
  //====================== SETUP Angin ================================================

  // Inisialisasi RTC
  if (!rtc.begin())
  {
    Serial.println("Couldn't find RTC");
    while (1)
      ;
  }
  //=======Hanya dibuka komen nya jika akan kalibrasi waktu saja (hanya sekali) setelah itu harus di tutup komennya kembali supaya tidak set waktu terus menerus=======
  //rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); // Set waktu langsung dari waktu PC
  //rtc.adjust(DateTime(2022, 1, 5, 16, 23, 0)); // Set Tahun, bulan, tanggal, jam, menit, detik secara manual
  // Cukup dibuka salah satu dari 2 baris diatas, pilih set waktu secara manual atau dari PC
  //===================================================================================================================================================================
  bacaRTC();
  printSerial_angin();
  printSerial_curah_hujan();
  
  pinMode (sensor_tanah, INPUT);
  pinMode (sensor_hujan, INPUT);
  rtc.begin();
  lcd.init();
  // Print a message to the LCD.
  lcd.backlight();
  // jadikan pin power sebagai output

  Serial.println(F("BMP280 test"));
  if (!bmp.begin(0x76))
    {
      Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
      while (1);
    }

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
  
  dht.begin();
  Serial.print("Initializing SD card...");

  if (!SD.begin(53)) {
    Serial.println("initialization failed!");
    while (1);
  }
  //membuat file csv
  if(!SD.exists("logger.csv")){
    myFile = SD.open("logger.csv",FILE_WRITE);
    if (myFile){
      myFile.println("tgl_jam,Kelembaban_tanah,Kelembaban_udara,Suhu,Presure,Altitude,Kecepatan_angin,Cuaca,NilaiCuaca_HariIni,Nilai_hujan");
      myFile.close();    
    }
  }
  Serial.println("initialization done SD card.");
  //setWifi(ssid, pass);

}

void loop() {

  //Measure RPM
  if ((millis() - timeold) >= timemeasure * 1000)
  {
    detachInterrupt(digitalPinToInterrupt(GPIO_pulse)); // Disable interrupt when calculating
    rps = float(rpmcount) / float(timemeasure);         // rotations per second
    rpm = 60 * rps;                                     // rotations per minute
    omega = 2 * PI * rps;                               // rad/s
    velocity_ms = omega * radius * calibration_value;   // m/s
    velocity_kmh = velocity_ms * 3.6;                   // km/h
    printSerial_angin();                                // print serial 25 detik sekali
    timeold = millis();
    rpmcount = 0;
    attachInterrupt(digitalPinToInterrupt(GPIO_pulse), rpm_anemometer, RISING); // enable interrupt
  }

  if (flag == true) 
  {
    curah_hujan += milimeter_per_tip; // Akan bertambah nilainya saat tip penuh
    jumlah_tip++;
    delay(500);
    flag = false; // reset flag
  }
  bacaRTC();
  curah_hujan_hari_ini = jumlah_tip * milimeter_per_tip;
  temp_curah_hujan_per_menit = curah_hujan;

  //Probabilistik Curah Hujan https://www.bmkg.go.id/cuaca/probabilistik-curah-hujan.bmkg
  if (curah_hujan_hari_ini <= 0.00 && curah_hujan_hari_ini <= 0.50)
  {
    cuaca = "Berawan           ";
  }
  if (curah_hujan_hari_ini > 0.50 && curah_hujan_hari_ini <= 20.00)
  {
    cuaca = "Hujan Ringan      ";
  }
  if (curah_hujan_hari_ini > 20.00 && curah_hujan_hari_ini <= 50.00)
  {
    cuaca = "Hujan Sedang      ";
  }
  if (curah_hujan_hari_ini > 50.00 && curah_hujan_hari_ini <= 100.00)
  {
    cuaca = "Hujan Lebat       ";
  }
  if (curah_hujan_hari_ini > 100.00 && curah_hujan_hari_ini <= 150.00)
  {
    cuaca = "Hujan Sangat Lebat";
  }
  if (curah_hujan_hari_ini > 150.00)
  {
    cuaca = "Hujan ekstrem     ";
  }
  if (detik.equals("0")) // Hanya print pada detik 0
  {
    curah_hujan_per_menit = temp_curah_hujan_per_menit; // Curah hujan per menit dihitung ketika detik 0
    temp_curah_hujan_per_jam += curah_hujan_per_menit;  // Curah hujan per jam dihitung dari penjumlahan curah hujan per menit namun disimpan dulu dalam variabel temp
    if (menit.equals("0"))
    {
      curah_hujan_per_jam = temp_curah_hujan_per_jam;   // Curah hujan per jam baru dihitung ketika menit 0
      temp_curah_hujan_per_hari += curah_hujan_per_jam; //// Curah hujan per hari dihitung dari penjumlahan curah hujan per jam namun disimpan dulu dalam variabel temp
      temp_curah_hujan_per_jam = 0.00;                  // Reset temp curah hujan per jam
    }
    if (menit.equals("0") && jam.equals("0"))
    {
      curah_hujan_per_hari = temp_curah_hujan_per_hari; // Curah hujan per jam baru dihitung ketika menit 0 dan jam 0 (Tengah malam)
      temp_curah_hujan_per_hari = 0.00;                 // Reset temp curah hujan per hari
      curah_hujan_hari_ini = 0.00;                      // Reset curah hujan hari ini
      jumlah_tip = 0;                                   // Jumlah tip di reset setap 24 jam sekali (Tengah malam)
    }
    temp_curah_hujan_per_menit = 0.00;
    curah_hujan = 0.00;
    delay(1000);
  }
  if ((jumlah_tip != temp_jumlah_tip) || (detik.equals("0"))) // Print serial setiap 1 menit atau ketika jumlah_tip berubah
  {
    printSerial_curah_hujan();
  }
  temp_jumlah_tip = jumlah_tip;
  
  float kelembaban = dht.readHumidity();
  float suhu = dht.readTemperature();

  Serial.print("kelembaban: ");
  Serial.print(kelembaban);
  Serial.print(" ");
  Serial.print("suhu: ");
  Serial.println(suhu);

  int kondisi_tanah = analogRead(sensor_tanah); //read dari 1000-0
  Serial.print("kondisi_tanah");
  Serial.println(kondisi_tanah);

  int kondisi_hujan = analogRead(sensor_hujan); //read dari 1000-0
  Serial.print("kondisi_hujan");
  Serial.println(kondisi_hujan);

  Serial.print(F("Temperature = "));
  Serial.print(bmp.readTemperature());
  Serial.println(" C");

  Serial.print(F("Pressure = "));
  Serial.print(bmp.readPressure());
  Serial.println(" Pa");

  Serial.print(F("Approx altitude = "));
  Serial.print(bmp.readAltitude(1013.25)); /* Adjusted to local forecast! */
  Serial.println(" m");

  Serial.println(hari);  
  Serial.println(bulan);  
  Serial.println(tahun); 
  
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("TGL: ");
  lcd.setCursor(4, 0);
  lcd.print(hari);
  lcd.setCursor(7, 0);
  lcd.print(bulan);
  lcd.setCursor(10, 0);
  lcd.print(tahun);
  

  lcd.setCursor(0, 1);
  lcd.print("Jam: ");
  lcd.setCursor(5, 1);
  lcd.print(jam);
  lcd.setCursor(8, 1);
  lcd.print(menit);
  lcd.setCursor(11, 1);
  lcd.print(detik);
  
  delay(5000); //waktu tunda 5 detik
  
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Cuaca : ");
  lcd.setCursor(7, 0);
  lcd.print(cuaca);

  lcd.setCursor(0,1);
  lcd.print("Jumlah_tip : ");
  lcd.setCursor(12,1);
  lcd.print(jumlah_tip);

  delay(5000); //waktu tunda 5 detik

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("K_Angin: ");
  lcd.setCursor(9, 0);
  lcd.print(velocity_ms);
  lcd.setCursor(13, 0);
  lcd.print("m/s");
  

  lcd.setCursor(0, 1);
  lcd.print("kel_tanah:");
  lcd.setCursor(10, 1);
  lcd.print(kondisi_tanah);
  lcd.setCursor(14, 1);
  lcd.print("RH");

  delay(5000); //waktu tunda 5 detik
  
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("kel_Udara: ");
  lcd.setCursor(10, 0);
  lcd.print(kelembaban);
  lcd.setCursor(14, 0);
  lcd.print("RH");
  

  lcd.setCursor(0, 1);
  lcd.print("Suhu :");
  lcd.setCursor(10, 1);
  lcd.print(bmp.readTemperature());
  lcd.setCursor(14, 1);
  lcd.print("C");

  delay(5000); //waktu tunda 5 detik
  
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Pressure : ");
  lcd.setCursor(10, 0);
  lcd.print(bmp.readPressure());
  lcd.setCursor(14, 0);
  lcd.print("Pa");

  lcd.setCursor(0, 1);
  lcd.print("Altitude:");
  lcd.setCursor(10, 1);
  lcd.print(bmp.readAltitude(1013.25));
  lcd.setCursor(14, 1);
  lcd.print("m   ");

  delay(5000); //waktu tunda 5 detik
  String tgl_jam = String() + hari +":"+ bulan+":"+ tahun +"-"+jam+":"+menit+":"+detik;
  float nilai_hujan = kondisi_hujan;
  float Kecepatan_angin = velocity_ms;
  int kelembaban_tanah = kondisi_tanah;
  int kelembaban_udara = kelembaban;
  float suhu1 = bmp.readTemperature();
  float presure = bmp.readPressure();
  float altitude = bmp.readAltitude(1013.25);
  String data = String()+tgl_jam+","+kelembaban_tanah+","+kelembaban_udara+","+suhu1+","+presure+","+altitude+","+Kecepatan_angin+","+cuaca+","+curah_hujan_hari_ini+","+nilai_hujan ;
  
   myFile = SD.open("logger.csv",FILE_WRITE);
   
  if (myFile){
    myFile.println(data);
    Serial.println("data telah di simpan di SD card");
    myFile.close();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Save SDcard Done ");
    delay(1000); 
  }else{
    Serial.println("ERORR simpan di SD card");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("gagal save SDCard");
    delay(1000); 
  }
  delay(5000);
}

//fungsi

String konversi_jam(String angka) // Fungsi untuk supaya jika angka satuan ditambah 0 di depannya, Misalkan jam 1 maka jadi 01 pada LCD
{
  if (angka.length() == 1)
  {
    angka = "0" + angka;
  }
  else
  {
    angka = angka;
  }
  return angka;
}

void bacaRTC()
{
  now = rtc.now(); // Ambil data waktu dari DS3231
  jam = String(now.hour(), DEC);
  menit = String(now.minute(), DEC);
  detik = String(now.second(), DEC);
  hari = String(now.day(), DEC);
  bulan = String(now.month(), DEC);
  tahun = String(now.year(), DEC);
}

void printSerial_angin()
{
  Serial.print("RPS=");
  Serial.println(rps);
  Serial.print("RPM=");
  Serial.println(rpm);
  Serial.print("Kecepatan=");
  Serial.print(velocity_ms);
  Serial.print(" m/s, ");
  Serial.print(velocity_kmh);
  Serial.println(" Km/jam");
}

void printSerial_curah_hujan()
{
  Serial.println("Waktu=" + konversi_jam(jam) + ":" + konversi_jam(menit));
  Serial.print("Cuaca=");
  Serial.println(cuaca); // Print cuaca hari ini (Ini bukan ramalan cuaca tapi membaca cuaca yang sudah terjadi/ sedang terjadi hari ini)
  Serial.print("Jumlah tip=");
  Serial.print(jumlah_tip);
  Serial.println(" kali ");
  Serial.print("Curah hujan hari ini=");
  Serial.print(curah_hujan_hari_ini, 1);
  Serial.println(" mm ");
  Serial.print("Curah hujan per menit=");
  Serial.print(curah_hujan_per_menit, 1);
  Serial.println(" mm ");
  Serial.print("Curah hujan per jam=");
  Serial.print(curah_hujan_per_jam, 1);
  Serial.println(" mm ");
  Serial.print("Curah hujan per hari=");
  Serial.print(curah_hujan_per_hari, 1);
  Serial.println(" mm ");
}
