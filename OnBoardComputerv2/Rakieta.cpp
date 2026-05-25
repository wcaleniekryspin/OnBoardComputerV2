#include <sys/_stdint.h>
#include "Rakieta.h"


/*
ZASADY:
  -nie może być setOffsets w setupie - jak się zresetuje w locie, to musi szybko przejść do loopa


DONE:
  -dodać wartości inicjalizacyjne do zmiennych
  -dodać odejmowanie offsetów przy handle


TO DO:
  -sprawdzić  "AIR VEHICLE C++ CODING STANDARDS FOR THE SYSTEM DEVELOPMENT AND DEMONSTRATION PROGRAM"
  -zrobić graficzne przedstawienie blokowe jak ma działać cały system


  -dodać funkcję sprawdzającą ile jest dostępnego miejsca na flash - jeśli jest mało powiadom użytkownika
  -dodać led_2 jako oznacznie errorów
  -dodać flush po FLUSH_AFTER_WRITES zapisach
  -w funkcjach filter dodać pozostałe czujniki ze sprawdzaniem errorów
  -zrobić funkcję handleErrors()

  Dokumentacja i komentarze:
      Dodaj szczegółowe komentarze do kodu, zwłaszcza w sekcjach związanych z obsługą błędów i watchdogiem.
      Zaktualizuj sekcję TO DO w kodzie na podstawie Twoich odpowiedzi.

  Obsługa błędów:
      Rozbuduj funkcję handleErrors(), aby automatycznie restartowała uszkodzone komponenty.
      Dodaj logikę do trybu FLIGHT, która sprawdza stan wszystkich komponentów na początku lotu.

  Zapis danych:
      Upewnij się, że zapis do flash jest odporny na przerwania (np. poprzez blokady).
      Rozważ kompresję danych przed zapisem, jeśli pamięć flash jest ograniczona.

  Optymalizacja energii:
      Wprowadź bardziej zaawansowane strategie oszczędzania energii (np. głębszy sleep dla nieużywanych komponentów).
      Monitoruj zużycie energii w różnych trybach.

  Testowanie:
      Stwórz scenariusze testowe w trybie DEBUG, aby przetestować różne ścieżki kodu.
      Symuluj awarie komponentów i sprawdź, czy mechanizmy naprawcze działają poprawnie.

*/

// === Konstruktor ===
Rakieta::Rakieta():
  errorFlags(LORA_ERROR | LSM_ERROR | BMP1_ERROR | BMP2_ERROR | ADXL_ERROR | GPS_ERROR | FLASH_ERROR),
  spiFast(SPI4_SCK, SPI4_MISO, SPI4_MOSI),
  spiFlash(SPI1_SCK, SPI1_MISO, SPI1_MOSI),
  spiLora(SPI_LORA_SCK, SPI_LORA_MISO, SPI_LORA_MOSI),
  lsm(),
  bmp1(),
  bmp2(),
  adxl(CS_ADXL, &spiFast),
  flashTransport(CS_FLASH, &spiFlash),
  flash(&flashTransport),
  loraSettings(2000000, MSBFIRST, SPI_MODE0),
  lora(new Module(CS_LORA, DIO1_LORA, RST_LORA, BUSY_LORA, spiLora)),
  gpsSerial(RX_MAX, TX_MAX)
{
}

Rakieta::~Rakieta()
{
  lora.clearDio1Action();

  if (flashDataFile) flashCloseFile();
}

void Rakieta::initGPIO()
{
  pinMode(LED_1, OUTPUT);
  pinMode(LED_2, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(SOLENOID_1, OUTPUT);
  pinMode(SOLENOID_2, OUTPUT);
  pinMode(BATTERY, INPUT);
  
  digitalWrite(LED_1, LOW);
  digitalWrite(LED_2, LOW);
  digitalWrite(BUZZER, LOW);
  digitalWrite(SOLENOID_1, LOW);
  digitalWrite(SOLENOID_2, LOW);
  
  Serial.println("[GPIO] OK");
  
  pinMode(MODE1, INPUT_PULLUP);
  pinMode(MODE2, INPUT_PULLUP);
  pinMode(MODE3, INPUT_PULLUP);
  pinMode(MODE4, INPUT_PULLUP);
  Serial.println("[DIP SWITCH] OK");
}

void Rakieta::initSPI()
{
  spiFlash.begin();
  Serial.println("[SPI_FLASH] OK");
  
  spiLora.begin();
  Serial.println("[SPI_LORA] OK");
  
  spiFast.begin();
  Serial.println("[SPI_CZUJNIKI] OK");
}

bool Rakieta::initLSM()
{
  if (lsm.begin_SPI(CS_LSM, &spiFast))
  {
    Serial.println("[LSM] OK");
    errorFlags &= ~LSM_ERROR;
    return true;
  }
  Serial.println("[LSM] BRAK ODPOWIEDZI");
  errorFlags |= LSM_ERROR;
  return false;
}

bool Rakieta::initADXL()
{
  if (adxl.begin())
  {
    adxl.setRange(ADXL343_RANGE_16_G);
    Serial.println("[ADXL375] OK, zakres 16G");
    errorFlags &= ~ADXL_ERROR;
    return true;
  }
  Serial.println("[ADXL375] BRAK ODPOWIEDZI");
  errorFlags |= ADXL_ERROR;
  return false;
}

bool Rakieta::initBMP()
{
  bool ok = true;
  
  if (bmp1.begin_SPI(CS_BMP1, &spiFast))
  {
    bmp1.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp1.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp1.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp1.setOutputDataRate(BMP3_ODR_100_HZ);
    Serial.println("[BMP388 #1] OK");
    errorFlags &= ~BMP1_ERROR;
  }
  else
  {
    Serial.println("[BMP388 #1] BRAK ODPOWIEDZI");
    errorFlags |= BMP1_ERROR;
    ok = false;
  }
  
  if (bmp2.begin_SPI(CS_BMP2, &spiFast))
  {
    bmp2.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp2.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp2.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp2.setOutputDataRate(BMP3_ODR_100_HZ);
    Serial.println("[BMP388 #2] OK");
    errorFlags &= ~BMP2_ERROR;
  }
  else
  {
    Serial.println("[BMP388 #2] BRAK ODPOWIEDZI");
    errorFlags |= BMP2_ERROR;
    ok = false;
  }
  
  return ok;
}

bool Rakieta::initGPS()
{
  gpsSerial.begin(9600, SERIAL_8N1);
  Serial.println("[GPS] UART2 OK (9600 baud)");
  
  delay(500);
  while (gpsSerial.available())
  {
    gps.encode(gpsSerial.read());
  }
  
  if (gps.charsProcessed() > 0)
  {
    Serial.println("[GPS] Dane odbierane");
    errorFlags &= ~GPS_ERROR;
    return true;
  }
  Serial.println("[GPS] Brak danych (sprawdź połączenie)");
  errorFlags |= GPS_ERROR;
  return false;
}

void Rakieta::updateLeds(uint32_t now)
{
  if (led1IsOn && led1OffTime < now)
  {
    digitalWrite(LED_1, LOW);
    Serial.printf("[LED1] Updated");
  }
  if (led2IsOn && led2OffTime < now)
  {
    digitalWrite(LED_2, LOW);
    Serial.printf("[LED2] Updated");
  }
}

void Rakieta::updateBuzzer(uint32_t now)
{
  if (buzzerIsOn && buzzerOffTime < now)
  {
    digitalWrite(BUZZER, LOW);
    Serial.printf("[BUZZER] Updated");
  }
}

void Rakieta::updateSolenoid(uint32_t now)
{
  if (solenoid1IsOn && solenoid1OffTime < now)
  {
    digitalWrite(SOLENOID_1, LOW);
    Serial.println("[SOLENOID1] Updated");
  }
  
  if (solenoid2IsOn && solenoid2OffTime < now)
  {
    digitalWrite(SOLENOID_2, LOW);
    Serial.println("[SOLENOID2] Updated");
  }
}

void Rakieta::systemReset()
{
  lora.clearDio1Action();
  if (flashDataFile) flashCloseFile();

  NVIC_SystemReset();
}

void Rakieta::systemSleep(uint32_t time)
{
  delay(time);
  // HAL_SuspendTick();
  // HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
  // HAL_Delay(time);
  // HAL_ResumeTick();
}

void Rakieta::setSystemMode()
{
  uint8_t m1 = digitalRead(MODE1) == LOW ? 0 : 1;
  uint8_t m2 = digitalRead(MODE2) == LOW ? 0 : 1;
  uint8_t m3 = digitalRead(MODE3) == LOW ? 0 : 1;
  uint8_t m4 = digitalRead(MODE4) == LOW ? 0 : 1;
  
  uint8_t modeCode = (m4 << 3) | (m3 << 2) | (m2 << 1) | m1;
  
  switch (modeCode)
  {
    case 0b0000: currentMode = DEBUG;  break;
    case 0b0001: currentMode = FLIGHT; break;
    case 0b0010: currentMode = DUMP;   break;
    case 0b0011: currentMode = SLEEP;  break;
    default:     currentMode = DEBUG;  break;
  }
}

void Rakieta::printSystemMode()
{
  Serial.print("[SYSTEM MODE] ");
  switch (currentMode)
  {
    case DEBUG:  Serial.println("DEBUG");  break;
    case FLIGHT: Serial.println("FLIGHT"); break;
    case DUMP:   Serial.println("DUMP");   break;
    case SLEEP:  Serial.println("SLEEP");  break;
  }
}

void Rakieta::printFlightMode()  /// prawdopodobnie do usunięcia bo nigdzie nie będzie używane
{
  Serial.print("[FLIGHT MODE] ");
  switch (currentMode)
  {
    case IDLE:    Serial.println("IDLE");     break;
    case BOOST:   Serial.println("BOOST");    break;
    case COAST:   Serial.println("COAST");    break;
    case APOGEE:  Serial.println("APOGEE");   break;
    case DESCENT: Serial.println("DESCENT");  break;
    case LANDED:  Serial.println("LANDED");   break;
  }
}

void Rakieta::handleBattery()  // tzreba sprawdzić czy odpowiednie są przeliczniki
{
  int rawValue = analogRead(BATTERY);
  data.battery.voltage = (rawValue * 4.2f / 4095.0f);
}

void Rakieta::handleLsm()
{
  sensors_event_t accel, gyro, temp;
  if(lsm.getEvent(&accel, &gyro, &temp))
  {
    data.lsm.ax = accel.acceleration.x - offsets.lsm.ax;
    data.lsm.ay = accel.acceleration.y - offsets.lsm.ay;
    data.lsm.az = accel.acceleration.z - offsets.lsm.az;
    data.lsm.gx = gyro.gyro.x - offsets.lsm.gx;
    data.lsm.gy = gyro.gyro.y - offsets.lsm.gy;
    data.lsm.gz = gyro.gyro.z - offsets.lsm.gz;
    data.lsm.temp = temp.temperature;

    uint32_t now = millis();
    float dt = (now - data.lsm.lastTime) / 1000.0f;
    data.lsm.lastTotalAccel = sqrt(data.lsm.ax * data.lsm.ax + 
                                   data.lsm.ay * data.lsm.ay + 
                                   data.lsm.az * data.lsm.az);
    data.lsm.lastTotalSpeed += data.lsm.lastTotalAccel * dt;
    data.lsm.lastTotalAlti += data.lsm.lastTotalSpeed * dt;
    data.lsm.lastTotalRotation = data.lsm.gx + data.lsm.gy + data.lsm.gz;
    data.lsm.lastTime = now;

    errorFlags &= ~LSM_ERROR;
  }
  else
    errorFlags |= LSM_ERROR;
}

void Rakieta::handleAdxl()
{
  sensors_event_t event;
  if (adxl.getEvent(&event))
  {
    data.adxl.ax = event.acceleration.x - offsets.adxl.ax;
    data.adxl.ay = event.acceleration.y - offsets.adxl.ay;
    data.adxl.az = event.acceleration.z - offsets.adxl.az;
    
    uint32_t now = millis();
    float dt = (now - data.adxl.lastTime) / 1000.0f;
    data.adxl.lastTotalAccel = sqrt(data.adxl.ax * data.adxl.ax + 
                                    data.adxl.ay * data.adxl.ay + 
                                    data.adxl.az * data.adxl.az);
    data.adxl.lastTotalSpeed += data.adxl.lastTotalAccel * dt;
    data.adxl.lastTime = now;

    errorFlags &= ~ADXL_ERROR;
  }
  else
    errorFlags |= ADXL_ERROR;
}

void Rakieta::handleBmp()
{
  if (bmp1.performReading())
  {
    data.bmp1.pressure = bmp1.pressure;
    data.bmp1.altitude = bmp1.readAltitude(1013.25) - offsets.bmp1.alti;
    data.bmp1.temp = bmp1.temperature;
    
    if (data.bmp1.maxAltitude < data.bmp1.altitude)
      data.bmp1.maxAltitude = data.bmp1.altitude;
    
    uint32_t now = millis();
    float dt = (now - data.bmp1.lastTime) / 1000.0f;
    if (dt > 0.001)
    {
      data.bmp1.lastVerticalSpeed = (data.bmp1.altitude - data.bmp1.lastAltitude) / dt;
      data.bmp1.lastAltitude = data.bmp1.altitude;
      data.bmp1.lastTime = now;
    }

    errorFlags &= ~BMP1_ERROR;
  }
  else
    errorFlags |= BMP1_ERROR;

  if (bmp2.performReading())
  {
    data.bmp2.pressure = bmp2.pressure;
    data.bmp2.altitude = bmp2.readAltitude(1013.25) - offsets.bmp2.alti;
    data.bmp2.temp = bmp2.temperature;
    
    if (data.bmp2.maxAltitude < data.bmp2.altitude)
      data.bmp2.maxAltitude = data.bmp2.altitude;
    
    uint32_t now = millis();
    float dt = (now - data.bmp2.lastTime) / 1000.0f;
    if (dt > 0.001)
    {
      data.bmp2.lastVerticalSpeed = (data.bmp2.altitude - data.bmp2.lastAltitude) / dt;
      data.bmp2.lastAltitude = data.bmp2.altitude;
      data.bmp2.lastTime = now;
    }

    errorFlags &= ~BMP2_ERROR;
  }
  else
    errorFlags |= BMP2_ERROR;
}

void Rakieta::handleGPS()
{
  while (gpsSerial.available())
    gps.encode(gpsSerial.read());

  if (gps.location.isValid() && gps.location.isUpdated())
  {
    data.gps.lat = gps.location.lat() - offsets.gps.lat;
    data.gps.lng = gps.location.lng() - offsets.gps.lng;
  }
  if (gps.altitude.isValid() && gps.altitude.isUpdated())
  {
    data.gps.alti = gps.altitude.meters();
    if (data.gps.alti > data.gps.maxAlti)
      data.gps.maxAlti = data.gps.alti - offsets.gps.alti;
  }
  if (gps.time.isValid() && gps.time.isUpdated())
  {
    data.gps.h = gps.time.hour();
    data.gps.m = gps.time.minute();
    data.gps.s = gps.time.second();
    data.gps.centi = gps.time.centisecond();
  }
  if (gps.speed.isValid() && gps.speed.isUpdated())
    data.gps.speed = gps.speed.mps();
  if (gps.course.isValid() && gps.course.isUpdated())
    data.gps.course = gps.course.deg();
  if (gps.satellites.isValid() && gps.satellites.isUpdated())
    data.gps.satNum = gps.satellites.value();
  if (gps.hdop.isValid() && gps.hdop.isUpdated())
    data.gps.hdop = gps.hdop.hdop();
}

void Rakieta::readSensorsData()
{
  handleBattery();
  handleLsm();
  handleAdxl();
  handleBmp();
  handleGPS();
}

void Rakieta::printData()
{
  Serial.printf("IMU: Accel: %.2f %.2f %.2f G | Gyro: %.2f %.2f %.2f dps | Temp: %.2f *C\n",
                data.lsm.ax, data.lsm.ay, data.lsm.az, data.lsm.gx, data.lsm.gy, data.lsm.gz, data.lsm.temp);
  Serial.printf("IMU: Speed: %.2f m/s | Accel: %.3f m/s^2 | Gyro: %.3f d\n",
                data.lsm.lastTotalSpeed, data.lsm.lastTotalAccel, data.lsm.lastTotalRotation);

  Serial.printf("ADXL: X=%.2f Y=%.2f Z=%.2f G\n", data.adxl.ax, data.adxl.ay, data.adxl.az);
  Serial.printf("ADXL: Speed: %.2f m/s | Accel: %.3f m/s^2\n", data.adxl.lastTotalSpeed, data.adxl.lastTotalAccel);

  Serial.printf("BMP1: P=%.2f hPa, Alt=%.1f m, T=%.2f C\n", data.bmp1.pressure, data.bmp1.altitude, data.bmp1.temp);
  Serial.printf("BMP1: Speed: %.2f\n", data.bmp1.lastVerticalSpeed);
  
  Serial.printf("BMP2: P=%.2f hPa, Alt=%.1f m, T=%.2f C\n", data.bmp2.pressure, data.bmp2.altitude, data.bmp2.temp);
  Serial.printf("BMP2: Speed: %.2f\n", data.bmp2.lastVerticalSpeed);
  
  Serial.printf("GPS: Lat=%.6f Lon=%.6f\n", data.gps.lat, data.gps.lng);
  Serial.printf("GPS: Alt=%.1f m\n",data.gps.alti);
  Serial.printf("GPS: Time=%2d:%2d:%2d:%d\n", data.gps.h, data.gps.m, data.gps.s, data.gps.centi);
  Serial.printf("GPS: Speed:%.2f\n", data.gps.speed);
  Serial.printf("GPS: Course:%.3f\n", data.gps.course);
  Serial.printf("GPS: Sat:%2d\n", data.gps.satNum);
  Serial.printf("GPS: hdop:%2d\n", data.gps.hdop);
}

void Rakieta::setOffsets()
{
  Serial.println("\n=== Wyznaczanie offsetów ===\n");

  uint8_t num = 50;
  for (uint8_t i = 0; i < num; i++)
  {
    sensors_event_t accel, gyro, temp;
    if (lsm.getEvent(&accel, &gyro, &temp))
    {
      offsets.lsm.ax += accel.acceleration.x;
      offsets.lsm.ay += accel.acceleration.y;
      offsets.lsm.az += accel.acceleration.z;
      offsets.lsm.gx += gyro.gyro.x;
      offsets.lsm.gy += gyro.gyro.y;
      offsets.lsm.gz += gyro.gyro.z;
      offsets.lsm.valid++;
    }

    sensors_event_t event;
    if (adxl.getEvent(&event))
    {
      offsets.adxl.ax += event.acceleration.x * ADXL375_MG2G_MULTIPLIER;
      offsets.adxl.ay += event.acceleration.y * ADXL375_MG2G_MULTIPLIER;
      offsets.adxl.az += event.acceleration.z * ADXL375_MG2G_MULTIPLIER;
      offsets.adxl.valid++;
    }

    if (bmp1.performReading())
    {
      offsets.bmp1.altitude += bmp1.readAltitude(1013.25);
      offsets.bmp1.valid++;
    }

    if (bmp2.performReading())
    {
      offsets.bmp2.altitude += bmp2.readAltitude(1013.25);
      offsets.bmp2.valid++;
    }

    delay(25);
  }

  offsets.lsm.ax = (offsets.lsm.valid ? ((float)offsets.lsm.ax / (float)offsets.lsm.valid) : 0);
  offsets.lsm.ay = (offsets.lsm.valid ? ((float)offsets.lsm.ay / (float)offsets.lsm.valid) : 0);
  offsets.lsm.az = (offsets.lsm.valid ? ((float)offsets.lsm.az / (float)offsets.lsm.valid) : 0);
  offsets.lsm.gx = (offsets.lsm.valid ? ((float)offsets.lsm.gx / (float)offsets.lsm.valid) : 0);
  offsets.lsm.gy = (offsets.lsm.valid ? ((float)offsets.lsm.gy / (float)offsets.lsm.valid) : 0);
  offsets.lsm.gz = (offsets.lsm.valid ? ((float)offsets.lsm.gz / (float)offsets.lsm.valid) : 0);
  offsets.adxl.ax = (offsets.adxl.valid ? ((float)offsets.adxl.ax / (float)offsets.adxl.valid) : 0);
  offsets.adxl.ay = (offsets.adxl.valid ? ((float)offsets.adxl.ay / (float)offsets.adxl.valid) : 0);
  offsets.adxl.az = (offsets.adxl.valid ? ((float)offsets.adxl.az / (float)offsets.adxl.valid) : 0);
  offsets.bmp1.altitude = (offsets.bmp1.valid ? ((float)offsets.bmp1.altitude / (float)offsets.bmp1.valid) : 0);
  offsets.bmp2.altitude = (offsets.bmp2.valid ? ((float)offsets.bmp2.altitude / (float)offsets.bmp2.valid) : 0);

  num = 10;
  uint32_t timeout = millis() + 30000;

  while (offsets.gps.valid < num || millis() < timeout)
  {
    if (gps.location.isValid() && gps.altitude.isValid())
    {
      offsets.gps.lat += gps.location.lat();
      offsets.gps.lng += gps.location.lng();
      offsets.gps.alti += gps.altitude.meters();
      offsets.gps.valid++;
    }
    delay(200);
  }

  offsets.gps.lat = int32_t(offsets.gps.valid ? ((float)offsets.gps.lat / (float)offsets.gps.valid) : 0);
  offsets.gps.lng = int32_t(offsets.gps.valid ? ((float)offsets.gps.lng / (float)offsets.gps.valid) : 0);
  offsets.gps.alti = (offsets.gps.valid ? ((float)offsets.gps.alti / (float)offsets.gps.valid) : 0);

  offsetsSet = true;
}

String Rakieta::prepareOffsetsMsg()
{
  if (!offsetsSet) return "Offsets not set";
  
  String msg = "Valid num:Lsm:";
  msg += String(offsets.lsm.valid);
  msg += ",Adxl:";
  msg += String(offsets.adxl.valid);
  msg += ",Bmp1:";
  msg += String(offsets.bmp1.valid);
  msg += ",Bmp2:";
  msg += String(offsets.bmp2.valid);
  msg += ",GPS:";
  msg += String(offsets.gps.valid);
  msg += "Offsets:Lsm:{ax:";
  msg += String(offsets.lsm.ax);
  msg += ",ay:";
  msg += String(offsets.lsm.ay);
  msg += ",az:";
  msg += String(offsets.lsm.az);
  msg += ",gx:";
  msg += String(offsets.lsm.gx);
  msg += ",gy:";
  msg += String(offsets.lsm.gy);
  msg += ",gz:";
  msg += String(offsets.lsm.gz);
  msg += "}Adxl:{ax:";
  msg += String(offsets.adxl.ax);
  msg += ",ay:";
  msg += String(offsets.adxl.ay);
  msg += ",az:";
  msg += String(offsets.adxl.az);
  msg += "}GPS:{lat:";
  msg += String(offsets.gps.lat);
  msg += ",lng:";
  msg += String(offsets.gps.lng);
  msg += ",altiM:";
  msg += String(offsets.gps.alti);
  msg += "}";
  return msg;
}

String Rakieta::prepareDataLineMsg()
{
  String msg = "RocketData:";

  // Timestamp
  msg += String(millis());
  msg += ",";
  msg += String(packet);
  msg += ",";
  msg += String(static_cast<int>(currentFlightState));
  msg += ",";
  msg += String(errorFlags, HEX);
  msg += ",";
  
  // GPS
  msg += String(data.gps.lat, 6);
  msg += ",";
  msg += String(data.gps.lng, 6);
  msg += ",";
  msg += String(data.gps.alti, 2);
  msg += ",";
  msg += String(data.gps.h);
  msg += ",";
  msg += String(data.gps.m);
  msg += ",";
  msg += String(data.gps.s);
  msg += ",";
  msg += String(data.gps.centi);
  msg += ",";
  msg += String(data.gps.speed, 2);
  msg += ",";
  msg += String(data.gps.course, 0);
  msg += ",";
  msg += String(data.gps.satNum);
  msg += ",";
  msg += String(data.gps.hdop);
  msg += ",";
  
  // LSM6
  msg += String(data.lsm.ax, 4);
  msg += ",";
  msg += String(data.lsm.ay, 4);
  msg += ",";
  msg += String(data.lsm.az, 4);
  msg += ",";
  msg += String(data.lsm.gx, 3);
  msg += ",";
  msg += String(data.lsm.gy, 3);
  msg += ",";
  msg += String(data.lsm.gz, 3);
  msg += ",";
  msg += String(data.lsm.temp, 2);
  msg += ",";
  msg += String(data.lsm.lastTotalSpeed, 4);
  msg += ",";

  // ADXL
  msg += String(data.adxl.ax, 4);
  msg += ",";
  msg += String(data.adxl.ay, 4);
  msg += ",";
  msg += String(data.adxl.az, 4);
  msg += ",";
  msg += String(data.adxl.lastTotalSpeed, 4);
  msg += ",";
  
  // BMP
  msg += String(data.bmp1.temp, 2);
  msg += ",";
  msg += String(data.bmp1.pressure, 2);  /// sprawdzić czy jest w Pa czy w hPa
  msg += ",";
  msg += String(data.bmp1.altitude, 2);
  msg += ",";
  msg += String(data.bmp1.lastVerticalSpeed, 4);
  msg += ",";
  msg += String(data.bmp2.temp, 2);
  msg += ",";
  msg += String(data.bmp2.pressure, 2);  /// sprawdzić czy jest w Pa czy w hPa
  msg += ",";
  msg += String(data.bmp2.altitude, 2);
  msg += ",";
  msg += String(data.bmp2.lastVerticalSpeed, 4);
  msg += ",";

  // BATTERY
  msg += String(data.battery.voltage, 1);
  return msg;
}

void Rakieta::filterAcceleration()  /// nigdzie nie używane
{
  const float alpha = 0.5f;
  filteredAccelX = alpha * data.lsm.ax + (1 - alpha) * filteredAccelX;
  filteredAccelY = alpha * data.lsm.ay + (1 - alpha) * filteredAccelY;
  filteredAccelZ = alpha * data.lsm.az + (1 - alpha) * filteredAccelZ;
}

void Rakieta::filterGyro()  /// nigdzie nie używane
{
  const float alpha = 0.5f;
  filteredGyroX = alpha * data.lsm.gx + (1 - alpha) * filteredGyroX;
  filteredGyroY = alpha * data.lsm.gy + (1 - alpha) * filteredGyroY;
  filteredGyroZ = alpha * data.lsm.gz + (1 - alpha) * filteredGyroZ;
}

void Rakieta::calculateOrientation()  /// nigdzie nie używane
{
  data.orientation.pitch = atan2(-filteredAccelX, sqrt(filteredAccelY*filteredAccelY + filteredAccelZ*filteredAccelZ));
  data.orientation.roll = atan2(filteredAccelY, filteredAccelZ);
}

void Rakieta::fuseBMPAndIMU()  /// nigdzie nie używane
{
  const float alpha = 0.5f;
  uint32_t now = millis();
  float dt = (now - data.lsm.lastTime) / 1000.0f;
  if (dt > 0.001)
  {
    float altitudeFromIMU = prevFusedAltitude + data.lsm.lastTotalAlti;
    
    fusedAltitude = alpha * data.bmp1.altitude + (1 - alpha) * altitudeFromIMU;
    prevFusedAltitude = fusedAltitude;
  }
}

bool Rakieta::initLora()
{
  int state = lora.begin(FREQUENCY, BANDWIDTH, SF, CODING_RATE, RADIOLIB_SX126X_SYNC_WORD_PRIVATE, POWER, PREAMBLE_LENGTH);

  if (state == RADIOLIB_ERR_NONE)
  {
    lora.setDio1Action(setOperationFlag);
    Serial.println(prepareLoraStatusMsg());
    Serial.println("[LoRa] OK");
    errorFlags &= ~LORA_ERROR;
    startListening();
    return true;
  }
  Serial.print("[LoRa] Błąd: ");
  Serial.println(state);
  errorFlags |= LORA_ERROR;
  return false;
}

String Rakieta::prepareLoraStatusMsg()
{
  String msg = "=== RADIO STATUS ===\nFrequency: ";
  msg += String(FREQUENCY);
  msg += " MHz,Moc: ";
  msg += String(POWER);
  msg += " dBm,SF: ";
  msg += String(SF);
  msg += "BW: ";
  msg += String(BANDWIDTH);
  msg += " kHz,CR: 4/";
  msg += String(CODING_RATE);
  msg += "\n";
  return msg;
}

void Rakieta::preparePacket()
{
  uint32_t now = millis();
  message.add(uint32_t(now / 10), timePos, timeLen);
  message.add(uint16_t(packet), packetPos, packetLen);
  message.add(uint16_t(errorFlags), errorPos, errorLen);
  message.add(uint8_t(currentFlightState), statusPos, statusLen);

  message.add(int16_t(data.gps.lat * 100000), gpsLatPos, gpsLatLen, true);
  message.add(int16_t(data.gps.lng * 100000), gpsLngPos, gpsLngLen, true);
  message.add(uint16_t(data.gps.alti * 10), gpsAltiPos, gpsAltiLen);
  message.add(uint8_t(data.gps.h), gpsHourPos, gpsHourLen);
  message.add(uint8_t(data.gps.m), gpsMinPos, gpsMinLen);
  message.add(uint8_t(data.gps.s), gpsSecPos, gpsSecLen);
  message.add(uint8_t(data.gps.centi), gpsCentisecPos, gpsCentisecLen);
  message.add(int16_t(data.gps.speed * 10), gpsSpeedPos, gpsSpeedLen, true);
  message.add(uint16_t(data.gps.course), gpsCoursePos, gpsCourseLen);
  message.add(uint8_t(data.gps.satNum), gpsSatNumPos, gpsSatNumLen);
  message.add(uint8_t(data.gps.hdop), gpsHdopPos, gpsHdopLen);
  
  message.add(int16_t(data.lsm.ax * 100), lsmAccelXPos, lsmAccelXLen, true);
  message.add(int16_t(data.lsm.ay * 100), lsmAccelYPos, lsmAccelYLen, true);
  message.add(int16_t(data.lsm.az * 100), lsmAccelZPos, lsmAccelZLen, true);
  message.add(int16_t(data.lsm.gx * 10), lsmGyroXPos, lsmGyroXLen, true);
  message.add(int16_t(data.lsm.gy * 10), lsmGyroYPos, lsmGyroYLen, true);
  message.add(int16_t(data.lsm.gz * 10), lsmGyroZPos, lsmGyroZLen, true);
  message.add(int8_t(data.lsm.temp), lsmTempPos, lsmTempLen, true);
  message.add(int16_t(data.lsm.lastTotalSpeed * 10), lsmSpeedPos, lsmSpeedLen, true);
  
  message.add(int16_t(data.adxl.ax * 10), adxlAccelXPos, adxlAccelXLen, true);
  message.add(int16_t(data.adxl.ay * 10), adxlAccelYPos, adxlAccelYLen, true);
  message.add(int16_t(data.adxl.az * 10), adxlAccelZPos, adxlAccelZLen, true);
  message.add(int16_t(data.adxl.lastTotalSpeed * 10), adxlSpeedPos, adxlSpeedLen, true);

  message.add(int8_t(data.bmp1.temp), bmp1TempPos, bmp1TempLen, true);
  message.add(uint16_t(data.bmp1.pressure * 10), bmp1PressPos, bmp1PressLen);
  message.add(uint16_t(data.bmp1.altitude * 10), bmp1AltiPos, bmp1AltiLen);
  message.add(int16_t(data.bmp1.lastVerticalSpeed * 10), bmp1SpeedPos, bmp1SpeedLen, true);

  message.add(int8_t(data.bmp2.temp), bmp2TempPos, bmp2TempLen, true);
  message.add(uint16_t(data.bmp2.pressure * 10), bmp2PressPos, bmp2PressLen);
  message.add(uint16_t(data.bmp2.altitude * 10), bmp2AltiPos, bmp2AltiLen);
  message.add(int16_t(data.bmp2.lastVerticalSpeed * 10), bmp2SpeedPos, bmp2SpeedLen, true);

  message.add(uint8_t(data.battery.voltage * 10), batteryPos, batteryLen);
}

void Rakieta::sendPacket()
{
  packet++;
  preparePacket();
  uint8_t *txPacket = message.data();

  Serial.println("\n\nSending: ");

  for (int i=0; i<ARRAY_SIZE; i++)
    Serial.print(txPacket[i], HEX);
  Serial.println("");

  for (int i=0; i<ARRAY_SIZE; i++)
    for (int j=7; j>=0; --j)
      Serial.print((txPacket[i] >> j) & 1);

  Serial.println("\nEnd of the message.\n\n");

  transmit(txPacket, ARRAY_SIZE);

  led1IsOn = true;
  led1OffTime = millis() + 250;
  digitalWrite(LED_1, HIGH);

  Serial.println("Sending DONE");
}

void Rakieta::transmit(String msg)
{
  if (msg.length() > 255)
  {
    Serial.println(F("Error: Message too long"));
    return;
  }

  if (!operationDone)
  {
    if (millis() - loraMsgStartTime >= TX_TIMEOUT)
    {
      Serial.println("ERROR: LoRa timeout, forcing radio reset");
      lora.standby();
      delay(1);
      startListening();
    }
    else
    {
      Serial.println("LoRa busy");
      return;
    }
  }
  
  Serial.print(F("Sending: "));
  Serial.println(msg);

  operationDone = false;
  loraMsgStartTime = millis();
  int state = lora.startTransmit(msg);

  if (state != RADIOLIB_ERR_NONE)
  {
    Serial.print(F("Transmit error: "));
    Serial.println(state);
    operationDone = true;
    startListening();
  }
}

void Rakieta::transmit(uint8_t *msg, size_t len)
{
  if (len > 255)
  {
    Serial.println(F("Error: Message too long"));
    return;
  }

  if (!operationDone)
  {
    if (millis() - loraMsgStartTime >= TX_TIMEOUT)
    {
      Serial.println("ERROR: LoRa timeout, forcing radio reset");
      lora.standby();
      delay(1);
      startListening();
    }
    else
    {
      Serial.println("LoRa busy");
      return;
    }
  }
  
  Serial.print(F("Sending: "));
  Serial.println(*msg);

  operationDone = false;
  loraMsgStartTime = millis();
  int state = lora.startTransmit(msg, len);

  if (state != RADIOLIB_ERR_NONE)
  {
    Serial.print(F("Transmit error: "));
    Serial.println(state);
    operationDone = true;
    startListening();
  }
}

void Rakieta::startListening()
{
  int state = lora.startReceive();
  if (state != RADIOLIB_ERR_NONE)
  {
    Serial.print(F("Error starting listening: "));
    Serial.println(state);
  }
}

void Rakieta::handleCommand(String command)  /// trzeba pododawać komendy
{
  if (command == "RESET") { systemReset(); }
  else if (command == "SET_OFFSETS") { setOffsets(); }
  else if (command == "GET_OFFSETS") { transmit(prepareOffsetsMsg()); }  /// możliwe, że będzie za duże i trzeba będzie skompresować
  else if (command == "GET_GPS_OFFSET") { sendGpsOffset(); }
  else if (command == "GET_DATA") { transmit(prepareDataLineMsg()); }  /// możliwe, że będzie za duże i trzeba będzie skompresować
  else if (command == "GET_RAW_DATA") { readSensorsData(); sendPacket(); }
}

void Rakieta::checkRadio()
{
  String msg;
  int state = lora.readData(msg);

  if (state == RADIOLIB_ERR_NONE)
  {
    Serial.println(F("== MESSAGE RECEIVED =="));
    Serial.print(F("Message: "));
    Serial.println(msg);
    Serial.print(F("RSSI: "));
    Serial.print(lora.getRSSI());
    Serial.print(F(" dBm, "));
    Serial.print(F("SNR: "));
    Serial.print(lora.getSNR());
    Serial.println(F(" dB"));

    handleCommand(msg);
    errorFlags &= ~LORA_ERROR;
  }
  else
  {
    Serial.print(F("Data reading error: "));
    Serial.println(state);
    errorFlags |= LORA_ERROR;
  }
}

void Rakieta::sendGpsOffset()
{
  if (offsetsSet)
  {
    String msg = "GSP offset:Now:";

    msg += String(millis());
    msg += ",lat:";
    msg += offsets.gps.lat;
    msg += ",lng:";
    msg += offsets.gps.lng;
    msg += ",alti:";
    msg += offsets.gps.alti;

    transmit(msg);
  }
  else
    transmit("Offsets not set");
}

bool Rakieta::initFlash()
{
  if (flash.begin())
  {
    uint32_t jedec = flash.getJEDECID();
    Serial.print("[FLASH] OK, JEDEC: 0x");
    Serial.println(jedec, HEX);
    errorFlags &= ~FLASH_ERROR;
    return true;
  }
  Serial.println("[FLASH] BRAK ODPOWIEDZI");
  errorFlags |= FLASH_ERROR;
  return false;
}

bool Rakieta::flashFindNextFileNumber()
{
  uint16_t fileNumber = 0;
  char fileName[15];

  while (fileNumber < 9999) // Max 9999 files
  {
    sprintf(fileName, "data_%04d.csv", fileNumber);
    
    if (!fatfs.exists(fileName))
    {
      currentFileName = String(fileName);
      break;
    }
      
    fileNumber++;
  }
  
  if (fileNumber >= 9999)
  {
    Serial.println("Too many data files!");
    errorFlags |= FLASH_FILE_ERROR;
    return false;
  }
  
  Serial.print("Creating new file: ");
  Serial.println(currentFileName.c_str());
  errorFlags &= ~FLASH_FILE_ERROR;
  return true;
}

bool Rakieta::flashOpenNewFile()
{
  if (flashDataFile)
  {
    flashCloseFile();
  }

  if (!flashFindNextFileNumber())
  {
    Serial.println("Cannot open a file");
    return false;
  }
  
  flashDataFile = fatfs.open(currentFileName.c_str(), FILE_WRITE);
  if (!flashDataFile)
  {
    Serial.println("Failed to open file for writing");
    errorFlags |= FLASH_FILE_ERROR;
    return false;
  }
  
  /// trzeba pozmieniać te nagłówki
  flashWriteString("timestamp,packet,status,error,"
                 "gps_lat,gps_lng,gps_alt,gps_h,gps_m,gps_s,gps_c,"
                 "gps_speed,gps_course,gps_satNum,gps_hdop,"
                 "lsm_ax,lsm_ay,lsm_az,lsm_gx,lsm_gy,lsm_gz,lsm_temp,lsm_speed,"
                 "adxl_ax,adxl_ay,adxl_az,adxl_speed,"
                 "bmp1_temp,bmp1_press,bmp1_alt,bmp1_speed,"
                 "bmp2_temp,bmp2_press,bmp2_alt,bmp2_speed,"
                 "max_temp,battery");
  flashDataFile.flush();
  
  Serial.print("File opened: "); 
  Serial.println(currentFileName.c_str());
  errorFlags &= ~FLASH_FILE_ERROR;
  return true;
}

void Rakieta::flashWriteString(const String& msg)
{
  if (flash.isBusy())
  {
    Serial.println("Flash not ready!");
    return;
  }
  
  if (!flashDataFile)
  {
    Serial.println("File not open!");
    errorFlags |= FLASH_FILE_ERROR;
    return;
  }
  
  int written = flashDataFile.println(msg);

  if (written <= 0)
  {
    Serial.println("Write failed!");
    errorFlags |= FLASH_FILE_ERROR;
    return;
  }
  
  flashWriteCount++;
}

void Rakieta::flashFlushBuffer()
{
  if (flashDataFile)
  {
    flashDataFile.flush();
    flashWriteCount = 0;
    Serial.println("[FLASH] Buffer flushed.");
  }
}

void Rakieta::flashCloseFile()
{
  if (flashDataFile)
  {
    flashFlushBuffer();

    flashDataFile.close();
    Serial.println("[FLASH] File closed.");
  }
}

bool Rakieta::flashRecoverAfterReset()  /// wydaje mi się, że trzeba by było zmienić bo te funkcje istnieją ?
{
  if (!fatfs.begin(&flash))
  {
    Serial.println("[FLASH] Mount failed. Attempting format...");
    if (!fatfs.begin(&flash, true))
    {
      Serial.println("[FLASH] Format failed!");
      errorFlags |= FLASH_ERROR;
      return false;
    }
    Serial.println("[FLASH] Format successful.");
  }

  File32 root = fatfs.open("/");
  if (!root || !root.isDirectory())
  {
    Serial.println("[FLASH] Cannot open root directory.");
    return false;
  }

  File32 file;
  uint16_t maxNumber = 0;
  char nazwaPliku[256];

  while ((file = root.openNextFile()))
  {
    if (!file.isDirectory())
    {
      file.getName(nazwaPliku, sizeof(nazwaPliku));
      String name = String(nazwaPliku);
      if (name.startsWith("data_") && name.endsWith(".csv"))
      {
        int num = name.substring(5, name.length() - 4).toInt();
        if (num > maxNumber) maxNumber = num;
      }
    }
    file.close();
  }
  root.close();

  char filename[32];
  snprintf(filename, sizeof(filename), "data_%04d.csv", maxNumber);
  flashDataFile = fatfs.open(filename, FILE_WRITE);
  if (!flashDataFile)
  {
    flashDataFile = fatfs.open(filename, FILE_WRITE | O_CREAT | O_APPEND);
    if (!flashDataFile)
    {
      Serial.println("[FLASH] Cannot open/create file for recovery!");
      return false;
    }
  }

  flashDataFile.seekEnd();
  flashDataFile.println("\n--- RECOVERY AFTER RESET ---");
  flashDataFile.flush();
  Serial.printf("[FLASH] Recovery OK. File: %s\n", filename);
  return true;
}

void Rakieta::flashDumpFileList()
{
  Serial.println("\n=== FLASH FILE LIST ===");

  File32 root = fatfs.open("/");
  if (!root || !root.isDirectory())
  {
    Serial.println("[FLASH] Cannot open root directory.");
    return;
  }

  char nazwaPliku[256];

  while ((dumpFile = root.openNextFile()))
  {
    if (!dumpFile.isDirectory())
    {
      dumpFile.getName(nazwaPliku, sizeof(nazwaPliku));
      String name = String(nazwaPliku);
      if (name.endsWith(".csv"))
      {
        Serial.print(" - ");
        Serial.print(name);
        Serial.print(" (size: ");
        Serial.print(dumpFile.size());
        Serial.println(" bytes)");
      }
    }
    dumpFile.close();
  }
  root.close();
  Serial.println("========================\n");
}

void Rakieta::flashDumpFileData(uint16_t fileNumber)
{
  char filename[32];
  snprintf(filename, sizeof(filename), "data_%04d.csv", fileNumber);

  File32 dumpFile = fatfs.open(filename, FILE_READ);
  if (!dumpFile)
  {
    Serial.printf("[FLASH] Cannot open file %s\n", filename);
    return;
  }

  String line;

  Serial.printf("=== DUMP OF %s ===\n", filename);
  while (dumpFile.available())
  {
    line = dumpFile.readStringUntil('\n');
    Serial.println(line);
  }
  dumpFile.close();
  Serial.println("=== END OF FILE ===\n");
}

void Rakieta::flashDumpLastFile()
{
  File32 root = fatfs.open("/");
  if (!root || !root.isDirectory())
  {
    Serial.println("[FLASH] Cannot open root directory.");
    return;
  }

  uint16_t lastNumber = 0;
  char nazwaPliku[15];

  while ((dumpFile = root.openNextFile()))
  {
    if (!dumpFile.isDirectory())
    {
      dumpFile.getName(nazwaPliku, sizeof(nazwaPliku));
      String name = String(nazwaPliku);
      if (name.startsWith("data_") && name.endsWith(".csv"))
      {
        int num = name.substring(5, name.length() - 4).toInt();
        if (num > lastNumber) lastNumber = num;
      }
    }
    dumpFile.close();
  }
  root.close();

  if (lastNumber == 0)
  {
    Serial.println("[FLASH] No data files found.");
    return;
  }

  flashDumpFileData(lastNumber);
}

// do zastanowienia się
bool Rakieta::detectLaunch()
{
  if (data.lsm.lastTotalAccel > LAUNCH_ACCEL_THRESHOLD)
  {
    uint32_t now = millis();
    if (launchDetectTime == 0) launchDetectTime = now;
    if (now - launchDetectTime >= LAUNCH_DEBOUNCE_MS) return true;
  }
  else launchDetectTime = 0;

  return false;
}

// do zastanowienia się
bool Rakieta::detectBurnout()
{
  if (data.lsm.lastTotalAccel < BURNOUT_ACCEL_THRESHOLD)
  {
    uint32_t now = millis();
    if (burnoutDetectTime == 0) burnoutDetectTime = now;
    if (now - burnoutDetectTime >= BURNOUT_DEBOUNCE_MS) return true;
  }
  else burnoutDetectTime = 0;

  return false;
}

// do zastanowienia się
bool Rakieta::detectApogee()
{
  // Apogeum gdy prędkość spada poniżej progu i osiągnięto maksimum wysokości
  if (data.bmp1.lastVerticalSpeed <= APOGEE_VELOCITY_THRESHOLD && data.bmp1.altitude <= data.bmp1.maxAltitude - 1.0f)
  {
    uint32_t now = millis();
    if (apogeeDetectTime == 0) apogeeDetectTime = now;
    if (now - apogeeDetectTime >= APOGEE_DEBOUNCE_MS) return true;
  }
  else apogeeDetectTime = 0;

  return false;
}

// do zastanowienia się
bool Rakieta::detectLanding()
{
  if (abs(data.bmp1.lastVerticalSpeed) < LANDING_VELOCITY_THRESHOLD)
  {
    uint32_t now = millis();
    if (landedDetectTime == 0) landedDetectTime = now;
    if (now - landedDetectTime >= LANDING_DEBOUNCE_MS) return true;
  }
  else landedDetectTime = 0;
  
  return false;
}

// do zastanowienia się
bool Rakieta::checkDeploymentConditions(ParachuteType type)
{
  float currentSpeed = abs(data.bmp1.lastVerticalSpeed);
  float currentAltitude = data.bmp1.altitude;

  if (type == DROGUE)
  {
    if (currentSpeed > DEPLOY_DROGUE_MAX_SPEED || currentAltitude < DEPLOY_DROGUE_MIN_ALTITUDE) return false;
    return true;
  }
  else if (type == MAIN)
  {
    if (currentSpeed > DEPLOY_MAIN_MAX_SPEED || currentAltitude < DEPLOY_MAIN_MIN_ALTITUDE) return false;
    return drogueDeployed;
  }
  return false;
}

void Rakieta::sendFlightSummary()
{
  uint32_t now = millis();
  String msg = "Flight summary:";
  msg += "launch:" + String(now - launchDetectTime);
  msg += ",burnout:" + String(now - burnoutDetectTime);
  msg += ",apogee:" + String(now - apogeeDetectTime);
  msg += ",descent:" + String(now - descentDetectTime);
  msg += ",landed:" + String(now - landedDetectTime);

  flashWriteString(msg);
  transmit(msg);
  Serial.println(msg);
}

void Rakieta::drogueParashuteOpen()
{
  solenoid1IsOn = true;
  solenoid1OffTime = millis() + SOLENOID_DELAY;
  digitalWrite(SOLENOID_1, HIGH);
}

void Rakieta::mainParashuteOpen()
{
  solenoid2IsOn = true;
  solenoid2OffTime = millis() + SOLENOID_DELAY;
  digitalWrite(SOLENOID_2, HIGH);
}

void Rakieta::updateFlightState()
{
  uint32_t now = millis();
  
  switch (currentFlightState)
  {
    case IDLE:
      if (detectLaunch())
      {
        currentFlightState = BOOST;
        launchDetectTime = now;
        inFlight = true;
        Serial.println("[FLIGHT] -> BOOST");
      }
      break;
    case BOOST:
    {
      if (detectBurnout())
      {
        currentFlightState = COAST;
        burnoutDetectTime = now;
        Serial.println("[FLIGHT] -> COAST");
      }
      break;
    }
    case COAST:
    {
      if (detectApogee())
      {
        currentFlightState = APOGEE;
        apogeeDetectTime = now;
        Serial.println("[FLIGHT] -> APOGEE");
      }
      break;
    }
    case APOGEE:
    {
      if ((!drogueDeployed && checkDeploymentConditions(DROGUE)) || (millis() - launchDetectTime > DROGUE_PARASHUTE_TIMEOUT))
      {
        drogueParashuteOpen();
        drogueDeployed = true;
        currentFlightState = DESCENT;
        descentDetectTime = now;
        Serial.println("[FLIGHT] Drogue parachute deployed.");
        Serial.println("[FLIGHT] -> DESCENT");
      }
      break;
    }
    case DESCENT:
    {
      if ((!mainDeployed && checkDeploymentConditions(MAIN)) || (millis() - launchDetectTime > MAIN_PARASHUTE_TIMEOUT))
      {
        mainParashuteOpen();
        mainDeployed = true;
        Serial.println("[FLIGHT] Main parachute deployed.");
      }

      if (detectLanding())
      {
        currentFlightState = LANDED;
        landedDetectTime = now;
        Serial.println("[FLIGHT] -> LANDED");
      }
      break;
    }
    case LANDED:
      sendFlightSummary();
      inFlight = false;
      flashCloseFile();
      break;

    default:
      break;
  }
}

void Rakieta::initWatchdog()
{
  // Włącz watchdog z timeoutem ~8 sekund (LSI ~32 kHz, preskaler 64, reload 4000)
  IWDG1->KR = 0x5555;   // odblokowanie dostępu do rejestrów
  IWDG1->PR = 4;        // preskaler 64 (2^4 = 64)
  IWDG1->RLR = 4000;    // reload value
  IWDG1->KR = 0xAAAA;   // odświeżenie (wymagane przed uruchomieniem)
  IWDG1->KR = 0xCCCC;   // uruchomienie
}

void Rakieta::watchdog()
{
  if (inFlight)
  {
    IWDG1->KR = 0xAAAA;
    lastWatchdogTime = millis();
  }
}





// === Obsługa trybów ===
void Rakieta::handleDebugMode()  /// do zmieniania w locie
{
  Serial.println("\n=== TRYB DEBUG: wypisywanie czujnikow ===");
  Serial.println("----------------------------------------");

  checkRadio();

  /// trzeba usuwać i sprawdzać po kolei, na końcu zamienić na readSensorsData()
  handleLsm();
  handleBmp();
  handleAdxl();
  handleGPS();
  handleBattery();

  printData();
  systemSleep(2500);
}

void Rakieta::handleFlightMode()
{
  uint32_t lastRun = 0;
  uint32_t interval = inFlight ? 20 : 60000;  /// do zmiany na define
  
  checkRadio();

  uint32_t now = millis();

  if (now - lastRun >= interval)
  {
    lastRun = now;

    readSensorsData();
    updateFlightState();

    String msg = prepareDataLineMsg();
    sendPacket();
    flashWriteString(msg);
  }
}

void Rakieta::handleDumpMode()  /// nie wiem co tu się dzieje obecnie
{
  const uint32_t startAddr = 0;
  const uint32_t totalLength = 512;
  uint8_t buffer[16];
  char ascii[17];
  
  for (uint32_t i = 0; i < 16 && dumpAddress < totalLength; i++)
  {
    flash.readBuffer(dumpAddress, buffer, 16);
    
    Serial.printf("%08X: ", dumpAddress);
    for (int j = 0; j < 16; j++)
    {
      Serial.printf("%02X ", buffer[j]);
      ascii[j] = (buffer[j] >= 32 && buffer[j] <= 126) ? buffer[j] : '.';
    }
    ascii[16] = '\0';
    Serial.printf(" | %s\n", ascii);
    
    dumpAddress += 16;
  }
  
  if (dumpAddress >= totalLength)
  {
    Serial.println("\n--- Koniec dump. Przechodzę do SLEEP ---");
    currentMode = SLEEP;
  }
}

void Rakieta::handleSleepMode()  /// chyba będzie ok
{
  SystemMode oldMode = currentMode;
  setSystemMode();
  
  if (currentMode != oldMode)
  {
    Serial.println("Zmiana trybu – resetowanie systemu...");
    systemSleep(100);
    systemReset();
  }
  else
  {
    currentMode = SLEEP;
  }
  
  // TODO: tu można dodać faktyczne uśpienie (__WFI())
  systemSleep(5000);
}





void Rakieta::init()
{
  Serial.println("\n=== ROCKET ON-BOARD COMPUTER INIT ===\n");

  initGPIO();
  initSPI();

  initLSM();
  initADXL();
  initBMP();
  initGPS();
  initLora();
  initFlash();

  setSystemMode();
  printSystemMode();

  uint8_t ile = 5;
  if (errorFlags)
  {
    ile = 10;
    Serial.println(errorFlags);
  }

  for (int i=0; i<ile; i++)
  {
    digitalWrite(LED_1, HIGH);
    systemSleep(200);
    digitalWrite(LED_1, LOW);
    systemSleep(200);
  }
  
  if (currentMode != SLEEP) setOffsets();

  Serial.println("\n=== INICJALIZACJA ZAKOŃCZONA ===\n");
}

void Rakieta::loop()
{
  uint32_t now = millis();
  
  switch (currentMode)
  {
    case DEBUG:
      if (now - lastDebugPrint >= INTERVAL_DEBUG)
      {
        lastDebugPrint = now;
        handleDebugMode();
      }
      break;
    case FLIGHT:
      if (!flashDataFile) flashOpenNewFile();

      if (now - lastFlightLoop >= INTERVAL_FLIGHT)
      {
        lastFlightLoop = now;
        handleFlightMode();
      }
      break;
    case DUMP:
      if (now - lastDumpProgress >= INTERVAL_DUMP)
      {
        lastDumpProgress = now;
        handleDumpMode();
      }
      break;
    case SLEEP:
      if (now - lastSleepCheck >= INTERVAL_SLEEP)
      {
        lastSleepCheck = now;
        handleSleepMode();
      }
      break;
  }

  updateLeds();
  updateBuzzer();
  updateSolenoid();

  systemSleep(5);
}











