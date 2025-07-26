#include <M5AtomS3.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>

HardwareSerial GPS_Serial(2);
TinyGPSPlus gps;

#define GPS_RX_PIN 5  // GPS TX → Atom RX
// #define GPS_TX_PIN 6  // Atom TX → GPS RX
#define GPS_BAUD    9600

#define GREEN   0x07E0
#define ORANGE  0xFD20
#define RED     0xF800

// Enter your quad here
const double latMin = 30.012526;
const double latMax = 30.049381;
const double lonMin = -95.65976;
const double lonMax = -95.5856;
const double warningDistanceMeters = 1000.0;

double haversine(double lat1, double lon1, double lat2, double lon2) {
  const double R = 6371000.0;
  double dLat = radians(lat2 - lat1);
  double dLon = radians(lon2 - lon1);
  double a = sin(dLat/2)*sin(dLat/2) +
             cos(radians(lat1))*cos(radians(lat2)) *
             sin(dLon/2)*sin(dLon/2);
  double c = 2*atan2(sqrt(a), sqrt(1 - a));
  return R*c;
}

bool insideBoundary(double lat, double lon) {
  return (lat >= latMin && lat <= latMax && lon >= lonMin && lon <= lonMax);
}

bool nearBoundary(double lat, double lon) {
  double nlat = constrain(lat, latMin, latMax);
  double nlon = constrain(lon, lonMin, lonMax);
  return haversine(lat, lon, nlat, nlon) <= warningDistanceMeters;
}

void showColor(uint16_t c) {
  M5.Display.fillScreen(c);
}

void setup() {
  M5.begin();
  M5.Display.setTextSize(1);
  M5.Display.setTextColor(WHITE);
  M5.Display.setCursor(10,10);
  M5.Display.println("Atomic GPS Base");

  Serial.begin(115200);
  GPS_Serial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  delay(1000);
  M5.Display.fillScreen(BLACK);
}

void loop() {
  while (GPS_Serial.available()) gps.encode(GPS_Serial.read());

  if (gps.location.isUpdated()) {
    double lat = gps.location.lat();
    double lon = gps.location.lng();
    if (insideBoundary(lat, lon)) showColor(GREEN);
    else if (nearBoundary(lat, lon)) showColor(ORANGE);
    else showColor(RED);

    Serial.printf("Lat: %.6f Lon: %.6f\n", lat, lon);
  }
}
