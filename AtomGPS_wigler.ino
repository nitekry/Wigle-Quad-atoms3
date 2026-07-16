#include <M5Unified.h>
#include <TinyGPS++.h>

HardwareSerial GPS_Serial(2);
TinyGPSPlus gps;

#define GPS_RX_PIN 5      // GPS TX -> AtomS3 G5
#define GPS_BAUD   9600

#define GREEN   0x07E0
#define ORANGE  0xFD20
#define RED     0xF800

const double latMin = 0.00000;
const double latMax = 0.00001;
const double lonMin = -0.00000;
const double lonMax = -0.00001;
const double warningDistanceMeters = 1000.0;

uint16_t lastColor = 0xFFFF;   // force first paint

double haversine(double lat1, double lon1, double lat2, double lon2) {
  const double R = 6371000.0;
  double dLat = radians(lat2 - lat1);
  double dLon = radians(lon2 - lon1);
  double a = sin(dLat/2)*sin(dLat/2) +
             cos(radians(lat1))*cos(radians(lat2)) *
             sin(dLon/2)*sin(dLon/2);
  return R * 2 * atan2(sqrt(a), sqrt(1 - a));
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
  if (c == lastColor) return;      // avoid constant full-screen repaints
  M5.Display.fillScreen(c);
  lastColor = c;
}

void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);
  M5.Display.setRotation(0);
  M5.Display.setTextSize(2);
  M5.Display.setTextColor(WHITE, BLACK);
  M5.Display.setCursor(6, 40);
  M5.Display.println("Atomic GPS");

  Serial.begin(115200);
  GPS_Serial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, -1);

  delay(1000);
  M5.Display.fillScreen(BLACK);
}

void loop() {
  M5.update();

  while (GPS_Serial.available()) gps.encode(GPS_Serial.read());

  if (gps.location.isUpdated() && gps.location.isValid()) {
    double lat = gps.location.lat();
    double lon = gps.location.lng();

    if (insideBoundary(lat, lon))      showColor(GREEN);
    else if (nearBoundary(lat, lon))   showColor(ORANGE);
    else                               showColor(RED);

    Serial.printf("Lat: %.6f Lon: %.6f Sats: %d\n",
                  lat, lon, gps.satellites.value());
  }

  // no-fix indicator
  if (millis() > 10000 && gps.charsProcessed() < 10) {
    M5.Display.setCursor(6, 40);
    M5.Display.print("NO GPS DATA");
  }
}
