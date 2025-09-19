#include <M5Cardputer.h>
#include <TinyGPSPlus.h>
#include <SD.h>
#include <vector>
#include <cmath>

#define GPS_RX 1
#define GPS_TX 2

HardwareSerial GPS_Serial(1);
TinyGPSPlus gps;

File gpxFile;
bool isRecording = false;
String gpxFilename;

String lastGGA;
float hdop = 0.0;
int fixQuality = 0;

const int screen_w = 240;
const int screen_h = 135;
const int gap = 4;
const int total_w = screen_w - 3 * gap;
const int col0_w = total_w * 0.5;
const int col1_w = total_w * 0.5;
const int field_h = 14;
const int field_gap = 1;
bool screenOn = true;

#define SCREEN_DATA 0
#define SCREEN_MAP 1
#define SCREEN_WAYPOINT_SELECT 2
int currentScreen = SCREEN_DATA;

struct Waypoint {
  String name;
  double lat;
  double lon;
};

std::vector<Waypoint> waypoints;
int selectedWaypoint = 0;
double centerLat = 0.0, centerLon = 0.0, zoom = 1.0;

String lastMessage;
uint32_t lastMessageTime = 0;
bool forceRedraw = true;

// Key codes
#define KEY_REC 'r'
#define KEY_SAVE_WP 's'
#define KEY_MAP 'm'
#define KEY_DISPLAY_TOGGLE 'd'
#define KEY_WP_LIST 'n'
#define KEY_HOME 'h'       // NEW: ritorna alla schermata home
#define KEY_ENTER 'x'
#define KEY_UP    'o'
#define KEY_DOWN  'k'
#define KEY_LEFT  'j'
#define KEY_RIGHT 'g'
#define KEY_PLUS  '+'
#define KEY_MINUS '-'

// Debounce/time threshold
const uint32_t KEY_DEBOUNCE_MS = 180;
uint32_t lastKeyTime = 0;

// Constants for tuning (NEW)
const double BASE_PX_PER_M = 0.05;
const double PAN_STEP_DEG = 0.001;

///////////////////////
// Utility functions //
///////////////////////

void showMessage(const String &msg) {
  lastMessage = msg;
  lastMessageTime = millis();
  forceRedraw = true;
}

double haversineDist(double lat1, double lon1, double lat2, double lon2) { // NEW
  const double R = 6371000.0;
  double dLat = radians(lat2 - lat1);
  double dLon = radians(lon2 - lon1);
  double a = sin(dLat/2)*sin(dLat/2) +
             cos(radians(lat1))*cos(radians(lat2))*
             sin(dLon/2)*sin(dLon/2);
  return R * 2 * atan2(sqrt(a), sqrt(1-a));
}

double bearingDeg(double lat1, double lon1, double lat2, double lon2) { // NEW
  double dLon = radians(lon2 - lon1);
  lat1 = radians(lat1); lat2 = radians(lat2);
  double y = sin(dLon) * cos(lat2);
  double x = cos(lat1)*sin(lat2) - sin(lat1)*cos(lat2)*cos(dLon);
  return fmod((degrees(atan2(y, x)) + 360.0), 360.0);
}

// Robust parse of GGA NMEA line
void parseGGA(const String &gga) {
  if (gga.length() < 6) return;
  int start = 0;
  int field = 0;
  int len = gga.length();
  for (int i = 0; i <= len; ++i) {
    char c = (i < len) ? gga[i] : ',';
    if (c == ',' || c == '*') {
      String val = gga.substring(start, i);
      if (field == 5) fixQuality = val.toInt();
      else if (field == 7) hdop = val.toFloat();
      start = i + 1;
      field++;
      if (c == '*') break;
    }
  }
}

void drawField(int x, int w, int y, const char* label, const char* value) {
  M5Cardputer.Display.drawRect(x, y, w, field_h, TFT_WHITE);
  M5Cardputer.Display.setTextColor(TFT_WHITE, TFT_BLACK);
  M5Cardputer.Display.setCursor(x + 2, y + 3);
  M5Cardputer.Display.setTextSize(1);
  M5Cardputer.Display.printf("%-5s: %s", label, value);
}

//////////////////////
// GPX / Waypoints  //
//////////////////////

String makeGPXFilename() {
  char buf[64];
  if (gps.date.isValid() && gps.time.isValid()) {
    snprintf(buf, sizeof(buf), "/track_%04d%02d%02d_%02d%02d%02d.gpx",
             gps.date.year(), gps.date.month(), gps.date.day(),
             gps.time.hour(), gps.time.minute(), gps.time.second());
  } else {
    snprintf(buf, sizeof(buf), "/track_%lu.gpx", millis() / 1000);
  }
  return String(buf);
}

bool startRecordingGPX() {
  if (isRecording) return false;
  if (!SD.cardType()) { // NEW: check SD
    showMessage("No SD");
    return false;
  }
  gpxFilename = makeGPXFilename();
  gpxFile = SD.open(gpxFilename.c_str(), FILE_WRITE);
  if (!gpxFile) {
    showMessage("SD write fail");
    return false;
  }
  gpxFile.println("<?xml version='1.0' encoding='UTF-8'?>");
  gpxFile.println("<gpx version='1.1' creator='M5Cardputer'>");
  gpxFile.println("<trk><name>M5 Track</name><trkseg>");
  isRecording = true;
  showMessage("Recording ON");
  return true;
}

int gpxPointsSinceFlush = 0;
const int GPX_FLUSH_PERIOD_POINTS = 10;

void writeGPXPointIfNeeded() {
  if (!isRecording || !gpxFile) return;
  if (!gps.location.isValid()) return;

  char buf[256];
  int n = snprintf(buf, sizeof(buf),
    "<trkpt lat=\"%.6f\" lon=\"%.6f\">", gps.location.lat(), gps.location.lng());
  gpxFile.println(buf);

  if (gps.date.isValid() && gps.time.isValid()) {
    char tbuf[64];
    snprintf(tbuf, sizeof(tbuf), "<time>%04d-%02d-%02dT%02d:%02d:%02dZ</time>",
             gps.date.year(), gps.date.month(), gps.date.day(),
             gps.time.hour(), gps.time.minute(), gps.time.second());
    gpxFile.println(tbuf);
  }

  if (gps.altitude.isValid())
    gpxFile.printf("<ele>%.2f</ele>\n", gps.altitude.meters());
  if (gps.speed.isValid())
    gpxFile.printf("<speed>%.2f</speed>\n", gps.speed.kmph());
  if (gps.course.isValid())
    gpxFile.printf("<course>%.1f</course>\n", gps.course.deg());
  if (hdop > 0.0)
    gpxFile.printf("<hdop>%.2f</hdop>\n", hdop);

  gpxFile.println("</trkpt>");

  gpxPointsSinceFlush++;
  if (gpxPointsSinceFlush >= GPX_FLUSH_PERIOD_POINTS) {
    gpxFile.flush();
    gpxPointsSinceFlush = 0;
  }
}

void stopRecordingGPX() {
  if (!isRecording) return;
  if (gpxFile) {
    gpxFile.println("</trkseg></trk></gpx>");
    gpxFile.flush();
    gpxFile.close();
  }
  isRecording = false;
  showMessage("Recording OFF");
  gpxPointsSinceFlush = 0;
}

void saveWaypoint(const String &name, double lat, double lon) {
  if (isnan(lat) || isnan(lon)) {
    showMessage("Invalid WP");
    return;
  }
  Waypoint wp = {name, lat, lon};
  waypoints.push_back(wp);
  File f = SD.open("/waypoints.txt", FILE_APPEND);
  if (f) {
    f.printf("%s,%.6f,%.6f\n", name.c_str(), lat, lon);
    f.close();
    // NEW: aggiungi anche al file GPX se attivo
    if (isRecording && gpxFile) {
      gpxFile.printf("<wpt lat=\"%.6f\" lon=\"%.6f\"><name>%s</name></wpt>\n", lat, lon, name.c_str());
      gpxFile.flush();
    }
    showMessage("Saved " + name);
  } else {
    showMessage("WP save fail");
  }
  forceRedraw = true;
}

void loadWaypoints() {
  waypoints.clear();
  File f = SD.open("/waypoints.txt");
  if (!f) return;
  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) continue;
    int i1 = line.indexOf(',');
    int i2 = line.lastIndexOf(',');
    if (i1 > 0 && i2 > i1) {
      String name = line.substring(0, i1);
      double lat = line.substring(i1 + 1, i2).toDouble();
      double lon = line.substring(i2 + 1).toDouble();
      if (!isnan(lat) && !isnan(lon)) {
        waypoints.push_back({name, lat, lon});
      }
    }
  }
  f.close();
}

//////////////////////
// Drawing screens   //
//////////////////////

double degreesToPixelsLat(double deg) {
  return deg * 111320.0 * (zoom * BASE_PX_PER_M);
}
double degreesToPixelsLon(double deg, double atLat) {
  return deg * 111320.0 * cos(atLat * M_PI / 180.0) * (zoom * BASE_PX_PER_M);
}

void drawMap() {
  M5Cardputer.Display.startWrite();
  M5Cardputer.Display.fillScreen(TFT_BLACK);

  int cx = screen_w / 2;
  int cy = screen_h / 2;

  for (auto &wp : waypoints) {
    double dxDeg = wp.lon - centerLon;
    double dyDeg = wp.lat - centerLat;
    int x = cx + (int)degreesToPixelsLon(dxDeg, centerLat);
    int y = cy - (int)degreesToPixelsLat(dyDeg);
    if (x < -10 || x > screen_w + 10 || y < -10 || y > screen_h + 10) continue;
    M5Cardputer.Display.fillCircle(x, y, 3, TFT_GREEN);
    M5Cardputer.Display.setCursor(x + 5, y - 3);
    M5Cardputer.Display.setTextColor(TFT_WHITE);
    M5Cardputer.Display.setTextSize(1);
    M5Cardputer.Display.print(wp.name);
  }

  // center marker (color by fix quality / hdop)
  uint16_t color = TFT_RED;
  if (fixQuality > 0) {
    if (hdop < 1.5) color = TFT_GREEN;
    else if (hdop < 3.0) color = TFT_YELLOW;
    else color = TFT_ORANGE;
  }
  M5Cardputer.Display.fillCircle(cx, cy, 4, color);

  // direction arrow if course available
  if (gps.course.isValid()) {
    double ang = gps.course.deg() * M_PI / 180.0;
    int x2 = cx + 10 * cos(ang);
    int y2 = cy - 10 * sin(ang);
    M5Cardputer.Display.drawLine(cx, cy, x2, y2, TFT_CYAN);
  }

  M5Cardputer.Display.setTextSize(1);
  M5Cardputer.Display.setTextColor(TFT_CYAN, TFT_BLACK);
  M5Cardputer.Display.setCursor(4, screen_h - 12);
  M5Cardputer.Display.printf("Z:%.2f", zoom);

  M5Cardputer.Display.endWrite();
  forceRedraw = false;
}

void drawDataScreen() {
  M5Cardputer.Display.fillScreen(TFT_BLACK);
  M5Cardputer.Display.setTextSize(1);
  M5Cardputer.Display.setTextColor(TFT_CYAN, TFT_BLACK);
  M5Cardputer.Display.setCursor(gap, 2);
  M5Cardputer.Display.println("GPS Viewer");

  int y0 = 14;
  int x0 = gap;
  int x1 = gap * 2 + col0_w;

  char lat[32], alt[32], date[32], qual[32], crs[32];
  if (gps.location.isValid()) snprintf(lat, sizeof(lat), "%.6f", gps.location.lat());
  else snprintf(lat, sizeof(lat), "NoFix");
  drawField(x0, col0_w, y0, "Lat", lat);

  if (gps.altitude.isValid()) snprintf(alt, sizeof(alt), "%.2f", gps.altitude.meters());
  else snprintf(alt, sizeof(alt), "NoData");
  drawField(x0, col0_w, y0 + (field_h + field_gap), "Alt", alt);

  if (gps.date.isValid()) snprintf(date, sizeof(date), "%02d/%02d/%02d", gps.date.day(), gps.date.month(), gps.date.year() % 100);
  else snprintf(date, sizeof(date), "NoData");
  drawField(x0, col0_w, y0 + 2 * (field_h + field_gap), "Date", date);

  snprintf(qual, sizeof(qual), "%d", fixQuality);
  drawField(x0, col0_w, y0 + 3 * (field_h + field_gap), "Qual", qual);

  if (gps.course.isValid()) snprintf(crs, sizeof(crs), "%.1f", gps.course.deg());
  else snprintf(crs, sizeof(crs), "NoData");
  drawField(x0, col0_w, y0 + 4 * (field_h + field_gap), "Crs", crs);

  char lng[32], timebuf[32], sats[32], spd[32], hdopStr[32];
  if (gps.location.isValid()) snprintf(lng, sizeof(lng), "%.6f", gps.location.lng());
  else snprintf(lng, sizeof(lng), "NoFix");
  drawField(x1, col1_w, y0, "Lng", lng);

  if (gps.time.isValid()) snprintf(timebuf, sizeof(timebuf), "%02d:%02d:%02d", gps.time.hour(), gps.time.minute(), gps.time.second());
  else snprintf(timebuf, sizeof(timebuf), "NoData");
  drawField(x1, col1_w, y0 + (field_h + field_gap), "Time", timebuf);

  if (gps.satellites.isValid()) snprintf(sats, sizeof(sats), "%d", gps.satellites.value());
  else snprintf(sats, sizeof(sats), "NoData");
  drawField(x1, col1_w, y0 + 2 * (field_h + field_gap), "Sat", sats);

  if (gps.speed.isValid()) snprintf(spd, sizeof(spd), "%.1f", gps.speed.kmph());
  else snprintf(spd, sizeof(spd), "0.0");
  drawField(x1, col1_w, y0 + 3 * (field_h + field_gap), "Spd", spd);

  snprintf(hdopStr, sizeof(hdopStr), "%.2f", hdop);
  drawField(x1, col1_w, y0 + 4 * (field_h + field_gap), "HDOP", hdopStr);

  if (isRecording) {
    M5Cardputer.Display.setTextColor(TFT_RED, TFT_BLACK);
    M5Cardputer.Display.setCursor(screen_w - 60, 2);
    M5Cardputer.Display.print("[REC]");
  }

  int level = M5Cardputer.Power.getBatteryLevel();
  char battery[16];
  snprintf(battery, sizeof(battery), "%d%%", level);
  M5Cardputer.Display.setCursor(screen_w - 40, screen_h - 12);
  M5Cardputer.Display.setTextColor(TFT_GREEN, TFT_BLACK);
  M5Cardputer.Display.print(battery);

  if (millis() - lastMessageTime < 2500) {
    M5Cardputer.Display.setTextColor(TFT_YELLOW, TFT_BLACK);
    M5Cardputer.Display.setCursor(gap, screen_h - 12);
    M5Cardputer.Display.print(lastMessage);
  }
}

void drawWaypointSelect() {
  M5Cardputer.Display.fillScreen(TFT_BLACK);
  M5Cardputer.Display.setTextSize(1);
  M5Cardputer.Display.setTextColor(TFT_CYAN, TFT_BLACK);
  M5Cardputer.Display.setCursor(gap, 2);
  M5Cardputer.Display.println("Waypoints");
  int y = 18;
  int idx = 0;
  for (auto &wp : waypoints) {
    if (idx == selectedWaypoint) {
      M5Cardputer.Display.fillRect(gap, y-2, screen_w - 2*gap, field_h, TFT_WHITE);
      M5Cardputer.Display.setCursor(gap+2, y);
      M5Cardputer.Display.setTextColor(TFT_BLACK, TFT_WHITE);
    } else {
      M5Cardputer.Display.setTextColor(TFT_WHITE, TFT_BLACK);
      M5Cardputer.Display.setCursor(gap+2, y);
    }
    M5Cardputer.Display.printf("%02d %s", idx, wp.name.c_str());
    // NEW: distanza e bearing se GPS valido
    if (gps.location.isValid()) {
      double dist = haversineDist(gps.location.lat(), gps.location.lng(), wp.lat, wp.lon);
      double brg = bearingDeg(gps.location.lat(), gps.location.lng(), wp.lat, wp.lon);
      M5Cardputer.Display.printf(" %.0fm %.0f°", dist, brg);
    }
    y += field_h + field_gap;
    idx++;
    if (y > screen_h - 20) break;
  }
}

//////////////////////
// Setup / Loop     //
//////////////////////

void setup() {
  auto cfg = M5.config();
  M5Cardputer.begin(cfg);
  M5Cardputer.Display.setBrightness(32);

  // start GPS at 115200 as requested
  GPS_Serial.begin(115200, SERIAL_8N1, GPS_RX, GPS_TX);

  if (!SD.begin()) {
    M5Cardputer.Display.setTextColor(TFT_RED, TFT_BLACK);
    M5Cardputer.Display.setCursor(0, 0);
    M5Cardputer.Display.println("SD Card Fail!");
    while (1) delay(1000);
  }

  loadWaypoints();

  // initial center - if we have a fix, center on it
  if (gps.location.isValid()) {
    centerLat = gps.location.lat();
    centerLon = gps.location.lng();
  }
  forceRedraw = true;
}

void handleKeyPresses() {
  static bool mLast = false, sLast = false, dLast = false;
  static bool rLast = false;
  bool mNow = M5Cardputer.Keyboard.isKeyPressed('m');
  bool sNow = M5Cardputer.Keyboard.isKeyPressed('s');
  bool dNow = M5Cardputer.Keyboard.isKeyPressed('d');
  bool rNow = M5Cardputer.Keyboard.isKeyPressed('r');

  uint32_t now = millis();
  // simple debounce: ensure a minimum time between key actions
  if (now - lastKeyTime < KEY_DEBOUNCE_MS) {
    // but we still update last states to avoid sticky transitions
    mLast = mNow; sLast = sNow; dLast = dNow; rLast = rNow;
    return;
  }

  // RECORD toggle
  if (rNow && !rLast) {
    if (!isRecording) {
      if (startRecordingGPX()) lastKeyTime = now;
    } else {
      stopRecordingGPX();
      lastKeyTime = now;
    }
    forceRedraw = true;
  }
  rLast = rNow;

  // Display toggle
  if (dNow && !dLast) {
    screenOn = !screenOn;
    M5Cardputer.Display.setBrightness(screenOn ? 32 : 0);
    lastKeyTime = now;
  }
  dLast = dNow;

  // Save waypoint
  if (sNow && !sLast) {
    if (gps.location.isValid()) {
      char name[32];
      snprintf(name, sizeof(name), "WP%02d_%02d%02d_%02d%02d",
               (int)waypoints.size() % 100,
               gps.date.day(), gps.date.month(),
               gps.time.hour(), gps.time.minute());
      saveWaypoint(String(name), gps.location.lat(), gps.location.lng());
      lastKeyTime = now;
    } else {
      showMessage("No GPS fix");
      lastKeyTime = now;
    }
    forceRedraw = true;
  }
  sLast = sNow;

  // Map: center on current position and open map
  if (mNow && !mLast) {
    if (gps.location.isValid()) {
      centerLat = gps.location.lat();
      centerLon = gps.location.lng();
    }
    currentScreen = SCREEN_MAP;
    forceRedraw = true;
    lastKeyTime = now;
  }
  mLast = mNow;

  // Waypoint list
  if (M5Cardputer.Keyboard.isKeyPressed('n')) {
    currentScreen = SCREEN_WAYPOINT_SELECT;
    selectedWaypoint = 0;
    forceRedraw = true;
    lastKeyTime = now;
  }

  // Enter/back
  if (M5Cardputer.Keyboard.isKeyPressed(KEY_ENTER)) {
    currentScreen = SCREEN_DATA;
    forceRedraw = true;
    lastKeyTime = now;
  }

  // Navigation in map or waypoint list
  if (currentScreen == SCREEN_MAP) {
    if (M5Cardputer.Keyboard.isKeyPressed(KEY_UP)) {
      centerLat += 0.001 * zoom;
      forceRedraw = true;
      lastKeyTime = now;
    }
    if (M5Cardputer.Keyboard.isKeyPressed(KEY_DOWN)) {
      centerLat -= 0.001 * zoom;
      forceRedraw = true;
      lastKeyTime = now;
    }
    if (M5Cardputer.Keyboard.isKeyPressed(KEY_LEFT)) {
      centerLon -= 0.001 * zoom;
      forceRedraw = true;
      lastKeyTime = now;
    }
    if (M5Cardputer.Keyboard.isKeyPressed(KEY_RIGHT)) {
      centerLon += 0.001 * zoom;
      forceRedraw = true;
      lastKeyTime = now;
    }
    // zoom with + / -
    if (M5Cardputer.Keyboard.isKeyPressed('+')) {
      zoom *= 1.2;
      forceRedraw = true;
      lastKeyTime = now;
    }
    if (M5Cardputer.Keyboard.isKeyPressed('-')) {
      zoom /= 1.2;
      if (zoom < 0.01) zoom = 0.01;
      forceRedraw = true;
      lastKeyTime = now;
    }
  } else if (currentScreen == SCREEN_WAYPOINT_SELECT) {
    if (M5Cardputer.Keyboard.isKeyPressed(KEY_UP)) {
      if (selectedWaypoint > 0) selectedWaypoint--;
      forceRedraw = true;
      lastKeyTime = now;
    }
    if (M5Cardputer.Keyboard.isKeyPressed(KEY_DOWN)) {
      if (selectedWaypoint < (int)waypoints.size() - 1) selectedWaypoint++;
      forceRedraw = true;
      lastKeyTime = now;
    }
    // center map on selected waypoint with Enter
    if (M5Cardputer.Keyboard.isKeyPressed(KEY_ENTER)) {
      if (selectedWaypoint >= 0 && selectedWaypoint < (int)waypoints.size()) {
        centerLat = waypoints[selectedWaypoint].lat;
        centerLon = waypoints[selectedWaypoint].lon;
        currentScreen = SCREEN_MAP;
        forceRedraw = true;
        lastKeyTime = now;
      }
    }
  }
}

void processGPSChar(char c) {
  gps.encode(c);
  static String nmeaLine = "";
  if (c == '\n') {
    if (nmeaLine.startsWith("$GPGGA")) {
      lastGGA = nmeaLine;
      parseGGA(lastGGA); // update hdop/fixQuality reliably
    }
    nmeaLine = "";
  } else if (c != '\r') {
    nmeaLine += c;
    // limit growth
    if (nmeaLine.length() > 120) nmeaLine.remove(0, nmeaLine.length() - 120);
  }
}

void loop() {
  M5Cardputer.update();

  handleKeyPresses();

  // Read GPS
  while (GPS_Serial.available()) {
    char c = (char)GPS_Serial.read();
    processGPSChar(c);
  }

  // Recording: write a point every 3 seconds
  static uint32_t lastWrite = 0;
  if (millis() - lastWrite > 3000) {
    if (isRecording) writeGPXPointIfNeeded();
    lastWrite = millis();
  }

  // Display updates
  static uint32_t lastDisplay = 0;
  if ((millis() - lastDisplay > 500 || forceRedraw) && screenOn) {
    lastDisplay = millis();
    forceRedraw = false;
    if (currentScreen == SCREEN_DATA) {
      drawDataScreen();
    } else if (currentScreen == SCREEN_MAP) {
      drawMap();
    } else if (currentScreen == SCREEN_WAYPOINT_SELECT) {
      drawWaypointSelect();
    }
    // small safeguard: if recording and SD unexpectedly removed, stop safely
    if (isRecording && !SD.cardType()) {
      stopRecordingGPX();
      showMessage("SD removed");
    }
  }

  delay(10);
}
