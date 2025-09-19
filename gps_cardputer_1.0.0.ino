#include <M5Cardputer.h>
#include <TinyGPSPlus.h>
#include <SD.h>

#define GPS_RX 1
#define GPS_TX 2

HardwareSerial GPS_Serial(1);
TinyGPSPlus gps;

File gpxFile;
bool isRecording = false;
String filename;

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

void parseGGA(const String& gga) {
  int field = 0;
  int lastIndex = 0;
  for (int i = 0; i < gga.length(); ++i) {
    if (gga[i] == ',' || gga[i] == '*') {
      String value = gga.substring(lastIndex, i);
      lastIndex = i + 1;
      field++;
      if (field == 6) fixQuality = value.toInt();
      if (field == 9) hdop = value.toFloat();
      if (field > 9) break;
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

void createGPX() {
  if (!isRecording) {
    filename = "/track_";
    if (gps.date.isValid()) {
      filename += String(gps.date.year()) + "-";
      filename += String(gps.date.month()) + "-";
      filename += String(gps.date.day()) + "_";
    }
    filename += String(millis() / 1000);
    filename += ".gpx";

    gpxFile = SD.open(filename, FILE_WRITE);
    if (gpxFile) {
      gpxFile.println("<?xml version='1.0' encoding='UTF-8'?>");
      gpxFile.println("<gpx version='1.1'>");
      gpxFile.println("<trk><trkseg>");
      isRecording = true;
    }
  }
}

void writeGPXPoint() {
  if (gps.location.isValid() && isRecording && gpxFile) {
    gpxFile.print("<trkpt lat=\"");
    gpxFile.print(gps.location.lat(), 6);
    gpxFile.print("\" lon=\"");
    gpxFile.print(gps.location.lng(), 6);
    gpxFile.println("\">");

    gpxFile.print("<time>");
    if (gps.date.isValid() && gps.time.isValid()) {
      gpxFile.printf("%04d-%02d-%02dT%02d:%02d:%02dZ",
        gps.date.year(), gps.date.month(), gps.date.day(),
        gps.time.hour(), gps.time.minute(), gps.time.second());
    }
    gpxFile.println("</time>");

    if (gps.altitude.isValid()) {
      gpxFile.print("<ele>");
      gpxFile.print(gps.altitude.meters());
      gpxFile.println("</ele>");
    }
    if (gps.speed.isValid()) {
      gpxFile.print("<speed>");
      gpxFile.print(gps.speed.kmph());
      gpxFile.println("</speed>");
    }
    if (gps.course.isValid()) {
      gpxFile.print("<course>");
      gpxFile.print(gps.course.deg());
      gpxFile.println("</course>");
    }
    if (hdop > 0.0) {
      gpxFile.print("<hdop>");
      gpxFile.print(hdop, 1);
      gpxFile.println("</hdop>");
    }
    gpxFile.println("</trkpt>");
    gpxFile.flush();
  }
}

void stopRecording() {
  if (isRecording && gpxFile) {
    gpxFile.println("</trkseg></trk></gpx>");
    gpxFile.close();
    isRecording = false;
  }
}

void saveWaypoint(String name, double lat, double lon) {
  Waypoint wp = { name, lat, lon };
  waypoints.push_back(wp);
  File f = SD.open("/waypoints.txt", FILE_APPEND);
  if (f) {
    f.printf("%s,%.6f,%.6f\n", name.c_str(), lat, lon);
    f.close();
  }
  lastMessage = "Saved " + name;
  lastMessageTime = millis();
  forceRedraw = true;
}

void loadWaypoints() {
  waypoints.clear();
  File f = SD.open("/waypoints.txt");
  if (f) {
    while (f.available()) {
      String line = f.readStringUntil('\n');
      int i1 = line.indexOf(',');
      int i2 = line.lastIndexOf(',');
      if (i1 > 0 && i2 > i1) {
        String name = line.substring(0, i1);
        double lat = line.substring(i1+1, i2).toDouble();
        double lon = line.substring(i2+1).toDouble();
        waypoints.push_back({name, lat, lon});
      }
    }
    f.close();
  }
}

void drawMap() {
  M5Cardputer.Display.startWrite();
  M5Cardputer.Display.fillScreen(TFT_BLACK);
  for (auto& wp : waypoints) {
    int x = screen_w / 2 + (wp.lon - centerLon) * zoom * 100;
    int y = screen_h / 2 - (wp.lat - centerLat) * zoom * 100;
    M5Cardputer.Display.fillCircle(x, y, 3, TFT_GREEN);
    M5Cardputer.Display.setCursor(x + 5, y - 3);
    M5Cardputer.Display.setTextColor(TFT_WHITE);
    M5Cardputer.Display.setTextSize(1);
    M5Cardputer.Display.print(wp.name);
  }
  M5Cardputer.Display.drawCircle(screen_w / 2, screen_h / 2, 4, TFT_RED);
  M5Cardputer.Display.endWrite();
}

void setup() {
  auto cfg = M5.config();
  M5Cardputer.begin(cfg);
  M5Cardputer.Display.setBrightness(32);
  GPS_Serial.begin(115200, SERIAL_8N1, GPS_RX, GPS_TX);
  if (!SD.begin()) {
    M5Cardputer.Display.setTextColor(TFT_RED, TFT_BLACK);
    M5Cardputer.Display.setCursor(0, 0);
    M5Cardputer.Display.println("SD Card Fail!");
    while(1);
  }
  loadWaypoints();
}

void loop() {
  M5Cardputer.update();

  static bool mLast = false, sLast = false, dLast = false;
  bool mNow = M5Cardputer.Keyboard.isKeyPressed('m');
  bool sNow = M5Cardputer.Keyboard.isKeyPressed('s');
  bool dNow = M5Cardputer.Keyboard.isKeyPressed('d');

  static bool rLast = false;
  bool rNow = M5Cardputer.Keyboard.isKeyPressed('r');
  if (rNow && !rLast) {
    if (!isRecording) createGPX();
    else stopRecording();
    forceRedraw = true;
  }
  rLast = rNow;

  if (dNow && !dLast) {
    screenOn = !screenOn;
    M5Cardputer.Display.setBrightness(screenOn ? 32 : 0);
  }
  dLast = dNow;

  if (sNow && !sLast) {
    if (gps.location.isValid() && gps.date.isValid() && gps.time.isValid()) {
      char name[32];
      sprintf(name, "WP%02d_%02d%02d_%02d%02d", waypoints.size(),
              gps.date.day(), gps.date.month(),
              gps.time.hour(), gps.time.minute());
      saveWaypoint(name, gps.location.lat(), gps.location.lng());
    }
  }
  sLast = sNow;

  if (mNow && !mLast) {
    if (gps.location.isValid()) {
      centerLat = gps.location.lat();
      centerLon = gps.location.lng();
    }
    currentScreen = SCREEN_MAP;
    forceRedraw = true;
  }
  mLast = mNow;

  if (M5Cardputer.Keyboard.isKeyPressed('n')) {
    currentScreen = SCREEN_WAYPOINT_SELECT;
    selectedWaypoint = 0;
    forceRedraw = true;
  }
  if (M5Cardputer.Keyboard.isKeyPressed('\n')) {
    currentScreen = SCREEN_DATA;
    forceRedraw = true;
  }

  if (currentScreen == SCREEN_MAP) {
    if (M5Cardputer.Keyboard.isKeyPressed(130)) centerLat += 0.001 * zoom;
    if (M5Cardputer.Keyboard.isKeyPressed(131)) centerLat -= 0.001 * zoom;
    if (M5Cardputer.Keyboard.isKeyPressed(129)) centerLon -= 0.001 * zoom;
    if (M5Cardputer.Keyboard.isKeyPressed(128)) centerLon += 0.001 * zoom;
    if (forceRedraw) drawMap();
  }

  while (GPS_Serial.available()) {
    char c = GPS_Serial.read();
    gps.encode(c);
    static String nmeaLine = "";
    if (c == '\n') {
      if (nmeaLine.startsWith("$GPGGA")) {
        lastGGA = nmeaLine;
        parseGGA(lastGGA);
      }
      nmeaLine = "";
    } else if (c != '\r') {
      nmeaLine += c;
    }
  }

  static uint32_t lastWrite = 0;
  if (millis() - lastWrite > 3000 && isRecording) {
    writeGPXPoint();
    lastWrite = millis();
  }

  static uint32_t lastDisplay = 0;
  if ((millis() - lastDisplay > 500 || forceRedraw) && screenOn && currentScreen == SCREEN_DATA) {
    lastDisplay = millis();
    forceRedraw = false;

    M5Cardputer.Display.fillScreen(TFT_BLACK);
    M5Cardputer.Display.setTextSize(1);
    M5Cardputer.Display.setTextColor(TFT_CYAN, TFT_BLACK);
    M5Cardputer.Display.setCursor(gap, 2);
    M5Cardputer.Display.println("GPS Viewer");

    int y0 = 14;
    int x0 = gap;
    int x1 = gap * 2 + col0_w;

    char lat[20], alt[20], date[20], qual[20], crs[20];
    if (gps.location.isValid()) sprintf(lat, "%.6f", gps.location.lat());
    else sprintf(lat, "NoFix");
    drawField(x0, col0_w, y0, "Lat", lat);

    if (gps.altitude.isValid()) sprintf(alt, "%.2f", gps.altitude.meters());
    else sprintf(alt, "NoData");
    drawField(x0, col0_w, y0 + (field_h + field_gap), "Alt", alt);

    if (gps.date.isValid()) sprintf(date, "%02d/%02d/%02d", gps.date.day(), gps.date.month(), gps.date.year() % 100);
    else sprintf(date, "NoData");
    drawField(x0, col0_w, y0 + 2 * (field_h + field_gap), "Date", date);

    sprintf(qual, "%d", fixQuality);
    drawField(x0, col0_w, y0 + 3 * (field_h + field_gap), "Qual", qual);

    sprintf(crs, "%.1f", gps.course.deg());
    drawField(x0, col0_w, y0 + 4 * (field_h + field_gap), "Crs", crs);

    char lng[20], time[20], sats[20], spd[20], hdopStr[20];
    if (gps.location.isValid()) sprintf(lng, "%.6f", gps.location.lng());
    else sprintf(lng, "NoFix");
    drawField(x1, col1_w, y0, "Lng", lng);

    if (gps.time.isValid()) sprintf(time, "%02d:%02d:%02d", gps.time.hour(), gps.time.minute(), gps.time.second());
    else sprintf(time, "NoData");
    drawField(x1, col1_w, y0 + (field_h + field_gap), "Time", time);

    if (gps.satellites.isValid()) sprintf(sats, "%d", gps.satellites.value());
    else sprintf(sats, "NoData");
    drawField(x1, col1_w, y0 + 2 * (field_h + field_gap), "Sat", sats);

    sprintf(spd, "%.1f", gps.speed.kmph());
    drawField(x1, col1_w, y0 + 3 * (field_h + field_gap), "Spd", spd);

    sprintf(hdopStr, "%.1f", hdop);
    drawField(x1, col1_w, y0 + 4 * (field_h + field_gap), "HDOP", hdopStr);

    if (isRecording) {
      M5Cardputer.Display.setTextColor(TFT_RED, TFT_BLACK);
      M5Cardputer.Display.setCursor(screen_w - 60, 2);
      M5Cardputer.Display.print("[REC]");
    }

    int level = M5Cardputer.Power.getBatteryLevel();
    char battery[16];
    sprintf(battery, "%d%%", level);
    M5Cardputer.Display.setCursor(screen_w - 40, screen_h - 12);
    M5Cardputer.Display.setTextColor(TFT_GREEN, TFT_BLACK);
    M5Cardputer.Display.print(battery);

    if (millis() - lastMessageTime < 2000) {
      M5Cardputer.Display.setTextColor(TFT_YELLOW, TFT_BLACK);
      M5Cardputer.Display.setCursor(gap, screen_h - 12);
      M5Cardputer.Display.print(lastMessage);
    }
  }
  delay(10);
}
