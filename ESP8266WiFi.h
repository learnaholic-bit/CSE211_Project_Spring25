#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

const char* ssid = "OPPO F11";
const char* password = "m1042004";

ESP8266WebServer server(80);

// Global location variables
float latitude = 0.0;
float longitude = 0.0;
String location_description = "Unknown";  // String for location description
// Path and coordinates array
float prev_lat = 0.0;
float prev_lon = 0.0;

// HTML page with Leaflet map
String htmlPage() {
  String html = R"rawliteral(
    <!DOCTYPE html><html>
    <head>
      <title>Live GPS Map</title>
      <meta name="viewport" content="width=device-width, initial-scale=1.0">
      <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"/>
      <style> #map { height: 90vh; width: 100%; } </style>
    </head>
    <body>
      <h3 style="text-align:center;">Live Location</h3>
      <div id="map"></div>
      <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
      <script>
        var map = L.map('map').setView([LAT, LON], 15);
        var marker = L.marker([LAT, LON]).addTo(map).bindPopup("Location: LOC_DESC").openPopup();
        var latlngs = [[LAT, LON]];  // Initial position
        var polyline = L.polyline(latlngs, {color: 'blue'}).addTo(map); // Path line

        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
          attribution: '&copy; OpenStreetMap contributors'
        }).addTo(map);

        setInterval(() => {
          fetch('/coords')
            .then(res => res.json())
            .then(data => {
              // Update marker position
              marker.setLatLng([data.lat, data.lon]);
              marker.setPopupContent("Location: " + data.desc);  // Show location description
              map.setView([data.lat, data.lon]);

              // Add new coordinate to the path (latlngs array) and update the polyline
              latlngs.push([data.lat, data.lon]);
              polyline.setLatLngs(latlngs);
            });
        }, 5000);
      </script>
    </body></html>
  )rawliteral";

  html.replace("LAT", String(latitude, 6));
  html.replace("LON", String(longitude, 6));
  html.replace("LOC_DESC", location_description);  // Set the location description

  return html;
}

// Handle root page
void handleRoot() {
  server.send(200, "text/html", htmlPage());
}

// Return current coordinates and description as JSON
void handleCoords() {
  String json = "{";
  json += "\"lat\":" + String(latitude, 6) + ",";
  json += "\"lon\":" + String(longitude, 6) + ",";
  json += "\"desc\":\"" + location_description + "\"";  // Add the location description
  json += "}";
  server.send(200, "application/json", json);
}

/**
 * @brief Sets up the ESP8266 as an access point and starts the web server.
 *
 * @details This function initializes the serial communication, connects to the
 *          specified WiFi network, and sets up the web server to serve the
 *          HTML page with the Leaflet map.
 */
void setup() {
  Serial.begin(9600); // Or match your GPS/controller baud rate

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500); Serial.print(".");
  }
  Serial.println("Connected! IP: " + WiFi.localIP().toString());

  server.on("/", handleRoot);
  server.on("/coords", handleCoords);
  server.begin();
}

// Example: Receive coordinates from another controller via Serial
void loop() {
  server.handleClient();

  if (Serial.available()) {
    String data = Serial.readStringUntil('\n');
    // Expected format: lat,lon,location_description (e.g., 25.123456,55.123456,Hall A)
    int firstComma = data.indexOf(',');
    int secondComma = data.indexOf(',', firstComma + 1);

    if (firstComma > 0 && secondComma > firstComma) {
      latitude = data.substring(0, firstComma).toFloat();
      longitude = data.substring(firstComma + 1, secondComma).toFloat();
      location_description = data.substring(secondComma + 1);  // Get the location description
    }
  }
}