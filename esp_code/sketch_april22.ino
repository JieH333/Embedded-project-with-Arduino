#include <ESP8266WiFi.h>        // Library for Wi-Fi functionality
#include <ESP8266WebServer.h>   // Library to create and manage a web server
#include <FS.h>                 // Library for working with file systems (SPIFFS)

const char* ssid = "Titenet-IoT";
const char* password = "7kDtaphg";
String lidarData = "0 cm";
String currentDegree = "0";
String colorData = "";
String encoderData = "0 mm/pulse";
const char* http_username = "admin";
const char* http_password = "admin";

ESP8266WebServer server(80);    // Create an instance of the WebServer on port 80 (default HTTP port)

void setup() {
  Serial.begin(9600);

  // Initialize the file system (SPIFFS) on the ESP8266
  if (!SPIFFS.begin()) {        // Initialize and check success on SPIFFS
    Serial.println("Error while mounting SPIFFS");
    return;
  }

  // Connect to the Wi-Fi network using the provided SSID and password
  Serial.print("\nConnecting to " + (String)ssid);
  WiFi.begin(ssid, password);               // Begin connecting to Wi-Fi
  while (WiFi.status() != WL_CONNECTED) {   // Wait in a loop until the connection is successful
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nIP address: " + WiFi.localIP().toString()); // Print the IP address of the ESP8266 when connected


  // Set up the web pages (URLS) and files that the server will show/use when someone visits the site
  server.on("/", []() {
    if (!server.authenticate(http_username, http_password)) {
      server.requestAuthentication(); // Prompt for login
      return;
    }
    server.send(200, "text/html", SPIFFS.open("/index.html", "r").readString());
  });

  server.on("/logout", HTTP_GET, []() {
    server.send(401, "text/plain", "Unauthorized - You have been logged out");
  });

  server.on("/logged-out", HTTP_GET, []() {
    server.send(200, "text/html", SPIFFS.open("/logout.html", "r").readString());
  });

  server.serveStatic("/style.css", SPIFFS, "/style.css");     // Serve the CSS file for styling
  server.serveStatic("/script.js", SPIFFS, "/script.js");     // Serve the JavaScript file
  server.serveStatic("/favicon.ico", SPIFFS, "/favicon.png"); // Serve a favicon (small icon) for the website

  // Define custom actions/function calls for specific URLs (ie when a button press from webpage requests a particular URL)
  server.on("/forwards500", [](){ handleAuthenticatedRequest([]() { handleMove(500); }); });      // When requesting URL "/forwards5", call the handleMove function with parameter 5
  server.on("/lidar", [](){ handleAuthenticatedRequest(handleLidar); });                   // When requesting URL "/lidar", call the handleLidar function.
  server.on("/compassData", [](){ handleAuthenticatedRequest(handleCurrentDegree); });
  server.on("/color", [](){ handleAuthenticatedRequest(handleColor); });
  server.on("/forwards20", [](){ handleAuthenticatedRequest([]() { handleMove(20); }); });
  server.on("/encoder", [](){ handleAuthenticatedRequest(handleEncoder); });

  //  If someone tries to access a URL that does not exist (e.g. due to a typo), call the handleNotFound function
  server.onNotFound(handleNotFound);

  // Start the web server
  server.begin();
}

void loop() {
  server.handleClient();

  // Check if data is available from the Arduino Mega
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    if (data.startsWith("LIDAR:")) {
      lidarData = data.substring(6);
    }
    else if (data.startsWith("Degree:")) {
      currentDegree = data.substring(7);
    }
    else if (data.startsWith("R:")) {
      colorData = data;
    }
    else if (data.startsWith("Encoder:")) {
      encoderData = data.substring(8);
    }
  }
}

// Middleware function to protect routes
void handleAuthenticatedRequest(std::function<void()> handler) {
  if (!server.authenticate(http_username, http_password)) {
    server.requestAuthentication();
    return;
  }
  handler();
}

// This function is called when a non-existing URL is accessed (404 error)
void handleNotFound() {
  server.send(404, "text/plain", "404: Not Found"); // Send a 404 response with a plain text message
}

// This function handles movement commands like "/forwards5" or "/backwards20"
void handleMove(int distance) {
  Serial.println("Move:" + String(distance));       // Print the movement distance to the serial monitor for Arduino Mega
  server.send(200);                                 // Send a 200 OK response to the client (browser)
}

// This function handles the compass command (like "/compass?value=30")
void handleCompass() {
  if (server.hasArg("value")) {                     // Check if there is a "value" argument in the request URL
    String valueString = server.arg("value");       // Get the "value" argument from the URL
    Serial.println("Turn:" + valueString);          // Print the compass value to the serial monitor
  }
  server.send(200);
}

// This function handles the lidar command (like "/lidar")
void handleLidar() {
  server.send(200, "text/plain", lidarData); // Send the latest Lidar value with a 200 OK status.
}

// This function handles the lidar command (like "/compassData")
void handleCurrentDegree() {
  server.send(200, "text/plain", currentDegree); // Send the latest Degree value with a 200 OK status.
}

// This function handles the color command (like "/color")
void handleColor() {
  server.send(200, "text/plain", colorData); // Send the latest Color value with a 200 OK status.
}

// This function handles the encoder command (like "/encoder")
void handleEncoder() {
  server.send(200, "text/plain", encoderData); // Send the latest Encoder value with a 200 OK status.
}
