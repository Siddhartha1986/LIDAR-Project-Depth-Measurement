#include <HardwareSerial.h>
#include <math.h>
#include <WiFi.h>
#include <HTTPClient.h>

HardwareSerial LidarSerial(2); // Use hardware serial port 2

#define POINT_PER_PACK 12
#define PKG_HEADER 0x54

// WiFi details
const char* ssid = "DNA-WLAN-2G-EADF";
const char* password = "95498247170";

// CRC Table for CRC8 calculation
static const uint8_t CrcTable[256] = {
  // Full CRC table was previously provided
    0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3, 0xae, 0xf2, 0xbf, 0x68, 0x25,
    0x8b, 0xc6, 0x11, 0x5c, 0xa9, 0xe4, 0x33, 0x7e, 0xd0, 0x9d, 0x4a, 0x07,
    0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8, 0xf5, 0x1f, 0x52, 0x85, 0xc8,
    0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77, 0x3a, 0x94, 0xd9, 0x0e, 0x43,
    0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55, 0x18, 0x44, 0x09, 0xde, 0x93,
    0x3d, 0x70, 0xa7, 0xea, 0x3e, 0x73, 0xa4, 0xe9, 0x47, 0x0a, 0xdd, 0x90,
    0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f, 0x62, 0x97, 0xda, 0x0d, 0x40,
    0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff, 0xb2, 0x1c, 0x51, 0x86, 0xcb,
    0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2, 0x8f, 0xd3, 0x9e, 0x49, 0x04,
    0xaa, 0xe7, 0x30, 0x7d, 0x88, 0xc5, 0x12, 0x5f, 0xf1, 0xbc, 0x6b, 0x26,
    0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99, 0xd4, 0x7c, 0x31, 0xe6, 0xab,
    0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14, 0x59, 0xf7, 0xba, 0x6d, 0x20,
    0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36, 0x7b, 0x27, 0x6a, 0xbd, 0xf0,
    0x5e, 0x13, 0xc4, 0x89, 0x63, 0x2e, 0xf9, 0xb4, 0x1a, 0x57, 0x80, 0xcd,
    0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72, 0x3f, 0xca, 0x87, 0x50, 0x1d,
    0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2, 0xef, 0x41, 0x0c, 0xdb, 0x96,
    0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1, 0xec, 0xb0, 0xfd, 0x2a, 0x67,
    0xc9, 0x84, 0x53, 0x1e, 0xeb, 0xa6, 0x71, 0x3c, 0x92, 0xdf, 0x08, 0x45,
    0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa, 0xb7, 0x5d, 0x10, 0xc7, 0x8a,
    0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35, 0x78, 0xd6, 0x9b, 0x4c, 0x01,
    0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17, 0x5a, 0x06, 0x4b, 0x9c, 0xd1,
    0x7f, 0x32, 0xe5, 0xa8
};

typedef struct __attribute__((packed)) {
  uint16_t distance;
  uint8_t intensity;
} LidarPointStructType;

typedef struct __attribute__((packed)) {
  uint8_t header;
  uint8_t ver_len;
  uint16_t speed;
  uint16_t start_angle;
  LidarPointStructType point[POINT_PER_PACK];
  uint16_t end_angle;
  uint16_t timestamp;
  uint8_t crc8;
} LiDARMeasureDataType;

uint8_t CalCRC8(const uint8_t *data, uint8_t len) {
  uint8_t crc = 0;
  for (uint16_t i = 0; i < len; i++) {
    crc = CrcTable[(crc ^ *data++) & 0xff];
  }
  return crc;
}

bool parseDataPacket(const uint8_t *buffer, size_t len, LiDARMeasureDataType &measureData) {
  if (len < sizeof(LiDARMeasureDataType)) {
    return false; // Not enough data
  }
  if (buffer[0] != PKG_HEADER || buffer[1] != 0x2C) {
    return false; // Incorrect header or VerLen, invalid packet
  }
  uint8_t crc = CalCRC8(buffer, len - 1);
  if (crc == buffer[len - 1]) {
    memcpy(&measureData, buffer, sizeof(LiDARMeasureDataType));
    return true; // CRC check passed, packet parsed successfully
  }
  return false; // CRC check failed
}


// Global Variables
bool dataCollectionStarted = false;
bool collectionComplete = false; // Flag to indicate the entire process is complete
unsigned long dataStartTime = 0;
const long collectionDuration = 2000; // 2 sec

static uint32_t distanceSum[361] = {0};
static uint16_t distanceCount[361] = {0};
static float xCoordinates[361] = {0};
static float yCoordinates[361] = {0};
static float depths[361] = {0};

void setup() {
    // Start the Serial communication to output the status
    Serial.begin(115200);
    delay(100);   // Give time for the serial monitor to initialize
    
    // Attempt to connect to WiFi
    WiFi.begin(ssid, password);
    Serial.println("Attempting to connect to WiFi...");
    int connectionAttempts = 0;
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.print(".");
        connectionAttempts++;
        if (connectionAttempts >= 10) {  // Max attempts to connect
              Serial.println("Failed to connect to WiFi, resetting...");
              ESP.restart(); // Resets the ESP32
            //Serial.println("Failed to connect to WiFi. Please check your credentials or WiFi signal.");
            //while(true) {
                //delay(1000);  // Infinite loop to stop further execution
            //}
        }
    }
    Serial.println("\nConnected to WiFi");
    Serial.print("IP Address Assigned to the ESP32 by the network: ");
    Serial.println(WiFi.localIP()); // Print the local IP address
    
    // If connected, then start the Lidar
    LidarSerial.begin(230400, SERIAL_8N1, 16, -1); // Initialize LidarSerial with specified pins
}




void loop() {
  if (collectionComplete) {
    // If collection and processing are complete, do nothing further.
    return;
  }
  
  static uint8_t buffer[1024];
  static size_t bufferIndex = 0;
  LiDARMeasureDataType measureData;
  static unsigned long lastCheckTime = 0;
  const unsigned long checkInterval = 10000;  // Check every 10 seconds

  // Periodic WiFi check and auto-reconnect logic
  if (millis() - lastCheckTime >= checkInterval) {
    lastCheckTime = millis();
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi connection lost. Trying to reconnect...");
      WiFi.disconnect();
      WiFi.reconnect();
      return;  // Skip further processing until WiFi is reconnected
    } else {
      Serial.println("WiFi is still connected.");
    }
  }

  
  // processing the data received from the LIDAR based on the datasheet and the github link provided in the product
  
  while (LidarSerial.available()) {
    if (!dataCollectionStarted) {
      dataStartTime = millis(); // Start timing at the first byte received
      dataCollectionStarted = true;
    }

    if (dataCollectionStarted && millis() - dataStartTime < collectionDuration) {
      uint8_t byte = LidarSerial.read();
      buffer[bufferIndex++] = byte;

      if (bufferIndex >= sizeof(buffer)) bufferIndex = 0;

      if (bufferIndex >= sizeof(LiDARMeasureDataType) && parseDataPacket(buffer, bufferIndex, measureData)) {
        float startAngle = measureData.start_angle / 100.0f;
        float endAngle = measureData.end_angle / 100.0f;
        float angleStep = (endAngle - startAngle) / (POINT_PER_PACK - 1);

        for (int i = 0; i < POINT_PER_PACK; i++) {
          float angle = startAngle + angleStep * i;
          if (angle >= 360.0f) angle -= 360.0f;
          int roundedAngle = static_cast<int>(round(angle));
          uint16_t distance = measureData.point[i].distance;

          distanceSum[roundedAngle] += distance;
          distanceCount[roundedAngle]++;
        }
        bufferIndex = 0; // Reset buffer index after processing a packet
      }
    } else {
      // Break out of the while loop if 3 minutes have elapsed
      break;
    }
  }

 // Process data after collection
  if (dataCollectionStarted && millis() - dataStartTime >= collectionDuration) {
    String jsonData = "{ \"angles\": [";
    bool isFirst = true;
    float minY = 1e6; // Initialize with a very large value for minY

    
    for (int i = 0; i <= 360; i++) {
      if (distanceCount[i] > 0) {
        float avgDistance = static_cast<float>(distanceSum[i]) / distanceCount[i];
        if (!isFirst) jsonData += ", ";
        jsonData += "{\"angle\": " + String(i) + ", \"distance\": " + String(avgDistance) + "}";
        isFirst = false;
        
        Serial.print(i); Serial.print(", "); Serial.println(avgDistance, 2);
        float angleRadians = i * M_PI / 180.0; // Convert angle to radians
        xCoordinates[i] = avgDistance * sin(angleRadians);
        yCoordinates[i] = avgDistance * (-cos(angleRadians));
        if (yCoordinates[i] < minY) {
          minY = yCoordinates[i]; // Update maxY to find the ground level
        }
        
      }
    }

    

    

    // Calculate depth for each point relative to the ground level
    for (int i = 0; i <= 360; i++) {
      if (distanceCount[i] > 0) {
        depths[i] = yCoordinates[i] - minY;
        Serial.print("Angle: "); Serial.print(i);
        Serial.print(", Depth: "); Serial.println(depths[i], 2);
      }
    }

    
    
    // Calculate average depths for the specified angle ranges
    float sumDepth75to85 = 0;
    int count75to85 = 0;
    float sumDepth275to285 = 0;
    int count275to285 = 0;

    for (int i = 75; i <= 85; i++) {
      if (distanceCount[i] > 0) {
        sumDepth75to85 += depths[i];
        count75to85++;
      }
    }

    for (int i = 275; i <= 285; i++) {
      if (distanceCount[i] > 0) {
        sumDepth275to285 += depths[i];
        count275to285++;
      }
    }

    float averageDepth75to85 = (count75to85 > 0) ? (sumDepth75to85 / count75to85) : 0;
    float averageDepth275to285 = (count275to285 > 0) ? (sumDepth275to285 / count275to285) : 0;

    // Compare the two average depths and print the smaller one
    float finalDepth = (averageDepth75to85 < averageDepth275to285) ? averageDepth75to85 : averageDepth275to285;

    jsonData += "], \"finalDepth\": " + String(finalDepth) + "}";

    Serial.println(jsonData);  // Debug: Print JSON data to serial
    sendData(jsonData);        // Send data to the server

    Serial.print("Final depth value for the ditch is: ");
    Serial.print(finalDepth, 2);
    Serial.println(" mm");

    if (finalDepth <= 0) {  // Check if the depth is above a certain minimal threshold
      Serial.println("Invalid depth detected. Resetting device...");
      ESP.restart();  // Reset the device to start a new measurement cycle if depth is invalid
    } else {
      sendData(jsonData);  // Only send data if the depth is valid
      markCollectionComplete();  // Function to reset and mark the process as complete
    }

    
  }
}



// Function to reset and mark the process as complete

void markCollectionComplete() {
  // Reset all counters and states for the next possible run
  for (int i = 0; i <= 360; i++) {
    distanceSum[i] = 0;
    distanceCount[i] = 0;
    xCoordinates[i] = 0;
    yCoordinates[i] = 0;
    depths[i] = 0;
  }
  Serial.println("Data collection and processing complete.");
  dataCollectionStarted = false;  // Reset data collection flag
  collectionComplete = true;  // Set flag to prevent further processing
}





// A function to send data to the server

void sendData(String jsonData) {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin("http://192.168.1.114:3000/receive_data");
    http.addHeader("Content-Type", "application/json");
    int httpResponseCode = http.POST(jsonData);

    if (httpResponseCode > 0) {
      String response = http.getString();
      Serial.println("HTTP Response code: " + String(httpResponseCode));
      Serial.println("Response: " + response);
    } else {
      Serial.println("Error on sending POST: " + String(httpResponseCode));
    }

    http.end();
  } else {
    Serial.println("WiFi Disconnected");
  }
}
