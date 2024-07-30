
/* 
- **Include Libraries and Define Constants:**
  - Includes libraries such as `HardwareSerial`, `math.h`, `WiFi`, `HTTPClient`.
  - Defines constants for debug mode, points per packet, and package header.

- **WiFi Configuration:**
  - Sets WiFi SSID and password for connection.

- **CRC8 Calculation:**
  - Defines CRC table and function for CRC8 calculation.

- **Lidar Data Structures:**
  - Defines structures for LiDAR point and measurement data.

- **Data Packet Parsing:**
  - Function to parse LiDAR data packets and verify using CRC8.

- **Circle Fitting Function:**
  - Function to fit a circle to given points and calculate residuals.

- **Global Variables:**
  - Variables for data collection status, timing, and measurement storage.

- **Setup Function:**
  - Initializes serial communication, connects to WiFi, and starts LiDAR.

- **Main Loop Function:**
  - Checks WiFi connection periodically.
  - Collects and processes LiDAR data for a fixed duration.
  - Calculates angles, distances, coordinates, depths, and thresholds.
  - Defines angle ranges and collects depths.
  - Filters points and finds the lowest point.
  - Adjusts depths relative to the new lowest point.
  - Collects bump points and fits a circle.
  - Checks fit quality and determines detection status.
  - Appends data to JSON and sends to the server.
  - Resets device if depth is invalid.

- **Function to Mark Collection Complete:**
  - Resets counters and states for the next run.

- **Function to Send Data to Server:**
  - Sends JSON data to the server with retries and timeout handling.
*/





#include <HardwareSerial.h>
#include <math.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <float.h>
#include <vector>
#include <algorithm>
#include <numeric>

#define DEBUG_MODE 1 // Set to 1 for debugging, 0 for production

HardwareSerial LidarSerial(2); // Use hardware serial port 2

#define POINT_PER_PACK 12
#define PKG_HEADER 0x54

// WiFi details
const char* ssid = "DNA-WLAN-2G-EADF";
const char* password = "95498247170";

// CRC Table for CRC8 calculation
static const uint8_t CrcTable[256] = {
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

// Function to fit a circle to given points
void fit_circle(const float* x, const float* y, int n, float &xc, float &yc, float &R, float &max_residual, float &mean_residual) {
    auto calc_R = [&x, &y, n](float xc, float yc) {
        float* R = new float[n];
        for (int i = 0; i < n; ++i) {
            R[i] = sqrt((x[i] - xc) * (x[i] - xc) + (y[i] - yc) * (y[i] - yc));
        }
        return R;
    };

    auto f_2 = [&calc_R, n](float* c, float* residuals) {
        float* Ri = calc_R(c[0], c[1]);
        float Ri_mean = 0;
        for (int i = 0; i < n; ++i) {
            Ri_mean += Ri[i];
        }
        Ri_mean /= n;
        for (int i = 0; i < n; ++i) {
            residuals[i] = Ri[i] - Ri_mean;
        }
        delete[] Ri;
    };

    float center_estimate[2] = {0, 0};
    for (int i = 0; i < n; ++i) {
        center_estimate[0] += x[i];
        center_estimate[1] += y[i];
    }
    center_estimate[0] /= n;
    center_estimate[1] /= n;

    float c[2] = {center_estimate[0], center_estimate[1]};
    float residuals[n];
    f_2(c, residuals);
    xc = c[0];
    yc = c[1];
    R = 0;
    for (int i = 0; i < n; ++i) {
        R += sqrt((x[i] - xc) * (x[i] - xc) + (y[i] - yc) * (y[i] - yc));
    }
    R /= n;

    // Calculate the residuals
    float abs_residuals[n];
    for (int i = 0; i < n; i++) {
        abs_residuals[i] = fabs(sqrt((x[i] - xc) * (x[i] - xc) + (y[i] - yc) * (y[i] - yc)) - R);
    }

    // Calculate fit quality metrics
    max_residual = *std::max_element(abs_residuals, abs_residuals + n);
    mean_residual = std::accumulate(abs_residuals, abs_residuals + n, 0.0f) / n;
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

// Tilt angle in degrees can be either positive or negative
const float tiltAngle = 0.0;

void setup() {
    #ifdef DEBUG_MODE  
        // Start the Serial communication to output the status
        Serial.begin(115200);
        delay(100);   // Give time for the serial monitor to initialize
    #endif
    
    // Attempt to connect to WiFi
    WiFi.begin(ssid, password);
    #ifdef DEBUG_MODE
        Serial.println("Attempting to connect to WiFi...");
    #endif

    int connectionAttempts = 0;
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        
        #ifdef DEBUG_MODE  
            Serial.print(".");
        #endif

        connectionAttempts++;
        if (connectionAttempts >= 10) {  // Max attempts to connect
            
            #ifdef DEBUG_MODE  
                Serial.println("Failed to connect to WiFi, resetting...");
            #endif

            ESP.restart(); // Resets the ESP32
        }
    }

    #ifdef DEBUG_MODE  
        Serial.println("\nConnected to WiFi");
        Serial.print("IP Address Assigned to the ESP32 by the network: ");
        Serial.println(WiFi.localIP()); // Print the local IP address
    #endif
    
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
            #ifdef DEBUG_MODE  
                Serial.println("WiFi connection lost. Trying to reconnect...");
            #endif
            WiFi.disconnect();
            WiFi.reconnect();
            return;  // Skip further processing until WiFi is reconnected
        } else {
            #ifdef DEBUG_MODE 
                Serial.println("WiFi is still connected.");
            #endif
        }
    }

    // Processing the data received from the LIDAR based on the datasheet and the GitHub link provided in the product
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
            // Break out of the while loop if collection time mentioned above have elapsed
            break;
        }
    }

    // Process data after collection
    if (dataCollectionStarted && millis() - dataStartTime >= collectionDuration) {
        String jsonData = "{ \"angles\": [";
        bool isFirst = true;
        float maxY = 0; // Initialize with a very small value for maxY

        // Calculate the rotation matrix components
        float rotationAngle = tiltAngle * M_PI / 180.0;
        float cosAngle = cos(rotationAngle);
        float sinAngle = sin(rotationAngle);

        // Define angle ranges with tilt angle adjustment
        float startAngleRange1 = fmod(290 + tiltAngle, 360);
        if (startAngleRange1 < 0) startAngleRange1 += 360;
        
        float endAngleRange1 = fmod(359 + tiltAngle, 360);
        if (endAngleRange1 < 0) endAngleRange1 += 360;
        
        float startAngleRange2 = fmod(0 + tiltAngle, 360);
        if (startAngleRange2 < 0) startAngleRange2 += 360;
        
        float endAngleRange2 = fmod(70 + tiltAngle, 360);
        if (endAngleRange2 < 0) endAngleRange2 += 360;

        // Collect filtered angles and distances based on the angle range and non-zero values
        std::vector<int> filteredAngles;
        std::vector<float> filteredDistances;

        for (int i = 0; i <= 360; i++) {
            if (distanceCount[i] > 0) {
                float avgDistance = static_cast<float>(distanceSum[i]) / distanceCount[i];
                if ((startAngleRange1 <= endAngleRange1 && i >= startAngleRange1 && i <= endAngleRange1) ||
                    (startAngleRange1 > endAngleRange1 && (i >= startAngleRange1 || i <= endAngleRange1)) ||
                    (startAngleRange2 <= endAngleRange2 && i >= startAngleRange2 && i <= endAngleRange2) ||
                    (startAngleRange2 > endAngleRange2 && (i >= startAngleRange2 || i <= endAngleRange2))) {
                    filteredAngles.push_back(i);
                    filteredDistances.push_back(avgDistance);
                }
            }
        }

        for (size_t i = 0; i < filteredAngles.size(); i++) {
            int angle = filteredAngles[i];
            float distance = filteredDistances[i];
            if (!isFirst) jsonData += ", ";
            jsonData += "{\"angle\": " + String(angle) + ", \"distance\": " + String(distance) + "}";
            isFirst = false;
                
            #ifdef DEBUG_MODE
                Serial.print(angle); Serial.print(", "); Serial.println(distance, 2);
            #endif

            float angleRadians = angle * M_PI / 180.0; // Convert angle to radians
            float x = distance * sin(angleRadians);
            float y = distance * cos(angleRadians);

            // Rotate the coordinates
            xCoordinates[angle] = x * cosAngle - y * sinAngle;
            yCoordinates[angle] = x * sinAngle + y * cosAngle;

            if (yCoordinates[angle] > maxY) {
                maxY = yCoordinates[angle]; // Update maxY to find the ground level
            }
        }

        // Calculate depth for each point relative to the ground level
        for (size_t i = 0; i < filteredAngles.size(); i++) {
            int angle = filteredAngles[i];
            depths[angle] = maxY - yCoordinates[angle];

            #ifdef DEBUG_MODE
                Serial.print("Angle: "); Serial.print(angle);
                Serial.print(", Depth: "); Serial.println(depths[angle], 2);
            #endif
        }

        // Collect depths for specified angle ranges
        std::vector<float> depthsRange1;
        std::vector<float> depthsRange2;

        for (size_t i = 0; i < filteredAngles.size(); i++) {
            int angle = filteredAngles[i];
            float depth = depths[angle];

            if ((startAngleRange1 <= endAngleRange1 && angle >= startAngleRange1 && angle <= endAngleRange1) ||
                (startAngleRange1 > endAngleRange1 && (angle >= startAngleRange1 || angle <= endAngleRange1))) {
                depthsRange1.push_back(depth);
            }

            if ((startAngleRange2 <= endAngleRange2 && angle >= startAngleRange2 && angle <= endAngleRange2) ||
                (startAngleRange2 > endAngleRange2 && (angle >= startAngleRange2 || angle <= endAngleRange2))) {
                depthsRange2.push_back(depth);
            }
        }

        // Sort the depths and take the top 10 highest values
        std::sort(depthsRange1.begin(), depthsRange1.end(), std::greater<float>());
        std::sort(depthsRange2.begin(), depthsRange2.end(), std::greater<float>());
        
        int topCountRange1 = std::min(10, static_cast<int>(depthsRange1.size()));
        int topCountRange2 = std::min(10, static_cast<int>(depthsRange2.size()));
        
        float minTop10DepthRange1 = (topCountRange1 > 0) ? depthsRange1[topCountRange1 - 1] : FLT_MAX;
        float minTop10DepthRange2 = (topCountRange2 > 0) ? depthsRange2[topCountRange2 - 1] : FLT_MAX;
        
        // Determine the final depth value
        float finalDepth = std::min(minTop10DepthRange1, minTop10DepthRange2);

        // Calculate the threshold as 5% of the final depth value
        float thresholdValue = 0.05 * finalDepth;

        // Filter points above the threshold value
        std::vector<int> thresholdIndices;
        for (size_t i = 0; i < filteredAngles.size(); i++) {
            int angle = filteredAngles[i];
            if (yCoordinates[angle] >= maxY - thresholdValue) {
                thresholdIndices.push_back(angle);
            }
        }

        // Check if there are points above the threshold
        if (thresholdIndices.empty()) {
            #ifdef DEBUG_MODE
            Serial.println("No points above the threshold value.");
            #endif

            // Set default values and send data
            float newFinalDepth = finalDepth;
            String detection_status = "No Pipe Detected";
            float R = 0;  // Default value for radius

            jsonData += "], \"tiltAngle\": " + String(tiltAngle) +
                        ", \"finalDepth\": " + String(finalDepth) +
                        ", \"newFinalDepth\": " + String(newFinalDepth) +
                        ", \"detectionStatus\": \"" + detection_status + "\"," +
                        "\"xc\": 0, \"yc\": 0, \"R\": " + String(R) +
                        ", \"max_residual\": 0, \"mean_residual\": 0, " +
                        "\"radius_limit\": 0, \"is_good_fit\": false, " +
                        "\"diameter_limit\": 0, " +
                        "\"pipeDiameter\": 0" +
                        "}";
        } else {
            // Determine the angle range for the points above the threshold
            std::vector<int> thresholdAngles = thresholdIndices;
            std::sort(thresholdAngles.begin(), thresholdAngles.end());

            float angleRangeStart, angleRangeEnd;
            if (!thresholdAngles.empty()) {
                if (thresholdAngles.back() - thresholdAngles.front() <= 180) {
                    // No wrap-around needed
                    angleRangeStart = thresholdAngles.front();
                    angleRangeEnd = thresholdAngles.back();
                } else {
                    // Handle wrap-around case
                    for (size_t i = 1; i < thresholdAngles.size(); i++) {
                        if (thresholdAngles[i] - thresholdAngles[i - 1] > 180) {
                            angleRangeStart = thresholdAngles[i];
                            angleRangeEnd = thresholdAngles[i - 1] + 360;
                            break;
                        }
                    }
                }

                // Adjust angle range for wrap-around
                if (angleRangeEnd < angleRangeStart) {
                    angleRangeEnd += 360;
                }

                // Collect all points within this new angle range
                std::vector<std::tuple<float, float, float, int>> filteredPoints;
                for (size_t i = 0; i < filteredAngles.size(); i++) {
                    int angle = filteredAngles[i];
                    if (angleRangeStart <= angle && angle <= angleRangeEnd) {
                        filteredPoints.emplace_back(xCoordinates[angle], yCoordinates[angle], depths[angle], angle);
                    } else if (angleRangeStart <= angle + 360 && angle + 360 <= angleRangeEnd) {
                        filteredPoints.emplace_back(xCoordinates[angle], yCoordinates[angle], depths[angle], angle);
                    } else if (angleRangeStart <= angle - 360 && angle - 360 <= angleRangeEnd) {
                        filteredPoints.emplace_back(xCoordinates[angle], yCoordinates[angle], depths[angle], angle);
                    }
                }

                // Ensure the wrap-around angle logic is correct
                std::vector<int> adjustedAngles;
                for (const auto& p : filteredPoints) {
                    int a = std::get<3>(p);
                    if (angleRangeStart <= a && a <= angleRangeEnd) {
                        adjustedAngles.push_back(a);
                    } else if (angleRangeStart <= a + 360 && a + 360 <= angleRangeEnd) {
                        adjustedAngles.push_back(a + 360);
                    } else if (angleRangeStart <= a - 360 && a - 360 <= angleRangeEnd) {
                        adjustedAngles.push_back(a - 360);
                    }
                }

                // Collect points with adjusted angles
                std::vector<std::tuple<float, float, float, int>> finalFilteredPoints;
                for (size_t i = 0; i < filteredPoints.size(); i++) {
                    if (angleRangeStart <= adjustedAngles[i] && adjustedAngles[i] <= angleRangeEnd) {
                        finalFilteredPoints.push_back(filteredPoints[i]);
                    }
                }

                // Sort the filtered points by their y-coordinate in ascending order
                std::sort(finalFilteredPoints.begin(), finalFilteredPoints.end(),
                          [](const std::tuple<float, float, float, int>& a, const std::tuple<float, float, float, int>& b) {
                              return std::get<1>(a) < std::get<1>(b);
                          });

                // Initialize lowest_point as None
                auto lowestPointIter = finalFilteredPoints.end();

                // Find the lowest y-coordinate point that is also less than the final depth
                for (auto it = finalFilteredPoints.begin(); it != finalFilteredPoints.end(); ++it) {
                    if (std::get<2>(*it) < finalDepth) {
                        lowestPointIter = it;
                        break;
                    }
                }

                // Ensure we found a valid point
                if (lowestPointIter != finalFilteredPoints.end()) {
                    auto lowestPoint = *lowestPointIter;
                    int lowestAngleIndex = std::get<3>(lowestPoint);

                    // Adjust depths relative to the new lowest point
                    float lowestY = std::get<1>(lowestPoint);
                    float newDepths[361];
                    for (size_t i = 0; i < filteredAngles.size(); i++) {
                        int angle = filteredAngles[i];
                        newDepths[angle] = lowestY - yCoordinates[angle];
                    }

                    // Calculate the new R limit based on your logic
                    float pipe = maxY - lowestY;
                    float diameter_limit = pipe + 0.005 * pipe;  // Adding 0.5% of the pipe depth to account for sinking into the mud
                    float radius_limit = diameter_limit / 2;

                    // Define the maximum allowable Euclidean distance gap to consider points as continuous
                    float MAX_EUCLIDEAN_DISTANCE_GAP = sqrt(2) * radius_limit;
                    
                    // Ensure proper index wrapping around the list
                    int lowest_angle_index = std::get<3>(lowestPoint);
                    
                    // Initialize variables for bump points collection
                    std::vector<int> bump_indices;
                    bump_indices.push_back(lowest_angle_index);
                    
                    int left_collected = 0;
                    int right_collected = 0;
                    int i = 1;
                    
                    // Function to calculate Euclidean distance between two points
                    auto euclidean_distance = [](float x1, float y1, float x2, float y2) {
                        return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
                    };
                    
                    // Collect points to the left and right of the lowest point
                    while (left_collected < 5 || right_collected < 5) {
                        if (left_collected < 5) {
                            int left_index = (lowest_angle_index - i + 361) % 361;
                            if (euclidean_distance(xCoordinates[lowest_angle_index], yCoordinates[lowest_angle_index],
                                                  xCoordinates[left_index], yCoordinates[left_index]) <= MAX_EUCLIDEAN_DISTANCE_GAP) {
                                bump_indices.push_back(left_index);
                                left_collected++;
                            } else {
                                left_collected = 5;  // Stop collecting points on this side if a gap is detected
                            }
                        }
                        if (right_collected < 5) {
                            int right_index = (lowest_angle_index + i) % 361;
                            if (euclidean_distance(xCoordinates[lowest_angle_index], yCoordinates[lowest_angle_index],
                                                  xCoordinates[right_index], yCoordinates[right_index]) <= MAX_EUCLIDEAN_DISTANCE_GAP) {
                                bump_indices.push_back(right_index);
                                right_collected++;
                            } else {
                                right_collected = 5;  // Stop collecting points on this side if a gap is detected
                            }
                        }
                        i++;
                        if (i > 361) {  // Safety check to prevent infinite loop in case of too many missing points
                            break;
                        }
                    }
                    
                    // Prepare bump points for circle fitting
                    std::vector<std::tuple<float, float, float, float, int>> bumpPoints;
                    for (int idx : bump_indices) {
                        bumpPoints.emplace_back(xCoordinates[idx], yCoordinates[idx], depths[idx], newDepths[idx], idx);
                    }
                    
                    // Fit a circle to these points
                    int n = bumpPoints.size();
                    float x_bump[n], y_bump[n];
                    for (int i = 0; i < n; i++) {
                        x_bump[i] = std::get<0>(bumpPoints[i]);
                        y_bump[i] = std::get<1>(bumpPoints[i]);
                    }
                    float xc, yc, R, max_residual, mean_residual;
                    fit_circle(x_bump, y_bump, n, xc, yc, R, max_residual, mean_residual);
                    
                    // Print fit quality metrics for debugging
                    #ifdef DEBUG_MODE
                    Serial.print("Max Residual: "); Serial.println(max_residual);
                    Serial.print("Mean Residual: "); Serial.println(mean_residual);
                    #endif
                    
                    // Check fit quality and radius
                    bool is_good_fit = max_residual < 1 * R && mean_residual < 1 * R && R <= radius_limit;  // Updated radius limit
                    
                    // Determine detection status based on residual values
                    String detection_status = is_good_fit ? "Pipe Detected" : "No Pipe Detected";
                    
                    // Print detection status
                    #ifdef DEBUG_MODE
                    Serial.println(detection_status);
                    #endif
                    
                    // Calculate new final depth
                    float newFinalDepth = abs(finalDepth - (maxY - lowestY));
                    
                    // Append additional data to JSON
                    jsonData += "], \"tiltAngle\": " + String(tiltAngle) +
                                ", \"finalDepth\": " + String(finalDepth) +
                                ", \"newFinalDepth\": " + String(newFinalDepth) +
                                ", \"detectionStatus\": \"" + detection_status + "\"," +
                                "\"xc\": " + String(xc) + ", \"yc\": " + String(yc) + ", \"R\": " + String(R) +
                                ", \"max_residual\": " + String(max_residual) + ", \"mean_residual\": " + String(mean_residual) + 
                                ", \"radius_limit\": " + String(radius_limit) + ", \"is_good_fit\": " + String(is_good_fit) +
                                ", \"diameter_limit\": " + String(diameter_limit);

                    // Add the radius pipe diamter to the JSON data if a pipe is detected
                    if (is_good_fit) {
                        jsonData += ", \"pipeDiameter\": " + String(2 * R);
                    }
                  
                    jsonData += "}";

                    #ifdef DEBUG_MODE
                    Serial.print("Final depth value for the ditch is: ");
                    Serial.print(finalDepth, 2);
                    Serial.println(" mm");
                    Serial.println("New final depth value: ");
                    Serial.print(newFinalDepth, 2);
                    Serial.println(" mm");
                    Serial.println(jsonData);  // Debug: Print JSON data to serial
                    #endif

                } else {
                    #ifdef DEBUG_MODE
                    Serial.println("No valid point found for the bump top.");
                    #endif

                    // Set default values and send data
                    float newFinalDepth = finalDepth;
                    String detection_status = "No Pipe Detected";
                    float R = 0;  // Default value for radius

                    jsonData += "], \"tiltAngle\": " + String(tiltAngle) +
                                ", \"finalDepth\": " + String(finalDepth) +
                                ", \"newFinalDepth\": " + String(newFinalDepth) +
                                ", \"detectionStatus\": \"" + detection_status + "\"," +
                                "\"xc\": 0, \"yc\": 0, \"R\": " + String(R) +
                                ", \"max_residual\": 0, \"mean_residual\": 0, " +
                                "\"radius_limit\": 0, \"is_good_fit\": false, " +
                                "\"diameter_limit\": 0, " +
                                "\"pipeDiameter\": 0" +
                                "}";
                }
            }
        }

        if (finalDepth <= 0) {  // Check if the depth is above a certain minimal threshold
            #ifdef DEBUG_MODE
            Serial.println("Invalid depth detected. Resetting device...");
            #endif
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
    
    #ifdef DEBUG_MODE
        Serial.println("Data collection and processing complete.");
    #endif

    dataCollectionStarted = false;  // Reset data collection flag
    collectionComplete = true;  // Set flag to prevent further processing
}

// A function to send data to the server
void sendData(String jsonData) {
    if (WiFi.status() == WL_CONNECTED) {
        HTTPClient http;
        http.begin("http://192.168.1.103:3000/receive_data");  // Use the correct IP address in which your server is running in VS code
        http.addHeader("Content-Type", "application/json");
        
        const int maxRetries = 5; // Max number of retries
        int retryCount = 0;
        int httpResponseCode = -1;
        bool success = false;

        while (retryCount < maxRetries && !success) {
            http.setTimeout(20000); // Set timeout to 20 seconds
            httpResponseCode = http.POST(jsonData);

            if (httpResponseCode > 0) {
                success = true;
                String response = http.getString();
                Serial.println("HTTP Response code: " + String(httpResponseCode));
                Serial.println("Response: " + response);
            } else {
                Serial.println("Error on sending POST: " + String(httpResponseCode));
                Serial.println("Retry attempt: " + String(retryCount + 1));
                Serial.println("Payload: " + jsonData);
                Serial.println("HTTP error: " + http.errorToString(httpResponseCode));
                retryCount++;
                delay(2000); // Wait for 2 seconds before retrying
            }
        }

        http.end();
    } else {
        Serial.println("WiFi Disconnected");
    }
}
