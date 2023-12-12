// Helperfunction for C++ projects on Arduino - project I2 - 06.12.2023
// runnable 07.12.2023

////////////////////////////////////////////////////////////////////////////////

// Imports
#include <Arduino.h>

// Definition of datatypes
struct SensorData {
    int sensorid;
    double value;
    String timestamp;
};

String createJson(bool debug, const SensorData sensorData[], size_t dataSize) {
    String result = "{\n";
    
// append debug
    result += "  \"debug\": " + String(debug ? "true" : "false") + ",\n";

// append sensors
    result += "  \"sensors\": [\n";
    
// insert sensordata into json structure
    for (size_t i = 0; i < dataSize; ++i) {
        result += "    {\n";
        result += "      \"sensorid\": " + String(sensorData[i].sensorid) + ",\n";
        result += "      \"value\": " + String(sensorData[i].value) + ",\n";
        result += "      \"timestamp\": \"" + (sensorData[i].timestamp) + "\"\n";
        result += "    },\n";
    }

// remove last comma and new line in case there is no other sensor to append
    if (dataSize > 0) {
        result.remove(result.length() - 2, 2);
    }

// Formatting the head of the json
    result += "  \n  ]\n";
    result += "}\n";

// Return finised json back to main
    return result;
}
