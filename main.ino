#include <Wire.h>
#include "mlx_sensor.h"
#include "thermal_image_processor.h"
#include "config.h"

// MLX sensor instance
MlxSensor mlx_sensor;

// Thermal processor instance
ThermalProcessor thermal_processor;

// Frame buffer for raw MLX90640 data (768 pixels)
float mlx_frame[768];

// Converted 8-bit thermal image (24x32)
uint8_t thermal_image[IMAGE_HEIGHT][IMAGE_WIDTH];

// Detected people array
DetectedPerson detected_people[MAX_PEOPLE];

// Background initialization counter
uint16_t bg_init_counter = 0;

void setup() {
    Serial.begin(57600);
    while (!Serial);
    
    Serial.println("=== CrowdAware Node 2026 ===");
    Serial.println("Initializing MLX90640 thermal sensor...");
    
    // Initialize MLX sensor with custom I2C pins (adjust as needed)
    // For Arduino boards: SDA=4, SCL=5, Speed=1MHz
    if (!mlx_sensor.init(MLX90640_ADDRESS, 4, 5, MLX_I2C_SPEED)) {
        Serial.println("ERROR: Failed to initialize MLX90640!");
        while (1);
    }
    
    Serial.println("MLX90640 initialized successfully.");
    
    // Initialize thermal processor
    thermal_processor_init(&thermal_processor);
    Serial.println("Thermal processor initialized.");
    
    Serial.println("Beginning background frame initialization...");
    Serial.print("Frames remaining: ");
    Serial.println(BG_FRAME_COUNT);
}

void loop() {
    // Read thermal frame from MLX90640
    if (!mlx_sensor.readFrame(mlx_frame)) {
        Serial.println("ERROR: Failed to read MLX90640 frame!");
        return;
    }
    
    // Convert float values to 8-bit grayscale (0-255)
    // Assuming temperature range: -40°C to 85°C
    // Map to 0-255 range
    convert_to_8bit_image(mlx_frame, thermal_image);
    
    // Background initialization phase
    if (bg_init_counter < BG_FRAME_COUNT) {
        // Use exponential moving average for background (lower alpha for slower convergence)
        uint8_t alpha = 25; // Slow adaptation (0-255 scale)
        update_background(&thermal_processor, (const uint8_t (*)[IMAGE_WIDTH])thermal_image, alpha);
        
        bg_init_counter++;
        
        if (bg_init_counter % 5 == 0) {
            Serial.print("Background init: ");
            Serial.print(BG_FRAME_COUNT - bg_init_counter);
            Serial.println(" frames remaining");
        }
        return; // Skip processing during background initialization
    }
    
    // Process thermal frame
    uint8_t num_detected = process_thermal_frame(
        &thermal_processor,
        (const uint8_t (*)[IMAGE_WIDTH])thermal_image,
        detected_people,
        MAX_PEOPLE
    );
    
    // Update background with current frame (adaptive)
    uint8_t bg_alpha = 15; // Slower background update after initialization
    update_background(&thermal_processor, (const uint8_t (*)[IMAGE_WIDTH])thermal_image, bg_alpha);
    
    // Output results
    output_detection_results(num_detected);
}

/**
 * @brief Output detected human coordinates to serial
 */
void output_detection_results(uint8_t num_detected) {
    Serial.print("Detected: ");
    Serial.print(num_detected);
    Serial.println(" person(s)");
    
    if (num_detected > 0) {
        for (uint8_t i = 0; i < num_detected; i++) {
            Serial.print("  Person ");
            Serial.print(i + 1);
            Serial.print(": X=");
            Serial.print(detected_people[i].x);
            Serial.print(" Y=");
            Serial.print(detected_people[i].y);
            Serial.print(" Area=");
            Serial.print(detected_people[i].area);
            Serial.print(" pixels, MaxDist=");
            Serial.println(detected_people[i].max_distance);
        }
    }
    
    Serial.println();
}