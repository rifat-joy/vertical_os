#include "main.h"

void setup()
{
  Serial.begin(115200);
  Serial.println("Detecting Chip Info...");

  // Get the chip information
  esp_chip_info_t chip_info;
  esp_chip_info(&chip_info);

  // Print the chip information
  Serial.printf("Chip Info:\n");
  Serial.printf("  Model: %d\n", chip_info.model);
  Serial.printf("  Cores: %d\n", chip_info.cores);
  Serial.printf("  Features: 0x%x\n", chip_info.features);

  Serial.println("Detecting Flash, RAM and PSRAM Size...");

  // Get the Flash and RAM size
  uint32_t flash_size = ESP.getFlashChipSize();
  uint32_t ram_size = ESP.getFreeHeap();
  size_t psram_size = ESP.getPsramSize();

  // Print the Flash and RAM size in bytes and megabytes
  Serial.printf("Flash Size: %d bytes (%.2f MB)\n", flash_size, (float)flash_size / (1024.0 * 1024.0));
  Serial.printf("RAM Size: %d bytes (%.2f KB)\n", ram_size, (float)ram_size / 1024.0);
  Serial.printf("PSRAM Size: %d bytes (%.2f MB)\n", psram_size, (float)psram_size / (1024.0 * 1024.0));
}

void loop()
{
  // Do nothing
}