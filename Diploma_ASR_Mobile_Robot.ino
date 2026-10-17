#include "esp_camera.h"

// ========== Визначення пінів камери AI Thinker ESP32-CAM ==========
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27
#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

// ========== Налаштування послідовного порту ==========
#define SERIAL_BAUD 115200

// ========== Параметри розпізнавання ліній ==========
#define SCAN_ROW_RATIO 0.75
#define LINE_THRESHOLD 100
#define IMAGE_WIDTH 320
#define IMAGE_HEIGHT 240
#define CENTER_X (IMAGE_WIDTH / 2)

// ========== Параметри керування ==========
#define BASE_SPEED 80
#define KP 0.5f
#define MAX_STEER 60

// ========== Прапорці режимів ==========
#define DEBUG_MODE true     // true = виводити в USB-серіал, false = тихо
#define SEND_TO_UART2 false // true = надсилати пакети на Arduino (коли підключите)
#define PRINT_RAW_ROW false // true = виводити сирі значення пікселів (дуже багато)

void setup()
{
    Serial.begin(115200);
    delay(500);
    Serial.println();
    Serial.println("========================================");
    Serial.println(" ESP32-CAM Line Follower - DEBUG MODE");
    Serial.println("========================================");

    // Ініціалізація камери (режим сірого зображення)
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sccb_sda = SIOD_GPIO_NUM;
    config.pin_sccb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_GRAYSCALE;
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK)
    {
        Serial.printf("[ERROR] Camera init failed: 0x%x\n", err);
        return;
    }
    Serial.println("[OK] Camera initialized (GRAYSCALE QVGA)");

    // Ініціалізація UART2 (поки не використовується для передачі, але готуємо)
    Serial2.begin(SERIAL_BAUD, SERIAL_8N1, -1, 12);
    Serial.println("[OK] UART2 ready on GPIO12 (TX)");
    Serial.println("[INFO] Starting line detection loop...");
    Serial.println("----------------------------------------");
    Serial.println("Формат виводу:");
    Serial.println("  [n] Line:xxx Err:xxx Steer:xxx L:xxx R:xxx Dark:xxx");
    Serial.println("  Line  = позиція центру лінії в пікселях (0-319)");
    Serial.println("  Err   = відхилення від центру (-160...+159)");
    Serial.println("  Steer = величина коригування");
    Serial.println("  L/R   = швидкість лівого та правого коліс");
    Serial.println("  Dark  = кількість темних пікселів у рядку сканування");
    Serial.println("----------------------------------------");
}

void loop()
{
    static uint32_t frameCounter = 0;
    static uint32_t lastTime = 0;
    uint32_t now = millis();

    // ========== 1. Захоплення кадру ==========
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb)
    {
        Serial.println("[ERROR] Frame capture failed!");
        delay(100);
        return;
    }

    // ========== 2. Сканування рядка ==========
    int scanRow = (int)(IMAGE_HEIGHT * SCAN_ROW_RATIO);
    uint8_t *rowPtr = fb->buf + (scanRow * IMAGE_WIDTH);

    long weightedSum = 0;
    int darkPixelCount = 0;
    int minPixel = 255;
    int maxPixel = 0;

    for (int x = 0; x < IMAGE_WIDTH; x++)
    {
        uint8_t pixel = rowPtr[x];
        if (pixel < minPixel)
            minPixel = pixel;
        if (pixel > maxPixel)
            maxPixel = pixel;

        if (pixel < LINE_THRESHOLD)
        {
            weightedSum += x * (255 - pixel);
            darkPixelCount++;
        }
    }

    // ========== 3. Обчислення позиції лінії ==========
    int linePos = CENTER_X;
    bool lineFound = (darkPixelCount > 5);
    if (lineFound)
    {
        linePos = (int)(weightedSum / darkPixelCount);
    }

    esp_camera_fb_return(fb);

    // ========== 4. Обчислення керування ==========
    int error = linePos - CENTER_X;
    int steer = (int)(error * KP);
    steer = constrain(steer, -MAX_STEER, MAX_STEER);

    int leftSpeed = BASE_SPEED - steer;
    int rightSpeed = BASE_SPEED + steer;
    leftSpeed = constrain(leftSpeed, -127, 127);
    rightSpeed = constrain(rightSpeed, -127, 127);

    // ========== 5. Вивід у серійний порт ==========
    if (DEBUG_MODE)
    {
        frameCounter++;

        // Компактний рядок кожного кадру
        Serial.printf("[%lu] Line:%d Err:%d Steer:%d L:%d R:%d Dark:%d",
                      frameCounter, linePos, error, steer, leftSpeed, rightSpeed, darkPixelCount);

        if (!lineFound)
        {
            Serial.print("  <-- LINE LOST!");
        }
        Serial.println();

        // Додаткова діагностика кожні 20 кадрів
        if (frameCounter % 20 == 0)
        {
            float fps = 20000.0f / (now - lastTime); // 20 кадрів / час
            lastTime = now;
            Serial.printf("  [STATS] FPS:%.1f MinPixel:%d MaxPixel:%d Threshold:%d\n",
                          fps, minPixel, maxPixel, LINE_THRESHOLD);
        }

        // Сирий рядок пікселів (для глибокого відлагодження, дуже об'ємно)
        if (PRINT_RAW_ROW)
        {
            Serial.print("  RAW: ");
            for (int x = 0; x < IMAGE_WIDTH; x += 8)
            { // кожен 8-й піксель
                Serial.printf("%3d ", rowPtr[x]);
            }
            Serial.println();
        }
    }

    // ========== 6. Відправка на Arduino (поки вимкнено) ==========
    if (SEND_TO_UART2)
    {
        uint8_t packet[6];
        packet[0] = 0xAA;
        packet[1] = 0x55;
        packet[2] = (leftSpeed >> 8) & 0xFF;
        packet[3] = leftSpeed & 0xFF;
        packet[4] = (rightSpeed >> 8) & 0xFF;
        packet[5] = rightSpeed & 0xFF;
        Serial2.write(packet, 6);
    }

    delay(50); // ~20 Гц
}