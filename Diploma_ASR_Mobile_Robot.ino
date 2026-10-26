#include "esp_camera.h"

// ========== Піни камери AI Thinker ==========
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

// ========== UART ==========
#define SERIAL_BAUD 115200

// ========== Параметри розпізнавання ==========
#define SCAN_ROW_RATIO 0.40
#define LINE_THRESHOLD 120
#define IMAGE_WIDTH 320
#define IMAGE_HEIGHT 240
#define CENTER_X (IMAGE_WIDTH / 2)

// ===== Надійність =====
#define MIN_DARK_PIXELS 20
#define MAX_GAP_PIXELS 12
#define DEAD_ZONE 6
#define MAX_JUMP_PER_FRAME 60

// ========== Керування ==========
#define BASE_SPEED 60
#define KP 1.0f
#define MAX_STEER 70
#define LOST_SPEED_FACTOR 0.8f

// ========== Прапорці ==========
#define DEBUG_MODE true
#define SEND_TO_UART2 true
#define PRINT_RAW_ROW false

// ========== Черга між ядрами ==========
typedef struct
{
    int linePos;
    int darkCount;
    bool lineFound;
} LineData;

QueueHandle_t lineQueue;

// ========== Стан (доступний лише в Core 1) ==========
static int prevLinePos = CENTER_X;
static int lastKnownDirection = 0;

// ========== Функція сканування рядка (виконується в Core 0) ==========
bool isCompactGroup(uint8_t *row, int maxGap, int minPixels, int &outDarkCount)
{
    int count = 0;
    int lastDark = -1;
    int maxGapSeen = 0;

    for (int x = 0; x < IMAGE_WIDTH; x++)
    {
        if (row[x] < LINE_THRESHOLD)
        {
            count++;
            if (lastDark >= 0)
            {
                int gap = x - lastDark;
                if (gap > maxGapSeen)
                    maxGapSeen = gap;
            }
            lastDark = x;
        }
    }
    outDarkCount = count;
    if (count < minPixels)
        return false;
    return (maxGapSeen <= maxGap);
}

// ========== Завдання Core 0: отримання кадру + вилучення даних рядка ==========
void cameraTask(void *pvParameters)
{
    for (;;)
    {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb)
        {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        int scanRow = (int)(IMAGE_HEIGHT * SCAN_ROW_RATIO);
        uint8_t *rowPtr = fb->buf + (scanRow * IMAGE_WIDTH);

        long weightedSum = 0;
        long weightSum = 0;
        int darkPixelCount = 0;

        for (int x = 0; x < IMAGE_WIDTH; x++)
        {
            uint8_t pixel = rowPtr[x];
            if (pixel < LINE_THRESHOLD)
            {
                int w = 255 - pixel;
                weightedSum += (long)x * w;
                weightSum += w;
                darkPixelCount++;
            }
        }

        int dummyDark = 0;
        bool lineFound = isCompactGroup(rowPtr, MAX_GAP_PIXELS, MIN_DARK_PIXELS, dummyDark);

        LineData data;
        data.lineFound = lineFound;
        data.darkCount = darkPixelCount;

        if (lineFound && weightSum > 0)
            data.linePos = (int)lroundf((float)weightedSum / (float)weightSum);
        else
            data.linePos = CENTER_X;

        // Надсилаємо в чергу, не блокуємо (якщо черга повна, відкидаємо найстаріший кадр)
        if (xQueueSend(lineQueue, &data, 0) != pdTRUE)
        {
            // Черга повна: видаляємо найстаріший елемент і повторюємо спробу
            LineData old;
            xQueueReceive(lineQueue, &old, 0);
            xQueueSend(lineQueue, &data, 0);
        }

        esp_camera_fb_return(fb);
        // Невелика затримка, щоб дозволити планувальнику працювати з іншими завданнями
        vTaskDelay(1);
    }
}

void setup()
{
    Serial.begin(115200);
    delay(500);

    Serial.println("==========================================");
    Serial.println(" ESP32-CAM Line Follower - DUAL CORE");
    Serial.println("==========================================");

    // Ініціалізація камери
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
    config.fb_count = 2;
    config.grab_mode = CAMERA_GRAB_LATEST;

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK)
    {
        Serial.printf("[ERROR] Camera init failed: 0x%x\n", err);
        return;
    }
    Serial.println("[OK] Camera initialized (GRAYSCALE QVGA)");

    // Перевертання
    sensor_t *s = esp_camera_sensor_get();
    if (s != NULL)
    {
        s->set_vflip(s, 1);
        s->set_hmirror(s, 1);
        Serial.println("[OK] Frame rotated 180°");
    }

    // UART2
    Serial2.begin(SERIAL_BAUD, SERIAL_8N1, -1, 13);
    Serial.println("[OK] UART2 ready on GPIO13 (TX)");

    // Створення черги (глибина 2 достатньо для конвеєра)
    lineQueue = xQueueCreate(2, sizeof(LineData));
    if (lineQueue == NULL)
    {
        Serial.println("[ERROR] Queue creation failed!");
        return;
    }

    // Створення завдання камери в Core 0, пріоритет 2
    xTaskCreatePinnedToCore(
        cameraTask,
        "CameraTask",
        4096,
        NULL,
        2,
        NULL,
        0);
    Serial.println("[OK] Camera task pinned to Core 0");
    Serial.println("------------------------------------------");
}

void loop()
{
    // ========== Core 1: обробка даних рядка та відправка UART ==========
    static uint32_t frameCounter = 0;
    static uint32_t lastTime = 0;

    LineData data;
    // Чекаємо дані з черги, максимум 100 мс
    if (xQueueReceive(lineQueue, &data, pdMS_TO_TICKS(100)) != pdTRUE)
    {
        // Немає даних, нічого не робимо
        return;
    }

    frameCounter++;
    uint32_t now = millis();

    // Фільтрація стрибків
    int rawLinePos = data.linePos;
    if (data.lineFound)
    {
        int delta = rawLinePos - prevLinePos;
        if (delta > MAX_JUMP_PER_FRAME)
            delta = MAX_JUMP_PER_FRAME;
        if (delta < -MAX_JUMP_PER_FRAME)
            delta = -MAX_JUMP_PER_FRAME;
        rawLinePos = prevLinePos + delta;
        prevLinePos = rawLinePos;
    }

    // Обчислення похибки
    int error = 0;
    int direction = 0;

    if (data.lineFound)
    {
        error = rawLinePos - CENTER_X;
        if (abs(error) < DEAD_ZONE)
        {
            error = 0;
            direction = 0;
        }
        else if (error < 0)
            direction = -1;
        else
            direction = 1;
        lastKnownDirection = direction;
    }
    else
    {
        direction = lastKnownDirection;
        error = lastKnownDirection * CENTER_X;
    }

    // Обчислення швидкості
    int steer = 0;
    int leftSpeed, rightSpeed;

    if (data.lineFound)
    {
        steer = (int)lroundf(error * KP);
        steer = constrain(steer, -MAX_STEER, MAX_STEER);
        leftSpeed = BASE_SPEED - steer;
        rightSpeed = BASE_SPEED + steer;
        leftSpeed = constrain(leftSpeed, -100, 100);
        rightSpeed = constrain(rightSpeed, -100, 100);
    }
    else
    {
        int searchSpeed = (int)(BASE_SPEED * LOST_SPEED_FACTOR);
        if (lastKnownDirection > 0)
        {
            leftSpeed = 0;
            rightSpeed = searchSpeed;
        }
        else if (lastKnownDirection < 0)
        {
            leftSpeed = searchSpeed;
            rightSpeed = 0;
        }
        else
        {
            leftSpeed = 0;
            rightSpeed = 0;
        }
    }

    // Вивід
    if (DEBUG_MODE)
    {
        const char *dirStr = data.lineFound ? (direction == -1 ? "LEFT   " : direction == 1 ? "RIGHT  "
                                                                                            : "STRAIGHT")
                                            : "LOST   ";
        Serial.printf("[%lu] LINE:%3d DIR:%s ERR:%+4d L:%+4d R:%+4d DARK:%3d\n",
                      frameCounter, data.lineFound ? rawLinePos : -1, dirStr,
                      data.lineFound ? error : 0, leftSpeed, rightSpeed, data.darkCount);

        if (frameCounter % 20 == 0)
        {
            float fps = (lastTime > 0) ? 20000.0f / (now - lastTime) : 0;
            lastTime = now;
            Serial.printf("  [STATS] FPS:%.1f\n", fps);
        }
    }

    // Відправка UART
    if (SEND_TO_UART2)
    {
        uint8_t packet[6] = {0xAA, 0x55,
                             (uint8_t)((leftSpeed >> 8) & 0xFF), (uint8_t)(leftSpeed & 0xFF),
                             (uint8_t)((rightSpeed >> 8) & 0xFF), (uint8_t)(rightSpeed & 0xFF)};
        Serial2.write(packet, 6);
    }
}