#include "esp_camera.h" // Драйвер камери ESP32 (OV2640 на платі AI Thinker)

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

// ========== UART ==========
#define SERIAL_BAUD 115200

// ========== Параметри розпізнавання ==========
// ПІСЛЯ ПЕРЕВЕРТАННЯ 180°: рядок 0.85 тепер фізично відповідає
// верхній частині сцени (далі від робота). Якщо хочете сканувати
// зону БЛИЖЧЕ до робота — поставте 0.15 (бо кадр перевернуто).
#define SCAN_ROW_RATIO 0.85
#define LINE_THRESHOLD 100
#define IMAGE_WIDTH 320
#define IMAGE_HEIGHT 240
#define CENTER_X (IMAGE_WIDTH / 2)

// ===== Параметри надійності =====
#define MIN_DARK_PIXELS 20
#define MAX_GAP_PIXELS 12
#define DEAD_ZONE 6
#define MAX_JUMP_PER_FRAME 60

// ========== Параметри керування ==========
#define BASE_SPEED 70
#define KP 1.0f
#define MAX_STEER 70
#define LOST_SPEED_FACTOR 0.8f

// ========== Прапорці ==========
#define DEBUG_MODE true
#define SEND_TO_UART2 true
#define PRINT_RAW_ROW false

// ========== Стан між кадрами ==========
static int prevLinePos = CENTER_X;
static int lastKnownDirection = 0;
static bool wasLost = false;

void setup()
{
    Serial.begin(115200);
    delay(500);

    Serial.println();
    Serial.println("==========================================");
    Serial.println(" ESP32-CAM Line Follower - DEBUG MODE");
    Serial.println("==========================================");

    // ===== Ініціалізація камери =====
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

    // ===== ПЕРЕВЕРТАННЯ КАДРУ НА 180° (ДОДАНО) =====
    // vflip  = вертикальне перевертання (догори ногами)
    // hmirror = горизонтальне дзеркалення (ліворуч-праворуч)
    // Разом дають поворот на 180°.
    sensor_t *s = esp_camera_sensor_get();
    if (s != NULL)
    {
        s->set_vflip(s, 1);   // <-- ДОДАНО
        s->set_hmirror(s, 1); // <-- ДОДАНО
        Serial.println("[OK] Frame rotated 180° (vflip=1, hmirror=1)");
    }
    else
    {
        Serial.println("[WARN] Could not get sensor handle — rotation not applied");
    }

    // ===== Ініціалізація UART2 =====
    Serial2.begin(SERIAL_BAUD, SERIAL_8N1, -1, 13);
    Serial.println("[OK] UART2 ready on GPIO13 (TX)");

    Serial.println("------------------------------------------");
    Serial.println("Формат: [n] LINE:pos DIR:напрямок ERR:x L:x R:x DARK:x");
    Serial.println("  DIR: STRAIGHT | LEFT | RIGHT | LOST");
    Serial.println("------------------------------------------");
}

// ===== Допоміжна: перевіряє компактність групи темних пікселів =====
bool isCompactGroup(uint8_t *row, int maxGap, int minPixels)
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

    if (count < minPixels)
        return false;
    return (maxGapSeen <= maxGap);
}

const char *directionStr(int dir)
{
    switch (dir)
    {
    case -1:
        return "LEFT   ";
    case 1:
        return "RIGHT  ";
    case 0:
        return "STRAIGHT";
    default:
        return "?";
    }
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
    long weightSum = 0;
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
            int w = 255 - pixel;
            weightedSum += (long)x * w;
            weightSum += w;
            darkPixelCount++;
        }
    }

    // ========== 3. Перевірка, чи це справді лінія ==========
    bool lineFound = isCompactGroup(rowPtr, MAX_GAP_PIXELS, MIN_DARK_PIXELS);

    // ========== 4. Обчислення позиції лінії з фільтрацією ==========
    int rawLinePos = CENTER_X;
    if (lineFound && weightSum > 0)
    {
        rawLinePos = (int)lroundf((float)weightedSum / (float)weightSum);

        int delta = rawLinePos - prevLinePos;
        if (delta > MAX_JUMP_PER_FRAME)
            delta = MAX_JUMP_PER_FRAME;
        if (delta < -MAX_JUMP_PER_FRAME)
            delta = -MAX_JUMP_PER_FRAME;
        rawLinePos = prevLinePos + delta;

        prevLinePos = rawLinePos;
        wasLost = false;
    }
    else
    {
        wasLost = true;
    }

    esp_camera_fb_return(fb);

    // ========== 5. Обчислення похибки з мертвою зоною ==========
    int error = 0;
    int direction = 0;

    if (lineFound)
    {
        error = rawLinePos - CENTER_X;

        if (abs(error) < DEAD_ZONE)
        {
            error = 0;
            direction = 0;
        }
        else if (error < 0)
        {
            direction = -1;
        }
        else
        {
            direction = 1;
        }

        lastKnownDirection = direction;
    }
    else
    {
        direction = lastKnownDirection;
        error = lastKnownDirection * CENTER_X;
    }

    // ========== 6. Обчислення швидкостей ==========
    int steer = 0;
    int leftSpeed, rightSpeed;

    if (lineFound)
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

        if (lastKnownDirection < 0)
        {
            leftSpeed = 0;
            rightSpeed = searchSpeed;
        }
        else if (lastKnownDirection > 0)
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

    // ========== 7. Вивід у серіал ==========
    if (DEBUG_MODE)
    {
        frameCounter++;

        const char *dirStr = lineFound ? directionStr(direction) : "LOST   ";

        Serial.printf("[%lu] LINE:%3d DIR:%s ERR:%+4d L:%+4d R:%+4d DARK:%3d",
                      frameCounter,
                      lineFound ? rawLinePos : -1,
                      dirStr,
                      lineFound ? error : 0,
                      leftSpeed,
                      rightSpeed,
                      darkPixelCount);

        if (!lineFound)
            Serial.print("  <-- LOST, searching ");
        if (!lineFound && lastKnownDirection < 0)
            Serial.print("LEFT");
        if (!lineFound && lastKnownDirection > 0)
            Serial.print("RIGHT");
        if (!lineFound && lastKnownDirection == 0)
            Serial.print("(stop)");
        Serial.println();

        if (frameCounter % 20 == 0)
        {
            float fps = 0;
            if (lastTime > 0)
                fps = 20000.0f / (now - lastTime);
            lastTime = now;

            Serial.printf("  [STATS] FPS:%.1f Min:%d Max:%d Thr:%d Compact:%s\n",
                          fps, minPixel, maxPixel, LINE_THRESHOLD,
                          lineFound ? "YES" : "NO");
        }

        if (PRINT_RAW_ROW)
        {
            Serial.print("  RAW: ");
            for (int x = 0; x < IMAGE_WIDTH; x += 8)
                Serial.printf("%3d ", rowPtr[x]);
            Serial.println();
        }
    }

    // ========== 8. Відправка на Arduino ==========
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

    delay(20);
}