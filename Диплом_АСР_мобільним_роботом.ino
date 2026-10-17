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

// ========== Налаштування послідовного порту (для зв'язку з Arduino) ==========
// Використовуємо UART2, лише TX (GPIO12) -> RX Arduino, обов'язково з'єднати GND
#define SERIAL_BAUD 115200

// ========== Параметри розпізнавання ліній ==========
#define SCAN_ROW_RATIO 0.75 // Сканування рядка на 75% висоти зображення (ближче до нижньої частини)
#define LINE_THRESHOLD 100  // Поріг темних пікселів: значення нижче цього вважається лінією
#define IMAGE_WIDTH 320     // Ширина QVGA
#define IMAGE_HEIGHT 240    // Висота QVGA
#define CENTER_X (IMAGE_WIDTH / 2)

// ========== Параметри керування ==========
#define BASE_SPEED 80 // Базова швидкість руху вперед
#define KP 0.5f       // Пропорційний коефіцієнт (чим більше, тим чутливіше керування)
#define MAX_STEER 60  // Максимальна різниця швидкості

void setup()
{
    Serial.begin(115200);
    Serial.println("ESP32-CAM Line Follower Start...");

    // Ініціалізація камери (режим сірого зображення для максимальної швидкості)
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
    config.pixel_format = PIXFORMAT_GRAYSCALE; // Ключове: сіре зображення, швидша обробка
    config.frame_size = FRAMESIZE_QVGA;        // 320x240
    config.jpeg_quality = 12;
    config.fb_count = 1;

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK)
    {
        Serial.printf("Camera init failed: 0x%x\n", err);
        return;
    }

    // Ініціалізація UART2, лише TX (GPIO12), RX не використовується
    Serial2.begin(SERIAL_BAUD, SERIAL_8N1, -1, 12);
    Serial.println("Camera Ready.");
}

void loop()
{
    // 1. Захоплення кадру
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb)
    {
        Serial.println("Capture Failed");
        return;
    }

    // 2. Сканування одного рядка в нижній частині зображення для пошуку центру лінії
    int scanRow = (int)(IMAGE_HEIGHT * SCAN_ROW_RATIO);
    uint8_t *rowPtr = fb->buf + (scanRow * IMAGE_WIDTH);

    long weightedSum = 0;
    int darkPixelCount = 0;

    for (int x = 0; x < IMAGE_WIDTH; x++)
    {
        uint8_t pixel = rowPtr[x];
        if (pixel < LINE_THRESHOLD)
        {                                     // Вважаємо темні пікселі лінією
            weightedSum += x * (255 - pixel); // Чим темніше, тим більша вага
            darkPixelCount++;
        }
    }

    // 3. Обчислення позиції лінії
    int linePos = CENTER_X; // Якщо лінію не знайдено, за замовчуванням центр (рух прямо)
    if (darkPixelCount > 5)
    {
        linePos = (int)(weightedSum / darkPixelCount);
    }

    esp_camera_fb_return(fb); // Обов'язково звільнити буфер

    // 4. Обчислення відхилення та команди керування
    int error = linePos - CENTER_X; // Від'ємне = лінія ліворуч, додатне = праворуч
    int steer = (int)(error * KP);
    steer = constrain(steer, -MAX_STEER, MAX_STEER);

    // Лінія ліворуч -> потрібно повертати ліворуч -> праве колесо швидше, ліве повільніше
    // (Конкретний напрямок залежить від монтажу, за потреби поміняйте місцями leftSpeed та rightSpeed)
    int leftSpeed = BASE_SPEED - steer;
    int rightSpeed = BASE_SPEED + steer;

    leftSpeed = constrain(leftSpeed, -127, 127);
    rightSpeed = constrain(rightSpeed, -127, 127);

    // 5. Пакування та надсилання на Arduino
    // Формат: 0xAA 0x55 [left_high] [left_low] [right_high] [right_low]
    uint8_t packet[6];
    packet[0] = 0xAA;
    packet[1] = 0x55;
    packet[2] = (leftSpeed >> 8) & 0xFF;
    packet[3] = leftSpeed & 0xFF;
    packet[4] = (rightSpeed >> 8) & 0xFF;
    packet[5] = rightSpeed & 0xFF;

    Serial2.write(packet, 6);

    // Відлагоджувальний вивід (можна переглянути через USB)
    Serial.printf("Line:%d Err:%d L:%d R:%d\n", linePos, error, leftSpeed, rightSpeed);

    delay(50); // Частота циклу близько 20 Гц, достатньо для плавного руху
}