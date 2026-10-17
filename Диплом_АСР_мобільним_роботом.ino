#include "esp_camera.h"

// ========== Визначення пінів для AI Thinker ESP32-CAM ==========
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

// ========== Параметри послідовного порту ==========
// Використовуємо UART2, TX = GPIO12, RX не використовується (лише надсилання)
// Підключення: GPIO12 -> Arduino RX (D0), спільний GND
#define SERIAL_BAUD 115200

// ========== Параметри розпізнавання ліній ==========
#define SCAN_ROW_RATIO 0.75 // Сканування рядка на 3/4 висоти (ближче до нижньої частини)
#define LINE_THRESHOLD 100  // Поріг темних пікселів (0-255), менше цього значення вважається лінією
#define IMAGE_WIDTH 320     // Ширина QVGA
#define IMAGE_HEIGHT 240    // Висота QVGA
#define CENTER_X (IMAGE_WIDTH / 2)

// ========== Параметри керування ==========
#define BASE_SPEED 80 // Базова швидкість руху
#define KP 0.5f       // Пропорційний коефіцієнт
#define MAX_STEER 60  // Максимальна різниця швидкості

// ========== Глобальні змінні ==========
int lastLinePos = CENTER_X; // Позиція лінії в попередньому кадрі (для фільтрації)

void setup()
{
    Serial.begin(115200);
    Serial.println("ESP32-CAM Line Follower Starting...");

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
    config.pixel_format = PIXFORMAT_GRAYSCALE; // Сіре зображення для прискорення обробки
    config.frame_size = FRAMESIZE_QVGA;        // 320x240
    config.jpeg_quality = 12;
    config.fb_count = 1; // Один буфер достатньо для покадрової обробки

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK)
    {
        Serial.printf("Camera init failed with error 0x%x\n", err);
        return;
    }

    // Ініціалізація UART2 для зв'язку з Arduino
    Serial2.begin(SERIAL_BAUD, SERIAL_8N1, -1, 12); // TX = GPIO12
    Serial.println("Camera ready. Line detection active.");
}

void loop()
{
    // Захоплення кадру
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb)
    {
        Serial.println("Frame capture failed");
        return;
    }

    // Сканування рядка для пошуку позиції лінії
    int scanRow = (int)(IMAGE_HEIGHT * SCAN_ROW_RATIO);
    uint8_t *rowPtr = fb->buf + (scanRow * IMAGE_WIDTH);

    // Обчислення центру мас темних пікселів (лінії)
    long weightedSum = 0;
    int darkPixelCount = 0;

    for (int x = 0; x < IMAGE_WIDTH; x++)
    {
        uint8_t pixel = rowPtr[x];
        if (pixel < LINE_THRESHOLD)
        {                                     // Темний піксель = лінія
            weightedSum += x * (255 - pixel); // Чим темніше, тим більша вага
            darkPixelCount++;
        }
    }

    // Якщо знайдено достатньо темних пікселів, вважаємо, що лінію виявлено
    int linePos = CENTER_X; // За замовчуванням у центрі
    if (darkPixelCount > 5)
    {
        linePos = (int)(weightedSum / darkPixelCount);
    }

    // Звільнення буфера кадру
    esp_camera_fb_return(fb);

    // ========== Обчислення керуючої величини ==========
    int error = linePos - CENTER_X; // Додатне = лінія праворуч, від'ємне = лінія ліворуч
    int steer = (int)(error * KP);  // Пропорційне керування

    // Обмеження максимальної різниці швидкості
    steer = constrain(steer, -MAX_STEER, MAX_STEER);

    // Розрахунок швидкості лівого та правого коліс
    // Лінія праворуч -> правий поворот -> ліве колесо швидше, праве повільніше
    int leftSpeed = BASE_SPEED + steer;
    int rightSpeed = BASE_SPEED - steer;

    // Обмеження діапазону швидкості [-127, 127]
    leftSpeed = constrain(leftSpeed, -127, 127);
    rightSpeed = constrain(rightSpeed, -127, 127);

    // ========== Надсилання команди на Arduino ==========
    // Формат пакету: 0xAA 0x55 leftSpeed rightSpeed (кожне 2 байти, int16_t)
    uint8_t packet[6];
    packet[0] = 0xAA;
    packet[1] = 0x55;
    packet[2] = (leftSpeed >> 8) & 0xFF;
    packet[3] = leftSpeed & 0xFF;
    packet[4] = (rightSpeed >> 8) & 0xFF;
    packet[5] = rightSpeed & 0xFF;

    Serial2.write(packet, 6);

    // Відлагоджувальний вивід
    Serial.printf("Line: %d, Error: %d, L: %d, R: %d\n",
                  linePos, error, leftSpeed, rightSpeed);

    delay(50); // Цикл ~20 Гц, достатньо для базового слідування
}