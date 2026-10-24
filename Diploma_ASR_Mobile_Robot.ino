#include "esp_camera.h" // Драйвер камери ESP32 (OV2640 на платі AI Thinker)

// ========== Визначення пінів камери AI Thinker ESP32-CAM ==========
// Ці номери GPIO жорстко прив'язані до апаратної розводки плати.
// Змінювати їх не можна — інакше камера не працюватиме.
// Саме через те, що майже всі GPIO зайняті камерою,
// на ESP32-CAM майже не залишається вільних пінів для інших задач.
#define PWDN_GPIO_NUM 32  // Power Down — вимикання камери
#define RESET_GPIO_NUM -1 // Reset камери не підключений (-1 = не використовується)
#define XCLK_GPIO_NUM 0   // Зовнішній тактовий сигнал 20 МГц для камери
#define SIOD_GPIO_NUM 26  // SCCB (I2C-подібна шина) — лінія даних
#define SIOC_GPIO_NUM 27  // SCCB — лінія тактування
#define Y9_GPIO_NUM 35    // 8-бітна паралельна шина даних зображення
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25 // Вертикальна синхронізація (початок кадру)
#define HREF_GPIO_NUM 23  // Горизонтальна синхронізація (початок рядка)
#define PCLK_GPIO_NUM 22  // Тактовий сигнал пікселів

// ========== UART ==========
#define SERIAL_BAUD 115200 // Швидкість UART2 для зв'язку з Arduino.
                           // Має збігатися зі швидкістю Serial на Arduino!

// ========== Параметри розпізнавання ==========
#define SCAN_ROW_RATIO 0.75        // Скануємо рядок на 75% висоти кадру.
                                   // 0.75 — це "ближче до низу", тобто ближче до землі
                                   // перед роботом: саме там лінія найважливіша.
#define LINE_THRESHOLD 100         // Поріг яскравості: піксель із значенням < 100
                                   // вважається "лінією" (темним). Якщо лінія сіра —
                                   // зменшіть; якщо фон теж темний — збільшіть.
#define IMAGE_WIDTH 320            // Ширина кадру QVGA
#define IMAGE_HEIGHT 240           // Висота кадру QVGA
#define CENTER_X (IMAGE_WIDTH / 2) // Центр кадру = 160. Відносно нього
                                   // рахуємо відхилення лінії.

// ===== Параметри надійності =====
#define MIN_DARK_PIXELS 15    // Мінімум темних пікселів у рядку, щоб вважати,
                              // що лінію знайдено. 6 — це шум; 15 — уже схоже на лінію.
#define MAX_GAP_PIXELS 12     // Максимальний розрив між сусідніми темними пікселями.
                              // Якщо розрив більший — це вже дві окремі плями,
                              // а не одна лінія (захист від тіней/країв).
#define DEAD_ZONE 6           // Мертва зона: якщо |error| < 6 пікселів —
                              // вважаємо, що їдемо прямо (щоб робот не тремтів
                              // через дрібні коливання позиції лінії).
#define MAX_JUMP_PER_FRAME 40 // Наскільки linePos може змінитися за один кадр.
                              // Захист від різких викидів (тінь, край стола тощо):
                              // навіть якщо "сира" позиція стрибнула на 200 пікселів,
                              // ми дозволяємо змінитися лише на 40.

// ========== Параметри керування ==========
#define BASE_SPEED 80          // Базова швидкість руху прямо (0..127)
#define KP 0.5f                // Пропорційний коефіцієнт: наскільки сильно
                               // реагувати на відхилення. Більше KP — різкіші
                               // повороти; менше — плавніше, але робот може
                               // "вилітати" з траси.
#define MAX_STEER 60           // Обмеження максимального коригування.
                               // Не дає роботу "смикатися" на різких поворотах.
#define LOST_SPEED_FACTOR 0.5f // Коли лінію втрачено — шукаємо її
                               // зі швидкістю 50% від базової (щоб не вилетіти).

// ========== Прапорці ==========
#define DEBUG_MODE true     // true = друкувати діагностику в USB-серіал
#define SEND_TO_UART2 true // true = надсилати пакети на Arduino.
                            // Поки false — тільки дивимось у Serial Monitor.
#define PRINT_RAW_ROW false // true = друкувати сирі значення пікселів
                            // (для ручного калібрування порогу LINE_THRESHOLD)

// ========== Стан між кадрами ==========
// Ці змінні зберігають інформацію між викликами loop(),
// щоб можна було фільтрувати стрибки та пам'ятати, куди їхати при втраті лінії.
static int prevLinePos = CENTER_X; // Позиція лінії в попередньому кадрі
static int lastKnownDirection = 0; // Останній відомий напрямок:
                                   // -1 = ліворуч, 0 = прямо, +1 = праворуч
static bool wasLost = false;       // Чи була лінія втрачена в попередньому кадрі

void setup()
{
    // USB-серіал для людини (Serial Monitor)
    Serial.begin(115200);
    delay(500); // Пауза, щоб Serial Monitor встиг підключитися
                // і не "з'їв" перші рядки виводу

    Serial.println();
    Serial.println("==========================================");
    Serial.println(" ESP32-CAM Line Follower - DEBUG MODE");
    Serial.println("==========================================");

    // ===== Ініціалізація камери (режим сірого зображення) =====
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0; // Апаратний таймер для XCLK
    config.ledc_timer = LEDC_TIMER_0;

    // Прив'язка пінів камери (зверху)
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

    config.xclk_freq_hz = 20000000;            // 20 МГц — стандарт для OV2640
    config.pixel_format = PIXFORMAT_GRAYSCALE; // КЛЮЧОВЕ: сіре зображення, а не JPEG.
                                               // 1 байт = 1 піксель, без декодування.
                                               // Це дає прямий доступ до яскравості
                                               // і працює набагато швидше.
    config.frame_size = FRAMESIZE_QVGA;        // 320x240 — компроміс між
                                               // деталізацією та швидкістю
    config.jpeg_quality = 12;                  // У GRAYSCALE не використовується,
                                               // але поле має бути заповнене
    config.fb_count = 1;                       // Один буфер кадру — достатньо
                                               // для покадрової обробки

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK)
    {
        Serial.printf("[ERROR] Camera init failed: 0x%x\n", err);
        return; // Якщо камера не стартувала — далі немає сенсу
    }
    Serial.println("[OK] Camera initialized (GRAYSCALE QVGA)");

    // ===== Ініціалізація UART2 для зв'язку з Arduino =====
    // Параметри: швидкість, 8 біт даних / без парності / 1 стоп-біт,
    // RX = -1 (не використовуємо, ми лише передаємо),
    // TX = GPIO12 (фізичний пін, який підключимо до RX Arduino).
    //
    // УВАГА: GPIO12 — це strapping-пін. Під час старту ESP32 він
    // визначає напругу флеш-пам'яті. Якщо під час увімкнення на ньому
    // буде високий рівень — плата може не завантажитися.
    // Зазвичай RX Arduino не має підтяжки, тому все ок.
    Serial2.begin(SERIAL_BAUD, SERIAL_8N1, -1, 12);
    Serial.println("[OK] UART2 ready on GPIO12 (TX)");

    // ===== Пояснення формату виводу =====
    Serial.println("------------------------------------------");
    Serial.println("Формат: [n] LINE:pos DIR:напрямок ERR:x L:x R:x DARK:x");
    Serial.println("  DIR: STRAIGHT | LEFT | RIGHT | LOST");
    Serial.println("------------------------------------------");
}

// ===== Допоміжна: перевіряє, чи темні пікселі утворюють одну компактну групу =====
// Ідея: якщо між сусідніми темними пікселями є великий розрив (>maxGap),
// то це, ймовірно, дві різні плями (наприклад, лінія + тінь), а не одна лінія.
// Повертає true, якщо група компактна і містить достатньо пікселів.
bool isCompactGroup(uint8_t *row, int maxGap, int minPixels)
{
    int count = 0;      // Скільки темних пікселів знайшли
    int lastDark = -1;  // Позиція попереднього темного пікселя
    int maxGapSeen = 0; // Найбільший розрив між сусідніми темними пікселями

    for (int x = 0; x < IMAGE_WIDTH; x++)
    {
        if (row[x] < LINE_THRESHOLD) // Темний піксель
        {
            count++;
            if (lastDark >= 0)
            {
                int gap = x - lastDark; // Розрив між сусідніми темними
                if (gap > maxGapSeen)
                    maxGapSeen = gap;
            }
            lastDark = x;
        }
    }

    if (count < minPixels) // Замало темних пікселів — це шум
        return false;
    return (maxGapSeen <= maxGap); // Компактна група — лінія
}

// ===== Допоміжна: повертає напрямок словами для Serial =====
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
    static uint32_t frameCounter = 0; // Лічильник кадрів (зберігається між викликами)
    static uint32_t lastTime = 0;     // Час останнього вимірювання FPS

    uint32_t now = millis(); // Поточний час у мс

    // ========== 1. Захоплення кадру ==========
    // esp_camera_fb_get() блокує виконання, поки камера не віддасть готовий кадр.
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb)
    {
        // Камера іноді "заїкається" — пропускаємо кадр і пробуємо знову
        Serial.println("[ERROR] Frame capture failed!");
        delay(100);
        return;
    }

    // ========== 2. Сканування рядка ==========
    // Беремо ОДИН рядок пікселів на висоті 75% кадру.
    // fb->buf — плаский масив байтів: для GRAYSCALE 1 байт = 1 піксель.
    // Зсув = номер рядка * ширина кадру.
    int scanRow = (int)(IMAGE_HEIGHT * SCAN_ROW_RATIO);
    uint8_t *rowPtr = fb->buf + (scanRow * IMAGE_WIDTH);

    long weightedSum = 0;   // Сума (x * вага) — для обчислення центру мас
    long weightSum = 0;     // Сума ВАГ (а не кількості!) — ВИПРАВЛЕННЯ:
                            // раніше ділили на darkPixelCount, що давало
                            // величезні значення linePos (1600, 20000...).
                            // Тепер ділимо на суму ваг, і результат
                            // завжди буде в межах 0..319.
    int darkPixelCount = 0; // Кількість темних пікселів у рядку
    int minPixel = 255;     // Мінімальна яскравість у рядку (для діагностики)
    int maxPixel = 0;       // Максимальна яскравість у рядку (для діагностики)

    for (int x = 0; x < IMAGE_WIDTH; x++)
    {
        uint8_t pixel = rowPtr[x]; // Яскравість пікселя 0..255

        // Оновлюємо статистику рядка (для [STATS])
        if (pixel < minPixel)
            minPixel = pixel;
        if (pixel > maxPixel)
            maxPixel = pixel;

        // Перевіряємо, чи це "лінія"
        if (pixel < LINE_THRESHOLD)
        {
            // Вага = наскільки піксель темний (255 - pixel).
            // Чим темніше — тим сильніше він "тягне" центр мас до себе.
            // Це робить оцінку позиції стійкішою до шуму та розмитих країв.
            int w = 255 - pixel;
            weightedSum += (long)x * w; // x * вага
            weightSum += w;             // сума ваг (ВИПРАВЛЕННЯ)
            darkPixelCount++;
        }
    }

    // ========== 3. Перевірка, чи це справді лінія (а не шум/пляма) ==========
    bool lineFound = isCompactGroup(rowPtr, MAX_GAP_PIXELS, MIN_DARK_PIXELS);

    // ========== 4. Обчислення позиції лінії з фільтрацією ==========
    int rawLinePos = CENTER_X; // За замовчуванням — центр
    if (lineFound && weightSum > 0)
    {
        // Середньозважена позиція = Σ(x·w) / Σw
        // ВИПРАВЛЕННЯ: ділимо на weightSum, а не на darkPixelCount.
        // lroundf — правильне округлення (а не відсікання дробової частини).
        rawLinePos = (int)lroundf((float)weightedSum / (float)weightSum);

        // Обмеження стрибка: linePos не може змінитися більше ніж на MAX_JUMP_PER_FRAME.
        // Захист від викидів (тінь, край стола, випадкова темна пляма).
        int delta = rawLinePos - prevLinePos;
        if (delta > MAX_JUMP_PER_FRAME)
            delta = MAX_JUMP_PER_FRAME;
        if (delta < -MAX_JUMP_PER_FRAME)
            delta = -MAX_JUMP_PER_FRAME;
        rawLinePos = prevLinePos + delta;

        prevLinePos = rawLinePos; // Запам'ятовуємо для наступного кадру
        wasLost = false;          // Лінію знайдено
    }
    else
    {
        // Лінію втрачено — позицію не оновлюємо
        wasLost = true;
    }

    // Буфер кадру ОБОВ'ЯЗКОВО повертаємо — інакше камера скоро перестане
    // видавати кадри (буфер не звільняється).
    esp_camera_fb_return(fb);

    // ========== 5. Обчислення похибки з мертвою зоною ==========
    int error = 0;     // Відхилення лінії від центру
    int direction = 0; // -1 = ліворуч, 0 = прямо, +1 = праворуч

    if (lineFound)
    {
        error = rawLinePos - CENTER_X; // < 0 — лінія ліворуч, > 0 — праворуч

        // Мертва зона: малі похибки ігноруємо, щоб робот не тремтів
        if (abs(error) < DEAD_ZONE)
        {
            error = 0;
            direction = 0;
        }
        else if (error < 0)
        {
            direction = -1; // Лінія ліворуч
        }
        else
        {
            direction = 1; // Лінія праворуч
        }

        lastKnownDirection = direction; // Запам'ятовуємо напрямок
    }
    else
    {
        // Лінію втрачено: їдемо в бік, де вона була востаннє,
        // але повільно (щоб не вилетіти з траси).
        direction = lastKnownDirection;
        error = lastKnownDirection * CENTER_X; // Максимальна похибка в той бік
    }

    // ========== 6. Обчислення швидкостей ==========
    int steer = 0;
    int leftSpeed, rightSpeed;

    if (lineFound)
    {
        // Пропорційне коригування: більша похибка — сильніше підрулюємо.
        // lroundf — правильне округлення (при KP=0.5 та error=1 отримаємо 1, а не 0).
        steer = (int)lroundf(error * KP);
        steer = constrain(steer, -MAX_STEER, MAX_STEER); // Обмеження

        // Диференціал швидкості:
        // Лінія ліворуч (error<0, steer<0) → праве колесо швидше, ліве повільніше
        // → робот повертає ліворуч. І навпаки.
        leftSpeed = BASE_SPEED - steer;
        rightSpeed = BASE_SPEED + steer;

        leftSpeed = constrain(leftSpeed, -127, 127);
        rightSpeed = constrain(rightSpeed, -127, 127);
    }
    else
    {
        // При втраті лінії — повільний пошук у бік останньої лінії
        int searchSpeed = (int)(BASE_SPEED * LOST_SPEED_FACTOR);

        if (lastKnownDirection < 0)
        {
            // Лінія була ліворуч — крутимося ліворуч:
            // праве колесо вперед, ліве стоїть
            leftSpeed = 0;
            rightSpeed = searchSpeed;
        }
        else if (lastKnownDirection > 0)
        {
            // Лінія була праворуч — крутимося праворуч:
            // ліве колесо вперед, праве стоїть
            leftSpeed = searchSpeed;
            rightSpeed = 0;
        }
        else
        {
            // Напрямок невідомий — стоїмо
            leftSpeed = 0;
            rightSpeed = 0;
        }
    }

    // ========== 7. Вивід у серіал ==========
    if (DEBUG_MODE)
    {
        frameCounter++;

        // Якщо лінію не знайдено — пишемо "LOST", інакше — реальний напрямок
        const char *dirStr = lineFound ? directionStr(direction) : "LOST   ";

        // Основний рядок діагностики
        Serial.printf("[%lu] LINE:%3d DIR:%s ERR:%+4d L:%+4d R:%+4d DARK:%3d",
                      frameCounter,
                      lineFound ? rawLinePos : -1, // -1 = лінію не знайдено
                      dirStr,
                      lineFound ? error : 0,
                      leftSpeed,
                      rightSpeed,
                      darkPixelCount);

        // Якщо лінію втрачено — показуємо, у який бік шукаємо
        if (!lineFound)
            Serial.print("  <-- LOST, searching ");
        if (!lineFound && lastKnownDirection < 0)
            Serial.print("LEFT");
        if (!lineFound && lastKnownDirection > 0)
            Serial.print("RIGHT");
        if (!lineFound && lastKnownDirection == 0)
            Serial.print("(stop)");
        Serial.println();

        // Діагностика кожні 20 кадрів: FPS та реальний діапазон яскравості.
        // MinPixel/MaxPixel допомагають підібрати LINE_THRESHOLD:
        //   - якщо MinPixel набагато більший за 100 → лінія не дуже темна,
        //     поріг варто підняти;
        //   - якщо MaxPixel близький до 100 → фон теж темний, поріг варто знизити.
        if (frameCounter % 20 == 0)
        {
            float fps = 0;
            if (lastTime > 0)                      // Перший вимір ігноруємо (час від старту)
                fps = 20000.0f / (now - lastTime); // 20 кадрів / час
            lastTime = now;

            Serial.printf("  [STATS] FPS:%.1f Min:%d Max:%d Thr:%d Compact:%s\n",
                          fps, minPixel, maxPixel, LINE_THRESHOLD,
                          lineFound ? "YES" : "NO");
        }

        // Сирий рядок пікселів (кожен 8-й) — для ручного калібрування порогу.
        // УВАГА: дуже багато виводу, вмикати лише за потреби.
        if (PRINT_RAW_ROW)
        {
            Serial.print("  RAW: ");
            for (int x = 0; x < IMAGE_WIDTH; x += 8)
                Serial.printf("%3d ", rowPtr[x]);
            Serial.println();
        }
    }

    // ========== 8. Відправка на Arduino ==========
    // Формат пакета (6 байт):
    //   [0] 0xAA  — маркер початку 1
    //   [1] 0x55  — маркер початку 2
    //   [2] L_H   — старший байт швидкості лівого колеса
    //   [3] L_L   — молодший байт швидкості лівого колеса
    //   [4] R_H   — старший байт швидкості правого колеса
    //   [5] R_L   — молодший байт швидкості правого колеса
    // Arduino шукає 0xAA 0x55, потім читає 4 байти та відновлює int16_t.
    if (SEND_TO_UART2)
    {
        uint8_t packet[6];
        packet[0] = 0xAA;
        packet[1] = 0x55;
        packet[2] = (leftSpeed >> 8) & 0xFF; // Старший байт
        packet[3] = leftSpeed & 0xFF;        // Молодший байт
        packet[4] = (rightSpeed >> 8) & 0xFF;
        packet[5] = rightSpeed & 0xFF;
        Serial2.write(packet, 6);
    }

    delay(50); // ~20 Гц. Менше — більше навантаження на камеру й більше "шуму";
               // більше — робот стає повільнішим у реакції на повороти.
}