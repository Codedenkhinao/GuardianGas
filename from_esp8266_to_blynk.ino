#define BLYNK_TEMPLATE_ID "TMPL62sdjazhx"     // ID template from Blynk
#define BLYNK_TEMPLATE_NAME "CanhBaoKhiGas"  // Tên template từ Blynk
#define BLYNK_AUTH_TOKEN "QNu6BX42q9tJU9s8aC3s0ZbWHy-qpEFf" // Token từ Blynk

#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <SoftwareSerial.h> // Thư viện cho UART giả lập (SoftwareSerial)

// WiFi credentials
const char* ssid = "GoBi Lau";          // Tên WiFi của bạn
const char* password = "gobicamon";                    // Mật khẩu WiFi của bạn

// Khai báo UART giữa ESP8266 và STM32
#define STM32_RX D5  // Chân RX của ESP8266 (nhận dữ liệu từ STM32)
#define STM32_TX D6  // Chân TX của ESP8266 (gửi dữ liệu tới STM32)
SoftwareSerial STM32Serial(STM32_RX, STM32_TX); // Tạo giao tiếp UART với STM32

BlynkTimer timer; // Timer của Blynk để gửi dữ liệu định kỳ

float gasValue = 0.0; // Biến lưu giá trị khí gas

// Hàm gửi dữ liệu khí gas lên Blynk
void sendGasDataToBlynk() {
  Blynk.virtualWrite(V0, gasValue); // Gửi giá trị khí gas lên Virtual Pin V0
  Serial.println("Sent gas value to Blynk: " + String(gasValue)); // In ra debug
}

void setup() {
  // Khởi tạo UART và Serial debug
  Serial.begin(115200);        // Serial debug
  STM32Serial.begin(9600);     // UART giao tiếp với STM32

  // Kết nối WiFi
  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi");

  // Kết nối Blynk
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, password);
  Serial.println("Connected to Blynk");

  // Đặt lịch gửi dữ liệu lên Blynk mỗi 1 giây
  timer.setInterval(1000L, sendGasDataToBlynk);
}

void loop() {
  Blynk.run(); // Chạy tiến trình Blynk
  timer.run(); // Chạy tiến trình Timer của Blynk

  // Đọc dữ liệu từ STM32
  if (STM32Serial.available()) {
    String data = STM32Serial.readStringUntil('\n'); // Đọc chuỗi dữ liệu đến khi gặp ký tự '\n'
    gasValue = data.toFloat(); // Chuyển đổi chuỗi sang kiểu float
    Serial.println("Received gas value from STM32: " + String(gasValue)); // Hiển thị giá trị nhận được
  }
}
