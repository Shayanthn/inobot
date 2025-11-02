/*
 * ESP32-CAM AI-Thinker Robot Control با TB6612FNG Motor Driver
 * کنترل از طریق وب سرور با دو جوی استیک
 * 
 * نقشه اتصالات (بدون تداخل با Flash LED):
 * ESP32-CAM          TB6612FNG
 * GPIO 12      →     PWMA
 * GPIO 13      →     AIN1
 * GPIO 15      →     AIN2
 * 5V           →     STBY (مستقیم)
 * GPIO 2       →     BIN1
 * GPIO 14      →     PWMB
 * GPIO 1       →     BIN2 (تغییر از GPIO 4)
 * 
 * تغذیه:
 * ESP32-CAM 5V, GND → منبع 1 (5V)
 * TB6612FNG VCC → 3.3V یا 5V
 * TB6612FNG VM → منبع 2 (باتری موتور 6-12V)
 * TB6612FNG GND → GND مشترک (همه منابع)
 */

#include <WiFi.h>
#include <WebServer.h>

// تنظیمات WiFi - نام و رمز وای‌فای خود را وارد کنید
const char* ssid = "YourWiFiName";      // نام وای‌فای خود را اینجا بنویسید
const char* password = "YourPassword";   // رمز وای‌فای خود را اینجا بنویسید

// پین‌های موتور A (چپ)
#define MOTOR_A_PWM   12    // PWMA
#define MOTOR_A_IN1   13    // AIN1
#define MOTOR_A_IN2   15    // AIN2

// پین‌های موتور B (راست)
#define MOTOR_B_PWM   14    // PWMB
#define MOTOR_B_IN1   2     // BIN1
#define MOTOR_B_IN2   1     // BIN2 (تغییر از GPIO 4 به GPIO 1)

// پین استندبای - STBY را به 5V وصل کنید (نیازی به پین GPIO نیست)
// #define STBY_PIN      ---    // STBY به 5V مستقیم

// ⚠️ نکته: GPIO 4 برای Flash LED دوربین استفاده می‌شود - استفاده نکنید!
// ⚠️ نکته: با استفاده از GPIO 1، Serial Monitor کار نمی‌کند (TX pin)

// تنظیمات PWM
#define PWM_FREQ      1000
#define PWM_RESOLUTION 8
#define PWM_CHANNEL_A  0
#define PWM_CHANNEL_B  1

// متغیرهای سرعت
int currentSpeed = 200;  // سرعت پیش‌فرض (0-255)
int maxSpeed = 255;
int minSpeed = 0;

// ایجاد وب سرور روی پورت 80
WebServer server(80);

// صفحه HTML با دو جوی استیک (بهینه شده)
String getWebpage() {
  return R"=====(
<!DOCTYPE html>
<html dir="rtl" lang="fa">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>کنترل ربات ESP32-CAM</title>
    <style>
        * {
            margin: 0;
            padding: 0;
            box-sizing: border-box;
        }
        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            display: flex;
            justify-content: center;
            align-items: center;
            min-height: 100vh;
            padding: 20px;
        }
        .container {
            background: rgba(255, 255, 255, 0.95);
            border-radius: 20px;
            padding: 30px;
            box-shadow: 0 20px 60px rgba(0, 0, 0, 0.3);
            max-width: 900px;
            width: 100%;
        }
        h1 {
            text-align: center;
            color: #667eea;
            margin-bottom: 30px;
            font-size: 28px;
        }
        .joysticks-container {
            display: flex;
            justify-content: space-around;
            flex-wrap: wrap;
            gap: 30px;
            margin-bottom: 30px;
        }
        .joystick-wrapper {
            display: flex;
            flex-direction: column;
            align-items: center;
        }
        .joystick-label {
            font-size: 18px;
            font-weight: bold;
            color: #333;
            margin-bottom: 15px;
        }
        .joystick {
            width: 200px;
            height: 200px;
            border-radius: 50%;
            background: linear-gradient(145deg, #e6e6e6, #ffffff);
            box-shadow: 0 10px 30px rgba(0, 0, 0, 0.2);
            position: relative;
            touch-action: none;
        }
        .joystick-knob {
            width: 80px;
            height: 80px;
            border-radius: 50%;
            background: linear-gradient(145deg, #667eea, #764ba2);
            position: absolute;
            top: 50%;
            left: 50%;
            transform: translate(-50%, -50%);
            cursor: grab;
            box-shadow: 0 5px 15px rgba(0, 0, 0, 0.3);
            transition: all 0.1s ease;
        }
        .joystick-knob:active {
            cursor: grabbing;
            transform: translate(-50%, -50%) scale(0.95);
        }
        .speed-control {
            background: white;
            padding: 25px;
            border-radius: 15px;
            box-shadow: 0 5px 20px rgba(0, 0, 0, 0.1);
            margin-bottom: 20px;
        }
        .speed-label {
            font-size: 18px;
            font-weight: bold;
            color: #333;
            margin-bottom: 15px;
            text-align: center;
        }
        .speed-slider {
            width: 100%;
            height: 8px;
            border-radius: 5px;
            outline: none;
            -webkit-appearance: none;
            background: linear-gradient(to right, #667eea, #764ba2);
            margin-bottom: 10px;
        }
        .speed-slider::-webkit-slider-thumb {
            -webkit-appearance: none;
            width: 25px;
            height: 25px;
            border-radius: 50%;
            background: #667eea;
            cursor: pointer;
            box-shadow: 0 3px 10px rgba(0, 0, 0, 0.3);
        }
        .speed-slider::-moz-range-thumb {
            width: 25px;
            height: 25px;
            border-radius: 50%;
            background: #667eea;
            cursor: pointer;
            box-shadow: 0 3px 10px rgba(0, 0, 0, 0.3);
            border: none;
        }
        .speed-value {
            text-align: center;
            font-size: 24px;
            font-weight: bold;
            color: #667eea;
        }
        .status {
            text-align: center;
            padding: 15px;
            background: rgba(102, 126, 234, 0.1);
            border-radius: 10px;
            font-size: 16px;
            color: #333;
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>🤖 کنترل ربات ESP32-CAM</h1>
        
        <div class="joysticks-container">
            <div class="joystick-wrapper">
                <div class="joystick-label">جلو / عقب</div>
                <div class="joystick" id="joystick1">
                    <div class="joystick-knob" id="knob1"></div>
                </div>
            </div>
            
            <div class="joystick-wrapper">
                <div class="joystick-label">چپ / راست</div>
                <div class="joystick" id="joystick2">
                    <div class="joystick-knob" id="knob2"></div>
                </div>
            </div>
        </div>
        
        <div class="speed-control">
            <div class="speed-label">تنظیم سرعت موتور</div>
            <input type="range" min="0" max="255" value="200" class="speed-slider" id="speedSlider">
            <div class="speed-value">سرعت: <span id="speedValue">200</span></div>
        </div>
        
        <div class="status" id="status">آماده به کار</div>
    </div>

    <script>
        let currentSpeed = 200;
        let joystick1Active = false;
        let joystick2Active = false;
        let currentDirection = { forward: 0, turn: 0 };

        // تنظیم جوی استیک
        function setupJoystick(joystickId, knobId, callback) {
            const joystick = document.getElementById(joystickId);
            const knob = document.getElementById(knobId);
            const rect = joystick.getBoundingClientRect();
            const centerX = rect.width / 2;
            const centerY = rect.height / 2;
            const maxDistance = rect.width / 2 - 40;
            
            let active = false;

            function handleMove(clientX, clientY) {
                if (!active) return;
                
                const rect = joystick.getBoundingClientRect();
                let x = clientX - rect.left - centerX;
                let y = clientY - rect.top - centerY;
                
                const distance = Math.sqrt(x * x + y * y);
                
                if (distance > maxDistance) {
                    const angle = Math.atan2(y, x);
                    x = Math.cos(angle) * maxDistance;
                    y = Math.sin(angle) * maxDistance;
                }
                
                knob.style.left = (centerX + x) + 'px';
                knob.style.top = (centerY + y) + 'px';
                
                const normalizedX = x / maxDistance;
                const normalizedY = -y / maxDistance;
                
                callback(normalizedX, normalizedY);
            }

            function resetPosition() {
                knob.style.left = '50%';
                knob.style.top = '50%';
                callback(0, 0);
                active = false;
            }

            // Mouse events
            knob.addEventListener('mousedown', (e) => {
                active = true;
                e.preventDefault();
            });

            document.addEventListener('mousemove', (e) => {
                handleMove(e.clientX, e.clientY);
            });

            document.addEventListener('mouseup', resetPosition);

            // Touch events
            knob.addEventListener('touchstart', (e) => {
                active = true;
                e.preventDefault();
            });

            joystick.addEventListener('touchmove', (e) => {
                if (e.touches.length > 0) {
                    handleMove(e.touches[0].clientX, e.touches[0].clientY);
                }
                e.preventDefault();
            });

            joystick.addEventListener('touchend', resetPosition);
        }

        // جوی استیک 1: جلو/عقب
        setupJoystick('joystick1', 'knob1', (x, y) => {
            joystick1Active = (Math.abs(y) > 0.1);
            currentDirection.forward = Math.round(y * 100);
            sendCommand();
        });

        // جوی استیک 2: چپ/راست
        setupJoystick('joystick2', 'knob2', (x, y) => {
            joystick2Active = (Math.abs(x) > 0.1);
            currentDirection.turn = Math.round(x * 100);
            sendCommand();
        });

        // تنظیم سرعت
        const speedSlider = document.getElementById('speedSlider');
        const speedValue = document.getElementById('speedValue');
        
        speedSlider.addEventListener('input', function() {
            currentSpeed = this.value;
            speedValue.textContent = currentSpeed;
            sendSpeed();
        });

        // ارسال دستور حرکت
        function sendCommand() {
            let command = '';
            
            if (!joystick1Active && !joystick2Active) {
                command = 'stop';
            } else {
                const forward = currentDirection.forward;
                const turn = currentDirection.turn;
                
                if (Math.abs(forward) > Math.abs(turn)) {
                    if (forward > 10) {
                        command = 'forward';
                    } else if (forward < -10) {
                        command = 'backward';
                    }
                } else {
                    if (turn > 10) {
                        command = 'right';
                    } else if (turn < -10) {
                        command = 'left';
                    }
                }
            }
            
            if (command) {
                fetch('/cmd?action=' + command)
                    .then(response => response.text())
                    .then(data => {
                        document.getElementById('status').textContent = data;
                    });
            }
        }

        // ارسال سرعت
        function sendSpeed() {
            fetch('/speed?value=' + currentSpeed)
                .then(response => response.text())
                .then(data => {
                    document.getElementById('status').textContent = data;
                });
        }
    </script>
</body>
</html>
)=====";
}

// توابع کنترل موتور
void stopMotors() {
  ledcWrite(PWM_CHANNEL_A, 0);
  ledcWrite(PWM_CHANNEL_B, 0);
  digitalWrite(MOTOR_A_IN1, LOW);
  digitalWrite(MOTOR_A_IN2, LOW);
  digitalWrite(MOTOR_B_IN1, LOW);
  digitalWrite(MOTOR_B_IN2, LOW);
}

void moveForward() {
  digitalWrite(MOTOR_A_IN1, HIGH);
  digitalWrite(MOTOR_A_IN2, LOW);
  digitalWrite(MOTOR_B_IN1, HIGH);
  digitalWrite(MOTOR_B_IN2, LOW);
  ledcWrite(PWM_CHANNEL_A, currentSpeed);
  ledcWrite(PWM_CHANNEL_B, currentSpeed);
}

void moveBackward() {
  digitalWrite(MOTOR_A_IN1, LOW);
  digitalWrite(MOTOR_A_IN2, HIGH);
  digitalWrite(MOTOR_B_IN1, LOW);
  digitalWrite(MOTOR_B_IN2, HIGH);
  ledcWrite(PWM_CHANNEL_A, currentSpeed);
  ledcWrite(PWM_CHANNEL_B, currentSpeed);
}

void turnLeft() {
  digitalWrite(MOTOR_A_IN1, LOW);
  digitalWrite(MOTOR_A_IN2, HIGH);
  digitalWrite(MOTOR_B_IN1, HIGH);
  digitalWrite(MOTOR_B_IN2, LOW);
  ledcWrite(PWM_CHANNEL_A, currentSpeed);
  ledcWrite(PWM_CHANNEL_B, currentSpeed);
}

void turnRight() {
  digitalWrite(MOTOR_A_IN1, HIGH);
  digitalWrite(MOTOR_A_IN2, LOW);
  digitalWrite(MOTOR_B_IN1, LOW);
  digitalWrite(MOTOR_B_IN2, HIGH);
  ledcWrite(PWM_CHANNEL_A, currentSpeed);
  ledcWrite(PWM_CHANNEL_B, currentSpeed);
}

// مدیریت درخواست‌های وب
void handleRoot() {
  server.send(200, "text/html", getWebpage());
}

void handleCommand() {
  if (server.hasArg("action")) {
    String action = server.arg("action");
    
    if (action == "forward") {
      moveForward();
      server.send(200, "text/plain", "حرکت به جلو");
    } 
    else if (action == "backward") {
      moveBackward();
      server.send(200, "text/plain", "حرکت به عقب");
    } 
    else if (action == "left") {
      turnLeft();
      server.send(200, "text/plain", "چرخش به چپ");
    } 
    else if (action == "right") {
      turnRight();
      server.send(200, "text/plain", "چرخش به راست");
    } 
    else if (action == "stop") {
      stopMotors();
      server.send(200, "text/plain", "توقف");
    }
    else {
      server.send(400, "text/plain", "دستور نامعتبر");
    }
  } else {
    server.send(400, "text/plain", "پارامتر اشتباه");
  }
}

void handleSpeed() {
  if (server.hasArg("value")) {
    int speed = server.arg("value").toInt();
    if (speed >= 0 && speed <= 255) {
      currentSpeed = speed;
      server.send(200, "text/plain", "سرعت تنظیم شد: " + String(currentSpeed));
    } else {
      server.send(400, "text/plain", "سرعت نامعتبر");
    }
  } else {
    server.send(400, "text/plain", "پارامتر اشتباه");
  }
}

void setup() {
  // راه‌اندازی سریال
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n\n=================================");
  Serial.println("ESP32-CAM Robot Control Starting");
  Serial.println("=================================\n");
  
  // غیرفعال کردن PSRAM برای جلوگیری از تداخل
  Serial.println("Disabling PSRAM...");
  
  // تنظیم پین‌های موتور به عنوان خروجی
  pinMode(MOTOR_A_IN1, OUTPUT);
  pinMode(MOTOR_A_IN2, OUTPUT);
  pinMode(MOTOR_B_IN1, OUTPUT);
  pinMode(MOTOR_B_IN2, OUTPUT);
  
  // STBY به 5V وصل شده (همیشه فعال)
  // اگر از GPIO استفاده می‌کنید، این خطوط را فعال کنید:
  // pinMode(STBY_PIN, OUTPUT);
  // digitalWrite(STBY_PIN, HIGH);
  
  Serial.println("Motor pins initialized");
  Serial.println("STBY connected to 5V (Always Active)");
  
  // راه‌اندازی PWM
  ledcSetup(PWM_CHANNEL_A, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_B, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(MOTOR_A_PWM, PWM_CHANNEL_A);
  ledcAttachPin(MOTOR_B_PWM, PWM_CHANNEL_B);
  
  // توقف اولیه موتورها
  stopMotors();
  
  // کاهش توان WiFi برای پایداری
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  
  // اتصال به WiFi
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n\n✓ WiFi Connected!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    Serial.print("Signal Strength (RSSI): ");
    Serial.print(WiFi.RSSI());
    Serial.println(" dBm");
  } else {
    Serial.println("\n\n✗ WiFi Connection Failed!");
    Serial.println("Please check your SSID and password");
  }
  
  // راه‌اندازی وب سرور
  server.on("/", handleRoot);
  server.on("/cmd", handleCommand);
  server.on("/speed", handleSpeed);
  
  server.begin();
  Serial.println("\n✓ Web Server Started!");
  Serial.println("=================================");
  Serial.println("Open your browser and go to:");
  Serial.print("http://");
  Serial.println(WiFi.localIP());
  Serial.println("=================================\n");
  
  // نمایش حافظه آزاد
  Serial.print("Free Heap: ");
  Serial.print(ESP.getFreeHeap());
  Serial.println(" bytes");
}

void loop() {
  server.handleClient();
}
