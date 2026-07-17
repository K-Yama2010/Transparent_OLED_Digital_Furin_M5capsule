#include <Arduino.h>
#include <M5UnitGLASS2.h>
#include <M5Unified.h>
#include <esp_now.h>
#include <WiFi.h>

M5UnitGLASS2 display(13, 15, 400000); // SDA, SCL, FREQ  //

#define W 128
#define H 64

M5Canvas canvas(&M5.Display);

uint8_t peerAddress[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; //受取先のMACアドレス

// モノクロ用なのでRGBバッファを1つに統合して処理を軽量化
int16_t buf1[W * H] = {0};
int16_t buf2[W * H] = {0};
int16_t *p1 = buf1, *p2 = buf2;

void updateWave(int16_t* b1, int16_t* b2) {
    int16_t *ptr1 = b1 + W + 1;
    int16_t *ptr2 = b2 + W + 1;
    for (int y = 1; y < H - 1; y++) {
        for (int x = 1; x < W - 1; x++) {
            *ptr2 = ((*(ptr1 - 1) + *(ptr1 + 1) + *(ptr1 - W) + *(ptr1 + W)) >> 1) - *ptr2;
            *ptr2 -= *ptr2 >> 5;
            ptr1++;
            ptr2++;
        }
        ptr1 += 2; // 両端のピクセルをスキップ
        ptr2 += 2;
    }
}

void injectDrop(int px, int py, int force) {
    for (int dy = -1; dy <= 1; dy++) {
        for (int dx = -1; dx <= 1; dx++) {
            int x = px + dx, y = py + dy;
            if (x > 0 && x < W - 1 && y > 0 && y < H - 1) {
                p1[y * W + x] = force;
            }
        }
    }
}

void setup(void){
    auto cfg = M5.config();
    M5.begin(cfg);
    
    WiFi.mode(WIFI_STA);
    if (esp_now_init() == ESP_OK) {
        esp_now_peer_info_t peerInfo = {};
        memcpy(peerInfo.peer_addr, peerAddress, 6);
        peerInfo.channel = 0;  
        peerInfo.encrypt = false;
        esp_now_add_peer(&peerInfo);
    }
    
    // M5Unifiedの管理下に外部ディスプレイを明示的に追加
    M5.addDisplay(display);
    
    M5.Speaker.begin();
    M5.Speaker.setVolume(200);
    
    M5.Displays(0).setBrightness(255);
    M5.Displays(0).setRotation(1);
    M5.Displays(0).setColorDepth(1);
    
    M5.update();
    
    M5.Displays(0).setTextColor(TFT_WHITE, TFT_BLACK);
    M5.Display.clear();
    
    M5.Imu.begin();
    M5.BtnA.setHoldThresh(2000);
    
    // I2C通信の遅延対策: グレースケール変換できる最低限度（8ビット）に設定
    canvas.setColorDepth(8);
    canvas.createSprite(W, H);
}

void loop(void) {
    M5.update();
    
    // IMU（風による揺れ）の検知 (初期値を0.0に設定してゴミを防止)
    float ax = 0.0, ay = 0.0, az = 0.0;
    M5.Imu.getAccel(&ax, &ay, &az);
    float accelSum = sqrt(ax * ax + ay * ay + az * az);
    static float lastAccel = 1.0;
    static uint32_t lastShakeTime = 0;
    
    // 揺れの強さを判定（5段階）
    float diffAccel = abs(accelSum - lastAccel);
    
    // ↓↓↓ IMU感度調整はここからです ↓↓↓
    // diffAccel > 0.025 の「0.025」が波紋が発生する最低の揺れ閾値です。
    // 反応しすぎる場合はこの数値を上げ、反応が鈍い場合は下げてください。
    if (diffAccel > 0.025 && millis() - lastShakeTime > 200) {
        int force = 4000;
        // 以下の数値（2.0, 1.5, 1.0, 0.6）で揺れの激しさに応じた波紋の強さを決めています。
        if (diffAccel > 2.0) force = 20000;
        else if (diffAccel > 1.5) force = 16000;
        else if (diffAccel > 1.0) force = 12000;
        else if (diffAccel > 0.6) force = 8000;
        
        injectDrop(random(10, W - 10), random(10, H - 10), force);
        
        uint8_t data = 1;
        esp_now_send(peerAddress, &data, sizeof(data));
        
        lastShakeTime = millis();
    }
    // ↑↑↑ IMU感度調整はここまで ↑↑↑
    
    lastAccel = accelSum;

    // 波紋の計算（伝播速度向上のため1フレーム内で2ステップ実行）
    updateWave(p1, p2);
    int16_t *temp = p1; p1 = p2; p2 = temp;
    updateWave(p1, p2);
    temp = p1; p1 = p2; p2 = temp;

    canvas.clear();
    int16_t *rPtr = p2;
    
    // M5Unifiedのグレースケール変換に任せるため、0〜255の階調を計算
    for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {
            int val = *rPtr++;
            if (val < 0) val = -val;
            if (val > 255) val = 255;
            
            // 階調をそのまま描画（M5Unifiedが転送時にディザリング処理してくれる）
            canvas.drawPixel(x, y, canvas.color888(val, val, val));
        }
    }
    
    // 透過OLEDへ転送
    canvas.pushSprite(0, 0);
    delay(1);

     if (M5.BtnA.wasHold())
     {
     //  M5.Speaker.tone(1200, 100);
       delay(10);
       M5.Power.powerOff();
     }

}