#include <U8g2lib.h>
#include <ESP32Servo.h>
#include <SPI.h>
#include <Keypad.h>
#include <MFRC522.h>  // RFID Library
#include <EEPROM.h>   // EEPROM Library
#include <WiFi.h>     // Untuk fitur WiFi dan NTP
#include <time.h>     // Untuk fitur waktu (NTP)


// --- RFID & EEPROM DEFINES ---
#define SS_PIN 5
#define RST_PIN 2
MFRC522 rfid(SS_PIN, RST_PIN);


#define EEPROM_SIZE 512  // Sesuaikan ukuran EEPROM sesuai kebutuhan
// Batasan alamat untuk UID yang disimpan
const int alamat_UID_start = 0;
const int alamat_UID_end = 100;  // Contoh: alokasi 100 byte untuk UID (10 UID @ 10 byte/UID)
const int alamat_UID_count = 101;


// --- OLED DEFINES & BITMAPS ---
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0);


// BITMAPS (Pastikan bitmap Anda sama seperti yang Anda berikan)
const unsigned char add_card[] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x7f, 0xfa, 0xff, 0xfd, 0xfe, 0x0d, 0xc3, 0xfd, 0xc3, 0xfd, 0xc2, 0x0d,
  0xc3, 0xfd, 0xc3, 0xfd, 0xfe, 0x0d, 0xff, 0xfd, 0xff, 0xfd, 0x7f, 0xfa, 0x00, 0x00, 0x00, 0x00
};
const unsigned char history[] PROGMEM = {
  0x7f, 0xfe, 0x20, 0x04, 0x20, 0x04, 0x20, 0x04, 0x1, 0x08, 0x17, 0xe8, 0x0b, 0xd0, 0x05, 0xa0,
  0x05, 0x0a, 0x09, 0x90, 0x11, 0x88, 0x13, 0xc8, 0x2f, 0xf4, 0x2f, 0xf4, 0x20, 0x04, 0x7f, 0xfe
};
const unsigned char lock[] PROGMEM = {
  0x00, 0x00, 0x00, 0xf8, 0x01, 0xfc, 0x03, 0xe6, 0x03, 0xc3, 0x03, 0xc3, 0x03, 0xe7, 0x03, 0xff,
  0x07, 0xff, 0x0d, 0xfe, 0x1b, 0xbc, 0x37, 0x80, 0x6c, 0x00, 0xdc, 0x00, 0xb0, 0x00, 0xf0, 0x00
};
const unsigned char remove_card[] PROGMEM = {
  0xaf, 0xff, 0xd7, 0xff, 0x00, 0x00, 0x55, 0x56, 0x68, 0x8a, 0x55, 0x56, 0x62, 0x22, 0x35, 0x54,
  0x28, 0x8c, 0x35, 0x54, 0x22, 0x24, 0x15, 0x58, 0x18, 0x88, 0x15, 0x58, 0x00, 0x00, 0x17, 0xf8
};
const unsigned char outline[] PROGMEM = {
  0x1f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0x20,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x40, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x40, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x40, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x40, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x03, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x03, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x03, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x03, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x03, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03,
  0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x40,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x40, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x3f, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x1f, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc
};
const unsigned char handle[] PROGMEM = {
  0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07
};
const unsigned char scrollbar[] PROGMEM = {
  0x00, 0x02, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00,
  0x00, 0x02, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00,
  0x00, 0x02, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00,
  0x00, 0x02, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00
};


const int num_menu_items = 4;
const char* menu_item[] = {
  "Buka Kunci",
  "Tambah RFID",
  "Hapus RFID",
  "Riwayat Akses",
};


const unsigned char* bitmap_icons[4] = {
  lock,
  add_card,
  remove_card,
  history
};


volatile int menu_sel = 0;  // Use volatile for variables modified in ISRs or outside main loop


// --- SERVO DEFINES ---
Servo myServo;
#define servo_pin 15  // Pin 15 adalah pin output PWM yang bagus untuk ESP32


// --- KEYPAD DEFINES ---
const byte ROWS = 4;
const byte COLS = 4;
char keys[ROWS][COLS] = {
  { '1', '2', '3', 'A' },
  { '4', '5', '6', 'B' },
  { '7', '8', '9', 'C' },
  { '*', '0', '#', 'D' }
};
byte rowPins[ROWS] = { 13, 12, 14, 27 };
byte colPins[COLS] = { 26, 25, 33, 32 };
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);
char key;
unsigned long start;


// --- GLOBAL VARIABLES ---
String inputPassword = "";
const String correctPassword = "1234";  // Password utama
const String adminPassword = "7777";    // Password admin
int STATE = 0;                          // Menggunakan 'STATE' untuk menghindari konflik dengan 'state' dari Keypad library


String current_uid_card = "";  // UID dari kartu yang baru saja di-tap


// --- FUNCTION PROTOTYPES ---
void cardTap();
bool isUidRegistered(String uid);                    // Cek apakah UID sudah terdaftar
void saveUid(String uid);                            // Simpan UID baru ke EEPROM
void deleteUid(String uid);                          // Hapus UID dari EEPROM
void displayString(int x, int y, const char* text);  // Helper function untuk menampilkan teks di OLED
void displayString(int x, int y, String text);       // Overload untuk String


void display_menu();  // Untuk menampilkan menu admin


void state_rfid_scan();       // State 0: Scan RFID
void state_input_password();  // State 1: Input Password
void state_open_lock();       // State 2: Open Lock
void state_admin_menu();      // State 3: Admin Menu
void state_add_rfid();        // State 4: Add RFID
void state_delete_rfid();     // State 5: Delete RFID
void state_view_history();    // State 6: View History (Placeholder)


// --- Fitur Waktu (NTP) ---
void gettime();
char hariStr[15];
char tanggalStr[20];
char waktuStr[10];




// --- SETUP ---
void setup() {
  Serial.begin(115200);


  EEPROM.begin(EEPROM_SIZE);
  Serial.println("EEPROM Ready.");


  SPI.begin();
  rfid.PCD_Init();
  Serial.println("RFID Ready.");


  u8g2.begin();
  u8g2.setColorIndex(1);
  Serial.println("OLED Ready.");


  myServo.attach(servo_pin);
  myServo.write(45);  // Set posisi awal pintu tertutup (misal 45 derajat)
  Serial.println("Servo Ready.");


  // Konfigurasi WiFi dan NTP
  Serial.println("Menghubungkan ke WiFi...");
  // Ganti "nama_wifi_anda" dan "password_wifi_anda" dengan kredensial WiFi Anda
  WiFi.begin("Apelu", "pecaron123");
  int wifi_attempts = 0;
  while (WiFi.status() != WL_CONNECTED && wifi_attempts < 20) {  // Coba 20 kali (10 detik)
    delay(500);
    Serial.print(".");
    wifi_attempts++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi Terhubung!");
    configTime(7 * 3600, 0, "pool.ntp.org");
    Serial.println("NTP Disinkronkan.");
  } else {
    Serial.println("\nGagal Terhubung ke WiFi/NTP. Waktu mungkin tidak akurat.");
    displayString(0, 0, "WiFi Gagal!");
    displayString(0, 20, "Waktu Tidak\nAkura!");
    delay(2000);
  }


  displayString(0, 20, "Sistem Siap!");
  delay(1000);  // Tampilkan sebentar
}


// --- MAIN LOOP ---
void loop() {
  key = keypad.getKey();
  start = millis();
  switch (STATE) {
    case 0: state_rfid_scan(); break;
    case 1: state_input_password(); break;
    case 2: state_open_lock(); break;
    case 3: state_admin_menu(); break;
    case 4: state_add_rfid(); break;
    case 5: state_delete_rfid(); break;
    case 6: state_view_history(); break;
    default: STATE = 0; break;  // Kembali ke state awal jika state tidak valid
  }
}


// --- STATE FUNCTIONS ---


// State 0: Scan RFID
void state_rfid_scan() {
  displayString(0, 20, "Tempelkan Kartu RFID");


  if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) {
    cardTap();  // Baca UID kartu yang baru


    Serial.print("UID Kartu Terbaca: ");
    Serial.println(current_uid_card);


    if (isUidRegistered(current_uid_card) || current_uid_card == "93C28E1D") {
      displayString(0, 20, "Kartu Terdaftar!\nMasukkan Password.");
      delay(1000);
      STATE = 1;  // Lanjut ke state input password
    } else {
      displayString(0, 20, "Kartu Tidak Dikenal.\nCoba Lagi.");
      delay(2000);
      rfid.PICC_HaltA();  // Hentikan pembacaan setelah proses selesai
      rfid.PCD_StopCrypto1();
    }
  }
  // Tidak perlu delay di sini agar loop RFID bisa terus berjalan
}


// State 1: Input Password
void state_input_password() {
  inputPassword = "";  // Kosongkan password setiap kali masuk state ini
  start = millis();    // Timer lokal untuk timeout 60 detik


  // Tampilan awal input password
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_t0_13b_tr);
  u8g2.drawStr(0, 14, "Masukkan Password");
  u8g2.drawStr(0, 30, "# RESET | * SUBMIT");
  u8g2.drawStr(0, 46, "");  // Kosongkan area password awal
  u8g2.sendBuffer();


  while (millis() - start < 60000) {  // Loop selama 60 detik
    key = keypad.getKey();            // Baca tombol dari keypad


    if (key) {           // Jika ada tombol ditekan
      start = millis();  // Reset timer setiap kali ada input


      if (key == '#') {  // Tombol RESET
        inputPassword = "";
        displayString(0, 20, "Input Direset.\nSilahkan Ulangi.");
        delay(1000);
        STATE = 0;  // Kembali ke state scan RFID
        return;
      } else if (key == '*') {  // Tombol SUBMIT
        if (inputPassword == correctPassword) {
          displayString(0, 20, "Password Benar!");
          delay(1000);
          STATE = 2;              // Ke state buka kunci
          current_uid_card = "";  // Kosongkan UID setelah penggunaan
          inputPassword = "";
          return;
        } else if (inputPassword == adminPassword) {  // Password Admin
          displayString(0, 20, "Password Admin!");
          delay(1000);
          STATE = 3;              // Ke state menu admin
          current_uid_card = "";  // Kosongkan UID setelah penggunaan
          inputPassword = "";
          return;
        } else {
          displayString(0, 20, ("Password Salah!\n" + inputPassword));
          delay(1500);
          current_uid_card = "";  // Kosongkan UID setelah penggunaan
          inputPassword = "";
          STATE = 0;  // Kembali ke state scan RFID
          return;
        }
      } else if (key >= '0' && key <= '9') {  // Input angka
        inputPassword += key;
        // Perbarui tampilan password di OLED
        u8g2.setFont(u8g2_font_t0_13b_tr);
        u8g2.setDrawColor(0);  // Hitam untuk menghapus
        u8g2.drawBox(0, 46 - u8g2.getFontAscent(), 128, u8g2.getFontAscent() - u8g2.getFontDescent() + 2);
        u8g2.setDrawColor(1);  // Putih untuk menulis
        u8g2.drawStr(0, 46, inputPassword.c_str());
        u8g2.sendBuffer();
      }
    }
  }
  // Jika loop keluar karena timeout
  displayString(0, 20, "TIMEOUT!\nSilahkan Ulangi");
  delay(1000);
  STATE = 0;  // Kembali ke state scan RFID
}


// State 2: Open Lock
void state_open_lock() {
  gettime();  // Dapatkan waktu terbaru dari NTP


  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_t0_13b_tr);
  u8g2.drawStr(0, 14, "Kunci Dibuka");
  u8g2.drawStr(0, 30, hariStr);
  u8g2.drawStr(0, 46, tanggalStr);
  u8g2.drawStr(0, 62, waktuStr);
  u8g2.sendBuffer();


  // Animasi membuka kunci
  Serial.println("Pintu dibuka..");
  for (int pos = 45; pos >= 0; pos--) {  // Gerak servo ke posisi terbuka (misal 0 derajat)
    myServo.write(pos);
    delay(15);
  }
  delay(5000);  // Pintu terbuka selama 5 detik


  // Animasi menutup kunci
  Serial.println("Pintu ditutup..");
  for (int pos = 0; pos <= 45; pos++) {  // Gerak servo ke posisi tertutup (misal 45 derajat)
    myServo.write(pos);
    delay(15);
  }


  // Pesan setelah kunci tertutup
  displayString(0, 14, "Kunci Tertutup.");
  delay(1000);
  STATE = 0;  // Kembali ke state scan RFID
}


// State 3: Admin Menu
void state_admin_menu() {
  start = millis();  // Timer lokal untuk timeout 60 detik
  display_menu();    // Tampilkan menu admin pertama kali


  while (millis() - start < 60000) {
    key = keypad.getKey();
    if (key) {
      start = millis();  // Reset timer setiap ada input


      if (key == '*') {  // Select menu
        switch (menu_sel) {
          case 0: STATE = 2; break;   // Buka Kunci (langsung buka, tidak perlu password lagi)
          case 1: STATE = 4; break;   // Tambah RFID
          case 2: STATE = 5; break;   // Hapus RFID
          case 3: STATE = 6; break;   // Riwayat Akses
          default: STATE = 0; break;  // Kembali ke state awal jika tidak valid
        }
        return;                                                       // Keluar dari loop admin menu setelah pilihan
      } else if (key == 'B') {                                        // 'B' untuk navigasi ke atas (menu sebelumnya)
        menu_sel = (menu_sel - 1 + num_menu_items) % num_menu_items;  // Pastikan menu_sel tetap di range
        display_menu();                                               // Perbarui tampilan setelah navigasi
      } else if (key == 'C') {                                        // 'C' untuk navigasi ke bawah (menu selanjutnya)
        menu_sel = (menu_sel + 1) % num_menu_items;                   // Pastikan menu_sel tetap di range
        display_menu();                                               // Perbarui tampilan setelah navigasi
      }
    }
  }
  // Jika loop keluar karena timeout
  displayString(0, 20, "TIMEOUT!\nKembali ke Utama");
  delay(1000);
  STATE = 0;  // Kembali ke state scan RFID
}


// State 4: Tambah RFID
void state_add_rfid() {
  displayString(0, 20, "Tempelkan Kartu RFID\nuntuk Ditambahkan.");
  displayString(0, 40, "Tekan # Kembali");


  start = millis();  // Timer lokal untuk timeout
  bool adding_card = true;


  while (adding_card && (millis() - start < 60000)) {  // 60 detik timeout
    key = keypad.getKey();
    if (key == '#') {
      displayString(0, 20, "Pembatalan.\nKembali ke Admin Menu.");
      delay(1000);
      STATE = 3;  // Kembali ke menu admin
      return;
    }


    if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) {
      cardTap();  // Baca UID


      if (isUidRegistered(current_uid_card)) {
        displayString(0, 20, "UID Sudah Terdaftar!\nCoba Kartu Lain.");
        delay(2000);
      } else {
        saveUid(current_uid_card);  // Simpan UID baru (fungsi ini juga update OLED)
        delay(2000);                // Beri waktu pengguna membaca pesan konfirmasi
        adding_card = false;        // Berhenti menambahkan setelah berhasil
      }
      rfid.PICC_HaltA();
      rfid.PCD_StopCrypto1();
    }
  }
  // Jika timeout
  if (adding_card) {
    displayString(0, 20, "TIMEOUT!\nKembali ke Admin Menu.");
    delay(1000);
  }
  STATE = 3;  // Kembali ke menu admin setelah selesai atau timeout
}


// State 5: Hapus RFID
void state_delete_rfid() {
  inputPassword = "";  // Digunakan kembali untuk input UID yang akan dihapus
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_t0_13b_tr);
  u8g2.drawStr(0, 14, "Ket. UID Hapus:");  // Pesan lebih singkat
  u8g2.setFont(u8g2_font_t0_11_tr);
  u8g2.drawStr(0, 30, "# Kembali | * Submit");
  u8g2.setFont(u8g2_font_t0_13b_tr);
  u8g2.drawStr(0, 46, "");  // Area untuk UID input
  u8g2.sendBuffer();


  start = millis();  // Timer lokal untuk timeout


  while (millis() - start < 60000) {
    key = keypad.getKey();
    if (key) {
      start = millis();  // Reset timer


      if (key == '#') {
        displayString(0, 20, "Pembatalan.\nKembali ke Admin Menu.");
        delay(1000);
        STATE = 3;
        return;
      } else if (key == '*') {
        if (isUidRegistered(inputPassword)) {  // Check if input UID exists
          deleteUid(inputPassword);            // Fungsi ini juga update OLED
          delay(2000);
        } else {
          displayString(0, 20, "UID Tidak Ditemukan:\n" + inputPassword);
          delay(2000);
        }
        inputPassword = "";
        STATE = 3;  // Kembali ke menu admin
        return;
      } else if (key >= '0' && key <= '9' || (key >= 'A' && key <= 'D')) {  // Izinkan angka dan huruf A-D
        inputPassword += key;
        // Update display for input UID
        u8g2.setFont(u8g2_font_t0_13b_tr);
        u8g2.setDrawColor(0);  // Clear area
        u8g2.drawBox(0, 46 - u8g2.getFontAscent(), 128, u8g2.getFontAscent() - u8g2.getFontDescent() + 2);
        u8g2.setDrawColor(1);
        u8g2.drawStr(0, 46, inputPassword.c_str());
        u8g2.sendBuffer();
      }
    }
  }
  displayString(0, 20, "TIMEOUT!\nKembali ke Admin Menu.");
  delay(1000);
  STATE = 3;  // Kembali ke menu admin
}


// State 6: Riwayat Akses (Placeholder)
void state_view_history() {
  displayString(0, 20, "Memuat Riwayat...");
  displayString(0, 40, "Tekan # Kembali");


  unsigned long start = millis();


  Serial.println("Viewing History...");


  while (millis() - start < 60000) {  // Loop selama 60 detik atau sampai # ditekan
    key = keypad.getKey();
    if (key == '#') {
      displayString(0, 20, "Kembali ke Admin Menu.");
      delay(1000);
      STATE = 3;  // Kembali ke menu admin
      return;
    }
    // Jika ada logika scroll, tambahkan di sini (B dan C)
  }
  displayString(0, 20, "TIMEOUT!\nKembali ke Admin Menu.");
  delay(1000);
  STATE = 3;  // Kembali ke menu admin
}


// --- HELPER FUNCTIONS ---


// Read card UID
void cardTap() {
  current_uid_card = "";
  for (byte i = 0; i < rfid.uid.size; i++) {
    if (rfid.uid.uidByte[i] < 0x10) current_uid_card += "0";
    current_uid_card += String(rfid.uid.uidByte[i], HEX);
  }
  current_uid_card.toUpperCase();
}


// Check if UID is registered in EEPROM
bool isUidRegistered(String uid) {
  int uid_count = EEPROM.read(alamat_UID_count);
  for (int i = 0; i < uid_count; i++) {
    String stored_uid = "";
    for (int j = 0; j < uid.length(); j++) {                        // Baca UID sepanjang UID yang dicari (asumsi max 10 char)
      char c = EEPROM.read(alamat_UID_start + ((long)i * 10) + j);  // Pastikan cast ke long untuk menghindari overflow
      if (c == '\0') break;                                         // Hentikan jika null terminator
      stored_uid += c;
    }
    if (stored_uid == uid) {
      return true;
    }
  }
  return false;
}


// Save UID to EEPROM
void saveUid(String uid) {
  if (isUidRegistered(uid)) {
    Serial.println("UID " + uid + " sudah terdaftar.");
    displayString(0, 20, "UID " + uid + "\nSudah Terdaftar!");  // Tampilkan di OLED
    return;
  }


  int uid_count = EEPROM.read(alamat_UID_count);
  int next_address = alamat_UID_start + (uid_count * 10);  // Asumsi 10 karakter per UID


  if (next_address + uid.length() + 1 > alamat_UID_end) {  // +1 untuk null terminator
    Serial.println("EEPROM penuh, tidak bisa menambah UID.");
    displayString(0, 20, "EEPROM Penuh!\nTidak Bisa Menambah.");  // Tampilkan di OLED
    return;
  }


  for (int i = 0; i < uid.length(); i++) {
    EEPROM.write(next_address + i, uid.charAt(i));
  }
  EEPROM.write(next_address + uid.length(), '\0');  // Tambahkan null terminator
  EEPROM.write(alamat_UID_count, uid_count + 1);
  EEPROM.commit();
  Serial.println("UID " + uid + " disimpan ke EEPROM.");
  displayString(0, 20, "UID Ditambahkan:\n" + uid);  // Tampilkan di OLED
}


// Delete UID from EEPROM
void deleteUid(String uid) {
  int uid_count = EEPROM.read(alamat_UID_count);
  int found_index = -1;


  for (int i = 0; i < uid_count; i++) {
    String stored_uid = "";
    for (int j = 0; j < 10; j++) {  // Baca max 10 karakter untuk setiap UID
      char c = EEPROM.read(alamat_UID_start + ((long)i * 10) + j);
      if (c == '\0') break;
      stored_uid += c;
    }
    if (stored_uid == uid) {
      found_index = i;
      break;
    }
  }


  if (found_index != -1) {
    // Geser UID setelah yang dihapus
    for (int i = found_index; i < uid_count - 1; i++) {
      for (int j = 0; j < 10; j++) {  // Asumsi max 10 chars per UID
        char c = EEPROM.read(alamat_UID_start + ((long)(i + 1) * 10) + j);
        EEPROM.write(alamat_UID_start + ((long)i * 10) + j, c);
      }
    }
    // Bersihkan slot terakhir dan kurangi hitungan
    for (int j = 0; j < 10; j++) {
      EEPROM.write(alamat_UID_start + ((long)(uid_count - 1) * 10) + j, '\0');
    }
    EEPROM.write(alamat_UID_count, uid_count - 1);
    EEPROM.commit();
    Serial.println("UID " + uid + " berhasil dihapus dari EEPROM.");
    displayString(0, 20, "UID Dihapus:\n" + uid);  // Tampilkan di OLED
  } else {
    Serial.println("UID " + uid + " tidak ditemukan di EEPROM.");
    displayString(0, 20, "UID " + uid + "\nTidak Ditemukan!");  // Tampilkan di OLED
  }
}


// Display Menu
void display_menu() {
  // Normalize menu_sel to handle wrap-around correctly
  if (menu_sel < 0) menu_sel = num_menu_items - 1;
  if (menu_sel >= num_menu_items) menu_sel = 0;


  int menu_prev = (menu_sel - 1 + num_menu_items) % num_menu_items;
  int menu_next = (menu_sel + 1) % num_menu_items;


  u8g2.clearBuffer();
  u8g2.drawBitmap(120, 0, 8 / 8, 64, scrollbar);
  u8g2.drawBitmap(120, menu_sel * 16, 8 / 8, 16, handle);  // Adjust handle position based on menu_sel
  u8g2.drawBitmap(0, 22, 120 / 8, 21, outline);


  u8g2.setFont(u8g2_font_t0_13_tr);
  u8g2.drawStr(24, 15, menu_item[menu_prev]);
  u8g2.drawBitmap(4, 2, 16 / 8, 16, bitmap_icons[menu_prev]);


  u8g2.setFont(u8g2_font_t0_13b_tr);  // Font bold for selected item
  u8g2.drawStr(24, 37, menu_item[menu_sel]);
  u8g2.drawBitmap(4, 24, 16 / 8, 16, bitmap_icons[menu_sel]);


  u8g2.setFont(u8g2_font_t0_13_tr);
  u8g2.drawStr(24, 59, menu_item[menu_next]);
  u8g2.drawBitmap(4, 46, 16 / 8, 16, bitmap_icons[menu_next]);


  u8g2.sendBuffer();
}


// Generic display string function with newline support
void displayString(int x, int y, const char* text) {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_t0_13b_tr);  // Menggunakan font bold untuk default


  int lineHeight = 16;  // Tinggi baris untuk font ini
  int currentY = y;


  char tempText[128];  // Buffer sementara untuk string yang akan ditampilkan
  strncpy(tempText, text, sizeof(tempText) - 1);
  tempText[sizeof(tempText) - 1] = '\0';  // Pastikan null-terminated


  char* token = strtok(tempText, "\n");
  while (token != NULL) {
    u8g2.drawStr(x, currentY, token);
    currentY += lineHeight;
    token = strtok(NULL, "\n");
  }
  u8g2.sendBuffer();
}


// Overload untuk String
void displayString(int x, int y, String text) {
  displayString(x, y, text.c_str());
}
void gettime() {
  struct tm timeinfo;
  if (getLocalTime(&timeinfo)) {
    strftime(hariStr, sizeof(hariStr), "%A", &timeinfo);
    strftime(tanggalStr, sizeof(tanggalStr), "%d %B %Y", &timeinfo);
    strftime(waktuStr, sizeof(waktuStr), "%H:%M:%S", &timeinfo);
  } else {
    strncpy(hariStr, "Gagal", sizeof(hariStr));
    hariStr[sizeof(hariStr) - 1] = '\0';
    strncpy(tanggalStr, "Koneksi Waktu", sizeof(tanggalStr));
    tanggalStr[sizeof(tanggalStr) - 1] = '\0';
    waktuStr[0] = '\0';  // Kosongkan waktu jika gagal
  }
}


#include <U8g2lib.h>            // Memasukkan library untuk OLED (U8g2)
#include <ESP32Servo.h>          // Memasukkan library untuk mengontrol servo motor
#include <SPI.h>                 // Memasukkan library SPI untuk komunikasi serial peripheral interface
#include <Keypad.h>              // Memasukkan library untuk keypad matriks
#include <MFRC522.h>             // Memasukkan library untuk modul RFID
#include <EEPROM.h>              // Memasukkan library untuk menggunakan EEPROM
#include <WiFi.h>                // Memasukkan library untuk koneksi Wi-Fi
#include <time.h>                // Memasukkan library untuk menggunakan waktu NTP (Network Time Protocol)


// --- RFID & EEPROM DEFINES ---
#define SS_PIN 5                // Menentukan pin untuk slave select (SS) RFID
#define RST_PIN 2               // Menentukan pin untuk reset RFID
MFRC522 rfid(SS_PIN, RST_PIN);  // Inisialisasi objek RFID dengan pin SS dan reset


#define EEPROM_SIZE 512         // Ukuran EEPROM yang digunakan (512 byte)
const int alamat_UID_start = 0;  // Alamat awal untuk menyimpan UID
const int alamat_UID_end = 100;  // Batas akhir alamat untuk penyimpanan UID (100 byte)
const int alamat_UID_count = 101;  // Alamat untuk menghitung jumlah UID yang terdaftar


// --- OLED DEFINES & BITMAPS ---
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0);  // Inisialisasi OLED dengan resolusi 128x64, menggunakan I2C


// BITMAPS (Menentukan gambar untuk ikon yang akan ditampilkan di layar OLED)
const unsigned char add_card[] PROGMEM = { ... };  // Bitmap untuk ikon "Tambah Kartu"
const unsigned char history[] PROGMEM = { ... };   // Bitmap untuk ikon "Riwayat"
const unsigned char lock[] PROGMEM = { ... };      // Bitmap untuk ikon "Kunci"
const unsigned char remove_card[] PROGMEM = { ... }; // Bitmap untuk ikon "Hapus Kartu"
const unsigned char outline[] PROGMEM = { ... };   // Bitmap untuk outline


const int num_menu_items = 4;   // Menentukan jumlah item menu yang ada
const char* menu_item[] = {     // Daftar item menu
  "Buka Kunci",                 
  "Tambah RFID",               
  "Hapus RFID",               
  "Riwayat Akses",            
};

const unsigned char* bitmap_icons[4] = {    // Menyimpan bitmap untuk masing-masing item menu
  lock,
  add_card,
  remove_card,
  history
};

volatile int menu_sel = 0;  // Variabel untuk menentukan item menu yang dipilih, menggunakan 'volatile' karena bisa diubah dalam ISR (Interrupt Service Routine)


// --- SERVO DEFINES ---
Servo myServo;                    // Objek untuk mengontrol servo
#define servo_pin 15              // Pin untuk menghubungkan servo (pin 15 di ESP32)


const byte ROWS = 4;            // Jumlah baris pada keypad
const byte COLS = 4;            // Jumlah kolom pada keypad
char keys[ROWS][COLS] = {       // Penempatan tombol pada keypad matriks
  { '1', '2', '3', 'A' },
  { '4', '5', '6', 'B' },
  { '7', '8', '9', 'C' },
  { '*', '0', '#', 'D' }
};
byte rowPins[ROWS] = { 13, 12, 14, 27 };  // Pin baris pada ESP32
byte colPins[COLS] = { 26, 25, 33, 32 };  // Pin kolom pada ESP32
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);  // Inisialisasi keypad


String inputPassword = "";       // Variabel untuk menyimpan input password
const String correctPassword = "1234";  // Password yang benar
const String adminPassword = "7777";    // Password untuk akses admin
int STATE = 0;                    // Variabel untuk menyimpan status program (state)


String current_uid_card = "";     // Variabel untuk menyimpan UID kartu yang sedang dipindai


// --- FUNCTION PROTOTYPES ---
void cardTap();                  // Fungsi untuk membaca kartu RFID yang ditempel
bool isUidRegistered(String uid); // Fungsi untuk mengecek apakah UID sudah terdaftar
void saveUid(String uid);        // Fungsi untuk menyimpan UID ke EEPROM
void deleteUid(String uid);      // Fungsi untuk menghapus UID dari EEPROM
void displayString(int x, int y, const char* text); // Fungsi untuk menampilkan teks di layar OLED
void displayString(int x, int y, String text);      // Overload fungsi displayString untuk String


void display_menu();             // Fungsi untuk menampilkan menu admin


void state_rfid_scan();          // Fungsi untuk state pemindaian RFID
void state_input_password();     // Fungsi untuk state input password
void state_open_lock();          // Fungsi untuk state membuka kunci
void state_admin_menu();         // Fungsi untuk state menu admin
void state_add_rfid();           // Fungsi untuk state menambah RFID
void state_delete_rfid();        // Fungsi untuk state menghapus RFID
void state_view_history();       // Fungsi untuk state melihat riwayat akses


// --- Fitur Waktu (NTP) ---
void gettime();                  // Fungsi untuk mendapatkan waktu dari NTP
char hariStr[15];                // Menyimpan nama hari
char tanggalStr[20];             // Menyimpan tanggal
char waktuStr[10];               // Menyimpan waktu


// --- SETUP ---
void setup() {
  Serial.begin(115200);            // Memulai komunikasi serial dengan baudrate 115200


  EEPROM.begin(EEPROM_SIZE);       // Menginisialisasi EEPROM dengan ukuran yang sudah ditentukan
  Serial.println("EEPROM Ready.");  // Menampilkan pesan bahwa EEPROM siap digunakan


  SPI.begin();                     // Memulai komunikasi SPI
  rfid.PCD_Init();                 // Menginisialisasi modul RFID
  Serial.println("RFID Ready.");    // Menampilkan pesan bahwa RFID siap digunakan


  u8g2.begin();                    // Menginisialisasi OLED
  u8g2.setColorIndex(1);            // Mengatur warna indeks untuk OLED (1: putih, 0: hitam)
  Serial.println("OLED Ready.");    // Menampilkan pesan bahwa OLED siap digunakan


  myServo.attach(servo_pin);       // Menghubungkan servo dengan pin yang telah ditentukan
  myServo.write(45);               // Menetapkan posisi servo ke 45 derajat (pintu tertutup)
  Serial.println("Servo Ready.");  // Menampilkan pesan bahwa servo siap digunakan


  // Konfigurasi WiFi dan NTP
  Serial.println("Menghubungkan ke WiFi...");
  WiFi.begin("Apelu", "pecaron123");  // Menghubungkan ESP32 ke Wi-Fi
  int wifi_attempts = 0;
  while (WiFi.status() != WL_CONNECTED && wifi_attempts < 20) {  // Coba 20 kali (10 detik)
    delay(500);  // Menunggu setengah detik
    Serial.print(".");  // Menampilkan titik setiap kali percobaan
    wifi_attempts++;  // Menambah jumlah percobaan
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi Terhubung!");    // Jika terhubung, tampilkan pesan
    configTime(7 * 3600, 0, "pool.ntp.org");  // Sinkronisasi waktu dengan NTP
    Serial.println("NTP Disinkronkan.");      // Menampilkan pesan waktu berhasil disinkronkan
  } else {
    Serial.println("\nGagal Terhubung ke WiFi/NTP. Waktu mungkin tidak akurat.");
    displayString(0, 0, "WiFi Gagal!");      // Menampilkan pesan kegagalan pada OLED
    displayString(0, 20, "Waktu Tidak\nAkura!");  // Menampilkan pesan bahwa waktu tidak akurat
    delay(2000);  // Menunggu 2 detik
  }


  displayString(0, 20, "Sistem Siap!");  // Menampilkan pesan bahwa sistem siap digunakan
  delay(1000);  // Menunggu sebentar
}


// --- MAIN LOOP ---
void loop() {
  key = keypad.getKey();            // Membaca tombol yang ditekan pada keypad
  start = millis();                 // Menyimpan waktu saat ini untuk timer
  switch (STATE) {                  // Memilih state berdasarkan nilai STATE
    case 0: state_rfid_scan(); break;  // Jika STATE 0, pindah ke fungsi state_rfid_scan
    case 1: state_input_password(); break;  // Jika STATE 1, pindah ke fungsi state_input_password
    case 2: state_open_lock(); break;  // Jika STATE 2, pindah ke fungsi state_open_lock
    case 3: state_admin_menu(); break;  // Jika STATE 3, pindah ke fungsi state_admin_menu
    case 4: state_add_rfid(); break;  // Jika STATE 4, pindah ke fungsi state_add_rfid
    case 5: state_delete_rfid(); break;  // Jika STATE 5, pindah ke fungsi state_delete_rfid
    case 6: state_view_history(); break;  // Jika STATE 6, pindah ke fungsi state_view_history
    default: STATE = 0; break;        // Jika tidak ada match, kembali ke state 0
  }
}

// Fungsi untuk menyimpan UID ke EEPROM
for (int i = 0; i < uid.length(); i++) {  // Loop untuk membaca setiap karakter dari UID
    EEPROM.write(next_address + i, uid.charAt(i));  // Menulis karakter UID ke EEPROM pada alamat yang ditentukan
}
EEPROM.write(next_address + uid.length(), '\0');  // Menambahkan null terminator setelah UID untuk menandakan akhir string
EEPROM.write(alamat_UID_count, uid_count + 1);  // Menyimpan jumlah UID yang terdaftar di EEPROM
EEPROM.commit();  // Menyimpan perubahan ke EEPROM
Serial.println("UID " + uid + " disimpan ke EEPROM.");  // Menampilkan pesan di Serial Monitor
displayString(0, 20, "UID Ditambahkan:\n" + uid);  // Menampilkan pesan di OLED bahwa UID berhasil ditambahkan
}


// Fungsi untuk menghapus UID dari EEPROM
void deleteUid(String uid) {
  int uid_count = EEPROM.read(alamat_UID_count);  // Membaca jumlah UID yang terdaftar dari EEPROM
  int found_index = -1;  // Variabel untuk menyimpan index dari UID yang ditemukan
  
  for (int i = 0; i < uid_count; i++) {  // Loop untuk mencari UID di EEPROM
    String stored_uid = "";  // String untuk menyimpan UID yang terdaftar
    for (int j = 0; j < 10; j++) {  // Loop untuk membaca maksimal 10 karakter UID
      char c = EEPROM.read(alamat_UID_start + ((long)i * 10) + j);  // Membaca setiap karakter UID dari EEPROM
      if (c == '\0') break;  // Jika menemukan null terminator, berhenti membaca karakter
      stored_uid += c;  // Menambahkan karakter ke string stored_uid
    }
    if (stored_uid == uid) {  // Jika UID yang ditemukan sama dengan UID yang akan dihapus
      found_index = i;  // Simpan index UID yang ditemukan
      break;  // Keluar dari loop
    }
  }

  if (found_index != -1) {  // Jika UID ditemukan
    // Geser UID setelah yang dihapus
    for (int i = found_index; i < uid_count - 1; i++) {  // Loop untuk menggeser UID setelah yang dihapus
      for (int j = 0; j < 10; j++) {  // Membaca maksimal 10 karakter UID
        char c = EEPROM.read(alamat_UID_start + ((long)(i + 1) * 10) + j);  // Membaca UID yang akan digeser
        EEPROM.write(alamat_UID_start + ((long)i * 10) + j, c);  // Menyimpan UID yang digeser ke alamat yang benar
      }
    }
    // Bersihkan slot terakhir dan kurangi hitungan
    for (int j = 0; j < 10; j++) {  // Menghapus data pada slot terakhir yang sudah digeser
      EEPROM.write(alamat_UID_start + ((long)(uid_count - 1) * 10) + j, '\0');  // Menulis null ke EEPROM untuk membersihkan data
    }
    EEPROM.write(alamat_UID_count, uid_count - 1);  // Mengurangi jumlah UID yang terdaftar di EEPROM
    EEPROM.commit();  // Menyimpan perubahan ke EEPROM
    Serial.println("UID " + uid + " berhasil dihapus dari EEPROM.");  // Menampilkan pesan di Serial Monitor
    displayString(0, 20, "UID Dihapus:\n" + uid);  // Menampilkan pesan di OLED bahwa UID berhasil dihapus
  } else {
    Serial.println("UID " + uid + " tidak ditemukan di EEPROM.");  // Menampilkan pesan bahwa UID tidak ditemukan
    displayString(0, 20, "UID " + uid + "\nTidak Ditemukan!");  // Menampilkan pesan di OLED bahwa UID tidak ditemukan
  }
}


// Fungsi untuk menampilkan menu di OLED
void display_menu() {
  // Normalize menu_sel to handle wrap-around correctly
  if (menu_sel < 0) menu_sel = num_menu_items - 1;  // Menangani jika menu_sel kurang dari 0 (wrap-around)
  if (menu_sel >= num_menu_items) menu_sel = 0;  // Menangani jika menu_sel melebihi jumlah item menu

  int menu_prev = (menu_sel - 1 + num_menu_items) % num_menu_items;  // Menentukan menu sebelumnya
  int menu_next = (menu_sel + 1) % num_menu_items;  // Menentukan menu berikutnya

  u8g2.clearBuffer();  // Menghapus tampilan sebelumnya di OLED
  u8g2.drawBitmap(120, 0, 8 / 8, 64, scrollbar);  // Menampilkan scrollbar
  u8g2.drawBitmap(120, menu_sel * 16, 8 / 8, 16, handle);  // Menampilkan handle sesuai dengan menu yang dipilih
  u8g2.drawBitmap(0, 22, 120 / 8, 21, outline);  // Menampilkan outline di OLED

  u8g2.setFont(u8g2_font_t0_13_tr);  // Mengatur font untuk menu sebelumnya
  u8g2.drawStr(24, 15, menu_item[menu_prev]);  // Menampilkan menu sebelumnya
  u8g2.drawBitmap(4, 2, 16 / 8, 16, bitmap_icons[menu_prev]);  // Menampilkan ikon menu sebelumnya

  u8g2.setFont(u8g2_font_t0_13b_tr);  // Mengatur font tebal untuk menu yang dipilih
  u8g2.drawStr(24, 37, menu_item[menu_sel]);  // Menampilkan menu yang dipilih
  u8g2.drawBitmap(4, 24, 16 / 8, 16, bitmap_icons[menu_sel]);  // Menampilkan ikon menu yang dipilih

  u8g2.setFont(u8g2_font_t0_13_tr);  // Mengatur font untuk menu berikutnya
  u8g2.drawStr(24, 59, menu_item[menu_next]);  // Menampilkan menu berikutnya
  u8g2.drawBitmap(4, 46, 16 / 8, 16, bitmap_icons[menu_next]);  // Menampilkan ikon menu berikutnya

  u8g2.sendBuffer();  // Mengirimkan data ke OLED untuk ditampilkan
}


// Fungsi untuk menampilkan string di OLED
void displayString(int x, int y, const char* text) {
  u8g2.clearBuffer();  // Menghapus tampilan sebelumnya di OLED
  u8g2.setFont(u8g2_font_t0_13b_tr);  // Menggunakan font untuk teks

  int lineHeight = 16;  // Menentukan tinggi baris untuk font ini
  int currentY = y;     // Menyimpan posisi Y untuk teks yang ditampilkan

  char tempText[128];   // Buffer sementara untuk teks
  strncpy(tempText, text, sizeof(tempText) - 1);  // Menyalin teks ke buffer
  tempText[sizeof(tempText) - 1] = '\0';  // Memastikan teks null-terminated

  char* token = strtok(tempText, "\n");  // Memisahkan teks berdasarkan baris
  while (token != NULL) {  // Selama ada token (baris) untuk ditampilkan
    u8g2.drawStr(x, currentY, token);  // Menampilkan token pada posisi yang ditentukan
    currentY += lineHeight;  // Menambahkan tinggi baris untuk baris berikutnya
    token = strtok(NULL, "\n");  // Mengambil token berikutnya
  }
  u8g2.sendBuffer();  // Mengirimkan data ke OLED untuk ditampilkan
}


// Overload fungsi untuk menerima String sebagai parameter
void displayString(int x, int y, String text) {
  displayString(x, y, text.c_str());  // Mengkonversi String ke const char* dan memanggil fungsi sebelumnya
}


// Fungsi untuk mendapatkan waktu NTP
void gettime() {
  struct tm timeinfo;
  if (getLocalTime(&timeinfo)) {  // Jika waktu berhasil diambil
    strftime(hariStr, sizeof(hariStr), "%A", &timeinfo);  // Menyimpan nama hari
    strftime(tanggalStr, sizeof(tanggalStr), "%d %B %Y", &timeinfo);  // Menyimpan tanggal
    strftime(waktuStr, sizeof(waktuStr), "%H:%M:%S", &timeinfo);  // Menyimpan waktu
  } else {
    strncpy(hariStr, "Gagal", sizeof(hariStr));  // Jika gagal, tampilkan "Gagal"
    hariStr[sizeof(hariStr) - 1] = '\0';  // Menambahkan null terminator
    strncpy(tanggalStr, "Koneksi Waktu", sizeof(tanggalStr));  // Menampilkan "Koneksi Waktu"
    tanggalStr[sizeof(tanggalStr) - 1] = '\0';  // Menambahkan null terminator
    waktuStr[0] = '\0';  // Kosongkan waktu jika gagal
  }
}










KODE PROGRAM FINAL

#include <U8g2lib.h>
#include <Wire.h>
#include <WiFi.h>
#include <time.h>
#include <Keypad.h>  // Include the Keypad library




// --- Existing Bitmap Definitions (no change needed here) ---
// ' add_card', 16x16px
const unsigned char add_card[] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x7f, 0xfa, 0xff, 0xfd, 0xfe, 0x0d, 0xc3, 0xfd, 0xc3, 0xfd, 0xc2, 0x0d,
  0xc3, 0xfd, 0xc3, 0xfd, 0xfe, 0x0d, 0xff, 0xfd, 0xff, 0xfd, 0x7f, 0xfa, 0x00, 0x00, 0x00, 0x00
};
// ' history', 16x16px
const unsigned char history[] PROGMEM = {
  0x7f, 0xfe, 0x20, 0x04, 0x20, 0x04, 0x20, 0x04, 0x1, 0x08, 0x17, 0xe8, 0x0b, 0xd0, 0x05, 0xa0,
  0x05, 0xa0, 0x09, 0x90, 0x11, 0x88, 0x13, 0xc8, 0x2f, 0xf4, 0x2f, 0xf4, 0x20, 0x04, 0x7f, 0xfe
};
// ' key', 16x16px
const unsigned char key[] PROGMEM = {
  0x00, 0x00, 0x00, 0xf8, 0x01, 0xfc, 0x03, 0xe6, 0x03, 0xc3, 0x03, 0xc3, 0x03, 0xe7, 0x03, 0xff,
  0x07, 0xff, 0x0d, 0xfe, 0x1b, 0xbc, 0x37, 0x80, 0x6c, 0x00, 0xdc, 0x00, 0xb0, 0x00, 0xf0, 0x00
};
// ' remove_card', 16x16px
const unsigned char remove_card[] PROGMEM = {
  0xaf, 0xff, 0xd7, 0xff, 0x00, 0x00, 0x55, 0x56, 0x68, 0x8a, 0x55, 0x56, 0x62, 0x22, 0x35, 0x54,
  0x28, 0x8c, 0x35, 0x54, 0x22, 0x24, 0x15, 0x58, 0x18, 0x88, 0x15, 0x58, 0x00, 0x00, 0x17, 0xf8
};




// 'outline', 128x21px
const unsigned char outline[] PROGMEM = {
  0x1f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0x20,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x40, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x40, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x40, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x40, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x03, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x03, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x03, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x03, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x03, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03,
  0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x40,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x40, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x3f, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x1f, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc
};
// ' handle', 8x16px
const unsigned char handle[] PROGMEM = {
  0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07
};
// ' scrollbar', 8x64px
const unsigned char scrollbar[] PROGMEM = {
  0x00, 0x02, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00,
  0x00, 0x02, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00,
  0x00, 0x02, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00,
  0x00, 0x02, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00
};


U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0);


// --- Keypad Definitions ---
const byte ROWS = 4;  // four rows
const byte COLS = 4;  // four columns
char keys[ROWS][COLS] = {
  { '1', '2', '3', 'A' },
  { '4', '5', '6', 'B' },
  { '7', '8', '9', 'C' },
  { '*', '0', '#', 'D' }
};


byte rowPins[ROWS] = { 13, 12, 14, 27 };
byte colPins[COLS] = { 26, 25, 33, 32 };




// Initialize the keypad
Keypad customKeypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);


// Array of all bitmaps for convenience. (Total bytes used to store images in PROGMEM = 336)
const unsigned char* bitmap_icons[4] = {
  key,
  add_card,
  remove_card,
  history
};


// Menu Items
const int num_items = 4;
const char* menu_item[] = {
  "Buka Kunci",
  "Tambah RFID",
  "Hapus RFID",
  "Riwayat Akses",
};


int item_sel = 0;
int item_prev;
int item_next;
bool timeUpdated = false;


char hariStr[15];
char tanggalStr[20];  // Contoh: "Saturday, 01 June 2025"
char waktuStr[10];


int STATE = -1;  // Mulai di menu utama


void display();
void gettime();


void setup() {
  Serial.begin(115200);  // For debugging output


  WiFi.begin("ZURIN ABDILLAH", "07082004");
  while (WiFi.status() != WL_CONNECTED)
    ;


  u8g2.begin();           // Initialize the U8g2 library
  u8g2.setColorIndex(1);  // Set display color (usually 1 for white pixels)


  configTime(7 * 3600, 0, "pool.ntp.org");
  delay(2000);
}


void loop() {
  char key = customKeypad.getKey();  // Panggil fungsi untuk mendapatkan tombol yang ditekan
  switch (STATE) {
    case -1:  //MAIN MENU
      state_menuadmin();
      break;


    case 0:  //BUKA KUNCI
      state_bukakunci();
      break;


    case 1:  //TAMBAH RFID
      state_TambahRFID();
      break;


    case 2:  //HAPUS RFID
      state_HapusRFID();
      break;


    case 3:  //RIWAYAT AKSES
      state_RiwayatAkses();
      break;
  }
  display();
}


void display() {
  u8g2.firstPage();
  do {
    u8g2.clearBuffer();  // Membersihkan seluruh buffer menjadi hitam (0)


    // Logika gambar tampilan disesuaikan berdasarkan STATE
    switch (STATE) {
      case -1:
        // --- Tampilan Menu Utama ---
        item_prev = (item_sel - 1 + num_items) % num_items;  // Handle wrap around for prev
        item_next = (item_sel + 1) % num_items;              // Handle wrap around for next


        u8g2.drawBitmap(120, 0, 8 / 8, 64, scrollbar);
        u8g2.drawBitmap(120, item_sel * 16, 8 / 8, 16, handle);
        u8g2.drawBitmap(0, 22, 120 / 8, 21, outline);


        u8g2.setFont(u8g2_font_t0_13_tr);
        u8g2.drawStr(24, 15, menu_item[item_prev]);
        u8g2.drawBitmap(4, 2, 16 / 8, 16, bitmap_icons[item_prev]);


        u8g2.setFont(u8g2_font_t0_13b_tr);
        u8g2.drawStr(24, 37, menu_item[item_sel]);
        u8g2.drawBitmap(4, 24, 16 / 8, 16, bitmap_icons[item_sel]);


        u8g2.setFont(u8g2_font_t0_13_tr);
        u8g2.drawStr(24, 59, menu_item[item_next]);
        u8g2.drawBitmap(4, 46, 16 / 8, 16, bitmap_icons[item_next]);
        break;


      case 0:  // BUKA KUNCI
        u8g2.setFont(u8g2_font_t0_13b_tr);
        u8g2.drawStr(0, 15, hariStr);
        u8g2.drawStr(0, 30, tanggalStr);
        u8g2.drawStr(0, 45, waktuStr);
        // Tambahkan elemen UI lain untuk input PIN jika diperlukan
        break;


      case 1:
        u8g2.setFont(u8g2_font_t0_13b_tr);
        u8g2.drawStr(0, 30, "Dekatkan RFID...");
        u8g2.drawStr(0, 45, "Tekan # Kembali");
        break;


      case 2:
        u8g2.setFont(u8g2_font_t0_13b_tr);
        u8g2.drawStr(0, 30, "Hapus RFID: Input ID");
        u8g2.drawStr(0, 45, "Tekan # Kembali");
        break;


      case 3:
        u8g2.setFont(u8g2_font_t0_13b_tr);
        u8g2.drawStr(0, 30, "Memuat Riwayat...");
        u8g2.drawStr(0, 45, "Tekan # Kembali");
        break;
    }
  } while (u8g2.nextPage());
}


void gettime() {
  struct tm timeinfo;
  if (getLocalTime(&timeinfo)) {
    strftime(hariStr, sizeof(hariStr), "%A", &timeinfo);              // Hari saja
    strftime(tanggalStr, sizeof(tanggalStr), "%d %B %Y", &timeinfo);  // Tanggal saja
    strftime(waktuStr, sizeof(waktuStr), "%H:%M:%S", &timeinfo);      // Jam menit detik
  } else {
    strncpy(hariStr, "Gagal waktu", sizeof(hariStr));
    hariStr[sizeof(hariStr) - 1] = '\0';
    strncpy(tanggalStr, "", sizeof(tanggalStr));
    waktuStr[0] = '\0';
  }
}


void displaystring(int x, int y, char txt[]) {
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_t0_13b_tr);
    u8g2.drawStr(0, 30, txt);
  } while (u8g2.nextPage());
}


void state_menuadmin {
  if (key == 'A') {  // Tombol 'A' untuk naik
    item_sel--;
    if (item_sel < 0) {
      item_sel = num_items - 1;
    }
  } else if (key == 'B') {  // Tombol 'B' untuk turun
    item_sel++;
    if (item_sel >= num_items) {
      item_sel = 0;
    }
  } else if (key == '*') {  // Tombol '*' untuk memilih
    Serial.print("Selected item: ");
    Serial.println(menu_item[item_sel]);
    // Transisi state berdasarkan item yang dipilih
    switch (item_sel) {
      case 0: STATE = 0; break;
      case 1: STATE = 1; break;
      case 2: STATE = 2; break;
      case 3: STATE = 3; break;
      default: break;
    }
  }
}


void state_bukakunci() {
  if (!timeUpdated) {
    gettime();
    timeUpdated = true;  // Tandai sudah update waktu
  }
  if (key == '#') {
    Serial.println("Returning from Buka Kunci...");
    STATE = -1;
    timeUpdated = false;  // Reset flag supaya saat masuk lagi waktu bisa diupdate ulang
  }
}


void state_TambahRFID() {
  if (key == '#') {
    Serial.println("Returning from Tambah RFID...");
    STATE = -1;
  }
}


void state_HapusRFID() {
  if (key == '#') {
    Serial.println("Returning from Hapus RFID...");
    STATE = -1;
  }
}


void state_RiwayatAkses() {
  if (key == '#') {
    Serial.println("Returning from Riwayat Akses...");
    STATE = -1;
  }
}





PENJELASAN
Protokol komunikasi
SPI (Serial Peripheral Interface)
Digunakan untuk komunikasi antara ESP32 dan modul pembaca RFID (MFRC522).
Implementasi dalam kode:

I2C (Inter-Integrated Circuit) 
Digunakan untuk komunikasi antara ESP32 dan modul OLED (SSD1306).
Implementasi dalam kode:

Wi-Fi (IEEE 802.11)
Digunakan untuk konektivitas jaringan, khususnya untuk menyinkronkan waktu dengan server NTP (Network Time Protocol).

Fitur
FSM 
Salah satu fitur utama dalam projek ini adalah penggunaan Finite State Machine (FSM). Ini merupakan model komputasional yang digunakan untuk  merepresentasikan dan mengendalikan alur eksekusi sebuah program. Sebuah FSM terdiri dari beberapa kondisi/state, masukan/input dan keluaran/output. Pemrograman berbasis FSM digunakan di dalam sistem yang bekerja berdasarkan perubahan input atau pemicu tertentu. 
Implementasi dalam kode:

Program ini terdiri dari 7 state yakni:
STATE = 0: state_rfid_scan() - Menunggu kartu RFID di-tap.
STATE = 1: state_input_password() - Meminta dan memproses input kata sandi.
STATE = 2: state_open_lock() - Membuka dan menutup kunci fisik.
STATE = 3: state_admin_menu() - Menampilkan menu pilihan untuk administrator.
STATE = 4: state_add_rfid() - Proses penambahan kartu RFID baru.
STATE = 5: state_delete_rfid() - Proses penghapusan kartu RFID.
STATE = 6: state_view_history() - (Placeholder) Untuk melihat riwayat akses.
Antarmuka Pengguna (User Interface) dengan OLED
Projek ini menggunakan OLED display untuk memberikan umpan balik visual kepada user dengan memanfaatkan library U8g2lib
Sinkronisasi Waktu Jaringan (NTP)
Pada saat kunci terbuka sistem akan mengambil waktu dari server NTP melalui koneksi wifi. Hal ini dilakukan untuk menyesuaikan waktu dan tanggal yang akurat. Fitur ini dapat dikembangkan lebih lanjut agar administrator dapat menampilkan waktu dan tanggal akses kunci dari pengguna umum.
Implementasi dalam kode:


Keandalan (Reliability)
Penyimpanan Data Non-Volatil (EEPROM)
Projek ini menggunakan EEROM di flash memory ESP32 untuk menyimpan daftar UID kartu yang diizinkan membuka kunci. Hal ini memastikan bahwa daftar kartu valid tidak hilang saat sistem direset atau kehilangan daya.
Penanganan Kegagalan Koneksi Wi-Fi/NTP
Sistem akan mengambil waktu dari server NTP melalui koneksi internet. Ketika sistem gagal terhubung ke server atau wifi, sistem harus tetap berfungsi. Untuk itu pada kode diatas jika koneksi wifi tidak bisa terhubung setelah 20 kali percobaan maka sistem akan mencetak pesan kesalahan lalu melanjutkan boot. Kemudian  ketika sistem tidak bisa mengambil ke server NTP sistem akan mencetak “Gagal Koneksi Waktu” ke oled dan melanjutkan membuka kunci.
Timeout Input Pengguna
Pada setiap keadaan yang menunggu input seperti input password, sistem memiliki timeout 60 detik menggunakan millis(). Jika tidak ada input dalam waktu tersebut, sistem akan kembali ke state default.
Penanganan Kartu RFID
Sistem akan memastikan pembacaan kartu dan memberikan penanganan yang tepat. Ketika sistem membaca kartu RFID, penting untuk menghentikan sesi komunikasi dengan kartu tersebut agar RFID reader siap untuk kartu berikutnya. Hal ini mencegah masalah pembacaan ganda pada kartu yang sama jika kartu tetap berada di dekat RFID reader.
