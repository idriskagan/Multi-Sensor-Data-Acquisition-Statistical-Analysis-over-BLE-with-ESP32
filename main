```cpp

/*
 * Sensörler:
 *   MPL3115A2  -> 0x60  (Basınç, Sıcaklık)
 *   ICG-20660L -> 0x69  (6 Eksen: İvmeölçer + Jiroskop)
 *   BME280     -> 0x76  (Basınç, Nem, Sıcaklık)
 *
 * ESP32 Varsayılan I2C Pinleri:
 *   SDA -> GPIO 21  |  SCL -> GPIO 2
 */

#include <Wire.h>
#include <string.h>     /* memset, memcpy */
#include <math.h>       /* sqrtf          */
#include <NimBLEDevice.h>
#include <NimBLEServer.h>
#include <NimBLEUtils.h>

/* ── Sensör I2C Adresleri  */
#define MPL3115A2_ADDR      0x60
#define ICG20660L_ADDR      0x69
#define BME280_ADDR         0x76

/* ── MPL3115A2 Registerları  */
#define MPL_REG_STATUS      0x00
#define MPL_REG_OUT_P_MSB   0x01
#define MPL_REG_PT_DATA_CFG 0x13
#define MPL_REG_CTRL1       0x26

/* ── ICG-20660L Registerları  */
#define ICG_REG_PWR_MGMT_1   0x6B
#define ICG_REG_WHO_AM_I     0x75
#define ICG_REG_ACCEL_XOUT_H 0x3B

/* ── BME280 Registerları  */
#define BME_REG_CHIP_ID     0xD0
#define BME_REG_RESET       0xE0
#define BME_REG_CTRL_HUM    0xF2
#define BME_REG_CTRL_MEAS   0xF4
#define BME_REG_CONFIG      0xF5
#define BME_REG_PRESS_MSB   0xF7
#define BME_REG_CALIB_00    0x88
#define BME_REG_CALIB_26    0xE1

/* ── BME280 Kalibrasyon Değişkenleri */
uint16_t dig_T1;
int16_t  dig_T2, dig_T3;
uint16_t dig_P1;
int16_t  dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
uint8_t  dig_H1, dig_H3;
int16_t  dig_H2, dig_H4, dig_H5;
int8_t   dig_H6;
int32_t  t_fine;



// Medyan Filtresi için gereken değişkenler

#define MAX_WINDOW_SIZE  9
#define FILTER_CHANNELS  11

static float   f_buf[FILTER_CHANNELS][MAX_WINDOW_SIZE];
static uint8_t f_idx[FILTER_CHANNELS];
static uint8_t f_cnt[FILTER_CHANNELS];



struct kanal_istat_t {
    float min;
    float max;
    float median;
    float stddev;
};


static uint8_t okuma_sayaci = 0;
#define BLE_SEND_PERIYOT  BLE_PERIOD_S   /* 30 saniye = 30 örnek @ 1 Hz */

/* ══════════════════════════════════════════════════════════════════════════
 *
 *  Kanal haritası:
 *    0  MPL – Basınç(Pa)      1  MPL – Sıcaklık(C)
 *    2  ICG – Accel X(g)      3  ICG – Accel Y(g)     4  ICG – Accel Z(g)
 *    5  ICG – Gyro X(d/s)     6  ICG – Gyro Y(d/s)    7  ICG – Gyro Z(d/s)
 *    8  BME – Sıcaklık(C)     9  BME – Basınç(Pa)     10 BME – Nem(%)

 * ══════════════════════════════════════════════════════════════════════════ */

#define CIRC_BUF_SIZE   32
#define CIRC_CHANNELS   11

struct buf_handle_t {
    float   data[CIRC_BUF_SIZE]; /* Statik veri deposu                    */
    uint8_t head;                /* Bir sonraki yazma konumu               */
    uint8_t tail;                /* Bir sonraki okuma konumu (en eski)     */
    uint8_t count;               /* Tampondaki geçerli eleman sayısı       */
};

/* 11 kanal için statik tamponlar */
static struct buf_handle_t circ_buf[CIRC_CHANNELS];

/* Kanal isimleri (seri çıktıda görünür) */
static const char *kanal_adi[CIRC_CHANNELS] = {
    "MPL-Basinc  (Pa) ",
    "MPL-Sicaklik( C) ",
    "ICG-Accel-X ( g) ",
    "ICG-Accel-Y ( g) ",
    "ICG-Accel-Z ( g) ",
    "ICG-Gyro-X  (d/s)",
    "ICG-Gyro-Y  (d/s)",
    "ICG-Gyro-Z  (d/s)",
    "BME-Sicaklik( C) ",
    "BME-Basinc  (Pa) ",
    "BME-Nem     ( %) "
};

#define BLE_PERIOD_S     30
#define BLE_DEVICE_NAME  "ESP32-SEN"
#define BLE_MTU          256

/* Nordic UART Service UUID'leri */
#define NUS_SVC_UUID  "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_TX_UUID   "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_RX_UUID   "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"

/* BLE nesneleri */
static NimBLEServer         *ble_server  = nullptr;
static NimBLECharacteristic *ble_tx_char = nullptr;   /* ESP32 → Telefon */
static bool                  ble_connected = false;

class BleCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer *srv, NimBLEConnInfo &connInfo) override {
        ble_connected = true;
        Serial.println("[BLE] İstemci baglandi");
    }
    void onDisconnect(NimBLEServer *srv, NimBLEConnInfo &connInfo,
                      int reason) override {
        ble_connected = false;
        Serial.println("[BLE] İstemci ayrildi, reklam yeniden baslatiliyor");
        NimBLEDevice::startAdvertising();
    }
};


static void buffer_put_value(struct buf_handle_t *p_handle, float sensor_data)
{
    if (p_handle == NULL) return;

    if (p_handle->count == CIRC_BUF_SIZE) {
        /* Tampon dolu: tail ilerletilir, en eski veri silinir */
        p_handle->tail = (p_handle->tail + 1) % CIRC_BUF_SIZE;
    } else {
        p_handle->count++;
    }

    p_handle->data[p_handle->head] = sensor_data;
    p_handle->head = (p_handle->head + 1) % CIRC_BUF_SIZE;
}

/* ── buffer_get_value ──────────────────────────────────────────────────── */
/*
 * Tamponden en eski değeri (FIFO sırası) okur, tail'i ilerletir.
 *  Dönüş:  0  başarılı
 *         -1  geçersiz handle / pointer
 *         -2  tampon boş
 */
int buffer_get_value(struct buf_handle_t *p_handle, float *p_sensor_data)
{
    if (p_handle == NULL || p_sensor_data == NULL) return -1;
    if (p_handle->count == 0)                      return -2;

    *p_sensor_data = p_handle->data[p_handle->tail];
    p_handle->tail  = (p_handle->tail + 1) % CIRC_BUF_SIZE;
    p_handle->count--;
    return 0;
}

static int buffer_peek_latest(struct buf_handle_t *p_handle, float *p_sensor_data)
{
    if (p_handle == NULL || p_sensor_data == NULL) return -1;
    if (p_handle->count == 0)                      return -2;

    uint8_t last = (p_handle->head == 0) ? (CIRC_BUF_SIZE - 1)
                                          : (p_handle->head - 1);
    *p_sensor_data = p_handle->data[last];
    return 0;
}

 // *  HAREKETLİ MEDYAN FİLTRESİ

static void insertion_sort(float *arr, uint8_t n)
{
    for (uint8_t i = 1; i < n; i++) {
        float  key = arr[i];
        int8_t j   = (int8_t)i - 1;
        while (j >= 0 && arr[j] > key) { arr[j + 1] = arr[j]; j--; }
        arr[j + 1] = key;
    }
}


float filter_sensor_value(float raw_sensor_value,
                          uint8_t window_size,
                          uint8_t channel)
{
    if (window_size < 1)               window_size = 1;
    if (window_size > MAX_WINDOW_SIZE) window_size = MAX_WINDOW_SIZE;
    if (channel >= FILTER_CHANNELS)    channel = 0;

    f_buf[channel][f_idx[channel]] = raw_sensor_value;
    f_idx[channel] = (f_idx[channel] + 1) % window_size;
    if (f_cnt[channel] < window_size) f_cnt[channel]++;

    uint8_t n = f_cnt[channel];
    float   filtered;

    if (n == 1) {
        filtered = raw_sensor_value;
    } else {
        float tmp[MAX_WINDOW_SIZE];
        for (uint8_t i = 0; i < n; i++) tmp[i] = f_buf[channel][i];
        insertion_sort(tmp, n);
        filtered = tmp[n / 2];
    }

    /* Filtreli değeri dairesel arabelleğe kaydet */
    buffer_put_value(&circ_buf[channel], filtered);

    return filtered;   /* iç kullanım; seri ekranda kullanılmaz */
}

/* ══════════════════════════════════════════════════════════════════════════
 *  YARDIMCI FONKSİYONLAR (I2C)
 * ══════════════════════════════════════════════════════════════════════════ */

void i2c_yaz(uint8_t adres, uint8_t reg, uint8_t deger)
{
    Wire.beginTransmission(adres);
    Wire.write(reg);
    Wire.write(deger);
    Wire.endTransmission();
}

void i2c_oku(uint8_t adres, uint8_t reg, uint8_t *buf, uint8_t adet)
{
    Wire.beginTransmission(adres);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)adres, adet);
    for (uint8_t i = 0; i < adet; i++)
        buf[i] = Wire.available() ? Wire.read() : 0;
}

// *  METOT 1: MPL3115A2   (Kanal 0=Basınç, 1=Sıcaklık)


void mpl3115a2_baslat()
{
    i2c_yaz(MPL3115A2_ADDR, MPL_REG_PT_DATA_CFG, 0x07);
    i2c_yaz(MPL3115A2_ADDR, MPL_REG_CTRL1,       0xB9);
    delay(100);
    Serial.println("[MPL3115A2] Baslatildi");
}

void oku_mpl3115a2()
{
    uint8_t buf[5] = {0};

    i2c_oku(MPL3115A2_ADDR, MPL_REG_STATUS, buf, 1);
    if (!(buf[0] & 0x08)) {
        Serial.println("[MPL3115A2] Veri henuz hazir degil");
        return;
    }

    i2c_oku(MPL3115A2_ADDR, MPL_REG_OUT_P_MSB, buf, 5);

    int32_t raw_p    = ((int32_t)buf[0] << 16) | ((int32_t)buf[1] << 8) | buf[2];
    float ham_basinc = (float)(raw_p >> 6) + ((raw_p & 0x30) >> 4) * 0.25f;

    int8_t  temp_msb   = (int8_t)buf[3];
    uint8_t temp_lsb   = (buf[4] >> 4) & 0x0F;
    float ham_sicaklik = (float)temp_msb + temp_lsb * 0.0625f;

    /* Filtrele → buffer'a yaz (filter_sensor_value içinde otomatik) */
    filter_sensor_value(ham_basinc,   7, 0);
    filter_sensor_value(ham_sicaklik, 7, 1);
}

 //  METOT 2: ICG-20660L  (Kanal 2-4=Accel, 5-7=Gyro)

void icg20660l_baslat()
{
    i2c_yaz(ICG20660L_ADDR, ICG_REG_PWR_MGMT_1, 0x01);
    delay(50);
    uint8_t who = 0;
    i2c_oku(ICG20660L_ADDR, ICG_REG_WHO_AM_I, &who, 1);
    Serial.print("[ICG-20660L] WHO_AM_I = 0x"); Serial.print(who, HEX);
    Serial.println(who == 0xF1 ? " (OK)" : " (beklenmeyen deger)");
}

void oku_icg20660l()
{
    uint8_t buf[14] = {0};
    i2c_oku(ICG20660L_ADDR, ICG_REG_ACCEL_XOUT_H, buf, 14);

    int16_t ax = (int16_t)((buf[0]  << 8) | buf[1]);
    int16_t ay = (int16_t)((buf[2]  << 8) | buf[3]);
    int16_t az = (int16_t)((buf[4]  << 8) | buf[5]);
    int16_t gx = (int16_t)((buf[8]  << 8) | buf[9]);
    int16_t gy = (int16_t)((buf[10] << 8) | buf[11]);
    int16_t gz = (int16_t)((buf[12] << 8) | buf[13]);

    /* Filtrele → buffer'a yaz */
    filter_sensor_value(ax / 4096.0f, 7, 2);
    filter_sensor_value(ay / 4096.0f, 7, 3);
    filter_sensor_value(az / 4096.0f, 7, 4);
    filter_sensor_value(gx / 32.8f,   7, 5);
    filter_sensor_value(gy / 32.8f,   7, 6);
    filter_sensor_value(gz / 32.8f,   7, 7);
}

 // METOT 3: BME280  (Kanal 8=Sıcaklık, 9=Basınç, 10=Nem)

void bme280_kalibrasyon_oku()
{
    uint8_t c[26] = {0};
    i2c_oku(BME280_ADDR, BME_REG_CALIB_00, c, 26);
    dig_T1=(uint16_t)(c[1]<<8|c[0]); dig_T2=(int16_t)(c[3]<<8|c[2]); dig_T3=(int16_t)(c[5]<<8|c[4]);
    dig_P1=(uint16_t)(c[7]<<8|c[6]); dig_P2=(int16_t)(c[9]<<8|c[8]); dig_P3=(int16_t)(c[11]<<8|c[10]);
    dig_P4=(int16_t)(c[13]<<8|c[12]);dig_P5=(int16_t)(c[15]<<8|c[14]);dig_P6=(int16_t)(c[17]<<8|c[16]);
    dig_P7=(int16_t)(c[19]<<8|c[18]);dig_P8=(int16_t)(c[21]<<8|c[20]);dig_P9=(int16_t)(c[23]<<8|c[22]);
    dig_H1=c[25];
    uint8_t c2[7]={0};
    i2c_oku(BME280_ADDR, BME_REG_CALIB_26, c2, 7);
    dig_H2=(int16_t)(c2[1]<<8|c2[0]); dig_H3=c2[2];
    dig_H4=(int16_t)((c2[3]<<4)|(c2[4]&0x0F));
    dig_H5=(int16_t)((c2[5]<<4)|(c2[4]>>4));
    dig_H6=(int8_t)c2[6];
}

float bme280_sicaklik_hesapla(int32_t adc_T)
{
    int32_t v1=((((adc_T>>3)-((int32_t)dig_T1<<1)))*dig_T2)>>11;
    int32_t v2=(((((adc_T>>4)-(int32_t)dig_T1)*((adc_T>>4)-(int32_t)dig_T1))>>12)*dig_T3)>>14;
    t_fine=v1+v2;
    return (float)((t_fine*5+128)>>8)/100.0f;
}

float bme280_basinc_hesapla(int32_t adc_P)
{
    int64_t v1=(int64_t)t_fine-128000, v2=v1*v1*dig_P6;
    v2+=(v1*dig_P5)<<17; v2+=(int64_t)dig_P4<<35;
    v1=((v1*v1*dig_P3)>>8)+((v1*dig_P2)<<12);
    v1=(((int64_t)1<<47)+v1)*dig_P1>>33;
    if(v1==0) return 0;
    int64_t p=1048576-adc_P;
    p=(((p<<31)-v2)*3125)/v1;
    v1=((int64_t)dig_P9*(p>>13)*(p>>13))>>25;
    v2=((int64_t)dig_P8*p)>>19;
    p=((p+v1+v2)>>8)+((int64_t)dig_P7<<4);
    return (float)p/256.0f;
}

float bme280_nem_hesapla(int32_t adc_H)
{
    int32_t v=t_fine-76800;
    v=(((adc_H<<14)-((int32_t)dig_H4<<20)-((int32_t)dig_H5*v)+16384)>>15)*
      ((((((v*dig_H6)>>10)*(((v*dig_H3)>>11)+32768))>>10)+2097152)*dig_H2+8192)>>14;
    v-=((((v>>15)*(v>>15))>>7)*dig_H1)>>4;
    if(v<0) v=0; if(v>419430400) v=419430400;
    return (float)(v>>12)/1024.0f;
}

void bme280_baslat()
{
    uint8_t chip_id=0;
    i2c_oku(BME280_ADDR, BME_REG_CHIP_ID, &chip_id, 1);

    i2c_yaz(BME280_ADDR, BME_REG_RESET,    0xB6); delay(10); /* soft reset  */
    bme280_kalibrasyon_oku();
    i2c_yaz(BME280_ADDR, BME_REG_CTRL_MEAS,0x00);            /* sleep       */
    i2c_yaz(BME280_ADDR, BME_REG_CONFIG,   0xA0);            /* standby     */
    i2c_yaz(BME280_ADDR, BME_REG_CTRL_HUM, 0x01);            /* nem OSR x1  */
    i2c_yaz(BME280_ADDR, BME_REG_CTRL_MEAS,0x27);            /* normal mod  */
    delay(100);

    Serial.print("[BME280] Chip ID=0x"); Serial.print(chip_id,HEX);
    if(chip_id==0x60)                    Serial.println(" -> BME280 OK");
    else if(chip_id==0x58||chip_id==0x56)Serial.println(" -> BMP280! Nem YOK.");
    else                                 Serial.println(" -> Tanimsiz");
    Serial.println("[BME280] Baslatildi");
}

void oku_bme280()
{
    uint8_t buf[8]={0};
    i2c_oku(BME280_ADDR, BME_REG_PRESS_MSB, buf, 8);

    int32_t adc_P=((int32_t)buf[0]<<12)|((int32_t)buf[1]<<4)|(buf[2]>>4);
    int32_t adc_T=((int32_t)buf[3]<<12)|((int32_t)buf[4]<<4)|(buf[5]>>4);
    int32_t adc_H=((int32_t)buf[6]<<8)|(int32_t)buf[7];

    /* Filtrele → buffer'a yaz
     * NOT: bme280_sicaklik_hesapla() t_fine'i günceller; sıcaklık önce
     * hesaplanmalıdır, aksi hâlde basınç ve nem hesapları bozulur. */
    filter_sensor_value(bme280_sicaklik_hesapla(adc_T), 7,  8);
    filter_sensor_value(bme280_basinc_hesapla(adc_P),   7,  9);
    filter_sensor_value(bme280_nem_hesapla(adc_H),      7, 10);
}


static int kanal_oku_yazdir(uint8_t kanal, uint8_t ondalik,
                             const char *birim, float *p_out)
{
    float deger = 0.0f;
    int   ret   = buffer_peek_latest(&circ_buf[kanal], &deger);
    if (ret == 0) {
        if (p_out) *p_out = deger;
        Serial.print(deger, ondalik);
    } else {
        Serial.print("(veri yok)");
    }
    if (birim) Serial.print(birim);
    return ret;
}

static void seri_yazdir()
{
    float pa = 0.0f;

    /* ── MPL3115A2 ─────────────────────────────────────────── */
    Serial.println("---- MPL3115A2 (0x60) ----");

    Serial.print("  Basinc  : ");
    kanal_oku_yazdir(0, 2, " Pa", &pa);
    Serial.print(" | "); Serial.print(pa / 100.0f, 4); Serial.println(" hPa");

    Serial.print("  Sicaklik: ");
    kanal_oku_yazdir(1, 4, " C", NULL);
    Serial.println();

    Serial.println("--------------------------");

    /* ── ICG-20660L ────────────────────────────────────────── */
    Serial.println("---- ICG-20660L (0x69) ----");

    Serial.print("  Ivme X: "); kanal_oku_yazdir(2, 4, " g", NULL);
    Serial.print("   Y: ");     kanal_oku_yazdir(3, 4, " g", NULL);
    Serial.print("   Z: ");     kanal_oku_yazdir(4, 4, " g", NULL);
    Serial.println();

    Serial.print("  Gyro X: "); kanal_oku_yazdir(5, 3, " d/s", NULL);
    Serial.print("   Y: ");     kanal_oku_yazdir(6, 3, " d/s", NULL);
    Serial.print("   Z: ");     kanal_oku_yazdir(7, 3, " d/s", NULL);
    Serial.println();

    Serial.println("---------------------------");

    /* ── BME280 ─────────────────────────────────────────────── */
    Serial.println("---- BME280 (0x76) --------");

    Serial.print("  Sicaklik: ");
    kanal_oku_yazdir(8, 2, " C", NULL);
    Serial.println();

    Serial.print("  Basinc  : ");
    kanal_oku_yazdir(9, 2, " Pa", &pa);
    Serial.print(" | "); Serial.print(pa / 100.0f, 4); Serial.println(" hPa");

    Serial.print("  Nem     : ");
    kanal_oku_yazdir(10, 2, " %", NULL);
    Serial.println();

    Serial.println("---------------------------");
}



/*
 *  BUFFER SNAPSHOT  –  tampona dokunmadan kopyasını al
 *
 *  istatistik_hesapla() tamponu tüketmeden okuyabilmek için önce snapshot
 *  alır. Böylece mevcut buffer verisi korunur
 *  mevcut içeriğini FIFO sırasında out_arr dizisine kopyalar.
  */

static uint8_t buffer_snapshot(const struct buf_handle_t *p_handle,
                                float *out_arr, uint8_t max_len)
{
    if (p_handle == NULL || out_arr == NULL || p_handle->count == 0)
        return 0;

    uint8_t n   = (p_handle->count < max_len) ? p_handle->count : max_len;
    uint8_t idx = p_handle->tail;

    for (uint8_t i = 0; i < n; i++) {
        out_arr[i] = p_handle->data[idx];
        idx = (idx + 1) % CIRC_BUF_SIZE;
    }
    return n;
}

 //  İSTATİSTİK HESAPLAMA



static void istatistik_hesapla(const float *arr, uint8_t n,
                                struct kanal_istat_t *out)
{
    if (n == 0 || arr == NULL || out == NULL) {
        if (out) { out->min = out->max = out->median = out->stddev = 0.0f; }
        return;
    }

    /* Sıralama için yerel kopya */
    float tmp[CIRC_BUF_SIZE];
    for (uint8_t i = 0; i < n; i++) tmp[i] = arr[i];
    insertion_sort(tmp, n);   /* zaten tanımlı */

    out->min    = tmp[0];
    out->max    = tmp[n - 1];
    out->median = (n % 2 == 1) ? tmp[n / 2]
                               : (tmp[n / 2 - 1] + tmp[n / 2]) * 0.5f;

    /* Standart sapma (popülasyon) */
    float sum = 0.0f;
    for (uint8_t i = 0; i < n; i++) sum += arr[i];
    float mean = sum / (float)n;

    float var = 0.0f;
    for (uint8_t i = 0; i < n; i++) {
        float d = arr[i] - mean;
        var += d * d;
    }
    out->stddev = sqrtf(var / (float)n);
}


static void buffer_istat_yazdir(uint8_t kanal)
{
    if (kanal >= CIRC_CHANNELS) return;

    float   snap[CIRC_BUF_SIZE];
    uint8_t n = buffer_snapshot(&circ_buf[kanal], snap, CIRC_BUF_SIZE);

    Serial.print("  [K");
    if (kanal < 10) Serial.print("0");
    Serial.print(kanal);
    Serial.print("] ");
    Serial.print(kanal_adi[kanal]);
    Serial.print(" n=");
    Serial.print(n);

    if (n == 0) {
        Serial.println(" | (bos)");
        return;
    }

    struct kanal_istat_t ist;
    istatistik_hesapla(snap, n, &ist);

    Serial.print(" | Min=");  Serial.print(ist.min,    4);
    Serial.print(" Max=");    Serial.print(ist.max,    4);
    Serial.print(" Med=");    Serial.print(ist.median, 4);
    Serial.print(" Std=");    Serial.println(ist.stddev, 4);
}

/* ── tum_bufferlar_yazdir ──────────────────────────────────────────────── */
static void tum_bufferlar_yazdir()
{
    Serial.println("╔══ BUFFER İSTATİSTİKLERİ ═════════════════════════╗");
    for (uint8_t k = 0; k < CIRC_CHANNELS; k++) {
        buffer_istat_yazdir(k);
    }
    Serial.println("╚══════════════════════════════════════════════════╝");
}

/* ══════════════════════════════════════════════════════════════════════════
 *  BLE BAŞLAT  –  Nordic UART Service (NUS)
 * ══════════════════════════════════════════════════════════════════════════ */

static void ble_baslat()
{
    NimBLEDevice::init(BLE_DEVICE_NAME);
    NimBLEDevice::setPower(ESP_PWR_LVL_P9);
    NimBLEDevice::setMTU(BLE_MTU);       /* MTU 256: uzun string tek pakette sığar */

    ble_server = NimBLEDevice::createServer();
    ble_server->setCallbacks(new BleCallbacks());

    /* NUS Servisi */
    NimBLEService *svc = ble_server->createService(NUS_SVC_UUID);

    /* TX Characteristic: ESP32 → Telefon (Notify) */
    ble_tx_char = svc->createCharacteristic(
        NUS_TX_UUID,
        NIMBLE_PROPERTY::NOTIFY
    );

    /* RX Characteristic: Telefon → ESP32 (Write) — şimdilik kullanılmıyor */
    svc->createCharacteristic(
        NUS_RX_UUID,
        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR
    );

    svc->start();

    /* Advertise */
    NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();
    adv->addServiceUUID(NUS_SVC_UUID);
    adv->enableScanResponse(true);
    adv->start();

    Serial.print("[BLE] Cihaz adı: "); Serial.println(BLE_DEVICE_NAME);
    Serial.print("[BLE] MTU      : "); Serial.println(BLE_MTU);
    Serial.println("[BLE] NUS Advertise baslatildi");
    Serial.println("[BLE] Telefonda 'Serial Bluetooth Terminal' uygulamasini ac");
}


static void ble_istatistik_gonder()
{
    Serial.println("╔══ BLE İSTATİSTİK GÖNDERİMİ ══════════════════════╗");

    /* Başlık satırı */
    const char *baslik = "=== ESP32 Sensor Istatistikleri ===\r\n";
    Serial.print(baslik);
    if (ble_connected) {
        ble_tx_char->setValue((uint8_t *)baslik, strlen(baslik));
        ble_tx_char->notify();
        delay(40);
    }

    for (uint8_t k = 0; k < CIRC_CHANNELS; k++) {

        /* 1) Snapshot */
        float   snap[CIRC_BUF_SIZE];
        uint8_t n = buffer_snapshot(&circ_buf[k], snap, CIRC_BUF_SIZE);

        if (n == 0) {
            Serial.print("K"); Serial.print(k);
            Serial.println(" veri yok");
            continue;
        }

        /* 2) İstatistik */
        struct kanal_istat_t ist;
        istatistik_hesapla(snap, n, &ist);

        /* 3) dtostrf ile float → string (ESP32'de %f güvenilmez)
         *    Her değer için ayrı tampon, sonra birleştir
         */
        char s_min[12], s_max[12], s_med[12], s_std[12];
        dtostrf(ist.min,    8, 2, s_min);
        dtostrf(ist.max,    8, 2, s_max);
        dtostrf(ist.median, 8, 2, s_med);
        dtostrf(ist.stddev, 8, 4, s_std);

        /* Satır: "K00 MPL-Basinc   Min:99800.12 Max:99850.34 Med:99825.00 Std:  12.3456" */
        char satir[96];
        satir[0] = '\0';
        strcat(satir, "K");

        /* kanal numarası */
        char kno[4];
        if (k < 10) { kno[0]='0'; kno[1]='0'+k; kno[2]=' '; kno[3]='\0'; }
        else         { kno[0]='1'; kno[1]='0'+(k-10); kno[2]=' '; kno[3]='\0'; }
        strcat(satir, kno);

        /* kanal adı (ilk 12 karakter) */
        char kad[13];
        strncpy(kad, kanal_adi[k], 12); kad[12] = '\0';
        strcat(satir, kad);

        strcat(satir, " Min:");  strcat(satir, s_min);
        strcat(satir, " Max:");  strcat(satir, s_max);
        strcat(satir, " Med:");  strcat(satir, s_med);
        strcat(satir, " Std:");  strcat(satir, s_std);
        strcat(satir, "\r\n");

        /* 4) Seri çıktı — önce seri ekrana yaz, değerler doğruysa BLE'ye git */
        Serial.print("  "); Serial.print(satir);

        /* 5) BLE gönder */
        if (ble_connected) {
            ble_tx_char->setValue((uint8_t *)satir, strlen(satir));
            ble_tx_char->notify();
            delay(40);
        }
    }

    /* Bitiş */
    const char *bitis = "====================================\r\n";
    Serial.print(bitis);
    if (ble_connected) {
        ble_tx_char->setValue((uint8_t *)bitis, strlen(bitis));
        ble_tx_char->notify();
        delay(40);
    }

    Serial.println("╚══════════════════════════════════════════════════╝");
}



void setup()
{
    Serial.begin(115200);
    delay(1000);

    memset(circ_buf, 0, sizeof(circ_buf));
    memset(f_buf,    0, sizeof(f_buf));
    memset(f_idx,    0, sizeof(f_idx));
    memset(f_cnt,    0, sizeof(f_cnt));

    Wire.begin();   /* ESP32: SDA=21, SCL=22 */

    Serial.println("=== I2C Sensorler Baslatiliyor ===");
    mpl3115a2_baslat();
    icg20660l_baslat();
    bme280_baslat();

    Serial.println("=== BLE Baslatiliyor ===");
    ble_baslat();

    Serial.println("=== Okuma dongusu basliyor (1 Hz, BLE her 30 s) ===\n");
}

void loop()
{
    /* 1) Tüm sensörleri oku → filtrele → buffer'a yaz */
    oku_mpl3115a2();
    oku_icg20660l();
    oku_bme280();

    /* 2) Anlık değeri seri ekrana yaz (peek: buffer korunur) */
    seri_yazdir();
    Serial.println();

    okuma_sayaci++;

    /* 3) Her 30 saniyede:
     *      a) Buffer istatistiklerini (min/max/medyan/std) seri ekrana yaz
     *      b) Aynı istatistikleri BLE notify ile gönder
     *      c) Sayacı sıfırla — buffer sıfırlanmaz, veri birikmaya devam eder
     */
    if (okuma_sayaci >= BLE_SEND_PERIYOT) {
        okuma_sayaci = 0;
        tum_bufferlar_yazdir();   /* Seri: her kanal için min/max/medyan/std */
        Serial.println();
        ble_istatistik_gonder();  /* BLE: aynı istatistikleri notify gönder  */
        Serial.println();
    }

    delay(1000);   /* 1 Hz örnekleme */
}
