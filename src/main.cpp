#include <Arduino.h>
#include "BluetoothA2DPSource.h"
#include <Preferences.h>
#include <driver/i2s.h>
#include <esp_gap_bt_api.h>
#include <esp_bt.h>
#include <esp_bt_main.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/stream_buffer.h>
#include <freertos/timers.h>


// ============================================================
// MAESTRO BT V7 - 3 zapamietane odbiorniki + szybki manager
// ============================================================

static volatile bool btStackReady = false;
static volatile bool btDiscoveryStoppedEvent = false;

class MaestroA2DPSource : public BluetoothA2DPSource
{
public:
    bool readStoredLibraryLastConnection(esp_bd_addr_t out)
    {
        init_nvs();
        esp_bd_addr_t tmp = {0,0,0,0,0,0};
        if (!read_address(last_bda_nvs_name(), tmp))
            return false;
        memcpy(out, tmp, ESP_BD_ADDR_LEN);
        return true;
    }

    void rememberLibraryLastConnection(const esp_bd_addr_t mac)
    {
        esp_bd_addr_t tmp;
        memcpy(tmp, mac, ESP_BD_ADDR_LEN);
        set_last_connection(tmp);
    }

    esp_err_t startManagedDiscovery(uint8_t inquiryLen = 4)
    {
        if (discovery_active)
            esp_bt_gap_cancel_discovery();

        is_end = false;
        btDiscoveryStoppedEvent = false;
        s_a2d_state = APP_AV_STATE_DISCOVERING;

        return esp_bt_gap_start_discovery(
            ESP_BT_INQ_MODE_GENERAL_INQUIRY,
            inquiryLen,
            0
        );
    }

    void stopManagedDiscovery()
    {
        if (discovery_active)
            esp_bt_gap_cancel_discovery();
    }

    // APP_AV_MEDIA_STATE_* jest prywatnym enumem w BluetoothA2DPSource.cpp,
    // ale s_media_state jest chronionym intem:
    //   0 = IDLE
    //   1 = STARTING
    //   2 = STARTED
    //   3 = STOPPING
    //
    // Po reconnect zerujemy tylko stan TRANSPORTU MEDIA, nie caly stos BT.
    // To usuwa pozostalosci poprzedniej sesji A2DP.
    void resetMediaForReconnect()
    {
        s_media_state = 0; // APP_AV_MEDIA_STATE_IDLE
        s_intv_cnt = 0;
        s_pkt_cnt = 0;
        s_connecting_heatbeat_count = 0;
    }

    int mediaStateCode() const
    {
        return s_media_state;
    }

    bool mediaIsIdle() const
    {
        return s_media_state == 0;
    }

    // Przerywa probe DOKLADNIE do podanego MAC.
    // Nie uzywamy BluetoothA2DPCommon::disconnect(), bo ono korzysta
    // z last_connection, ktore przy MRU #1/#2/#3 moze wskazywac inny peer.
    bool abortPeer(const esp_bd_addr_t mac)
    {
        esp_bd_addr_t tmp;
        memcpy(tmp, mac, ESP_BD_ADDR_LEN);

        const esp_err_t err = esp_a2d_disconnect(tmp);

        if (err == ESP_OK)
            s_a2d_state = APP_AV_STATE_DISCONNECTING;

        return err == ESP_OK;
    }

    // Po recznym deinit Bluedroid/kontrolera trzeba zsynchronizowac
    // wewnetrzne flagi obiektu biblioteki z rzeczywistym stanem IDF.
    // Inaczej start() moglby uznac, ze Bluedroid nadal jest zainicjalizowany.
    void prepareForColdBtCoreStart()
    {
        is_bluedroid_initialized = false;
        is_start_disabled = false;
        is_end = false;

        reconnect_status = NoReconnect;
        is_autoreconnect_allowed = false;
        reconnect_retries = 0;

        connection_state = ESP_A2D_CONNECTION_STATE_DISCONNECTED;
        audio_state = ESP_A2D_AUDIO_STATE_SUSPEND;

        s_a2d_state = APP_AV_STATE_IDLE;
        s_a2d_last_state = APP_AV_STATE_IDLE;
        s_media_state = 0;
        s_intv_cnt = 0;
        s_pkt_cnt = 0;
        s_connecting_heatbeat_count = 0;
        last_heart_beat = 0;

        discovery_active = false;
        is_target_status_active = true;
    }

    bool is_valid_cod_service(uint32_t cod) override
    {
        if (!esp_bt_gap_is_valid_cod(cod))
            return false;

        if (esp_bt_gap_get_cod_major_dev(cod) != ESP_BT_COD_MAJOR_DEV_AV)
            return false;

        const uint32_t srvc = esp_bt_gap_get_cod_srvc(cod);
        return (srvc & (ESP_BT_COD_SRVC_RENDERING |
                        ESP_BT_COD_SRVC_AUDIO)) != 0;
    }

protected:
    // W oryginalnej bibliotece ten etap zawiera delay 10 s.
    // Tu uruchamiamy stos od razu i NIE startujemy discovery w tle.
    void av_hdl_stack_evt(uint16_t event, void *p_param) override
    {
        (void)p_param;
        if (event != 0)
            return;

        esp_bt_gap_set_device_name(dev_name);
        esp_bt_gap_register_callback(ccall_app_gap_callback);

        esp_avrc_ct_init();
        esp_avrc_ct_register_callback(ccall_app_rc_ct_callback);

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 0, 0)
        if (is_passthru_active)
        {
            esp_avrc_tg_init();
            esp_avrc_tg_register_callback(ccall_app_rc_tg_callback);
        }
#endif

        esp_a2d_source_init();
        esp_a2d_register_callback(&ccall_app_a2d_callback);
        esp_a2d_source_register_data_callback(&ccall_bt_app_a2d_data_cb);

        set_scan_mode_connectable(false);

        reconnect_status = NoReconnect;
        is_autoreconnect_allowed = false;
        reconnect_retries = 0;
        s_a2d_state = APP_AV_STATE_UNCONNECTED;
        is_end = false;

        if (s_tmr == nullptr)
        {
            s_tmr = xTimerCreate(
                "connTmr",
                (10000 / portTICK_PERIOD_MS),
                pdTRUE,
                nullptr,
                ccall_a2d_app_heart_beat
            );

            if (s_tmr != nullptr)
                xTimerStart(s_tmr, portMAX_DELAY);
        }

        btStackReady = true;
        Serial.println("[BT STACK] READY - fast custom manager.");
    }

    // Zatrzymujemy automatyczne ponawianie discovery biblioteki.
    void app_gap_callback(esp_bt_gap_cb_event_t event,
                          esp_bt_gap_cb_param_t *param) override
    {
        if (event == ESP_BT_GAP_DISC_STATE_CHANGED_EVT)
        {
            if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STARTED)
            {
                discovery_active = true;
                if (discovery_mode_callback)
                    discovery_mode_callback(ESP_BT_GAP_DISCOVERY_STARTED);
            }
            else
            {
                discovery_active = false;
                if (discovery_mode_callback)
                    discovery_mode_callback(ESP_BT_GAP_DISCOVERY_STOPPED);

                s_a2d_state = APP_AV_STATE_UNCONNECTED;
                btDiscoveryStoppedEvent = true;
            }
            return;
        }

        BluetoothA2DPSource::app_gap_callback(event, param);
    }
};

static MaestroA2DPSource a2dp;
static Preferences btPrefs;

static volatile bool btConnected = false;
static volatile bool btAudioStarted = false;
static volatile bool btPcmFlushRequested = false;

// Po utracie aktywnego odbiornika restartujemy CALY rdzen Classic BT:
// A2DP/AVRCP + Bluedroid + kontroler Bluetooth.
// Nie zwalniamy pamieci BT i nie resetujemy ESP32, wiec start() moze
// wszystko ponownie zainicjalizowac. Preferences/NVS/MRU-3 zostaja.
static volatile bool btProfileRestartRequested = false;
static bool btProfileRestartInProgress = false;
static uint32_t btProfileRestartCount = 0;

static constexpr uint32_t BT_PROFILE_RESTART_SETTLE_MS = 350;
static constexpr uint32_t BT_CORE_STATE_WAIT_MS = 1500;

static constexpr uint32_t A2DP_SAMPLE_RATE = 44100;

// ============================================================
// 3 ostatnio POPRAWNIE polaczone odbiorniki - MRU
// ============================================================

static constexpr uint8_t KNOWN_DEVICE_COUNT = 3;
static constexpr size_t BT_NAME_LEN = 64;

struct KnownBtDevice
{
    bool valid = false;
    esp_bd_addr_t mac = {0,0,0,0,0,0};
    char name[BT_NAME_LEN] = {0};
};

static KnownBtDevice knownDevices[KNOWN_DEVICE_COUNT];

static esp_bd_addr_t pendingPeerMac = {0,0,0,0,0,0};
static char pendingPeerName[BT_NAME_LEN] = {0};
static bool pendingPeerValid = false;

// Kandydat znaleziony podczas discovery.
static bool scanCandidateValid = false;
static bool scanCandidateWasKnown = false;
static esp_bd_addr_t scanCandidateMac = {0,0,0,0,0,0};
static char scanCandidateName[BT_NAME_LEN] = {0};
static int scanCandidateRssi = -127;

enum BtManagerState : uint8_t
{
    BTM_WAIT_STACK = 0,
    BTM_PAUSE,
    BTM_TRY_KNOWN,
    BTM_WAIT_KNOWN,
    BTM_WAIT_LINK_IDLE,
    BTM_SCAN,
    BTM_WAIT_SCAN_CONNECT,
    BTM_CONNECTED,
    BTM_PROFILE_RESTART
};

static BtManagerState btManagerState = BTM_WAIT_STACK;
static uint8_t btKnownIndex = 0;
static uint32_t btStateStartedAt = 0;
static uint32_t btPauseUntil = 0;
static bool btAttemptFailed = false;
static bool btBondImportDone = false;
static uint32_t btManagerCycle = 0;

// 0..2 = nastepny slot MRU; 0xFF = po czystym DISCONNECTED wejdz w scan.
static uint8_t btNextKnownAfterIdle = 0xFF;
static bool btIdleObserved = false;
static uint32_t btIdleObservedAt = 0;

// Ochrona przed spoznionym CONNECTED po wymuszonym timeout/abort.
static volatile bool btRejectLateConnected = false;
static volatile bool btLateDisconnectRequested = false;
static esp_bd_addr_t btCancelledPeerMac = {0, 0, 0, 0, 0, 0};

static constexpr uint32_t BT_STACK_SETTLE_MS = 250;
// Classic BT/A2DP potrafi potrzebowac wiecej niz 4.5 s.
static constexpr uint32_t BT_KNOWN_CONNECT_TIMEOUT_MS = 12000;
static constexpr uint32_t BT_LINK_IDLE_SETTLE_MS = 650;
static constexpr uint32_t BT_ABORT_SETTLE_MS = 300;
static constexpr uint32_t BT_AFTER_DISCONNECT_RETRY_MS = 600;
static constexpr uint32_t BT_SCAN_RESTART_DELAY_MS = 800;
static constexpr uint8_t BT_PAIR_SCAN_INQUIRY_LEN = 4; // ~5.12 s
static constexpr int BT_PAIR_MIN_RSSI = -78;

static volatile bool btAudioKickPending = false;
static bool btAudioKickOutstanding = false;
static uint32_t btConnectedAt = 0;
static uint32_t btLastAudioKickAt = 0;
static uint8_t btAudioKickCount = 0;
static constexpr uint32_t BT_AUDIO_KICK_FIRST_MS = 250;
static constexpr uint32_t BT_AUDIO_KICK_RETRY_MS = 4000;
static constexpr uint8_t BT_AUDIO_KICK_MAX = 2;

// ============================================================
// Pomocnicze MAC / NVS
// ============================================================

static bool macIsZero(const esp_bd_addr_t mac)
{
    for (int i = 0; i < ESP_BD_ADDR_LEN; ++i)
        if (mac[i] != 0)
            return false;
    return true;
}

static bool macEqual(const esp_bd_addr_t a, const esp_bd_addr_t b)
{
    return memcmp(a, b, ESP_BD_ADDR_LEN) == 0;
}

static void formatMac(const esp_bd_addr_t mac, char *out, size_t outSize)
{
    snprintf(out, outSize,
             "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2],
             mac[3], mac[4], mac[5]);
}

static void printMac(const esp_bd_addr_t mac)
{
    char txt[18];
    formatMac(mac, txt, sizeof(txt));
    Serial.print(txt);
}

static const char *connectionStateName(esp_a2d_connection_state_t state)
{
    switch (state)
    {
        case ESP_A2D_CONNECTION_STATE_DISCONNECTED:  return "DISCONNECTED";
        case ESP_A2D_CONNECTION_STATE_CONNECTING:    return "CONNECTING";
        case ESP_A2D_CONNECTION_STATE_CONNECTED:     return "CONNECTED";
        case ESP_A2D_CONNECTION_STATE_DISCONNECTING: return "DISCONNECTING";
        default:                                     return "UNKNOWN";
    }
}

static int findKnownDevice(const esp_bd_addr_t mac)
{
    for (int i = 0; i < KNOWN_DEVICE_COUNT; ++i)
        if (knownDevices[i].valid &&
            macEqual(knownDevices[i].mac, mac))
            return i;
    return -1;
}

static int countKnownDevices()
{
    int n = 0;
    for (int i = 0; i < KNOWN_DEVICE_COUNT; ++i)
        if (knownDevices[i].valid)
            ++n;
    return n;
}

static void saveKnownDevices()
{
    for (int i = 0; i < KNOWN_DEVICE_COUNT; ++i)
    {
        char km[12], kn[12];
        snprintf(km, sizeof(km), "k%d_mac", i);
        snprintf(kn, sizeof(kn), "k%d_name", i);

        if (knownDevices[i].valid)
        {
            btPrefs.putBytes(km, knownDevices[i].mac, ESP_BD_ADDR_LEN);
            btPrefs.putString(kn, knownDevices[i].name);
        }
        else
        {
            btPrefs.remove(km);
            btPrefs.remove(kn);
        }
    }
}

static void appendKnownIfFree(const esp_bd_addr_t mac, const char *name = nullptr)
{
    if (macIsZero(mac) || findKnownDevice(mac) >= 0)
        return;

    for (int i = 0; i < KNOWN_DEVICE_COUNT; ++i)
    {
        if (!knownDevices[i].valid)
        {
            knownDevices[i].valid = true;
            memcpy(knownDevices[i].mac, mac, ESP_BD_ADDR_LEN);

            if (name && name[0])
            {
                strncpy(knownDevices[i].name, name,
                        sizeof(knownDevices[i].name) - 1);
            }
            return;
        }
    }
}

static void loadKnownDevices()
{
    btPrefs.begin("maestro-bt", false);

    for (int i = 0; i < KNOWN_DEVICE_COUNT; ++i)
    {
        knownDevices[i] = {};

        char km[12], kn[12];
        snprintf(km, sizeof(km), "k%d_mac", i);
        snprintf(kn, sizeof(kn), "k%d_name", i);

        if (btPrefs.isKey(km) &&
            btPrefs.getBytesLength(km) == ESP_BD_ADDR_LEN)
        {
            btPrefs.getBytes(km, knownDevices[i].mac, ESP_BD_ADDR_LEN);
            knownDevices[i].valid = !macIsZero(knownDevices[i].mac);
        }

        if (knownDevices[i].valid && btPrefs.isKey(kn))
        {
            String n = btPrefs.getString(kn, "");
            strncpy(knownDevices[i].name, n.c_str(),
                    sizeof(knownDevices[i].name) - 1);
        }
    }

    // Migracja z V6.6/V6.7: good_mac -> slot #1.
    if (countKnownDevices() == 0 &&
        btPrefs.isKey("good_mac") &&
        btPrefs.getBytesLength("good_mac") == ESP_BD_ADDR_LEN)
    {
        esp_bd_addr_t oldMac = {0,0,0,0,0,0};
        btPrefs.getBytes("good_mac", oldMac, ESP_BD_ADDR_LEN);

        String oldName = btPrefs.isKey("good_name")
            ? btPrefs.getString("good_name", "")
            : "";

        appendKnownIfFree(oldMac, oldName.c_str());
        saveKnownDevices();
    }
}

static void bootstrapLegacyLastDevice()
{
    esp_bd_addr_t legacy = {0,0,0,0,0,0};

    if (a2dp.readStoredLibraryLastConnection(legacy) &&
        !macIsZero(legacy))
    {
        appendKnownIfFree(legacy, nullptr);
        saveKnownDevices();
    }
}

static void importBondedDevices()
{
    if (btBondImportDone)
        return;

    btBondImportDone = true;

    const int count = esp_bt_gap_get_bond_device_num();
    Serial.printf("[BT BOND] Bonded devices: %d\n", count);

    if (count <= 0)
        return;

    esp_bd_addr_t *list = new esp_bd_addr_t[count];
    if (!list)
        return;

    int n = count;

    if (esp_bt_gap_get_bond_device_list(&n, list) == ESP_OK)
    {
        for (int i = 0; i < n; ++i)
        {
            char txt[18];
            formatMac(list[i], txt, sizeof(txt));
            Serial.printf("[BT BOND] #%d %s\n", i + 1, txt);
            appendKnownIfFree(list[i], nullptr);
        }
        saveKnownDevices();
    }

    delete[] list;
}

static void printKnownDevices()
{
    Serial.println("[BT MRU] Remembered receivers:");

    for (int i = 0; i < KNOWN_DEVICE_COUNT; ++i)
    {
        if (!knownDevices[i].valid)
        {
            Serial.printf("[BT MRU] #%d -- empty --\n", i + 1);
            continue;
        }

        char txt[18];
        formatMac(knownDevices[i].mac, txt, sizeof(txt));

        Serial.printf("[BT MRU] #%d %s | %s\n",
                      i + 1,
                      knownDevices[i].name[0]
                          ? knownDevices[i].name
                          : "(name unknown)",
                      txt);
    }
}

static void promoteKnownDevice(const esp_bd_addr_t mac, const char *name)
{
    KnownBtDevice promoted = {};
    promoted.valid = true;
    memcpy(promoted.mac, mac, ESP_BD_ADDR_LEN);

    const int oldIndex = findKnownDevice(mac);

    if (name && name[0])
    {
        strncpy(promoted.name, name, sizeof(promoted.name) - 1);
    }
    else if (oldIndex >= 0 && knownDevices[oldIndex].name[0])
    {
        strncpy(promoted.name, knownDevices[oldIndex].name,
                sizeof(promoted.name) - 1);
    }

    KnownBtDevice old[KNOWN_DEVICE_COUNT];
    for (int i = 0; i < KNOWN_DEVICE_COUNT; ++i)
        old[i] = knownDevices[i];

    knownDevices[0] = promoted;
    int dst = 1;

    for (int i = 0; i < KNOWN_DEVICE_COUNT && dst < KNOWN_DEVICE_COUNT; ++i)
    {
        if (!old[i].valid || macEqual(old[i].mac, mac))
            continue;
        knownDevices[dst++] = old[i];
    }

    while (dst < KNOWN_DEVICE_COUNT)
        knownDevices[dst++] = {};

    saveKnownDevices();

    char txt[18];
    formatMac(mac, txt, sizeof(txt));
    Serial.printf("[BT MRU] Promoted to #1: %s | %s\n",
                  promoted.name[0] ? promoted.name : "(name unknown)",
                  txt);

    printKnownDevices();
}

static void setPendingPeer(const esp_bd_addr_t mac, const char *name)
{
    memcpy(pendingPeerMac, mac, ESP_BD_ADDR_LEN);
    pendingPeerValid = true;
    pendingPeerName[0] = '\0';

    if (name && name[0])
    {
        strncpy(pendingPeerName, name, sizeof(pendingPeerName) - 1);
        pendingPeerName[sizeof(pendingPeerName) - 1] = '\0';
    }
}

// ============================================================
// I2S RX Z MAESTRO ESP32-S3
//
// Maestro GPIO06 BCLK  -> WROOM GPIO26
// Maestro GPIO16 LRCK  -> WROOM GPIO25
// Maestro GPIO15 DATA  -> WROOM GPIO22
// Maestro GND          -> WROOM GND
//
// WROOM pracuje jako I2S SLAVE / RX.
// ============================================================

static constexpr i2s_port_t I2S_RX_PORT = I2S_NUM_0;
static constexpr int I2S_RX_BCLK = 26;
static constexpr int I2S_RX_LRCK = 25;
static constexpr int I2S_RX_DATA = 22;

// W trybie SLAVE wartosc sample_rate w konfiguracji sterownika jest tylko
// wartoscia nominalna. Rzeczywisty zegar LRCK/BCLK dostarcza Maestro.
static constexpr uint32_t I2S_DRIVER_NOMINAL_RATE = 48000;
static constexpr size_t I2S_STREAM_BUFFER_SIZE = 32768;

enum InputRateMode : uint8_t
{
    INPUT_RATE_UNKNOWN = 0,
    INPUT_RATE_44100,
    INPUT_RATE_48000
};

static volatile InputRateMode inputRateMode = INPUT_RATE_UNKNOWN;
static volatile uint32_t measuredI2sRate = 0;
static volatile uint32_t inputRateGeneration = 0;

static StreamBufferHandle_t i2sStream = nullptr;

static volatile uint32_t i2sBytesReceived = 0;
static volatile uint32_t i2sBytesForwarded = 0;
static volatile uint32_t i2sBytesDropped = 0;
static volatile uint32_t a2dpUnderruns = 0;

// Ile ramek na sekunde realnie pobiera stos A2DP.
static volatile uint32_t a2dpFramesRequested = 0;
static volatile uint32_t a2dpFramesWithPcm = 0;


// Dzwiek powitalny usuniety - audio z Maestro startuje bezposrednio po polaczeniu A2DP.


static bool initI2sReceiver()
{
    i2s_config_t config = {};
    config.mode = (i2s_mode_t)(I2S_MODE_SLAVE | I2S_MODE_RX);
    config.sample_rate = I2S_DRIVER_NOMINAL_RATE;
    config.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
    config.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;
    config.communication_format = I2S_COMM_FORMAT_STAND_I2S;
    config.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
    config.dma_buf_count = 8;
    config.dma_buf_len = 256;
    config.use_apll = false;
    config.tx_desc_auto_clear = false;
    config.fixed_mclk = 0;

    i2s_pin_config_t pins = {};
    pins.bck_io_num = I2S_RX_BCLK;
    pins.ws_io_num = I2S_RX_LRCK;
    pins.data_out_num = I2S_PIN_NO_CHANGE;
    pins.data_in_num = I2S_RX_DATA;

    esp_err_t err = i2s_driver_install(I2S_RX_PORT, &config, 0, nullptr);
    if (err != ESP_OK)
    {
        Serial.printf("[I2S] ERROR driver_install: %d\n", (int)err);
        return false;
    }

    err = i2s_set_pin(I2S_RX_PORT, &pins);
    if (err != ESP_OK)
    {
        Serial.printf("[I2S] ERROR set_pin: %d\n", (int)err);
        i2s_driver_uninstall(I2S_RX_PORT);
        return false;
    }

    i2s_zero_dma_buffer(I2S_RX_PORT);

    i2sStream = xStreamBufferCreate(I2S_STREAM_BUFFER_SIZE, 4);
    if (i2sStream == nullptr)
    {
        Serial.println("[I2S] ERROR: brak pamieci na bufor PCM");
        i2s_driver_uninstall(I2S_RX_PORT);
        return false;
    }

    Serial.println("[I2S] RX READY");
    Serial.printf("[I2S] BCLK GPIO%d | LRCK GPIO%d | DATA GPIO%d\n",
                  I2S_RX_BCLK, I2S_RX_LRCK, I2S_RX_DATA);
    Serial.println("[I2S] Format: AUTO 44.1/48 kHz / 16-bit / stereo");
    return true;
}

static const char *inputRateName(InputRateMode mode)
{
    switch (mode)
    {
        case INPUT_RATE_44100: return "44.1 kHz";
        case INPUT_RATE_48000: return "48 kHz";
        default:               return "UNKNOWN";
    }
}

static InputRateMode classifyInputRate(uint32_t rate)
{
    // Celowo zostawiamy martwa strefe miedzy 44.1 i 48 kHz.
    // Gdy zmiana formatu wypadnie w srodku okna pomiarowego,
    // nie przelaczamy trybu na podstawie usrednionego wyniku.
    if (rate >= 42500 && rate <= 45500)
        return INPUT_RATE_44100;

    if (rate >= 46500 && rate <= 50000)
        return INPUT_RATE_48000;

    return INPUT_RATE_UNKNOWN;
}

static void i2sReceiverTask(void *)
{
    // 16-bit stereo = 4 bajty na jedna ramke PCM.
    static constexpr size_t PCM_FRAME_BYTES = sizeof(Frame);
    static constexpr uint32_t RATE_MEASURE_WINDOW_MS = 500;

    uint8_t temp[1024];
    uint32_t rateWindowStart = millis();
    uint32_t rateWindowFrames = 0;

    for (;;)
    {
        size_t bytesRead = 0;
        esp_err_t err = i2s_read(
            I2S_RX_PORT,
            temp,
            sizeof(temp),
            &bytesRead,
            portMAX_DELAY
        );

        if (err != ESP_OK || bytesRead == 0)
            continue;

        const size_t alignedRead = bytesRead - (bytesRead % PCM_FRAME_BYTES);
        if (alignedRead == 0)
            continue;

        i2sBytesReceived += alignedRead;
        rateWindowFrames += (uint32_t)(alignedRead / PCM_FRAME_BYTES);

        // --------------------------------------------------------
        // AUTODETEKCJA 44.1 / 48 kHz z rzeczywistego tempa I2S.
        // Nie analizujemy audio - mierzymy liczbe ramek taktowanych LRCK.
        // --------------------------------------------------------
        const uint32_t now = millis();
        const uint32_t elapsed = now - rateWindowStart;

        if (elapsed >= RATE_MEASURE_WINDOW_MS)
        {
            const uint32_t rate = (uint32_t)(
                ((uint64_t)rateWindowFrames * 1000ULL) / elapsed
            );

            measuredI2sRate = rate;
            const InputRateMode detected = classifyInputRate(rate);
            const InputRateMode oldMode = inputRateMode;

            if (detected != oldMode)
            {
                inputRateMode = detected;
                inputRateGeneration++;

                // Usuwamy probki poprzedniego formatu. Przy wyniku UNKNOWN
                // celowo przechodzimy na chwile w cisze zamiast odtwarzac dane
                // z niewlasciwym przelicznikiem. Czytnik A2DP po zmianie
                // generation wyzeruje rowniez lokalny cache i faze resamplera.
                if (i2sStream != nullptr)
                    xStreamBufferReset(i2sStream);

                Serial.printf(
                    "[I2S AUTO] RATE~%lu Hz -> %s (switch #%lu)\n",
                    (unsigned long)rate,
                    inputRateName(detected),
                    (unsigned long)inputRateGeneration
                );
            }

            rateWindowFrames = 0;
            rateWindowStart = now;
        }

        if (i2sStream == nullptr)
            continue;

        // Bufor PCM wlaczamy dopiero po AUDIO STARTED.
        // Sam BT CONNECTED nie oznacza jeszcze, ze A2DP pobiera audio.
        if (!btConnected || !btAudioStarted)
            continue;

        // Dopoki format nie jest rozpoznany, nie gromadzimy niepewnych danych.
        if (inputRateMode == INPUT_RATE_UNKNOWN)
            continue;

        // Nigdy nie zapisujemy niepelnej ramki stereo.
        size_t freeBytes = xStreamBufferSpacesAvailable(i2sStream);
        size_t toSend = (alignedRead < freeBytes) ? alignedRead : freeBytes;
        toSend -= (toSend % PCM_FRAME_BYTES);

        size_t sent = 0;
        if (toSend > 0)
            sent = xStreamBufferSend(i2sStream, temp, toSend, 0);

        sent -= (sent % PCM_FRAME_BYTES);
        i2sBytesForwarded += sent;

        if (sent < alignedRead)
            i2sBytesDropped += (alignedRead - sent);
    }
}


// ============================================================
// AUTOMATYCZNY ADAPTER CZESTOTLIWOSCI -> A2DP 44.1 kHz
// ============================================================
//
// Wejscie 44.1 kHz : krok 1/1       -> praktycznie BYPASS
// Wejscie 48 kHz   : krok 160/147   -> resampling 48000 -> 44100
//
// Tryb wybiera automatycznie pomiar wykonywany w i2sReceiverTask().
// ============================================================

static constexpr size_t RESAMPLE_PREBUFFER_BYTES = 4096;

static Frame resampleA = {};
static Frame resampleB = {};
static uint32_t resamplePhase = 0;
static bool resampleReady = false;
static bool directReady = false;

// Lokalny cache ogranicza liczbe wywolan FreeRTOS StreamBuffer.
static constexpr size_t PCM_READ_CACHE_FRAMES = 256;
static Frame pcmReadCache[PCM_READ_CACHE_FRAMES];
static size_t pcmReadCachePos = 0;
static size_t pcmReadCacheCount = 0;

static bool refillPcmReadCache()
{
    pcmReadCachePos = 0;
    pcmReadCacheCount = 0;

    if (i2sStream == nullptr)
        return false;

    size_t available = xStreamBufferBytesAvailable(i2sStream);
    size_t toRead = available;
    const size_t cacheBytes = sizeof(pcmReadCache);

    if (toRead > cacheBytes)
        toRead = cacheBytes;

    toRead -= (toRead % sizeof(Frame));
    if (toRead < sizeof(Frame))
        return false;

    size_t got = xStreamBufferReceive(
        i2sStream,
        reinterpret_cast<uint8_t *>(pcmReadCache),
        toRead,
        0
    );

    got -= (got % sizeof(Frame));
    pcmReadCacheCount = got / sizeof(Frame);
    return pcmReadCacheCount > 0;
}

static bool readPcmFrame(Frame &frame)
{
    if (pcmReadCachePos >= pcmReadCacheCount)
    {
        if (!refillPcmReadCache())
            return false;
    }

    frame = pcmReadCache[pcmReadCachePos++];
    return true;
}

static void resetInputReaderState()
{
    resampleA = {};
    resampleB = {};
    resamplePhase = 0;
    resampleReady = false;
    directReady = false;
    pcmReadCachePos = 0;
    pcmReadCacheCount = 0;
}

static void flushPcmAfterBtDisconnect()
{
    if (i2sStream != nullptr)
        xStreamBufferReset(i2sStream);

    resetInputReaderState();
    btPcmFlushRequested = false;

    Serial.println("[BT RECOVERY] PCM buffer cleared.");
}

// ------------------------------------------------------------
// 44.1 kHz -> 44.1 kHz: bez zmiany tempa i bez interpolacji.
// ------------------------------------------------------------
static bool getDirect44100Frame(Frame &out)
{
    if (!directReady)
    {
        if (i2sStream == nullptr ||
            xStreamBufferBytesAvailable(i2sStream) < RESAMPLE_PREBUFFER_BYTES)
            return false;

        // Cache sprzed zmiany formatu nie moze byc wykorzystany.
        pcmReadCachePos = 0;
        pcmReadCacheCount = 0;
        directReady = true;
    }

    if (!readPcmFrame(out))
    {
        directReady = false;
        return false;
    }

    return true;
}

// ------------------------------------------------------------
// 48 kHz -> 44.1 kHz: interpolacja liniowa 160/147.
// ------------------------------------------------------------
static bool start48000ResamplerIfPossible()
{
    if (resampleReady)
        return true;

    if (i2sStream == nullptr ||
        xStreamBufferBytesAvailable(i2sStream) < RESAMPLE_PREBUFFER_BYTES)
        return false;

    pcmReadCachePos = 0;
    pcmReadCacheCount = 0;

    if (!readPcmFrame(resampleA))
        return false;

    if (!readPcmFrame(resampleB))
        return false;

    resamplePhase = 0;
    resampleReady = true;
    return true;
}

static inline int16_t interpolate48000Sample(int16_t a, int16_t b, uint32_t phase)
{
    static constexpr int32_t DEN = 147;
    const int32_t aa = (int32_t)a;
    const int32_t diff = (int32_t)b - aa;
    const int32_t value = aa + (diff * (int32_t)phase) / DEN;

    if (value > 32767) return 32767;
    if (value < -32768) return -32768;
    return (int16_t)value;
}

static bool getResampled48000Frame(Frame &out)
{
    static constexpr uint32_t STEP_NUM = 160;
    static constexpr uint32_t STEP_DEN = 147;

    if (!start48000ResamplerIfPossible())
        return false;

    out.channel1 = interpolate48000Sample(resampleA.channel1, resampleB.channel1, resamplePhase);
    out.channel2 = interpolate48000Sample(resampleA.channel2, resampleB.channel2, resamplePhase);

    resamplePhase += STEP_NUM;
    uint32_t advance = resamplePhase / STEP_DEN;
    resamplePhase %= STEP_DEN;

    while (advance-- > 0)
    {
        resampleA = resampleB;

        if (!readPcmFrame(resampleB))
        {
            resampleReady = false;
            return false;
        }
    }

    return true;
}

static bool getAutoRateFrame(Frame &out)
{
    const InputRateMode mode = inputRateMode;

    if (mode == INPUT_RATE_44100)
        return getDirect44100Frame(out);

    if (mode == INPUT_RATE_48000)
        return getResampled48000Frame(out);

    return false;
}

static int32_t provideAudio(Frame *frames, int32_t frameCount)
{
    // Bez dzwieku powitalnego:
    // od pierwszego callbacku A2DP podajemy PCM z Maestro.
    static uint32_t seenRateGeneration = 0xFFFFFFFFUL;

    if (frameCount > 0)
        a2dpFramesRequested += (uint32_t)frameCount;

    for (int32_t i = 0; i < frameCount; ++i)
    {
        // Format moze zmienic sie nawet w trakcie jednego callbacku A2DP.
        const uint32_t generation = inputRateGeneration;
        if (seenRateGeneration != generation)
        {
            seenRateGeneration = generation;
            resetInputReaderState();
        }

        Frame out = {};

        if (getAutoRateFrame(out))
        {
            frames[i] = out;
            a2dpFramesWithPcm++;
        }
        else
        {
            frames[i].channel1 = 0;
            frames[i].channel2 = 0;
            a2dpUnderruns++;
        }
    }

    return frameCount;
}


// ============================================================
// BLUETOOTH MANAGER V7
// ============================================================

static void resetScanCandidate()
{
    scanCandidateValid = false;
    scanCandidateWasKnown = false;
    scanCandidateRssi = -127;
    scanCandidateName[0] = '\0';
    memset(scanCandidateMac, 0, ESP_BD_ADDR_LEN);
}

static void scheduleKnownSequence(uint32_t delayMs)
{
    btKnownIndex = 0;
    btAttemptFailed = false;
    btPauseUntil = millis() + delayMs;
    btManagerState = BTM_PAUSE;
}

static void startPairScan();

static uint8_t findNextKnownIndex(uint8_t afterIndex)
{
    for (uint8_t i = (uint8_t)(afterIndex + 1);
         i < KNOWN_DEVICE_COUNT;
         ++i)
    {
        if (knownDevices[i].valid)
            return i;
    }

    return 0xFF;
}

static void enterWaitLinkIdle(uint8_t nextKnown)
{
    btNextKnownAfterIdle = nextKnown;
    btIdleObserved = false;
    btIdleObservedAt = 0;
    btAttemptFailed = false;
    btManagerState = BTM_WAIT_LINK_IDLE;

    if (a2dp.get_connection_state() ==
        ESP_A2D_CONNECTION_STATE_DISCONNECTED)
    {
        btIdleObserved = true;
        btIdleObservedAt = millis();
    }
}

static bool startKnownAttempt(uint8_t index)
{
    if (index >= KNOWN_DEVICE_COUNT || !knownDevices[index].valid)
        return false;

    btRejectLateConnected = false;
    btLateDisconnectRequested = false;

    a2dp.stopManagedDiscovery();

    btKnownIndex = index;
    btAttemptFailed = false;
    btStateStartedAt = millis();

    setPendingPeer(knownDevices[index].mac, knownDevices[index].name);

    // Kazda nowa sesja A2DP zaczyna transport media od czystego IDLE.
    a2dp.resetMediaForReconnect();

    char txt[18];
    formatMac(knownDevices[index].mac, txt, sizeof(txt));

    Serial.printf("[BT KNOWN] TRY #%u/3: %s | %s\n",
                  (unsigned)(index + 1),
                  knownDevices[index].name[0]
                      ? knownDevices[index].name
                      : "(name unknown)",
                  txt);

    const bool ok = a2dp.connect_to(knownDevices[index].mac);

    if (!ok)
    {
        btAttemptFailed = true;
        Serial.println("[BT KNOWN] connect_to rejected immediately.");
        return false;
    }

    btManagerState = BTM_WAIT_KNOWN;
    return true;
}

static void tryNextKnownOrScan()
{
    enterWaitLinkIdle(findNextKnownIndex(btKnownIndex));
}

static void startPairScan()
{
    resetScanCandidate();

    btDiscoveryStoppedEvent = false;
    btAttemptFailed = false;

    Serial.println("[BT PAIR] No remembered receiver answered.");
    Serial.println("[BT PAIR] Scanning discoverable Audio/Video receivers...");

    const esp_err_t err =
        a2dp.startManagedDiscovery(BT_PAIR_SCAN_INQUIRY_LEN);

    if (err != ESP_OK)
    {
        Serial.printf("[BT PAIR] Scan start error=%d\n", (int)err);
        scheduleKnownSequence(BT_SCAN_RESTART_DELAY_MS);
        return;
    }

    btManagerState = BTM_SCAN;
    btStateStartedAt = millis();

    Serial.printf("[BT PAIR] Scan ~%.1f s | min RSSI %d dBm\n",
                  BT_PAIR_SCAN_INQUIRY_LEN * 1.28f,
                  BT_PAIR_MIN_RSSI);
}

// Biblioteka wywoluje callback tylko dla urzadzen zgodnych z filtrem
// Audio/Video. V7 NIE laczy sie z "pierwszym znalezionym".
// Zbiera kandydatow i wybiera najsilniejszy.
static bool deviceFound(const char *name, esp_bd_addr_t address, int rssi)
{
    if (btManagerState != BTM_SCAN)
        return false;

    const char *safeName = (name && name[0]) ? name : "(bez nazwy)";

    char txt[18];
    formatMac(address, txt, sizeof(txt));

    Serial.printf("[BT SCAN] %s | %s | RSSI %d dBm\n",
                  safeName, txt, rssi);

    const int knownIndex = findKnownDevice(address);

    // Zapamietane urzadzenie wykryte podczas skanu ma pierwszenstwo.
    if (knownIndex >= 0)
    {
        scanCandidateValid = true;
        scanCandidateWasKnown = true;
        scanCandidateRssi = rssi;
        memcpy(scanCandidateMac, address, ESP_BD_ADDR_LEN);

        const char *n = knownDevices[knownIndex].name[0]
            ? knownDevices[knownIndex].name
            : safeName;

        strncpy(scanCandidateName, n, sizeof(scanCandidateName) - 1);
        scanCandidateName[sizeof(scanCandidateName) - 1] = '\0';

        Serial.println("[BT SCAN] Remembered receiver visible -> stop scan.");
        esp_bt_gap_cancel_discovery();
        return false;
    }

    if (!name || !name[0] || rssi < BT_PAIR_MIN_RSSI)
    {
        Serial.println("[BT SCAN] Ignored: weak signal or no name.");
        return false;
    }

    if (!scanCandidateValid ||
        (!scanCandidateWasKnown && rssi > scanCandidateRssi))
    {
        scanCandidateValid = true;
        scanCandidateWasKnown = false;
        scanCandidateRssi = rssi;
        memcpy(scanCandidateMac, address, ESP_BD_ADDR_LEN);

        strncpy(scanCandidateName, name, sizeof(scanCandidateName) - 1);
        scanCandidateName[sizeof(scanCandidateName) - 1] = '\0';

        Serial.printf("[BT SCAN] Best candidate: %s | RSSI %d dBm\n",
                      scanCandidateName,
                      scanCandidateRssi);
    }

    // Zawsze false - manager laczy po zakonczeniu skanu.
    return false;
}

static void connectScanCandidate()
{
    btRejectLateConnected = false;
    btLateDisconnectRequested = false;

    if (!scanCandidateValid)
    {
        scheduleKnownSequence(BT_SCAN_RESTART_DELAY_MS);
        return;
    }

    setPendingPeer(scanCandidateMac, scanCandidateName);

    a2dp.resetMediaForReconnect();

    char txt[18];
    formatMac(scanCandidateMac, txt, sizeof(txt));

    Serial.printf("[BT PAIR] CONNECT: %s | %s | RSSI %d dBm%s\n",
                  scanCandidateName[0]
                      ? scanCandidateName
                      : "(name unknown)",
                  txt,
                  scanCandidateRssi,
                  scanCandidateWasKnown ? " | REMEMBERED" : " | NEW");

    btAttemptFailed = false;
    btStateStartedAt = millis();
    btManagerState = BTM_WAIT_SCAN_CONNECT;

    if (!a2dp.connect_to(scanCandidateMac))
    {
        Serial.println("[BT PAIR] connect_to rejected.");
        btAttemptFailed = true;
    }
}

static void connectionChanged(esp_a2d_connection_state_t state, void *)
{
    btConnected = (state == ESP_A2D_CONNECTION_STATE_CONNECTED);

    Serial.printf("[BT] STATE: %s\n", connectionStateName(state));

    if (state == ESP_A2D_CONNECTION_STATE_CONNECTED &&
        btRejectLateConnected)
    {
        btConnected = false;
        btAudioStarted = false;
        btAudioKickPending = false;
        btAudioKickOutstanding = false;
        btLateDisconnectRequested = true;

        Serial.println("[BT MANAGER] LATE CONNECTED after timeout -> reject.");
        Serial.println("[BT MANAGER] Audio will not start for cancelled attempt.");
        return;
    }

    if (btConnected)
    {
        btProfileRestartRequested = false;
        btAudioStarted = false;

        // Nie przenosimy zadnego PCM ani stanu resamplera ze starej sesji.
        if (i2sStream != nullptr)
            xStreamBufferReset(i2sStream);

        resetInputReaderState();
        btPcmFlushRequested = false;

        // CONNECTED zaczyna nowa sesje transportu media.
        // Stan MEDIA musi byc IDLE przed pierwszym CHECK_SRC_RDY.
        a2dp.resetMediaForReconnect();

        btAudioKickPending = true;
        btAudioKickOutstanding = false;
        btConnectedAt = millis();
        btLastAudioKickAt = 0;
        btAudioKickCount = 0;

        if (pendingPeerValid)
        {
            promoteKnownDevice(pendingPeerMac, pendingPeerName);
            a2dp.rememberLibraryLastConnection(pendingPeerMac);
            pendingPeerValid = false;
        }

        btManagerState = BTM_CONNECTED;
        btAttemptFailed = false;

        Serial.println("[BT] A2DP CONNECTED");
        Serial.println("[BUILD] V7.4 MRU-3 FULL-BT-CORE-RESTART");
        Serial.println("[BT] Waiting for AUDIO STARTED...");
        return;
    }

    if (state == ESP_A2D_CONNECTION_STATE_DISCONNECTED)
    {
        btAudioStarted = false;
        btAudioKickPending = false;
        btAudioKickOutstanding = false;
        btPcmFlushRequested = true;

        // Kluczowa poprawka reconnect:
        // stary STARTING/STARTED nie moze przejsc do nastepnej sesji.
        a2dp.resetMediaForReconnect();

        if (btManagerState == BTM_WAIT_LINK_IDLE)
        {
            btRejectLateConnected = false;
            btLateDisconnectRequested = false;

            btIdleObserved = true;
            btIdleObservedAt = millis();
            Serial.println("[BT MANAGER] Previous attempt fully DISCONNECTED.");
            return;
        }

        if (btManagerState == BTM_WAIT_KNOWN ||
            btManagerState == BTM_WAIT_SCAN_CONNECT)
        {
            btAttemptFailed = true;
            return;
        }

        if (btManagerState == BTM_CONNECTED)
        {
            Serial.println("[BT] Active receiver lost.");
            Serial.println("[BT CORE] Full Bluedroid/controller restart requested.");

            btProfileRestartRequested = true;
            btManagerState = BTM_PROFILE_RESTART;
        }
    }
}

static void audioStateChanged(esp_a2d_audio_state_t state, void *)
{
    switch (state)
    {
        case ESP_A2D_AUDIO_STATE_STARTED:
        {
            if (i2sStream != nullptr)
                xStreamBufferReset(i2sStream);

            resetInputReaderState();

            btAudioStarted = true;
            btAudioKickPending = false;
            btAudioKickOutstanding = false;

            Serial.println("[AUDIO] STARTED");
            Serial.println("[AUDIO] PCM buffer enabled.");
            break;
        }

        case ESP_A2D_AUDIO_STATE_STOPPED:
            btAudioStarted = false;
            btAudioKickOutstanding = false;
            btPcmFlushRequested = true;
            Serial.println("[AUDIO] STOPPED");
            break;

        case ESP_A2D_AUDIO_STATE_REMOTE_SUSPEND:
            btAudioStarted = false;
            btAudioKickOutstanding = false;
            btPcmFlushRequested = true;
            Serial.println("[AUDIO] REMOTE SUSPEND");
            break;

        default:
            Serial.printf("[AUDIO] STATE=%d\n", (int)state);
            break;
    }
}

static void serviceBtAudioKick()
{
    if (!btConnected ||
        btAudioStarted ||
        !btAudioKickPending)
    {
        return;
    }

    const uint32_t now = millis();

    if (now - btConnectedAt < BT_AUDIO_KICK_FIRST_MS)
        return;

    // Jezeli biblioteka jest juz w STARTING albo STARTED,
    // absolutnie nie wysylamy kolejnego CHECK_SRC_RDY.
    //
    // To byl glowny problem poprzedniej wersji:
    // ACK od kolejnego CHECK_SRC_RDY mogl wpasc w stan STARTING
    // i zrzucic wewnetrzny s_media_state ponownie do IDLE.
    const int mediaState = a2dp.mediaStateCode();

    if (mediaState != 0)
    {
        return;
    }

    // Jesli poprzedni CHECK_SRC_RDY zostal przyjety przez API,
    // dajemy bibliotece pelne 4 s na ACK -> START.
    // Dopiero jesli po tym czasie nadal jest IDLE, pozwalamy
    // na druga i ostatnia probe.
    if (btAudioKickOutstanding)
    {
        if (now - btLastAudioKickAt < BT_AUDIO_KICK_RETRY_MS)
            return;

        btAudioKickOutstanding = false;

        Serial.println(
            "[AUDIO] Previous media kick timed out in IDLE -> one retry."
        );
    }

    if (btAudioKickCount >= BT_AUDIO_KICK_MAX)
    {
        btAudioKickPending = false;

        Serial.printf(
            "[AUDIO] Media start not confirmed after %u controlled attempts. "
            "Heartbeat remains active.\n",
            (unsigned)BT_AUDIO_KICK_MAX
        );

        return;
    }

    btAudioKickCount++;
    btLastAudioKickAt = now;

    const esp_err_t err =
        esp_a2d_media_ctrl(
            ESP_A2D_MEDIA_CTRL_CHECK_SRC_RDY
        );

    Serial.printf(
        "[AUDIO] CHECK_SRC_RDY %u/%u -> %d | MEDIA=%d\n",
        (unsigned)btAudioKickCount,
        (unsigned)BT_AUDIO_KICK_MAX,
        (int)err,
        mediaState
    );

    if (err == ESP_OK)
    {
        // JEDNA komenda jest teraz "w locie".
        // Nie wysylamy nastepnej dopoki:
        // - nie nadejdzie AUDIO STARTED, albo
        // - nie minie 4 s i media nadal pozostanie IDLE.
        btAudioKickOutstanding = true;
    }
}


static bool waitBtControllerNotEnabled(uint32_t timeoutMs)
{
    const uint32_t started = millis();

    while (esp_bt_controller_get_status() ==
           ESP_BT_CONTROLLER_STATUS_ENABLED)
    {
        if (millis() - started >= timeoutMs)
            return false;

        delay(20);
    }

    return true;
}


static bool shutdownBtCoreWithoutMemRelease()
{
    bool ok = true;

    Serial.printf("[BT CORE] Bluedroid status before: %d\n",
                  (int)esp_bluedroid_get_status());

    // --------------------------------------------------------
    // BLUEDROID
    // --------------------------------------------------------
    esp_bluedroid_status_t bStatus =
        esp_bluedroid_get_status();

    if (bStatus == ESP_BLUEDROID_STATUS_ENABLED)
    {
        const esp_err_t err = esp_bluedroid_disable();

        Serial.printf("[BT CORE] esp_bluedroid_disable -> %d\n",
                      (int)err);

        if (err != ESP_OK)
            ok = false;

        delay(80);
        bStatus = esp_bluedroid_get_status();
    }

    if (bStatus == ESP_BLUEDROID_STATUS_INITIALIZED)
    {
        const esp_err_t err = esp_bluedroid_deinit();

        Serial.printf("[BT CORE] esp_bluedroid_deinit -> %d\n",
                      (int)err);

        if (err != ESP_OK)
            ok = false;

        delay(80);
    }

    // --------------------------------------------------------
    // CONTROLLER
    // --------------------------------------------------------
    esp_bt_controller_status_t cStatus =
        esp_bt_controller_get_status();

    Serial.printf("[BT CORE] Controller status before: %d\n",
                  (int)cStatus);

    if (cStatus == ESP_BT_CONTROLLER_STATUS_ENABLED)
    {
        const esp_err_t err = esp_bt_controller_disable();

        Serial.printf("[BT CORE] esp_bt_controller_disable -> %d\n",
                      (int)err);

        if (err != ESP_OK)
            ok = false;

        if (!waitBtControllerNotEnabled(BT_CORE_STATE_WAIT_MS))
        {
            Serial.println(
                "[BT CORE] ERROR: controller stayed ENABLED."
            );

            ok = false;
        }

        cStatus = esp_bt_controller_get_status();
    }

    if (cStatus == ESP_BT_CONTROLLER_STATUS_INITED)
    {
        const esp_err_t err = esp_bt_controller_deinit();

        Serial.printf("[BT CORE] esp_bt_controller_deinit -> %d\n",
                      (int)err);

        if (err != ESP_OK)
            ok = false;

        delay(100);
    }

    // BARDZO WAZNE:
    // NIE WOLNO wywolywac:
    //
    //   esp_bt_controller_mem_release(...)
    //
    // bo po zwolnieniu pamieci Classic BT nie mozna juz uruchomic
    // ponownie bez resetu ukladu.
    //
    // Bonding/link keys siedza w NVS, wiec pozostaja zachowane.

    Serial.printf("[BT CORE] Bluedroid status after: %d\n",
                  (int)esp_bluedroid_get_status());

    Serial.printf("[BT CORE] Controller status after: %d\n",
                  (int)esp_bt_controller_get_status());

    a2dp.prepareForColdBtCoreStart();

    return ok;
}


static void performBtProfileRestart()
{
    if (btProfileRestartInProgress)
        return;

    btProfileRestartInProgress = true;
    btProfileRestartRequested = false;
    btProfileRestartCount++;

    Serial.println();
    Serial.printf(
        "[BT CORE] FULL RESTART #%lu START\n",
        (unsigned long)btProfileRestartCount
    );

    // --------------------------------------------------------
    // 1. STOP lokalnego audio
    // --------------------------------------------------------
    btConnected = false;
    btAudioStarted = false;
    btAudioKickPending = false;
    btAudioKickOutstanding = false;
    btPcmFlushRequested = false;

    btRejectLateConnected = false;
    btLateDisconnectRequested = false;

    if (i2sStream != nullptr)
        xStreamBufferReset(i2sStream);

    resetInputReaderState();

    // --------------------------------------------------------
    // 2. ZATRZYMAJ manager
    // --------------------------------------------------------
    btStackReady = false;
    btDiscoveryStoppedEvent = false;
    btAttemptFailed = false;
    pendingPeerValid = false;

    btIdleObserved = false;
    btIdleObservedAt = 0;

    // --------------------------------------------------------
    // 3. ZATRZYMAJ A2DP/AVRCP/BtAppT
    // --------------------------------------------------------
    Serial.println(
        "[BT CORE] Stage 1/3: stopping A2DP/AVRCP..."
    );

    // false = biblioteka NIE robi mem_release.
    a2dp.end(false);

    delay(BT_PROFILE_RESTART_SETTLE_MS);

    // --------------------------------------------------------
    // 4. PELNY RESET BLUEDROID + CONTROLLER
    // --------------------------------------------------------
    Serial.println(
        "[BT CORE] Stage 2/3: deinit Bluedroid + controller..."
    );

    const bool shutdownOk =
        shutdownBtCoreWithoutMemRelease();

    if (!shutdownOk)
    {
        Serial.println(
            "[BT CORE] WARNING: one shutdown step returned an error."
        );
    }

    delay(BT_PROFILE_RESTART_SETTLE_MS);

    // --------------------------------------------------------
    // 5. PONOWNY START
    // --------------------------------------------------------
    Serial.println(
        "[BT CORE] Stage 3/3: cold start Bluetooth stack..."
    );

    a2dp.set_ssp_enabled(true);
    a2dp.set_auto_reconnect(false);
    a2dp.set_ssid_callback(deviceFound);
    a2dp.set_data_callback_in_frames(provideAudio);
    a2dp.set_on_connection_state_changed(connectionChanged);
    a2dp.set_on_audio_state_changed(audioStateChanged);
    a2dp.set_volume(75);

    btKnownIndex = 0;
    btManagerState = BTM_WAIT_STACK;

    // start() ponownie wywoluje bt_start(), inicjalizuje kontroler,
    // Bluedroid, BtAppT oraz nasz av_hdl_stack_evt().
    a2dp.start();

    btProfileRestartInProgress = false;

    Serial.println(
        "[BT CORE] Cold start issued; waiting for [BT STACK] READY."
    );

    Serial.println(
        "[BT CORE] Bonding and MRU-3 preserved."
    );

    Serial.println();
}


static void serviceBtManager()
{
    const uint32_t now = millis();

    if (btPcmFlushRequested && !btConnected)
        flushPcmAfterBtDisconnect();

    if (btProfileRestartRequested &&
        !btProfileRestartInProgress &&
        !btConnected)
    {
        performBtProfileRestart();
        return;
    }

    if (btLateDisconnectRequested)
    {
        btLateDisconnectRequested = false;

        Serial.println("[BT MANAGER] Disconnecting cancelled late connection...");
        const bool ok = a2dp.abortPeer(btCancelledPeerMac);
        Serial.printf("[BT MANAGER] late disconnect -> %s\n",
                      ok ? "OK" : "ERROR");
        return;
    }

    if (!btStackReady)
        return;

    // Po starcie stosu pobieramy liste bonded i uzupelniamy wolne
    // sloty do maksymalnie 3. Dzieki temu aktualne sparowane odbiorniki
    // sa od razu dostepne po przejsciu z V6.x na V7.
    if (!btBondImportDone)
    {
        importBondedDevices();
        printKnownDevices();

        btPauseUntil = now + BT_STACK_SETTLE_MS;
        btManagerState = BTM_PAUSE;
        btKnownIndex = 0;
        return;
    }

    switch (btManagerState)
    {
        case BTM_WAIT_STACK:
            btPauseUntil = now + BT_STACK_SETTLE_MS;
            btManagerState = BTM_PAUSE;
            break;

        case BTM_PAUSE:
            if ((int32_t)(now - btPauseUntil) >= 0)
            {
                btManagerCycle++;
                btManagerState = BTM_TRY_KNOWN;
            }
            break;

        case BTM_TRY_KNOWN:
        {
            while (btKnownIndex < KNOWN_DEVICE_COUNT &&
                   !knownDevices[btKnownIndex].valid)
            {
                btKnownIndex++;
            }

            if (btKnownIndex >= KNOWN_DEVICE_COUNT)
            {
                startPairScan();
                break;
            }

            if (!startKnownAttempt(btKnownIndex))
            {
                tryNextKnownOrScan();
            }
            break;
        }

        case BTM_WAIT_KNOWN:
        {
            if (btConnected)
            {
                btManagerState = BTM_CONNECTED;
                break;
            }

            const uint32_t elapsed = now - btStateStartedAt;
            const esp_a2d_connection_state_t st =
                a2dp.get_connection_state();

            if (btAttemptFailed)
            {
                Serial.printf("[BT KNOWN] #%u unavailable -> wait clean link idle.\n",
                              (unsigned)(btKnownIndex + 1));

                enterWaitLinkIdle(findNextKnownIndex(btKnownIndex));
                break;
            }

            // Gdy stos jest CONNECTING, nie abortujemy po 4.5 s.
            // Czekamy do 12 s na prawidlowy wynik procedury Classic BT/A2DP.
            if (st == ESP_A2D_CONNECTION_STATE_CONNECTING &&
                elapsed < BT_KNOWN_CONNECT_TIMEOUT_MS)
            {
                break;
            }

            if (st == ESP_A2D_CONNECTION_STATE_DISCONNECTED)
            {
                if (elapsed >= 1200)
                {
                    Serial.printf("[BT KNOWN] #%u returned DISCONNECTED -> next.\n",
                                  (unsigned)(btKnownIndex + 1));
                    enterWaitLinkIdle(findNextKnownIndex(btKnownIndex));
                }
                break;
            }

            // Dopiero bardzo dlugie CONNECTING uznajemy za zawieszone.
            if (elapsed >= BT_KNOWN_CONNECT_TIMEOUT_MS)
            {
                Serial.printf("[BT KNOWN] #%u CONNECTING > %lu ms -> cancel.\n",
                              (unsigned)(btKnownIndex + 1),
                              (unsigned long)BT_KNOWN_CONNECT_TIMEOUT_MS);

                memcpy(btCancelledPeerMac,
                       knownDevices[btKnownIndex].mac,
                       ESP_BD_ADDR_LEN);

                btRejectLateConnected = true;

                const uint8_t next =
                    findNextKnownIndex(btKnownIndex);

                const bool ok =
                    a2dp.abortPeer(knownDevices[btKnownIndex].mac);

                Serial.printf("[BT KNOWN] abort stuck peer -> %s\n",
                              ok ? "OK" : "ERROR");

                enterWaitLinkIdle(next);
            }

            break;
        }

        case BTM_WAIT_LINK_IDLE:
        {
            const esp_a2d_connection_state_t st =
                a2dp.get_connection_state();

            if (!btIdleObserved)
            {
                if (st == ESP_A2D_CONNECTION_STATE_DISCONNECTED)
                {
                    btIdleObserved = true;
                    btIdleObservedAt = now;
                    Serial.println("[BT MANAGER] Link idle -> settle 650 ms.");
                }

                break;
            }

            if (st != ESP_A2D_CONNECTION_STATE_DISCONNECTED)
            {
                btIdleObserved = false;
                btIdleObservedAt = 0;
                break;
            }

            if (now - btIdleObservedAt < BT_LINK_IDLE_SETTLE_MS)
                break;

            if (btNextKnownAfterIdle < KNOWN_DEVICE_COUNT)
            {
                btKnownIndex = btNextKnownAfterIdle;
                btManagerState = BTM_TRY_KNOWN;

                Serial.printf("[BT MANAGER] Clean idle -> MRU #%u.\n",
                              (unsigned)(btKnownIndex + 1));
            }
            else
            {
                Serial.println("[BT MANAGER] Clean idle -> pairing scan.");
                startPairScan();
            }

            break;
        }

        case BTM_SCAN:
            if (btDiscoveryStoppedEvent)
            {
                btDiscoveryStoppedEvent = false;

                if (scanCandidateValid)
                    connectScanCandidate();
                else
                {
                    Serial.println("[BT PAIR] No suitable candidate.");
                    scheduleKnownSequence(BT_SCAN_RESTART_DELAY_MS);
                }
            }
            break;

        case BTM_WAIT_SCAN_CONNECT:
        {
            if (btConnected)
            {
                btManagerState = BTM_CONNECTED;
                break;
            }

            const uint32_t elapsed = now - btStateStartedAt;
            const esp_a2d_connection_state_t st =
                a2dp.get_connection_state();

            if (btAttemptFailed ||
                st == ESP_A2D_CONNECTION_STATE_DISCONNECTED)
            {
                Serial.println("[BT PAIR] Candidate failed -> wait clean link idle.");
                resetScanCandidate();
                enterWaitLinkIdle(0);
                break;
            }

            if (st == ESP_A2D_CONNECTION_STATE_CONNECTING &&
                elapsed < BT_KNOWN_CONNECT_TIMEOUT_MS)
            {
                break;
            }

            if (elapsed >= BT_KNOWN_CONNECT_TIMEOUT_MS)
            {
                Serial.println("[BT PAIR] Candidate CONNECTING too long -> cancel.");

                memcpy(btCancelledPeerMac,
                       scanCandidateMac,
                       ESP_BD_ADDR_LEN);

                btRejectLateConnected = true;
                a2dp.abortPeer(scanCandidateMac);

                resetScanCandidate();
                enterWaitLinkIdle(0);
            }

            break;
        }

        case BTM_CONNECTED:
            break;

        case BTM_PROFILE_RESTART:
            break;
    }
}


void setup()
{
    Serial.begin(115200);
    delay(800);

    Serial.println();
    Serial.println("========================================");
    Serial.println(" MAESTRO BT V7.4 / MRU-3 FULL BT CORE RESTART");
    Serial.println(" ESP32-WROOM-32U / A2DP SOURCE");
    Serial.println("========================================");

    Serial.printf("[SYSTEM] Chip: %s\n", ESP.getChipModel());
    Serial.printf("[SYSTEM] Free heap: %u B\n", ESP.getFreeHeap());
    Serial.printf("[SYSTEM] I2S AUTO 44.1/48 kHz -> A2DP %lu Hz\n",
                  (unsigned long)A2DP_SAMPLE_RATE);

    loadKnownDevices();
    bootstrapLegacyLastDevice();

    if (!initI2sReceiver())
    {
        Serial.println("[SYSTEM] STOP: I2S RX init failed");
        while (true)
            delay(1000);
    }

    xTaskCreatePinnedToCore(
        i2sReceiverTask,
        "i2s-rx",
        3072,
        nullptr,
        20,
        nullptr,
        1
    );

    // W V7 caly reconnect/pairing prowadzi nasz manager.
    a2dp.set_ssp_enabled(true);
    a2dp.set_auto_reconnect(false);

    a2dp.set_ssid_callback(deviceFound);
    a2dp.set_data_callback_in_frames(provideAudio);
    a2dp.set_on_connection_state_changed(connectionChanged);
    a2dp.set_on_audio_state_changed(audioStateChanged);
    a2dp.set_volume(75);

    Serial.println("[BT] Policy V7:");
    Serial.println("[BT] 1) try MRU #1, #2, #3 directly by MAC");
    Serial.println("[BT] 2) if absent -> short Audio/Video discovery");
    Serial.println("[BT] 3) choose strongest suitable candidate");
    Serial.println("[BT] 4) successful receiver becomes MRU #1");

    btManagerState = BTM_WAIT_STACK;
    btStateStartedAt = millis();

    a2dp.start();
}

void loop()
{
    serviceBtAudioKick();
    serviceBtManager();

    static uint32_t lastStats = 0;
    static uint32_t lastRx = 0;
    static uint32_t lastA2dpReq = 0;
    static uint32_t lastA2dpPcm = 0;

    const uint32_t now = millis();

    if (now - lastStats >= 3000)
    {
        lastStats = now;

        const uint32_t rxNow = i2sBytesReceived;
        const uint32_t delta = rxNow - lastRx;
        lastRx = rxNow;

        const uint32_t reqNow = a2dpFramesRequested;
        const uint32_t pcmNow = a2dpFramesWithPcm;
        const uint32_t reqDelta = reqNow - lastA2dpReq;
        const uint32_t pcmDelta = pcmNow - lastA2dpPcm;

        lastA2dpReq = reqNow;
        lastA2dpPcm = pcmNow;

        const uint32_t reqFps = reqDelta / 3;
        const uint32_t pcmFps = pcmDelta / 3;

        const size_t buffered =
            (i2sStream != nullptr)
                ? xStreamBufferBytesAvailable(i2sStream)
                : 0;

        Serial.printf(
            "[I2S] RX=%lu B | RATE~%lu Hz | MODE=%s | BUF=%u B | DROP=%lu B | UNDERRUN=%lu | BT=%s | AUDIO=%s | MEDIA=%d | A2DP_REQ~%lu | PCM~%lu | BTM=%u | KNOWN=%d | CYCLE=%lu | BTRST=%lu\n",
            (unsigned long)delta,
            (unsigned long)measuredI2sRate,
            inputRateName(inputRateMode),
            (unsigned)buffered,
            (unsigned long)i2sBytesDropped,
            (unsigned long)a2dpUnderruns,
            btConnected ? "ON" : "OFF",
            btAudioStarted ? "ON" : "OFF",
            a2dp.mediaStateCode(),
            (unsigned long)reqFps,
            (unsigned long)pcmFps,
            (unsigned)btManagerState,
            countKnownDevices(),
            (unsigned long)btManagerCycle,
            (unsigned long)btProfileRestartCount
        );
    }

    delay(20);
}
