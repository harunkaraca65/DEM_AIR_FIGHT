/*
 * SPDX-FileCopyrightText: 2021-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include "driver/gpio.h" // GPIO Sürücüsü eklendi
#include "driver/uart.h" // UART Sürücüsü eklendi
#include "esp_bt.h"
#include "esp_bt_device.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_log.h"
#include "esp_spp_api.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "sys/time.h"
#include "time.h"

#define SPP_TAG "DEM_AIR_SPP"
#define SPP_SERVER_NAME "SPP_SERVER"
#define EXAMPLE_DEVICE_NAME "DEM_AIR_FIGHTER_ESP" // İstenen Cihaz Adı
#define SPP_SHOW_DATA 0
#define SPP_SHOW_SPEED 1
#define SPP_SHOW_MODE SPP_SHOW_SPEED

/* --- UART CONFIGURATION --- */
#define UART_PORT_NUM UART_NUM_2
#define UART_TX_PIN GPIO_NUM_25
#define UART_RX_PIN GPIO_NUM_26
#define UART_BAUD_RATE 115200
#define UART_BUF_SIZE 1024

static const esp_spp_mode_t esp_spp_mode = ESP_SPP_MODE_CB;
static const bool esp_spp_enable_l2cap_ertm = true;

static struct timeval time_new, time_old;
static long data_num = 0;

/* Global değişkenler */
static uint32_t spp_handle = 0;
static bool is_spp_connected = false;
static bool is_spp_congested = false; // <-- YENİ: Tıkanıklık bayrağı

static const esp_spp_sec_t sec_mask = ESP_SPP_SEC_AUTHENTICATE;
static const esp_spp_role_t role_slave = ESP_SPP_ROLE_SLAVE;

static char *bda2str(uint8_t *bda, char *str, size_t size) {
  if (bda == NULL || str == NULL || size < 18) {
    return NULL;
  }

  uint8_t *p = bda;
  sprintf(str, "%02x:%02x:%02x:%02x:%02x:%02x", p[0], p[1], p[2], p[3], p[4],
          p[5]);
  return str;
}

static void print_speed(void) {
  float time_old_s = time_old.tv_sec + time_old.tv_usec / 1000000.0;
  float time_new_s = time_new.tv_sec + time_new.tv_usec / 1000000.0;
  float time_interval = time_new_s - time_old_s;
  float speed = data_num * 8 / time_interval / 1000.0;
  ESP_LOGI(SPP_TAG, "speed(%fs ~ %fs): %f kbit/s", time_old_s, time_new_s,
           speed);
  data_num = 0;
  time_old.tv_sec = time_new.tv_sec;
  time_old.tv_usec = time_new.tv_usec;
}

/* --- UART INIT FUNCTION --- */
void init_uart(void) {
  uart_config_t uart_config = {
      .baud_rate = UART_BAUD_RATE,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .source_clk = UART_SCLK_DEFAULT,
  };

  // UART Sürücüsünü yükle
  ESP_ERROR_CHECK(
      uart_driver_install(UART_PORT_NUM, UART_BUF_SIZE * 2, 0, 0, NULL, 0));
  ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
  ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, UART_TX_PIN, UART_RX_PIN,
                               UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

  ESP_LOGI(SPP_TAG, "UART2 Initialized: TX=GPIO%d, RX=GPIO%d, Baud=%d",
           UART_TX_PIN, UART_RX_PIN, UART_BAUD_RATE);
}

/* --- UART TO BLUETOOTH BRIDGE TASK --- */
/* --- UART TO BLUETOOTH BRIDGE TASK (GÜNCELLENMİŞ) --- */
void uart_rx_task(void *arg) {
  uint8_t *data = (uint8_t *)malloc(UART_BUF_SIZE);

  ESP_LOGI(SPP_TAG, "UART Bridge Task Started");

  while (1) {
    // 1. UART'tan veriyi her zaman oku (Timeout 20ms)
    // Eğer okumazsak STM32 tarafında şişme olabilir, o yüzden hep okuyup
    // buffer'ı boşaltıyoruz.
    int len = uart_read_bytes(UART_PORT_NUM, data, UART_BUF_SIZE,
                              20 / portTICK_PERIOD_MS);

    // 2. Veri geldiyse kontrol et
    if (len > 0) {
      // SADECE ve SADECE Bluetooth bağlıysa yazmaya çalış
      if (is_spp_connected && spp_handle != 0) {
        esp_spp_write(spp_handle, len, data);
      } else {
        // Bağlantı yoksa hiçbir şey yapma.
        // Veri zaten 'data' değişkenine okundu, bir sonraki döngüde üzerine
        // yazılacak. Böylece "btc_spp_write" hatası almazsın çünkü göndermeye
        // çalışmıyoruz.

        // İsterseniz buraya "Boşa düştü" logu koyabilirsiniz ama çok hızlı
        // akar, tavsiye etmem.
      }
    }
  }
  free(data);
}

static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
  // char bda_str[18] = {0};

  switch (event) {
  case ESP_SPP_INIT_EVT:
    if (param->init.status == ESP_SPP_SUCCESS) {
      ESP_LOGI(SPP_TAG, "ESP_SPP_INIT_EVT");
      esp_spp_start_srv(sec_mask, role_slave, 0, SPP_SERVER_NAME);
    } else {
      ESP_LOGE(SPP_TAG, "ESP_SPP_INIT_EVT status:%d", param->init.status);
    }
    break;

  case ESP_SPP_DISCOVERY_COMP_EVT:
    ESP_LOGI(SPP_TAG, "ESP_SPP_DISCOVERY_COMP_EVT");
    break;

  case ESP_SPP_OPEN_EVT:
    ESP_LOGI(SPP_TAG, "ESP_SPP_OPEN_EVT");
    break;
  case ESP_SPP_CLOSE_EVT:
    // Bağlantı koptu
    ESP_LOGI(SPP_TAG,
             "ESP_SPP_CLOSE_EVT status:%d handle:%" PRIu32
             " close_by_remote:%d",
             param->close.status, param->close.handle, param->close.async);
    is_spp_connected = false;
    spp_handle = 0;
    break;

  case ESP_SPP_START_EVT:
    if (param->start.status == ESP_SPP_SUCCESS) {
      ESP_LOGI(SPP_TAG, "ESP_SPP_START_EVT handle:%" PRIu32 " sec_id:%d scn:%d",
               param->start.handle, param->start.sec_id, param->start.scn);
      // Cihaz ismini burada ayarlıyoruz
      esp_bt_gap_set_device_name(EXAMPLE_DEVICE_NAME);
      esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
    } else {
      ESP_LOGE(SPP_TAG, "ESP_SPP_START_EVT status:%d", param->start.status);
    }
    break;

  case ESP_SPP_CL_INIT_EVT:
    ESP_LOGI(SPP_TAG, "ESP_SPP_CL_INIT_EVT");
    break;

  case ESP_SPP_DATA_IND_EVT:
    // Telefondan ESP32'ye veri gelirse (Gerekirse STM32'ye iletmek için burası
    // kullanılır)
#if (SPP_SHOW_MODE == SPP_SHOW_DATA)
    ESP_LOGI(SPP_TAG, "ESP_SPP_DATA_IND_EVT len:%d handle:%" PRIu32,
             param->data_ind.len, param->data_ind.handle);
    if (param->data_ind.len < 128) {
      esp_log_buffer_hex("", param->data_ind.data, param->data_ind.len);
    }
#else
    gettimeofday(&time_new, NULL);
    data_num += param->data_ind.len;
    if (time_new.tv_sec - time_old.tv_sec >= 3) {
      print_speed();
    }
#endif
    break;

  case ESP_SPP_CONG_EVT:
    // YENİ: Tıkanıklık durumu değiştiğinde burası çalışır
    // param->cong.cong TRUE ise tıkalı, FALSE ise yol açıldı demektir.
    is_spp_congested = param->cong.cong;
    if (is_spp_congested) {
      ESP_LOGW(SPP_TAG, "Bluetooth Buffer Doldu! (Veriler çöpe gidecek)");
    } else {
      ESP_LOGI(SPP_TAG, "Bluetooth Yolu Açıldı.");
    }
    break;

  case ESP_SPP_WRITE_EVT:
    // Veri gönderimi tamamlandığında tetiklenir (Bloklamasız gönderim için
    // önemli) ESP_LOGI(SPP_TAG, "ESP_SPP_WRITE_EVT");
    break;

  case ESP_SPP_SRV_OPEN_EVT:
    ESP_LOGI(SPP_TAG, "ESP_SPP_SRV_OPEN_EVT");
    spp_handle = param->srv_open.handle;
    is_spp_connected = true;
    is_spp_congested = false; // <-- YENİ: Bağlanınca yol açık başla
    break;

  case ESP_SPP_SRV_STOP_EVT:
    ESP_LOGI(SPP_TAG, "ESP_SPP_SRV_STOP_EVT");
    break;

  case ESP_SPP_UNINIT_EVT:
    ESP_LOGI(SPP_TAG, "ESP_SPP_UNINIT_EVT");
    break;
  default:
    break;
  }
}

void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param) {
  char bda_str[18] = {0};

  switch (event) {
  case ESP_BT_GAP_AUTH_CMPL_EVT: {
    if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
      ESP_LOGI(SPP_TAG, "authentication success: %s bda:[%s]",
               param->auth_cmpl.device_name,
               bda2str(param->auth_cmpl.bda, bda_str, sizeof(bda_str)));
    } else {
      ESP_LOGE(SPP_TAG, "authentication failed, status:%d",
               param->auth_cmpl.stat);
    }
    break;
  }
  case ESP_BT_GAP_PIN_REQ_EVT: {
    ESP_LOGI(SPP_TAG, "ESP_BT_GAP_PIN_REQ_EVT min_16_digit:%d",
             param->pin_req.min_16_digit);
    if (param->pin_req.min_16_digit) {
      ESP_LOGI(SPP_TAG, "Input pin code: 0000 0000 0000 0000");
      esp_bt_pin_code_t pin_code = {0};
      esp_bt_gap_pin_reply(param->pin_req.bda, true, 16, pin_code);
    } else {
      ESP_LOGI(SPP_TAG, "Input pin code: 1234");
      esp_bt_pin_code_t pin_code;
      pin_code[0] = '1';
      pin_code[1] = '2';
      pin_code[2] = '3';
      pin_code[3] = '4';
      esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code);
    }
    break;
  }

#if (CONFIG_EXAMPLE_SSP_ENABLED == true)
  case ESP_BT_GAP_CFM_REQ_EVT:
    ESP_LOGI(
        SPP_TAG,
        "ESP_BT_GAP_CFM_REQ_EVT Please compare the numeric value: %" PRIu32,
        param->cfm_req.num_val);
    esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
    break;
  case ESP_BT_GAP_KEY_NOTIF_EVT:
    ESP_LOGI(SPP_TAG, "ESP_BT_GAP_KEY_NOTIF_EVT passkey:%" PRIu32,
             param->key_notif.passkey);
    break;
  case ESP_BT_GAP_KEY_REQ_EVT:
    ESP_LOGI(SPP_TAG, "ESP_BT_GAP_KEY_REQ_EVT Please enter passkey!");
    break;
#endif

  case ESP_BT_GAP_MODE_CHG_EVT:
    ESP_LOGI(SPP_TAG, "ESP_BT_GAP_MODE_CHG_EVT mode:%d bda:[%s]",
             param->mode_chg.mode,
             bda2str(param->mode_chg.bda, bda_str, sizeof(bda_str)));
    break;

  default: {
    ESP_LOGI(SPP_TAG, "event: %d", event);
    break;
  }
  }
  return;
}

void app_main(void) {
  char bda_str[18] = {0};
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

  esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
  if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
    ESP_LOGE(SPP_TAG, "%s initialize controller failed: %s", __func__,
             esp_err_to_name(ret));
    return;
  }

  if ((ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) != ESP_OK) {
    ESP_LOGE(SPP_TAG, "%s enable controller failed: %s", __func__,
             esp_err_to_name(ret));
    return;
  }

  esp_bluedroid_config_t bluedroid_cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
#if (CONFIG_EXAMPLE_SSP_ENABLED == false)
  bluedroid_cfg.ssp_en = false;
#endif
  if ((ret = esp_bluedroid_init_with_cfg(&bluedroid_cfg)) != ESP_OK) {
    ESP_LOGE(SPP_TAG, "%s initialize bluedroid failed: %s", __func__,
             esp_err_to_name(ret));
    return;
  }

  if ((ret = esp_bluedroid_enable()) != ESP_OK) {
    ESP_LOGE(SPP_TAG, "%s enable bluedroid failed: %s", __func__,
             esp_err_to_name(ret));
    return;
  }

  if ((ret = esp_bt_gap_register_callback(esp_bt_gap_cb)) != ESP_OK) {
    ESP_LOGE(SPP_TAG, "%s gap register failed: %s", __func__,
             esp_err_to_name(ret));
    return;
  }

  if ((ret = esp_spp_register_callback(esp_spp_cb)) != ESP_OK) {
    ESP_LOGE(SPP_TAG, "%s spp register failed: %s", __func__,
             esp_err_to_name(ret));
    return;
  }

  esp_spp_cfg_t bt_spp_cfg = {
      .mode = esp_spp_mode,
      .enable_l2cap_ertm = esp_spp_enable_l2cap_ertm,
      .tx_buffer_size = 0, /* Only used for ESP_SPP_MODE_VFS mode */
  };
  if ((ret = esp_spp_enhanced_init(&bt_spp_cfg)) != ESP_OK) {
    ESP_LOGE(SPP_TAG, "%s spp init failed: %s", __func__, esp_err_to_name(ret));
    return;
  }

#if (CONFIG_EXAMPLE_SSP_ENABLED == true)
  /* Set default parameters for Secure Simple Pairing */
  esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
  esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_IO;
  esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));
#endif

  /*
   * Set default parameters for Legacy Pairing
   * Use variable pin, input pin code when pairing
   */
  esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_VARIABLE;
  esp_bt_pin_code_t pin_code;
  esp_bt_gap_set_pin(pin_type, 0, pin_code);

  ESP_LOGI(
      SPP_TAG, "Own address:[%s]",
      bda2str((uint8_t *)esp_bt_dev_get_address(), bda_str, sizeof(bda_str)));

  /* --- ÖZEL KOD BAŞLANGICI --- */
  // 1. UART'ı başlat
  init_uart();

  // 2. UART Okuma ve BT Yollama Görevini başlat (4KB Stack, Priority 2)
  xTaskCreate(uart_rx_task, "uart_rx_task", 4096, NULL, 2, NULL);
  /* --- ÖZEL KOD BİTİŞİ --- */
}