/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <esp_err.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <esp_random.h>
#include <driver/i2c.h>
#include <driver/uart.h>
#include <sdkconfig.h>

#include <lib/support/CodeUtils.h>

#include <drivers/p1.h>

static const char * TAG = "p1";

#define P1_BAUD_RATE 115200
#define P1_RXD 4
#define P1_TXD 5

typedef struct {
    p1_sensor_config_t *config;
    esp_timer_handle_t timer;
    bool is_initialized = false;
} p1_sensor_ctx_t;

static p1_sensor_ctx_t s_ctx;

unsigned int currentCRC = 0;

unsigned int currentUsage = 0;

static esp_err_t p1_init_uart()
{
    // Setup UART buffered IO with event queue
    const int uart_buffer_size = (1024 * 2);
    QueueHandle_t uart_queue;
    const uart_port_t uart_num = UART_NUM_1;

    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_CTS_RTS,
        .rx_flow_ctrl_thresh = 122,
    };
    // Configure UART parameters
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
    // Inverse the received signal as per 5.7.2 "Data" line specification
    ESP_ERROR_CHECK(uart_set_line_inverse(uart_num, UART_SIGNAL_RXD_INV));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, 5, 4, -1, -1));

    // Install UART driver using an event queue here
    return uart_driver_install(uart_num, uart_buffer_size, uart_buffer_size, 10, &uart_queue, 0);
}

unsigned int crc16(unsigned int crc, unsigned char *buf, int len)
{
    for (int pos = 0; pos < len; pos++)
    {
        crc ^= (unsigned int)buf[pos];

        for (int i = 8; i != 0; i--)
        {
            if ((crc & 0x0001) != 0)
            {
                crc >>= 1;
                crc ^= 0xA001;
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    return crc;
}

bool isNumber(char *res, int len)
{
    for (int i = 0; i < len; i++)
    {
        if (((res[i] < '0') || (res[i] > '9')) && (res[i] != '.' && res[i] != 0))
        {
            return false;
        }
    }
    return true;
}

int findCharInArrayRev(char array[], char c, int len)
{
    for (int i = len - 1; i >= 0; i--)
    {
        if (array[i] == c)
        {
            return i;
        }
    }

    return -1;
}

long getValue(char *buffer, int maxlen, char startchar, char endchar)
{
    int s = findCharInArrayRev(buffer, startchar, maxlen - 2);
    int l = findCharInArrayRev(buffer, endchar, maxlen - 2) - s - 1;

    char res[16];
    memset(res, 0, sizeof(res));

    if (strncpy(res, buffer + s + 1, l))
    {
        if (endchar == '*')
        {
            if (isNumber(res, l))
                return (1000 * atof(res));
        }
        else if (endchar == ')')
        {
            if (isNumber(res, l))
                return atof(res);
        }
    }

    return 0;
}

/**
 *  Decodes the telegram PER line. Not the complete message. 
 */
bool decode_telegram(char *telegram, int len)
{
    int startChar = findCharInArrayRev(telegram, '/', len);
    int endChar = findCharInArrayRev(telegram, '!', len);
    bool validCRCFound = false;

#ifdef DEBUG
    for (int cnt = 0; cnt < len; cnt++)
    {
        Serial.print(telegram[cnt]);
    }
    Serial.print("\n");
#endif

    if (startChar >= 0)
    {
        // * Start found. Reset CRC calculation
        currentCRC = crc16(0x0000, (unsigned char *)telegram + startChar, len - startChar);
    }
    else if (endChar >= 0)
    {
        // * Add to crc calc
        currentCRC = crc16(currentCRC, (unsigned char *)telegram + endChar, 1);

        char messageCRC[5];
        strncpy(messageCRC, telegram + endChar + 1, 4);

        messageCRC[4] = 0; // * Thanks to HarmOtten (issue 5)
        validCRCFound = (strtol(messageCRC, NULL, 16) == currentCRC);

        if (validCRCFound)
            ESP_LOGI(TAG, "CRC Valid!");
        else
            ESP_LOGI(TAG, "CRC Invalid!");
        currentCRC = 0;
    }
    else
    {
        currentCRC = crc16(currentCRC, (unsigned char *)telegram, len);
    }

    if (strncmp(telegram, "1-0:1.7.0", strlen("1-0:1.7.0")) == 0)
    {
        long newValue = getValue(telegram, len, '(', '*');
        currentUsage = newValue;

        ESP_LOGI(TAG, "Found usage %ld", newValue);
    }

    return validCRCFound;
}

static size_t read_bytes_until(char terminator, char *buffer, size_t length) {
  size_t index = 0;
  while (index < length) {
    uint8_t data[1] = {0};
    int num_bytes = uart_read_bytes(UART_NUM_1, data, 1, 100);

    uint8_t c = data[0];

    if (c < 0 || (char)c == terminator) {
      break;
    }
    *buffer++ = (char)c;
    index++;
  }
  return index;  // return number of characters, not including null terminator
}


static bool p1_read(char *data, size_t size)
{

    const uart_port_t uart_num = UART_NUM_1;
    int length = 0;
    ESP_ERROR_CHECK(uart_get_buffered_data_len(uart_num, (size_t*)&length));
    
    // serial available
    if (length > 0) { 
        ESP_LOGI(TAG, "p1 data available");
        while (true) {
            int len = read_bytes_until('\n', data, 1050);

            data[len] = '\n';
            data[len + 1] = 0;
            bool result = decode_telegram(data, len + 1);
            // When the CRC is check which is also the end of the telegram
            // if valid decode return true
            if (result)
            {
                return true;
            }
        }
    }

    return false;
}

static esp_err_t p1_read_data(float & usage)
{
    // foreach temperature and humidity: two bytes data, one byte for checksum
    char data[1050] = {0};

    if (p1_read(data, sizeof(data))) {
        usage = currentUsage;
    }

    return ESP_OK;
}

static void timer_cb_internal(void *arg)
{
    auto *ctx = (p1_sensor_ctx_t *) arg;
    if (!(ctx && ctx->config)) {
        return;
    }

    float usage;
    esp_err_t err = p1_read_data(usage);
    if (err != ESP_OK) {
        return;
    }
    if (ctx->config->usage.cb) {
        ctx->config->usage.cb(ctx->config->usage.endpoint_id, usage, ctx->config->user_data);
    }
}

esp_err_t p1_sensor_init(p1_sensor_config_t *config)
{
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    // we need at least one callback so that we can start notifying application layer
    // TODO Uncomment
    // if (config->usage.cb == NULL) {
    //     return ESP_ERR_INVALID_ARG;
    // }
    if (s_ctx.is_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = p1_init_uart();
    if (err != ESP_OK) {
        return err;
    }

    // keep the pointer to config
    s_ctx.config = config;

    esp_timer_create_args_t args = {
        .callback = timer_cb_internal,
        .arg = &s_ctx,
    };

    err = esp_timer_create(&args, &s_ctx.timer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_timer_create failed, err:%d", err);
        return err;
    }

    err = esp_timer_start_periodic(s_ctx.timer, config->interval_ms * 1000);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_timer_start_periodic failed: %d", err);
        return err;
    }

    s_ctx.is_initialized = true;
    ESP_LOGI(TAG, "p1 initialized successfully");

    return ESP_OK;
}
