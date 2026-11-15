/* The rover's board: an ESP32-P4 board fixed to the chassis, joined to the hub over Wi-Fi and to the rover's six
 * servos through a BusLinker in UART mode (docs/BOM.md, DECISIONS MD48).
 *
 * This file is the part that needs the board: Wi-Fi, one TCP connection from the hub, the UART. Everything the
 * board decides is in bridge.c, which also runs inside the simulation. The hub connects to port
 * CONFIG_ROVER_BRIDGE_PORT and from then on the connection carries the servo bus's bytes, both ways, unread.
 *
 * NOT YET BUILT OR FLASHED. Written against ESP-IDF 5.x; see ../README.md for what to check on the first build.
 */
#include <errno.h>
#include <string.h>

#include "driver/uart.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"
#include "nvs_flash.h"

#include "bridge.h"

static const char *TAG = "rover";

#define BUS_UART ((uart_port_t)CONFIG_ROVER_BRIDGE_UART_NUM)
#define CHUNK 256

static uint32_t now_ms(void) { return (uint32_t)(esp_timer_get_time() / 1000); } /* wraps after 49 days: bridge.c allows for it */

static void on_wifi(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && (id == WIFI_EVENT_STA_START || id == WIFI_EVENT_STA_DISCONNECTED)) {
        if (id == WIFI_EVENT_STA_DISCONNECTED) {
            ESP_LOGW(TAG, "Wi-Fi lost: joining again (the wheels stop by themselves meanwhile)");
        }
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        const ip_event_got_ip_t *got = (const ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "on the network at " IPSTR ", port %d", IP2STR(&got->ip_info.ip), CONFIG_ROVER_BRIDGE_PORT);
    }
}

static void wifi_start(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, on_wifi, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, on_wifi, NULL, NULL));
    wifi_config_t cfg = {0};
    strlcpy((char *)cfg.sta.ssid, CONFIG_ROVER_BRIDGE_WIFI_SSID, sizeof(cfg.sta.ssid));
    strlcpy((char *)cfg.sta.password, CONFIG_ROVER_BRIDGE_WIFI_PASSWORD, sizeof(cfg.sta.password));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &cfg));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE)); /* power save adds tens of milliseconds to every reply */
    ESP_ERROR_CHECK(esp_wifi_start());
}

static void bus_start(void)
{
    const uart_config_t cfg = {
        .baud_rate = CONFIG_ROVER_BRIDGE_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(BUS_UART, 2048, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(BUS_UART, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(BUS_UART, CONFIG_ROVER_BRIDGE_TX_GPIO, CONFIG_ROVER_BRIDGE_RX_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

static int listen_socket(void)
{
    int s = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (s < 0) {
        return -1;
    }
    int yes = 1;
    setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
    struct sockaddr_in addr = {.sin_family = AF_INET, .sin_port = htons(CONFIG_ROVER_BRIDGE_PORT), .sin_addr.s_addr = htonl(INADDR_ANY)};
    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) != 0 || listen(s, 1) != 0) {
        close(s);
        return -1;
    }
    fcntl(s, F_SETFL, fcntl(s, F_GETFL, 0) | O_NONBLOCK);
    return s;
}

static void bridge_task(void *arg)
{
    static uint8_t in[CHUNK], out[CHUNK > BRIDGE_STOP_PACKET_MAX ? CHUNK : BRIDGE_STOP_PACKET_MAX];
    bridge_t bridge;
    const bridge_config_t cfg = {
        .wheel_ids = {CONFIG_ROVER_BRIDGE_WHEEL_LEFT_ID, CONFIG_ROVER_BRIDGE_WHEEL_RIGHT_ID},
        .n_wheels = 2,
        .speed_addr = CONFIG_ROVER_BRIDGE_SPEED_ADDR,
        .speed_size = 2,
        .watchdog_ms = CONFIG_ROVER_BRIDGE_WATCHDOG_MS,
        .repeat_ms = 1000,
#ifdef CONFIG_ROVER_BRIDGE_STRIP_ECHO
        .strip_echo = true,
#endif
    };
    if (!bridge_init(&bridge, &cfg, now_ms())) {
        ESP_LOGE(TAG, "the bridge's configuration cannot be used");
        vTaskDelete(NULL);
    }
    int server = -1, hub = -1;
    uint32_t said = now_ms();
    for (;;) {
        uint32_t t = now_ms();
        if (server < 0) {
            server = listen_socket(); /* (fails until the network is up: tried again each time round) */
        }
        if (hub < 0 && server >= 0) {
            hub = accept(server, NULL, NULL);
            if (hub >= 0) {
                int yes = 1;
                setsockopt(hub, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof(yes)); /* servo packets are a few bytes: send each at once */
                fcntl(hub, F_SETFL, fcntl(hub, F_GETFL, 0) | O_NONBLOCK);
                ESP_LOGI(TAG, "the hub has connected");
            }
        }
        /* hub -> bus */
        if (hub >= 0) {
            int n = recv(hub, in, sizeof(in), 0);
            if (n > 0) {
                size_t m = bridge_from_hub(&bridge, in, (size_t)n, t, out, sizeof(out));
                uart_write_bytes(BUS_UART, out, m);
            } else if (n == 0 || (errno != EWOULDBLOCK && errno != EAGAIN)) {
                ESP_LOGW(TAG, "the hub has gone: waiting for it to connect again");
                close(hub);
                hub = -1;
            }
        }
        /* bus -> hub (waits up to one tick for bytes, which is what paces this loop) */
        int n = uart_read_bytes(BUS_UART, in, sizeof(in), 1);
        if (n > 0) {
            size_t m = bridge_from_bus(&bridge, in, (size_t)n, t, out, sizeof(out));
            if (hub >= 0 && m > 0 && send(hub, out, m, 0) < 0 && errno != EWOULDBLOCK && errno != EAGAIN) {
                close(hub);
                hub = -1;
            }
        }
        /* the hub has gone quiet: stop the wheels */
        size_t stop = bridge_tick(&bridge, now_ms(), out, sizeof(out));
        if (stop > 0) {
            uart_write_bytes(BUS_UART, out, stop);
        }
        if ((uint32_t)(t - said) >= 10000) {
            said = t;
            ESP_LOGI(TAG, "hub %s, wheels %s by this board; stopped them %u times; %u bytes to the bus, %u back",
                     hub >= 0 ? "connected" : "not connected", bridge.stopped ? "held stopped" : "not held",
                     (unsigned)bridge.trips, (unsigned)bridge.hub_bytes, (unsigned)bridge.bus_bytes);
        }
    }
}

void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    bus_start(); /* before the network: the wheels are stopped from the first moment, hub or no hub */
    xTaskCreate(bridge_task, "bridge", 6144, NULL, 10, NULL);
    wifi_start();
}
