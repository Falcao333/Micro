// Manipulação de strings
#include <string.h>

// Componentes do FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

// Sistema ESP32
#include "esp_system.h"

// Configuração e operações do Wi-Fi
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"

// Armazenamento não volátil (NVS)
#include "nvs_flash.h"

// Manipulação de GPIO (General Purpose Input/Output)
#include "driver/gpio.h"

// Biblioteca para controle de fitas de LED
#include "led_strip.h"

// Configuração do SDK
#include "sdkconfig.h"

// Manipulação de erros na pilha de protocolos de Internet (lwIP)
#include "lwip/err.h"
#include "lwip/sys.h"

// Servidor HTTP do ESP32
#include "esp_http_server.h"

// Configuração de interfaces de rede
#include "esp_netif.h"

// Funções comuns de exemplo para protocolos
#include "protocol_examples_common.h"

// Controlador analógico digital (ADC)
#include "driver/adc.h"

// Controlador de LED de Alta Eficiência (LEDC)
#include "driver/ledc.h"

#include "esp_timer.h"   // Para medir o tempo
#include "driver/gpio.h" // Para manipulação de GPIO

// Definição de pinos utilizados
#define BLINK_GPIO 2 // Pino de LED para testes

// Parâmetros do sensor de umidade do solo
#define ANALOG_GPIO 34
#define DIGITAL_GPIO 25
#define WATER_PUMP_GPIO 26



// Parâmetros WiFi
#define WIFI_SSID      "Redmi Note 12S"
#define WIFI_PASS      "jatefalo"
#define WIFI_MAXIMUM_RETRY  5
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static const char *TAG = "umidade_sensor";
static uint8_t s_led_state = 0;
static int update_interval = 1000; // Intervalo padrão de 1000 ms (1 segundo)

// Controla o estado do LED
static void blink_led(void)
{
    gpio_set_level(BLINK_GPIO, s_led_state);
}

// Configuração inicial do LED
static void configure_led(void)
{
    ESP_LOGI(TAG, "Example configured to blink GPIO LED!");
    gpio_reset_pin(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
}

// Configuração inicial do sensor de umidade do solo
static void configure_soil_moisture_sensor(void)
{
    // Configuração do pino analógico do sensor de umidade do solo
    adc1_config_width(ADC_WIDTH_BIT_12); // Configura a largura do ADC
    adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11); // Configura a atenuação para o canal 8 (GPIO 25)
}

void configure_io(void) {
    // Configura o pino do LED como saída
    gpio_reset_pin(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);

    // Configura o pino analógico do sensor
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11); // GPIO 34

    // Configura o pino digital do sensor
    gpio_reset_pin(DIGITAL_GPIO);
    gpio_set_direction(DIGITAL_GPIO, GPIO_MODE_INPUT);

    // Configura o pino de saida da bomba
    gpio_reset_pin(WATER_PUMP_GPIO);
    gpio_set_direction(WATER_PUMP_GPIO, GPIO_MODE_OUTPUT);

}

// Função para medir a umidade usando o sensor de umidade do solo
float measure_soil_moisture(void)
{
    int adc_value;
    adc_value = adc1_get_raw(ADC1_CHANNEL_6); // Lê o valor analógico do pino GPIO 34

    // Ajuste para que 4095 seja 0% e 0 seja 100%
    float moisture_percentage = ((4095 - adc_value) * 100) / 4095;

    return moisture_percentage;
}


// Definição da página HTML
// Função para manipular a página HTML
esp_err_t root_handler(httpd_req_t *req) {
    char resp_str[2630];
    double moisture = measure_soil_moisture();
    snprintf(resp_str, sizeof(resp_str),
    "<html>"
    "<head>"
    "<meta charset=\"UTF-8\">"
    "<style>"
    "body {"
    "display:flex;"
    "flex-direction:column;"
    "align-items:center;"
    "justify-content:center;"
    "height:100vh;"
    "}"
    "button {"
    "font-size:20px;"
    "padding:10px;"
    "margin-top:10px;"
    "font-weight:bold;"
    "}"
    "input {"
    "font-size:16px;"
    "padding:5px;"
    "margin-top:10px;"
    "}"
    ".input-container {"
    "display: flex;"
    "flex-direction: column;"
    "align-items: center;"
    "}"
    ".input-container label {"
    "margin-bottom: 5px;"
    "font-size: 16px;"
    "align-items: flex-start;"
    "}"
    "</style>"
    "</head>"
    "<body>"
    "<h1>Trabalho Final de Micro</h1>"
    "<p>Altera o estado do LED pino 2</p>"
    "<button onclick=\"toggleLed()\">LED Button</button>"
    "<br>"
    "<p id=\"moisture\">Umidade do Solo: %.2f%%</p>"
    "<br>"
    "<div class=\"input-container\">"
    "<label for=\"interval\">Insira o intervalo de tempo para a verificação da umidade:</label>"
    "<input type=\"number\" id=\"interval\" placeholder=\"Intervalo (segundos)\">"
    "<button onclick=\"setCustomInterval()\">Definir Intervalo</button>"
    "</div>"
    "<br>"
    "<div class=\"input-container\">"
    "<label for=\"threshold\">Defina o limite de umidade para acionar a bomba (0-100%%):</label>"
    "<input type=\"number\" id=\"threshold\" placeholder=\"Limite de umidade\">"
    "<button onclick=\"setMoistureThreshold()\">Definir Limite</button>"
    "</div>"
    "<script>"
    "let intervalID;"
    "function toggleLed() {"
    "var xhr=new XMLHttpRequest();"
    "xhr.open('GET','/toggle_led',true);"
    "xhr.send();"
    "}"
    "function updateValues() {"
    "console.log('Updating values...');"
    "var xhr=new XMLHttpRequest();"
    "xhr.onreadystatechange=function() {"
    "if (xhr.readyState==4 && xhr.status==200) {"
    "var values=JSON.parse(xhr.responseText);"
    " document.getElementById('moisture').innerHTML='Umidade do Solo: '+values.moisture+'%%';"
    "}"
    "};"
    "xhr.open('GET','/values',true);"
    "xhr.send();"
    "}"
    "function setCustomInterval() {"
    "var interval=document.getElementById('interval').value;"
    "if (interval>0 && interval<=7200) {"
    "clearInterval(intervalID);"
    "intervalID=setInterval(updateValues,interval*1000);"
    "var xhr=new XMLHttpRequest();"
    "xhr.open('POST','/set_interval',true);"
    "xhr.setRequestHeader('Content-Type','application/x-www-form-urlencoded');"
    "xhr.onreadystatechange=function () {"
    "if(xhr.readyState === 4 && xhr.status === 200) {"
    "console.log('Intervalo atualizado');"
    "}"
    "};"
    "xhr.send('interval='+interval);"
    "}else{"
    "alert('Por favor, Informe um valor entre 1 e 7200 segundos.');"
    "}"
    "}"
    "function setMoistureThreshold() {"
    "var threshold=document.getElementById('threshold').value;"
    "if(threshold>=0 && threshold<=100) {"
    "var xhr=new XMLHttpRequest();"
    "xhr.open('POST','/set_threshold',true);"
    "xhr.setRequestHeader('Content-Type','application/x-www-form-urlencoded');"
    "xhr.send('threshold='+threshold);"
    "}else{"
    "alert('Por favor, informe um valor entre 0 e 100.');"
    "}"
    "}"
    "intervalID=setInterval(updateValues,1000);"
    "</script>"
    "</body>"
    "</html>",
    moisture);

    // Envio da resposta HTML ao cliente
    httpd_resp_send(req, resp_str, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}


esp_err_t set_interval_handler(httpd_req_t *req) {
    char buf[100]; // Garante espaço sufeiciente para até 7200 segundos
    int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (ret <= 0) {
        if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
            httpd_resp_send_408(req);
        }
        return ESP_FAIL;
    }

    buf[ret] = '\0'; // Garante que a string está corretamente terminada
    char *interval_str = strstr(buf, "interval=");
    if (interval_str) {
        interval_str += strlen("interval=");
        int interval = atoi(interval_str);
        if (interval > 0 && interval <= 7200) { // Verifica se o intervalo é válido e menor ou igual a 7200 segundos
            update_interval = interval * 1000; // Converte de segundos para milissegundos
        } else {
            // Se o intervalo for inválido, retorne uma mensagem de erro
            httpd_resp_send(req, "Invalid interval. Please enter a value between 1 and 7200 seconds.", HTTPD_RESP_USE_STRLEN);
            return ESP_FAIL;
        }
    } else {
        // Se o parâmetro não for encontrado, retorne uma mensagem de erro
        httpd_resp_send(req, "Parameter 'interval' not found.", HTTPD_RESP_USE_STRLEN);
        return ESP_FAIL;
    }

    httpd_resp_send(req, "Interval updated", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

httpd_uri_t set_interval = {
    .uri       = "/set_interval",
    .method    = HTTP_POST,
    .handler   = set_interval_handler,
    .user_ctx  = NULL
};

static int moisture_threshold = 30; // Limite padrão de umidade para acionar a bomba (30%)
esp_err_t set_threshold_handler(httpd_req_t *req) {
    char buf[100];
    int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (ret <= 0) {
        if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
            httpd_resp_send_408(req);
        }
        return ESP_FAIL;
    }

    buf[ret] = '\0';
    char *threshold_str = strstr(buf, "threshold=");
    if (threshold_str) {
        threshold_str += strlen("threshold=");
        int threshold = atoi(threshold_str);
        if (threshold >= 0 && threshold <= 100) { // Verifica se o valor é válido
            moisture_threshold = threshold; // Atualiza o limite de umidade
            ESP_LOGI(TAG, "Limite de umidade atualizado para: %d%%", moisture_threshold);
        } else {
            httpd_resp_send(req, "Invalid threshold. Please enter a value between 0 and 100.", HTTPD_RESP_USE_STRLEN);
            return ESP_FAIL;
        }
    } else {
        httpd_resp_send(req, "Parameter 'threshold' not found.", HTTPD_RESP_USE_STRLEN);
        return ESP_FAIL;
    }

    httpd_resp_send(req, "Threshold updated", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

httpd_uri_t set_threshold = {
    .uri       = "/set_threshold",
    .method    = HTTP_POST,
    .handler   = set_threshold_handler,
    .user_ctx  = NULL
};

// Função para ler os dados do sensor de umidade e enviar para página HTML
esp_err_t values_handler(httpd_req_t *req) {
    float moisture = measure_soil_moisture(); // Mede a umidade do solo usando o sensor resistivo

    char resp_str[100];
    snprintf(resp_str, sizeof(resp_str), "{\"moisture\": %.2f}", moisture);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, resp_str, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}


// Definição da rota no servidor para enviar os dados do sensor de umidade
httpd_uri_t values = {
    .uri       = "/values", // URI da rota
    .method    = HTTP_GET, // Método HTTP
    .handler   = values_handler, // Função referência
    .user_ctx  = NULL
};

// Definição da rota principal do servidor
httpd_uri_t root = {
    .uri       = "/", // URI da rota
    .method    = HTTP_GET, // Método HTTP
    .handler   = root_handler, // Função de referência
    .user_ctx  = NULL
};


// Função para alterar o estado do LED
esp_err_t toggle_led_handler(httpd_req_t *req) {
    s_led_state = (s_led_state == 1) ? 0 : 1;
    blink_led();
    httpd_resp_send(req, "LED state toggled", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

// Rota para alterar o estado do LED
httpd_uri_t toggle_led = {
    .uri       = "/toggle_led",
    .method    = HTTP_GET,
    .handler   = toggle_led_handler,
    .user_ctx  = NULL
};


// Função para inicializar o servidor web
static httpd_handle_t start_webserver(void) {
    // Configuração padrão do servidor HTTPD
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    // Variável para representar o servidor
    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) == ESP_OK) {
        // Registra os manipuladores de URI (Uniform Resource Identifier)
        // Estes manipuladores são responsáveis por lidar com requisições HTTP específicas
        httpd_register_uri_handler(server, &root);
        httpd_register_uri_handler(server, &toggle_led);
        httpd_register_uri_handler(server, &values);
        httpd_register_uri_handler(server, &set_interval);
        httpd_register_uri_handler(server, &set_threshold);
    }
    return server;
}

// Função para finalizar o web server
void stop_webserver(httpd_handle_t server) {
    httpd_stop(server);
}

static int s_retry_num = 0; // Tentativas para conectar ao WiFi

// Manipulador de eventos relacionados a conexão WiFi
static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    // Verifica se o evento é relacionado à inicialização da estação Wi-Fi
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect(); // Inicia a conexão WiFi
    // Verifica se o evento é relacionado à desconexão da estação Wi-Fi
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        // Verifica se ainda há tentativas de reconexão disponíveis
        if (s_retry_num < WIFI_MAXIMUM_RETRY) {
            esp_wifi_connect(); // Tenta reconectar à rede Wi-Fi
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            // Define o bit de falha de conexão para indicar que todas as tentativas falharam
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT); 
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    // Verifica se o evento é relacionado ao estabelecimento de uma conexão IP
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data; // Obtém informações do evento de conexão IP
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip)); // Exibe o endereço IP obtido
        s_retry_num = 0;
        // Define o bit de conexão bem-sucedida para indicar que a conexão foi estabelecida
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

// Função para inicializar a conexão Wi-Fi
void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate(); // Cria um grupo de eventos para gerenciar os eventos WiFi

    ESP_ERROR_CHECK(esp_netif_init()); // Inicializa a interface de rede do ESP32

    ESP_ERROR_CHECK(esp_event_loop_create_default()); // Cria o loop de eventos padrão

    esp_netif_create_default_wifi_sta(); // Cria a interface de estação Wi-Fi padrão

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT(); // Configurações padrão de inicialização do Wi-Fi
    ESP_ERROR_CHECK(esp_wifi_init(&cfg)); // Inicializa o Wi-Fi com as configurações especificadas

    // Registra os manipuladores de eventos Wi-Fi e IP para tratar eventos relacionados ao Wi-Fi
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        NULL));

    wifi_config_t wifi_config = { // Configurações de conexão Wi-Fi
        .sta = {
            .ssid = WIFI_SSID, // SSID da rede Wi-Fi
            .password = WIFI_PASS, // Senha da rede Wi-Fi
            .threshold.authmode = WIFI_AUTH_WPA2_PSK, // Modo de autenticação
            .pmf_cfg = {
                .capable = true, // Capacidade de Gerenciamento de Proteção de Rede (PMF)
                .required = false // PMF não é obrigatório
            },
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) ); // Define o modo Wi-Fi como estação (STA)
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) ); // Configura o Wi-Fi com as configurações especificadas
    ESP_ERROR_CHECK(esp_wifi_start() ); // Inicia o Wi-Fi

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    // Aguarda até que a conexão Wi-Fi seja estabelecida (WIFI_CONNECTED_BIT) ou que todas as tentativas de conexão falhem (WIFI_FAIL_BIT)
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    // Verifica qual bit foi definido para determinar o estado da conexão Wi-Fi
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 WIFI_SSID, WIFI_PASS);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 WIFI_SSID, WIFI_PASS);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
}

void moisture_task(void *pvParameter) {
    while (1) {
        float moisture = measure_soil_moisture();
        ESP_LOGI(TAG, "Umidade do solo: %.2f%%", moisture);
        
        // Verifica se a umidade está abaixo do limite definido pelo usuário
        if (moisture < moisture_threshold) {
            gpio_set_level(WATER_PUMP_GPIO, 1); // Liga a bomba d'água
            ESP_LOGI(TAG, "Bomba d'água LIGADA (Umidade abaixo do limite de %d%%)", moisture_threshold);

            // Mantém a bomba ligada por 10 segundos
            vTaskDelay(10000 / portTICK_PERIOD_MS); // Espera 10 segundos

            gpio_set_level(WATER_PUMP_GPIO, 0); // Desliga a bomba d'água
            ESP_LOGI(TAG, "Bomba d'água DESLIGADA após 10 segundos");
        } else {
            gpio_set_level(WATER_PUMP_GPIO, 0); // Desliga a bomba d'água
            ESP_LOGI(TAG, "Bomba d'água DESLIGADA (Umidade dentro do limite de %d%%)", moisture_threshold);
        }

        vTaskDelay(update_interval / portTICK_PERIOD_MS);
    }
}

// Função principal
void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    wifi_init_sta();

    configure_io();
    configure_led();
    configure_soil_moisture_sensor();

    httpd_handle_t server = start_webserver();
    if (server == NULL) {
        ESP_LOGE(TAG, "Failed to start the web server");
    } else {
        httpd_register_uri_handler(server, &set_interval);
    }

    xTaskCreate(&moisture_task, "moisture_task", 4096, NULL, 5, NULL);
}
