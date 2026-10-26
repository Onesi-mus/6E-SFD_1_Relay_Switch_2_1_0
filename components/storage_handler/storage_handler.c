#include "storage_handler.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_spiffs.h"
#include "esp_log.h"
#include "cJSON.h"
#include "nvs.h"  //use to store and retrieve data

#include "app_config.h"
#include "relay_controls.h"


//SPIFFS
#define SPIFFS_PART_LABEL "spiffs"
#define AWS_PATH "/spiffs/aws.json"


//NVS Namespaces
#define WIFI_NS  "wifi_creds"
#define NVS_NAMESPACE  "storage" 
#define PROV_FLAG_KEY  "prov_flag"
#define NVS_RELAY_NS  "relay_state"

//Forward deceleration
bool storage_handler_save_certs(const char *cert_pem, const char *key_pem);
bool storage_handler_load_certs(char **cert_pem, char **key_pem);
esp_err_t storage_handler_set_prov_status_flag(bool status);
esp_err_t storage_handler_get_prov_status_flag(bool *status);
bool storage_handler_load_relay_state(sig_r_s_t *state);
bool storage_handler_save_relay_state(const sig_r_s_t *state);



const char *STORE_TAG = "STORE";

//SPIFFS SECTION
//function for the spiffs mount
void storage_handler_mount(void){
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = SPIFFS_PART_LABEL,
        .max_files = 5,
        .format_if_mount_failed = true
    };
    ESP_ERROR_CHECK(esp_vfs_spiffs_register(&conf));
    size_t total = 0, used = 0;
    ESP_ERROR_CHECK(esp_spiffs_info(conf.partition_label, &total, &used));
    ESP_LOGI(STORE_TAG, "SPIFFS total : %d, used: %d", total, used);
}

//Function to save iot certificates to SPIFFS
bool storage_handler_save_certs(const char *cert_pem, const char *key_pem){
    cJSON *root = cJSON_CreateObject();
    if (!root){
        ESP_LOGE(STORE_TAG, "CJson creation for cert storage failed...");
        return false;
    }
    cJSON_AddStringToObject(root, "certificatePem", cert_pem);
    cJSON_AddStringToObject(root, "privateKey", key_pem);
    char *json = cJSON_PrintUnformatted(root);
    if (!json){
        ESP_LOGE(STORE_TAG, "Failed to create the json for cert storing...");
        return false;
    }
    FILE *f = fopen(AWS_PATH, "w");
    if (!f){
        free(json);
        return false;
    }
    fwrite(json, 1, strlen(json), f);
    fclose(f);
    free(json);
    cJSON_Delete(root);
    ESP_LOGI(STORE_TAG, "certs saved successfully...");
    return true;
}

//Function to load the AWS certificates
bool storage_handler_load_certs(char **cert_pem, char **key_pem) {
    FILE *f = fopen(AWS_PATH, "r");
    if (!f) {
        ESP_LOGW(STORE_TAG, "No AWS credentials file found...");
        return false;
    }
    fseek(f, 0, SEEK_END);
    long len = ftell(f);
    fseek(f, 0, SEEK_SET);
    char *buf = (char*)malloc(len + 1);
    if (!buf) { fclose(f); return false; }
    fread(buf, 1, len, f);
    buf[len] = '\0';
    fclose(f);

    cJSON *root = cJSON_Parse(buf);
    free(buf);
    if (!root) { return false; }

    cJSON *cert = cJSON_GetObjectItem(root, "certificatePem");
    cJSON *key  = cJSON_GetObjectItem(root, "privateKey");
    if (!cJSON_IsString(cert) || !cJSON_IsString(key)) {
        cJSON_Delete(root);
        return false;
    }

    *cert_pem = strdup(cert->valuestring);
    *key_pem  = strdup(key->valuestring);
    cJSON_Delete(root);

    if (!*cert_pem || !*key_pem) {
        if (*cert_pem) free(*cert_pem);
        if (*key_pem) free(*key_pem);
        *cert_pem = NULL; *key_pem = NULL;
        return false;
    }
    ESP_LOGI(STORE_TAG, "Loaded AWS certs...");
    return true;
}


//NVS SECTION
//Function to load wifi credentials from nvs
bool storage_handler_load_wifi_credentials(char *ssid, size_t ssid_len, char *password, size_t pass_len){
    nvs_handle_t wifi_handle;
    esp_err_t err = nvs_open(WIFI_NS, NVS_READONLY, &wifi_handle);
    if (err != ESP_OK) {
        ESP_LOGI(STORE_TAG, "No wifi creds found....");
        return false;
    }
    size_t s_len = ssid_len;  //To avoid overwrite  
    size_t p_len = pass_len;
    err = nvs_get_str(wifi_handle, "ssid", ssid, &s_len);
    if (err != ESP_OK){
        ESP_LOGI(STORE_TAG, "Failed to get ssid");
        nvs_close(wifi_handle);
        return false;
    }
    err = nvs_get_str(wifi_handle, "password", password, &p_len);
    nvs_close(wifi_handle);
    if (err != ESP_OK){
        ESP_LOGI(STORE_TAG, "Failed to get password");
        return false;
    }
    ESP_LOGI(STORE_TAG, "Wifi creds loaded successfully....");
    return true;
}



//Function to save wifi credentials to nvs
bool storage_handler_save_wifi_credentials(const char* ssid, const char* password){
    nvs_handle_t wifi_handle;
    esp_err_t err = nvs_open(WIFI_NS, NVS_READWRITE, &wifi_handle);
    if (err != ESP_OK) return false;
    err = nvs_set_str(wifi_handle, "ssid", ssid);
    if (err != ESP_OK){
        ESP_LOGI(STORE_TAG, "Failed to set ssid");
        nvs_close(wifi_handle);
        return false;
    }
    err = nvs_set_str(wifi_handle, "password", password);
    if (err != ESP_OK){
        ESP_LOGI(STORE_TAG, "Filed to set password");
        nvs_close(wifi_handle);
        return false;
    }
    nvs_commit(wifi_handle);
    nvs_close(wifi_handle);
    ESP_LOGI(STORE_TAG, "WIFI creds saved successfully...");
    return true;
}

//Function to set the provision flag
esp_err_t storage_handler_set_prov_status_flag(bool status){
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return err;

    uint8_t value = status ? 1 : 0;
    err = nvs_set_u8(my_handle, PROV_FLAG_KEY, value);
    if (err == ESP_OK) {
        err = nvs_commit(my_handle);
    }
    nvs_close(my_handle);
    ESP_LOGI(STORE_TAG, "Provisioning status flag set to: %d", value);
    return err;
}

//Function to load the provisioning flag
esp_err_t storage_handler_get_prov_status_flag(bool *status){
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &my_handle);
    if (err != ESP_OK) return err;

    uint8_t value = 0; 
    err = nvs_get_u8(my_handle, PROV_FLAG_KEY, &value);
    if (err == ESP_OK) {
        *status = (value == 1);
    } else if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(STORE_TAG, "No yet provisioned...");
        *status = false; 
        err = ESP_OK;
    }
    nvs_close(my_handle);
    return err;
}


poweron_mode_t storage_handler_load_poweron_mode(void){
    nvs_handle_t h;
    if (nvs_open(NVS_RELAY_NS, NVS_READONLY, &h) != ESP_OK) return POWERON_MEMORY;
    uint8_t m = POWERON_MEMORY;
    esp_err_t err = nvs_get_u8(h, "poweron", &m);
    nvs_close(h);
    if (err != ESP_OK || m > POWERON_MEMORY) return POWERON_MEMORY;
    return (poweron_mode_t)m;
}

void storage_handler_save_poweron_mode(poweron_mode_t mode){
    nvs_handle_t h;
    if (nvs_open(NVS_RELAY_NS, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_u8(h, "poweron", (uint8_t)mode);
    nvs_commit(h);
    nvs_close(h);
    ESP_LOGI(STORE_TAG, "Power-on mode saved: %d", (int)mode);
}

//Function to load relay state from NVS
bool storage_handler_load_relay_state(sig_r_s_t *state){
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_RELAY_NS, NVS_READONLY, &h);
    if (err != ESP_OK) {
        ESP_LOGW(STORE_TAG, "No relay state in NVS");
        return false;
    }

    uint8_t val8;
    if (nvs_get_u8(h, "relay_on", &val8) == ESP_OK) state->relay_on = val8;
    else { nvs_close(h); return false; }

    nvs_close(h);
    ESP_LOGI(STORE_TAG, "Loaded relay state from NVS (R=%d)", state->relay_on);
    return true;
}


//Function to save relay state to NVS
bool storage_handler_save_relay_state(const sig_r_s_t *state){
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_RELAY_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) return false;

    nvs_set_u8(h, "relay_on", state->relay_on ? 1 : 0);

    nvs_commit(h);
    nvs_close(h);
    ESP_LOGI(STORE_TAG, "Saved relay state to NVS");
    return true;
}

//
//MESH
//
#define NVS_MESH_NS "mesh"

bool storage_handler_load_mesh_group(char *out, size_t len){
    nvs_handle_t h;
    if (nvs_open(NVS_MESH_NS, NVS_READONLY, &h) != ESP_OK) return false;
    size_t l = len;
    esp_err_t err = nvs_get_str(h, "group", out, &l);
    nvs_close(h);
    if (err != ESP_OK || out[0] == '\0') return false;
    return true;
}

bool storage_handler_save_mesh_group(const char *group_id){
    nvs_handle_t h;
    if (nvs_open(NVS_MESH_NS, NVS_READWRITE, &h) != ESP_OK) return false;
    esp_err_t err = nvs_set_str(h, "group", group_id ? group_id : "");
    nvs_commit(h);
    nvs_close(h);
    ESP_LOGI(STORE_TAG, "Mesh group saved: '%s'", group_id ? group_id : "");
    return err == ESP_OK;
}

bool storage_handler_load_mesh_enabled(void){
    nvs_handle_t h;
    if (nvs_open(NVS_MESH_NS, NVS_READONLY, &h) != ESP_OK) return false;
    uint8_t v = 0;
    esp_err_t err = nvs_get_u8(h, "enabled", &v);
    nvs_close(h);
    return (err == ESP_OK) && v;
}

void storage_handler_save_mesh_enabled(bool enabled){
    nvs_handle_t h;
    if (nvs_open(NVS_MESH_NS, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_u8(h, "enabled", enabled ? 1 : 0);
    nvs_commit(h);
    nvs_close(h);
    ESP_LOGI(STORE_TAG, "Mesh %s", enabled ? "enabled" : "disabled");
}

uint8_t storage_handler_load_home_channel(void){
    nvs_handle_t h;
    if (nvs_open(NVS_MESH_NS, NVS_READONLY, &h) != ESP_OK) return 0;
    uint8_t v = 0;
    if (nvs_get_u8(h, "chan", &v) != ESP_OK) v = 0;
    nvs_close(h);
    return v;
}

void storage_handler_save_home_channel(uint8_t ch){
    if (ch < 1 || ch > 13) return;
    if (storage_handler_load_home_channel() == ch) return;   //avoid NVS churn
    nvs_handle_t h;
    if (nvs_open(NVS_MESH_NS, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_u8(h, "chan", ch);
    nvs_commit(h);
    nvs_close(h);
}

//
//UNLINK
//
//Owner identity only. Two things are deliberately NOT erased because neither can
//be recreated without a bench:
//  /spiffs/aws.json  device cert + private key — Firmware B has no fleet
//                    provisioning, so a wiped device boot-loops forever
//  bl0937_cal        energy-meter K-factors written by the factory jig
//See MESH_BACKEND_CONTRACT.md.
static void nvs_wipe_ns(const char *ns){
    nvs_handle_t h;
    if (nvs_open(ns, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_erase_all(h);
    nvs_commit(h);
    nvs_close(h);
}

bool storage_handler_unlink(void){
    static const char *ns[] = { WIFI_NS, NVS_NAMESPACE, NVS_RELAY_NS, NVS_MESH_NS, "rocker" };
    for (size_t i = 0; i < sizeof(ns)/sizeof(ns[0]); i++) nvs_wipe_ns(ns[i]);

    //Re-read rather than trust the erase.
    char ssid[32], pass[64], group[64];
    bool ok = !storage_handler_load_wifi_credentials(ssid, sizeof(ssid), pass, sizeof(pass)) &&
              !storage_handler_load_mesh_group(group, sizeof(group));
    ESP_LOGW(STORE_TAG, "Unlink: %s", ok ? "erased." : "erase INCOMPLETE.");
    return ok;
}



