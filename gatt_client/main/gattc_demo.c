/*
   Dejan Milojica
*/



/****************************************************************************
*
* This demo showcases BLE GATT client. It can scan BLE devices and connect to one device.
* Run the gatt_server demo, the client demo will automatically connect to the gatt_server demo.
* Client demo will enable gatt_server's notify after connection. The two devices will then exchange
* data.
*
****************************************************************************/

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "nvs.h"
#include "nvs_flash.h"

#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"


#define GATTC_TAG "GATTC_DEMO"
#define REMOTE_SERVICE_UUID        0x00FF
#define REMOTE_NOTIFY_CHAR_UUID    0xFF01
#define PROFILE_NUM      1
#define PROFILE_A_APP_ID 0
#define INVALID_HANDLE   0

#define PERION_TRANSFERA_PODATAKA        (3000)   // milliseconds, PERIOD MJERENJA!
#include "parametri.h"
#define INTR_FLAG_DEFAULT 0

/*PINOVI: */
#define KONFIGURACIONI_PIN  GPIO_NUM_19
#define VENTIL_PIN          GPIO_NUM_21
#define KALIBRACIJA_PIN 	GPIO_NUM_5


static xQueueHandle gpio_evt_queue = NULL;
static bool VENTIL_UKLJUCEN_NA_DUGME = false;

static const char remote_device_name[] = "ESP_GATTS_DEMO";
static bool connect    = false;
static bool get_server = false;
static esp_gattc_char_elem_t *char_elem_result   = NULL;
static esp_gattc_descr_elem_t *descr_elem_result = NULL;

/* Declare static functions */
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
static void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);


static esp_bt_uuid_t remote_filter_service_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {.uuid16 = REMOTE_SERVICE_UUID,},
};

static esp_bt_uuid_t remote_filter_char_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {.uuid16 = REMOTE_NOTIFY_CHAR_UUID,},
};

static esp_bt_uuid_t notify_descr_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG,},
};

static esp_ble_scan_params_t ble_scan_params = {
    .scan_type              = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval          = 0x50,
    .scan_window            = 0x30,
    .scan_duplicate         = BLE_SCAN_DUPLICATE_DISABLE
};

struct gattc_profile_inst {
    esp_gattc_cb_t gattc_cb;
    uint16_t gattc_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_start_handle;
    uint16_t service_end_handle;
    uint16_t char_handle;
    esp_bd_addr_t remote_bda;
};

/* One gatt-based profile one app_id and one gattc_if, this array will store the gattc_if returned by ESP_GATTS_REG_EVT */
static struct gattc_profile_inst gl_profile_tab[PROFILE_NUM] = {
    [PROFILE_A_APP_ID] = {
        .gattc_cb = gattc_profile_event_handler,
        .gattc_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};

/*NAJBITNIJA PROCEDURA ZA TRANSFER PODATAKA!*/
static void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    esp_ble_gattc_cb_param_t *p_data = (esp_ble_gattc_cb_param_t *)param;

    switch (event) {
    case ESP_GATTC_REG_EVT:
        ESP_LOGI(GATTC_TAG, "REG_EVT");
        esp_err_t scan_ret = esp_ble_gap_set_scan_params(&ble_scan_params);
        if (scan_ret){
            ESP_LOGE(GATTC_TAG, "set scan params error, error code = %x", scan_ret);
        }
        break;
    case ESP_GATTC_CONNECT_EVT:{
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_CONNECT_EVT conn_id %d, if %d", p_data->connect.conn_id, gattc_if);
        gl_profile_tab[PROFILE_A_APP_ID].conn_id = p_data->connect.conn_id;
        memcpy(gl_profile_tab[PROFILE_A_APP_ID].remote_bda, p_data->connect.remote_bda, sizeof(esp_bd_addr_t));
        ESP_LOGI(GATTC_TAG, "REMOTE BDA:");
        esp_log_buffer_hex(GATTC_TAG, gl_profile_tab[PROFILE_A_APP_ID].remote_bda, sizeof(esp_bd_addr_t));
        esp_err_t mtu_ret = esp_ble_gattc_send_mtu_req (gattc_if, p_data->connect.conn_id);
        if (mtu_ret){
            ESP_LOGE(GATTC_TAG, "config MTU error, error code = %x", mtu_ret);
        }
        break;
    }
    case ESP_GATTC_OPEN_EVT:
        if (param->open.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "open failed, status %d", p_data->open.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "open success");
        break;
    case ESP_GATTC_CFG_MTU_EVT:
        if (param->cfg_mtu.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG,"config mtu failed, error status = %x", param->cfg_mtu.status);
        }
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_CFG_MTU_EVT, Status %d, MTU %d, conn_id %d", param->cfg_mtu.status, param->cfg_mtu.mtu, param->cfg_mtu.conn_id);
        esp_ble_gattc_search_service(gattc_if, param->cfg_mtu.conn_id, &remote_filter_service_uuid);
        break;
    case ESP_GATTC_SEARCH_RES_EVT: {
        ESP_LOGI(GATTC_TAG, "SEARCH RES: conn_id = %x is primary service %d", p_data->search_res.conn_id, p_data->search_res.is_primary);
        ESP_LOGI(GATTC_TAG, "start handle %d end handle %d current handle value %d", p_data->search_res.start_handle, p_data->search_res.end_handle, p_data->search_res.srvc_id.inst_id);
        if (p_data->search_res.srvc_id.uuid.len == ESP_UUID_LEN_16 && p_data->search_res.srvc_id.uuid.uuid.uuid16 == REMOTE_SERVICE_UUID) {
            ESP_LOGI(GATTC_TAG, "service found");
            get_server = true;
            gl_profile_tab[PROFILE_A_APP_ID].service_start_handle = p_data->search_res.start_handle;
            gl_profile_tab[PROFILE_A_APP_ID].service_end_handle = p_data->search_res.end_handle;
            ESP_LOGI(GATTC_TAG, "UUID16: %x", p_data->search_res.srvc_id.uuid.uuid.uuid16);
        }
        break;
    }
    case ESP_GATTC_SEARCH_CMPL_EVT:
        if (p_data->search_cmpl.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "search service failed, error status = %x", p_data->search_cmpl.status);
            break;
        }
        if(p_data->search_cmpl.searched_service_source == ESP_GATT_SERVICE_FROM_REMOTE_DEVICE) {
            ESP_LOGI(GATTC_TAG, "Get service information from remote device");
        } else if (p_data->search_cmpl.searched_service_source == ESP_GATT_SERVICE_FROM_NVS_FLASH) {
            ESP_LOGI(GATTC_TAG, "Get service information from flash");
        } else {
            ESP_LOGI(GATTC_TAG, "unknown service source");
        }
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_SEARCH_CMPL_EVT");
        if (get_server){
            uint16_t count = 0;
            esp_gatt_status_t status = esp_ble_gattc_get_attr_count( gattc_if,
                                                                     p_data->search_cmpl.conn_id,
                                                                     ESP_GATT_DB_CHARACTERISTIC,
                                                                     gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
                                                                     gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
                                                                     INVALID_HANDLE,
                                                                     &count);
            if (status != ESP_GATT_OK){
                ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_attr_count error");
            }

            if (count > 0){				
                char_elem_result = (esp_gattc_char_elem_t *)malloc(sizeof(esp_gattc_char_elem_t) * count);
                if (!char_elem_result){
                    ESP_LOGE(GATTC_TAG, "gattc no mem");
                }else{
                    status = esp_ble_gattc_get_char_by_uuid( gattc_if,
                                                             p_data->search_cmpl.conn_id,
                                                             gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
                                                             gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
                                                             remote_filter_char_uuid,
                                                             char_elem_result,
                                                             &count);
                    if (status != ESP_GATT_OK){
                        ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_char_by_uuid error");
                    }

                    /*  Every service have only one char in our 'ESP_GATTS_DEMO' demo, so we used first 'char_elem_result' */
                    if (count > 0 && (char_elem_result[0].properties & ESP_GATT_CHAR_PROP_BIT_NOTIFY)){
                        gl_profile_tab[PROFILE_A_APP_ID].char_handle = char_elem_result[0].char_handle;
                        esp_ble_gattc_register_for_notify (gattc_if, gl_profile_tab[PROFILE_A_APP_ID].remote_bda, char_elem_result[0].char_handle);
                    }					
                }
                /* free char_elem_result */
                free(char_elem_result);
            }else{
                ESP_LOGE(GATTC_TAG, "no char found");
            }
        }
         break;
		 
		 
    case ESP_GATTC_REG_FOR_NOTIFY_EVT: {
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_REG_FOR_NOTIFY_EVT");
        if (p_data->reg_for_notify.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "REG FOR NOTIFY failed: error status = %d", p_data->reg_for_notify.status);
        }else{
            uint16_t count = 0;
            uint16_t notify_en = 1;
            esp_gatt_status_t ret_status = esp_ble_gattc_get_attr_count( gattc_if,
                                                                         gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                                                                         ESP_GATT_DB_DESCRIPTOR,
                                                                         gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
                                                                         gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
                                                                         gl_profile_tab[PROFILE_A_APP_ID].char_handle,
                                                                         &count);
            if (ret_status != ESP_GATT_OK){
                ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_attr_count error");
            }
            if (count > 0){
                descr_elem_result = malloc(sizeof(esp_gattc_descr_elem_t) * count);
                if (!descr_elem_result){
                    ESP_LOGE(GATTC_TAG, "malloc error, gattc no mem");
                }else{
                    ret_status = esp_ble_gattc_get_descr_by_char_handle( gattc_if,
                                                                         gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                                                                         p_data->reg_for_notify.handle,
                                                                         notify_descr_uuid,
                                                                         descr_elem_result,
                                                                         &count);
                    if (ret_status != ESP_GATT_OK){
                        ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_descr_by_char_handle error");
                    }
                    /* Every char has only one descriptor in our 'ESP_GATTS_DEMO' demo, so we used first 'descr_elem_result' */
                    if (count > 0 && descr_elem_result[0].uuid.len == ESP_UUID_LEN_16 && descr_elem_result[0].uuid.uuid.uuid16 == ESP_GATT_UUID_CHAR_CLIENT_CONFIG){
                        ret_status = esp_ble_gattc_write_char_descr( gattc_if,
                                                                     gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                                                                     descr_elem_result[0].handle,
                                                                     sizeof(notify_en),
                                                                     (uint8_t *)&notify_en,
                                                                     ESP_GATT_WRITE_TYPE_RSP,
                                                                     ESP_GATT_AUTH_REQ_NONE);
                    }

                    if (ret_status != ESP_GATT_OK){
                        ESP_LOGE(GATTC_TAG, "esp_ble_gattc_write_char_descr error");
                    }

                    /* free descr_elem_result */
                    free(descr_elem_result);
                }
            }
            else{
                ESP_LOGE(GATTC_TAG, "decsr not found");
            }

        }
        break;
    }
	
	
/*PRIMANJE PODATAKA SA SERVERA: */	
    case ESP_GATTC_NOTIFY_EVT:
        if (p_data->notify.is_notify){
			printf("\n\n");
			printf("----------------------------------------------------------------------------------\n");
			printf("PRIJEM PODATAKA SA SERVERA: \n");
        }else{
            ESP_LOGI(GATTC_TAG, "ESP_GATTC_NOTIFY_EVT, receive indicate value:");
        }
		/*for(int i=0;i<p_data->notify.value_len;i++)
			printf("%c",p_data->notify.value[i]);*/
		
		/*ISPIS PRIMLJENIH PODATAKA: */
		float V_TEMPERATURA = 0.0;
		bool IMA_PROTOKA = false;
		float V_PH = 0.0;
		int tarabe_na_pozicijama[3] = {0};
			
		//IMAMO PRIMLJENJE PODATKE SA SENZORA:
		int brojac = 0;
		for(int i=0;i<p_data->notify.value_len;i++){
			 if(p_data->notify.value[i]=='#')
				 tarabe_na_pozicijama[brojac++] = i;
		}			
		
		/*Analiza primljenih vrijednosti: */
		/*1.)*/
		printf("TEMPERATURNI SENZOR: \n");
		printf("\t\tVrijednost Temperature: ");
		char vrijednosti[5];
		int k=0;
		for(int i=0;i<tarabe_na_pozicijama[0];i++){
			vrijednosti[k++] = p_data->notify.value[i];	
		}
		V_TEMPERATURA = atof(vrijednosti);
		printf("%.2f [oC]\n",V_TEMPERATURA);
		
		/*2.)*/
		printf("PROTOCNI SENZOR: \n");
		printf("\t\tProtok: ");
		for(int i=tarabe_na_pozicijama[0]+1;i<tarabe_na_pozicijama[1];i++){
			if(p_data->notify.value[i] == 'I')
				IMA_PROTOKA = true;
			printf("%c",p_data->notify.value[i]);
		}
		printf("\n");
	  
		/*3.)*/
		printf("pH SENZOR: \n");
		printf("\t\tpH Vrijednost: ");
		char v_ph_string[4];
		int br = 0;
		for(int i=tarabe_na_pozicijama[1]+1;i<p_data->notify.value_len-1;i++){
			v_ph_string[br++] = p_data->notify.value[i];
		}		
		V_PH = atof(v_ph_string);
		printf("%.2f",V_PH);
		printf("\n");
		
		/*4.)*/
		printf("VENTIL: \n");
		printf("\t\tVrijednost: ");
		if(VENTIL_ON_OFF==1)
			printf("UKLJUCEN!\n");
		else
			printf("ISKLJUCEN!\n");
		printf("----------------------------------------------------------------------------------\n");

		/*Nakon sto kontrolni cvor primi podatke, ako je VENTIL OTVOREN, a nema PROTOKA, aktivira se alarm!*/
		/*Ako je VENTIL zatvoren, a ima PROTOKA, opet ALARM!*/
		/*Ako je temperatura izvan granica koje su definisane opsegom, opet ALARM!*/
		if(V_TEMPERATURA < TEMP_D_P || V_TEMPERATURA > TEMP_G_P){
			ESP_LOGE(GATTC_TAG, "ALARM: TEMPERATURA IZVAN OPSEGA!");
		}
		if(IMA_PROTOKA == false && VENTIL_ON_OFF == 1){
			ESP_LOGE(GATTC_TAG, "ALARM: VENTIL OTVOREN, A NEMA PROTOKA!");
		}
		if(IMA_PROTOKA == true && VENTIL_ON_OFF == 0){
			ESP_LOGE(GATTC_TAG, "ALARM: VENTIL ZATVOREN, A IMA PROTOKA!");
		}
		if(V_PH < PH_D_P || V_PH > PH_G_P){
			ESP_LOGE(GATTC_TAG, "ALARM: PH VRIJEDNOS IZVAN OPSEGA! OTVARANJE VENTILA!");
			if(!VENTIL_UKLJUCEN_NA_DUGME)
				VENTIL_ON_OFF = 1;
		}else
			if(!VENTIL_UKLJUCEN_NA_DUGME)
				VENTIL_ON_OFF = 0;

        break;
/***/		
		
		
    case ESP_GATTC_WRITE_DESCR_EVT:
        if (p_data->write.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "write descr failed, error status = %x", p_data->write.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "write descr success ");
        uint8_t write_char_data[35];
        for (int i = 0; i < sizeof(write_char_data); ++i)
        {
            write_char_data[i] = i % 256;
        }
        esp_ble_gattc_write_char( gattc_if,
                                  gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                                  gl_profile_tab[PROFILE_A_APP_ID].char_handle,
                                  sizeof(write_char_data),
                                  write_char_data,
                                  ESP_GATT_WRITE_TYPE_RSP,
                                  ESP_GATT_AUTH_REQ_NONE);
        break;
    case ESP_GATTC_SRVC_CHG_EVT: {
        esp_bd_addr_t bda;
        memcpy(bda, p_data->srvc_chg.remote_bda, sizeof(esp_bd_addr_t));
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_SRVC_CHG_EVT, bd_addr:");
        esp_log_buffer_hex(GATTC_TAG, bda, sizeof(esp_bd_addr_t));
        break;
    }

	
/*UPIIIIIIS! */	
    case ESP_GATTC_WRITE_CHAR_EVT:
        if (p_data->write.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "write char failed, error status = %x", p_data->write.status);
            break;
        }
		        if (p_data->write.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "write descr failed, error status = %x", p_data->write.status);
            break;
        }

		//Slanje/Primanje podataka sa nekom pauzom!
		vTaskDelay(PERION_TRANSFERA_PODATAKA / portTICK_PERIOD_MS);
		
		char str[30];
   	    char temp[6] ={};
	    gcvt(PERIOD_TEMP,5,temp);
		  strcpy (str,temp);
    	gcvt(PERIOD_PH,6,temp);
				  strcat (str,"#");
		  strcat (str,temp);
		gcvt(PERIOD_PROTOK,5,temp);
				  strcat (str,"#");
		  strcat (str,temp);
		gcvt(PRAG_PROTOKA,2,temp);
				  strcat (str,"#");
		  strcat (str,temp);
		gcvt(PH_D_P,2,temp);
				  strcat (str,"#");
		  strcat (str,temp);
		gcvt(PH_G_P,2,temp);
				  strcat (str,"#");
		  strcat (str,temp);
		gcvt(TEMP_D_P,2,temp);
				  strcat (str,"#");
		  strcat (str,temp);
		gcvt(TEMP_G_P,2,temp);
				  strcat (str,"#");
		  strcat (str,temp);
		gcvt(VENTIL_ON_OFF,1,temp);
		  		  strcat (str,"#");
		  strcat (str,temp);
		  		  strcat (str,"#");
		  strcat (str,(IZVRSITI_KALIBRACIJU==false?"0":"1"));
		  if(IZVRSITI_KALIBRACIJU)
					IZVRSITI_KALIBRACIJU = false;
		  		  strcat (str,"#");
		 // printf("PORUKA: %s\n",str);

			 uint8_t *podaci = (uint8_t*)str;
			 int velicina = strlen(str); //sizeof(podaci)

		/*Slanje podataka ka SERVERU: */
        esp_ble_gattc_write_char( gattc_if,
                                  gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                                  gl_profile_tab[PROFILE_A_APP_ID].char_handle,
                                  velicina, //sizeof(write_char_data_NOVO)
                                  podaci,//write_char_data_NOVO,
                                  ESP_GATT_WRITE_TYPE_RSP,
                                  ESP_GATT_AUTH_REQ_NONE);								  
        break;
		
		
    case ESP_GATTC_DISCONNECT_EVT:
        connect = false;
        get_server = false;
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_DISCONNECT_EVT, reason = %d", p_data->disconnect.reason);
        break;
    default:
        break;
    }
}

static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    uint8_t *adv_name = NULL;
    uint8_t adv_name_len = 0;
    switch (event) {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: {
        //the unit of the duration is second
        uint32_t duration = 30;
        esp_ble_gap_start_scanning(duration);
        break;
    }
    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
        //scan start complete event to indicate scan start successfully or failed
        if (param->scan_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(GATTC_TAG, "scan start failed, error status = %x", param->scan_start_cmpl.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "scan start success");

        break;
    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
        esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;
        switch (scan_result->scan_rst.search_evt) {
        case ESP_GAP_SEARCH_INQ_RES_EVT:
            esp_log_buffer_hex(GATTC_TAG, scan_result->scan_rst.bda, 6);
            ESP_LOGI(GATTC_TAG, "searched Adv Data Len %d, Scan Response Len %d", scan_result->scan_rst.adv_data_len, scan_result->scan_rst.scan_rsp_len);
            adv_name = esp_ble_resolve_adv_data(scan_result->scan_rst.ble_adv,
                                                ESP_BLE_AD_TYPE_NAME_CMPL, &adv_name_len);
            ESP_LOGI(GATTC_TAG, "searched Device Name Len %d", adv_name_len);
            esp_log_buffer_char(GATTC_TAG, adv_name, adv_name_len);

#if CONFIG_EXAMPLE_DUMP_ADV_DATA_AND_SCAN_RESP
            if (scan_result->scan_rst.adv_data_len > 0) {
                ESP_LOGI(GATTC_TAG, "adv data:");
                esp_log_buffer_hex(GATTC_TAG, &scan_result->scan_rst.ble_adv[0], scan_result->scan_rst.adv_data_len);
            }
            if (scan_result->scan_rst.scan_rsp_len > 0) {
                ESP_LOGI(GATTC_TAG, "scan resp:");
                esp_log_buffer_hex(GATTC_TAG, &scan_result->scan_rst.ble_adv[scan_result->scan_rst.adv_data_len], scan_result->scan_rst.scan_rsp_len);
            }
#endif
            ESP_LOGI(GATTC_TAG, "\n");

            if (adv_name != NULL) {
                if (strlen(remote_device_name) == adv_name_len && strncmp((char *)adv_name, remote_device_name, adv_name_len) == 0) {
                    ESP_LOGI(GATTC_TAG, "searched device %s\n", remote_device_name);
                    if (connect == false) {
                        connect = true;
                        ESP_LOGI(GATTC_TAG, "connect to the remote device.");
                        esp_ble_gap_stop_scanning();
                        esp_ble_gattc_open(gl_profile_tab[PROFILE_A_APP_ID].gattc_if, scan_result->scan_rst.bda, scan_result->scan_rst.ble_addr_type, true);
                    }
                }
            }
            break;
        case ESP_GAP_SEARCH_INQ_CMPL_EVT:
            break;
        default:
            break;
        }
        break;
    }

    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
        if (param->scan_stop_cmpl.status != ESP_BT_STATUS_SUCCESS){
            ESP_LOGE(GATTC_TAG, "scan stop failed, error status = %x", param->scan_stop_cmpl.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "stop scan successfully");
        break;

    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS){
            ESP_LOGE(GATTC_TAG, "adv stop failed, error status = %x", param->adv_stop_cmpl.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "stop adv successfully");
        break;
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
         ESP_LOGI(GATTC_TAG, "update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
                  param->update_conn_params.status,
                  param->update_conn_params.min_int,
                  param->update_conn_params.max_int,
                  param->update_conn_params.conn_int,
                  param->update_conn_params.latency,
                  param->update_conn_params.timeout);
        break;
    default:
        break;
    }
}

static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    /* If event is register event, store the gattc_if for each profile */
    if (event == ESP_GATTC_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            gl_profile_tab[param->reg.app_id].gattc_if = gattc_if;
        } else {
            ESP_LOGI(GATTC_TAG, "reg app failed, app_id %04x, status %d",
                    param->reg.app_id,
                    param->reg.status);
            return;
        }
    }

    /* If the gattc_if equal to profile A, call profile A cb handler,
     * so here call each profile's callback */
    do {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++) {
            if (gattc_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
                    gattc_if == gl_profile_tab[idx].gattc_if) {
                if (gl_profile_tab[idx].gattc_cb) {
                    gl_profile_tab[idx].gattc_cb(event, gattc_if, param);
                }
            }
        }
    } while (0);
}

static void IRAM_ATTR switch_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void switch_task_entry(void* arg)
{
    uint32_t io_num;

    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
			if(io_num == KONFIGURACIONI_PIN){
				ESP_LOGE(GATTC_TAG, "UNOS PARAMETARA:\n");
				int pom = 9999;
				//1.)
				printf("Unesite period mjerenja TEMPERATURE: ");
				scanf("%d",&pom);
				if(pom!=9999)
					PERIOD_TEMP = pom;
				printf("Period mjerenja TEMPERATURE: %d\n",PERIOD_TEMP);			
				
				//2.)
				pom = 9999;
				printf("Unesite period PH: ");
				scanf("%d",&pom);
				if(pom!=9999)
					PERIOD_PH = pom;
				printf("Perion PH: %d\n",PERIOD_PH);			

				//3.)
				pom = 9999;
				printf("Unesite period mjerenja PROTOKA: ");
				scanf("%d",&pom);
				if(pom!=9999)
					PERIOD_PROTOK = pom;
				printf("Period mjerenja PROTOKA: %d\n",PERIOD_PROTOK);

				//4.)
				pom = 9999;
				printf("Unesite PRAG PROTOKA: ");
				scanf("%d",&pom);
				if(pom!=9999)
					PRAG_PROTOKA = pom;
				printf("PRAG PROTOKA: %d\n",PRAG_PROTOKA);	

				//5.)
				pom = 9999;
				printf("Unesite PH DONJI PRAG: ");
				scanf("%d",&pom);
				if(pom!=9999)
					PH_D_P = pom;
				printf("PH DONJI PRAG: %d\n",PH_D_P);	

				//6.)
				pom = 9999;
				printf("Unesite PH GORNJI PRAG: ");
				scanf("%d",&pom);
				if(pom!=9999)
					PH_G_P = pom;
				printf("PH GORNJI PRAG: %d\n",PH_G_P);		

				//7.)
				pom = 9999;
				printf("Unesite TEMP. DONJI PRAG: ");
				scanf("%d",&pom);
				if(pom!=9999)
					TEMP_D_P = pom;
				printf("TEMP. DONJI PRAG: %d\n",TEMP_D_P);			

				//8.)
				pom = 9999;
				printf("Unesite TEMP GORNJI PRAG: ");
				scanf("%d",&pom);
				if(pom!=9999)
					TEMP_G_P = pom;
				printf("TEMP. GORNJI PRAG: %d\n",TEMP_G_P);						

			}else if(io_num == VENTIL_PIN){
				VENTIL_ON_OFF = !VENTIL_ON_OFF;
				printf("VENTIL: %d\n",VENTIL_ON_OFF);
				VENTIL_UKLJUCEN_NA_DUGME = !VENTIL_UKLJUCEN_NA_DUGME;
				
			}else if(io_num == KALIBRACIJA_PIN){
				printf("KALIBRACIJA!\n");
				IZVRSITI_KALIBRACIJU = true;
			}				
        }
		
		vTaskDelay(250 / portTICK_PERIOD_MS);  //Period ponavljanja 0.25s!
    }
}

static void switch_key_init(uint32_t key)
{
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_NEGEDGE;   //opadajuca ivica!
    io_conf.pin_bit_mask = 1 << key;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 1;
    io_conf.pull_down_en = 0;
    gpio_config(&io_conf);

    gpio_set_intr_type(key, GPIO_INTR_NEGEDGE);
    gpio_install_isr_service(INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(key, switch_isr_handler, (void *)key); //Moja promjena!
}

static void switch_init(gpio_num_t gpio_num)
{
	if (!gpio_evt_queue) //Ako ne postoji red, kreira se!
		gpio_evt_queue = xQueueCreate(1, sizeof(uint32_t)); //3
	
    if (!gpio_evt_queue) {
        return;
    }

    BaseType_t ret = xTaskCreate(switch_task_entry, "switch", 4096, NULL, 10, NULL); //Prioritet 4!
    if (ret == pdFAIL) { 
        goto fail;
    }

    switch_key_init(gpio_num);
    return;

fail:
    vQueueDelete(gpio_evt_queue);
}


void app_main()
{
	printf("UNOS PARAMETARA SE OBAVLJA PO PRITISKU NA TASTER! ZA ZADRZAVANJE VRIJEDNOSTI PARAMETRA, UNESITE VRIJEDNOST '9999'");
    // Initialize NVS.
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(GATTC_TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(GATTC_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(GATTC_TAG, "%s init bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(GATTC_TAG, "%s enable bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    //register the  callback function to the gap module
    ret = esp_ble_gap_register_callback(esp_gap_cb);
    if (ret){
        ESP_LOGE(GATTC_TAG, "%s gap register failed, error code = %x\n", __func__, ret);
        return;
    }

    //register the callback function to the gattc module
    ret = esp_ble_gattc_register_callback(esp_gattc_cb);
    if(ret){
        ESP_LOGE(GATTC_TAG, "%s gattc register failed, error code = %x\n", __func__, ret);
        return;
    }

    ret = esp_ble_gattc_app_register(PROFILE_A_APP_ID);
    if (ret){
        ESP_LOGE(GATTC_TAG, "%s gattc app register failed, error code = %x\n", __func__, ret);
    }
    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
    if (local_mtu_ret){
        ESP_LOGE(GATTC_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
    }
	
	
		//Inicijalizacija prekidaca:
	switch_init(KONFIGURACIONI_PIN);
	switch_init(VENTIL_PIN);
	switch_init(KALIBRACIJA_PIN);

}

