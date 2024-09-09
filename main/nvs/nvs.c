#include "nvs.h"

/**
 * @brief Inicializa la partición de NVS (Non-Volatile Storage).
 *
 * Esta función inicializa la partición de NVS y, si es necesario,
 * borra y reinicializa la partición si no hay páginas libres o se ha encontrado una nueva versión de NVS.
 *
 * @return esp_err_t
 *         - ESP_OK: NVS fue inicializado correctamente.
 *         - ESP_ERR_NVS_NO_FREE_PAGES: No hay páginas libres en la partición NVS.
 *         - ESP_ERR_NVS_NEW_VERSION_FOUND: Se encontró una nueva versión de NVS.
 *         - Otros códigos de error relacionados con NVS.
 */

esp_err_t init_nvs()
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    return ret;
}

/**
 * @brief Guarda un valor entero en la memoria NVS.
 *
 * Esta función guarda un valor de tipo `uint32_t` en la NVS bajo un espacio de nombres (`namespace`)
 * y una clave (`key`). El valor es guardado en la memoria flash para su persistencia.
 *
 * @param namespace Espacio de nombres donde se almacenará el valor.
 * @param key Clave bajo la cual se almacenará el valor.
 * @param value El valor entero que se guardará.
 *
 * @return esp_err_t
 *         - ESP_OK: El valor fue guardado correctamente.
 *         - Otros códigos de error si hubo un problema al guardar o hacer commit en NVS.
 */

esp_err_t save_value_to_nvs(const char *namespace, const char *key, uint32_t value)
{
    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open(namespace, NVS_READWRITE, &nvs_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG_NVS, "Error al abrir NVS: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = nvs_set_u32(nvs_handle, key, value);
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG_NVS, "Valor guardado: %ld", value);
        ret = nvs_commit(nvs_handle); // Hacer commit
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG_NVS, "Error al hacer commit en NVS: %s", esp_err_to_name(ret));
        }
    }
    else
    {
        ESP_LOGE(TAG_NVS, "Error al guardar el valor: %s", esp_err_to_name(ret));
    }

    nvs_close(nvs_handle); // Cerrar NVS
    return ret;
}

/**
 * @brief Recupera un valor entero desde la memoria NVS.
 *
 * Esta función recupera un valor de tipo `uint32_t` desde la NVS utilizando un espacio de nombres (`namespace`)
 * y una clave (`key`). Si el valor no existe en la NVS, se establece un valor por defecto de 0.
 *
 * @param namespace Espacio de nombres desde donde se recuperará el valor.
 * @param key Clave bajo la cual se almacenó el valor.
 * @param value Puntero a una variable donde se almacenará el valor recuperado.
 *
 * @return esp_err_t
 *         - ESP_OK: El valor fue leído correctamente.
 *         - ESP_ERR_NVS_NOT_FOUND: El valor no se encontró en NVS.
 *         - Otros códigos de error si hubo un problema al leer desde NVS.
 */
esp_err_t get_value_from_nvs(const char *namespace, const char *key, uint32_t *value)
{
    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open(namespace, NVS_READWRITE, &nvs_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG_NVS, "Error al abrir NVS: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = nvs_get_u32(nvs_handle, key, value);
    switch (ret)
    {
    case ESP_OK:
        ESP_LOGI(TAG_NVS, "Valor leído: %ld", *value);
        break;
    case ESP_ERR_NVS_NOT_FOUND:
        ESP_LOGI(TAG_NVS, "Valor no encontrado");
        *value = 0; // Valor por defecto
        break;
    default:
        ESP_LOGE(TAG_NVS, "Error al leer el valor: %s", esp_err_to_name(ret));
        break;
    }

    nvs_close(nvs_handle); // Cerrar NVS
    return ret;
}

/**
 * @brief Guarda un valor entero en NVS si no existe previamente.
 *
 * @param namespace Espacio de nombres donde se almacenará el valor.
 * @param key Clave bajo la cual se almacenará el valor.
 * @param value El valor entero que se guardará.
 *
 * @return esp_err_t ESP_OK si se guarda el valor o si ya existe, o un código de error en caso contrario.
 */
esp_err_t save_value_if_not_exists(const char *namespace, const char *key, uint32_t value)
{
    uint32_t existing_value;
    esp_err_t ret = get_value_from_nvs(namespace, key, &existing_value);

    if (ret == ESP_ERR_NVS_NOT_FOUND)
    {
        ESP_LOGI(TAG_NVS, "Valor no encontrado, guardando valor de calibración: %ld", value);
        ret = save_value_to_nvs(namespace, key, value);
        if (ret == ESP_OK)
        {
            ESP_LOGI(TAG_NVS, "Valor de calibración guardado correctamente");
        }
        else
        {
            ESP_LOGE(TAG_NVS, "Error al guardar el valor de calibración: %s", esp_err_to_name(ret));
        }
    }
    else if (ret == ESP_OK)
    {
        ESP_LOGI(TAG_NVS, "Valor ya existe, no se realiza la calibración");
    }
    else
    {
        ESP_LOGE(TAG_NVS, "Error al verificar si existe valor de calibración: %s", esp_err_to_name(ret));
    }

    return ret;
}