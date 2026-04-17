from pathlib import Path

Import("env")


def patch_wifi_generic() -> None:
    framework_dir = env.PioPlatform().get_package_dir("framework-arduinoespressif32")
    if not framework_dir:
        print("Hosted WiFi patch skipped: framework-arduinoespressif32 package not found yet")
        return

    wifi_generic = Path(framework_dir) / "libraries" / "WiFi" / "src" / "WiFiGeneric.cpp"
    original = wifi_generic.read_text(encoding="utf-8")

    patched_block = """      initWiFiEvents();
#if CONFIG_ESP_WIFI_REMOTE_ENABLED
      if (esp_netifs[ESP_IF_WIFI_AP] == NULL) {
        esp_netifs[ESP_IF_WIFI_AP] = esp_netif_get_handle_from_ifkey(\"WIFI_AP_DEF\");
      }
      if (esp_netifs[ESP_IF_WIFI_STA] == NULL) {
        esp_netifs[ESP_IF_WIFI_STA] = esp_netif_get_handle_from_ifkey(\"WIFI_STA_DEF\");
      }
#endif
      if (esp_netifs[ESP_IF_WIFI_AP] == NULL) {
        esp_netifs[ESP_IF_WIFI_AP] = esp_netif_create_default_wifi_ap();
      }
      if (esp_netifs[ESP_IF_WIFI_STA] == NULL) {
        esp_netifs[ESP_IF_WIFI_STA] = esp_netif_create_default_wifi_sta();
      }
"""

    if patched_block in original:
        print("Hosted WiFi netif reuse patch already applied")
        return

    original_block = """      initWiFiEvents();
      if (esp_netifs[ESP_IF_WIFI_AP] == NULL) {
        esp_netifs[ESP_IF_WIFI_AP] = esp_netif_create_default_wifi_ap();
      }
      if (esp_netifs[ESP_IF_WIFI_STA] == NULL) {
        esp_netifs[ESP_IF_WIFI_STA] = esp_netif_create_default_wifi_sta();
      }
"""

    if original_block not in original:
        raise RuntimeError("Unexpected WiFiGeneric.cpp contents; hosted WiFi patch not applied")

    wifi_generic.write_text(original.replace(original_block, patched_block, 1), encoding="utf-8")
    print(f"Applied hosted WiFi netif reuse patch to {wifi_generic}")


patch_wifi_generic()