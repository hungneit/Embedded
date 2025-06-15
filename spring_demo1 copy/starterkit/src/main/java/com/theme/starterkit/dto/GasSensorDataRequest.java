package com.theme.starterkit.dto;

import com.fasterxml.jackson.annotation.JsonProperty;
import lombok.Data;

import javax.validation.constraints.NotBlank;

@Data
public class GasSensorDataRequest {

    @NotBlank
    @JsonProperty("device_id")
    private String deviceId;

    @JsonProperty("timestamp")
    private Long timestamp;

    @JsonProperty("gas_value")
    private Integer gasValue;

    @JsonProperty("gas_status")
    private String gasStatus;

    @JsonProperty("system_status")
    private String systemStatus;

    @JsonProperty("relay_status")
    private String relayStatus;

    @JsonProperty("wifi_rssi")
    private Integer wifiRssi;

    @JsonProperty("uptime")
    private Long uptime;

    @JsonProperty("stm32_connected")
    private Boolean stm32Connected;

    @JsonProperty("protocol")
    private String protocol;

    @JsonProperty("urgent")
    private Boolean urgent;

    @JsonProperty("alert_level")
    private String alertLevel;

    @JsonProperty("binary_errors")
    private Integer binaryErrors;

    @JsonProperty("packet_success_rate")
    private Double successRate;

    @JsonProperty("immediate_send")
    private Boolean immediateSend;
}