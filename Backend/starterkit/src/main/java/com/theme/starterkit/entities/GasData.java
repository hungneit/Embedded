package com.theme.starterkit.entities;


import lombok.Data;

import javax.persistence.*;
import java.time.LocalDateTime;

@Entity
@Data
@Table(name = "gas_data")
public class GasData {

    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    private Long id;

    @Column(name = "device_id")
    private String deviceId;

    @Column(name = "timestamp")
    private LocalDateTime timestamp;

    @Column(name = "gas_value")
    private Integer gasValue;

    @Column(name = "gas_status")
    private String gasStatus;

    @Column(name = "system_status")
    private String systemStatus;

    @Column(name = "relay_status")
    private String relayStatus;

    @Column(name = "wifi_rssi")
    private Integer wifiRssi;

    @Column(name = "uptime")
    private Long uptime;

    @Column(name = "stm32_connected")
    private Boolean stm32Connected;

    @Column(name = "protocol")
    private String protocol;

    @Column(name = "urgent")
    private Boolean urgent;

    @Column(name = "alert_level")
    private String alertLevel;

    @Column(name = "created_at")
    private LocalDateTime createdAt;

    // Constructor
    public GasData() {
        this.createdAt = LocalDateTime.now();
    }


    public String getStatusColor() {
        if (!Boolean.TRUE.equals(stm32Connected)) return "gray";
        switch (gasStatus != null ? gasStatus : "UNKNOWN") {
            case "SAFE": return "green";
            case "WARNING": return "yellow";
            case "DANGER": return "orange";
            case "CRITICAL": return "red";
            default: return "gray";
        }
    }

    public boolean isOnline() {
        return Boolean.TRUE.equals(stm32Connected);
    }
}