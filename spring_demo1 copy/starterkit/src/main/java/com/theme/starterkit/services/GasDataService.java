package com.theme.starterkit.services;

import com.theme.starterkit.entities.GasData;
import com.theme.starterkit.repositories.GasDataRepository;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.data.domain.Page;
import org.springframework.data.domain.PageRequest;
import org.springframework.data.domain.Pageable;
import org.springframework.data.domain.Sort;
import org.springframework.messaging.simp.SimpMessagingTemplate;
import org.springframework.scheduling.annotation.Scheduled;
import org.springframework.stereotype.Service;

import javax.transaction.Transactional;
import java.time.Instant;
import java.time.LocalDateTime;
import java.time.ZoneId;
import java.util.List;
import java.util.Map;

@Service
public class GasDataService {

    @Autowired
    private GasDataRepository repository;

    @Autowired
    private SimpMessagingTemplate messagingTemplate;

    public GasData saveData(Map<String, Object> requestData) {
        GasData data = new GasData();
        data.setDeviceId((String) requestData.get("device_id"));
        Object timestamp = requestData.get("timestamp");
        if (timestamp instanceof Number) {
            data.setTimestamp(LocalDateTime.ofInstant(
                    Instant.ofEpochMilli(((Number) timestamp).longValue()),
                    ZoneId.systemDefault()));
        } else {
            data.setTimestamp(LocalDateTime.now());
        }

        Object gasVal = requestData.get("gas_value");
        if (gasVal instanceof Number) data.setGasValue(((Number) gasVal).intValue());

        data.setGasStatus((String) requestData.get("gas_status"));
        data.setSystemStatus((String) requestData.get("system_status"));
        data.setRelayStatus((String) requestData.get("relay_status"));

        Object rssi = requestData.get("wifi_rssi");
        if (rssi instanceof Number) data.setWifiRssi(((Number) rssi).intValue());

        Object uptime = requestData.get("uptime");
        if (uptime instanceof Number) data.setUptime(((Number) uptime).longValue());

        Object stm32 = requestData.get("stm32_connected");
        if (stm32 instanceof Boolean) {
            data.setStm32Connected((Boolean) stm32);
        } else if (stm32 instanceof Number) {
            data.setStm32Connected(((Number) stm32).intValue() != 0);
        }

        data.setProtocol((String) requestData.get("protocol"));

        Object urgent = requestData.get("urgent");
        if (urgent instanceof Boolean) {
            data.setUrgent((Boolean) urgent);
        } else if (urgent instanceof Number) {
            data.setUrgent(((Number) urgent).intValue() != 0);
        }

        data.setAlertLevel((String) requestData.get("alert_level"));

        GasData saved = repository.save(data);

        messagingTemplate.convertAndSend("/topic/gas-data", saved);

        return saved;
    }

    public Page<GasData> getAllData(int page, int size, String deviceFilter, String statusFilter) {
        Pageable pageable = PageRequest.of(page, size, Sort.by("timestamp").descending());

        if (deviceFilter != null && statusFilter != null) {
            return repository.findByDeviceIdContainingIgnoreCaseAndGasStatus(deviceFilter, statusFilter, pageable);
        } else if (deviceFilter != null) {
            return repository.findByDeviceIdContainingIgnoreCase(deviceFilter, pageable);
        } else if (statusFilter != null) {
            return repository.findByGasStatus(statusFilter, pageable);
        } else {
            return repository.findAll(pageable);
        }
    }

    public List<GasData> getLatestData() {
        return repository.findLatestByDevice();
    }

    public List<GasData> getRecentAlerts() {
        LocalDateTime since = LocalDateTime.now().minusHours(24);
        return repository.findByUrgentTrueAndTimestampAfterOrderByTimestampDesc(since);
    }

    public Map<String, Long> getStatusCounts() {
        return Map.of(
                "SAFE", repository.countByStatus("SAFE"),
                "WARNING", repository.countByStatus("WARNING"),
                "DANGER", repository.countByStatus("DANGER"),
                "CRITICAL", repository.countByStatus("CRITICAL")
        );
    }

    // Delete data older than 24 hours
    @Transactional
    public void deleteOldData() {
        LocalDateTime cutoff = LocalDateTime.now().minusHours(24);
        repository.deleteByTimestampBefore(cutoff);
    }

    @Transactional
    @Scheduled(fixedRate = 3600000) // Run every hour
    public void scheduleOldDataDeletion() {
        deleteOldData();
    }
}