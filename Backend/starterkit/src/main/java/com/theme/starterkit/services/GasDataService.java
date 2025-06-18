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
import org.springframework.security.core.userdetails.UsernameNotFoundException;
import org.springframework.stereotype.Service;

import javax.transaction.Transactional;
import java.time.Duration;
import java.time.Instant;
import java.time.LocalDateTime;
import java.time.ZoneId;
import java.util.List;
import java.util.Map;
import java.util.UUID;

@Service
public class GasDataService {

    @Autowired
    private GasDataRepository repository;

    @Autowired
    private SimpMessagingTemplate messagingTemplate;

    @Autowired
    private EmailService emailService;

    private static final List<String> SEVERITY_ORDER = List.of("SAFE", "WARNING", "DANGER", "CRITICAL");

    private boolean isSeverityIncreased(String oldStatus, String newStatus) {
        if (oldStatus == null || newStatus == null) return false;
        int oldIndex = SEVERITY_ORDER.indexOf(oldStatus.toUpperCase());
        int newIndex = SEVERITY_ORDER.indexOf(newStatus.toUpperCase());
        return oldIndex >= 0 && newIndex > oldIndex;
    }


    public GasData saveData(Map<String, Object> requestData) {
        String deviceId = (String) requestData.get("device_id");
        String newGasStatus = (String) requestData.get("gas_status");

        // Lấy bản ghi mới nhất từ DB theo deviceId
        GasData latest = repository.findTopByDeviceIdOrderByTimestampDesc(deviceId).orElse(null);
        String oldGasStatus = latest != null ? latest.getGasStatus() : null;

        // Tạo và điền dữ liệu
        GasData data = new GasData();
        data.setDeviceId(deviceId);

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

        data.setGasStatus(newGasStatus);
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

        // Lưu dữ liệu
        GasData saved = repository.save(data);

        // Gửi realtime qua websocket
        messagingTemplate.convertAndSend("/topic/gas-data", saved);

        // So sánh nếu trạng thái nguy hiểm hơn thì gửi mail
        if (isSeverityIncreased(oldGasStatus, newGasStatus)) {
            sendAlertEmail(deviceId, newGasStatus);
        }

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
        repository.deleteByCreatedAtBefore(cutoff);
    }

    @Transactional
    @Scheduled(fixedRate = 3600000) // Run every hour
    public void scheduleOldDataDeletion() {
        deleteOldData();
    }


    public void sendAlertEmail(String deviceId, String status) {
        String subject = "[BingPay] Cảnh báo khí gas - Thiết bị " + deviceId;
        String content =
                "<div style='font-family: Arial, sans-serif; font-size: 16px; color: #333; line-height: 1.6; max-width: 600px; margin: auto; padding: 20px;'>" +
                        "  <p><strong style='color: red;'>CẢNH BÁO:</strong> Trạng thái khí gas đã thay đổi trên thiết bị <strong>" + deviceId + "</strong>.</p>" +
                        "  <p>Trạng thái hiện tại: <strong style='color: #dc3545;'>" + status + "</strong></p>" +
                        "  <p>Vui lòng kiểm tra ngay hệ thống của bạn để đảm bảo an toàn.</p>" +
                        "  <hr style='border: none; border-top: 1px solid #eee; margin: 30px 0;'>" +
                        "  <p style='font-size: 14px; color: #777;'>Email được gửi tự động từ hệ thống BingPay.</p>" +
                        "  <p style='margin-top: 40px;'>Trân trọng,<br><strong>Đội ngũ BingPay</strong></p>" +
                        "</div>";

        // Đổi email nhận cảnh báo tại đây:
        emailService.sendEmail("hungneit@gmail.com", subject, content);
    }

}