package com.theme.starterkit.controllers;

import com.theme.starterkit.entities.GasData;
import com.theme.starterkit.services.GasDataService;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.data.domain.Page;
import org.springframework.http.MediaType;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;

import javax.validation.Valid;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

@RestController
@RequestMapping("/api")
@CrossOrigin(origins = "*")
public class ApiController {

    @Autowired
    private GasDataService gasDataService;

    // ESP8266 endpoint
    @PostMapping(value = "/gas-sensor", produces = MediaType.APPLICATION_JSON_VALUE)
    public ResponseEntity<Map<String, Object>> receiveData(@RequestBody Map<String, Object> data) {
        try {
            System.out.println("📥 Received raw JSON data:");
            data.forEach((k, v) -> System.out.println("  " + k + ": " + v));

            GasData saved = gasDataService.saveData(data);

            Map<String, Object> response = Map.of(
                    "status", "success",
                    "message", "Data received",
                    "id", saved.getId(),
                    "timestamp", System.currentTimeMillis()
            );

            System.out.println("✅ Data saved with ID: " + saved.getId());
            return ResponseEntity.ok(response);
        } catch (Exception e) {
            System.out.println("❌ Error while processing data:");
            e.printStackTrace(); // In toàn bộ stack trace

            Map<String, Object> error = Map.of(
                    "status", "error",
                    "message", e.getMessage(),
                    "timestamp", System.currentTimeMillis()
            );

            return ResponseEntity.badRequest().body(error);
        }
    }


    // Get data with filters
    @GetMapping("/data")
    public ResponseEntity<Page<GasData>> getData(
            @RequestParam(defaultValue = "0") int page,
            @RequestParam(defaultValue = "20") int size,
            @RequestParam(required = false) String device,
            @RequestParam(required = false) String status) {

        Page<GasData> data = gasDataService.getAllData(page, size, device, status);
        return ResponseEntity.ok(data);
    }

    // Get latest data for realtime
    @GetMapping("/latest")
    public ResponseEntity<List<GasData>> getLatest() {
        List<GasData> data = gasDataService.getLatestData();
        return ResponseEntity.ok(data);
    }

    // Get recent alerts
    @GetMapping("/alerts")
    public ResponseEntity<List<GasData>> getAlerts() {
        List<GasData> alerts = gasDataService.getRecentAlerts();
        return ResponseEntity.ok(alerts);
    }

    // Get status counts
    @GetMapping("/stats")
    public ResponseEntity<Map<String, Long>> getStats() {
        Map<String, Long> stats = gasDataService.getStatusCounts();
        return ResponseEntity.ok(stats);
    }
}

