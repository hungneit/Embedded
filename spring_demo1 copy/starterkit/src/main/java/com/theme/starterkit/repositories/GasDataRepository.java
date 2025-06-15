package com.theme.starterkit.repositories;

import com.theme.starterkit.entities.GasData;
import org.springframework.data.domain.Page;
import org.springframework.data.domain.Pageable;
import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.data.jpa.repository.Query;
import org.springframework.data.repository.query.Param;
import org.springframework.stereotype.Repository;

import java.time.LocalDateTime;
import java.util.List;
import java.util.Optional;

@Repository
public interface GasDataRepository extends JpaRepository<GasData, Long> {

    // All data with pagination and filtering
    Page<GasData> findAll(Pageable pageable);

    // Filter by device
    Page<GasData> findByDeviceIdContainingIgnoreCase(String deviceId, Pageable pageable);

    // Filter by status
    Page<GasData> findByGasStatus(String gasStatus, Pageable pageable);

    // Filter by device and status
    Page<GasData> findByDeviceIdContainingIgnoreCaseAndGasStatus(String deviceId, String gasStatus, Pageable pageable);

    // Recent data for realtime
    List<GasData> findTop50ByOrderByTimestampDesc();

    // Latest data per device
    @Query("SELECT g FROM GasData g WHERE g.timestamp = " +
            "(SELECT MAX(g2.timestamp) FROM GasData g2 WHERE g2.deviceId = g.deviceId)")
    List<GasData> findLatestByDevice();

    // Count by status
    @Query("SELECT COUNT(g) FROM GasData g WHERE g.gasStatus = :status")
    Long countByStatus(@Param("status") String status);

    // Recent alerts
    List<GasData> findByUrgentTrueAndTimestampAfterOrderByTimestampDesc(LocalDateTime since);
}