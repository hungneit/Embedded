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

    Page<GasData> findAll(Pageable pageable);

    Page<GasData> findByDeviceIdContainingIgnoreCase(String deviceId, Pageable pageable);

    Page<GasData> findByGasStatus(String gasStatus, Pageable pageable);

    Page<GasData> findByDeviceIdContainingIgnoreCaseAndGasStatus(String deviceId, String gasStatus, Pageable pageable);

    List<GasData> findTop50ByOrderByTimestampDesc();

    @Query("SELECT g FROM GasData g WHERE g.timestamp = " +
            "(SELECT MAX(g2.timestamp) FROM GasData g2 WHERE g2.deviceId = g.deviceId)")
    List<GasData> findLatestByDevice();

    @Query("SELECT COUNT(g) FROM GasData g WHERE g.gasStatus = :status")
    Long countByStatus(@Param("status") String status);

    List<GasData> findByUrgentTrueAndTimestampAfterOrderByTimestampDesc(LocalDateTime since);

    // New method to delete old data
    void deleteByCreatedAtBefore(LocalDateTime cutoffTime);

    Optional<GasData> findTopByDeviceIdOrderByTimestampDesc(String deviceId);

}