package com.theme.starterkit.controllers;

import com.theme.starterkit.entities.GasData;
import com.theme.starterkit.services.GasDataService;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.data.domain.Page;
import org.springframework.stereotype.Controller;
import org.springframework.ui.Model;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.RequestParam;

@Controller
public class WebController {

    @Autowired
    private GasDataService gasDataService;

    @GetMapping("/")
    public String index(
            @RequestParam(defaultValue = "0") int page,
            @RequestParam(defaultValue = "20") int size,
            @RequestParam(required = false) String device,
            @RequestParam(required = false) String status,
            Model model) {

        // Get data with filters
        Page<GasData> gasData = gasDataService.getAllData(page, size, device, status);

        // Add to model
        model.addAttribute("gasData", gasData);
        model.addAttribute("latestData", gasDataService.getLatestData());
        model.addAttribute("recentAlerts", gasDataService.getRecentAlerts());
        model.addAttribute("statusCounts", gasDataService.getStatusCounts());
        model.addAttribute("currentPage", page);
        model.addAttribute("totalPages", gasData.getTotalPages());
        model.addAttribute("deviceFilter", device);
        model.addAttribute("statusFilter", status);

        return "pages/home";
    }
}