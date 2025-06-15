package com.theme.starterkit;

import org.springframework.boot.SpringApplication;
import org.springframework.boot.autoconfigure.SpringBootApplication;
import org.springframework.scheduling.annotation.EnableAsync;
import org.springframework.scheduling.annotation.EnableScheduling;

import java.io.File;
import java.io.IOException;

@SpringBootApplication
@EnableScheduling
@EnableAsync
public class StarterkitApplication {
	public static void main(String[] args) throws IOException {
		SpringApplication.run(StarterkitApplication.class, args);
	}

}
