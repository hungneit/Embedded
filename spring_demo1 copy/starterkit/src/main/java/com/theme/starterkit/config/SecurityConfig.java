package com.theme.starterkit.config;

import org.springframework.context.annotation.Configuration;
import org.springframework.security.config.annotation.web.builders.HttpSecurity;
import org.springframework.security.config.annotation.web.configuration.EnableWebSecurity;
import org.springframework.security.config.annotation.web.configuration.WebSecurityConfigurerAdapter;

@Configuration
@EnableWebSecurity
public class SecurityConfig extends WebSecurityConfigurerAdapter {

    @Override
    protected void configure(HttpSecurity http) throws Exception {
        http
                .csrf().disable() // Disable CSRF
                .cors().disable() // Disable CORS
                .authorizeRequests()
                .anyRequest().permitAll() // Allow all requests
                .and()
                .headers().frameOptions().sameOrigin(); // Allow H2 console if needed
    }
}