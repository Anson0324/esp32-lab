# Weather Retrieval & Reporting System

An embedded system built on the ESP32-C3 that retrieves weather data via HTTPS from wttr.in and posts both remote and local sensor data to a custom mobile-hosted Python web server.

## Features
- HTTPS GET request to weather API (`wttr.in`)
- Reads onboard temperature and humidity
- HTTP POST to display data on a local server

## Tech Stack
- ESP32-C3
- C / FreeRTOS
- Wi-Fi, HTTPS, HTTP
- Python (for web server)
