#!/bin/bash

function usage(){
    echo "Usage: $BASH_SOURCE <uart_device> <ssid> <password> <remote_host> [remote_port]" >/dev/stderr
    echo "Example: $BASH_SOURCE /dev/ttyUSB0 MySSID MyPassword 2001:db8::1 5665" >/dev/stderr
    echo "$*"
    exit 1
}
UART=${1:-${UART?$(usage "Error: UART device is not specified.")}}
SSID=${2:-${SSID?$(usage "Error: SSID is not specified.")}}
PASS=${3:-${PASS?$(usage "Error: Password is not specified.")}}
R_HOST=${4:-${R_HOST?$(usage "Error: Remote host is not specified.")}}
R_PORT=${5:-5775}
METRICS_KEY=${6:-${METRICS_KEY?$(usage "Error: Metrics key is not specified.")}}
do_uart(){
    echo -n "Command: $*: "
    (sleep 1; echo -ne "$*\r\n" >> $UART) &
    IFS=$'\n'
    while read -r l; do
        l=${l//[[:cntrl:]]/}
        if [[ -z "$l" ]]; then
            continue
        fi
        if [[ "$l" =~ OK ]] || [[ "$l" =~ ERROR ]]; then
            # let's color ok green and error red when printing, highlight the command
            if [[ "$l" =~ OK ]]; then
                echo -e "\e[32m$l\e[0m"
            else
                echo -e "\e[31m$l\e[0m"
            fi
            break
        else
            if [[ "$l" =~ "AT\+" ]]; then
                # highlight the command in yellow
                echo -e "\e[33m$l\e[0m"
            else
                # print the response normally
                echo -e "\e[33m$l\e[0m"
            fi
        fi
    done < $UART
}
do_uart "AT+WIFI_SSID=$SSID"
do_uart "AT+WIFI_PASS=$PASS"
do_uart "AT+KVMKEY=$METRICS_KEY"
do_uart "AT+UDP_PORT=$R_PORT"
do_uart "AT+UDP_HOST_IP=$R_HOST"
do_uart "AT+PRESSURE_LOG_INTERVAL=5000"
do_uart "AT+TEMPERATURE_LOG_INTERVAL=5000"
do_uart "AT+AIR_QUALITY_LOG_INTERVAL=5000"
do_uart "AT+LDR_ILLUMINANCE_LOG_INTERVAL=5000"
do_uart "AT+HUMIDITY_LOG_INTERVAL=5000"
do_uart "AT+APDS_ILLUMINANCE_LOG_INTERVAL=5000"
do_uart "AT+APDS_COLOR_LOG_INTERVAL=5000"
do_uart "AT+VERBOSE=0"
do_uart "AT+LOG_UART=0"
