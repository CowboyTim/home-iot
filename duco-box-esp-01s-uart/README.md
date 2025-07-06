## To configure the ESP-01, UART to TCP relay:

Check for the ESP-01's uart device in dmesg, and use that.

```
wifi_ssid=<fill in SSID here>
wifi_pass=<fill in PASS here>
metrics_ip=<fill in local IP here, e.g.: 192.168.1.17>
stty -F /dev/ttyUSB1 115200 cs8 -parenb -cstopb -echoe -echok -echoctl \
   -echoke -ixon -ixoff icrnl inlcr ocrnl onlcr -noflsh -opost -isig -icanon -echo
cat /dev/ttyUSB1
cat >> /dev/ttyUSB1 <<EOcfg
AT\r
AT+SYSLOG=1\r
AT+SYSSTORE?\r
AT+SYSSTORE=1\r
AT+CIPMODE=0\r
AT+CWMODE=3\r
AT+CWJAP=\"$wifi_ssid\",\"$wifi_pass\"\r
AT+CWCOUNTRY=0,\"BE\",1,13\r
AT+CWSTATE?\r
AT+UART_DEF?\r
AT+CWHOSTNAME=\"metrics-duco\"\r
AT+CIPDNS=1,\"8.8.8.8\"\r
AT+PING=\"$metrics_ip\"\r
AT+CIPMUX=1\r
AT+CIPSNTPCFG=1,2,\"0.be.pool.ntp.org\",\"1.be.pool.ntp.org\"\r
AT+CIPSNTPTIME?\r
AT+SAVETRANSLINK=1,\"$metrics_ip\",2235,\"TCP\"\r
EOcfg
```

To ever exit this uart->tcp link, use this:
```
echo -ne "+++" >> /dev/ttyUSB1
```

## On PC:
1. download DUCO Network Tool
```
curl -sSQvLk 'https://data.duco.eu/DucoNetworkTool/Release/Setup%20Duco%20Network%20Tool.exe' \
  -o duco_network_tool.exe
```

2. install Wine and winetricks to get .NET working
```
sudo apt install wine winetricks
wine wineboot
```

The DUCO Network Tool needs at least Windows 7, start up winecfg and configure
the OS in "Applications" -> "Windows Version":
```
wine winecfg
```

Manually babysit wine through .NET installs, high enough for the DUCO Network
Tool so it doesn't want to install .NET (that hangs):
```
winetricks dotnet40_kb2468871
winetricks --force dotnet45
winetricks --force dotnet452
winetricks dotnet46
```

3. install DUCO Network Tool
```
wine duco_network_tool.exe
```

## On PC/Host receiving the incoming ESP-01 uart/tcp connection, for debian/mint/raspbian:

1. allow incoming tcp connections:
```
sudo ufw allow from 192.168.1.0/24 to any port 2235 proto tcp
```

2. run socat to accept and create a virtualcom0 device
```
sudo socat pty,link=/dev/virtualcom0,raw TCP-LISTEN:2235,reuseaddr
```

3. run a terminal program
```
sudo picocom /dev/virtualcom0 -b 115200 --imap crcrlf
```

