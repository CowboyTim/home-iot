#!/bin/bash
exec 5<>/dev/tcp/ifconfig.me/80
echo -ne "GET /ip HTTP/1.1\r\nHost: ifconfig.me\r\n\r\n" >&5
IFS="\n"
L=0
while :; do
    read LINE <&5
    LINE=${LINE//$'\r'/}
    LINE=${LINE%$'\n'}
    if [[ "$LINE" =~ Content-Length:\ ([0-9]+) ]]; then
        L="${BASH_REMATCH[1]}"
    fi
    [[ -z "$LINE" ]] && break
done
if [ "$L" -eq 0 ]; then
    exit 0
fi
B=""
while :; do
    read -r -u 5 -N 1 C
    [[ -z "$C" ]] && break
    B+="$C"
    S=$((S+1))
    [[ $S -ge $L ]] && break
done
echo "$B"
