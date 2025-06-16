#!/bin/bash

function s(){
    url="$1"
    port="${2:-80}"
    exec 5<>/dev/tcp/"$url"/"$port"
    cat >&5 &
    cat <&5
}
