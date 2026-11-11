#!/bin/bash

AR1021X_MAC="00:03:7f:20:44:d2"
IP_ADDR="192.168.8.1/24"
CONF="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/hostapd.conf"

IFACE=""
for addr_file in /sys/class/net/*/address; do
	if [ "$(cat "$addr_file")" = "$AR1021X_MAC" ]; then
		IFACE="$(basename "$(dirname "$addr_file")")"
		break
	fi
done
if [ -z "$IFACE" ]; then
	echo "error: AR1021x interface with MAC $AR1021X_MAC not found" >&2
	exit 1
fi
echo "using interface $IFACE"

for usb_dev in /sys/bus/usb/devices/*; do
	[ "$(cat "$usb_dev/idVendor" 2>/dev/null)" = "0cf3" ] || continue
	echo on > "$usb_dev/power/control" 2>/dev/null || true
	echo "usb autosuspend disabled for $(basename "$usb_dev")"
done

if command -v airmon-ng >/dev/null 2>&1; then
	airmon-ng check kill
elif command -v nmcli >/dev/null 2>&1; then
	nmcli dev set "$IFACE" managed no 2>/dev/null || true
fi
pkill -f "wpa_supplicant.*$IFACE" 2>/dev/null || true
pkill -f "hostapd.*$CONF" 2>/dev/null || true
sed -i "s/^interface=.*/interface=$IFACE/" "$CONF"
ip addr flush dev "$IFACE"
ip addr add "$IP_ADDR" dev "$IFACE"
ip link set "$IFACE" up
hostapd -B "$CONF"
