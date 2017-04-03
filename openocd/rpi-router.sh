#!/bin/sh
# routersh: turn your computer into another device's router
# Copyright (C) 2017 Aleksa Sarai <cyphar@cyphar.com>
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

set -ex

INTERNAL="${INTERNAL:-eth0}"
EXTERNAL="${EXTERNAL:-wlan0}"

[ -z "$1" ] || INTERNAL="$1"
[ -z "$2" ] || EXTERNAL="$2"

# First, bring the interface up and set its IP.
ip link set "$INTERNAL" up
ip addr add 10.13.37.1/24 dev "$INTERNAL" || :

# Enable port forwarding.
sysctl -w net.ipv4.ip_forward=1

# Set up NAT and MASQUERADE.
iptables -t nat -A POSTROUTING -o "$EXTERNAL" -j MASQUERADE
iptables -A FORWARD -m conntrack --ctstate RELATED,ESTABLISHED -j ACCEPT
iptables -A FORWARD -i "$INTERNAL" -o "$EXTERNAL" -j ACCEPT

# Run dnsmasq to set up dhcp.
DNSMASQDIR="$(mktemp -d --tmpdir routersh-dnsmasq.XXXXXX)"
PIDFILE="$DNSMASQDIR/pidfile"
LEASES="$DNSMASQDIR/leases"

chown dhcpd "$DNSMASQDIR"

# Run dnsmasq.
echo "LEASES: $LEASES"
dnsmasq --log-async --enable-dbus --keep-in-foreground \
		--interface="$INTERNAL" --expand-hosts --domain="hacks.local" \
		--dhcp-range="$INTERNAL,10.13.37.2,10.13.37.255,255.255.255.0,1h" \
		--pid-file="$PIDFILE" --dhcp-leasefile="$LEASES" --user=dhcpd

rm -f "$PIDFILE" "$LEASES"
