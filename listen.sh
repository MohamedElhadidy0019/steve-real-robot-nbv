#!/bin/bash
# Passively sniff enp3s0 to discover the robot's REAL address/subnet, and check for
# a DHCP server on that port. Container is privileged + --net=host so this works.
OUT=/home/ws/diag_out; mkdir -p "$OUT"; F="$OUT/listen.txt"; : > "$F"
IFACE=enp3s0

sudo apt-get install -y -q tcpdump nmap iproute2 iputils-ping >/dev/null 2>&1

{
echo "===== $IFACE link ====="
cat /sys/class/net/$IFACE/carrier /sys/class/net/$IFACE/operstate 2>/dev/null

echo
echo "===== passive capture: 25s of whatever is on $IFACE (no packets sent) ====="
echo "(ARP / DHCP / mDNS / DDS / anything -- source IPs here = the robot's real net)"
sudo timeout 25 tcpdump -i $IFACE -nn -e -l 2>/dev/null | head -120

echo
echo "===== unique src IPs / MACs seen ====="
sudo timeout 15 tcpdump -i $IFACE -nn 2>/dev/null | \
  grep -oE '([0-9]{1,3}\.){3}[0-9]{1,3}' | sort -u | head -40

echo
echo "===== DHCP server present on $IFACE? ====="
sudo nmap -e $IFACE --script broadcast-dhcp-discover 2>&1 | grep -A15 "broadcast-dhcp-discover" || echo "(no DHCP offer)"

echo
echo "===== LLDP / CDP (switch identity) ====="
sudo timeout 20 tcpdump -i $IFACE -nn -v -s 1500 'ether proto 0x88cc or ether[20:2]=0x2000' 2>/dev/null | head -30 || echo "(none)"

echo
echo "===== full ARP after listening ====="
sudo timeout 10 tcpdump -i $IFACE -nn arp 2>/dev/null | head -20
cat /proc/net/arp | grep -v '00:00:00:00:00:00'
} | tee "$F"

echo
echo "DONE -> $F"
