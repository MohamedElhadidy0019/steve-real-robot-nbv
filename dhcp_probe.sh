#!/bin/bash
# The robot's "DHCP" port is a DHCP CLIENT waiting for a lease. First listen for its
# DHCP DISCOVER, then run a short-lived DHCP server on enp3s0 ONLY so it gets an
# address and we can see/reach it. enp3s0 is the isolated robot link -> safe.
IFACE=enp3s0
OUT=/home/ws/diag_out; mkdir -p "$OUT"; F="$OUT/dhcp_probe.txt"; : > "$F"

# --- SAFETY: never run a DHCP server on the university NIC ---
IP=$(python3 -c "import subprocess;print(subprocess.run(['cat','/sys/class/net/$IFACE/address'],capture_output=True,text=True).stdout.strip())")
HOSTIP=$(grep -oE 'inet [0-9.]+/[0-9]+' <(ip -4 addr show "$IFACE" 2>/dev/null) | awk '{print $2}')
echo "iface=$IFACE mac=$IP addr=$HOSTIP" | tee -a "$F"
case "$HOSTIP" in
  192.168.1.*) : ;;
  *) echo "ABORT: $IFACE is not 192.168.1.x -- refusing to serve DHCP" | tee -a "$F"; exit 1;;
esac

sudo apt-get install -y -q dnsmasq tcpdump iproute2 iputils-ping >/dev/null 2>&1
sudo pkill dnsmasq 2>/dev/null; sleep 1
sudo rm -f /tmp/dnsmasq.leases /tmp/dnsmasq.log

echo -e "\n===== STEP 1: listen 30s for a DHCP DISCOVER from the robot =====" | tee -a "$F"
sudo timeout 30 tcpdump -i "$IFACE" -nn -e 'udp port 67 or udp port 68' 2>&1 | tee -a "$F"

echo -e "\n===== STEP 2: temporary DHCP server on $IFACE (serves 192.168.1.60-120) =====" | tee -a "$F"
sudo dnsmasq --interface="$IFACE" --bind-interfaces --port=0 \
  --dhcp-authoritative \
  --dhcp-range=192.168.1.60,192.168.1.120,255.255.255.0,3m \
  --dhcp-option=3 --dhcp-option=6 \
  --dhcp-leasefile=/tmp/dnsmasq.leases \
  --log-dhcp --log-facility=/tmp/dnsmasq.log 2>&1 | tee -a "$F"

echo "  (waiting 75s for the robot to request an address...)" | tee -a "$F"
sleep 75

echo -e "\n===== dnsmasq log =====" | tee -a "$F"
sudo cat /tmp/dnsmasq.log 2>/dev/null | tee -a "$F"
echo -e "\n===== leases handed out =====" | tee -a "$F"
sudo cat /tmp/dnsmasq.leases 2>/dev/null | tee -a "$F"

echo -e "\n===== who's on the wire now =====" | tee -a "$F"
grep -v '00:00:00:00:00:00' /proc/net/arp | tee -a "$F"
for h in $(sudo cat /tmp/dnsmasq.leases 2>/dev/null | awk '{print $3}'); do
  echo "--- $h ---" | tee -a "$F"
  for p in 22 29999 30001 30002 30003; do
    timeout 1 bash -c "echo >/dev/tcp/$h/$p" 2>/dev/null && echo "   $h:$p OPEN" | tee -a "$F"
  done
done

echo -e "\n===== ping UR 192.168.1.102 =====" | tee -a "$F"
ping -c 3 -W 1 192.168.1.102 2>&1 | tee -a "$F"

sudo pkill dnsmasq 2>/dev/null
echo -e "\nDONE (DHCP server stopped) -> $F" | tee -a "$F"
