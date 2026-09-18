#!/bin/bash
# Broader probe: auto-detects the subnet of every UP, non-loopback, non-docker,
# non-USB-tether interface and scans it fully for live hosts + common ports.
# Use this now that the robot cable moved to a different port with real DHCP.
OUT=/home/ws/diag_out; mkdir -p "$OUT"; F="$OUT/net_probe2.txt"
sudo apt-get install -y -q iproute2 iputils-ping >/dev/null 2>&1

python3 - "$F" <<'PY'
import socket, subprocess, sys, os, glob, ipaddress, concurrent.futures as cf
lines=[]
def p(*a):
    s=" ".join(str(x) for x in a); print(s); lines.append(s)
def sh(c):
    try: return subprocess.run(c,shell=True,capture_output=True,text=True,timeout=20).stdout.strip()
    except Exception as e: return f"(failed: {e})"

SKIP_PREFIX = ("lo","docker","veth","enx")  # enx* = USB tether (phone), not the robot

p("===== candidate interfaces =====")
targets=[]
addr_out = sh("ip -4 -brief addr show")
for line in addr_out.splitlines():
    parts=line.split()
    if len(parts) < 3: continue
    name, state, cidr = parts[0], parts[1], parts[2]
    if name.startswith(SKIP_PREFIX): continue
    if state != "UP": continue
    try:
        net = ipaddress.ip_interface(cidr).network
    except Exception:
        continue
    p(f"  {name:16s} {cidr:20s} state={state} -> scanning {net}")
    targets.append((name, net))

if not targets:
    p("  (no eligible UP interfaces with an address found)")

PORTS={22:"ssh",80:"http",443:"https",502:"modbus",8080:"http-alt",9090:"rosbridge",
       11311:"ros1-master",29999:"UR-dashboard",30001:"UR-primary",30002:"UR-secondary",
       30003:"UR-rt",50002:"ext-ctrl",5900:"vnc",7400:"dds-discovery"}

def probe(h):
    hit=[]
    for pt in PORTS:
        s=socket.socket(); s.settimeout(0.4)
        try:
            if s.connect_ex((h,pt))==0: hit.append(pt)
        except Exception: pass
        finally: s.close()
    return h,hit

for name, net in targets:
    p(f"\n===== scanning {net} on {name} =====")
    hosts=[str(h) for h in net.hosts()]
    if len(hosts) > 512:
        p(f"  (network too large: {len(hosts)} hosts, skipping full scan)")
        continue
    with cf.ThreadPoolExecutor(max_workers=128) as ex:
        for h,hit in sorted(ex.map(probe,hosts),key=lambda x:tuple(int(o) for o in x[0].split('.'))):
            if hit: p(f"  {h:16s} " + ", ".join(f"{pt}/{PORTS[pt]}" for pt in hit))

p("\n===== ARP table (all interfaces) =====")
p(sh("cat /proc/net/arp"))

open(sys.argv[1],"w").write("\n".join(lines)+"\n")
p(f"\nDONE -> {sys.argv[1]}")
PY
