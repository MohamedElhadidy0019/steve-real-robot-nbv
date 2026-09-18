#!/bin/bash
# Probe the robot link (enp3s0 / 192.168.1.0/24) on the Neobotix.
OUT=/home/ws/diag_out; mkdir -p "$OUT"; F="$OUT/net_probe.txt"
sudo apt-get install -y -q iproute2 iputils-ping net-tools >/dev/null 2>&1

python3 - "$F" <<'PY'
import socket, subprocess, sys, os, glob, concurrent.futures as cf
lines=[]
def p(*a):
    s=" ".join(str(x) for x in a); print(s); lines.append(s)
def sh(c):
    try: return subprocess.run(c,shell=True,capture_output=True,text=True,timeout=20).stdout.strip()
    except Exception as e: return f"(failed: {e})"

ROBOT_NET="192.168.1"

p("===== NIC carrier / link state =====")
for dev in sorted(os.path.basename(d) for d in glob.glob("/sys/class/net/*")):
    if dev=="lo": continue
    try:
        car=open(f"/sys/class/net/{dev}/carrier").read().strip()
        op =open(f"/sys/class/net/{dev}/operstate").read().strip()
        sp =sh(f"cat /sys/class/net/{dev}/speed 2>/dev/null")
        p(f"  {dev:12s} carrier={car} operstate={op} speed={sp}")
    except Exception as e:
        p(f"  {dev:12s} ? {e}")

p("\n===== addresses / routes =====")
p(sh("ip -brief addr 2>/dev/null || ifconfig -a"))
p(sh("ip route 2>/dev/null || route -n"))

p("\n===== ARP table =====")
p(sh("cat /proc/net/arp"))

p(f"\n===== TCP scan {ROBOT_NET}.0/24 =====")
PORTS={22:"ssh",80:"http",443:"https",29999:"UR-dashboard",30001:"UR-primary",
       30002:"UR-secondary",30003:"UR-rt",50002:"ext-ctrl",3389:"rdp",5900:"vnc"}
def probe(h):
    hit=[]
    for pt in PORTS:
        s=socket.socket(); s.settimeout(0.5)
        try:
            if s.connect_ex((h,pt))==0: hit.append(pt)
        except Exception: pass
        finally: s.close()
    return h,hit
hosts=[f"{ROBOT_NET}.{i}" for i in range(1,255)]
with cf.ThreadPoolExecutor(max_workers=128) as ex:
    for h,hit in sorted(ex.map(probe,hosts),key=lambda x:int(x[0].split('.')[-1])):
        if hit: p(f"  {h:16s} " + ", ".join(f"{pt}/{PORTS[pt]}" for pt in hit))

p(f"\n===== focused check on UR default {ROBOT_NET}.102 =====")
for pt in (29999,30001,30002,30003):
    s=socket.socket(); s.settimeout(1.0)
    r=s.connect_ex((f"{ROBOT_NET}.102",pt)); s.close()
    p(f"  {ROBOT_NET}.102:{pt} {'OPEN' if r==0 else 'closed/unreachable'}")

p("\n===== ROS2 over network =====")
p(f"ROS_LOCALHOST_ONLY={os.environ.get('ROS_LOCALHOST_ONLY')}  ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID')}")
p(sh("ROS_LOCALHOST_ONLY=0 timeout 8 ros2 node list --no-daemon 2>&1"))

open(sys.argv[1],"w").write("\n".join(lines)+"\n")
p(f"\nDONE -> {sys.argv[1]}")
PY
