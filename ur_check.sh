#!/bin/bash
# Check the UR controller found via DHCP on the "other port" (192.168.60.90),
# instead of the old hardcoded static default (192.168.1.102).
OUT=/home/ws/diag_out; mkdir -p "$OUT"; F="$OUT/ur_check.txt"
sudo apt-get install -y -q iputils-ping >/dev/null 2>&1

python3 - "$F" <<'PY'
import socket, subprocess, sys

lines=[]
def p(*a):
    s=" ".join(str(x) for x in a); print(s); lines.append(s)
def sh(c):
    try: return subprocess.run(c,shell=True,capture_output=True,text=True,timeout=15).stdout.strip()
    except Exception as e: return f"(failed: {e})"

HOST = "192.168.60.90"
PORTS = {29999: "dashboard", 30001: "primary", 30002: "secondary", 30003: "realtime"}

p(f"===== ping {HOST} =====")
p(sh(f"ping -c3 -W2 {HOST}"))

p(f"\n===== UR ports on {HOST} =====")
for port, name in PORTS.items():
    s = socket.socket(); s.settimeout(2.0)
    r = s.connect_ex((HOST, port)); s.close()
    p(f"  {HOST}:{port} ({name}) " + ("OPEN" if r == 0 else "closed/unreachable"))

open(sys.argv[1], "w").write("\n".join(lines) + "\n")
p(f"\nDONE -> {sys.argv[1]}")
PY
