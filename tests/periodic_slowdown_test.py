import subprocess
import sys
import re

NS3_CMD = ["./ns3", "run", "scratch/LedbatPlusPlusSlowdown"]

# regular expressions to check each stage of the slowdown
EXIT_INIT_SS_RE = re.compile(r"Exiting initial slow start due to exceeding 3/4 of target delay")
WAITING_RE = re.compile(r"Waiting 2 RTT for initial slowdown period to start")
HOLDING_RE = re.compile(r"Holding cwnd at 2 packets for 2 RTT")
RTT_OVER_RE = re.compile(r"2 RTT over, growing cwnd to ssthresh through slow start")
FINISHED_RE  = re.compile(r"Slowdown finished")

proc = subprocess.run(
    NS3_CMD,
    stdout=subprocess.PIPE,
    stderr=subprocess.STDOUT,
    text=True
)

# splits the output into individual lines
log = proc.stdout.splitlines()

# stores the order of the events in slowdown process
events = []
for line in log:
    if EXIT_INIT_SS_RE.search(line):
        events.append("exit_ss")
    elif WAITING_RE.search(line):
        events.append("waiting")
    elif HOLDING_RE.search(line):
        events.append("holding")
    elif RTT_OVER_RE.search(line):
        events.append("rtt_over")
    elif FINISHED_RE.search(line):
        events.append("finished")

# --- helper functions ---
# finds the target in the events list starting from start index
def find_next(events, target, start):
    for i in range(start, len(events)):
        if events[i] == target:
            return i
    return -1


all_pass = True

# Test 1: CHECK initial slow start exited 
if "exit_ss" not in events:
    print("FAIL: initial slow start never exited")
    all_pass = False
else:
    print("PASS: initial slow start exited correctly")

# CHECK waiting period followed exiting initial slow start
exit_idx = find_next(events, "exit_ss", 0)
if exit_idx > 0:
    waiting_idx = find_next(events, "waiting", exit_idx)
    if waiting_idx == -1:
        print("FAIL: no waiting period after initial slow start exit")
        all_pass = False
    else:
        print("PASS: waiting period observed after initial slow start exit")

# CHECK at least one full slowdown cycle completed
holding_idx = find_next(events, "holding", 0)
if holding_idx == -1:
    print("FAIL: cwnd was never frozen at 2 packets")
    all_pass = False
else:
    rtt_over_idx = find_next(events, "rtt_over", holding_idx)
    if rtt_over_idx == -1:
        print("FAIL: 2 RTT freeze started but never ended")
        all_pass = False
    else:
        finished_idx = find_next(events, "finished", rtt_over_idx)
        if finished_idx == -1:
            print("FAIL: slowdown recovery started but slowdown never finished")
            all_pass = False
        else:
            print("PASS: initial slowdown cycle completed (freeze -> recovery -> finished)")

# Test 2: periodic slowdown triggered at least once after initial slowdown
# find the first finished, then look for another holding after it
first_finished = find_next(events, "finished", 0)
if first_finished == -1:
    print("FAIL: no slowdown finished at all")
    all_pass = False
else:
    periodic_holding = find_next(events, "holding", first_finished + 1)
    if periodic_holding == -1:
        print("FAIL: no periodic slowdown triggered after initial slowdown")
        all_pass = False
    else:
        periodic_rtt_over = find_next(events, "rtt_over", periodic_holding)
        periodic_finished = find_next(events, "finished", periodic_rtt_over) if periodic_rtt_over >= 0 else -1
        if periodic_finished == -1:
            print("FAIL: periodic slowdown started but did not complete")
            all_pass = False
        else:
            print("PASS: periodic slowdown triggered and completed")


# Test 3: count how many full periodic cycles completed
# print(events)
cycles = 0
search_from = find_next(events, "finished", 0)
while search_from >= 0:
    h = find_next(events, "holding", search_from + 1)
    if h == -1:
        break  # no more cycles, not a failure
    r = find_next(events, "rtt_over", h)
    f = find_next(events, "finished", r) if r >= 0 else -1
    if f == -1:
        if r == -1:
            print(f"FAIL: cycle {cycles + 1} holding started but freeze never ended")
        else:
            print(f"FAIL: cycle {cycles + 1} recovery started but never finished")
        all_pass = False
        break
    cycles += 1
    search_from = f

print(f"INFO: {cycles} periodic slowdown cycle(s) completed after initial slowdown")
if cycles == 0:
    print("FAIL: expected at least 1 periodic slowdown cycle")
    all_pass = False

# Test Conclusion
print()
if all_pass:
    print("PASS: LEDBAT++ periodic slowdown conforms to RFC section 4.4")
    sys.exit(0)
else:
    print("FAIL: one or more periodic slowdown checks failed")
    sys.exit(1)