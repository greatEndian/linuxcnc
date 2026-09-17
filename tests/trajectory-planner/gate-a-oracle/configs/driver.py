#!/usr/bin/env python3
"""Headless DISPLAY program for the Gate A oracle.

LinuxCNC launches this instead of a GUI.  It brings the machine up, starts
halsampler, runs one G-code program to completion, stops the capture and
exits, which shuts the stack down.  Everything it needs beyond -ini comes
from the environment so the config stays generic.
"""
import os
import subprocess
import sys
import time

import linuxcnc


def ini_argument():
    for index, argument in enumerate(sys.argv):
        if argument == "-ini":
            return sys.argv[index + 1]
    raise SystemExit("driver.py: no -ini argument")


def fail(message):
    sys.stderr.write("driver.py: %s\n" % message)
    raise SystemExit(1)


def wait_for(predicate, timeout, what):
    deadline = time.time() + timeout
    while time.time() < deadline:
        status.poll()
        if predicate():
            return
        time.sleep(0.01)
    fail("timed out waiting for %s" % what)


ini_argument()
program = os.environ["ORACLE_PROGRAM"]
output = os.environ["ORACLE_OUT"]
settle = float(os.environ.get("ORACLE_SETTLE", "0.5"))

status = linuxcnc.stat()
command = linuxcnc.command()

wait_for(lambda: status.echo_serial_number >= 0, 20.0, "status channel")

command.state(linuxcnc.STATE_ESTOP_RESET)
command.state(linuxcnc.STATE_ON)
command.wait_complete(10.0)
wait_for(lambda: status.enabled and not status.estop, 20.0, "machine on")

command.mode(linuxcnc.MODE_MANUAL)
command.wait_complete(10.0)
for joint in range(status.joints):
    command.home(joint)
wait_for(lambda: all(status.homed[j] for j in range(status.joints)), 30.0, "homing")

command.mode(linuxcnc.MODE_AUTO)
command.wait_complete(10.0)

sampler = subprocess.Popen(
    ["halsampler", "-c", "0", "-t", output],
    stdout=subprocess.DEVNULL,
    stderr=subprocess.PIPE,
)
time.sleep(settle)

command.program_open(program)
command.wait_complete(10.0)
command.auto(linuxcnc.AUTO_RUN, 0)

wait_for(lambda: status.interp_state != linuxcnc.INTERP_IDLE, 20.0, "program start")
wait_for(
    lambda: status.interp_state == linuxcnc.INTERP_IDLE and status.queue == 0,
    600.0,
    "program completion",
)
time.sleep(settle)

sampler.terminate()
try:
    sampler.wait(timeout=10)
except subprocess.TimeoutExpired:
    sampler.kill()

status.poll()
sys.stderr.write(
    "driver.py: finished, position x=%.6f y=%.6f, %d samples\n"
    % (
        status.actual_position[0],
        status.actual_position[1],
        sum(1 for _ in open(output)),
    )
)
