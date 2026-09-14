#!/usr/bin/env python3
"""Exercise real tty single-key input, stage transitions and terminal cleanup."""
import os
from pathlib import Path
import pty
import select
import signal
import subprocess
import sys
import tempfile
import termios
import time

from run_b2_z1_v4 import ROOT, TerminalKeyboard


def main():
    master, slave = pty.openpty()
    original = termios.tcgetattr(slave)
    try:
        with os.fdopen(os.dup(slave), 'r') as terminal:
            try:
                with TerminalKeyboard(terminal) as keyboard:
                    settings = termios.tcgetattr(slave)
                    assert not settings[3] & (termios.ICANON | termios.ECHO)
                    assert settings[3] & termios.ISIG == original[3] & termios.ISIG
                    for key in b'019tT ':
                        os.write(master, bytes([key]))
                        assert select.select([slave], [], [], 1)[0]
                        assert keyboard.read() == [key], 'Key required Enter or was lost'
                    os.write(master, b'\x1b[')
                    assert select.select([slave], [], [], 1)[0]
                    assert keyboard.read() == []
                    os.write(master, b'1;5A')
                    assert select.select([slave], [], [], 1)[0]
                    assert keyboard.read() == [], 'Arrow-key digit activated a stage'
                    raise RuntimeError('Exercise exceptional terminal cleanup')
            except RuntimeError:
                pass
        assert termios.tcgetattr(slave) == original

        with tempfile.TemporaryDirectory(prefix='b2_z1_v4_terminal_check_') as output:
            process = subprocess.Popen(
                [sys.executable, '-u', str(ROOT/'scripts/run_b2_z1_v4.py'),
                 '--headless', '--mode', 'walk', '--duration', '20',
                 '--output', str(Path(output)/'run.json')],
                stdin=slave, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            )
            pending = b''
            transcript = []
            stage = 'launch'
            toggles = 0
            entered = time.monotonic()
            deadline = entered+30
            try:
                while process.poll() is None and time.monotonic() < deadline:
                    now = time.monotonic()
                    if stage == 'tracking' and now-entered >= 2.:
                        os.write(master, b't' if toggles == 0 else b'T')
                        stage = 'stopping'
                    if select.select([process.stdout], [], [], .05)[0]:
                        pending += os.read(process.stdout.fileno(), 8192)
                        while b'\n' in pending:
                            raw, pending = pending.split(b'\n', 1)
                            line = raw.decode(errors='replace')
                            transcript.append(line)
                            if 'TERMINAL keys' in line:
                                os.write(master, b'0')
                                stage, entered = 'standing', time.monotonic()
                            elif line.startswith('Standing settled:') and stage == 'standing':
                                os.write(master, b'1')
                                stage = 'preparing'
                            elif line == 'Stage: ready' and stage == 'preparing':
                                os.write(master, b't')
                            elif line == 'Stage: tracking':
                                stage, entered = 'tracking', time.monotonic()
                            elif line == 'Stage: ready' and stage == 'stopping':
                                toggles += 1
                                if toggles == 1:
                                    os.write(master, b't')
                                    stage = 'restarting'
                                else:
                                    process.send_signal(signal.SIGINT)
                                    stage = 'exiting'
                assert stage == 'exiting', '\n'.join(transcript)
                assert process.wait(timeout=5) == 0, '\n'.join(transcript)
                assert termios.tcgetattr(slave) == original, 'TTY was not restored after Ctrl+C'
                print('PASS: terminal 0 -> 1 -> t -> t (stop) -> t -> T (stop); no Enter; Ctrl+C cleanup and tty restoration')
            finally:
                if process.poll() is None:
                    process.kill()
                    process.wait(timeout=5)
                process.stdout.close()
    finally:
        os.close(master)
        os.close(slave)


if __name__ == '__main__':
    main()
