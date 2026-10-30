"""Send explicit commands to an already-running local OpenOCD TCL server."""
import argparse
import socket

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("command", nargs="+")
parser.add_argument("--port", type=int, default=6666)
parser.add_argument("--timeout", type=float, default=45.0)
args = parser.parse_args()
with socket.create_connection(("127.0.0.1", args.port), timeout=args.timeout) as connection:
    for command in args.command:
        wrapped = ("set __codex_rc [catch {capture {" + command +
                   "}} __codex_result]; list $__codex_rc $__codex_result")
        connection.sendall(wrapped.encode("utf-8") + b"\x1a")
        response = bytearray()
        while not response.endswith(b"\x1a"):
            block = connection.recv(65536)
            if not block:
                raise RuntimeError("OpenOCD disconnected before replying")
            response.extend(block)
        text = response[:-1].decode("utf-8", errors="replace")
        print(command + "\n" + text, flush=True)
        if command == "shutdown" and text.startswith("-600 "):
            continue
        if not (text == "0 {}" or text.startswith("0 ")):
            raise RuntimeError("OpenOCD command failed: " + text)
