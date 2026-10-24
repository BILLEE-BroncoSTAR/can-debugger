"""Tkinter desktop interface for the ODrive CANSimple candump decoder."""

from __future__ import annotations

import queue
import subprocess
import threading
import tkinter as tk
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

from .decoder import DecodedFrame, decode_line


SAMPLE = """(1700000000.000001) can0 029#0000A03F00000040
(1700000000.010001) can0 021#00000000080001
(1700000000.020001) can0 029#0000484100002040
(1700000000.030001) can0 02E#0000C03F
"""


class CandumpDecoderApp(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("ODrive CANSimple Candump Decoder")
        self.minsize(980, 620)
        self.geometry("1380x800")
        self._process: subprocess.Popen[str] | None = None
        self._lines: queue.Queue[str | None] = queue.Queue()
        self._build_ui()
        self.after(75, self._consume_live_lines)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build_ui(self) -> None:
        controls = ttk.Frame(self, padding=(10, 10, 10, 4))
        controls.grid(row=0, column=0, sticky="ew")
        controls.columnconfigure(7, weight=1)
        ttk.Label(controls, text="CAN interface:").grid(row=0, column=0, padx=(0, 5))
        self.interface = tk.StringVar(value="can0")
        ttk.Entry(controls, textvariable=self.interface, width=12).grid(row=0, column=1, padx=(0, 8))
        self.live_button = ttk.Button(controls, text="Start candump", command=self._toggle_candump)
        self.live_button.grid(row=0, column=2, padx=(0, 5))
        ttk.Button(controls, text="Open log…", command=self._open_log).grid(row=0, column=3, padx=5)
        ttk.Button(controls, text="Decode pasted text", command=self._decode_paste).grid(row=0, column=4, padx=5)
        ttk.Button(controls, text="Clear", command=self._clear).grid(row=0, column=5, padx=5)
        self.status = tk.StringVar(value="Paste candump lines below, open a log, or start live capture.")
        ttk.Label(controls, textvariable=self.status).grid(row=0, column=7, sticky="e", padx=(12, 0))

        paste_frame = ttk.LabelFrame(self, text="candump input", padding=7)
        paste_frame.grid(row=1, column=0, sticky="ew", padx=10, pady=(0, 7))
        paste_frame.columnconfigure(0, weight=1)
        self.paste = tk.Text(paste_frame, height=5, wrap="none", font="TkFixedFont")
        self.paste.grid(row=0, column=0, sticky="ew")
        self.paste.insert("1.0", SAMPLE)

        table_frame = ttk.Frame(self, padding=(10, 0, 10, 8))
        table_frame.grid(row=2, column=0, sticky="nsew")
        self.rowconfigure(2, weight=1)
        self.columnconfigure(0, weight=1)
        columns = ("time", "interface", "can_id", "node", "command", "data", "decoded")
        self.table = ttk.Treeview(table_frame, columns=columns, show="headings", selectmode="browse")
        headings = {"time": "Time", "interface": "Interface", "can_id": "CAN ID", "node": "Node ID", "command": "Command code / name", "data": "Raw payload", "decoded": "Human-readable message"}
        widths = {"time": 115, "interface": 80, "can_id": 75, "node": 85, "command": 190, "data": 180, "decoded": 540}
        for name in columns:
            self.table.heading(name, text=headings[name])
            self.table.column(name, width=widths[name], minwidth=50, stretch=name == "decoded")
        scroll_y = ttk.Scrollbar(table_frame, orient="vertical", command=self.table.yview)
        scroll_x = ttk.Scrollbar(table_frame, orient="horizontal", command=self.table.xview)
        self.table.configure(yscrollcommand=scroll_y.set, xscrollcommand=scroll_x.set)
        self.table.grid(row=0, column=0, sticky="nsew")
        scroll_y.grid(row=0, column=1, sticky="ns")
        scroll_x.grid(row=1, column=0, sticky="ew")
        table_frame.rowconfigure(0, weight=1)
        table_frame.columnconfigure(0, weight=1)

    def _add_line(self, line: str) -> bool:
        if not line.strip():
            return False
        try:
            decoded = decode_line(line)
        except ValueError as exc:
            self.status.set(f"Skipped: {exc} — {line.strip()[:85]}")
            return False
        self._insert(decoded)
        return True

    def _insert(self, decoded: DecodedFrame) -> None:
        frame = decoded.frame
        command = f"0x{decoded.command_id:02X}  {decoded.command_name}"
        node = f"{decoded.node_id} (0x{decoded.node_id:02X})" + (" broadcast" if decoded.node_id == 0x3F else "")
        self.table.insert("", "end", values=(frame.timestamp, frame.interface, f"0x{frame.can_id:03X}", node, command, frame.data.hex(" ").upper() or "(empty)", decoded.description))
        children = self.table.get_children()
        if len(children) > 5000:
            self.table.delete(children[0])
        self.table.yview_moveto(1)

    def _decode_paste(self) -> None:
        count = sum(self._add_line(line) for line in self.paste.get("1.0", "end").splitlines())
        self.status.set(f"Decoded {count} frame{'s' if count != 1 else ''} from pasted input.")

    def _open_log(self) -> None:
        filename = filedialog.askopenfilename(title="Open candump log", filetypes=[("Log files", "*.log *.txt *.asc"), ("All files", "*")])
        if not filename:
            return
        try:
            lines = Path(filename).read_text(encoding="utf-8", errors="replace").splitlines()
        except OSError as exc:
            messagebox.showerror("Could not open log", str(exc))
            return
        count = sum(self._add_line(line) for line in lines)
        self.status.set(f"Decoded {count} frame{'s' if count != 1 else ''} from {Path(filename).name}.")

    def _clear(self) -> None:
        self.table.delete(*self.table.get_children())
        self.status.set("Results cleared.")

    def _toggle_candump(self) -> None:
        if self._process is not None:
            self._stop_candump()
            return
        interface = self.interface.get().strip()
        if not interface or any(ch.isspace() for ch in interface):
            messagebox.showerror("Invalid interface", "Enter a SocketCAN interface such as can0.")
            return
        try:
            self._process = subprocess.Popen(["candump", interface], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
        except FileNotFoundError:
            messagebox.showerror("candump not found", "Install can-utils so the candump command is available, or use Open log / pasted input.")
            return
        except OSError as exc:
            messagebox.showerror("Could not start candump", str(exc))
            return
        threading.Thread(target=self._read_candump, daemon=True).start()
        self.live_button.configure(text="Stop candump")
        self.status.set(f"Capturing live frames from {interface}…")

    def _read_candump(self) -> None:
        assert self._process is not None and self._process.stdout is not None
        for line in self._process.stdout:
            self._lines.put(line)
        self._lines.put(None)

    def _consume_live_lines(self) -> None:
        count = 0
        while True:
            try:
                line = self._lines.get_nowait()
            except queue.Empty:
                break
            if line is None:
                if self._process is not None and self._process.poll() is not None:
                    code = self._process.returncode
                    self._process = None
                    self.live_button.configure(text="Start candump")
                    self.status.set(f"candump exited (status {code}).")
                continue
            count += self._add_line(line)
        if count:
            self.status.set(f"Captured and decoded {count} live frame{'s' if count != 1 else ''}.")
        self.after(75, self._consume_live_lines)

    def _stop_candump(self) -> None:
        if self._process is not None:
            self._process.terminate()
            self._process = None
        self.live_button.configure(text="Start candump")
        self.status.set("Live capture stopped.")

    def _on_close(self) -> None:
        self._stop_candump()
        self.destroy()


def main() -> None:
    """Run the desktop application."""
    app = CandumpDecoderApp()
    app.mainloop()
