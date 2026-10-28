"""
data_loader.py
Stage 1A: Multi-format file loader for .csv, .txt, .xlsx, and extensionless
legacy LabVIEW spreadsheet files (shown as "File" on Windows or "exec" on macOS).
Includes a visible Tkinter Inspector Window so macOS never hides the app.
"""

import csv
import os
import sys
from typing import List, Optional

from sample_model import RawFileData


SUPPORTED_EXTENSIONS = ("", ".csv", ".txt", ".dat", ".tsv", ".xlsx", ".xls")
REJECTED_BINARY_EXTENSIONS = (
    ".exe", ".dll", ".png", ".jpg", ".jpeg", ".gif", ".bmp",
    ".pdf", ".doc", ".docx", ".ppt", ".pptx", ".zip", ".rar", ".py", ".pyc", ".ds_store"
)


class UnsupportedFileError(Exception):
    """Raised when an unsupported binary file extension is selected."""
    pass


class FileReadError(Exception):
    """Raised when a file is empty, unreadable, or corrupted."""
    pass


def _is_float_token(val: str) -> bool:
    """Returns True if the string token can be parsed as a floating-point number."""
    s = val.strip()
    if not s:
        return False
    try:
        float(s)
        return True
    except ValueError:
        return False


def _detect_header_row(first_row: List[str]) -> bool:
    """
    Returns True if the first row looks like text column headers,
    or False if it is already numeric data (such as headerless LabVIEW files).
    """
    non_empty = [cell.strip() for cell in first_row if cell.strip() != ""]
    if not non_empty:
        return False
    numeric_count = sum(1 for cell in non_empty if _is_float_token(cell))
    return numeric_count < len(non_empty)


def _read_delimited_text(file_path: str) -> List[List[str]]:
    """Reads CSV, TXT, or extensionless LabVIEW spreadsheet files."""
    with open(file_path, "rb") as bf:
        sample_bytes = bf.read(2048)
        if b"\x00" in sample_bytes:
            raise UnsupportedFileError(
                "Unable to read this file.\n\n"
                "The selected file appears to be a binary file, not a spreadsheet data file."
            )

    encodings_to_try = ("utf-8-sig", "utf-8", "cp950", "big5", "latin-1")
    text_content = None

    for enc in encodings_to_try:
        try:
            with open(file_path, "r", encoding=enc, errors="strict") as f:
                text_content = f.read()
            break
        except UnicodeDecodeError:
            continue

    if text_content is None:
        raise FileReadError("Unable to decode text encoding of this file.")

    lines = [line for line in text_content.splitlines() if line.strip() != ""]
    if not lines:
        raise FileReadError("The selected file contains no usable data.")

    sample_line = lines[0]
    if "\t" in sample_line:
        delimiter = "\t"
    elif "," in sample_line:
        delimiter = ","
    elif ";" in sample_line:
        delimiter = ";"
    else:
        delimiter = None  # Whitespace-separated

    parsed_rows: List[List[str]] = []
    if delimiter is not None:
        reader = csv.reader(lines, delimiter=delimiter)
        for row in reader:
            cleaned = [cell.strip() for cell in row]
            while cleaned and cleaned[-1] == "":
                cleaned.pop()
            if any(cleaned):
                parsed_rows.append(cleaned)
    else:
        for line in lines:
            cleaned = line.strip().split()
            if cleaned:
                parsed_rows.append(cleaned)

    return parsed_rows


def _read_excel_file(file_path: str) -> List[List[str]]:
    """Reads .xlsx / .xls files if openpyxl or pandas is installed."""
    try:
        import openpyxl
        wb = openpyxl.load_workbook(file_path, read_only=True, data_only=True)
        sheet = wb.active
        rows: List[List[str]] = []
        for row in sheet.iter_rows(values_only=True):
            cleaned = ["" if v is None else str(v).strip() for v in row]
            while cleaned and cleaned[-1] == "":
                cleaned.pop()
            if any(cleaned):
                rows.append(cleaned)
        wb.close()
        return rows
    except ImportError:
        try:
            import pandas as pd
            df = pd.read_excel(file_path)
            header = [str(c).strip() for c in df.columns]
            data_rows = [
                ["" if pd.isna(v) else str(v).strip() for v in row]
                for row in df.to_numpy()
            ]
            return [header] + data_rows
        except ImportError as exc:
            raise FileReadError(
                "Reading Excel (.xlsx) files requires 'openpyxl' or 'pandas'.\n"
                "Please convert the file to CSV (.csv) or TXT (.txt), or install openpyxl."
            ) from exc


def load_raw_file(file_path: str) -> RawFileData:
    """
    Loads a .csv, .txt, .xlsx, or extensionless LabVIEW file without modifying
    the original file, detects whether headers exist, and returns RawFileData.
    """
    if not file_path or not os.path.exists(file_path):
        raise FileReadError(f"File not found:\n{file_path}")
    if os.path.isdir(file_path):
        raise FileReadError(f"Selected path is a folder, not a file:\n{file_path}")

    file_name = os.path.basename(file_path)
    _, ext = os.path.splitext(file_name)
    ext_lower = ext.lower()

    if ext_lower in REJECTED_BINARY_EXTENSIONS or file_name.startswith("."):
        raise UnsupportedFileError(
            "Unable to read this file.\n\n"
            f"Unsupported format: '{ext_lower or file_name}'\n\n"
            "Supported formats:\n"
            "- LabVIEW Raw Spreadsheet (No extension)\n"
            "- CSV (.csv)\n"
            "- Excel (.xlsx)\n"
            "- TXT (.txt)"
        )

    try:
        if ext_lower in (".xlsx", ".xls"):
            all_rows = _read_excel_file(file_path)
            detected_type = ext_lower.lstrip(".").upper()
        else:
            all_rows = _read_delimited_text(file_path)
            detected_type = (
                "LABVIEW_RAW (No Ext)" if ext_lower == "" else ext_lower.lstrip(".").upper()
            )
    except (UnsupportedFileError, FileReadError):
        raise
    except Exception as exc:
        raise FileReadError(
            f"Unable to read this file ({file_name}).\nDetails: {exc}"
        ) from exc

    if not all_rows:
        raise FileReadError("The selected file contains no usable data.")

    max_cols = max(len(r) for r in all_rows)
    if max_cols == 0:
        raise FileReadError("The selected file has 0 columns.")

    normalized_rows = [
        r + [""] * (max_cols - len(r)) for r in all_rows
    ]

    has_header = _detect_header_row(normalized_rows[0])
    if has_header:
        raw_headers = normalized_rows[0]
        column_names = [
            h if h != "" else f"Column_{idx}"
            for idx, h in enumerate(raw_headers)
        ]
        data_rows = normalized_rows[1:]
    else:
        if max_cols == 2:
            column_names = ["Column_0 (LabVIEW_Pos_Steps)", "Column_1 (LabVIEW_Force_V)"]
        else:
            column_names = [f"Column_{idx}" for idx in range(max_cols)]
        data_rows = normalized_rows

    if not data_rows:
        raise FileReadError("The selected file contains headers but 0 data rows.")

    return RawFileData(
        file_path=file_path,
        file_name=file_name,
        file_type=detected_type,
        has_header_row=has_header,
        column_names=column_names,
        rows=data_rows,
        total_rows=len(data_rows),
        total_cols=max_cols,
    )


def format_preview_text(raw_data: RawFileData, max_preview_rows: int = 15) -> str:
    """Formats a clean console/GUI preview of the loaded file and its first N rows."""
    lines = [
        "=" * 72,
        f"File Name:      {raw_data.file_name}",
        f"Full Path:      {raw_data.file_path}",
        f"Detected Type:  {raw_data.file_type}",
        f"Header Row:     {'Yes (Detected)' if raw_data.has_header_row else 'No (Headerless LabVIEW Data)'}",
        f"Total Rows:     {raw_data.total_rows}",
        f"Columns ({raw_data.total_cols}):    {', '.join(raw_data.column_names)}",
        "-" * 72,
        f"Preview (First {min(max_preview_rows, raw_data.total_rows)} Rows):",
    ]

    col_widths = [max(14, len(c) + 2) for c in raw_data.column_names]
    header_line = " | ".join(
        c.ljust(col_widths[i]) for i, c in enumerate(raw_data.column_names)
    )
    lines.append(header_line)
    lines.append("-" * len(header_line))

    for idx, row in enumerate(raw_data.rows[:max_preview_rows]):
        row_line = f"[{idx:02d}] " + " | ".join(
            str(val).ljust(col_widths[i]) for i, val in enumerate(row)
        )
        lines.append(row_line)

    if raw_data.total_rows > max_preview_rows:
        lines.append(f"... ({raw_data.total_rows - max_preview_rows} more rows)")
    lines.append("=" * 72)
    return "\n".join(lines)


# =============================================================================
# Visible Stage 1A Sample Inspector GUI (Works cleanly on macOS & Windows)
# =============================================================================
def launch_sample_inspector() -> None:
    import tkinter as tk
    from tkinter import filedialog, ttk

    root = tk.Tk()
    root.title("Stage 1A: Historical LabVIEW & CSV Sample Reader")
    root.geometry("950x560")

    # Bring window to front on macOS
    root.lift()
    root.attributes("-topmost", True)
    root.after(300, lambda: root.attributes("-topmost", False))

    # Try to find a default 'Samples' folder nearby, otherwise use current directory
    base_dir = os.path.dirname(os.path.abspath(__file__))
    candidates = [
        os.path.join(base_dir, "Samples"),
        os.path.join(base_dir, "samples"),
        os.path.join(os.path.dirname(base_dir), "Samples"),
        os.path.expanduser("~/Desktop/Samples"),
        base_dir,
    ]
    current_folder = next((p for p in candidates if os.path.isdir(p)), base_dir)

    top_bar = ttk.Frame(root, padding=8)
    top_bar.pack(fill=tk.X)

    folder_var = tk.StringVar(value=current_folder)
    ttk.Label(top_bar, text="Folder:").pack(side=tk.LEFT)
    folder_entry = ttk.Entry(top_bar, textvariable=folder_var, width=55)
    folder_entry.pack(side=tk.LEFT, padx=6, fill=tk.X, expand=True)

    main_pane = ttk.PanedWindow(root, orient=tk.HORIZONTAL)
    main_pane.pack(fill=tk.BOTH, expand=True, padx=8, pady=4)

    # Left panel: File list
    left_frame = ttk.Labelframe(
        main_pane, text="Files in Folder (Click Any File to Read)", padding=6
    )
    main_pane.add(left_frame, weight=1)

    file_listbox = tk.Listbox(left_frame, width=32, font=("Courier", 12))
    file_scroll = ttk.Scrollbar(left_frame, orient=tk.VERTICAL, command=file_listbox.yview)
    file_listbox.configure(yscrollcommand=file_scroll.set)
    file_listbox.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
    file_scroll.pack(side=tk.RIGHT, fill=tk.Y)

    # Right panel: File preview
    right_frame = ttk.Labelframe(
        main_pane, text="Loaded File Structure & Data Preview", padding=6
    )
    main_pane.add(right_frame, weight=3)

    preview_text = tk.Text(
        right_frame, wrap=tk.NONE, font=("Courier", 12), bg="#1e1e1e", fg="#d4d4d4"
    )
    y_scroll = ttk.Scrollbar(right_frame, orient=tk.VERTICAL, command=preview_text.yview)
    x_scroll = ttk.Scrollbar(right_frame, orient=tk.HORIZONTAL, command=preview_text.xview)
    preview_text.configure(yscrollcommand=y_scroll.set, xscrollcommand=x_scroll.set)

    y_scroll.pack(side=tk.RIGHT, fill=tk.Y)
    x_scroll.pack(side=tk.BOTTOM, fill=tk.X)
    preview_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

    def show_message(msg: str) -> None:
        preview_text.delete("1.0", tk.END)
        preview_text.insert(tk.END, msg)

    def refresh_file_list(folder_path: str) -> None:
        file_listbox.delete(0, tk.END)
        if not os.path.isdir(folder_path):
            show_message(f"Folder does not exist:\n{folder_path}")
            return
        folder_var.set(folder_path)
        items = sorted(os.listdir(folder_path))
        count = 0
        for name in items:
            full_p = os.path.join(folder_path, name)
            if os.path.isfile(full_p) and not name.startswith("."):
                _, ext = os.path.splitext(name)
                if ext.lower() not in REJECTED_BINARY_EXTENSIONS:
                    file_listbox.insert(tk.END, name)
                    count += 1
        show_message(
            f"Loaded folder: {folder_path}\n"
            f"Found {count} candidate sample files.\n\n"
            "-> Click any file on the left (e.g., RUBIN16036AGM1) to inspect its contents!"
        )

    def on_select_file(event=None) -> None:
        selection = file_listbox.curselection()
        if not selection:
            return
        file_name = file_listbox.get(selection[0])
        full_path = os.path.join(folder_var.get(), file_name)
        try:
            from data_validator import validate_raw_data, format_validation_report
            from data_mapper import map_raw_to_sample_data, format_mapped_sample_summary

            raw_data = load_raw_file(full_path)
            val_report = validate_raw_data(raw_data)
            sample_obj = map_raw_to_sample_data(raw_data)

            combined_report = (
                format_preview_text(raw_data, max_preview_rows=10)
                + "\n\n"
                + format_validation_report(val_report)
                + "\n" + ("=" * 72) + "\n"
                + format_mapped_sample_summary(sample_obj)
                + "\n" + ("=" * 72)
            )
            show_message(combined_report)
            print(combined_report)
        except Exception as exc:
            show_message(f"[ERROR READING {file_name}]\n\n{exc}")

    def browse_folder() -> None:
        chosen = filedialog.askdirectory(
            parent=root,
            initialdir=folder_var.get(),
            title="Select Folder Containing LabVIEW Sample Files (e.g., Samples)",
        )
        if chosen:
            refresh_file_list(chosen)

    ttk.Button(top_bar, text="Choose Folder...", command=browse_folder).pack(
        side=tk.LEFT, padx=4
    )
    ttk.Button(
        top_bar, text="Refresh", command=lambda: refresh_file_list(folder_var.get())
    ).pack(side=tk.LEFT, padx=4)

    file_listbox.bind("<<ListboxSelect>>", on_select_file)
    refresh_file_list(current_folder)

    root.mainloop()


if __name__ == "__main__":
    launch_sample_inspector()