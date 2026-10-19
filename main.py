"""
main.py
Application entry point for the Python Indenter Control System.
Run this file on the lab Windows PC to launch the complete S1 -> S4 GUI.
"""

import tkinter as tk
from gui import IndenterOperatorGUI


def main() -> None:
    root = tk.Tk()
    app = IndenterOperatorGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()