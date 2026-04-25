import App.global_vars
from App.global_vars import *
from Communication.Protocol import *
import threading
import struct
from time import sleep
import tkinter as tk
from tkinter import ttk
from PIL import ImageTk, Image

homing_toplevel = None
motor_homing_toplevel = False

torque_error_toplevel = None
torque_error_toplevel_active = False
info_img = ImageTk.PhotoImage(Image.open('img/info.png').resize((60, 60)))
err_img = ImageTk.PhotoImage(Image.open('img/error.png').resize((60, 60)))
err_img_copy = err_img

total_error_toplevel = None
total_error_prev_state = False


def _run_on_ui_thread(callback, *args):
    root.after(0, lambda: callback(*args))


def _close_window(window):
    if window is None:
        return
    try:
        window.grab_release()
    except tk.TclError:
        pass
    try:
        if window.winfo_exists():
            window.destroy()
    except tk.TclError:
        pass


def _show_total_error_dialog():
    global total_error_toplevel
    if total_error_toplevel is not None:
        try:
            if total_error_toplevel.winfo_exists():
                return
        except tk.TclError:
            pass

    total_error_toplevel = tk.Toplevel(root)
    total_error_toplevel.title('Error: Homing procedure interrupted.')
    total_error_toplevel.resizable(False, False)
    err_icon = tk.Label(total_error_toplevel, image=err_img_copy)
    err_icon.grid(row=0, column=0, sticky='NSEW', padx=10, pady=10)
    error_text = tk.Label(total_error_toplevel, text="Homing procedure interrupted. Service check needed",
                          font='Verdana 14')
    error_text.grid(row=0, column=1, sticky='NSEW')


def _show_homing_dialog():
    global motor_homing_toplevel, homing_toplevel
    if motor_homing_toplevel:
        return

    homing_toplevel = tk.Toplevel(root)
    homing_toplevel.title('Collimator homing')
    homing_toplevel.resizable(False, False)
    homing_toplevel.protocol("WM_DELETE_WINDOW", disable_close)
    info_icon = tk.Label(homing_toplevel, image=info_img)
    info_icon.grid(row=0, column=0, sticky='NSEW', padx=10, pady=10)
    homing_text = tk.Label(homing_toplevel, text="Motor is homing, wait for finish.", font='Verdana 14')
    homing_text.grid(row=0, column=1, sticky='NSEW')
    motor_homing_toplevel = True
    homing_toplevel.grab_set()


def _hide_homing_dialog():
    global motor_homing_toplevel, homing_toplevel
    _close_window(homing_toplevel)
    homing_toplevel = None
    motor_homing_toplevel = False


def _show_torque_error_dialog():
    global torque_error_toplevel_active, torque_error_toplevel
    if torque_error_toplevel_active:
        return

    torque_error_toplevel_active = True
    torque_error_toplevel = tk.Toplevel(root)
    torque_error_toplevel.title('Collimator blocked')
    torque_error_toplevel.resizable(False, False)
    torque_error_toplevel.protocol("WM_DELETE_WINDOW", disable_close)
    torque_error_toplevel.grab_set()

    error_icon = tk.Label(torque_error_toplevel, image=err_img)
    error_icon.grid(row=0, column=0, rowspan=2, sticky='NSEW', padx=10, pady=10)

    error_label = tk.Label(torque_error_toplevel,
                           text='Collimator is blocked.\nAcknowledge this error to resume operation.',
                           font='Verdana 12')
    error_label.grid(row=0, column=1, sticky='NSEW')
    ack_btn = tk.Button(torque_error_toplevel, text='Acknowledge error', command=command_clear_error,
                        justify='center', font='Verdana 14 bold')
    ack_btn.grid(row=1, column=1, sticky='NSE')


def _hide_torque_error_dialog():
    global torque_error_toplevel_active, torque_error_toplevel
    _close_window(torque_error_toplevel)
    torque_error_toplevel = None
    torque_error_toplevel_active = False


def _apply_application_state(movement_enabld, torque_error, homed, total_error):
    global total_error_prev_state

    set_movement_enable_state(movement_enabld)

    if total_error and not total_error_prev_state:
        _show_total_error_dialog()

    if not homed:
        _show_homing_dialog()
    else:
        _hide_homing_dialog()

    if torque_error:
        _show_torque_error_dialog()
    else:
        _hide_torque_error_dialog()

    total_error_prev_state = total_error

def start_application_handler_thread():
    thrd = threading.Thread(target=application_handler_thread, daemon=True)
    thrd.start()


def application_handler_thread():
    while True:
        if serial_handler.is_open:
            bytes = construct_message(HeaderId.HELLO_MSG_e, [])
            serial_handler.new_transaction(bytes, priority=0, callback=application_data_recieved)
            pass
        sleep(0.35)
    pass


def application_data_recieved(data):
    global motor_homing_toplevel, homing_toplevel, torque_error_toplevel, torque_error_toplevel_active
    global total_error_prev_state, total_error_toplevel
    global err_img, info_img, err_img_copy

    try:
        reconstructed = deconstruct_message(data)
        payload_data = struct.unpack('>B', reconstructed.payload)
        movement_enabld = bool(payload_data[0] & 1)
        torque_error = bool(payload_data[0] & (1 << 2))
        homed = bool(payload_data[0] & (1 << 3))
        total_error = bool(payload_data[0] & (1 << 4))
        _run_on_ui_thread(_apply_application_state, movement_enabld, torque_error, homed, total_error)

        set_transaction_lock(False)
    except Exception as e:
        print(f'[HELLO] Response error: {e}, raw data ({len(data)}B): {data.hex() if data else "empty"}')
        set_transaction_lock(True)


def clear_error_transaction_callback(data):
    try:
        resp_dec = deconstruct_message(data)
        print(resp_dec.payload)
        if resp_dec.payload[0] == 0x0:
            _run_on_ui_thread(_hide_torque_error_dialog)
    except Exception as e:
        print(f'[CLEAR_ERR] Response error: {e}, raw data ({len(data)}B): {data.hex() if data else "empty"}')


def command_clear_error():
    data = struct.pack('>B', RESET_ERROR_FLAGS_e)
    bytes = construct_message(HeaderId.COMMAND_e, data)
    serial_handler.new_transaction(bytes, priority=0, callback=clear_error_transaction_callback)
    print('Clearing')
    pass


def disable_close():
    pass
