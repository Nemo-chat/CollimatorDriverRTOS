import time
from tkinter import *
from tkinter import ttk

from App.custom_elements import InputSlider
from App.custom_elements import TwoStateBtn

import struct
from Communication.Protocol import *
from App.global_vars import *
from App import param_tab

app_tab_el = None

position = 0
pos_indicator = None
pos_input = None
start_stop_btn = None
set_position_btn_ref = None
last_pos = 0.0
act_pos_entry_var = StringVar()
pos_indicator_var = DoubleVar()
last_controls_disabled = None
last_svc_mode_state = None

def start():
    data = struct.pack('>BB', SET_MOVEMENT_STATE_e, 1)
    bytes = construct_message(HeaderId.COMMAND_e, data)
    state = serial_handler.new_transaction(bytes, priority=0, callback=None)
    if state is False:
        serial_handler.flush_transactions()
        serial_handler.new_transaction(bytes, priority=0, callback=None)


def stop():
    data = struct.pack('>BB', SET_MOVEMENT_STATE_e, 0)
    bytes = construct_message(HeaderId.COMMAND_e, data)
    state = serial_handler.new_transaction(bytes, priority=0, callback=None)
    if state is False:
        serial_handler.flush_transactions()
        serial_handler.new_transaction(bytes, priority=0, callback=None)


last_foc_btn_state = None

def foc_state_callback(data):
    try:
        resp_dec = deconstruct_message(data)
        if resp_dec.payload[0] == 0x00:  # RESPONSE_OK
            foc_enabled = bool(resp_dec.payload[1])
            set_movement_enable_state(foc_enabled)
            # Schedule UI update on main Tkinter thread
            app_tab_el.after(0, _update_foc_btn_ui)
    except Exception as e:
        print(f'[GET_FOC] Response error: {e}, raw data ({len(data)}B): {data.hex() if data else "empty"}')


def _update_foc_btn_ui():
    global last_foc_btn_state
    foc_enabled = get_movement_enable_state()
    new_btn_state = 0 if foc_enabled else 1
    if new_btn_state != last_foc_btn_state:
        last_foc_btn_state = new_btn_state
        start_stop_btn.overwrite_state(new_btn_state)
    update_controls_state()

def set_position_callback(data):
    try:
        resp_dec = deconstruct_message(data)
    except Exception as e:
        print(f'[SET_POS] Response error: {e}, raw data ({len(data)}B): {data.hex() if data else "empty"}')


def set_position():
    global position
    data = struct.pack('>BI', SET_REFERENCE_POSITION_e, int(position*1000))
    bytes = construct_message(HeaderId.COMMAND_e, data)
    state = serial_handler.new_transaction(bytes, priority=0, callback=set_position_callback)
    if state is False:
        serial_handler.flush_transactions()
        serial_handler.new_transaction(bytes, priority=0, callback=set_position_callback)


def write_position(val):
    global position
    position = val


def write_max_position_callback(data):
    try:
        resp_dec = deconstruct_message(data)
        max_pos = struct.unpack('>I', resp_dec.payload[1:])
        max_pos = float(max_pos[0]) / 1000.0
        max_pos = round(max_pos, 2)
        set_remote_max_position(max_pos)
        pass
    except Exception as e:
        print(f'[MAX_POS] Response error: {e}, raw data ({len(data)}B): {data.hex() if data else "empty"}')


def update_positions():
    global app_tab_el, act_pos_entry_var, last_pos
    max = get_remote_max_position_()
    if max == 0.0:
        data = struct.pack('>B', GET_MAXIMUM_POSITION_e)
        bytes = construct_message(HeaderId.COMMAND_e, data)
        serial_handler.new_transaction(bytes, priority=0, callback=write_max_position_callback)

    curr_pos = get_remote_position()
    act_pos_entry_var.set(str(curr_pos))
    pos_input.slider.config(to=max)
    pos_input.update_limits((0, max))
    pos_indicator.config(to=max)
    pos_indicator_var.set(curr_pos)
    act_pos_entry_var.set(curr_pos)

    last_pos = curr_pos
    app_tab_el.after(200, update_positions)


def update_foc_state():
    # Send GET_FOC_STATE command; button update happens in foc_state_callback
    foc_data = struct.pack('>B', GET_FOC_STATE_e)
    foc_bytes = construct_message(HeaderId.COMMAND_e, foc_data)
    serial_handler.new_transaction(foc_bytes, priority=0, callback=foc_state_callback)

    app_tab_el.after(1000, update_foc_state)


def update_service_mode():
    # Send GET_SERVICE_MODE command
    svc_data = struct.pack('>B', GET_SERVICE_MODE_e)
    svc_bytes = construct_message(HeaderId.COMMAND_e, svc_data)
    serial_handler.new_transaction(svc_bytes, priority=0, callback=service_mode_callback)

    app_tab_el.after(2000, update_service_mode)


def service_mode_callback(data):
    try:
        resp_dec = deconstruct_message(data)
        if resp_dec.payload[0] == 0x00:  # RESPONSE_OK
            svc_active = bool(resp_dec.payload[1])
            set_service_mode_active(svc_active)
            # Schedule UI update on main Tkinter thread
            app_tab_el.after(0, update_controls_state)
    except Exception as e:
        print(f'[GET_SVC] Response error: {e}, raw data ({len(data)}B): {data.hex() if data else "empty"}')


def update_controls_state():
    global last_controls_disabled, last_svc_mode_state
    svc_active = get_service_mode_active()
    # Disable SET controls when service mode is active OR FOC is disabled
    should_disable = svc_active or not get_movement_enable_state()
    if should_disable == last_controls_disabled and svc_active == last_svc_mode_state:
        return
    last_controls_disabled = should_disable
    last_svc_mode_state = svc_active
    new_state = 'disabled' if should_disable else 'normal'
    if set_position_btn_ref is not None:
        set_position_btn_ref.config(state=new_state)
    if start_stop_btn is not None:
        # On/Off button only disabled in service mode, NOT when FOC is off
        svc_state = 'disabled' if svc_active else 'normal'
        start_stop_btn.elmnt.config(state=svc_state)
    if pos_input is not None:
        pos_input.slider.config(state=new_state)
        pos_input.input_box.elmnt.config(state=new_state)
    if param_tab.save_btn_ref is not None:
        param_tab.save_btn_ref.config(state=new_state)
    # Update service mode label
    update_service_mode_label(svc_active)


def update_service_mode_label(svc_active):
    if param_tab.service_mode_label_ref is not None:
        if svc_active:
            param_tab.service_mode_label_ref.config(text='Drive in service mode')
        else:
            param_tab.service_mode_label_ref.config(text='')


def application_ctrl_tab(root):
    global app_tab_el, pos_indicator, pos_input, act_pos_entry_var, start_stop_btn, pos_indicator_var, set_position_btn_ref
    app_tab_el = ttk.LabelFrame(root, text='Position control')
    app_tab_el.columnconfigure(0, weight=1)
    app_tab_el.after(100, update_positions)
    app_tab_el.after(1000, update_foc_state)
    app_tab_el.after(2000, update_service_mode)

    pos_input = InputSlider(app_tab_el, limits=(0, 0), callback=write_position)
    pos_input.elmnt.grid(row=0, column=0, sticky='NSEW')

    set_position_btn_ref = Button(app_tab_el, text='Set', command=set_position)
    set_position_btn_ref.grid(row=0, column=1, sticky='NSEW')

    start_stop_btn = TwoStateBtn(app_tab_el, callbacks=(start, stop), default_state=1,
                                     colors=(('#cc3333', 'white'), ('#33aa33', 'white')))
    start_stop_btn.elmnt.grid(row=0, column=2, sticky='NSEW')

    pos_indicator = ttk.Scale(pos_input.elmnt, orient=HORIZONTAL, state='disabled', variable=pos_indicator_var)
    pos_indicator.grid(row=1, column=0, sticky='NSEW')

    act_pos_entry = Entry(pos_input.elmnt, state='readonly', textvariable=act_pos_entry_var, justify='center', width= 8)
    act_pos_entry.configure(font='Verdana 14')
    act_pos_entry.grid(row=1, column=1, sticky='NSEW')
    return app_tab_el
