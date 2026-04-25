from tkinter import *
from tkinter import ttk
from tkinter import messagebox
from App.custom_elements import ParamField

import struct
from Communication.Protocol import *
from App.global_vars import serial_handler
from App.global_vars import get_transaction_lock
from App.global_vars import set_remote_position

data_disp_el = None

parameters_strings = {
    'speed': None,
    'pos': None,
    'rotor_pos': None,
    'id': None,
    'iq': None,
    'link_voltage': None
}


def _run_on_ui_thread(callback, *args):
    data_disp_el.after(0, lambda: callback(*args))


def _apply_mechanical_data(speed, position, rotor_pos):
    parameters_strings['speed'].set(str(speed))
    parameters_strings['pos'].set(str(position))
    parameters_strings['rotor_pos'].set(str(rotor_pos))
    set_remote_position(position)


def _apply_electrical_data(curr_d, curr_q, link_voltage):
    parameters_strings['id'].set(str(curr_d))
    parameters_strings['iq'].set(str(curr_q))
    parameters_strings['link_voltage'].set(str(link_voltage))


def write_new_mech_data(data):
    try:
        resp_dec = deconstruct_message(data)
        loc_parameters = struct.unpack('>III', resp_dec.payload[1:])
        _run_on_ui_thread(_apply_mechanical_data,
                          loc_parameters[0] / 1000,
                          loc_parameters[1] / 1000,
                          loc_parameters[2] / 1000)
    except Exception as e:
        print(f'[MECH_DATA] Response error: {e}, raw data ({len(data)}B): {data.hex() if data else "empty"}')


def write_new_electrical_data(data):
    try:
        resp_dec = deconstruct_message(data)
        loc_parameters = struct.unpack('>III', resp_dec.payload[1:])
        _run_on_ui_thread(_apply_electrical_data,
                          loc_parameters[0] / 1000,
                          loc_parameters[1] / 1000,
                          loc_parameters[2] / 1000)
    except Exception as e:
        print(f'[ELEC_DATA] Response error: {e}, raw data ({len(data)}B): {data.hex() if data else "empty"}')


def update_data_electrical():
    if get_transaction_lock() is False:
        data = struct.pack('>B', GET_ELECTRICAL_DATA_e)
        bytes = construct_message(HeaderId.COMMAND_e, data)
        serial_handler.new_transaction(bytes, priority=2, callback=write_new_electrical_data)
    data_disp_el.after(200, update_data)


def update_data():
    if get_transaction_lock() is False:
        data = struct.pack('>B', GET_MECHANICAL_DATA_e)
        bytes = construct_message(HeaderId.COMMAND_e, data)
        serial_handler.new_transaction(bytes, priority=2, callback=write_new_mech_data)
    data_disp_el.after(200, update_data_electrical)


def data_display(root):
    global data_disp_el
    data_disp_el = ttk.LabelFrame(root, text='Data', width=350)
    data_disp_el.after(200, update_data)

    for key in parameters_strings:
        parameters_strings[key] = StringVar()
        parameters_strings[key].set('No data')

    # Mechanical data
    mech_data_frame = ttk.LabelFrame(data_disp_el, text='Mechanical')
    mech_data_frame.columnconfigure(0, weight=1)

    speed_data = ParamField(mech_data_frame, 'Speed [rad/s]', entry_state='readonly',
                            variable=parameters_strings['speed'])
    speed_data.elmnt.grid(row=0, column=0, sticky='NSEW')

    # position = ParamField(mech_data_frame, 'Position [mm]', entry_state='readonly',
    #                       variable=parameters_strings['pos'])
    # position.elmnt.grid(row=1, column=0, sticky='NSEW')

    rotor_position = ParamField(mech_data_frame, 'Rotor position [rad]', entry_state='readonly',
                                variable=parameters_strings['rotor_pos'])
    rotor_position.elmnt.grid(row=2, column=0, sticky='NSEW')

    mech_data_frame.grid(row=0, column=0, sticky='NSEW')

    # Electrical data
    el_data_frame = ttk.LabelFrame(data_disp_el, text='Electrical')
    el_data_frame.columnconfigure(0, weight=1)

    iq_data = ParamField(el_data_frame, 'id [A]', entry_state='readonly', variable=parameters_strings['id'])
    iq_data.elmnt.grid(row=0, column=0, sticky='NSEW')

    id_data = ParamField(el_data_frame, 'iq [A]', entry_state='readonly', variable=parameters_strings['iq'])
    id_data.elmnt.grid(row=1, column=0, sticky='NSEW')

    dc_link_voltage = iq_data = ParamField(el_data_frame, 'Link voltage [V]', entry_state='readonly',
                                           variable=parameters_strings['link_voltage'])
    dc_link_voltage.elmnt.grid(row=2, column=0, sticky='NSEW')

    el_data_frame.grid(row=1, column=0, sticky='NSEW')

    return data_disp_el
