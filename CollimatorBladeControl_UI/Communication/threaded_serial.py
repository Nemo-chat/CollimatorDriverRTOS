import serial
import threading
import queue
import time


class SerialInterface:
    serial_transaction_timeout = 0.5

    def __init__(self, serial_port: str, serial_baud: int):
        self.serial_port = serial_port
        self.serial_baud = serial_baud
        self.serial_if = serial.Serial()
        self.serial_if.timeout = self.serial_transaction_timeout
        self.serial_if.inter_byte_timeout = 0.05
        self.is_open = False
        self._counter = 0
        self._tx_log_callbacks = []
        self._rx_log_callbacks = []

        self.transaction_queue = queue.PriorityQueue(maxsize=100)
        self.transaction_thread = threading.Thread(target=self.__interface_handler_thread__, daemon=True)
        self.transaction_thread.start()

    def add_log_callback(self, tx_cb=None, rx_cb=None):
        if tx_cb is not None:
            self._tx_log_callbacks.append(tx_cb)
        if rx_cb is not None:
            self._rx_log_callbacks.append(rx_cb)

    def connect(self):
        self.serial_if.port = self.serial_port
        self.serial_if.baudrate = self.serial_baud

        try:
            if self.serial_if.is_open:
                self.serial_if.close()
            self.serial_if.open()
        except serial.SerialException:
            return False

        self.is_open = True
        return True

    def new_transaction(self, data: list, priority=0, callback=None):
        if (len(data) == 0) or self.transaction_queue.full() or self.is_open is False:
            return False
        loc_data = data.copy()
        transaction_data = (loc_data, callback)
        self.transaction_queue.put((priority, self._counter, transaction_data))
        self._counter += 1
        return True

    def __interface_handler_thread__(self):
        while True:
            if self.transaction_queue.qsize() != 0:
                (prio, _cnt, transaction) = self.transaction_queue.get()

                try:
                    for cb in self._tx_log_callbacks:
                        try:
                            cb(bytes(transaction[0]))
                        except Exception:
                            pass

                    self.serial_if.reset_input_buffer()
                    self.serial_if.write(transaction[0])
                    response = self.serial_if.read(256)

                    for cb in self._rx_log_callbacks:
                        try:
                            cb(bytes(response))
                        except Exception:
                            pass

                    if transaction[1] is not None:
                        transaction[1](response)
                except Exception as e:
                    print(f'[SERIAL] Thread error: {e}')
                time.sleep(0.05)
            time.sleep(0.01)

    def flush_transactions(self):
        while self.transaction_queue.qsize() != 0:
            self.transaction_queue.get()


if __name__ == '__main__':
    q = queue.PriorityQueue(maxsize=10)
    q.put((1, '1'))
    q.put((0, '0'))
    print(q.get())
    print(q.get())
    pass