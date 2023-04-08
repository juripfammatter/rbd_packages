from time import sleep
from multiprocessing import Process
from threading import Thread, Event
import sys

from auxillary_funcs.ssh_access import get_ssh_connection, check_connection, NoConnectionError
from auxillary_funcs.interprocess_comms import get_conn_listener, get_conn_client, COMMAND_PORT

# commands for usage without ROS
"""
COMMAND_SIT = './sit.sh'
COMMAND_STAND = 'rise'
COMMAND_WALK = './walk.sh'
COMMAND_WIGGLE = './wiggle.sh'
COMMAND_SPIN = './spin.sh'
COMMAND_DOWN = './lie_down.sh'

_COMMAND_STOP = 'stop'
"""

# commands for ROS
COMMAND_SIT = 'sit'
COMMAND_STAND = 'rise'
COMMAND_WALK = 'walk'
COMMAND_WIGGLE = 'wiggle'
COMMAND_SPIN = 'spin'
COMMAND_DOWN = 'lie_down'
_COMMAND_STOP = 'stop'


class CommandExecutor():

    def start(self):
        if check_connection():
            proc = Process(target=self.__start_listener__)
            proc.start()
            sleep(1)
            return proc
        else:
            raise NoConnectionError(message='Connection times out. Please check if Wifi is connected.')

    def __sit_func__(self):
        c = get_ssh_connection()
        c.exec_command(COMMAND_SIT)
        self._sit_stop.wait()
        c.close()

    def __sit__(self):
        if not self._sit_thrd.is_alive():
            self._sit_stop.clear()
            self._sit_thrd = Thread(target=self.__sit_func__)
            self._sit_thrd.daemon = True
            self._sit_thrd.start()

    def __stand__(self):
        if self._sit_thrd.is_alive():
            self._sit_stop.set()

    def __exec_comnd__(self, command):
        self.__stand__()
        sleep(2)
        c = get_ssh_connection()
        _, stdout, _ = c.exec_command(command)
        stdout.channel.recv_exit_status()
        c.close()
        sleep(1)
        self.__sit__()
        sleep(3)

    def __start_listener__(self):
        listener = get_conn_listener(COMMAND_PORT)
        self._cmd_thrd = Thread()
        self._sit_thrd = Thread()
        self._sit_stop = Event()
        while True:
            con = listener.accept()
            msg = con.recv()
            if msg == _COMMAND_STOP:
                sys.exit()
            elif msg == COMMAND_SIT:
                self.__sit__()
            elif msg == COMMAND_STAND:
                self.__stand__()
            elif (not self._cmd_thrd.is_alive()) and (self._sit_thrd.is_alive()):
                self._cmd_thrd = Thread(target=self.__exec_comnd__, args=(msg,))
                self._cmd_thrd.daemon = True
                self._cmd_thrd.start()


def execute_command(command):
    con = get_conn_client(COMMAND_PORT)
    con.send(command)
    con.close()


def commandexec_stop():
    execute_command(_COMMAND_STOP)
