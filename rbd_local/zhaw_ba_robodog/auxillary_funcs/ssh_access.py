from paramiko import SSHClient, AutoAddPolicy
import subprocess

host_ip = '192.168.123.12'
#host_ip = 'localhost'
username = 'unitree'
password = '123'


def check_connection():
    command = ['ping', '-c', '1', host_ip]
    return subprocess.call(command) == 0


def get_ssh_connection():
    c = SSHClient()
    c.set_missing_host_key_policy(AutoAddPolicy())
    c.connect(hostname=host_ip, username=username, password=password, )
    return c


class NoConnectionError(Exception):
    def __init__(self, message, *args: object) -> None:
        super().__init__(*args)
        self.message = message
