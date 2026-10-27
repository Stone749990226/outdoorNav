"""Read-only serial acquisition shared by the input collectors; never writes commands."""

import os
import pty
import termios

import pytest

from outdoornav.serial_input import SerialInput


def test_read_only_serial_exclusion_and_settings_restore():
    master,slave=pty.openpty()
    path=os.ttyname(slave)
    attrs=termios.tcgetattr(slave)
    os.close(slave)
    try:
        with SerialInput(path,115200) as reader:
            os.write(master,b'$HEADING,read-only\r\n')
            assert reader.read(1)==b'$HEADING,read-only\r\n'
            with pytest.raises(OSError):os.open(path,os.O_RDONLY|os.O_NOCTTY|os.O_NONBLOCK)
        check=os.open(path,os.O_RDONLY|os.O_NOCTTY|os.O_NONBLOCK)
        try:assert termios.tcgetattr(check)==attrs
        finally:os.close(check)
        with pytest.raises(ValueError,match='position receiver'):
            SerialInput(path,115200,forbidden=[path])
    finally:os.close(master)
