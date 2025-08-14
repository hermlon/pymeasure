#
# This file is part of the PyMeasure package.
#
# Copyright (c) 2013-2025 PyMeasure Developers
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#
import logging
from enum import StrEnum
from pymeasure.instruments import Instrument, SCPIMixin
from pymeasure.instruments.channel import Channel
from pymeasure.instruments.validators import (
    strict_discrete_range,
    strict_discrete_set,
    truncated_range,
)
from pymeasure.units import ureg


class Mode(StrEnum):
    CURRENT_AC = "CURR:AC"
    CURRENT_DC = "CURR"
    VOLTAGE_AC = "VOLT:AC"
    VOLTAGE_DC = "VOLT"


class OnOff(StrEnum):
    ON = "ON"
    OFF = "OFF"

    def __bool__(self):
        return self == OnOff.ON

    @classmethod
    def _missing_(cls, value):
        return OnOff.ON if value else OnOff.OFF


class SDM3065X(SCPIMixin, Instrument):
    """Siglent SDM3065X Digital Multimeter."""

    def __init__(self, adapter, name="Siglent SDM3065X Digital Multimeter", **kwargs):
        super().__init__(
            adapter,
            name,
            timeout=5000,  # set timeout to 5 seconds
            **kwargs,
        )

    mode = Instrument.control(
        "CONF?",
        "CONF:%s",
        """Control the measurement mode of the multimeter.
        
        Allowed modes are Mode.CURRENT_AC, Mode.CURREND_DC.""",
        validator=strict_discrete_set,
        map_values=True,
        values={u: u.value for u in Mode},
        preprocess_reply=lambda v: v[1:].split(" ")[0],
        check_set_errors=True,
    )

    current = Instrument.measurement(
        "READ?",
        """Measure a DC or AC current in Ampere, based on the active :attr:`mode`.""",
        get_process=lambda v: ureg.Quantity(v, ureg.A),
    )

    voltage = Instrument.measurement(
        "READ?",
        """Measure a DC or AC voltage in Volt, based on the active :attr:`mode`.""",
        get_process=lambda v: ureg.Quantity(v, ureg.V),
    )
