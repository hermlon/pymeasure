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
from pymeasure.instruments import Instrument, SCPIMixin
from pymeasure.instruments.validators import strict_discrete_set
from pymeasure.units import ureg

try:
    from enum import StrEnum
except ImportError:
    from enum import Enum

    class StrEnum(str, Enum):
        """Until StrEnum is broadly available from the standard library"""

        # Python>3.10 remove it.


class StaticMode(StrEnum):
    CURRENT = "CURRENT"
    VOLTAGE = "VOLTAGE"
    POWER = "POWER"
    RESISTANCE = "RESISTANCE"
    LED = "LED"


class SDL1000X(SCPIMixin, Instrument):
    """Siglent SDL1000X DC Electronic Load."""

    def __init__(self, adapter, name="Siglent SDL1000X DC Electronic Load", **kwargs):
        super().__init__(
            adapter,
            name,
            timeout=15000,  # set timeout to 15 seconds, *RST takes pretty long
            **kwargs,
        )

    enable = Instrument.control(
        "SOURce:INPut:STATe?",
        "SOURce:INPut:STATe %s",
        """
        Control the input status of the load.
        """,
        set_process=lambda v: int(v),
        get_process=lambda v: bool(v),
        check_set_errors=False,
    )

    static_mode = Instrument.control(
        "SOURce:FUNCtion?",
        "SOURce:FUNCtion %s",
        """Control mode in static operation.""",
        validator=strict_discrete_set,
        map_values=True,
        values={u: u.value for u in StaticMode},
        check_set_errors=True,
    )

    current = Instrument.control(
        "SOURce:CURRent:LEVel:IMMediate?",
        "SOURce:CURRent:LEVel:IMMediate %s",
        """Control the sink current value of CC mode in static operation.""",
        set_process=lambda v: v.m_as(ureg.A),
        get_process=lambda v: ureg.Quantity(v, ureg.A),
    )

    current_irange = Instrument.control(
        "SOURce:CURRent:IRANGe?",
        "SOURce:CURRent:IRANGe %s",
        """Control the current range of CC mode in static operation.
        5 or 30 ampere""",
        validator=strict_discrete_set,
        values=[5, 30],
        check_set_errors=True,
    )

    current_vrange = Instrument.control(
        "SOURce:CURRent:VRANGe?",
        "SOURce:CURRent:VRANGe %s",
        """Control the voltage range of CC mode in static operation.
        36 or 150 volt""",
        validator=strict_discrete_set,
        values=[36, 150],
        check_set_errors=True,
    )

    current_slope_rise = Instrument.control(
        "SOURce:CURRent:SLEW:POSitive?",
        "SOURce:CURRent:SLEW:POSitive %s",
        """Control the rise slope of CC mode in static operation in ampere per microsecond.""",
        check_set_errors=True,
    )

    current_slope_fall = Instrument.control(
        "SOURce:CURRent:SLEW:NEGative?",
        "SOURce:CURRent:SLEW:NEGative %s",
        """Control the falling slope of CC mode in static operation in ampere per microsecond.""",
        check_set_errors=True,
    )
