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

from pymeasure.instruments.siglenttechnologies.siglent_sdm3065x import SDM3065X, OnOff


class LoopMode(StrEnum):
    SCAN = "SCAN"
    STEP = "STEP"


class CycleMode(StrEnum):
    AUTO = "ON"
    MANUAL = "OFF"


class SDM3065XSC(SDM3065X):
    """Siglent SDM3065X-SC Digital Multimeter with scan card installed."""

    def __init__(
        self, adapter, name="Siglent SDM3065X-SC Digital Multimeter with scan card", **kwargs
    ):
        super().__init__(
            adapter,
            name,
            **kwargs,
        )
        assert self.sc_installed

    sc_installed = Instrument.measurement(
        "ROUTe:STATe?",
        """Query whether the SDM has a scan card installed.""",
        get_process=lambda v: bool(OnOff(v)),
    )

    sc_enabled = Instrument.control(
        "ROUTe:SCAN?",
        "ROUTe:SCAN %s",
        """Control the the scanning card function.""",
        set_process=lambda v: OnOff(v).value,
        get_process=lambda v: bool(OnOff(v)),
        check_set_errors=True,
    )

    sc_start = Instrument.control(
        "ROUTe:START?",
        "ROUTe:START %s",
        """Start or stop the scan card measurement""",
        set_process=lambda v: OnOff(v).value,
        get_process=lambda v: bool(OnOff(v)),
        check_set_errors=True,
    )

    sc_loop_mode = Instrument.control(
        "ROUTe:FUNCtion?",
        "ROUTe:FUNCtion %s",
        """Control the scan card loop mode.

        LoopMode.SCAN: measure all specified channels once, then wait for specified sc_delay time \
            before repeating
        LoopMode.STEP: wait specified sc_delay time after mesuring each channel
        """,
        validator=strict_discrete_set,
        map_values=True,
        values={u: u.value for u in LoopMode},
        check_set_errors=True,
    )

    sc_delay = Instrument.control(
        "ROUTe:DELay?",
        "ROUTe:DELay %s",
        """Control the scan card delay time in seconds.
        Depending on the sc_loop_mode, this delay be applied after each channel \
            (LoopMode.STEP) or after scanning all channels (LoopMode.SCAN)
        """,
        check_set_errors=True,
    )

    sc_cycle_mode = Instrument.control(
        "ROUTe:COUNt:AUTO?",
        "ROUTe:COUNt:AUTO %s",
        """Control the scan card cycle mode.
        
        CycleMode.AUTO: measure values when receiving sc_start signal until stop
        CycleMode.MANUAL: when receiving sc_start signal, stop after doing sc_count measurements \
            of all channels
        """,
        validator=strict_discrete_set,
        map_values=True,
        values={u: u.value for u in CycleMode},
        check_set_errors=True,
    )

    sc_count = Instrument.control(
        "ROUTe:COUNt?",
        "ROUTe:COUNt %s",
        """Control number of times to repeat scanning all channels after sc_start was sent.
        This setting is only relevant when sc_cycle_mode is CycleMode.MANUAL, as in CycleMode.AUTO \
            measurements run indefinitely until sc_start is disabled.
        """,
        validator=lambda v, vs: strict_discrete_range(v, vs, step=1),
        values=range(1, 1000),
        check_set_errors=True,
    )

    sc_ch_limit_low = Instrument.control(
        "ROUTe:LIMIt:LOW?",
        "ROUTe:LIMIt:LOW %s",
        """Control lower channel limit of channel numbers to be scanned""",
        validator=lambda v, vs: strict_discrete_range(v, vs, step=1),
        values=range(1, 17),
        set_process=lambda v: int(v),
        check_set_errors=True,
    )

    sc_ch_limit_high = Instrument.control(
        "ROUTe:LIMIt:HIGH?",
        "ROUTe:LIMIt:HIGH %s",
        """Control upper channel limit of channel numbers to be scanned""",
        validator=lambda v, vs: strict_discrete_range(v, vs, step=1),
        values=range(1, 17),
        set_process=lambda v: int(v),
        check_set_errors=True,
    )
