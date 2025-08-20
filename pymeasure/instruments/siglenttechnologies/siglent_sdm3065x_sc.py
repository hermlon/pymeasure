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
from enum import StrEnum

from pymeasure.instruments import Instrument
from pymeasure.instruments.channel import Channel
from pymeasure.instruments.validators import (
    strict_discrete_range,
    strict_discrete_set,
)
from pymeasure.units import ureg

from pymeasure.instruments.siglenttechnologies.siglent_sdm3065x import SDM3065X, OnOff


class LoopMode(StrEnum):
    SCAN = "SCAN"
    STEP = "STEP"


class CycleMode(StrEnum):
    AUTO = "ON"
    MANUAL = "OFF"


class VoltageChannelModeVoltage(StrEnum):
    VOLTAGE_DC = "DCV"
    VOLTAGE_AC = "ACV"
    FREQUENCY = "FRQ"


class VoltageChannelModeResistance(StrEnum):
    R2WIRE = "2W"
    R4WIRE = "4W"


class VoltageChannelModeCapacitance(StrEnum):
    CAPACITANCE = "CAP"


class VoltageChannelMode(StrEnum):
    CONTINUITY = "CONT"
    DIODE = "DIO"
    TEMPERATURE = "TEMP"


class CurrentChannelMode(StrEnum):
    CURRENT_DC = "DCI"
    CURRENT_AC = "ACI"


class VoltageRange(StrEnum):
    AUTO = "AUTO"
    V_200ML = "200MLV"
    V_2 = "2V"
    V_20 = "20V"
    V_200 = "200V"


class ResistanceRange(StrEnum):
    AUTO = "AUTO"
    OHM_200 = "200OHM"
    OHM_2K = "2KOHM"
    OHM_20K = "20KOHM"
    OHM_200K = "200KOHM"
    OHM_2MG = "2MGOHM"
    OHM_10MG = "10MGOHM"
    OHM_100MG = "100MGOHM"


class CapacitanceRange(StrEnum):
    AUTO = "AUTO"
    F_2N = "2NF"
    F_20N = "20NF"
    F_200N = "200NF"
    F_2U = "2UF"
    F_20U = "20UF"
    F_200U = "200UF"
    F_10000U = "10000UF"


class CurrentRange(StrEnum):
    A_2 = "2A"


class ChannelSpeed(StrEnum):
    SLOW = "SLOW"
    FAST = "FAST"


class ChannelConfig:
    def __init__(
        self,
        switch: bool,
        mode: VoltageChannelMode
        | VoltageChannelModeVoltage
        | VoltageChannelModeResistance
        | VoltageChannelModeCapacitance
        | CurrentChannelMode,
        range: VoltageRange | ResistanceRange | CapacitanceRange | CurrentRange | None = None,
        speed: ChannelSpeed | None = None,
        count=1,
    ):
        self._switch = switch
        self._mode = mode
        self._range = range
        self._speed = speed
        self._count = count
        self.validate()

    def validate(self):
        pass

    def __str__(self):
        # when mode is VoltageChannelMode
        if self._range is None and self._speed is None:
            assert self._mode in VoltageChannelMode
            return ",".join([OnOff(self._switch), self._mode, str(self._count)])
        else:
            return ",".join(
                [OnOff(self._switch), self._mode, self._range, self._speed, str(self._count)]
            )


class VoltageChannelConfig(ChannelConfig):
    def __init__(
        self,
        switch: bool,
        mode: VoltageChannelMode
        | VoltageChannelModeVoltage
        | VoltageChannelModeResistance
        | VoltageChannelModeCapacitance,
        range: VoltageRange | ResistanceRange | CapacitanceRange | None = None,
        speed: ChannelSpeed | None = None,
        count=1,
    ):
        super().__init__(switch, mode, range, speed, count)

    def validate(self):
        assert (
            (self._mode in VoltageChannelMode and self._range is None and self._speed is None)
            or (
                self._mode in VoltageChannelModeVoltage
                and self._range in VoltageRange
                and self._speed in ChannelSpeed
            )
            or (
                self._mode in VoltageChannelModeResistance
                and self._range in ResistanceRange
                and self._speed in ChannelSpeed
            )
            or (
                self._mode in VoltageChannelModeCapacitance
                and self._range in CapacitanceRange
                and self._speed in ChannelSpeed
            )
            or (
                self._mode in CurrentChannelMode
                and self._range in CurrentRange
                and self._speed in ChannelSpeed
            )
        )

    def __repr__(self):
        return (
            f"VoltageChannelConfig(switch={self._switch!r}, mode={self._mode!r},"
            + f" range={self._range!r}, speed={self._speed!r}, count={self._count!r})"
        )

    @classmethod
    def from_list(cls, lst):
        switch, mode, range, speed, count = lst
        if mode in VoltageChannelMode:
            return ChannelConfig(OnOff(switch), VoltageChannelMode(mode), count=int(count))
        else:
            if mode in VoltageChannelModeVoltage:
                enum_mode, enum_range = VoltageChannelModeVoltage(mode), VoltageRange(range)
            if mode in VoltageChannelModeResistance:
                enum_mode, enum_range = VoltageChannelModeResistance(mode), ResistanceRange(range)
            if mode in VoltageChannelModeCapacitance:
                enum_mode, enum_range = VoltageChannelModeCapacitance(mode), CapacitanceRange(range)
            return ChannelConfig(
                OnOff(switch),
                enum_mode,
                enum_range,
                ChannelSpeed(speed),
                int(count),
            )


class CurrentChannelConfig(ChannelConfig):
    def __init__(
        self,
        switch: bool,
        mode: CurrentChannelMode,
        range: CurrentRange,
        speed: ChannelSpeed,
        count=1,
    ):
        super().__init__(switch, mode, range, speed, count)

    def validate(self):
        assert (
            self._mode in CurrentChannelMode
            and self._range in CurrentRange
            and self._speed in ChannelSpeed
        )

    def __repr__(self):
        return (
            f"CurrentChannelConfig(switch={self._switch!r}, mode={self._mode!r},"
            + f" range={self._range!r}, speed={self._speed!r}, count={self._count!r})"
        )

    @classmethod
    def from_list(cls, lst):
        switch, mode, range, speed, count = lst
        return ChannelConfig(
            OnOff(switch),
            CurrentChannelMode(mode),
            CurrentRange(range),
            ChannelSpeed(speed),
            int(count),
        )


class VoltageChannel(Channel):
    voltage = Channel.measurement(
        "ROUTe:DATA? {ch}",
        """Measure a DC or AC voltage in Volt.""",
        preprocess_reply=lambda v: v.split(" ")[0],
        get_process=lambda v: ureg.Quantity(v, ureg.V),
    )

    config = Channel.control(
        "ROUTe:CHANnel? {ch}",
        "ROUTe:CHANnel {ch},%s",
        """""",
        get_process_list=lambda v: VoltageChannelConfig.from_list(v[1:]),
        check_set_errors=True,
    )


class CurrentChannel(Channel):
    current = Channel.measurement(
        "ROUTe:DATA? {ch}",
        """Measure a DC or AC current in Ampere.""",
        preprocess_reply=lambda v: v.split(" ")[0],
        get_process=lambda v: ureg.Quantity(v, ureg.A),
    )

    config = Channel.control(
        "ROUTe:CHANnel? {ch}",
        "ROUTe:CHANnel {ch},%s",
        """""",
        get_process_list=lambda v: CurrentChannelConfig.from_list(v[1:]),
        check_set_errors=True,
    )


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

    ch_1 = Instrument.ChannelCreator(VoltageChannel, "1")
    ch_2 = Instrument.ChannelCreator(VoltageChannel, "2")
    ch_3 = Instrument.ChannelCreator(VoltageChannel, "3")
    ch_4 = Instrument.ChannelCreator(VoltageChannel, "4")
    ch_5 = Instrument.ChannelCreator(VoltageChannel, "5")
    ch_6 = Instrument.ChannelCreator(VoltageChannel, "6")
    ch_7 = Instrument.ChannelCreator(VoltageChannel, "7")
    ch_8 = Instrument.ChannelCreator(VoltageChannel, "8")
    ch_9 = Instrument.ChannelCreator(VoltageChannel, "9")
    ch_10 = Instrument.ChannelCreator(VoltageChannel, "10")
    ch_11 = Instrument.ChannelCreator(VoltageChannel, "11")
    ch_12 = Instrument.ChannelCreator(VoltageChannel, "12")
    ch_13 = Instrument.ChannelCreator(CurrentChannel, "13")
    ch_14 = Instrument.ChannelCreator(CurrentChannel, "14")
    ch_15 = Instrument.ChannelCreator(CurrentChannel, "15")
    ch_16 = Instrument.ChannelCreator(CurrentChannel, "16")

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
