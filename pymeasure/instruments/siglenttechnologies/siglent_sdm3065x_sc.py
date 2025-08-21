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
import time

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


class VoltageChannelModeResistance(StrEnum):
    R2WIRE = "2W"
    R4WIRE = "4W"


class VoltageChannelModeFrequency(StrEnum):
    FREQUENCY = "FRQ"


class VoltageChannelModeCapacitance(StrEnum):
    CAPACITANCE = "CAP"


class VoltageChannelModeNoRange(StrEnum):
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
    OHM_1MG = "1MGOHM"
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
    F_2M = "2MF"
    F_20M = "20MF"
    F_100M = "100MF"


class CurrentRange(StrEnum):
    A_2 = "2A"


class ChannelSpeed(StrEnum):
    SLOW = "SLOW"
    FAST = "FAST"


class ChannelConfig:
    def __init__(
        self,
        switch: bool,
        mode: VoltageChannelModeNoRange
        | VoltageChannelModeVoltage
        | VoltageChannelModeResistance
        | VoltageChannelModeCapacitance
        | VoltageChannelModeFrequency
        | CurrentChannelMode,
        mrange: VoltageRange | ResistanceRange | CapacitanceRange | CurrentRange | None = None,
        speed: ChannelSpeed | None = None,
        count=1,
    ):
        self._switch = switch
        self._mode = mode
        self._mrange = mrange
        self._speed = speed
        self._count = count
        self.validate()

    def validate(self):
        pass

    def __str__(self):
        # for VoltageChannelMode
        if self._mrange is None and self._speed is None:
            assert self._mode in VoltageChannelModeNoRange
            return ",".join([OnOff(self._switch), self._mode, str(self._count)])
        # for VoltageChannelModeCapacitance and VoltageChannelModeFrequency
        elif self._speed is None:
            return ",".join([OnOff(self._switch), self._mode, self._mrange, str(self._count)])
        else:
            return ",".join(
                [OnOff(self._switch), self._mode, self._mrange, self._speed, str(self._count)]
            )


class VoltageChannelConfig(ChannelConfig):
    def __init__(
        self,
        switch: bool,
        mode: VoltageChannelModeNoRange
        | VoltageChannelModeVoltage
        | VoltageChannelModeResistance
        | VoltageChannelModeCapacitance
        | VoltageChannelModeFrequency,
        mrange: VoltageRange | ResistanceRange | CapacitanceRange | None = None,
        speed: ChannelSpeed | None = None,
        count=1,
    ):
        super().__init__(switch, mode, mrange, speed, count)

    def validate(self):
        assert (
            (
                self._mode in VoltageChannelModeNoRange
                and self._mrange is None
                and self._speed is None
            )
            or (
                self._mode in VoltageChannelModeVoltage
                and self._mrange in VoltageRange
                and self._speed in ChannelSpeed
            )
            or (
                self._mode in VoltageChannelModeResistance
                and self._mrange in ResistanceRange
                and self._speed in ChannelSpeed
            )
            or (
                self._mode in VoltageChannelModeCapacitance
                and self._mrange in CapacitanceRange
                and self._speed is None
            )
            or (
                self._mode in VoltageChannelModeFrequency
                and self._mrange in VoltageRange
                and self._speed is None
            )
            or (
                self._mode in CurrentChannelMode
                and self._mrange in CurrentRange
                and self._speed in ChannelSpeed
            )
        )

    def __repr__(self):
        return (
            f"VoltageChannelConfig(switch={self._switch!r}, mode={self._mode!r},"
            + f" mrange={self._mrange!r}, speed={self._speed!r}, count={self._count!r})"
        )

    @classmethod
    def from_list(cls, lst):
        switch, mode, mrange, speed, count = lst
        if mode in VoltageChannelModeNoRange:
            return ChannelConfig(OnOff(switch), VoltageChannelModeNoRange(mode), count=int(count))
        else:
            if mode in VoltageChannelModeVoltage:
                enum_mode, enum_mrange, enum_speed = (
                    VoltageChannelModeVoltage(mode),
                    VoltageRange(mrange),
                    ChannelSpeed(speed),
                )
            if mode in VoltageChannelModeResistance:
                enum_mode, enum_mrange, enum_speed = (
                    VoltageChannelModeResistance(mode),
                    ResistanceRange(mrange),
                    ChannelSpeed(speed),
                )
            if mode in VoltageChannelModeCapacitance:
                enum_mode, enum_mrange, enum_speed = (
                    VoltageChannelModeCapacitance(mode),
                    CapacitanceRange(mrange),
                    None,
                )
            if mode in VoltageChannelModeFrequency:
                enum_mode, enum_mrange, enum_speed = (
                    VoltageChannelModeFrequency(mode),
                    VoltageRange(mrange),
                    None,
                )
            return ChannelConfig(
                OnOff(switch),
                enum_mode,
                enum_mrange,
                enum_speed,
                int(count),
            )


class CurrentChannelConfig(ChannelConfig):
    def __init__(
        self,
        switch: bool,
        mode: CurrentChannelMode,
        mrange: CurrentRange,
        speed: ChannelSpeed,
        count=1,
    ):
        super().__init__(switch, mode, mrange, speed, count)

    def validate(self):
        assert (
            self._mode in CurrentChannelMode
            and self._mrange in CurrentRange
            and self._speed in ChannelSpeed
        )

    def __repr__(self):
        return (
            f"CurrentChannelConfig(switch={self._switch!r}, mode={self._mode!r},"
            + f" mrange={self._mrange!r}, speed={self._speed!r}, count={self._count!r})"
        )

    @classmethod
    def from_list(cls, lst):
        switch, mode, mrange, speed, count = lst
        return ChannelConfig(
            OnOff(switch),
            CurrentChannelMode(mode),
            CurrentRange(mrange),
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

    def validate_config(v, vs):
        assert isinstance(v, VoltageChannelConfig)
        return v

    config = Channel.control(
        "ROUTe:CHANnel? {ch}",
        "ROUTe:CHANnel {ch},%s",
        """""",
        validator=validate_config,
        get_process_list=lambda v: VoltageChannelConfig.from_list(v[1:]),
        check_set_errors=True,
        dynamic=True,
    )


class VoltageChannelNo4W(VoltageChannel):
    # this is a dynamic property that gets called instead of VoltageChannel's config.validator
    def config_validator(v, vs):
        assert isinstance(v, VoltageChannelConfig)
        if v._mode == VoltageChannelModeResistance.R4WIRE:
            raise ValueError(
                "4W mode is only allowed on channels 1 to 6. For each channel with 4W enabled,"
                + " the channel number +6 will be switched off automatically"
            )
        return v


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
        self.sc_enabled = True

    ch_1 = Instrument.ChannelCreator(VoltageChannel, "1")
    ch_2 = Instrument.ChannelCreator(VoltageChannel, "2")
    ch_3 = Instrument.ChannelCreator(VoltageChannel, "3")
    ch_4 = Instrument.ChannelCreator(VoltageChannel, "4")
    ch_5 = Instrument.ChannelCreator(VoltageChannel, "5")
    ch_6 = Instrument.ChannelCreator(VoltageChannel, "6")
    ch_7 = Instrument.ChannelCreator(VoltageChannelNo4W, "7")
    ch_8 = Instrument.ChannelCreator(VoltageChannelNo4W, "8")
    ch_9 = Instrument.ChannelCreator(VoltageChannelNo4W, "9")
    ch_10 = Instrument.ChannelCreator(VoltageChannelNo4W, "10")
    ch_11 = Instrument.ChannelCreator(VoltageChannelNo4W, "11")
    ch_12 = Instrument.ChannelCreator(VoltageChannelNo4W, "12")
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
        This setting automatically switches sc_cycle_mode to CycleMode.MANUAL, as in \
            CycleMode.AUTO measurements run indefinitely until sc_start is disabled.
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

    def reset(self):
        super().reset()
        self.sc_enabled = True

    def scan(self):
        self.sc_start = True
        self.wait_until_stopped()

    def wait_until_stopped(self, user_will_push_stop_button=False):
        if self.sc_cycle_mode == CycleMode.AUTO and not user_will_push_stop_button:
            raise Exception(
                "In CycleMode.AUTO the scan will not stop by itself, so the user has to push the "
                + "stop button to terminate the scan. Set user_will_push_stop_button=True "
                + "if you want to use this function with user interaction."
            )
        while self.sc_start:
            time.sleep(0.5)
