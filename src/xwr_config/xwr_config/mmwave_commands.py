from typing import List, Union

class MMwaveCommand:
    _command_types = []
    _extra_type = None
    _num_optional = 0
    _name = "base"

    def __init__(self, command_parameters:List[Union[int, float]]):
        self.command_parameters = command_parameters
    
    def __str__(self):
        return self._name + " " + " ".join([str(x) for x in self.command_parameters])
    
    @classmethod
    def _parse_args(cls, cmd_args:str) -> List[Union[int, float]]:
        if len(cmd_args) == 0 and len(cls._command_types) - cls._num_optional == 0:
            return []

        parts = cmd_args.split(" ")
        if len(parts) < len(cls._command_types) - cls._num_optional:
            raise ValueError(f"Not enough arguments for command {cls._name}: {cmd_args}")
        if len(parts) > len(cls._command_types) and cls._extra_type is None:
            raise ValueError(f"Too many arguments for command {cls._name}: {cmd_args}")
        try:
            args = [cls._command_types[i](parts[i]) for i in range(len(cls._command_types))]
            if cls._extra_type is not None:
                num_extra = len(parts) - len(cls._command_types)
                args.extend([cls._extra_type(parts[i + len(cls._command_types)]) for i in range(num_extra)])
            return args
        except ValueError:
            raise ValueError(f"Invalid string for command {cls._name}: {cmd_args}")
    
    @classmethod
    def parse(cls, string):
        return cls(cls._parse_args(string))


class dfeDataOutputModeCommand(MMwaveCommand):
    """The values in this command should not change between sensorStop and sensorStart. Reboot the board to try config with different set of values in this command This is a mandatory command."""

    _name = "dfeDataOutputMode"
    _command_types = [int]

    @property
    def modeType(self) -> int:
        """<modeType> 1 - frame based chirps 2 - continuous chirping 3 - advanced frame config

        usage: only option 1 and 3 are supported"""
        return self.command_parameters[0]

    @modeType.setter
    def modeType(self, value: int) -> None:
        self.command_parameters[0] = value


class channelCfgCommand(MMwaveCommand):
    """Channel config message to RadarSS. See mmwavelink doxgen for details. The values in this command should not change between sensorStop and sensorStart. Reboot the board to try config with different set of values in this command This is a mandatory command."""

    _name = "channelCfg"
    _command_types = [int, int, int]

    @property
    def rxChannelEn(self) -> int:
        """<rxChannelEn> Receive antenna mask e.g for 4 antennas, it is 0x1111b = 15

        usage: 4 antennas supported"""
        return self.command_parameters[0]

    @rxChannelEn.setter
    def rxChannelEn(self, value: int) -> None:
        self.command_parameters[0] = value

    @property
    def txChannelEn(self) -> int:
        """<txChannelEn> Transmit antenna mask

        usage: Refer to the antenna layout on the EVM /board to determine the right Tx antenna mask needed to enable the desired virtual antenna configuration. For example, in IWR6843 ISK, the 2 azimuth antennas can be enabled using bitmask 0x5 (i.e. tx1 and tx3). The azimuth and elevation antennas can both be enabled using bitmask 0x7 (i.e. tx1, tx2 and tx3). For example, in xWR1642BOOST, the azimuth antennas can be enabled using bitmask 0x3 (i.e. tx1 and tx2)."""
        return self.command_parameters[1]

    @txChannelEn.setter
    def txChannelEn(self, value: int) -> None:
        self.command_parameters[1] = value

    @property
    def cascading(self) -> int:
        """<cascading> SoC cascading, not applicable, set to 0

        usage: n/a"""
        return self.command_parameters[2]

    @cascading.setter
    def cascading(self, value: int) -> None:
        self.command_parameters[2] = value


class adcCfgCommand(MMwaveCommand):
    """ADC config message to RadarSS. See mmwavelink doxgen for details. The values in this command should not change between sensorStop and sensorStart. Reboot the board to try config with different set of values in this command This is a mandatory command."""

    _name = "adcCfg"
    _command_types = [int, int]

    @property
    def numADCBits(self) -> int:
        """<numADCBits> Number of ADC bits (0 for 12-bits, 1 for 14-bits and 2 for 16-bits)

        usage: only 16-bit is supported"""
        return self.command_parameters[0]

    @numADCBits.setter
    def numADCBits(self, value: int) -> None:
        self.command_parameters[0] = value

    @property
    def adcOutputFmt(self) -> int:
        """<adcOutputFmt> Output format : 0 - real 1 - complex 1x (image band filtered output) 2 - complex 2x (image band visible))

        usage: only complex modes are supported"""
        return self.command_parameters[1]

    @adcOutputFmt.setter
    def adcOutputFmt(self, value: int) -> None:
        self.command_parameters[1] = value


class adcbufCfgCommand(MMwaveCommand):
    """adcBuf hardware config. The values in this command can be changed between sensorStop and sensorStart. This is a mandatory command."""

    _name = "adcbufCfg"
    _command_types = [int, int, int, int, int]

    @property
    def subFrameIdx(self) -> int:
        """<subFrameIdx> subframe Index

        usage: For legacy mode, that field should be set to -1 For advanced frame mode, it should be set to either the intended subframe number or -1 to apply same config to all subframes."""
        return self.command_parameters[0]

    @subFrameIdx.setter
    def subFrameIdx(self, value: int) -> None:
        self.command_parameters[0] = value

    @property
    def adcOutputFmt(self) -> int:
        """<adcOutputFmt> ADCBUF out format 0-Complex, 1-Real

        usage: only complex modes are supported"""
        return self.command_parameters[1]

    @adcOutputFmt.setter
    def adcOutputFmt(self, value: int) -> None:
        self.command_parameters[1] = value

    @property
    def SampleSwap(self) -> int:
        """<SampleSwap> ADCBUF IQ swap selection: 0-I in LSB, Q in MSB, 1-Q in LSB, I in MSB

        usage: only option 1 is supported"""
        return self.command_parameters[2]

    @SampleSwap.setter
    def SampleSwap(self, value: int) -> None:
        self.command_parameters[2] = value

    @property
    def ChanInterleave(self) -> int:
        """<ChanInterleave> ADCBUF channel interleave configuration: 0 - interleaved(only supported for XWR14xx), 1 - non-interleaved

        usage: only option 1 is supported"""
        return self.command_parameters[3]

    @ChanInterleave.setter
    def ChanInterleave(self, value: int) -> None:
        self.command_parameters[3] = value

    @property
    def ChirpThreshold(self) -> int:
        """<ChirpThreshold> Chirp Threshold configuration used for ADCBUF buffer to trigger ping /pong buffer switch. Valid values: 0-8 for demos that use DSP for 1D FFT and LVDS streaming is disabled only 1 for demos that use HWA for 1D FFT

        usage: xwr16xx demo: Values 0-8 are supported sinc it uses DSP for 1D FFT However, only value of 1 is supported when LVDS streaming is enabled. xwr64xx/xwr68xx /xwr18xx: only value of 1 is supported since these demos use HWA for 1D FFT"""
        return self.command_parameters[4]

    @ChirpThreshold.setter
    def ChirpThreshold(self, value: int) -> None:
        self.command_parameters[4] = value


class profileCfgCommand(MMwaveCommand):
    """Profile config message to RadarSS and datapath. See mmwavelink doxgen for details. The values in this command can be changed between sensorStop and sensorStart. This is a mandatory command. txCalibEnCfg Field This CLI command doesn't expose the txCalibEnCfg field in the mmwavelink structure. User should follow the mmwavelink documentation and update the CLI profileCfg handler function accordingly. The current handler sets the value to 0 for this field (backward compatible mode) Combination of numAdcSamples in profileCfg (and numRangeBins), numDopplerChirps = total number of chirps/(num TX in MIMO mode) in frameCfg or subFrameCfg, number of TX and RX antennas in channelCfg and chirpCfg determine the size of Radarcube and other internal buffers/heap in the demo. It is possible that some combinations of these values result in out of memory conditions for these heaps and demo will reject such configuration. Refer to demo and DPC doxygen to understand the data buffer layout and use the system printfs on sensorStart in CCS console window to understand the exact heap usage for a given configuration."""

    _name = "profileCfg"
    _command_types = [int, float, float, float, float, int, int, float, float, int, float, int, int, int]

    @property
    def profileId(self) -> int:
        """<profileId> profile Identifier

        usage: Legacy frame (dfeOutputMode=1): could be any allowed value but only one vali profile per config is supported Advanced frame (dfeOutputMode=3): could be any allowed value but only one profile per subframe is supported. However, different subframes ca have different profiles"""
        return self.command_parameters[0]

    @profileId.setter
    def profileId(self, value: int) -> None:
        self.command_parameters[0] = value

    @property
    def startFreq(self) -> float:
        """<startFreq> "Frequency Start" in GHz (float values allowed) Examples: 77 61.38

        usage: any value as per mmwavelink doxgen /device datasheet but represented in GHz. Refer to the chirp diagram shown above to understand the relation between various profile parameters and inter- dependent constraints."""
        return self.command_parameters[1]

    @startFreq.setter
    def startFreq(self, value: float) -> None:
        self.command_parameters[1] = value

    @property
    def idleTime(self) -> float:
        """<idleTime> "Idle Time" in u-sec (float values allowed) Examples: 7 7.15

        usage: any value as per mmwavelink doxgen /device datasheet but represented in usec. Refer to the chirp diagram shown above to understand the relation between various profile parameters and inter- dependent constraints."""
        return self.command_parameters[2]

    @idleTime.setter
    def idleTime(self, value: float) -> None:
        self.command_parameters[2] = value

    @property
    def adcStartTime(self) -> float:
        """<adcStartTime> "ADC Valid Start Time" in u-sec (float values allowed) Examples: 7 7.34

        usage: any value as per mmwavelink doxgen /device datasheet but represented in usec. Refer to the chirp diagram shown above to understand the relation between various profile parameters and inter- dependent constraints."""
        return self.command_parameters[3]

    @adcStartTime.setter
    def adcStartTime(self, value: float) -> None:
        self.command_parameters[3] = value

    @property
    def rampEndTime(self) -> float:
        """<rampEndTime> "Ramp End Time" in u-sec (float values allowed) Examples: 58 216.15

        usage: any value as per mmwavelink doxgen /device datasheet but represented in usec Refer to the chirp diagram shown above to understand the relation between various profile parameters and inter- dependent constraints."""
        return self.command_parameters[4]

    @rampEndTime.setter
    def rampEndTime(self, value: float) -> None:
        self.command_parameters[4] = value

    @property
    def txOutPower(self) -> int:
        """<txOutPower> Tx output power back-off code for tx antennas

        usage: only value of '0' has been tested within context of mmW demo"""
        return self.command_parameters[5]

    @txOutPower.setter
    def txOutPower(self, value: int) -> None:
        self.command_parameters[5] = value

    @property
    def txPhaseShifter(self) -> int:
        """<txPhaseShifter> tx phase shifter for tx antennas

        usage: only value of '0' has been tested within context of mmW demo"""
        return self.command_parameters[6]

    @txPhaseShifter.setter
    def txPhaseShifter(self, value: int) -> None:
        self.command_parameters[6] = value

    @property
    def freqSlopeConst(self) -> float:
        """<freqSlopeConst> "Frequency slope" for the chirp in MHz/usec (float values allowed) Examples: 68 16.83

        usage: any value greater than as per mmwavelink doxgen/device datasheet but represented in MHz /usec. Refer to the chirp diagram shown above to understand the relation between various profile parameters and inter- dependent constraints."""
        return self.command_parameters[7]

    @freqSlopeConst.setter
    def freqSlopeConst(self, value: float) -> None:
        self.command_parameters[7] = value

    @property
    def txStartTime(self) -> float:
        """<txStartTime> "TX Start Time" in u-sec (float values allowed) Examples: 1 2.92

        usage: any value as per mmwavelink doxgen /device datasheet but represented in usec. Refer to the chirp diagram shown above to understand the relation between various profile parameters and inter- dependent constraints."""
        return self.command_parameters[8]

    @txStartTime.setter
    def txStartTime(self, value: float) -> None:
        self.command_parameters[8] = value

    @property
    def numAdcSamples(self) -> int:
        """<numAdcSamples> number of ADC samples collected during "ADC Sampling Time" as shown in the chirp diagram above Examples: 256 224

        usage: any value as per mmwavelink doxgen /device datasheet. Refer to the chirp diagram shown above to understand the relation between various profile parameters and inter- dependent constraints."""
        return self.command_parameters[9]

    @numAdcSamples.setter
    def numAdcSamples(self, value: int) -> None:
        self.command_parameters[9] = value

    @property
    def digOutSampleRate(self) -> float:
        """<digOutSampleRate> ADC sampling frequency in ksps. (<numAdcSamples> / <digOutSampleRate> = "ADC Sampling Time") Examples: 5500

        usage: any value as per mmwavelink doxgen /device datasheet. Refer to the chirp diagram shown above to understand the relation between various profile parameters and inter- dependent constraints."""
        return self.command_parameters[10]

    @digOutSampleRate.setter
    def digOutSampleRate(self, value: float) -> None:
        self.command_parameters[10] = value

    @property
    def hpfCornerFreq1(self) -> int:
        """<hpfCornerFreq1> HPF1 (High Pass Filter 1) corner frequency 0: 175 KHz 1: 235 KHz 2: 350 KHz 3: 700 KHz

        usage: any value as per mmwavelink doxgen /device datasheet"""
        return self.command_parameters[11]

    @hpfCornerFreq1.setter
    def hpfCornerFreq1(self, value: int) -> None:
        self.command_parameters[11] = value

    @property
    def hpfCornerFreq2(self) -> int:
        """<hpfCornerFreq2> HPF2 (High Pass Filter 2) corner frequency 0: 350 KHz 1: 700 KHz 2: 1.4 MHz 3: 2.8 MHz

        usage: any value as per mmwavelink doxgen /device datasheet"""
        return self.command_parameters[12]

    @hpfCornerFreq2.setter
    def hpfCornerFreq2(self, value: int) -> None:
        self.command_parameters[12] = value

    @property
    def rxGain(self) -> int:
        """<rxGain> OR'ed value of RX gain in dB and RF gain target (See mmwavelink doxgen for details)

        usage: any value as per mmwavelink doxgen /device datasheet"""
        return self.command_parameters[13]

    @rxGain.setter
    def rxGain(self, value: int) -> None:
        self.command_parameters[13] = value


class chirpCfgCommand(MMwaveCommand):
    """Chirp config message to RadarSS and datapath. See mmwavelink doxgen for details. The values in this command can be changed between sensorStop and sensorStart. This is a mandatory command."""

    _name = "chirpCfg"
    _command_types = [int, int, int, int, int, int, int, int]

    @property
    def chirpStartIndex(self) -> int:
        """chirp start index

        usage: any value as per mmwavelink doxygen"""
        return self.command_parameters[0]

    @chirpStartIndex.setter
    def chirpStartIndex(self, value: int) -> None:
        self.command_parameters[0] = value

    @property
    def chirpEndIndex(self) -> int:
        """chirp end index

        usage: any value as per mmwavelink doxygen"""
        return self.command_parameters[1]

    @chirpEndIndex.setter
    def chirpEndIndex(self, value: int) -> None:
        self.command_parameters[1] = value

    @property
    def profileIdentifier(self) -> int:
        """profile identifier

        usage: should match the profileCfg->profileId"""
        return self.command_parameters[2]

    @profileIdentifier.setter
    def profileIdentifier(self, value: int) -> None:
        self.command_parameters[2] = value

    @property
    def startFrequencyVariation(self) -> int:
        """start frequency variation in Hz (float values allowed)

        usage: only value of '0' has been tested within context of mmW demo"""
        return self.command_parameters[3]

    @startFrequencyVariation.setter
    def startFrequencyVariation(self, value: int) -> None:
        self.command_parameters[3] = value

    @property
    def frequencySlopeVariation(self) -> int:
        """frequency slope variation in kHz/us (float values allowed)

        usage: only value of '0' has been tested within context of mmW demo"""
        return self.command_parameters[4]

    @frequencySlopeVariation.setter
    def frequencySlopeVariation(self, value: int) -> None:
        self.command_parameters[4] = value

    @property
    def idleTimeVariation(self) -> int:
        """idle time variation in u-sec (float values allowed)

        usage: only value of '0' has been tested within context of mmW demo"""
        return self.command_parameters[5]

    @idleTimeVariation.setter
    def idleTimeVariation(self, value: int) -> None:
        self.command_parameters[5] = value

    @property
    def ADCStartTimeVariation(self) -> int:
        """ADC start time variation in u-sec (float values allowed)

        usage: only value of '0' has been tested within context of mmW demo"""
        return self.command_parameters[6]

    @ADCStartTimeVariation.setter
    def ADCStartTimeVariation(self, value: int) -> None:
        self.command_parameters[6] = value

    @property
    def txAntennaEnableMask(self) -> int:
        """tx antenna enable mask (Tx2,Tx1) e.g (10)b = Tx2 enabled, Tx1 disabled.

        usage: See note under "Channel Cfg" command above. Individual chirps should have either only one distinct Tx antenna enabled (MIMO) or same TX antennas should be enabled for all chirps"""
        return self.command_parameters[7]

    @txAntennaEnableMask.setter
    def txAntennaEnableMask(self, value: int) -> None:
        self.command_parameters[7] = value


class lowPowerCommand(MMwaveCommand):
    """Low Power mode config message to RadarSS. See mmwavelink doxgen for details. The values in this command should not change between sensorStop and sensorStart. Reboot the board to try config with different set of values in this command. This is a mandatory command."""

    _name = "lowPower"
    _command_types = [int, int]

    @property
    def _dontCare(self) -> int:
        """<don’t_care>

        usage: set to 0"""
        return self.command_parameters[0]

    @_dontCare.setter
    def _dontCare(self, value: int) -> None:
        self.command_parameters[0] = value

    @property
    def ADCMode(self) -> int:
        """ADC Mode 0x00 : Regular ADC mode 0x01 : Low power ADC mode (Not supported for xwr6xxx devices)

        usage: use value of '0' or '1' (depending on profileCfg- >digOutSampleRate)"""
        return self.command_parameters[1]

    @ADCMode.setter
    def ADCMode(self, value: int) -> None:
        self.command_parameters[1] = value


class frameCfgCommand(MMwaveCommand):
    """frame config message to RadarSS and datapath. See mmwavelink doxgen for details. dfeOutputMode should be set to 1 to use this command The values in this command can be changed between sensorStop and sensorStart. This is a mandatory command when dfeOutputMode is set to 1."""

    _name = "frameCfg"
    _command_types = [int, int, int, int, float, int, float]

    @property
    def chirpStartIndex(self) -> int:
        """chirp start index (0-511)

        usage: any value as per mmwavelink doxgen b corresponding chirpCfg should be defined"""
        return self.command_parameters[0]

    @chirpStartIndex.setter
    def chirpStartIndex(self, value: int) -> None:
        self.command_parameters[0] = value

    @property
    def chirpEndIndex(self) -> int:
        """chirp end index (chirp start index- 511)

        usage: any value as per mmwavelink doxgen b corresponding chirpCfg should be defined"""
        return self.command_parameters[1]

    @chirpEndIndex.setter
    def chirpEndIndex(self, value: int) -> None:
        self.command_parameters[1] = value

    @property
    def numberOfLoops(self) -> int:
        """number of loops (1 to 255)

        usage: any value as per mmwavelink doxgen /device datasheet but greater than or equal to 4. For xwr16xx/xwr68xx demos where DSP version of Doppler DPU is used, the Doppler chirps (i.e. number of loops) should be a multiple of 4 due to windowing requirement Note: If value of 2 is desired for number of Doppler Chirps, one must update the demo /object detection DPC source code to use rectangular window for Doppler DPU instead o Hanning window."""
        return self.command_parameters[2]

    @numberOfLoops.setter
    def numberOfLoops(self, value: int) -> None:
        self.command_parameters[2] = value

    @property
    def numberOfFrames(self) -> int:
        """number of frames (valid range is 0 to 65535, 0 means infinite)

        usage: any value as per mmwavelink doxgen"""
        return self.command_parameters[3]

    @numberOfFrames.setter
    def numberOfFrames(self, value: int) -> None:
        self.command_parameters[3] = value

    @property
    def framePeriodicity(self) -> float:
        """frame periodicity in ms (float values allowed)

        usage: any value as per mmwavelink doxgen and represented in msec. However frame should not have more than 50% duty cycle (i. e. active chirp time should be <= 50% of frame period). Also it should allow enough time for selected UART output to be shipped o (selections based on guiMonitor command) else demo will assert if the next frame start trigger is received from the front end and current frame is still ongoing. User can use the output of stats TLV to tune this parameter."""
        return self.command_parameters[4]

    @framePeriodicity.setter
    def framePeriodicity(self, value: float) -> None:
        self.command_parameters[4] = value

    @property
    def triggerSelect(self) -> int:
        """trigger select 1: Software trigger 2: Hardware trigger.

        usage: only option for Softwar trigger is supported"""
        return self.command_parameters[5]

    @triggerSelect.setter
    def triggerSelect(self, value: int) -> None:
        self.command_parameters[5] = value

    @property
    def frameTriggerDelay(self) -> float:
        """Frame trigger delay in ms (float values allowed)

        usage: any value as per mmwavelink doxgen and represented in msec."""
        return self.command_parameters[6]

    @frameTriggerDelay.setter
    def frameTriggerDelay(self, value: float) -> None:
        self.command_parameters[6] = value


class advFrameCfgCommand(MMwaveCommand):
    """Advanced config message to RadarSS and datapath. See mmwavelink doxgen for details. The dfeOutputMode should be set to 3 to use this command. See profile_advanced_subframe. cfg profile in the mmW demo profiles directory for example usage. The values in this command can be changed between sensorStop and sensorStart. This is a mandatory command when dfeOutputMode is set to 3."""

    _name = "advFrameCfg"
    _command_types = [int, int, int, int, float]

    @property
    def numOfSubFrames(self) -> int:
        """<numOfSubFrames> Number of sub frames enabled in this frame

        usage: any value as per mmwavelink doxgen"""
        return self.command_parameters[0]

    @numOfSubFrames.setter
    def numOfSubFrames(self, value: int) -> None:
        self.command_parameters[0] = value

    @property
    def forceProfile(self) -> int:
        """<forceProfile> Force profile

        usage: only value of 0 is supported"""
        return self.command_parameters[1]

    @forceProfile.setter
    def forceProfile(self, value: int) -> None:
        self.command_parameters[1] = value

    @property
    def numFrames(self) -> int:
        """<numFrames> Number of frames to transmit (1 frame = all enabled sub frames)

        usage: any value as per mmwavelink doxgen"""
        return self.command_parameters[2]

    @numFrames.setter
    def numFrames(self, value: int) -> None:
        self.command_parameters[2] = value

    @property
    def triggerSelect(self) -> int:
        """<triggerSelect> trigger select 1: Software trigger 2: Hardware trigger.

        usage: only option for Softwar trigger is supported"""
        return self.command_parameters[3]

    @triggerSelect.setter
    def triggerSelect(self, value: int) -> None:
        self.command_parameters[3] = value

    @property
    def frameTrigDelay(self) -> float:
        """<frameTrigDelay> Frame trigger delay in ms (float values allowed)

        usage: any value as per mmwavelink doxgen and represented in msec."""
        return self.command_parameters[4]

    @frameTrigDelay.setter
    def frameTrigDelay(self, value: float) -> None:
        self.command_parameters[4] = value


class subFrameCfgCommand(MMwaveCommand):
    """Subframe config message to RadarSS and datapath. See mmwavelink doxgen for details."""

    _name = "subFrameCfg"
    _command_types = [int, int, int, int, int, float, int, int, int, int]

    @property
    def subFrameNum(self) -> int:
        """<subFrameNum> subframe Number for which this command is being given

        usage: value of 0 to RL_MAX_SUBFRAME 1"""
        return self.command_parameters[0]

    @subFrameNum.setter
    def subFrameNum(self, value: int) -> None:
        self.command_parameters[0] = value

    @property
    def forceProfileIdx(self) -> int:
        """<forceProfileIdx> Force profile index

        usage: ignored as <forceProfile> in advFrameCfg should b set to 0"""
        return self.command_parameters[1]

    @forceProfileIdx.setter
    def forceProfileIdx(self, value: int) -> None:
        self.command_parameters[1] = value

    @property
    def chirpStartIdx(self) -> int:
        """<chirpStartIdx> Start Index of Chirp

        usage: any value as per mmwavelink doxgen b corresponding chirpCfg should be defined"""
        return self.command_parameters[2]

    @chirpStartIdx.setter
    def chirpStartIdx(self, value: int) -> None:
        self.command_parameters[2] = value

    @property
    def numOfChirps(self) -> int:
        """<numOfChirps> Num of unique Chirps per burst including start index

        usage: any value as per mmwavelink doxgen b corresponding number of chirpCfg should be defined"""
        return self.command_parameters[3]

    @numOfChirps.setter
    def numOfChirps(self, value: int) -> None:
        self.command_parameters[3] = value

    @property
    def numLoops(self) -> int:
        """<numLoops> No. of times to loop through the unique chirps

        usage: any value as per mmwavelink doxgen b greater than or equal to 4 For xwr16xx/xwr68xx demos where DSP version of Doppler DPU is used, the Doppler chirps (i.e. number of loops) should be a multiple of 4 due to windowing requirement Note: If value of 2 is desired for number of Doppler Chirps, one must update the demo /object detection DPC source code to use rectangular window for Doppler DPU instead o Hanning window."""
        return self.command_parameters[4]

    @numLoops.setter
    def numLoops(self, value: int) -> None:
        self.command_parameters[4] = value

    @property
    def burstPeriodicity(self) -> float:
        """<burstPeriodicity> Burst periodicty in msec (float values allowed) and meets the criteria burstPeriodicity >= ((numLoops)* (Sum total of time duration of all unique chirps in that burst)) + InterBurstBlankTime

        usage: any value as per mmwavelink doxgen and represented in msec but subframe should not have more than 50% duty cycle and allow enough time for selected UART output to be shipped o (selections based on guiMonitor command)"""
        return self.command_parameters[5]

    @burstPeriodicity.setter
    def burstPeriodicity(self, value: float) -> None:
        self.command_parameters[5] = value

    @property
    def chirpStartIdxOffset(self) -> int:
        """<chirpStartIdxOffset> Chirp Start address increament for next burst

        usage: set it to 0 since demo supports only one burs per subframe"""
        return self.command_parameters[6]

    @chirpStartIdxOffset.setter
    def chirpStartIdxOffset(self, value: int) -> None:
        self.command_parameters[6] = value

    @property
    def numOfBurst(self) -> int:
        """<numOfBurst> Num of bursts in the subframe

        usage: set it to 1 since demo supports only one burs per subframe"""
        return self.command_parameters[7]

    @numOfBurst.setter
    def numOfBurst(self, value: int) -> None:
        self.command_parameters[7] = value

    @property
    def numOfBurstLoops(self) -> int:
        """<numOfBurstLoops> Number of times to loop over the set of above defined bursts, in the sub frame

        usage: set it to 1 since demo supports only one burs per subframe"""
        return self.command_parameters[8]

    @numOfBurstLoops.setter
    def numOfBurstLoops(self, value: int) -> None:
        self.command_parameters[8] = value

    @property
    def subFramePeriodicity(self) -> int:
        """<subFramePeriodicity> subFrame periodicty in msec (float values allowed) and meets the criteria subFramePeriodicity >= Sum total time of all bursts + InterSubFrameBlankTime

        usage: set to same as <burstPeriodicity> sinc demo supports only on burst per subframe"""
        return self.command_parameters[9]

    @subFramePeriodicity.setter
    def subFramePeriodicity(self, value: int) -> None:
        self.command_parameters[9] = value


class guiMonitorCommand(MMwaveCommand):
    """Plot config message to datapath. The values in this command can be changed between sensorStop and sensorStart. This is a mandatory command."""

    _name = "guiMonitor"
    _command_types = [int, int, int, int, int, int, int]

    @property
    def subFrameIdx(self) -> int:
        """<subFrameIdx> subframe Index

        usage: For legacy mode, that field should be set to -1 whereas for advanced frame mode, it should be set to either the intended subframe number or -1 to apply same config to all subframes."""
        return self.command_parameters[0]

    @subFrameIdx.setter
    def subFrameIdx(self, value: int) -> None:
        self.command_parameters[0] = value

    @property
    def detectedObjectsExportMode(self) -> int:
        """<detected objects> 1 - enable export of point cloud (x,y, z,doppler) and point cloud sideinfo (SNR, noiseval) 2 - enable export of point cloud (x,y, z,doppler) 0 - disable

        usage: all values supported"""
        return self.command_parameters[1]

    @detectedObjectsExportMode.setter
    def detectedObjectsExportMode(self, value: int) -> None:
        self.command_parameters[1] = value

    @property
    def logMagnitudeRangeExport(self) -> int:
        """<log magnitude range> 1 - enable export of log magnitude range profile at zero Doppler 0 - disable

        usage: all values supported"""
        return self.command_parameters[2]

    @logMagnitudeRangeExport.setter
    def logMagnitudeRangeExport(self, value: int) -> None:
        self.command_parameters[2] = value

    @property
    def noiseProfileExport(self) -> int:
        """<noise profile> 1 - enable export of log magnitude noise profile 0 - disable

        usage: all values supported"""
        return self.command_parameters[3]

    @noiseProfileExport.setter
    def noiseProfileExport(self, value: int) -> None:
        self.command_parameters[3] = value

    @property
    def rangeAzimuthHeatMap(self) -> int:
        """<rangeAzimuthHeatMap> or <rangeAzimuthElevationHeatMap> range-azimuth or range-azimuth- elevation heat map related information <rangeAzimuthHeatMap> This output is provided only in demos that use AoA (legacy) DPU for AoA processing 1 - enable export of zero Doppler radar cube matrix, all range bins, all azimuth virtual antennas to calculate and display azimuth heat map. (The GUI computes the FFT of this to show heat map) 0 - disable < rangeAzimuthElevationHeatMap > This output is provided in demos that use AoA 2D DPU for AoA processing (ex: mmW demo for IWR6843AOP) 1 - enable export of zero Doppler radar cube matrix, all range bins, all virtual antennas to calculate and display azimuth heat map. (The GUI remaps the antenna symbols and computes the FFT of this stream to show azimuth heat map only). 0 - disable

        usage: all values supported"""
        return self.command_parameters[4]

    @rangeAzimuthHeatMap.setter
    def rangeAzimuthHeatMap(self, value: int) -> None:
        self.command_parameters[4] = value

    @property
    def rangeDopplerHeatMap(self) -> int:
        """<rangeDopplerHeatMap> range-doppler heat map 1 - enable export of the whole detection matrix. Note that the frame period should be adjusted according to UART transfer time. 0 - disable

        usage: all values supported"""
        return self.command_parameters[5]

    @rangeDopplerHeatMap.setter
    def rangeDopplerHeatMap(self, value: int) -> None:
        self.command_parameters[5] = value

    @property
    def statsInfo(self) -> int:
        """<statsInfo> statistics (CPU load, margins, device temperature readings, etc) 1 - enable export of stats data. 0 - disable

        usage: all values supported"""
        return self.command_parameters[6]

    @statsInfo.setter
    def statsInfo(self, value: int) -> None:
        self.command_parameters[6] = value


class cfarCfgCommand(MMwaveCommand):
    """CFAR config message to datapath. The values in this command can be changed between sensorStop and sensorStart and even when the sensor is running. This is a mandatory command."""

    _name = "cfarCfg"
    _command_types = [int, int, int, int, int, int, int, float, int]

    @property
    def subFrameIdx(self) -> int:
        """<subFrameIdx> subframe Index

        usage: For legacy mode, that field should be set to -1 whereas for advanced frame mode, it should be set to either the intended subframe number or -1 to apply same config to all subframes."""
        return self.command_parameters[0]

    @subFrameIdx.setter
    def subFrameIdx(self, value: int) -> None:
        self.command_parameters[0] = value

    @property
    def procDirection(self) -> int:
        """<procDirection> Processing direction: 0 – CFAR detection in range direction 1 – CFAR detection in Doppler direction

        usage: all values supported; 2 separate commands need to be sent; one for Range and other for doppler."""
        return self.command_parameters[1]

    @procDirection.setter
    def procDirection(self, value: int) -> None:
        self.command_parameters[1] = value

    @property
    def mode(self) -> int:
        """<mode> CFAR averaging mode: 0 - CFAR_CA (Cell Averaging) 1 - CFAR_CAGO (Cell Averaging Greatest Of) 2 - CFAR_CASO (Cell Averaging Smallest Of)

        usage: all values supported"""
        return self.command_parameters[2]

    @mode.setter
    def mode(self, value: int) -> None:
        self.command_parameters[2] = value

    @property
    def noiseWin(self) -> int:
        """<noiseWin> noise averaging window length: Length of the one sided noise averaged cells in samples Make sure 2*(noiseWIn+guardLen) <numRangeBins for range direction and 2*(noiseWIn+guardLen) <numDopplerBins for doppler direction.

        usage: supported"""
        return self.command_parameters[3]

    @noiseWin.setter
    def noiseWin(self, value: int) -> None:
        self.command_parameters[3] = value

    @property
    def guardLen(self) -> int:
        """<guardLen> one sided guard length in samples Make sure 2*(noiseWIn+guardLen) <numRangeBins for range direction and 2*(noiseWIn+guardLen) <numDopplerBins for doppler direction.

        usage: supported"""
        return self.command_parameters[4]

    @guardLen.setter
    def guardLen(self, value: int) -> None:
        self.command_parameters[4] = value

    @property
    def divShift(self) -> int:
        """<divShift> Cumulative noise sum divisor expressed as a shift. Sum of noise samples is divided by 2^<divShift>. Based on <mode> and <noiseWin> , this value should be set as shown in next columns. The value to be used here should match the "CFAR averaging mode" and the "noise averaging window length" that is selected above. The actual value that is used for division (2^x) is a power of 2, even though the "noise averaging window length" samples may not have that restriction.

        usage: CFAR_CA: <divShift> = ceil(log2(2 x <noiseWin>)) CFAR_CAGO/_CASO: <divShift> = ceil(log2 (<noiseWin>)) In profile_2d.cfg, value of 3 means that the noise sum is divided by 2^3=8 to get the average of noise samples with window length of 8 samples in CFAR -CASO mode."""
        return self.command_parameters[5]

    @divShift.setter
    def divShift(self, value: int) -> None:
        self.command_parameters[5] = value

    @property
    def cyclicModeEnabled(self) -> int:
        """cyclic mode or Wrapped around mode. 0- Disabled 1- Enabled

        usage: supported"""
        return self.command_parameters[6]

    @cyclicModeEnabled.setter
    def cyclicModeEnabled(self, value: int) -> None:
        self.command_parameters[6] = value

    @property
    def thresholdScale(self) -> float:
        """Threshold scale in dB using float representation. This is used in conjuntion with the noise sum divisor (say x). the CUT comparison for log input is: CUT > (Threshold scale converted from dB to Q8) + (noise sum / 2^x) For example: 15 10.75

        usage: Detection threshold is specified in dB scale. Maximum value allowe is 100dB"""
        return self.command_parameters[7]

    @thresholdScale.setter
    def thresholdScale(self, value: float) -> None:
        self.command_parameters[7] = value

    @property
    def peakGroupingEnabled(self) -> int:
        """peak grouping 0 - disabled 1 - enabled

        usage: supported"""
        return self.command_parameters[8]

    @peakGroupingEnabled.setter
    def peakGroupingEnabled(self, value: int) -> None:
        self.command_parameters[8] = value


class multiObjBeamFormingCommand(MMwaveCommand):
    """Multi Object Beamforming config message to datapath. This feature allows radar to separate reflections from multiple objects originating from the same range/Doppler detection. The procedure searches for the second peak after locating the highest peak in Azimuth FFT. If the second peak is greater than the specified threshold, the second object with the same range /Doppler is appended to the list of detected objects. The threshold is proportional to the height of the highest peak. The values in this command can be changed between sensorStop and sensorStart and even when the sensor is running. This is a mandatory command."""

    _name = "multiObjBeamForming"
    _command_types = [int, int, float]

    @property
    def subFrameIdx(self) -> int:
        """<subFrameIdx> subframe Index

        usage: For legacy mode, that field should be set to -1 whereas for advanced frame mode, it should be set to either the intended subframe number or -1 to apply same config to all subframes."""
        return self.command_parameters[0]

    @subFrameIdx.setter
    def subFrameIdx(self, value: int) -> None:
        self.command_parameters[0] = value

    @property
    def featureEnabled(self) -> int:
        """<Feature Enabled> 0 - disabled 1 - enabled

        usage: supported"""
        return self.command_parameters[1]

    @featureEnabled.setter
    def featureEnabled(self, value: int) -> None:
        self.command_parameters[1] = value

    @property
    def threshold(self) -> float:
        """<threshold> 0 to 1 – threshold scale for the second peak detection in azimuth FFT output. Detection threshold is equal to <thresholdScale> multiplied by the first peak height. Note that FFT output is magnitude squared.

        usage: supported"""
        return self.command_parameters[2]

    @threshold.setter
    def threshold(self, value: float) -> None:
        self.command_parameters[2] = value


class calibDcRangeSigCommand(MMwaveCommand):
    """DC range calibration config message to datapath. Antenna coupling signature dominates the range bins close to the radar. These are the bins in the range FFT output located around DC. When this feature is enabled, the signature is estimated during the first N chirps, and then it is subtracted during the subsequent chirps. During the estimation period the specified bins (defined as [negativeBinIdx, positiveBinIdx]) around DC are accumulated and averaged. It is assumed that no objects are present in the vicinity of the radar at that time. This procedure is initiated by the following CLI command, and it can be initiated any time while radar is running. Note that the maximum number of compensated bins is 32. The values in this command can be changed between sensorStop and sensorStart and even when the sensor is running. This is a mandatory command."""

    _name = "calibDcRangeSig"
    _command_types = [int, int, int, int, int]

    @property
    def subFrameIdx(self) -> int:
        """<subFrameIdx> subframe Index

        usage: For legacy mode, that field should be set to -1 whereas for advanced frame mode, it should be set to either the intended subframe number or -1 to apply same config to all subframes."""
        return self.command_parameters[0]

    @subFrameIdx.setter
    def subFrameIdx(self, value: int) -> None:
        self.command_parameters[0] = value

    @property
    def enabled(self) -> int:
        """<enabled> Enable DC removal using first few chirps 0 - disabled 1 - enabled

        usage: supported"""
        return self.command_parameters[1]

    @enabled.setter
    def enabled(self, value: int) -> None:
        self.command_parameters[1] = value

    @property
    def negativeBinIdx(self) -> int:
        """<negativeBinIdx> negative Bin Index (to remove DC from farthest range bins) Maximum negative range FFT index to be included for compensation. Negative indices are indices wrapped around from far end of 1D FFT. Ex: Value of -5 means last 5 bins starting from the farthest bin

        usage: supported"""
        return self.command_parameters[2]

    @negativeBinIdx.setter
    def negativeBinIdx(self, value: int) -> None:
        self.command_parameters[2] = value

    @property
    def positiveBinIdx(self) -> int:
        """<positiveBinIdx> positive Bin Index (to remove DC from closest range bins) Maximum positive range FFT index to be included for compensation Value of 8 means first 9 bins (including bin#0)

        usage: supported"""
        return self.command_parameters[3]

    @positiveBinIdx.setter
    def positiveBinIdx(self, value: int) -> None:
        self.command_parameters[3] = value

    @property
    def numAvg(self) -> int:
        """<numAvg> number of chirps to average to collect DC signature (which will then be applied to all chirps beyond this). Value of 256 means first 256 chirps (after command is issued and feature is enabled) will be used for collecting (averaging) DC signature in the bins specified above. From 257th chirp, the collected DC signature will be removed from every chirp.

        usage: The value must be power of 2, and must b greater than the numbe of Doppler bins."""
        return self.command_parameters[4]

    @numAvg.setter
    def numAvg(self, value: int) -> None:
        self.command_parameters[4] = value


class clutterRemovalCommand(MMwaveCommand):
    """Static clutter removal config message to datapath. Static clutter removal algorithm implemented by subtracting from the samples the mean value of the input samples to the 2D-FFT The values in this command can be changed between sensorStop and sensorStart and even when the sensor is running. This is a mandatory command."""

    _name = "clutterRemoval"
    _command_types = [int, int]

    @property
    def subFrameIdx(self) -> int:
        """<subFrameIdx> subframe Index

        usage: For legacy mode, that field should be set to -1 whereas for advanced frame mode, it should be set to either the intended subframe number or -1 to apply same config to all subframes."""
        return self.command_parameters[0]

    @subFrameIdx.setter
    def subFrameIdx(self, value: int) -> None:
        self.command_parameters[0] = value

    @property
    def enabled(self) -> int:
        """<enabled> Enable static clutter removal technique 0 - disabled 1 - enabled

        usage: supported"""
        return self.command_parameters[1]

    @enabled.setter
    def enabled(self, value: int) -> None:
        self.command_parameters[1] = value


class aoaFovCfgCommand(MMwaveCommand):
    """Command for datapath to filter out detected points outside the specified range in azimuth or elevation plane The values in this command can be changed between sensorStop and sensorStart and even when the sensor is running. This is a mandatory command."""

    _name = "aoaFovCfg"
    _command_types = [int, float, float, float, float]

    @property
    def subFrameIdx(self) -> int:
        """<subFrameIdx> subframe Index

        usage: For legacy mode, that field should be set to -1 whereas for advanced frame mode, it should be set to either the intended subframe number or -1 to apply same config to all subframes."""
        return self.command_parameters[0]

    @subFrameIdx.setter
    def subFrameIdx(self, value: int) -> None:
        self.command_parameters[0] = value

    @property
    def minAzimuthDeg(self) -> float:
        """<minAzimuthDeg>

        usage: minimum azimuth angl (in degrees) that specifies the start of field of view"""
        return self.command_parameters[1]

    @minAzimuthDeg.setter
    def minAzimuthDeg(self, value: float) -> None:
        self.command_parameters[1] = value

    @property
    def maxAzimuthDeg(self) -> float:
        """<maxAzimuthDeg>

        usage: maximum azimuth angle (in degrees) that specifies the end of fiel of view"""
        return self.command_parameters[2]

    @maxAzimuthDeg.setter
    def maxAzimuthDeg(self, value: float) -> None:
        self.command_parameters[2] = value

    @property
    def minElevationDeg(self) -> float:
        """<minElevationDeg>

        usage: minimum elevation angle (in degrees) that specifies the start of field of view"""
        return self.command_parameters[3]

    @minElevationDeg.setter
    def minElevationDeg(self, value: float) -> None:
        self.command_parameters[3] = value

    @property
    def maxElevationDeg(self) -> float:
        """<maxElevationDeg>

        usage: maximum elevation angle (in degrees) that specifies the end of fiel of view"""
        return self.command_parameters[4]

    @maxElevationDeg.setter
    def maxElevationDeg(self, value: float) -> None:
        self.command_parameters[4] = value


class cfarFovCfgCommand(MMwaveCommand):
    """Command for datapath to filter out detected points outside the specified limits in the range direction or doppler direction The values in this command can be changed between sensorStop and sensorStart and even when the sensor is running. This is a mandatory command."""

    _name = "cfarFovCfg"
    _command_types = [int, int, float, float]

    @property
    def subFrameIdx(self) -> int:
        """<subFrameIdx> subframe Index

        usage: For legacy mode, that field should be set to -1 whereas for advanced frame mode, it should be set to either the intended subframe number or -1 to apply same config to all subframes."""
        return self.command_parameters[0]

    @subFrameIdx.setter
    def subFrameIdx(self, value: int) -> None:
        self.command_parameters[0] = value

    @property
    def procDirection(self) -> int:
        """<procDirection> Processing direction: 0 – point filtering in range direction 1 – point filtering in Doppler direction

        usage: both values supported but this command should be given twice - one for range direction and other for doppler direction"""
        return self.command_parameters[1]

    @procDirection.setter
    def procDirection(self, value: int) -> None:
        self.command_parameters[1] = value

    @property
    def procDirection(self) -> float:
        """<min (meters or m/s)> the units depends on the value for <procDirection> field above. meters for Range direction and meters/sec for Doppler direction

        usage: minimum limits for the range or doppler below which the detected points are filtered out"""
        return self.command_parameters[2]

    @procDirection.setter
    def procDirection(self, value: float) -> None:
        self.command_parameters[2] = value

    @property
    def procDirection(self) -> float:
        """<max (meters or m/s)> the units depends on the value for <procDirection> field above. meters for Range direction and meters/sec for Doppler direction

        usage: maximum limits for the range or doppler above which the detected points are filtered out"""
        return self.command_parameters[3]

    @procDirection.setter
    def procDirection(self, value: float) -> None:
        self.command_parameters[3] = value


class compRangeBiasAndRxChanPhaseCommand(MMwaveCommand):
    """Command for datapath to compensate for bias in the range estimation and receive channel gain and phase imperfections. Refer to the procedure mentioned here The values in this command can be changed between sensorStop and sensorStart and even when the sensor is running. This is a mandatory command."""

    _name = "compRangeBiasAndRxChanPhase"
    _command_types = [float, float]
    _extra_type = float

    @property
    def rangeBias(self) -> float:
        """<rangeBias> Compensation for range estimation bias in meters

        usage: supported"""
        return self.command_parameters[0]

    @rangeBias.setter
    def rangeBias(self, value: float) -> None:
        self.command_parameters[0] = value

    @property
    def rxPhaseBiasMatrix(self) -> List[float]:
        """<Re(0,0)> <Im(0,0)> <Re(0,1)> <Im (0,1)> ... <Re(0,R-1)> <Im(0,R-1)> <Re(1,0)> <Im(1,0)> ... <Re(T-1,R- 1)> <Im(T-1,R-1)> Set of Complex value representing compensation for virtual Rx channel phase bias in Q15 format. Pairs of I and Q should be provided for all Tx and Rx antennas in the device

        usage: For xwr1843, xwr6843 and xwr6443 demos: 1 pairs of values should be provided here since the device has 4 Rx an 3 Tx (total of 12 virtual antennas). Note the sign reversal required for phase compensatio coefficients in xwr6443 demo running on IWR6843AOP device. For xwr1642 demo: 8 pairs of values should be provided here since the device has 4 Rx an 2 Tx (total of 8 virtual antennas)"""
        return self.command_parameters[1:]

    @rxPhaseBiasMatrix.setter
    def rxPhaseBiasMatrix(self, value: List[float]) -> None:
        self.command_parameters[1:] = value


class measureRangeBiasAndRxChanPhaseCommand(MMwaveCommand):
    """Command for datapath to enable the measurement of the range bias and receive channel gain and phase imperfections. Refer to the procedure mentioned here The values in this command can be changed between sensorStop and sensorStart and even when the sensor is running. This is a mandatory command."""

    _name = "measureRangeBiasAndRxChanPhase"
    _command_types = [int, float, float]

    @property
    def enabled(self) -> int:
        """<enabled> 1 - enable measurement. This parameter should be enabled only using the profile_calibration.cfg profile in the mmW demo profiles directory 0 - disable measurement. This should be the value to use for all other profiles.

        usage: supported"""
        return self.command_parameters[0]

    @enabled.setter
    def enabled(self, value: int) -> None:
        self.command_parameters[0] = value

    @property
    def targetDistance(self) -> float:
        """<targetDistance> distance in meters where strong reflector is located to be used as test object for measurement. This field is only used when measurement mode is enabled.

        usage: supported"""
        return self.command_parameters[1]

    @targetDistance.setter
    def targetDistance(self, value: float) -> None:
        self.command_parameters[1] = value

    @property
    def searchWin(self) -> float:
        """<searchWin> distance in meters of the search window around <targetDistance> where the peak will be searched

        usage: supported"""
        return self.command_parameters[2]

    @searchWin.setter
    def searchWin(self, value: float) -> None:
        self.command_parameters[2] = value


class extendedMaxVelocityCommand(MMwaveCommand):
    """Velocity disambiguation config message to datapath. A simple technique for velocity disambiguation is implemented. It corrects target velocities up to (2*vmax). The output of this feature may not be reliable when two or more objects are present in the same range bin and are too close in azimuth plane. The values in this command can be changed between sensorStop and sensorStart and even when the sensor is running. This is a mandatory command."""

    _name = "extendedMaxVelocity"
    _command_types = [int, int]

    @property
    def subFrameIdx(self) -> int:
        """<subFrameIdx> subframe Index

        usage: For legacy mode, that field should be set to -1 whereas for advanced frame mode, it should be set to either the intended subframe number or -1 to apply same config to all subframes."""
        return self.command_parameters[0]

    @subFrameIdx.setter
    def subFrameIdx(self, value: int) -> None:
        self.command_parameters[0] = value

    @property
    def enabled(self) -> int:
        """<enabled> Enable velocity disambiguation technique 0 - disabled 1 - enabled

        usage: supported. Only disabled is supported for xwr64xx demo running on IWR6843AOP device."""
        return self.command_parameters[1]

    @enabled.setter
    def enabled(self, value: int) -> None:
        self.command_parameters[1] = value


class CQRxSatMonitorCommand(MMwaveCommand):
    """Rx Saturation Monitoring config message for Chirp quality to RadarSS and datapath. See mmwavelink doxgen for details on rlRxSatMonConf_t. The enable/disable for this command is controlled via the "analogMonitor" CLI command. The values in this command can be changed between sensorStop and sensorStart. This is a mandatory command"""

    _name = "CQRxSatMonitor"
    _command_types = [int, int, int, int, int]

    @property
    def profile(self) -> int:
        """<profile> Valid profile Id for this monitoring configuration. This profile ID should have a matching profileCfg.

        usage: any value as per mmwavelink doxygen but corresponding profileCfg should be defined"""
        return self.command_parameters[0]

    @profile.setter
    def profile(self, value: int) -> None:
        self.command_parameters[0] = value

    @property
    def satMonSel(self) -> int:
        """<satMonSel> RX Saturation monitoring mode

        usage: any value as per mmwavelink doxygen"""
        return self.command_parameters[1]

    @satMonSel.setter
    def satMonSel(self, value: int) -> None:
        self.command_parameters[1] = value

    @property
    def priSliceDuration(self) -> int:
        """<priSliceDuration> Duration of each slice, 1LSB=0. 16us, range: 4 -number of ADC samples

        usage: any value as per mmwavelink doxygen"""
        return self.command_parameters[2]

    @priSliceDuration.setter
    def priSliceDuration(self, value: int) -> None:
        self.command_parameters[2] = value

    @property
    def numSlices(self) -> int:
        """<numSlices> primary + secondary slices ,range 1-127. Maximum primary slice is 64.

        usage: any value as per mmwavelink doxygen"""
        return self.command_parameters[3]

    @numSlices.setter
    def numSlices(self, value: int) -> None:
        self.command_parameters[3] = value

    @property
    def rxChanMask(self) -> int:
        """<rxChanMask> RX channel mask, 1 - Mask, 0 - unmask

        usage: any value as per mmwavelink doxygen"""
        return self.command_parameters[4]

    @rxChanMask.setter
    def rxChanMask(self, value: int) -> None:
        self.command_parameters[4] = value


class CQSigImgMonitorCommand(MMwaveCommand):
    """Signal and image band energy Monitoring config message for Chirp quality to RadarSS and datapath. See mmwavelink doxgen for details on rlSigImgMonConf_t. The enable/disable for this command is controlled via the "analogMonitor" CLI command. The values in this command can be changed between sensorStop and sensorStart. This is a mandatory command"""

    _name = "CQSigImgMonitor"
    _command_types = [int, int, int]

    @property
    def profile(self) -> int:
        """<profile> Valid profile Id for this monitoring configuraiton. This profile ID should have a matching profileCfg

        usage: any value as per mmwavelink doxygen but corresponding profileCfg should be defined"""
        return self.command_parameters[0]

    @profile.setter
    def profile(self, value: int) -> None:
        self.command_parameters[0] = value

    @property
    def numSlices(self) -> int:
        """<numSlices> primary + secondary slices, range 1-127. Maximum primary slice is 64.

        usage: any value as per mmwavelink doxygen"""
        return self.command_parameters[1]

    @numSlices.setter
    def numSlices(self, value: int) -> None:
        self.command_parameters[1] = value

    @property
    def numSamplePerSlice(self) -> int:
        """<numSamplePerSlice> Possible range is 4 to "number of ADC samples" in the corresponding profileCfg. It must be an even number.

        usage: any value as per mmwavelink doxygen"""
        return self.command_parameters[2]

    @numSamplePerSlice.setter
    def numSamplePerSlice(self, value: int) -> None:
        self.command_parameters[2] = value


class analogMonitorCommand(MMwaveCommand):
    """Controls the enable/disable of the various monitoring features supported in the demos. The values in this command can be changed between sensorStop and sensorStart. This is a mandatory command."""

    _name = "analogMonitor"
    _command_types = [int, int]

    @property
    def rxSaturation(self) -> int:
        """<rxSaturation> CQRxSatMonitor enable/disable 1:enable 0: disable

        usage: supported"""
        return self.command_parameters[0]

    @rxSaturation.setter
    def rxSaturation(self, value: int) -> None:
        self.command_parameters[0] = value

    @property
    def sigImgBand(self) -> int:
        """<sigImgBand> CQSigImgMonitor enable/disable 1:enable 0: disable

        usage: supported"""
        return self.command_parameters[1]

    @sigImgBand.setter
    def sigImgBand(self, value: int) -> None:
        self.command_parameters[1] = value


class lvdsStreamCfgCommand(MMwaveCommand):
    """Enables the streaming of various data streams over LVDS lanes. When this feature is enabled, make sure chirpThreshold in adcbufCfg is set to 1. The values in this command can be changed between sensorStop and sensorStart. This is a mandatory command."""

    _name = "lvdsStreamCfg"
    _command_types = [int, int, int, int]

    @property
    def subFrameIdx(self) -> int:
        """<subFrameIdx> subframe Index

        usage: For legacy mode, that field should be set to -1 whereas for advanced frame mode, it should be set to either the intended subframe number or -1 to apply same config to all subframes."""
        return self.command_parameters[0]

    @subFrameIdx.setter
    def subFrameIdx(self, value: int) -> None:
        self.command_parameters[0] = value

    @property
    def enableHeader(self) -> int:
        """<enableHeader> 0 - Disable HSI header for all active streams 1 - Enable HSI header for all active streams

        usage: supported"""
        return self.command_parameters[1]

    @enableHeader.setter
    def enableHeader(self, value: int) -> None:
        self.command_parameters[1] = value

    @property
    def dataFmt(self) -> int:
        """<dataFmt> Controls HW streaming. Specifies the HW streaming data format. 0-HW STREAMING DISABLED 1-ADC 4-CP_ADC_CQ

        usage: supported When choosing CP_ADC_CQ, please ensure that CQRxSatMonitor and CQSigImgMonitor commands are provide with appropriate values and these monitors are enabled using analogMonitor command."""
        return self.command_parameters[2]

    @dataFmt.setter
    def dataFmt(self, value: int) -> None:
        self.command_parameters[2] = value

    @property
    def enableSW(self) -> int:
        """<enableSW> 0 - Disable user data (SW session) 1 - Enable user data (SW session) <enableHeader> should be set to 1 when this field is enabled.

        usage: supported"""
        return self.command_parameters[3]

    @enableSW.setter
    def enableSW(self, value: int) -> None:
        self.command_parameters[3] = value


class bpmCfgCommand(MMwaveCommand):
    """BPM MIMO configuration. Every frame should consist of alternating chirps with pattern TXA+TxB and TXA-TXB where TXA and TXB are two azimuth TX antennas. This is alternate configuration to TDM-MIMO scheme and provides SNR improvement by running 2Tx simultaneously. When using this scheme, user should enable both the azimuth TX in the chirpCfg. See profile_2d_bpm.cfg profile in the xwr16xx mmW demo profiles directory for example usage. This config is supported and mandatory only for demos that use Doppler DSP DPU (xwr16xx/xwr68xx). This config is not supported and is not needed for demos that use Doppler HWA DPU (xwr18xx/xwr64xx)."""

    _name = "bpmCfg"
    _command_types = [int, int, int, int]

    @property
    def subFrameIdx(self) -> int:
        """<subFrameIdx> subframe Index

        usage: For legacy mode, that field should be set to -1 whereas for advanced frame mode, it should be set to either the intended subframe number or -1 to apply same config to all subframes."""
        return self.command_parameters[0]

    @subFrameIdx.setter
    def subFrameIdx(self, value: int) -> None:
        self.command_parameters[0] = value

    @property
    def enabled(self) -> int:
        """<enabled> 0-Disabled 1-Enabled

        usage: supported"""
        return self.command_parameters[1]

    @enabled.setter
    def enabled(self, value: int) -> None:
        self.command_parameters[1] = value

    @property
    def chirp0Idx(self) -> int:
        """<chirp0Idx> BPM enabled: If BPM is enabled in previous argument, this is the chirp index for the first BPM chirp. It will have phase 0 on both TX antennas (TXA+ , TXB+). Note that the chirpCfg command for this chirp index must have both TX antennas enabled. BPM disabled: If BPM is disabled, a BPM disable command (set phase to zero on both TX antennas) will be issued for the chirps in the range [chirp0Idx..chirp1Idx]

        usage: any value as per mmwavelink doxygen but corresponding chirpCfg should be defined"""
        return self.command_parameters[2]

    @chirp0Idx.setter
    def chirp0Idx(self, value: int) -> None:
        self.command_parameters[2] = value

    @property
    def chirp1Idx(self) -> int:
        """<chirp1Idx> BPM enabled: If BPM is enabled, this is the chirp index for the second BPM chirp.It will have phase 0 on TXA and phase 180 on TXB (TXA+ , TXB-). Note that the chirpCfg command for this chirp index must have both TX antennas enabled. BPM disabled: If BPM is disabled, a BPM disable command (set phase to zero on both TX antennas) will be issued for the chirps in the range [chirp0Idx..chirp1Idx].

        usage: any value as per mmwavelink doxygen but corresponding chirpCfg should be defined"""
        return self.command_parameters[3]

    @chirp1Idx.setter
    def chirp1Idx(self, value: int) -> None:
        self.command_parameters[3] = value


class calibDataCommand(MMwaveCommand):
    """Boot time RF calibration save/restore command. Provides user to either save the boot time RF calibration performed by the RadarSS onto the FLASH or to restore the previously saved RF calibration data from the FLASH and instruct RadarSS to not re-perform the boot-time calibration. User can either save or restore or perform neither operations. User is not allowed to simultaneous save and restore in a given boot sequence. xwr18xx/60 Ghz devices: Boot time phase shift calibration data is also saved along with all other calibration data. The values in this command should not change between sensorStop and sensorStart. Reboot the board to try config with different set of values in this command This is a mandatory command."""

    _name = "calibData"
    _command_types = [int, int, int]

    @property
    def saveEnable(self) -> int:
        """<save enable> 0 - Save enabled. Application will boot-up normally and configure the RadarSS to perform all applicable boot calibrations during mmWave_open. Once the calibrations are performed, application will retrieve the calibration data from RadarSS and save it to FLASH. User need to specify valid <flash offset> value. <restore enable> option should be set to 0. 1 - Save disabled.

        usage: supported"""
        return self.command_parameters[0]

    @saveEnable.setter
    def saveEnable(self, value: int) -> None:
        self.command_parameters[0] = value

    @property
    def restoreEnable(self) -> int:
        """<restore enable> 0 - Restore enabled. Application will check the FLASH for a valid calibration data section. If present, it will restore the data from FLASH and provide it to RadarSS while configuring it to skip any real-time boot calibrations and use provided calibration data. User need to specify valid <flash offset> value which was used during saving of calibration data. <save enable> option should be set to 0. 1 - Restore disabled.

        usage: supported"""
        return self.command_parameters[1]

    @restoreEnable.setter
    def restoreEnable(self, value: int) -> None:
        self.command_parameters[1] = value

    @property
    def flashOffset(self) -> int:
        """<Flash offset> Address offset in the flash to be used while saving or restoring calibration data. Make sure the address doesn't overlap the location  in FLASH where application images are stored and has enough space for saving rlCalibrationData_t and rlPhShiftCalibrationData_t (xwr18xx/60 Ghz devices only) This field is don't care if both save and restore are disabled

        usage: supported"""
        return self.command_parameters[2]

    @flashOffset.setter
    def flashOffset(self, value: int) -> None:
        self.command_parameters[2] = value


class compressCfgCommand(MMwaveCommand):
    """Compression configuration. This command enables compression configuration (supported only in the xwr64xx_compression beta demo) of radar data cube along Rx channel and range bin dimensions (reduced L3 RAM consumption) in Range DPU, based on the configuration. It is subsequently decompressed and processed in Doppler DPU accordingly. This config is supported and mandatory only for demos that use  Doppler HWA DPU (xwr18xx/xwr64xx). This config is not supported and is not needed for demos that use Doppler DSP DPU (xwr16xx/xwr68xx)."""

    _name = "compressCfg"
    _command_types = [int, float, int]

    @property
    def subFrameIdx(self) -> int:
        """<subFrameIdx> subframe Index

        usage: For legacy mode, that field should be set to -1 whereas for advanced frame mode, it should be set to either the intended subframe number or -1 to apply same config to all subframes."""
        return self.command_parameters[0]

    @subFrameIdx.setter
    def subFrameIdx(self, value: int) -> None:
        self.command_parameters[0] = value

    @property
    def ratio(self) -> float:
        """<ratio> Compression ratio needed in radar data cube size. For eg., value of 0.25 will achieve 25% compression in actual radar cube size.

        usage: Supported values: 0.25 0.5 and 0.75. (i.e., 25%, 50% and 75%)"""
        return self.command_parameters[1]

    @ratio.setter
    def ratio(self, value: float) -> None:
        self.command_parameters[1] = value

    @property
    def numRangeBins(self) -> int:
        """<numRangeBins> Number of range bins needed per compressed block.

        usage: Supported values from 1 to 32 (maximum possible)."""
        return self.command_parameters[2]

    @numRangeBins.setter
    def numRangeBins(self, value: int) -> None:
        self.command_parameters[2] = value


class sensorStartCommand(MMwaveCommand):
    """sensor Start command to RadarSS and datapath. Starts the sensor. This function triggers the transmission of the frames as per the frame and chirp configuration. By default, this function also sends the configuration to the mmWave Front End and the processing chain. This is a mandatory command."""

    _name = "sensorStart"
    _command_types = [int]
    _num_optional = 1

    @property
    def doReconfig(self) -> Union[int, None]:
        """Optionally, user can provide an argument 'doReconfig' 0 - Skip reconfiguration and just start the sensor using already provided configuration. <any other value> - not supported

        usage: supported"""
        return self.command_parameters[0]

    @doReconfig.setter
    def doReconfig(self, value: Union[int, None]) -> None:
        self.command_parameters[0] = value


class sensorStopCommand(MMwaveCommand):
    """sensor Stop command to RadarSS and datapath. Stops the sensor. If the sensor is running, it will stop the mmWave Front End and the processing chain. After the command is acknowledged, a new config can be provided and sensor can be restarted or sensor can be restarted without a new config (i.e. using old config). See 'sensorStart' command. This is mandatory before any reconfiguration is performed post sensorStart."""

    _name = "sensorStop"
    _command_types = []


class flushCfgCommand(MMwaveCommand):
    """This command should be issued after 'sensorStop' command to flush the old configuration and provide a new one. This is mandatory before any reconfiguration is performed post sensorStart."""

    _name = "flushCfg"
    _command_types = []


class configDataPortCommand(MMwaveCommand):
    """This is an optional command to change the baud rate of the DATA_port. By default, the baud rate is 921600. This command will be accepted only when sensor is in init state or stopped state i.e. between sensorStop and sensorStart. It is recommended to use this command outside of the CFG file so that PC tools can also be configured to accept data at the desired baud rate."""

    _name = "configDataPort"
    _command_types = [int, int]

    @property
    def baudrate(self) -> int:
        """<baudrate> The new baud rate for the DATA_port. Any valid baud rate upto max of 3125000. Recommended values: 921600, 1843200, 3125000.

        usage: supported"""
        return self.command_parameters[0]

    @baudrate.setter
    def baudrate(self, value: int) -> None:
        self.command_parameters[0] = value

    @property
    def ackPing(self) -> int:
        """<ackPing> 0 - Do not send any bytes on data port 1- Send 16 bytes of value '0xFF' to ack/sync over the DATA_port (binary) after change to baud rate is applied.

        usage: supported"""
        return self.command_parameters[1]

    @ackPing.setter
    def ackPing(self, value: int) -> None:
        self.command_parameters[1] = value


class queryDemoStatusCommand(MMwaveCommand):
    """This is an optional command that can be issued anytime to get the sensor state (0-init,1-opened,2-started,3-stopped) of the device and the current baud rate of the DATA_port. The response of this command is provided on the CLI port."""

    _name = "queryDemoStatus"
    _command_types = []


class idlePowerDownCommand(MMwaveCommand):
    """This is an optional command and can be issued anytime to put the system into power down mode. This command runs each of the low power functions to power the device down into Idle Mode and leaves it there indefinitely. A hard reset to the device is required in order to provide power and resume functional mode. This command is useful when needing to confirm power figures. This command is supported only for xWR6843 devices. This command will be available only when the macro SYS_COMMON_XWR68XX_LOW_POWER_MODE_EN is defined and the libsleep utils library is used."""

    _name = "idlePowerDown"
    _command_types = [int, int, int, int, int, int, int, int, int, int]

    @property
    def subframeidx(self) -> int:
        """<subframeidx> always set to -1

        usage: supported"""
        return self.command_parameters[0]

    @subframeidx.setter
    def subframeidx(self, value: int) -> None:
        self.command_parameters[0] = value

    @property
    def enDSPpowerdown(self) -> int:
        """<enDSPpowerdown> 1 enables DSP Power Domain Off, 0 disables

        usage: supported"""
        return self.command_parameters[1]

    @enDSPpowerdown.setter
    def enDSPpowerdown(self, value: int) -> None:
        self.command_parameters[1] = value

    @property
    def enDSSclkgate(self) -> int:
        """<enDSSclkgate> 1 enables DSS Clock Gating, 0 disables

        usage: supported"""
        return self.command_parameters[2]

    @enDSSclkgate.setter
    def enDSSclkgate(self, value: int) -> None:
        self.command_parameters[2] = value

    @property
    def enMSSvclkgate(self) -> int:
        """<enMSSvclkgate> 1 enables MSS Clock Gating, 0 disables

        usage: supported"""
        return self.command_parameters[3]

    @enMSSvclkgate.setter
    def enMSSvclkgate(self, value: int) -> None:
        self.command_parameters[3] = value

    @property
    def enBSSclkgate(self) -> int:
        """<enBSSclkgate> 1 enables BSS Clock Gating, 0 disables (Note: Performed last at code level as discussed above)

        usage: supported"""
        return self.command_parameters[4]

    @enBSSclkgate.setter
    def enBSSclkgate(self, value: int) -> None:
        self.command_parameters[4] = value

    @property
    def enRFpowerdown(self) -> int:
        """<enRFpowerdown> 1 enables RF Power Down, 0 disables

        usage: supported"""
        return self.command_parameters[5]

    @enRFpowerdown.setter
    def enRFpowerdown(self, value: int) -> None:
        self.command_parameters[5] = value

    @property
    def enAPLLpowerdown(self) -> int:
        """<enAPLLpowerdown> 1 enables APLL Power Down, 0 disables

        usage: supported"""
        return self.command_parameters[6]

    @enAPLLpowerdown.setter
    def enAPLLpowerdown(self, value: int) -> None:
        self.command_parameters[6] = value

    @property
    def enAPLLGPADCpowerdown(self) -> int:
        """<enAPLLGPADCpowerdown> 1 enables APLL/GPADC Power Down, 0 disables

        usage: supported"""
        return self.command_parameters[7]

    @enAPLLGPADCpowerdown.setter
    def enAPLLGPADCpowerdown(self, value: int) -> None:
        self.command_parameters[7] = value

    @property
    def componentMicroDelay(self) -> int:
        """<componentMicroDelay> specifies a delay duration, in microseconds, between each successive power function

        usage: supported"""
        return self.command_parameters[8]

    @componentMicroDelay.setter
    def componentMicroDelay(self, value: int) -> None:
        self.command_parameters[8] = value

    @property
    def idleModeMicroDelay(self) -> int:
        """<idleModeMicroDelay> specifies a delay duration, in microseconds, after Idle Mode has been acheived but before device is powered up (if using idlePowerCycle)

        usage: not supported"""
        return self.command_parameters[9]

    @idleModeMicroDelay.setter
    def idleModeMicroDelay(self, value: int) -> None:
        self.command_parameters[9] = value


_PARSE_TABLE = {
    "dfeDataOutputMode": dfeDataOutputModeCommand.parse,
    "channelCfg": channelCfgCommand.parse,
    "adcCfg": adcCfgCommand.parse,
    "adcbufCfg": adcbufCfgCommand.parse,
    "profileCfg": profileCfgCommand.parse,
    "chirpCfg": chirpCfgCommand.parse,
    "lowPower": lowPowerCommand.parse,
    "frameCfg": frameCfgCommand.parse,
    "advFrameCfg": advFrameCfgCommand.parse,
    "subFrameCfg": subFrameCfgCommand.parse,
    "guiMonitor": guiMonitorCommand.parse,
    "cfarCfg": cfarCfgCommand.parse,
    "multiObjBeamForming": multiObjBeamFormingCommand.parse,
    "calibDcRangeSig": calibDcRangeSigCommand.parse,
    "clutterRemoval": clutterRemovalCommand.parse,
    "aoaFovCfg": aoaFovCfgCommand.parse,
    "cfarFovCfg": cfarFovCfgCommand.parse,
    "compRangeBiasAndRxChanPhase": compRangeBiasAndRxChanPhaseCommand.parse,
    "measureRangeBiasAndRxChanPhase": measureRangeBiasAndRxChanPhaseCommand.parse,
    "extendedMaxVelocity": extendedMaxVelocityCommand.parse,
    "CQRxSatMonitor": CQRxSatMonitorCommand.parse,
    "CQSigImgMonitor": CQSigImgMonitorCommand.parse,
    "analogMonitor": analogMonitorCommand.parse,
    "lvdsStreamCfg": lvdsStreamCfgCommand.parse,
    "bpmCfg": bpmCfgCommand.parse,
    "calibData": calibDataCommand.parse,
    "compressCfg": compressCfgCommand.parse,
    "sensorStart": sensorStartCommand.parse,
    "sensorStop": sensorStopCommand.parse,
    "flushCfg": flushCfgCommand.parse,
    "configDataPort": configDataPortCommand.parse,
    "queryDemoStatus": queryDemoStatusCommand.parse,
    "idlePowerDown": idlePowerDownCommand.parse
}

def parse_command(command:str) -> MMwaveCommand:
    parts = command.split(" ", 1)
    name = parts[0]
    if len(parts) == 1:
        args = ""
    else:
        args = parts[1]
    return _PARSE_TABLE[name](args)

def parse_cfg_file(contents:str) -> List[MMwaveCommand]:
    commands = []
    for line in contents.split("\n"):
        if line.startswith("%") or line.strip() == "":
            continue
        commands.append(parse_command(line))
    return commands
