from .mmwave_commands import *
from ti_mmwave_msgs.msg import MMWaveConfig

class RadarConfig:
    def __init__(self, file_contents:str):
        self.commands = parse_cfg_file(file_contents)
        self.profileCommand = [c for c in self.commands if isinstance(c, profileCfgCommand)][0]
        self.channelCommand = [c for c in self.commands if isinstance(c, channelCfgCommand)][0]
        self.frameCommand = [c for c in self.commands if isinstance(c, frameCfgCommand)][0]
        self.lvdsStreamCfg = [c for c in self.commands if isinstance(c, lvdsStreamCfgCommand)][0]
        self.chirpCommands = [c for c in self.commands if isinstance(c, chirpCfgCommand)]
    
    def enable_lvds_stream(self, enable:bool=True) -> None:
        self.lvdsStreamCfg.subFrameIdx = -1
        self.lvdsStreamCfg.enableHeader = 0
        self.lvdsStreamCfg.dataFmt = int(enable)
        self.lvdsStreamCfg.enableSW = 0
    
    def to_msg(self) -> MMWaveConfig:
        c0 = 299792458
        message = MMWaveConfig()

        message.num_lvds_lanes = bin(self.channelCommand.rxChannelEn).count("1")
        message.num_adc_samples = self.profileCommand.numAdcSamples
        message.num_loops = self.frameCommand.numberOfLoops
        message.num_tx = self.frameCommand.chirpEndIndex - self.frameCommand.chirpStartIndex + 1

        adc_duration = message.num_adc_samples / self.profileCommand.digOutSampleRate
        kf = self.profileCommand.freqSlopeConst * 1e12

        message.f_s = self.profileCommand.digOutSampleRate * 1e3
        message.f_c = self.profileCommand.startFreq * 1e9 + \
            kf * (
                self.profileCommand.adcStartTime * 1e-6 + \
                adc_duration / 2
            )
        message.bw = adc_duration * kf
        message.pri = (self.profileCommand.idleTime + self.profileCommand.rampEndTime) * 1e-6

        message.range_max = message.num_adc_samples * c0 / (2 * message.bw)
        message.range_res = c0 / (2 * message.bw)
        message.doppler_max = c0 / (2 * message.f_c * message.pri) / message.num_tx
        message.doppler_res = message.doppler_max / message.num_loops

        message.raw_commands = str(self)

        return message

    def __str__(self) -> str:
        return "\n".join([str(c) for c in self.commands])
