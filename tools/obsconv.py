#! /usr/bin/env python3
import sys

class RINEX:
    t = ""
    sv = ""
    code = ""
    value = 0.0
    snr = 0.0
    clk_offset = 0.0
    KEYS = [
        "Epoch",
        "Flag",
        "Clock Offset [s]",
        "SV",
        "RINEX Code",
        "Value",
        "LLI",
        "SNR",
    ]
    def parse_snr(snr):
        if snr == "DbHz18_23":
            return -20
        return -10

    def parse(content):
        ret = {}
        items=content.split(',')
        for i in range(0, max(len(items), len(RINEX.KEYS))):
            content = items[i].strip()
            if RINEX.KEYS[i] == "Flag":
                if content != "0":
                    return None
            elif RINEX.KEYS[i] == "Value":
                ret[RINEX.KEYS[i]] = float(content)
            elif RINEX.KEYS[i] == "SNR":
                ret["SNR"] = RINEX.parse_snr(content)
            else:
                ret[RINEX.KEYS[i]] = content
        
        return ret

class RTK:
    KEYS = [
        "epoch",
        "clk_offset",
        "sv",
        "signal",
        "snr",
        "pr",
        "ph",
        "dop",
    ]
    def parse_signal(code):
        if "1" in code:
            return "L1"
        elif "2" in code:
            return "L2"
        else:
            return "L5"
    def __get_item__(key):
        if key == "Epoch":
            return self.epoch
        else:
            None
    def __init__(self, rnx):
        sv = rnx["SV"]
        code = rnx["RINEX Code"]
        self.t = rnx["Epoch"]
        self.sv = sv
        self.snr = rnx["SNR"]
        self.clk_offset = rnx["Clock Offset [s]"]
        self.signal = RTK.parse_signal(rnx["RINEX Code"])
        if code[0] == 'C':
            self.pseudo_range = float(rnx["Value"])
        elif code[0] == 'D':
            self.doppler = float(rnx["Value"])
        elif code[0] == 'L':
            self.phase_range = float(rnx["Value"])

    def write(self, fd):
        fd.write('{\n')
        fd.write('\t\"epoch\": {},\n'.format(self.t))
        fd.write('\t\"sv\": {},\n'.format(self.sv))
        fd.write('\t\"snr\": {},\n'.format(self.snr))
        fd.write('\t\"clk_offset\": {},\n'.format(self.clk_offset))
        fd.write('\t\"signal\": {}\n'.format(self.signal))
        fd.write('\t\"pr\": {}\n'.format(self.pseudo_range))
        # fd.write('\t\"ph\": {}\n'.format(self.phase_range))
        # fd.write('\t\"dop\": {}\n'.format(self.doppler))
        fd.write('},\n')

def main(argv):
    sv = ""
    t0 = ""
    tlast = ""
    total_epochs = 0
    svnn = []
    signals = []
    rtk = {}
    output = "examples/data"

    for i in range(len(argv)):
        if argv[i] == "-o":
            output = argv[i+1]

    for fp in argv:
        with open(fp, "r") as fd:
            filename = fp.split('/')[-1]
            output = output+"/"+filename
            with open(output, "w") as out:
                print("dataconv: \"{}\"".format(fp))
                fd.readline()
                for line in fd:
                    rnx = RINEX.parse(line)
                    t = rnx["Epoch"]
                    code = rnx["RINEX Code"]
                    value = rnx["Value"]
                    if t0 == "":
                        t0 = t
                    if tlast != t:
                        total_epochs += 1
                    if total_epochs > 1:
                        if sv != rnx["SV"]:
                            rtk.write(out)
                            out.write('\n')

                    sv = rnx["SV"]
                    rtk = RTK(rnx)
                    if not (sv in svnn):
                        svnn.append(sv)
                    if not rtk.signal in signals:
                        signals.append(rtk.signal)
                    tlast = rnx["Epoch"]

                print("t_first: {}".format(t0))
                print("t_last:  {}".format(tlast))
                print("total:   {}".format(total_epochs))
                print("sv:      {}".format(svnn))

if __name__ == "__main__":
    main(sys.argv[1:])
