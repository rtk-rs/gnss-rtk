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
            else:
                ret[RINEX.KEYS[i]] = content
        
        return ret

class Observation:
    def __init__(self, rnx):
        snr = rnx["SNR"]
        code = rnx["RINEX Code"]
        value = rnx["Value"]
        self.carrier = Observation.parse_signal(code)
        if snr == "None":
            self.snr = None
        else:
            self.snr = Observation.parse_snr(snr)

        if code[0] == 'C':
            self.pseudo_range = value
        else:
            self.pseudo_range = None
        if code[0] == 'L':
            self.phase_range = value
        else:
            self.phase_range = None
        if code[0] == 'D':
            self.doppler = value
        else:
            self.doppler = None
    def parse_snr(snr):
        if snr == "DbHz18_23":
            return -20
        return -10
    def parse_signal(code):
        if "1" in code:
            return "L1"
        elif "2" in code:
            return "L2"
        else:
            return "L5"

class ObservationData:
    KEYS = [
        "epoch",
        "sv",
        "observation",
    ]
    def __init__(self, rnx):
        sv = rnx["SV"]
        self.t = rnx["Epoch"]
        self.sv = rnx["SV"]
        self.observation = Observation(rnx) 

    def write(self, fd, is_first=True):
        if is_first:
            fd.write('\t{\n')
        else:
            fd.write('\t,{\n')
        fd.write('\t\t\"sv\": \"{}\",\n'.format(self.sv))
        fd.write('\t\t\"epoch\": \"{}\",\n'.format(self.t))
        fd.write('\t\t\"observation\": {\n')
        fd.write('\t\t\t\"carrier\": \"{}\",\n'.format(self.observation.carrier))
        if self.observation.snr is not None:
            fd.write('\t\t\t\"snr\": {},\n'.format(self.observation.snr))
        if self.observation.pseudo_range is not None:
            fd.write('\t\t\t\"pseudo\": {}'.format(self.observation.pseudo_range))
        if self.observation.phase_range is not None or self.observation.doppler is not None:
            fd.write(',\n')
        if self.observation.phase_range is not None:
            fd.write('\t\t\t\"phase\": {}'.format(self.observation.phase_range))
            if self.observation.doppler is not None:
                fd.write(',\n')
        if self.observation.doppler is not None:
            fd.write('\t\t\t\"doppler\": {}\n'.format(self.observation.doppler))
        fd.write('\t\t}\n')
        fd.write('\t}\n')

def main(argv):
    sv = ""
    t0 = ""
    tlast = ""
    total_epochs = 0
    svnn = []
    signals = []
    rtk = {}
    first = True
    prev_sv = None
    output = "examples/data"
    obsdata = None
    pending_observation = None

    for i in range(len(argv)):
        if argv[i] == "-o":
            output = argv[i+1]

    for fp in argv:
        with open(fp, "r") as fd:
            filename = fp.split('/')[-1]
            output = output+"/"+filename
            with open(output, "w") as out:
                print("dataconv: \"{}\"".format(fp))
                out.write('[\n')
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

                    sv = rnx["SV"]
                    tlast = rnx["Epoch"]

                    if not (sv in svnn):
                        svnn.append(sv)

                    if obsdata is None:
                        obsdata = ObservationData(rnx)
                    else:
                        if prev_sv is not None:
                            if sv != prev_sv:
                                # new SV
                                obsdata.write(out, first)
                                first = False
                                obsdata = ObservationData(rnx)
                            else:
                                new_obs = Observation(rnx)
                                pr = new_obs.pseudo_range
                                ph = new_obs.phase_range
                                dop = new_obs.doppler
                                if pr is not None:
                                    obsdata.observation.pseudo_range = pr
                                if ph is not None:
                                    obsdata.observation.phase_range = ph
                                if dop is not None:
                                    obsdata.observation.doppler = dop

                    
                    prev_sv = sv
                    
                out.write(']\n')
                print("t_first: {}".format(t0))
                print("t_last:  {}".format(tlast))
                print("total:   {}".format(total_epochs))
                print("sv:      {}".format(svnn))

if __name__ == "__main__":
    main(sys.argv[1:])
