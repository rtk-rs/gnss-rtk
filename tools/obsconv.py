#! /usr/bin/env python3
import sys

class OBS_RINEX:
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
        for i in range(0, max(len(items), len(OBS_RINEX.KEYS))):
            content = items[i].strip()
            if OBS_RINEX.KEYS[i] == "Flag":
                if content != "0":
                    return None
            elif OBS_RINEX.KEYS[i] == "Value":
                ret[OBS_RINEX.KEYS[i]] = float(content)
            else:
                ret[OBS_RINEX.KEYS[i]] = content
        
        return ret

class NAV_RINEX:
    t = ""
    sv = ""
    clk = 0.0
    KEYS = [
        "Epoch",
        "SV",
        "Correction",
    ]
    def parse(content):
        ret = {}
        items=content.split(',')
        for i in range(0, max(len(items), len(NAV_RINEX.KEYS))):
            content = items[i].strip()
            if NAV_RINEX.KEYS[i] == "Correction":
                ret[NAV_RINEX.KEYS[i]] = float(content)
            else:
                ret[NAV_RINEX.KEYS[i]] = content
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
        "clk",
        "observation",
    ]
    def __init__(self, rnx, corrections):
        self.t = rnx["Epoch"]
        self.sv = rnx["SV"]
        self.observation = Observation(rnx) 
        key = "{}:{}".format(self.t, self.sv)
        if key in corrections:
            self.clk = corrections[key]
        else:
            self.clk = None
            
    def write(self, fd, is_first):
        if self.clk is None:
            return is_first
        if is_first:
            fd.write('\t{\n')
        else:
            fd.write('\t,{\n')
        fd.write('\t\t\"sv\": \"{}\",\n'.format(self.sv))
        fd.write('\t\t\"epoch\": \"{}\",\n'.format(self.t))
        fd.write('\t\t\"clk\": {},\n'.format(self.clk))
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
        return False

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
    corrections = {}

    for i in range(len(argv)):
        if argv[i] == "-o":
            output = argv[i+1]
        if argv[i] == "--obs":
            obs_fp = argv[i+1]
        if argv[i] == "--nav":
            nav_fp = argv[i+1]

    with open(nav_fp, "r") as nav_fd:
        filename = nav_fp.split('/')[-1]
        print("dataconv: \"{}\"".format(filename))
        nav_fd.readline()
        for line in nav_fd:
            rnx = NAV_RINEX.parse(line)
            t = rnx["Epoch"]
            sv = rnx["SV"]
            key = "{}:{}".format(t, sv)
            corrections[key] = rnx["Correction"]

    with open(obs_fp, "r") as obs_fd:
        filename = obs_fp.split('/')[-1]
        output = output+"/"+filename
        with open("examples/data/obs.json", "w") as out:
            print("dataconv: \"{}\"".format(filename))
            out.write('[\n')
            obs_fd.readline()
            for line in obs_fd:
                obs_rnx = OBS_RINEX.parse(line)
                t = obs_rnx["Epoch"]
                code = obs_rnx["RINEX Code"]
                value = obs_rnx["Value"]
                sv = obs_rnx["SV"]

                if t0 == "":
                    t0 = t
                if tlast != t:
                    total_epochs += 1
                
                tlast = obs_rnx["Epoch"]

                if not (sv in svnn):
                    svnn.append(sv)

                if obsdata is None:
                    obsdata = ObservationData(obs_rnx, corrections)
                else:
                    if prev_sv is not None:
                        if sv != prev_sv:
                            # new SV
                            first = obsdata.write(out, first)
                            obsdata = ObservationData(obs_rnx, corrections)
                        else:
                            new_obs = Observation(obs_rnx)
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
