#! /usr/bin/env python3
import sys

class RINEX:
    t = ""
    sv = ""
    x = 0.0
    y = 0.0
    z = 0.0
    clk = 0.0
    KEYS = [
        "epoch",
        "sv",
        "x_ecef_km",
        "y_ecef_km",
        "z_ecef_km",
    ]
    TYPES = [
        "str",
        "str",
        "float",
        "float",
        "float",
    ]
    def parse(content):
        ret = {}
        items=content.split(',')
        for i in range(0, min(len(items), len(RINEX.KEYS))):
            content = items[i].strip()
            if RINEX.TYPES[i] == "float":
                ret[RINEX.KEYS[i]] = float(content)
            else:
                ret[RINEX.KEYS[i]] = content
        
        return ret

def main(argv):
    sv = ""
    t0 = ""
    tlast = ""
    total_epochs = 0
    first = True
    svnn = []
    output = "examples/data"

    for i in range(len(argv)):
        if argv[i] == "-o":
            output = argv[i+1]

    for fp in argv:
        with open(fp, "r") as fd:
            filename = fp.split('/')[-1]
            output = output+"/orbits.json"
            with open(output, "w") as out:
                print("navconv: \"{}\"".format(fp))
                fd.readline()
                out.write('[')
                for line in fd:
                    rnx = RINEX.parse(line)
                    sv = rnx["sv"]
                    t = rnx["epoch"]
                    
                    if t0 == "":
                        t0 = t
                    if tlast != t:
                        total_epochs += 1

                    if first:
                        first = False
                        out.write("{\n")
                    else:
                        out.write(",{\n")

                    for i in range(len(RINEX.KEYS) -1):
                        key = RINEX.KEYS[i]
                        ktype = RINEX.TYPES[i]
                        if ktype == 'str':
                            out.write("\t\"{}\": \"{}\",\n".format(key, rnx[key]))
                    
                    out.write("\t\"state\": {\n")
                    out.write("\t\t\"position\": [{},{},{}]\n".format(rnx["x_ecef_km"] * 1E3, rnx["y_ecef_km"] * 1E3, rnx["z_ecef_km"] *1E3))
                    # out.write("\t\t\"azimuth\": 0.0,\n")
                    # out.write("\t\t\"elevation\": 0.0\n")
                    out.write("\t}\n")
                    out.write("}\n")
                    if not (sv in svnn):
                        svnn.append(sv)
                    tlast = t
                out.write(']')
                print("t_first: {}".format(t0))
                print("t_last:  {}".format(tlast))
                print("total:   {}".format(total_epochs))
                print("sv:      {}".format(svnn))

if __name__ == "__main__":
    main(sys.argv[1:])
