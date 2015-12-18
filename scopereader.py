#! /usr/bin/env python3

import argparse, serial, time, datetime, scipy.signal, numpy, math, copy, textwrap, os

def processArguments():
    parser = argparse.ArgumentParser(description='Talk to a Fluke ScopeMeter.')

    parser.add_argument(
            '-p',
            '--port',
            default='/dev/ttyUSB0',
            help='serial port name (/dev/ttyS0)')

    parser.add_argument(
            '-i',
            '--identify',
            action='store_true',
            help='get identity of ScopeMeter')

    parser.add_argument(
            '-d',
            '--datetime',
            action='store_true',
            help='set the date/time of the ScopeMeter')

    parser.add_argument(
            '-s',
            '--screenshot',
            action='store_true',
            help='get a screenshot from the ScopeMeter')

    parser.add_argument(
            '-t',
            '--tex',
            action='store_true',
            help='Generate a tex report of results')

    parser.add_argument(
            '-l',
            '--html',
            action='store_true',
            help='Generate an html report of results')

    arguments = parser.parse_args()
    return arguments

def sendCommand(port, command, timeout=True):
    data = bytearray(command.encode("ascii"))
    data.append(ord('\r'))
    port.write(data)
    port.flush()
    ack = port.read(2)

    if len(ack) != 2:
        if timeout:
            print("error: command acknowledgement timed out")
            exit(1)
        else:
            return False
    
    if ack[1] != ord('\r'):
        print("error: did not receive CR after acknowledgement code")
        exit(1)

    code = int(chr(ack[0]))

    if code == 0:
        return
    elif code == 1:
        print("error: Command syntax error")
    elif code == 2:
        print("error: Command execution error")
    elif code == 3:
        print("error: Synchronization error")
    elif code == 4:
        print("error: Communication error")
    else:
        print(
                "error: Unknown error code ("
                +str(code)
                +") in command acknowledgement")
    exit(1)

    return True

def initializePort(portName):
    print("Opening and configuring serial port...", end="", flush=True)
    port = serial.Serial(portName, 1200, timeout=1)
    print("done")

    print("Reconciling serial port baud rate...", end="", flush=True)
    status = sendCommand(port, "PC 19200", False)
    port.baudrate = 19200
    if status == False:
        sendCommand(port, "PC 19200")
    print("done")

    return port

def identify(port):
    print("Getting identity of ScopeMeter...", end="", flush=True)
    sendCommand(port, "ID")
    identity = bytearray()
    while True:
        byte = port.read()
        if len(byte) != 1:
            print("error: timeout while receiving data")
            exit(1)
        if byte[0] == ord('\r'):
            break;
        identity.append(byte[0])

    identity = identity.split(b';')
    if len(identity) != 4:
        print("error: unable to decode identity string")
        exit(1)
    model = identity[0].decode()
    firmware = identity[1].decode()
    date = time.strptime(identity[2].decode(), "%Y-%m-%d")
    languages = identity[3].decode()
    print("done")
    print("     Model: "+model)
    print("   Version: "+firmware)
    print("Build Date: "+time.strftime("%B %d, %Y", date))

def dateTime(port):
    datetime = time.localtime(time.time()+1)
    print("Setting time of ScopeMeter...", end="", flush=True)
    sendCommand(port, "WT "+time.strftime("%H,%M,%S", datetime))
    print("done")
    print("Setting date of ScopeMeter...", end="", flush=True)
    sendCommand(port, "WD "+time.strftime("%Y,%m,%d", datetime))
    print("done")

def getUInt(data):
    return int.from_bytes(data, byteorder='big', signed=False)

def getInt(data):
    return int.from_bytes(data, byteorder='big', signed=True)

def getFloat(data):
    mantissa = getInt(data[0:2])
    exponent = getInt(data[2:3])

    return float(mantissa * 10.0**exponent)

def getHeader(port, intSize):
    dataSize = 3+intSize
    data = port.read(dataSize)
    if len(data) != dataSize:
        print("error: header reception timed out")
        exit(1)
    if data[0:2] != b"#0":
        print("error: header preamble incorrect")
        exit(1)

    header = int(data[2])
    size = getUInt(data[3:3+intSize])

    return (header, size)

def getData(port, size):
    size += 1
    data = port.read(size)
    if len(data) != size:
        print("error: data reception timed out")
        exit(1)
    if not checksum(data[:-1], data[-1]):
        print("error: checksum failed")
        exit(1)
    return data[:-1]

def getDecimal(port, sep=False):
    # Now get the number
    number = ""
    floating = False
    while True:
        byte = port.read()
        if len(byte) != 1:
            print("error: data length reception timed out")
            exit(1)
        byte = byte[0]
        if (ord('0') > byte or byte > ord('9')) \
                and byte != ord('.') \
                and byte != ord('+') \
                and byte != ord('-'):
            break;
        if byte == ord('.'):
            floating = True
        number += chr(byte)

    separator = byte

    if len(number) == 0:
        return None

    if floating:
        number = float(number)
    else:   
        number = int(number)

    if sep != False:
        if ord(sep) != separator:
            print("error: invalid field separator after decimal")
            exit(1)
        return number
    return (number, separator)

def checksum(data, check):
    checksum = 0
    for byte in data:
        checksum += byte
        checksum %= 256

    return (checksum == check)

def screenshot(port):
    print("Downloading screenshot from ScopeMeter...", end="", flush=True)
    sendCommand(port, "QP 0,12,B")
    
    dataLength = getDecimal(port, ',')

    image = bytearray()
    status = 0
    retries = 0
    while True:
        # Let's initiate a segment transfer
        sendCommand(port, "{:d}".format(status))

        header, size = getHeader(port, 2)
        size += 2

        # Now let's fetch the data
        data = port.read(size)
        if len(data) != size:
            print("error: segment data reception timed out")
            exit(1)

        if not checksum(data[:-2], data[-2]):
            retries += 1
            if retries >= 3:
                print("error: segment checksum failed 3 times")
                exit(1)
            status = 1
            continue

        # Check for final CR
        if data[-1] != ord('\r'):
            print("error: did not receive terminating CR in segment")
            exit(1)

        retries = 0
        image += data[:-2]
        dataLength -= len(data)-2

        if dataLength == 0 or (header&0x80) != 0:
            if dataLength == 0 and (header&0x80) != 0:
                break
            else:
                print("error: mismatch in data received and header flag")
                exit(1)
    print("done")

    filename=time.strftime("%Y-%m-%d-%H-%M-%S", time.localtime())+".png"
    print("Writing screenshot to "+filename+"...", end="", flush=True)
    imageFile = open(filename, 'wb')
    imageFile.write(image)
    imageFile.close()
    print("done")

class waveform_t:
    channel = ""
    trace_type = ""
    y_unit = ""
    x_unit = ""
    y_divisions = 0
    x_divisions = 0
    y_scale = 0.0
    x_scale = 0.0
    x_zero = 0.0
    y_at_0 = 0.0
    delta_x = 0.0
    timestamp = None
    samples = None
    averaged = False
    filename = ""
    title = ""

units = [
        None,
        "V",
        "A",
        "Ω",
        "W",
        "F",
        "K",
        "s",
        "h",
        "days",
        "Hz",
        "°",
        "°C",
        "°F",
        "%",
        "dBm 50 Ω",
        "dBm 600 Ω",
        "dBV",
        "dBA",
        "dBW",
        "VAR",
        "VA"]

class figure_t:
    title = ""
    filename = ""
    waveforms = []
    measurements = []

def si(number, precision, unit):
    def prefix(degree):
        posPrefixes = ['', 'k', 'M', 'G', 'T', 'P', 'E', 'Z', 'Y']
        negPrefixes = ['', 'm', 'μ', 'n', 'p', 'f', 'a', 'z', 'y']

        if degree < 0:
            return negPrefixes[-degree]
        else:
            return posPrefixes[degree]

    def degree(number):
        number = abs(number)

        return int(math.floor(math.log10(number)/3))
    
    if precision != 0:
        precision = math.ceil(
                precision 
                /10.0**math.floor(math.log10(precision))) \
            *10.0**math.floor(math.log10(precision))    

    significantDigits = 0
    decimalPoints = 0
    degree_var = 0
    if number != 0:
        degree_var = degree(number)
        if precision != 0:
            significantDigits = \
                    math.floor(math.log10(number)) \
                    -math.floor(math.log10(precision)) \
                    +1
            significantDigits -= int(
                    math.floor(
                        math.log10(
                            math.fabs(number/(1000.0 ** degree_var))
                        )))+1
            if significantDigits<0:
                significantDigits=0
    else:
        degree_var = degree(precision)
    
    precision = float(precision) / (1000.0 ** degree_var)
    number = float(number) / (1000.0 ** degree_var)

    if precision == 0:
        return "{:.0f} {:s}{:s}".format(
                number,
                prefix(degree_var),
                unit)
    else:
        return "({1:.{0:d}f} ± {2:.{0:d}f}) {3:s}{4:s}".format(
                significantDigits,
                number,
                precision,
                prefix(degree_var),
                unit)

def texify(string):
    string = string.replace('μ', '\\upmu ')
    string = string.replace('±', '\\pm ')
    string = string.replace('²', '^2')
    string = string.replace('%', '\\%')
    string = string.split(' ')
    string[-1] = string[-1].split('/')

    newstring = []
    if len(string[-1])==2:
        newstring.append("\\unitfrac")
        newstring.append("{{{:s}}}{{{:s}}}".format(string[-1][0], string[-1][1]))
    else:
        newstring.append("\\unit")
        newstring.append("{{{:s}}}".format(string[-1][0]))

    if len(string)==2:
        newstring.insert(1, "[{:s}]".format(string[0]))

    return "".join(newstring)

def formatSeconds(seconds):
    totalSeconds=seconds;

    days=int(math.floor(seconds/(60*60*24)))
    seconds=seconds-days*60*60*24;
    hours=int(math.floor(seconds/(60*60)))
    seconds=seconds-hours*60*60;
    minutes=int(math.floor(seconds/60))
    seconds=seconds-minutes*60;

    output=""
    if days>0:
        output="{:d} days, ".format(days)
    if hours>0:
        output=output+"{:d} hours, ".format(hours)
    if minutes>0:
        output=output+"{:d} minutes, ".format(minutes)
    
    output=output+"{:.3f} seconds".format(seconds)
    if totalSeconds>=60:
        output=output+" ({:.3f} seconds)".format(totalSeconds)
    return output

def waveform(port, source):
    print("Downloading waveform admin data from ScopeMeter...", end="", flush=True)
    sendCommand(port, "QW "+source)

    # Handle the administrative data
    header, size = getHeader(port, 2)

    if size != 47:
        print("error: admin data is a weird size ({:d})".format(size))
        exit(1)

    #if header != 0:
    #    print("error: received admin data but no samples ({:d})".format(header))
    #    exit(1)

    data = getData(port, size)

    print("done")
    print("Processing waveform admin data from ScopeMeter...", end="", flush=True)

    waveform = waveform_t()

    waveform.y_unit = units[data[1]]
    waveform.x_unit = units[data[2]]
    waveform.y_divisions = getUInt(data[3:5])
    waveform.x_divisions = getUInt(data[5:7])
    waveform.y_scale = getFloat(data[7:10])
    waveform.x_scale = getFloat(data[10:13])
    y_zero = getFloat(data[15:18])
    waveform.x_zero = getFloat(data[18:21])
    y_resolution = getFloat(data[21:24])
    waveform.delta_x = getFloat(data[24:27])
    waveform.y_at_0 = getFloat(data[27:30])
    waveform.timestamp = datetime.datetime(
            int(data[33:37].decode('ascii')),
            int(data[37:39].decode('ascii')),
            int(data[39:41].decode('ascii')),
            int(data[41:43].decode('ascii')),
            int(data[43:45].decode('ascii')),
            int(data[45:47].decode('ascii')))

    print("done")
    print("Downloading waveform sample data from ScopeMeter...", end="", flush=True)

    # Get our comma separator
    byte = port.read()
    if not (len(byte) == 1 and byte[0] == ord(',')):
        print("error: invalid separator between admin and samples")
        exit(1)

    # Handle the sample data
    header, size = getHeader(port, 4)
    #if header != 129:
    #    print("error: invalid header ({:d}) in sample data".format(header))
    #    exit(1)
    data = getData(port, size)

    terminator = port.read(1)
    if len(terminator) != 1 and terminator[0] != ord('\r'):
        print("error: got invalid terminator to trace data")

    print("done")
    print("Processing waveform sample data from ScopeMeter...", end="", flush=True)

    getNumber = getUInt
    if data[0]&0b10000000 != 0:
        getNumber = getInt
    sample_size =    data[0]&0b00000111
    samples_per_sample = 1

    if data[0]&0b01110000 == 0b01000000:
        samples_per_sample = 2
    if data[0]&0b01110000 == 0b01100000:
        samples_per_sample = 3
    if data[0]&0b01110000 == 0b01110000:
        if "trend" in waveform.trace_type:
            samples_per_sample = 3
        else:
            samples_per_sample = 2

    pointer = 1
    overload = getNumber(data[pointer:pointer+sample_size])
    pointer += sample_size
    underload = getNumber(data[pointer:pointer+sample_size])
    pointer += sample_size
    invalid = getNumber(data[pointer:pointer+sample_size])
    pointer += sample_size
    nbr_of_samples = getUInt(data[pointer:pointer+2])
    pointer += 2

    waveform.samples = numpy.empty([nbr_of_samples, samples_per_sample])
    
    for i in range(nbr_of_samples):
        for j in range(samples_per_sample):
            sample = getNumber(data[pointer:pointer+sample_size])
            if sample == overload:
                waveform.samples[i][j] = numpy.inf
            elif sample == underload:
                waveform.samples[i][j] = -numpy.inf
            elif sample == invalid:
                waveform.samples[i][j] = numpy.nan
            else:
                waveform.samples[i][j] = y_zero + sample*y_resolution
            pointer += sample_size

    if pointer != size:
        print("error: number of samples does not match block size")
        exit(1)
    print("done")

    return waveform

def waveforms(port):
    waveforms = []

    waveform_type = -1
    while waveform_type<0 or waveform_type>9:
        print(" (a) single trace")
        print(" (b) single psd")
        print(" (c) single envelope")
        print(" (d) single trend")
        print(" (e) dual trace")
        print(" (f) dual psd")
        print(" (g) dual envelope")
        print(" (h) dual trend")
        print(" (i) dual power")
        print(" (j) quit")
        waveform_type = input("What type of waveform will this be? ")
        waveform_type = ord(waveform_type[0])-ord('a')
    if waveform_type == 9:
        return;
    waveform_count = 1
    if waveform_type > 3:
        waveform_count = 2
    
    trace_type = ""
    source = "0"
    if waveform_type%4<2:
        source += '0'
        if waveform_type == 8:
            trace_type = 'power'
        elif waveform_type%4==1:
            trace_type = 'psd'
        else:
            trace_type = 'trace'
    elif waveform_type%4==2:
        source += '2'
        trace_type = 'envelope'
    elif waveform_type%4==3:
        source += '1'
        trace_type = 'trend'

    # Let's get our raw waveforms
    for waveform_number in range(waveform_count):
        source = "{:d}{:s}".format(waveform_number+1, source[1])

        data = waveform(port, source)
        data.channel = chr(ord('A')+waveform_number)
        if waveform_type%4<2:
            if data.samples.shape[1] == 2:
                matching = True
                for i in data.samples:
                    if i[0] != i[1]:
                        matching = False
                        break
                if matching:
                    samples = numpy.empty([data.samples.shape[0], 1])
                    for i in range(samples.shape[0]):
                        samples[i][0] = data.samples[i][0]
                    data.samples = samples
                    data.trace_type = "average"
                else:
                    data.trace_type = "glitch"
            else:
                data.trace_type = "trace"
        else:
            data.trace_type = trace_type
        waveforms.append(data)

    # We are doing a dual channel power calculation
    if waveform_type == 5 or waveform_type == 8:
        if waveforms[0].timestamp != waveforms[0].timestamp:
            print("error: timestamp of waveforms does not match for power")
            exit(1)
        if waveforms[0].y_unit != 'V' or waveforms[1].y_unit != 'A':
            print("error: waveform units are incorrect for power")
            exit(1)
        if waveforms[0].trace_type != waveforms[1].trace_type:
            print("error: trace types do not match for power")
            exit(1)
        if waveforms[0].samples.shape[1] != 1 \
                or waveforms[1].samples.shape[1] != 1:
            print("error: cannot do power with glitch on")
            exit(1)
        data = waveform_t()
        data.channel = "both"
        if waveforms[0].trace_type != 'trace':
            data.trace_type = waveforms[0].trace_type + ' '
        data.trace_type += "power"
        data.y_unit = "W"
        data.x_unit = waveforms[0].x_unit
        data.y_divisions = waveforms[0].y_divisions
        data.x_divisions = waveforms[0].x_divisions
        data.y_scale = waveforms[0].y_scale * waveforms[0].y_scale
        data.x_scale = waveforms[0].x_scale
        data.x_zero = waveforms[0].x_zero
        data.y_at_0 = waveforms[0].y_at_0 * waveforms[1].y_at_0
        data.delta_x = waveforms[0].delta_x
        data.timestamp = waveforms[0].timestamp
        data.samples = waveforms[0].samples
        for i in range(data.samples.shape[0]):
            data.samples[i] = waveforms[0].samples[i] * waveforms[1].samples[i]
        data.averaged = waveforms[0].averaged
        waveforms.clear()
        waveforms.append(data)

    # We are doing a power spectral density analysis
    if waveform_type%4 == 1:
        if waveform_type == 5:
            # Our values are in Watts here
            for i in data.samples[0]:
                i = math.sqrt(abs(i))

        x = numpy.empty(waveforms[0].samples.shape[0])
        for i in range(x.shape[0]):
            x[i] = waveforms[0].samples[i][0]
        segsize = int(min(int(2**math.floor(math.log2(len(x)))), 2048))
        frequency, power = scipy.signal.welch(
                x = x,
                fs = 1.0/waveforms[0].delta_x,
                window = "hamming",
                nperseg = segsize,
                noverlap = 3*segsize/4,
                return_onesided = True)
        
        data = waveform_t()
        data.window_type = "hamming"
        data.window_size = segsize
        data.trace_type = 'psd'
        if waveforms[0].y_unit == 'W':
            data.y_unit = 'dBW/Hz'
            data.channel = 'both'
        else:
            data.y_unit = 'dBV²/Hz'
            data.channel = 'A'
        data.x_unit = 'Hz'
        data.y_division = None
        data.x_division = None
        data.y_scale = None
        data.x_scale = None
        data.x_zero = frequency[0]
        data.y_at_0 = None
        data.delta_x = (frequency[-1]-frequency[0])/(len(frequency)-1)
        data.timestamp = waveforms[0].timestamp
        data.samples = numpy.empty([power.shape[0], 1])
        for i in range(power.shape[0]):
            data.samples[i][0] = 10*math.log10(power[i])
        waveforms.clear()
        waveforms.append(data)

    for i in range(len(waveforms)):
        waveforms[i].filename=waveforms[i].timestamp.strftime("%Y-%m-%d-%H-%M-%S") \
                + "_input-" + waveforms[i].channel \
                + "_" + waveforms[i].trace_type.lower().replace(' ', '-') \
                + "_" + waveforms[i].x_unit \
                + "-vs-" + waveforms[i].y_unit.replace('/', 'per')
        datFile = open(waveforms[i].filename+".dat", 'w')
        for j in range(waveforms[i].samples.shape[0]):
            datFile.write("{:.5e}".format(
                waveforms[i].x_zero+j*waveforms[i].delta_x))
            for k in range(waveforms[i].samples.shape[1]):
                datFile.write(" {:.5e}".format(waveforms[i].samples[j][k]))
            datFile.write("\n")
        datFile.close()

        waveforms[i].title = input("Enter title for waveform #{:d}: ".format(i))

    return waveforms

class measurement_t:
    source = ""
    units = ""
    value = 0.0
    name = ""
    precision = 0.0

def measurement(port):
    measurement_type = -1
    while measurement_type<0 or measurement_type>5:
        print(" (a) single")
        print(" (b) first + second")
        print(" (c) first - second")
        print(" (d) first * second")
        print(" (e) first / second")
        print(" (f) quit")
        measurement_type = input("What type of measurement will this be? ")
        measurement_type = ord(measurement_type[0])-ord('a')

    if measurement_type == 5:
        return None

    measurement_count = 1
    if measurement_type != 0:
        measurement_count = 2

    measurement = measurement_t()
    first = measurement_t()

    for measurement_number in range(measurement_count):
        print("Setup your ", end="")
        if measurement_count != 1:
            if measurement_number == 0:
                print("first ", end="")
            else:
                print("second ", end="")
        input("measurement and press enter when ready")

        print(
                "Downloading measurement metadata from ScopeMeter...",
                end="",
                flush=True)
        sendCommand(port, "QM")

        types = [
                None,
                "Mean",
                "RMS",
                "True RMS",
                "Peak to Peak",
                "Peak Maximum",
                "Peak Minimum",
                "Crest Factor",
                "Period",
                "Duty Cycle Negative",
                "Duty Cycle Positive",
                "Frequency",
                "Pulse Width Negative",
                "Pulse Width Positive",
                "Phase",
                "Diode",
                "Continuity",
                None,
                "Reactive Power",
                "Apparent Power",
                "Real Power",
                "Harmonic Reactive Power",
                "Harmonic Apparent Power",
                "Harmonic Real Power",
                "Harmonic RMS",
                "Displacement Power Factor",
                "Total Power Factor",
                "Total Harmonic Distortion",
                "Total Harmonic Distortion with respect to Fundamental",
                "K Factor (European)",
                "K Factor (US)",
                "Line Frequency",
                "Vac PWM or Vac+dc PWM",
                "Rise Time",
                "Fall Time"]

        nos = {
                11: "Reading 1",
                21: "Reading 2",
                31: "Cursor 1 Amplitude",
                41: "Cursor 2 Amplitude",
                53: "Cursor Maximum Amplitude",
                54: "Cursor Average Amplitude",
                55: "Cursor Minimum Amplitude",
                61: "Cursor Relative Amplitude",
                71: "Cursor Relative Time"}

        sources = {
                1: "Input A",
                2: "Input B",
                3: "Input C",
                4: "Input D",
                5: "External Input",
                12: "Input A vs Input B",
                21: "Input B vs Input A"}

        class reading_t:
            no = 0
            valid = False
            source = 0
            unit = 0
            thetype = 0
            pres = 0
            resolution = 0.0

        readings = []
        separator = ord(',')

        while separator == ord(','):
            reading = reading_t()
            reading.no = getDecimal(port, ',')
            if getDecimal(port, ',') == 1:
                reading.valid = True
            reading.source = getDecimal(port, ',')
            reading.unit = getDecimal(port, ',')
            reading.thetype = getDecimal(port, ',')
            reading.pres = getDecimal(port, ',')
            
            mantissa = getDecimal(port, 'E')
            exponent, separator = getDecimal(port)
            reading.resolution = mantissa * 10.0**exponent

            if reading.valid:
                readings.append(reading)
        print("done")

        letter = ord('a')
        for reading in readings:
            print(" ({}) {}:".format(chr(letter), nos[reading.no]))
            print("          Source: {}".format(sources[reading.source]))
            print("            Type: {}".format(types[reading.thetype]))
            print("            Unit: {}".format(units[reading.unit]))
            print("       Precision: {}".format(si(
                reading.resolution,
                0,
                units[reading.unit])))
            letter += 1
        desired = input("Enter desired reading and hit enter: ")
        desired = desired.lower()
        desired = desired[0]

        if ord('a') > ord(desired) or ord(desired) >= ord('a')+len(readings):
            print("error: answer is out of range")
            exit(1)

        reading = readings[ord(desired)-ord('a')]
        measurement.source = sources[reading.source]
        measurement.unit = units[reading.unit]
        measurement.precision = reading.resolution

        print("Fetching reading from ScopeMeter...", end="", flush=True)
        sendCommand(port, "QM {:d}".format(reading.no))
        measurement.value = getDecimal(port, 'E') * 10.0**getDecimal(port, '\r')
        print("done")

        print("Result: {}".format(
            si(measurement.value, measurement.precision, measurement.unit)))
        if measurement_number == 0:
            first = copy.deepcopy(measurement)

    if measurement_type != 0:
        if measurement_type < 3:
            if measurement_type == 1:
                measurement.value = first.value + measurement.value
            elif measurement_type == 2:
                measurement.value = first.value - measurement.value
            measurement.precision = first.precision + measurement.precision
            if measurement.unit != first.unit:
                print("error: units for first and second measurements differ")
                exit(1)
        else:
            value = 0.0
            if measurement_type == 3:
                value = first.value * measurement.value
                if first.unit == measurement.unit:
                    measurement.unit += '²'
                else:
                    measurement.unit = first.unit + measurement.unit
            elif measurement_type == 4:
                value = first.value / measurement.value
                if first.unit == measurement.unit:
                    measurement.unit = '%'
                else:
                    measurement.unit = first.unit + '/' + measurement.unit
            measurement.precision = value * (
                    measurement.precision/measurement.value
                    + first.precision/first.value)
            measurement.value = value
            if measurement.unit == '%':
                measurement.value *= 100
                measurement.precision *= 100



        print("Final Result: {}".format(
            si(measurement.value, measurement.precision, measurement.unit)))

    measurement.name = input("Enter title (empty to discard): ")

    return measurement

def measurements(port):
    print("\n***** Starting Measurements *****\n")

    measurements = []

    while True:
        print("\n***** Doing Measurement #{:d} *****\n".format(
            len(measurements)+1))
        x = measurement(port)
        if x == None:
            break
        if len(x.name):
            measurements.append(x)

    return measurements

def figure(port):
    figure = figure_t()
    figure.title = input("Enter figure title (blank to quit): ")
    if len(figure.title) == 0:
        return None
    figure.waveforms = waveforms(port)
    figure.measurements = measurements(port)
    figure.filename = \
            figure.waveforms[0].timestamp.strftime("%Y-%m-%d-%H-%M-%S") \
            + '_' + figure.title.replace(' ', '_').lower()


    print("\n***** Figure “{:s}” *****".format(figure.title))

    for data in figure.waveforms:
        print("\n***** Waveform “{:s}” Details *****".format(data.title))
        print("   Channel: {:s}".format(data.channel))
        print("Trace Type: {:s}".format(data.trace_type))
        print("     Units: {:s} vs {:s}".format(data.x_unit, data.y_unit))
        print(data.timestamp.strftime(" Timestamp: %H:%M:%S on %B %d, %Y"))
        print("      Size: {:d}".format(data.samples.shape[0]))
        print("  Filename: {:s}".format(data.filename))

    names = []
    nameLength = 0
    values = []
    valueLength = 0

    for measurement in figure.measurements:
        names.append(measurement.name)
        nameLength = max(nameLength, len(measurement.name))
        values.append(si(
            measurement.value,
            measurement.precision,
            measurement.unit))
        valueLength = max(valueLength, len(values[-1]))

    print("\n***** Measurement Details *****")
    print("┌{1:─<{0:d}}┬{3:─<{2:d}}┐".format(nameLength, "", valueLength, ""))
    for i in range(len(figure.measurements)):
        print("│{1: >{0:d}}│{3: <{2:d}}│".format(
            nameLength,
            names[i],
            valueLength,
            values[i]))
    print("└{1:─<{0:d}}┴{3:─<{2:d}}┘".format(nameLength, "", valueLength, ""))

    return figure

def figures(port):
    figures = []
    while True:
        fig = figure(port)
        if fig == None:
            break
        else:
            figures.append(fig)
    return figures

def tex(figs):
    try:
        os.mkdir("tex")
    except OSError:
        pass
    os.chdir("tex")

    makefile = open("Makefile", 'w')
    makefile.write(textwrap.dedent('''\
    all: report.pdf
    
    '''))
    figFiles = []

    texFile = open("report.tex", 'w')
    texFile.write(textwrap.dedent(r'''    \documentclass[letterpaper,pdftex]{article}

    \usepackage[fleqn,reqno]{amsmath}
    \usepackage{amssymb}
    \usepackage{units}
    \usepackage[pdftex]{graphicx}
    \usepackage{gnuplot-lua-tikz}
    \usepackage{upgreek}

    \begin{document}
    
    '''))

    for fig in figs:
        datFiles = []
        for waveform in fig.waveforms:
            datFiles.append("../"+waveform.filename+'.dat')
        makefile.write(textwrap.dedent('''\
        {0:s}.tex: {0:s}.gpi {1:s}
        \tgnuplot {0:s}.gpi

        '''.format(fig.filename, " ".join(datFiles))))
        figFiles.append(fig.filename  +'.tex')

        plotFile = open(fig.filename+".gpi", 'w')
        plotFile.write(textwrap.dedent('''\
                set term tikz size 4.75in,3.3in
                set output '{:s}.tex'
        '''.format(fig.filename)))
        if(fig.waveforms[0].x_unit == 'Hz'):
            x_min = 10.0**(math.log10(
                            fig.waveforms[0].x_zero
                            +fig.waveforms[0].delta_x
                            *fig.waveforms[0].samples.shape[0]
                        )-3)
            x_max = fig.waveforms[0].x_zero \
                    +fig.waveforms[0].delta_x \
                    *fig.waveforms[0].samples.shape[0]
            plotFile.write(textwrap.dedent('''\
                    set xlabel "Frequency (Hz)"
                    set ylabel "Spectral Density (${:s}$)"
                    set logscale x
                    set grid
                    set grid mxtics
                    set xtics format "$10^{{%L}}$"
                    unset key
                    set parametric
                    set xrange [{:e}:{:e}]
                    plot '../{:s}.dat' using 1:2 with lines lt 1 lc rgb 'black'
                    '''.format(
                        texify(fig.waveforms[0].y_unit).replace("\\", "\\\\"),
                        x_min,
                        x_max,
                        fig.waveforms[0].filename)))
        else:
            x_scale = texify(si(
                fig.waveforms[0].x_scale,
                0,
                fig.waveforms[0].x_unit+'/div')).replace("\\", "\\\\")
            y_scale = [texify(si(
                fig.waveforms[0].y_scale,
                0,
                fig.waveforms[0].y_unit+'/div'))]
            plotFile.write(textwrap.dedent('''\
                    set xlabel "Time (${:s}$)"
                    set xrange [{:e}:{:e}]
                    set xtics {:e}
                    set xzeroaxis lt 4 lc rgb 'black'
                    set format x ''
                    set yrange [{:e}:{:e}]
                    set ytics {:e}
                    set yzeroaxis lt 4 lc rgb 'black'
                    set format y ''
                    set grid
                    set parametric
                    unset key
                    '''.format(
                        x_scale,
                        fig.waveforms[0].x_zero,
                        fig.waveforms[0].x_zero
                            +fig.waveforms[0].x_divisions
                            *fig.waveforms[0].x_scale,
                        fig.waveforms[0].x_scale,
                        fig.waveforms[0].y_at_0,
                        fig.waveforms[0].y_at_0
                            +fig.waveforms[0].y_divisions
                            *fig.waveforms[0].y_scale,
                        fig.waveforms[0].y_scale)))
            if len(fig.waveforms) == 1:
                plotFile.write(textwrap.dedent('''\
                        set ylabel "{:s} (${:s}$)"
                        '''.format(
                            fig.waveforms[0].title,
                            y_scale[0].replace("\\", "\\\\"))))
                if fig.waveforms[0].samples.shape[1]==1:
                    plotFile.write(textwrap.dedent('''\
                            plot '../{:s}.dat' using 1:2 with lines lt 1 lc rgb 'black'
                            '''.format(fig.waveforms[0].filename)))
                else:
                    plotFile.write(textwrap.dedent('''\
                            plot '../{:s}.dat' using 1:2:3 with filledcurves fc rgb 'black'
                            '''.format(fig.waveforms[0].filename)))
            else:
                scalers = [1, fig.waveforms[0].y_scale/fig.waveforms[1].y_scale]
                y_scale.append(texify(si(
                    fig.waveforms[1].y_scale,
                    0,
                    fig.waveforms[1].y_unit+'/div')))
                wavs = []
                colors = ['red', 'blue']
                for i in range(2):
                    if fig.waveforms[i].samples.shape[1]==1:
                        wavs.append("'../{:s}.dat' using ($1):($2*{:e}) with lines lt 1 lc rgb '{:s}' title '{:s} (${:s}$)'".format(
                            fig.waveforms[i].filename,
                            scalers[i],
                            colors[i],
                            fig.waveforms[i].title,
                            y_scale[i]))
                    else:
                        wavs.append("'../{0:s}.dat' using ($1):($2*{1:e}):($3*{1:e}) with filledcurves fc rgb '{2:s}' title '{3:s} (${4:s}$)'".format(
                            fig.waveforms[i].filename,
                            scalers[i],
                            colors[i],
                            fig.waveforms[i].title,
                            y_scale[i]))
                plotFile.write(textwrap.dedent('''\
                        set key right top spacing 2
                        set ylabel "Amplitude"
                        plot {:s}
                        '''.format(", ".join(wavs))))

        plotFile.close()
        texFile.write(textwrap.dedent(r'''        \begin{figure}[p]
            \begin{center}
                \include{''')+fig.filename+textwrap.dedent(r'''        }
                \begin{tabular}{ | r | l | }
                    \hline
        '''))
        texFile.write(r"            Aquisition Time & "
                + fig.waveforms[0].timestamp.strftime("%B %d, %Y at %H:%M:%S")
                + " \\\\\n")
        if fig.waveforms[0].x_unit == 'Hz':
            texFile.write(
                    r"            Window Type & "
                    + fig.waveforms[0].window_type
                    + " \\\\\n"
                    + "            Window Size & {:d}".format(
                        fig.waveforms[0].window_size)
                    + " \\\\\n"
                    )

        for measurement in fig.measurements:
            texFile.write(
                    r"            "
                    +measurement.name
                    +r" & $"
                    +texify(si(
                        measurement.value,
                        measurement.precision,
                        measurement.unit))
                    + '$ \\\\\n')
        texFile.write(r'''            \hline
        \end{tabular}
        \caption{'''+fig.title+textwrap.dedent(r'''        }
            \end{center}
        \end{figure}
        '''))

    makefile.write(textwrap.dedent('''\
    report.pdf: {0:s}
    \ttexi2pdf report.tex

    clean:
    \trm -f {0:s} *.log report.pdf *.aux
    '''.format(" ".join(figFiles))))
    makefile.close()

    texFile.write(r"\end{document}")
    texFile.close()
    os.chdir("..")

def execute(arguments, port):
    if arguments.identify:
        identify(port)

    if arguments.datetime:
        dateTime(port)

    if arguments.screenshot:
        screenshot(port)

    if arguments.tex or arguments.html:
        figs = figures(port)
        if arguments.tex:
            tex(figs)

arguments = processArguments()
port = initializePort(arguments.port)
execute(arguments, port)
