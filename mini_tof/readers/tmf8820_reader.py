import time

import serial

TMF882X_CHANNELS = 10
TMF882X_BINS = 128
TMF882X_SKIP_FIELDS = 3  # skip first 3 items in each row
TMF882X_IDX_FIELD = 2  # second item in each row contains the idx field


class TMF8820Reader:
    def __init__(self, port):
        self.arduino = serial.Serial(port=port, baudrate=1000000, timeout=0.1)
        time.sleep(2)

    def get_measurement(self, output=None, flush_input=False, buffer_warnings=True):
        buffer = []
        frames_finished = 0

        all_processed_hists = []
        all_processed_dists = []

        if flush_input:
            self.arduino.reset_input_buffer()
            # because we might be starting in the middle of a measurement, we will throw out the
            # first few finished frames (number decided by trial and error)
            frames_finished = -3

        while frames_finished < 1:
            line = self.arduino.readline().rstrip()
            buffer.append(line)
            try:
                decoded_line = line.decode("utf-8").rstrip().split(",")
                if decoded_line == "":
                    print(
                        "Empty line read - sensor on {self.arduino.port} may not be flashed correctly"
                    )
                if (
                    len(decoded_line) > TMF882X_IDX_FIELD
                    and decoded_line[TMF882X_IDX_FIELD] == "29"
                ):
                    # if we're still flushing the first frames, don't process, just continue
                    if frames_finished < 0:
                        frames_finished += 1
                    else:  # if we're through flushing the first frames, process the input
                        processed_hists = TMF8820Reader.process_raw_hists(
                            buffer, buffer_warnings=buffer_warnings
                        )
                        processed_dists = TMF8820Reader.process_raw_dist(buffer)
                        if processed_hists is not None and processed_dists is not None:
                            all_processed_hists.append(processed_hists)
                            all_processed_dists.append(processed_dists)
                            frames_finished += 1
                            timestamp = time.time()
                    buffer = []

            # if you start reading in the middle of a character you get bad data, skip it
            except UnicodeDecodeError:
                pass
                buffer = []

        if output is not None:
            output[self.arduino.port] = {
                "hists": all_processed_hists,
                "dists": all_processed_dists,
                "timestamp": timestamp,
            }

        return all_processed_hists, all_processed_dists, timestamp

    @classmethod
    def process_raw_hists(cls, buffer, buffer_warnings=True):
        if len(buffer) != 31:
            if buffer_warnings:
                print(f"WARNING: Buffer wrong size ({len(buffer)}) - skipping and returning None")
            return None

        # initialize to -1 values so we can see which bins weren't filled in
        # (likely intentional if using partial histograms, an error if not)
        raw_sum = [[-1 for _ in range(TMF882X_BINS)] for _ in range(TMF882X_CHANNELS)]

        for line in buffer:
            data = line.decode("utf-8")
            data = data.replace("\r", "")
            data = data.replace("\n", "")
            row = data.split(",")

            if len(row) > 0 and len(row[0]) > 0 and row[0][0] == "#":
                if (
                    row[0] == "#Raw" and len(row) == TMF882X_BINS + TMF882X_SKIP_FIELDS
                ):  # ignore lines that start with #obj
                    if "" in row:
                        print("Empty entry in histogram data - skipping and returning None")
                        return None
                    idx = int(
                        row[TMF882X_IDX_FIELD]
                    )  # idx is the id of the histogram (e.g. 0-9 for 9 hists + calibration hist)
                    if idx >= 0 and idx <= 9:
                        for col in range(TMF882X_BINS):
                            raw_sum[idx][col] = int(
                                row[TMF882X_SKIP_FIELDS + col]
                            )  # meast signficant byte - just assign
                    elif idx >= 10 and idx <= 19:
                        idx = idx - 10
                        for col in range(TMF882X_BINS):
                            raw_sum[idx][col] = (
                                raw_sum[idx][col] + int(row[TMF882X_SKIP_FIELDS + col]) * 256
                            )  # middle byte - shift (mult. by 256) and add to existing
                    elif idx >= 20 and idx <= 29:
                        idx = idx - 20
                        for col in range(TMF882X_BINS):
                            raw_sum[idx][col] = (
                                raw_sum[idx][col] + int(row[TMF882X_SKIP_FIELDS + col]) * 256 * 256
                            )  # most significant byte - shift (mult. by 256^2) and add to existing
                    else:
                        print("Line read with invalid idx field - skipping line")

            elif row[0] == "#Prt":  # if it is a partial histogram
                idx = int(
                    row[TMF882X_IDX_FIELD]
                )  # idx is the id of the histogram (e.g. 0-9 for 9 hists + calibration hist)
                # when a partial histogram is printed, the value after the idx field is the
                # number of bins which were skipped
                skipped_bins = int(row[TMF882X_IDX_FIELD + 1])
                if idx >= 0 and idx <= 9:
                    for hist_bin in range(skipped_bins, len(row) - TMF882X_SKIP_FIELDS):
                        raw_sum[idx][hist_bin] += int(row[TMF882X_SKIP_FIELDS + hist_bin])
                elif idx >= 10 and idx <= 19:
                    for hist_bin in range(skipped_bins, len(row) - TMF882X_SKIP_FIELDS):
                        raw_sum[idx - 10][hist_bin] += int(row[TMF882X_SKIP_FIELDS + hist_bin]) * 256
                elif idx >= 20 and idx <= 29:
                    for hist_bin in range(skipped_bins, len(row) - TMF882X_SKIP_FIELDS):
                        raw_sum[idx - 20][hist_bin] += (
                            int(row[TMF882X_SKIP_FIELDS + hist_bin]) * 256 * 256
                        )
                else:
                    print("Line read with invalid idx field - skipping line")

            else:
                print("Histogram row incorrect length - skipping row")

        return raw_sum

    @classmethod
    def process_raw_dist(cls, buffer):
        for line in buffer:
            data = line.decode("utf-8")
            data = data.replace("\r", "")
            data = data.replace("\n", "")
            d = data.split(",")

            if len(d) == 78 and d[0] == "#Obj":
                result = {}
                result["I2C_address"] = int(d[1])
                result["measurement_num"] = int(d[2])
                result["temperature"] = int(d[3])
                result["num_valid_results"] = int(d[4])
                result["tick"] = int(d[5])
                result["depths_1"] = [
                    int(x) for x in [d[6], d[8], d[10], d[12], d[14], d[16], d[18], d[20], d[22]]
                ]
                result["confs_1"] = [
                    int(x) for x in [d[7], d[9], d[11], d[13], d[15], d[17], d[19], d[21], d[23]]
                ]
                # 18 that go in between here are unused, at least in 3x3 mode
                result["depths_2"] = [
                    int(x) for x in [d[42], d[44], d[46], d[48], d[50], d[52], d[54], d[56], d[58]]
                ]
                result["confs_2"] = [
                    int(x) for x in [d[43], d[45], d[47], d[49], d[51], d[53], d[55], d[57], d[59]]
                ]
                # last 18 are unused, at least in 3x3 mode

                return result
        return None