import matplotlib.pyplot as plt
import numpy as np
import csv
import os

NUM_READINGS_PER_TICK = 5
NUM_READINGS = 41
NUM_SENSORS = 8
MAX_SENSOR_VAL = 2500

PATH = os.path.realpath(os.path.join(os.getcwd(), os.path.dirname(__file__), "data"))
csv_header = ['S1', 'S2', 'S3', 'S4', 'S5', 'S6', 'S7', 'S8']


# Converts text copied from Arduino Serial Monitor to a csv file
def plaintext_to_csv(filename):
    with open(os.path.join(PATH, filename), "r") as plaintext_file, \
            open(os.path.join(PATH, "raw_data.csv"), "w") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(csv_header)

        i = 0
        for line in plaintext_file:
            if ":" in line:
                if i < NUM_READINGS_PER_TICK:
                    line = line.replace("    ", ",")

                    # Last line does not have trailing spaces
                    if line[-2] != ',':
                        line += ', '

                    csv_file.write(line[16:len(line)-2] + "\n")
                    i += 1
                else:
                    continue
            else:
                i = 0


def raw_to_average(filename):
    with open(os.path.join(PATH, filename), "r") as raw_data, open(os.path.join(PATH, "avg_data.csv"), "w") as avg_data:
        reader = csv.reader(raw_data)
        writer = csv.writer(avg_data)

        next(reader)
        writer.writerow(csv_header)
        row_avg = np.zeros([NUM_SENSORS])

        i = NUM_READINGS_PER_TICK
        for row in reader:
            if i == 0:
                writer.writerow(row_avg)
                row_avg = np.zeros([NUM_SENSORS])
                i = NUM_READINGS_PER_TICK

            for j in range(NUM_SENSORS):
                row_avg[j] += (float(row[j]) / NUM_READINGS_PER_TICK)

            i -= 1

        # Write last row
        writer.writerow(row_avg)


def average_to_min_sub(filename):
    with open(os.path.join(PATH, filename), "r") as avg_data, open(os.path.join(PATH, "min_data.csv"), "w") as min_data:
        reader = csv.reader(avg_data)
        writer = csv.writer(min_data)
        writer.writerow(csv_header)

        # Find minimum value
        min_vals = np.full(NUM_SENSORS, MAX_SENSOR_VAL)

        next(reader)
        for row in reader:
            for i in range(NUM_SENSORS):
                if float(row[i]) < min_vals[i]:
                    min_vals[i] = float(row[i])

        # Write subtracted values to min_data
        min_row = np.zeros([NUM_SENSORS])

        avg_data.seek(0)
        next(reader)
        for row in reader:
            for i in range(NUM_SENSORS):
                min_row[i] = float(row[i]) - min_vals[i]

            writer.writerow(min_row)


def min_sub_to_normalized(filename):
    with open(os.path.join(PATH, filename), "r") as min_data, open(os.path.join(PATH, "normalized_data.csv"), "w") as norm_data:
        reader = csv.reader(min_data)
        writer = csv.writer(norm_data)
        writer.writerow(csv_header)

        # Find maximum values
        max_vals = np.zeros([NUM_SENSORS])

        next(reader)
        for row in reader:
            for i in range(NUM_SENSORS):
                if float(row[i]) > max_vals[i]:
                    max_vals[i] = float(row[i])

        # Write normalized values to norm_data
        norm_row = np.zeros([NUM_SENSORS])

        min_data.seek(0)
        next(reader)
        for row in reader:
            for i in range(NUM_SENSORS):
                if row[i] == "0":
                    norm_row[i] = 0
                else:
                    norm_row[i] = float(row[i]) * 1000 / max_vals[i]

            writer.writerow(norm_row)


def data_fusion(filename):
    weights_8421_4 = np.array([-8, -4,  -2, -1, 1, 2, 4, 8], dtype='f') / 4.0
    weights_1514128_8 = np.array([-15, -14, -12, -8, 8, 12, 14, 15], dtype='f') / 8.0
    weights_linear = np.array([-4, -3, -2, -1, 1, 2, 3, 4], dtype='f') / 4.0

    with open(os.path.join(PATH, filename), "r") as norm_data, open(os.path.join(PATH, "fusion_output.csv"), "w") as fusion_data:
        reader = csv.reader(norm_data)
        writer = csv.writer(fusion_data)
        row_weights = [0, 0, 0]
        writer.writerow(["(8-4-2-1)/4", "(15-14-12-8)/8", 'linear'])

        next(reader)
        for row in reader:
            for i in range(NUM_SENSORS):
                row_weights[0] += weights_8421_4[i] * float(row[i])
                row_weights[1] += weights_1514128_8[i] * float(row[i])
                row_weights[2] += weights_linear[i] * float(row[i])

            writer.writerow(row_weights)
            row_weights = [0, 0, 0]


def graph_csv(filename, graph_name):
    with open(os.path.join(PATH, filename), "r") as file:
        reader = csv.reader(file)
        header = next(reader)
        length = len(header)
        sensor_data = [[] for i in range(length)]
        x = [-40, -38, -36, -34, -32, -30, -28, -26, -24, -22, -20, -18, -16, -14, -12, -10, -8, -6, -4, -2, 0,
             2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40]

        for row in reader:
            for i in range(length):
                sensor_data[i].append(float(row[i]))

        for i in range(length):
            plt.plot(x, sensor_data[i], label=header[i], marker='o')

        plt.legend()
        plt.grid()
        plt.xlabel("Error")
        plt.title(graph_name)
        plt.show()
