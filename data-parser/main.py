import parser


def main():
    parser.plaintext_to_csv("calibration_data.txt")
    parser.raw_to_average("raw_data.csv")
    parser.average_to_min_sub("avg_data.csv")
    parser.min_sub_to_normalized("min_data.csv")
    parser.data_fusion("normalized_data.csv")
    parser.graph_csv("avg_data.csv", "Average Raw Sensor Data")
    parser.graph_csv("min_data.csv", "Data with Min Value Subtracted")
    parser.graph_csv("normalized_data.csv", "Normalized Data")
    parser.graph_csv("fusion_output.csv", "Sensor Fusion")

    parser.truncate_data("fusion_output.csv")


if __name__ == "__main__":
    main()
