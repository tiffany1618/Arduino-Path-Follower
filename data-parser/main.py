import parser


def main():
    parser.plaintext_to_csv("calibration_data.txt")
    parser.raw_to_average("raw_data.csv")
    parser.average_to_min_sub("avg_data.csv")
    parser.min_sub_to_normalized("min_data.csv")
    parser.data_fusion("normalized_data.csv")
    parser.graph_csv("normalized_data.csv", "Normalized")
    parser.graph_csv("fusion_output.csv", "Sensor Fusion")


if __name__ == "__main__":
    main()
