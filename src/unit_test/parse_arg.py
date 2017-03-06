import argparse

parser = argparse.ArgumentParser(description='Process some integers.')
parser.add_argument('-f', '--file', type=str, help='calibration file path')

args = parser.parse_args()
