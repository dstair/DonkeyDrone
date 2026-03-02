#!/usr/bin/env python3
"""
Scripts to train a keras model using tensorflow.
Basic usage should feel familiar: train.py --tubs data/ --model models/mypilot.h5

Usage:
    train.py [--tubs=tubs] (--model=<model>)
    [--type=(linear|inferred|tensorrt_linear|tflite_linear)]
    [--comment=<comment>]
    [--myconfig=<filename>]

Options:
    -h --help              Show this screen.
    --myconfig=filename    Specify myconfig file to use.
                           [default: drone_config.py]
"""

import os
from docopt import docopt
import donkeycar as dk
from donkeycar.pipeline.training import train


def main():
    args = docopt(__doc__)
    cfg = dk.load_config(config_path=os.path.join(os.path.dirname(__file__), 'config.py'), myconfig=args['--myconfig'])
    tubs = args['--tubs']
    model = args['--model']
    model_type = args['--type']
    comment = args['--comment']
    train(cfg, tubs, model, model_type, comment)


if __name__ == "__main__":
    main()
