import argparse
import os.path
import sys
import json
import matplotlib
matplotlib.use("Agg")

import scripts
from pylocogym.cmake_variables import *


if __name__ == "__main__":
    # example of python script for training and testing
    parser = argparse.ArgumentParser(description='Process input arguments.')
    parser.add_argument('-t', '--test', help="Test (prediction) mode.", required=False, action='store_true')
    parser.add_argument('-d', '--debug', help="Debug mode.", required=False, action='store_true')
    parser.add_argument('-vr', '--videoRecorder', help="Activate video recorder to record a video footage.",
                        required=False, action='store_true')
    parser.add_argument('-c', '--config', help="Path to config file", required=False)
    parser.add_argument('--rewardFile', help="Path to reward file", required=True)
    parser.add_argument('--logDir', help="Name of log directory to use for prediction", required=False)
    parser.add_argument('--step', help="Predict using the model after n time steps of training", required=False)
    parser.add_argument('-wb', '--wandb', help="Enable logging to wandb", required=False, action='store_true')

    # do not pass args to sub-functions for a better readability.
    args = parser.parse_args()

    # log path
    log_path = PYLOCO_LOG_PATH
    data_path = PYLOCO_DATA_PATH

    rewardFile_formatted = args.rewardFile.replace(".py", "").replace("./","").replace("/","_")
    if args.rewardFile is not None:
        args.rewardFile = os.path.join("src", "python", "pylocogym", "envs", "rewards", args.rewardFile)

    if not args.test:
        # =============
        # training
        # =============

        # config file
        if args.config is None:
            sys.exit('Config name needs to be specified for training: --config <config file name>')
        else:
            config_path = os.path.join(data_path, 'conf', args.config)
            print('- config file path = {}'.format(config_path))

        with open(config_path, 'r') as f:
            params = json.load(f)

        # train parameters
        hyp_params = params['train_hyp_params']
        steps = hyp_params['time_steps']
        dir_name = "{id}-{rew}-{steps:.1f}M".format(id=params['env_id'], rew=rewardFile_formatted, steps=float(steps / 1e6))

        # training
        scripts.train(
            params=params,
            log_path=log_path,
            dir_name=dir_name,
            debug=args.debug,
            video_recorder=args.videoRecorder,
            wandb_log=args.wandb,
            config_path=config_path,
            reward_path=args.rewardFile
        )
