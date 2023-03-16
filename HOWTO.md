# How to train a policy

This is a detailed instruction for 
- setting up your learning pipeline
- training a policy on cluster  
- monitoring your policy

## Getting started

Installing conda (Linux / Cluster instructions):
- Download and run installer.
```sh 
$ wget https://repo.anaconda.com/miniconda/Miniconda3-py39_23.1.0-1-Linux-x86_64.sh
$ chmod +x ./Miniconda3-py39_23.1.0-1-Linux-x86_64.sh
```
- Accept License terms.

- Press enter to accept default installation.

- Allow installer to initialize Miniconda

- Disable activation of conda by default (Optional but recommended)
```sh 
$ source .bashrc
$ conda config --set auto_activate_base false
```

We start with [conda](https://docs.conda.io/en/latest/) to set up an environment with the minimum
dependencies: [wanda](https://docs.wandb.ai/), tensorboard
```sh 
$ conda create --name pylocoEnv python=3.9.2
$ conda activate pylocoEnv 
```

Navigate to path of repo folder e.g.
```sh 
$ cd <PATH TO repo folder>
```

Install requirements
```sh 
$ pip install -r requirements.txt 
```


Login to wandb if you want to use weights and biases over cloud (optional but strongly recommended.)
```sh
$ wandb login
wandb: Logging into wandb.ai. (Learn how to deploy a W&B server locally: https://wandb.me/wandb-server)
wandb: You can find your API key in your browser here: https://wandb.ai/authorize
wandb: Paste an API key from your profile and hit enter, or press ctrl+c to quit:
```

### Compile on server
- Build `pyloco.so` (python wrapper of pyloco C++ libraries)
  ```sh
  $ mkdir build && cd build   
  $ env2lmod
  $ module load gcc/8.2.0 python/3.9.9 cmake/3.25.0 freeglut/3.0.0 libxrandr/1.5.0  libxinerama/1.1.3 libxi/1.7.6  libxcursor/1.1.14 mesa/17.2.3 eth_proxy
  # IMPORTANT: If you use a conda environment or a virtual environment, you should add 
  # -DPython_EXECUTABLE=<PYTHON INTERPRETER PATH>  (Use absolute paths) e.g.:
  # cmake -DPython_EXECUTABLE=/cluster/home/mimora/miniconda3/envs/pylocoEnv/bin/python3 -DCMAKE_BUILD_TYPE=Release ../
  $ cmake -DPython_EXECUTABLE=/cluster/home/<YOUR_USERNAME>/miniconda3/envs/pylocoEnv/bin/python3 -DCMAKE_BUILD_TYPE=Release ../
  $ make 
  # Return to repo folder
  $ cd ..
  ```

### Run jobs on server
 
- Run
  ```sh
  # Before you start a job, make sure to run the following two commands, every time you start a new ssh connection to Euler.
  $ env2lmod
  $ module load gcc/8.2.0 python/3.9.9 cmake/3.25.0 freeglut/3.0.0 libxrandr/1.5.0  libxinerama/1.1.3 libxi/1.7.6  libxcursor/1.1.14 mesa/17.2.3 eth_proxy  
  # Submit job
  $ sbatch ./jobs/03_bob   
  ```

### Conda issues
- If you face issues with conda consider using a virtual environment.
- Load python3
  ```sh
  $ env2lmod
  $ module load gcc/8.2.0 python/3.9.9 cmake/3.25.0 freeglut/3.0.0 libxrandr/1.5.0  libxinerama/1.1.3 libxi/1.7.6  libxcursor/1.1.14 mesa/17.2.3 eth_proxy
  ```
- Create and source a virtual environment
  ```sh
  # Navigate to home folder
  $ cd
  $ mkdir venvs && cd venvs
  $ python3 -m venv pylocoEnv2
  $ source $HOME/venvs/pylocoEnv2/bin/activate  
  # Update pip
  $ python3 -m pip install --upgrade pip
  ```
- Install requirements
  ```sh
  # Navigate to repo folder
  $ cd <Path to repo>  
  $ pip3 install -r requirements.txt
  ```

 
- Compile with the virtual env
  ```sh
  # Navigate to build folder
  $ cd <Path to repo/build>
  # IMPORTANT: If you use a virtual environment, you should add 
  # -DPython_EXECUTABLE=<PYTHON INTERPRETER PATH>  (Use absolute paths) e.g.:
  # cmake -DPython_EXECUTABLE=/cluster/home/mimora/venvs/pylocoEnv2/bin/python3 -DCMAKE_BUILD_TYPE=Release ../
  $ cmake -DPython_EXECUTABLE=/cluster/home/<YOUR_USERNAME>/venvs/pylocoEnv2/bin/python3 -DCMAKE_BUILD_TYPE=Release ../
  $ make 
  # Return to repo folder
  $ cd ..
  ```
  
- Modify the job files (`jobs/01_quadratic_reward`, `jobs/02_gaussian_reward`, `jobs/03_bob`)
  - Replace `conda activate pylocoEnv` with `source $HOME/venvs/pylocoEnv2/bin/activate`
  
### Further issues
  - Check the amount of disk space available in your account using the command `lquota`. Your jobs will crash if you dont have enough disk space to write the log files to disk.
