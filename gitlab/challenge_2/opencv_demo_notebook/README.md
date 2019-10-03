# Computer Vision Experiments

## Getting up and running

Install `venv` on your computer

    sudo apt install python3-venv

Create a new virtual environment. We'll call it `robot_env`

    python3 -m venv robot_env

Activate our new environment.

    source robot_env/bin/activate

Install the requirements for our project.

    pip install -r requirements.txt

Make our new environment available to Jupyter Lab

    pip install ipykernel
    sudo robot_env/bin/python -m ipykernel install --name robot_kernel

Next time you open Jupyter Lab, you can change the kernel to `robot_env` and 
it will be able to run the notebook with no additional information.
