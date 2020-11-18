# pyffb

## Usage
This is a Python package to provide an interface to the force feedback joystick module [Brunner CLS-E](https://www.brunner-innovation.swiss/product/cls-e-joystick/).

To change initial parameters or configurations, change the parameter file `config/default.yaml`.
By specifying the `config_name` argument on initialisation you can redirect to your own configuration such as `config/new_config.yaml` and create the interface with `ffb.Interface(config_name="new_config.yaml")`

_Note: If the module has been installed using `setup.py` and you want to override the relative configuration, you need to specify the absolute path to the configuration using the argument `configuration_path` on initialisation._


## Setup
Install [CLS2SIM](https://www.brunner-innovation.swiss/product/cls2sim-software/) and navigate to its installation directory under `C:\Program Files (x86)\Brunnel Elektronik`.
Install the virtual joystick in the corresponding subdirectory and if necessary consult the pdf in the main directory for more details.

1. Power the joystick and connect it to the computer according to the instructions
2. Launch `CLS2Sim` and enable [external control](https://cls2sim.brunner-innovation.swiss/externalcontrol.htm)
    - Configure axis as either _active_ or _passive_, which means either movement, respectively resistance to movement
3. Either expose this python package to the `PYTHONPATH` or install permanently via `python setup.py`
4. Executing the tests should show current joystick positions.

_Note: This repository can be used irrespective of Windows or UNIX system. The `CLS2Sim` needs to run on Windows and requires firewall settings so that the external control configured IP is accessible where this Python package is executed._


## Requirements
- `>=CLS2Sim 3.20`
- `>=Python 3.X`

## References
- [external control](https://cls2sim.brunner-innovation.swiss/externalcontrol.htm)
