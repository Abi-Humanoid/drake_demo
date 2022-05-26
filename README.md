# Drake Demonstration

![Unit tests](https://github.com/Abi-Humanoid/drake_demo/actions/workflows/unit-tests.yml/badge.svg)

This repository contains a demonstration of [Drake](https://drake.mit.edu/).

A [Furuta pendulum](https://en.wikipedia.org/wiki/Furuta_pendulum) is taken as a case study.
It is modelled from analytical equations and a controller is created for it.

Take a look at the files in the `drake_demo` subfolder.
`furuta_pendulum.py` contains the model of the pendulum, `controllers.py` contains
three different controllers implemented in different ways, and `furuta_pendulum_demo.py`
brings everything together by running a closed-loop simulation.

## How to use

This package can run on Ubuntu, OSX, or on Windows using the
[Windows Subsystem for Linux](https://docs.microsoft.com/en-us/windows/wsl/install).

### Creating a virtual environment

To keep your global Python installation clean, create a Python environment using

```sh
python -m venv .venv/demo
```

Then, activate it using

```sh
. .venv/demo/bin/activate
```

### Installing requirements and the package

You need to install the dependencies of this package, including Drake, by using

```
pip install -r requirements.txt
```

You then need to install the package in editable mode using

```
pip install -e .
```

### Running tests

Make sure to check out the tests in `drake_demo/tests`.
You can run them all by running

```
pytest
```

Alternatively, Visual Studio Code has
[great support for unit testing](https://code.visualstudio.com/docs/python/testing).
