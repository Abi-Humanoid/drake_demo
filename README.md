# Drake Demonstration

This repository contains a demonstration of Drake.

A [Furuta pendulum](https://en.wikipedia.org/wiki/Furuta_pendulum) is taken as a case study.
It is modelled from analytical equations and a controller is created for it.

Take a look at the files in the `drake_demo` subfolder.
`furuta_pendulum.py` contains the model of the pendulum, `controllers.py` contains
three different controllers implemented in different ways, and `furuta_pendulum_demo.py`
brings everything together by running a closed-loop simulation.

## How to use

### Installing Drake

You first need to [install Drake](https://drake.mit.edu/from_binary.html).
It is only available on Linux and Mac.

It is strongly recommended __not__ to use the inbuilt Python on your Mac/Linux machine.
[Pyenv](https://opensource.com/article/19/5/python-3-default-mac) is a solution for
managing Python version. Follow the instructions at the link, but install Python 3.9.7.

### Installing Requirements and the Package

You need to install the dependencies of this package by using

```
pip install -I -r requirements.txt
```

Note: the `-I` option is necessary if Drake was installed in a virtual environment using
`--system-site-packages`, which means by default it will use packages outside of the virtual
environment if they are already installed there. This can cause issues when calling commands
like `pytest` in the command line. `-I` forces pip to install all the packages in the virtual
environment.

You then need to install the package in editable mode using

```
pip install -e .
```

### Running Tests

Make sure to check out the tests in `drake_demo/tests`.
You can run them all by running

```
pytest
```

Alternatively, Visual Studio Code has [great support for unit testing](https://code.visualstudio.com/docs/python/testing).
