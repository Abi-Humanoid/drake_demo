# Drake Demonstration

This repository contains a demonstration of Drake.

A [Furuta pendulum](https://en.wikipedia.org/wiki/Furuta_pendulum) is taken as a case study.
It is modelled from analytical equations and a controller is created for it.

## How to use

You first need to [install Drake](https://drake.mit.edu/from_binary.html). It is only available on Linux and Mac.

You then need to install the dependencies of this package by using

```
pip install -r requirements.txt
```

Finally, take a look at the files in the `drake_demo` subfolder.
`furuta_pendulum.py` contains the model of the pendulum, `controllers.py` contains
three different controllers implemented in different ways, and `furuta_pendulum_demo.py`
brings everything together by running a closed-loop simulation.

Also make sure to check out the tests in `drake_demo/tests`.
You can run them all by running

```
pytest
```

Alternatively, Visual Studio Code has [great support for unit testing](https://code.visualstudio.com/docs/python/testing).
