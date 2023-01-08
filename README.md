# README (control-engineering-examples)

The repository contains several examples for helping teach control engineering concepts.
These are utilized in a course named Control Engineering for System Engineers ([SYSE 580A1](https://www.online.colostate.edu/courses/SYSE/SYSE580A1.dot)) taught by [Dr. Daniel R. Herber](https://github.com/danielrherber) at Colorado State University.
All examples are for Matlab/Simulink.

## Summary

| Topic        | Folder          | # of Examples |
|--------------|-----------------|---------------|
| Introduction | [/1-introduction](1-introduction) | ?        |
| Linear Control Design | [/2-linear-control-design](2-linear-control-design) | ?        |
| Nonlinear Control Design | [/3-nonlinear-control-design](3-nonlinear-control-design) | ?         |
| Optimal Control | [/4-optimal-control](4-optimal-control) | ?         |
| Robust and Stochastic Control | [/5-robust-stochastic-control](5-robust-stochastic-control) | ?          |

## Running the Matlab Examples

1. Install an appropriate Matlab version and toolboxes to your machine
	- Validated version is ``R2022b`` but likely compatible with recent older versions
	- *[CSU-only installation instructions]* [https://www.engr.colostate.edu/ets/matlab/](https://www.engr.colostate.edu/ets/matlab/)
	- Required toolboxes:
		- ?Symbolic Math Toolbox
		- ?Optimization Toolbox
		- ?Global Optimization Toolbox
1. Open an example of interest and run the example
	- Examples might have some combination of generated command window text and figures 
1. *[Optional]* Run [test_examples.m](test_examples.m) to verify that all examples work
	- You will need to make sure all project files are in your path
	- There are some required user inputs as well (simple ``Enter`` commands will suffice)