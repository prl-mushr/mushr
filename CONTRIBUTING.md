# Contributing to MuSHR
MuSHR's growing community is what makes MuSHR so great! We welcome contributions from community members that created something cool, improved something that exists, or helps guide other members.
Primarily there are three forms of contributions that can be made to MuSHR:

1. **Fixing/Improving an existing repo:** If you find a bug or have an intersting extension to a current repo then please fork and submit a pull request with a description of the changes you've made (bulleted list helsp). Make sure your code passes all formatting requirements and tests associated with that repo (see below).
For internal MuSHR team please *branch* and then PR.

2. **Creating a new tutorial:** Have a cool tutorial that others may find interesting? Then fork (external) or branch (internal team) the [mushr-website](https://github.com/prl-mushr/mushr-website) repo and follow the instructions for how to create a new tutorial. A template is provide which will help you get started with our expectations/formatting. When ready submit a PR.

3. **Creating a new mushr_x repo:** If you've created something new and amazing that others would enjoy you can also submit a new mushr repo to be added to our list of external repos. That way you can still have control to maintain the repo how you wish, although we ask you follow similar practices to our repos as users are familiar with that format. For internal members, we can add your repo if it meets the linting, documentation, and testing requirements.

# Linting / Formatting
When evaluating format we look at code quality and overall directory structure. 
 
### Code Structure
As most MuSHR repos are ROS packages we follow the [ROS PyStyleGuide](http://wiki.ros.org/PyStyleGuide). In general this means the following:
```
mushr_x
├── CMakeLists.txt
├── package.xml
├── LICENSE.md # same as mushr
├── README.md
├── .github # must have linting/tests
│   ├── workflows
    │   ├── ci.yml
├── config # configs should be here, not in code or launchfile
│   ├── config.yaml
├── launch
│   ├── real.launch # do not need both, but can be helpful
│   ├── sim.launch
├── maps # only if needed
│   ├── my_map.pgm
│   ├── my_map.yaml
├── rviz # useful to have a custom config if you create a tutorial around your code
│   ├── mushr_x.rviz
├── scripts # scripts to start nodes or nodes themselves. If you have multiple nodes make a nodes directory and put them there instead
    ├── mushr_x_node # ROS wrapper around core code. No .py extension
└── src
    ├── mushr_x # no ROS code should be in core code
    │   ├── package_files.py
    │   ├── __init__.py
```
Most notably ROS specific code should be separated from functional code. This allows for easier testing, and transferability to other systems.

### Code Format
MuSHR generally follows [PEP8](https://www.python.org/dev/peps/pep-0008/) and enforces it using [flake8](https://flake8.pycqa.org/en/latest/). We also expect function, class, and file docstrings loosly following [google](https://google.github.io/styleguide/pyguide.html). Most importantly docstring should look like the following:
```python
"""
Description of my file

Author: John Doe
"""

import x

class MyClass:
"""My Class is awesome"""
    def __init__():
      """"Constructor"""
    
    def func(self, arg1, arg2):
        """
        Adds the two arguments.
        
        Attributes:
          arg1 (float): first number to add
          arg2 (float): second number to add
        
        Returns:
          sum (float): sum of two numbers
        """
```
You can check your style and even auto-format by using [isort](https://pypi.org/project/isort/), [flake8](https://pypi.org/project/flake8/), and [black](https://pypi.org/project/black/).   

To install: `pip install -r devtools/requirements.txt`.  

To run auto-formatting:  
`isort .`  
`black .`  

To lint:  
`isort . -c` If you auto-format this is not necessary  
`flake8 --config devtools/.flake8`  

## Documentation
For all MuSHR repos there must be docstrings (see linting) for each function, file, and class. Further each repo must contation a README.md that follows the EXAMPLE_README.md format.

## Tests
TODO

## Issues
If you have an issue that you think is a bug or feature request please create an issue in the appropriate repo. If you have questions, install problems, or need general support please post on our [community forum](https://github.com/prl-mushr/mushr/discussions).

## Pull Requests
We actively welcome your pull requests.

1. Fork the repo (external) or create a branch off of `main` (internal team)
2. If you've added code that should be tested, add tests.
3. If you've changed APIs, update the documentation.
4. Ensure the test suite passes.
5. Make sure your code lints.
6. Make a PR and assign @schmittlema as the reviewer. Make sure to have a description, preferably with a bulleted list of what you changed/added. If your PR is a work in progress, no problem just change the title to be WIP: my title and we will ignore it for now.

## License
By contributing to any projects under prl-mushr, you agree that your contributions will be licensed under the LICENSE file in the root directory of this source tree.
