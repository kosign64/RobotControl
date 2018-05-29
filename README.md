# Neural network and Fuzzy Logic control systems for remote control robots

The program implements neural network and fuzzy logic controllers with robot position visualization for remote control robots at Control Systems Department of Saint Petersburg Electrotechnical University "LETI"

## Tech
The programm connects to remote server via TCP and receives positions of all robots' light-reflecting points. It determines positions and orientations of each robot during initialization process. It sends controll action to the server according to neural network or fuzzy logic control system and desired robot position.

## Example of work

[![IMAGE ALT TEXT](http://img.youtube.com/vi/-wld90cr9TA/0.jpg)](http://www.youtube.com/watch?v=-wld90cr9TA "Remote control robots with neural network controller")

### Dependencies
[Qt] - C++ Framework\
[FuzzyLite] - Fuzzy Logic library

[Qt]: https://www.qt.io
[FuzzyLite]: https://www.fuzzylite.com
