USAGE INFO:

1. Clone this repository
2. `cd path/to/transforms_library`
3. `mkdir build`
4. `cmake ..`
5. `make`

To run unit tests, you will need the Boost unit_test_framework component. If you've run the above commands and everything build correctly, you can run the tests as `make test`.

Unfortunately Boost runs all the tests in a file as a single test, so if there is a problem, navigate to `path/to/transforms_library/build/test` and run the test file individually (e.g. ./transforms_test). This will show all test cases.
