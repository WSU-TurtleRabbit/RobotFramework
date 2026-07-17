// Host test runner. Optional argv[1] is a substring filter on test names.
#include "Motion/phx/testing.h"

int main(int argc, char** argv) { return phx::testing::run_all(argc, argv); }
