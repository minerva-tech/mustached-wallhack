#include "pchbarrier.h"
#include "settings.h"
#include "iscanner.h"
#include "exception.h"
#include "gtest/gtest.h"

int main(int argc, char* argv[])
{
	if (argc == 2 && strcmp(argv[1], "unittest") == 0) {
		::testing::InitGoogleTest(&argc, argv);
		return RUN_ALL_TESTS();
	}

	try {
		Settings settings;

		IScanner scanner(settings);

		scanner.process();
	}
	catch (const Exception& ex) {
		std::cout << "Exception: " << ex.what();
	}

	return 0;
}

