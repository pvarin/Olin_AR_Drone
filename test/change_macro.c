#include <stdio.h>
#include "test.h"

int main()
{
	printf("%d %d\n", UC_VALUE, C_VALUE);
	C_VALUE = 6555;
	printf("%d %d\n", UC_VALUE, C_VALUE);
	return 0;
}
