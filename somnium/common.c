
#include "common.h"
#ifdef _DEBUG

void common_create_log_file()
{
	log_file = fopen("log.txt", "w");
	assert(log_file != NULL);
}

void common_destroy_log_file()
{
	if (log_file != NULL)
		fclose(log_file);
}



#endif