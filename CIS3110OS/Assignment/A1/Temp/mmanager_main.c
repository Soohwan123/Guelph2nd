#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <getopt.h>
#include "mmanager.h"

	/* external (global) variable declarations for getopt(3) */
extern char *optarg;
extern int optind;
extern int optopt;
extern int opterr;
extern int optreset;


/** forward declarations */
static void printHelp(FILE *fp, char *progname);

int
main(int argc, char **argv)
{
	FILE *ofp = stdout;
	int verbosity = 0;
	int fitStrategy = 0;
	int filesProcessed = 0;
	long memsize = DEFAULT_MEMORY_SIZE;
	int optionchar;
	char *progname;
	int i;


	/**
	 * save the name of the program for printing help,
	 * as getopt plays with the argument list
	 */
	progname = argv[0];

	/** if no arguments, just print help */
	if (argc == 1)
	{
		printHelp(stdout, progname);
		exit (1);
	}
 

	/** use getopt(3) to parse up any option flags we may have */
	while ((optionchar = getopt(argc, argv, "vhs:o:m:")) != -1) {
		switch (optionchar) {

		case 'o':
			/* override the standard output destination */
			ofp = fopen(optarg, "w");
			if (ofp == NULL)
			{
				/* if we couldn't open the file, let the user know why */
				fprintf(stderr, "Can't open output file '%s': %s\n",
						optarg, strerror(errno));
				exit(1);
			}
			break;


		case 'm':
			/*
			 * record the amount of memory indicated.
			 * First we look for a hex number pattern
			 * (leading with 0x), if that fails, assume
			 * decimal
			 */
			if (sscanf(optarg, "0x%lx", &memsize) != 1)
			{
				if (sscanf(optarg, "%lu", &memsize) != 1)
				{
					fprintf(stderr,
							"Cannot parse memory size from '%s'\n", optarg);
					exit(-1);
				}
			}
			break;


		case 's':
			{
				if (strncasecmp(optarg, "fir", 3) == 0)
					fitStrategy = STRAT_FIRST;
				else if (strncasecmp(optarg, "bes", 3) == 0)
					fitStrategy = STRAT_BEST;
				else if (strncasecmp(optarg, "wor", 3) == 0)
					fitStrategy = STRAT_WORST;
				else
				{
					fprintf(stderr,
						"Strategy '%s' must be one of: first, best, worst\n",
						optarg);
					printHelp(stderr, progname);
					exit (-1);
				}
			}
			break;

		case 'v':
			verbosity++;
			break;

		case '?':
		case 'h':
		default:
			/* 
			 * use fall-through to get here with any of these
			 * three options
			 */
			printf("Unknown argument '%s' - ignoring\n", optarg);
		}
	}


		/** skip past the arguments processed by getopt(3) */
	argc -= optind;
	argv += optind;

		/** ensure a default strategy if none given, but warn of this */
	if (fitStrategy == 0)
	{
		printf("No strategy given -- assuming \"first fit\"\n");
		fitStrategy = STRAT_FIRST;
	}

		/* anything that remains should be a file */
	for (i = 0; i < argc; i++)
	{
		FILE *datafp = fopen(argv[i], "r");
		if (datafp == NULL)
		{
			fprintf(stderr,
					"Cannot open data file '%s' for reading\n", argv[i]);
			return -1;
		}

		if (runModel(ofp, datafp, memsize, fitStrategy, verbosity) < 0)
		{
			fprintf(stderr, "Error returned from modelling -- aborting\n");
			exit (2);
		}
		fclose(datafp);

		filesProcessed++;
	}

	/**
	 * if we didn't see any file arguments, let the user know
	 * why we didn't do any work
	 */
	if (filesProcessed == 0)
	{
		printf("No datafiles given, so no files processed!\n");
		printHelp(stdout, progname);
		exit(1);
	}

	return 0;
}



static void
printHelp(FILE *fp, char *progname)
{
	fprintf(fp, "\n");
	fprintf(fp, "%s [ <options> ] <input file> [ <input file> ...]\n",
					progname);
	fprintf(fp, "\n");
	fprintf(fp, "Options:\n");
	fprintf(fp, "\n");

	fprintf(fp, "-o<FILE>     : place output into file <FILE>\n");
	fprintf(fp, "-m<SIZE>     : model a memory of <SIZE> bytes\n");
	fprintf(fp, "-s<STRATEGY> : use <STRATEGY> to choose a free chunk.\n");
	fprintf(fp, "             : <STRATEGY> must be one of:\n");
	fprintf(fp, "             :    \"first\" - first fit of sufficient size\n");
	fprintf(fp, "             :    \"best\"  - best fit -- minimize excess\n");
	fprintf(fp, "             :    \"worst\" - worst fit -- maximize excess\n");
	fprintf(fp, "-v           : increase verbosity\n");

	fprintf(fp, "\n");
}


