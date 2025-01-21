#include <stdio.h>
#include <ctype.h>
#include <string.h>
#include <ctype.h>
#include <string.h>
#include "mmanager.h"

int printAction(FILE *outputfp, mmgr_action *action)
{
    if (action->type == ACTION_ALLOCATE)
    {
        fprintf(outputfp,
            "ALLOC   chunk %4d, req %4d (0x%04x) bytes, paint '%c'\n",
            action->id, action->size, action->size, action->paint);

    } else if (action->type == ACTION_RELEASE)
    {
        fprintf(outputfp,
                "RELEASE chunk %4d\n", action->id);

    } else
    {
        fprintf(outputfp,
                "INVALID ACTION (type %d) on chunk %4d, size %4d\n",
                action->type, action->id, action->size);
    }

    /** return whether we _don't_ have an error */
    return ! ferror(outputfp);
}

static char strip(char *s)
{
    while (*s != '\0') {
        if (isprint(*s) && ( ! isspace(*s))) {
            return *s;
        }
        s++;
    }
    return -1;
}

int getAction(
        mmgr_action *action, FILE *inputfp,
        FILE *outputfp, int verbosity
    )
{
    const int BufferSize = 128;
    char buffer[BufferSize];
    char *token = NULL, *last = NULL, c;

    /** loop until either data or EOF */
    while (token == NULL) {
        /** read a line, returning 0 if we are at EOF */
        if (fgets(buffer, BufferSize, inputfp) == NULL)
            return 0;

        if (verbosity > 3)
            fprintf(outputfp, " ++++>> read : %s", buffer);

        token = strtok_s(buffer, ":", &last);
    }

    /** parse out action */
    c = strip(token);
    if (toupper(c) == 'A')
        action->type = ACTION_ALLOCATE;
    else if ((toupper(c) == 'F') || (toupper(c) == 'R'))
        action->type = ACTION_RELEASE;
    else
    {
        fprintf(stderr,
                "Error: Cannot determine action from '%s'\n", 
                token);
        return (-1);
    }

    /** default size to -1 */
    action->size = (-1);

    /** parse out ID */
    token = strtok_s(NULL, ":", &last);
    if ((token == NULL)
            || (sscanf(token, "%d", &action->id) != 1)) {
        fprintf(stderr,
                "Error: Cannot determine ID from '%s'\n", 
                last);
        return (-1);
    }

    if (action->type == ACTION_ALLOCATE) {
        /** parse out size */
        token = strtok_s(NULL, ":", &last);
        if (token == NULL) {
            fprintf(stderr, "Error: No size given for allocation\n");
            return (-1);
        }

        /** convert size -- first, try hex) */
        if (sscanf(token, "0x%x", &action->size) != 1) {
            if (sscanf(token, "%d", &action->size) != 1) {
                fprintf(stderr,
                    "Error: cannot parse size from '%s'\n", token);
                return (-2);
            }
        }

        /** fill in the paint character if given - default to '~' */
        action->paint = '~';
        token = strtok_s(NULL, ":", &last);
        if (token != NULL) {
            c = strip(token);
            if (isprint(c)) {
                action->paint = c;
            }
        }

    }

    if (verbosity > 2)
    {
        fprintf(outputfp, " + action +");
        printAction(outputfp, action);
    }

    /** return success */
    return 1;
}