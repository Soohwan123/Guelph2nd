#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

#include "mmanager.h"

typedef struct {
    int id;          // -1 is free
    int size;       
} chunk_header;  


int runModel(FILE *outputfp, FILE *inputfp, long totalMemorySize, int fitStrategy, int verbosity)
{
    char *memoryBlock = NULL;
    int nSuccessfulActions = 0;
    int totalOverhead = 0;  // Total overhead
    
    memoryBlock = (char *) malloc(totalMemorySize);
    if (memoryBlock == NULL) {
        perror("allocation failed!");
        return -1;
    }

    // Header
    chunk_header *firstChunk = (chunk_header *)memoryBlock;
    firstChunk->id = -1;
    firstChunk->size = totalMemorySize;
    
    while (getAction(&action, inputfp, outputfp, verbosity) > 0) {
        if (action.type == ACTION_ALLOCATE) {
            chunk_header *current = (chunk_header *)memoryBlock;

			//debug check the size of current
			printf("current size: %d\n", current->size);
            bool allocated = false;
            
            // overhead
            int currentOverhead = OVERHEAD_SIZE; 
            
            // Check if the action size is larger than the total memory size
            if (action.size + currentOverhead + totalOverhead > totalMemorySize) {
                fprintf(outputfp, "alloc %d bytes : FAIL\n", action.size);
                continue;
            }

            while ((char*)current < memoryBlock + totalMemorySize - totalOverhead) {
                if (current->id == -1 && current->size >= action.size) {
                    current->id = action.id;
                    current->size = action.size;
                    totalOverhead += currentOverhead;  // overhead size
                    fprintf(outputfp, "alloc %d bytes : SUCCESS - return location %ld\n", 
                            action.size, (char*)current - memoryBlock);
                    allocated = true;
                    break;
                }
                current = (chunk_header *)((char*)current + current->size);
            }

            if (!allocated) {
                fprintf(outputfp, "alloc %d bytes : FAIL\n", action.size);
            }
        } else {
            // Release 
            chunk_header *current = (chunk_header *)memoryBlock;
            while ((char*)current < memoryBlock + totalMemorySize) {
                if (current->id == action.id) {
                    current->id = -1;  // mark as free
                    fprintf(outputfp, "free location %ld\n", (char*)current - memoryBlock);
                    break;
                }
                current = (chunk_header *)((char*)current + current->size);
            }
        }
        nSuccessfulActions++;
    }

	// Summary
    fprintf(outputfp, "SUMMARY :\n");
    
    int totalAllocated = 0;
    int chunkCount = 0;
    chunk_header *current = (chunk_header *)memoryBlock;
    
    while ((char*)current < memoryBlock + totalMemorySize) {
        if (current->id != -1) {  // allocated chunk
            totalAllocated += current->size;
        }
        chunkCount++;
        current = (chunk_header *)((char*)current + current->size);
    }
    
    fprintf(outputfp, "%d bytes allocated\n", totalAllocated);
    fprintf(outputfp, "%d bytes free\n", totalMemorySize - totalAllocated);
    fprintf(outputfp, "%d allocation chunks :\n", chunkCount);
    
    // Chunk details
    current = (chunk_header *)memoryBlock;
    int chunkNumber = 0;
    
    while ((char*)current < memoryBlock + totalMemorySize) {
        long location = (char*)current - memoryBlock;
        if (current->id == -1) {
            fprintf(outputfp, "chunk %d location %ld:%d bytes - free\n", 
                    chunkNumber, location, current->size);
        } else {
            fprintf(outputfp, "chunk %d location %ld:%d bytes - allocated\n", 
                    chunkNumber, location, current->size);
        }
        chunkNumber++;
        current = (chunk_header *)((char*)current + current->size);
    }

    free(memoryBlock);
    return nSuccessfulActions;
}
